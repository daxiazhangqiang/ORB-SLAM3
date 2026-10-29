// 单目实时建图主程序（封装层入口）
// 职责：CLI → 装配（采集 / SLAM 门面 / 只读轮速 / 监控提示 / 会话 IO）→ 主循环 → 退出校验 → 保存 → 回读验证
// 不实现算法：Tracking / LocalMapping / LoopClosing / Atlas 全部用 ORB-SLAM3 原生实现
//
// 用法：
//   orb_slam3_realtime <ORBvoc.txt> <settings.yaml> [选项]
//   --session <dir|auto>   会话目录（默认 auto=zhangqiang/map_sessions/<时间戳>）
//   --camera <dev>         V4L2 设备（默认 /dev/video41）
//   --size WxH             采集分辨率（默认 640x480）
//   --fps N                采集帧率，必须与 settings 的 Camera.fps 一致（默认 20）
//   --no-flip              不做左右翻正（仅对照用）
//   --iface can0           轮速只读 CAN 接口；--no-wheel 关闭轮速里程计
//   --no-save              只跑不保存（冒烟测试用）
//   --no-verify            跳过保存后的回读验证
//   --scale-check <cmd>    退出时逐子图尺度评估脚本（默认同目录 session_submap_scale.py）
//   --verify-tool <path>   回读验证工具（默认同目录 orb_slam3_offline）

#include "monitor_hint.h"
#include "map_quality_monitor.h"
#include "realtime_types.h"
#include "session_io.h"
#include "slam_backend.h"
#include "v4l2_source.h"
#include "wheel_odom.h"

#include <poll.h>
#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <deque>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace dacheng;

namespace
{

std::atomic<bool> g_stop{false};
bool g_validate_requested = false;
std::atomic<int> g_sigint_count{0};

void OnSignal(int)
{
    // 信号处理里只置标志：不保存、不加锁、不做阻塞调用
    g_stop = true;
    g_sigint_count++;
}

struct Options
{
    std::string vocab;
    std::string settings;
    std::string session = "auto";
    std::string camera = "/dev/video41";
    int width = 640;
    int height = 480;
    double fps = 20.0;
    bool flip = true;
    std::string iface = "can0";
    bool wheel = true;
    bool save = true;
    bool verify = true;
    bool record_frames = false;
    std::string scale_check_cmd;
    std::string verify_tool;
};

void PrintUsage(const char *prog)
{
    std::cout << "用法: " << prog << " <ORBvoc.txt> <settings.yaml> [选项]\n"
              << "  --session <dir|auto>  会话目录（默认 auto）\n"
              << "  --camera <dev>        V4L2 设备（默认 /dev/video41）\n"
              << "  --size WxH            采集分辨率（默认 640x480）\n"
              << "  --fps N               采集帧率（默认 20，同步写入 Camera.fps）\n"
              << "  --no-flip             不翻正图像\n"
              << "  --iface can0          CAN 接口；--no-wheel 关闭轮速里程计\n"
              << "  --no-save             不保存 Atlas\n"
              << "  --no-verify           跳过回读验证\n"
              << "  --record-frames       把每帧图像也落盘（离线复跑用，占用磁盘）\n"
              << "  --scale-check <cmd>   退出时自动尺度诊断脚本（不应用到保存地图）\n"
              << "  --verify-tool <path>  回读验证工具\n"
              << std::endl;
}

bool ParseArgs(int argc, char **argv, Options *opt)
{
    if (argc < 3)
        return false;
    opt->vocab = argv[1];
    opt->settings = argv[2];
    for (int i = 3; i < argc; i++)
    {
        const std::string a = argv[i];
        auto need_value = [&](std::string *dst) {
            if (i + 1 >= argc)
                return false;
            *dst = argv[++i];
            return true;
        };
        if (a == "--session")
        {
            if (!need_value(&opt->session)) return false;
        }
        else if (a == "--camera")
        {
            if (!need_value(&opt->camera)) return false;
        }
        else if (a == "--iface")
        {
            if (!need_value(&opt->iface)) return false;
        }
        else if (a == "--scale-check")
        {
            if (!need_value(&opt->scale_check_cmd)) return false;
        }
        else if (a == "--verify-tool")
        {
            if (!need_value(&opt->verify_tool)) return false;
        }
        else if (a == "--fps")
        {
            std::string v;
            if (!need_value(&v)) return false;
            opt->fps = std::atof(v.c_str());
        }
        else if (a == "--size")
        {
            std::string v;
            if (!need_value(&v)) return false;
            if (std::sscanf(v.c_str(), "%dx%d", &opt->width, &opt->height) != 2)
            {
                std::cerr << "分辨率格式应为 WxH" << std::endl;
                return false;
            }
        }
        else if (a == "--no-flip")   opt->flip = false;
        else if (a == "--no-wheel")  opt->wheel = false;
        else if (a == "--no-save")   opt->save = false;
        else if (a == "--no-verify") opt->verify = false;
        else if (a == "--record-frames") opt->record_frames = true;
        else
        {
            std::cerr << "未知参数：" << a << std::endl;
            return false;
        }
    }
    return true;
}

std::string TimestampTag()
{
    char buf[32];
    std::time_t t = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
    return buf;
}

// 把用户 settings 复制成会话内副本，并写入 Atlas 保存路径（相对会话目录）
bool WriteSessionSettings(const std::string &src, const std::string &dst, double fps, std::string *err)
{
    std::ifstream in(src);
    if (!in.is_open())
    {
        if (err) *err = "读不到 settings：" + src;
        return false;
    }
    std::ostringstream out;
    std::string line;
    while (std::getline(in, line))
    {
        if (line.rfind("System.", 0) == 0 || line.rfind("Camera.fps:", 0) == 0)
            continue;   // 丢掉外部配置里的 Atlas 相关项，避免路径互相覆盖
        out << line << '\n';
    }

    out << "Camera.fps: " << fps << '\n';
    std::ofstream f(dst, std::ios::out | std::ios::trunc);
    if (!f.is_open())
    {
        if (err) *err = "写不了会话 settings：" + dst;
        return false;
    }
    f << out.str();
    return true;
}

// 主循环只负责在线采集，用户结束后进入最终导出流程
bool RunMappingLoop(ISensorSource *source, SlamBackend *slam, AtlasMonitor *monitor,
                    SessionWriter *writer, double ring_keep_s, bool record_frames,
                    std::vector<std::vector<std::string>> *pending_split_snaps,
                    std::vector<double> *pending_split_times, const LogTee *tee)
{
    MapQualityMonitor quality("mapping_quality");
    constexpr double kQualityIntervalS = 2.0;
    constexpr double kInventoryIntervalS = 20.0;
    constexpr double kQualitySaveIntervalS = 10.0;
    constexpr double kSubmapSummaryIntervalS = 30.0;
    double last_quality = -10, last_inventory = -10, last_quality_save = -10;
    double last_weak_snapshot = -10;
    bool stdin_open = true;
    double last_submap_summary = -10;
    int last_quality_map_id = -1;
    int last_quality_state = -1;
    double last_ring_t = 0.0;
    double fps_win_start = NowSeconds();
    int fps_win_frames = 0;
    // 保留短缓冲用于跟踪丢失现场快照
    std::deque<std::pair<double, cv::Mat>> full_ring;
    // 跟踪质量采样环（5 Hz，保留 5 s）：用于丢失点归因
    std::deque<std::tuple<double, int, int>> track_ring;   // (t, extracted, matched)
    double last_track_poll = 0.0;
    FrameTrackStats last_stats;
    double last_cost_ms = 0.0;
    // pending_split_snaps / pending_split_times 由调用方持有（栈式）：
    // 焊接成功一次就回收最近一次分裂的快照，剩下的 lost_split_*.jpg = 还没 merge 到一起的场景

    // 丢失/分裂时按“断前 2 s 的特征与匹配数”给出可执行的判断
    auto record_loss = [&](const std::string &event, double t) {
        double sum_e = 0.0, sum_m = 0.0;
        int n = 0;
        int e_last = -1, m_last = -1;
        for (const auto &item : track_ring)
        {
            const double ts = std::get<0>(item);
            if (ts < t - 2.0 || ts > t + 0.001)
                continue;
            sum_e += std::get<1>(item);
            sum_m += std::get<2>(item);
            n++;
            e_last = std::get<1>(item);
            m_last = std::get<2>(item);
        }
        std::string verdict;
        double delta_pct = 0.0;
        const double e_mean = n > 0 ? sum_e / n : -1.0;
        const double m_mean = n > 0 ? sum_m / n : -1.0;
        if (n < 3)
        {
            verdict = "样本不足（跟踪统计还没采到）";
        }
        else if (e_mean < 400.0)
        {
            verdict = "画面本身特征太少：弱纹理/过曝/逆光/近距离遮挡";
        }
        else if (m_mean < 70.0)
        {
            // ORB-SLAM3 纯单目自己的丢失线是 TrackLocalMap 里 mnMatchesInliers<50
            verdict = "特征够但匹配数已跌到算法丢失线附近（纯单目 mnMatchesInliers<50）："
                      "多见于纯旋转/视差不足、动态物、朝向未建图区域";
        }
        else
        {
            delta_pct = (m_last - m_mean) / std::max(1.0, m_mean) * 100.0;
            if (delta_pct < -50.0)
                verdict = "匹配数在 2 s 内快速塌陷：快速运动/模糊/动态物";
            else
                verdict = "特征与匹配数都不低但跟踪断：多半是场景反射/玻璃或地图点被剔除，需看现场图";
        }
        writer->AppendLossStat(t, event, monitor->MapCountForLog(), e_last, m_last, e_mean, m_mean,
                               last_cost_ms, delta_pct, verdict);
        std::cout << "[loss] " << event << " t=" << t << "s  断前2s 特征均值 " << e_mean
                  << "、匹配均值 " << m_mean << "（末帧 特征 " << e_last << " 匹配 " << m_last
                  << "）→ " << verdict << std::endl;
    };

    // 从全帧环形缓冲里取“现在 / 前 1 s / 前 2 s”三张存成现场快照
    auto save_scene_series = [&](const std::string &tag, double now) {
        std::vector<std::string> saved;
        for (double offset : {0.0, 1.0, 2.0})
        {
            const double want = now - offset;
            const cv::Mat *best = nullptr;
            double best_dt = 1e9;
            for (const auto &item : full_ring)
            {
                const double dt = std::abs(item.first - want);
                if (dt < best_dt)
                {
                    best_dt = dt;
                    best = &item.second;
                }
            }
            if (best && best_dt < 0.5)
            {
                const std::string path = writer->SaveSnapshot(*best, want, tag);
                if (!path.empty())
                    saved.push_back(path);
            }
        }
        return saved;
    };

    while (!g_stop.load())
    {
        // stdin: q 退出（非阻塞，管道关闭后自动停查）
        if (stdin_open)
        {
            struct pollfd pfd;
            pfd.fd = STDIN_FILENO;
            pfd.events = POLLIN;
            if (::poll(&pfd, 1, 0) > 0)
            {
                char c = 0;
                const ssize_t n = ::read(STDIN_FILENO, &c, 1);
                if (n <= 0)
                    stdin_open = false;
                else if (c == 'v' || c == 'V')
                {
                    std::cout << "\n收到 v：请保持底盘停止，保存候选图后由封装启动独立回访验证" << std::endl;
                    g_validate_requested = true; g_stop = true;
                }
                else if (c == 'q' || c == 'Q')
                {
                    std::cout << "\n收到 q：准备收口" << std::endl;
                    g_stop = true;
                }
            }
        }

        if(g_stop.load()) break;
        FramePacket packet;
        if (!source->WaitLatest(&packet, 200))
            continue;

        const double t0 = NowSeconds();
        slam->Track(packet);
        const double cost_ms = (NowSeconds() - t0) * 1000.0;
        const int state = slam->TrackingState();
        const int maps = slam->MapCount();
        quality.ObserveState(packet.timestamp, state);
        if (packet.timestamp-last_quality >= kQualityIntervalS) {
            const bool inventory = packet.timestamp-last_inventory >= kInventoryIntervalS;
            const auto sample = slam->Quality(inventory);
            const bool tracking_stable = quality.Update(sample, packet.timestamp, inventory);
            last_quality_map_id = sample.active_map_id;
            last_quality_state = sample.state;
            // 只有主图已经初始化且当前帧在主图上跟踪时才保存弱质量现场
            const bool can_capture_weak = sample.state == 2 && sample.active_map_id == sample.map_id &&
                sample.map_id >= 0 && sample.points > 0;
            if (!tracking_stable && can_capture_weak && packet.timestamp-last_weak_snapshot >= 10) {
                const auto path = writer->SaveSnapshot(packet.left, packet.timestamp, "quality_weak");
                std::cout << "[补扫快照] " << path << std::endl;
                last_weak_snapshot=packet.timestamp;
            }
            if (packet.timestamp-last_quality_save >= kQualitySaveIntervalS) {
                quality.Save();
                last_quality_save=packet.timestamp;
            }
            last_quality=packet.timestamp;
            if (inventory) last_inventory=packet.timestamp;
        }
        if(packet.timestamp - last_submap_summary >= kSubmapSummaryIntervalS)
        {
            // 复用最近一次质量采样的状态，避免为摘要重复计算完整质量快照
            slam->PrintSubmapSummary(last_quality_map_id, last_quality_state);
            last_submap_summary = packet.timestamp;
        }
        monitor->OnFrame(state, maps, source->Stats(), cost_ms, packet.timestamp,
                         last_stats.extracted, last_stats.matched);

        // 全帧只保留5秒，180秒低频抽帧由独立缓冲维护
        full_ring.emplace_back(packet.timestamp, packet.left);
        while (!full_ring.empty() && packet.timestamp - full_ring.front().first > 5.0)
            full_ring.pop_front();

        if (record_frames)
            writer->SaveRawFrame(packet.left, packet.timestamp);

        // 5 Hz 采一次跟踪质量（只在有统计时采）
        if (packet.timestamp - last_track_poll >= 0.2)
        {
            last_track_poll = packet.timestamp;
            last_stats = slam->TrackStats();
            if (last_stats.extracted >= 0)
            {
                track_ring.emplace_back(packet.timestamp, last_stats.extracted, last_stats.matched);
                while (!track_ring.empty() && packet.timestamp - std::get<0>(track_ring.front()) > 5.0)
                    track_ring.pop_front();
            }
        }
        last_cost_ms = cost_ms;

        // 现场快照：子图分裂 / 跟踪丢失时，把此刻与前 1~2 s 的画面落盘
        if (monitor->TakeNewMapPending())
        {
            // 分裂快照记入“待焊接”清单；焊接成功后会被回收
            pending_split_snaps->push_back(save_scene_series("lost_split", packet.timestamp));
            pending_split_times->push_back(packet.timestamp);
            record_loss("split", packet.timestamp);
        }
        if (monitor->TakeLostPending())
        {
            // 普通丢失只保留归因统计，不再保存大量 lost_*.jpg
            record_loss("lost", packet.timestamp);
        }
        // 焊接成功：回收最近一次分裂的快照，只留仍未合并的
        if (monitor->TakeMergePending())
        {
            if (!pending_split_snaps->empty())
            {
                const std::vector<std::string> snaps = pending_split_snaps->back();
                const double split_t = pending_split_times->back();
                pending_split_snaps->pop_back();
                pending_split_times->pop_back();
                int removed = 0;
                for (const std::string &path : snaps)
                    if (writer->RemoveFile(path))
                        removed++;
                writer->AppendEvent(packet.timestamp, monitor->MapCountForLog(), "split_healed");
                std::cout << "[merge] 子图已焊接：回收分裂点 t=" << split_t << "s 的快照 "
                          << removed << " 张（";
                for (size_t i = 0; i < snaps.size(); i++)
                    std::cout << (i ? ", " : "") << snaps[i];
                std::cout << "）；剩余未合并的分裂 " << pending_split_snaps->size() << " 个"
                          << std::endl;
            }
            else
            {
                std::cout << "[merge] 子图已焊接（本次会话没有待回收的分裂快照）" << std::endl;
            }
        }

        // 1 FPS 抽帧（退出时做图像侧校验）
        if (packet.timestamp - last_ring_t >= 1.0)
        {
            last_ring_t = packet.timestamp;
            writer->PushRingFrame(packet.left, packet.timestamp, ring_keep_s);
        }
    }
    quality.Save();
    return !g_stop.load();
}

// 路径作为单个shell参数传递，兼容空格和特殊字符
std::string ShellQuote(const std::string &value)
{
    std::string out = "'";
    for(char c : value) out += c == '\'' ? "'\\''" : std::string(1, c);
    return out + "'";
}

// 调用外部命令并把输出逐行回显；返回退出码（-1 = 无法执行）
int RunAndEcho(const std::string &cmd)
{
    FILE *pipe = ::popen((cmd + " 2>&1").c_str(), "r");
    if (!pipe)
        return -1;
    char buf[512];
    while (std::fgets(buf, sizeof(buf), pipe))
        std::cout << buf;
    const int rc = ::pclose(pipe);
    if (WIFEXITED(rc))
        return WEXITSTATUS(rc);
    return -1;
}

// 读取最终主图 ID，返回-1时退回旧的全图一致性检查
int ReadPrimaryMapId()
{
    std::ifstream manifest("map_submaps.csv");
    std::string line;
    if (!std::getline(manifest, line))
        return -1;
    while (std::getline(manifest, line))
    {
        std::istringstream row(line);
        std::string id, keyframes, points, primary;
        if (!std::getline(row, id, ',') || !std::getline(row, keyframes, ',') ||
            !std::getline(row, points, ',') || !std::getline(row, primary))
            continue;
        if (!primary.empty() && primary.back() == '\r')
            primary.pop_back();
        if (primary == "1")
        {
            try { return std::stoi(id); }
            catch (...) { return -1; }
        }
    }
    return -1;
}

} // namespace

int main(int argc, char **argv)
{
    Options opt;
    if (!ParseArgs(argc, argv, &opt))
    {
        PrintUsage(argv[0]);
        return 2;
    }

    // 工具默认取本可执行文件同目录（与 orb_slam3_offline / session_scale_check.py 同级）
    const std::string self_dir = fs::absolute(argv[0]).parent_path().string();
    if (opt.scale_check_cmd.empty())
        opt.scale_check_cmd = "python3 " + ShellQuote(self_dir + "/session_submap_scale.py");
    if (opt.verify_tool.empty())
        opt.verify_tool = self_dir + "/orb_slam3_offline";

    opt.vocab = fs::absolute(opt.vocab).string();
    opt.settings = fs::absolute(opt.settings).string();
    if (!fs::exists(opt.vocab) || !fs::exists(opt.settings))
    {
        std::cerr << "词典或 settings 不存在：" << opt.vocab << " / " << opt.settings << std::endl;
        return 2;
    }

    const std::string session_root =
        (opt.session == "auto")
            // self_dir 已经是 <repo>/zhangqiang/tools，往上一级就是 zhangqiang/
            ? (fs::path(self_dir).parent_path() / "map_sessions" / TimestampTag()).string()
            : fs::absolute(opt.session).string();

    if (fs::exists(fs::path(session_root) / "online.log") || fs::exists(fs::path(session_root) / "map.osa"))
    {
        std::cerr << "会话目录已有建图产物，请指定新目录，避免混入历史地图" << std::endl;
        return 2;
    }
    SessionPaths paths = MakeSessionPaths(session_root);
    std::string err;
    if (!EnsureSessionDirs(paths, &err))
    {
        std::cerr << err << std::endl;
        return 2;
    }
    if (!WriteSessionSettings(opt.settings, paths.settings, opt.fps, &err))
    {
        std::cerr << err << std::endl;
        return 2;
    }
    // 切到会话目录：ORB-SLAM3 的 Atlas 保存与我们的 traj/ply 都是相对路径，
    // 不切目录会把 map.osa 落到启动目录（Config 里 SaveAtlasToFile 用的是 "./" 前缀）
    if (::chdir(session_root.c_str()) != 0)
    {
        std::cerr << "切到会话目录失败：" << std::strerror(errno) << std::endl;
        return 2;
    }

    LogTee tee(paths.log);
    std::cout << "================ 单目实时建图（纯单目，无 IMU） ================" << std::endl
              << "会话目录：" << session_root << std::endl
              << "词典：" << opt.vocab << std::endl
              << "配置：" << paths.settings << "（Camera.fps 必须等于 --fps " << opt.fps << "）"
              << std::endl
              << "相机：" << opt.camera << " " << opt.width << "x" << opt.height << "@" << opt.fps
              << "  翻正=" << (opt.flip ? "是" : "否") << std::endl
              << "交互：先停车，v + Enter 保存候选并进入回访；可跳过回访，按实时提示确认两个尺度点；"
                 "q / Ctrl+C 仅保存未验证地图" << std::endl
              << "==============================================================" << std::endl;

    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = OnSignal;   // 不设 SA_RESTART：Ctrl+C 能打断阻塞读
    ::sigaction(SIGINT, &sa, nullptr);
    ::sigaction(SIGTERM, &sa, nullptr);

    // 未合并分裂台账：右端=最近一次分裂，焊接成功就回收右端
    std::vector<std::vector<std::string>> pending_split_snaps;
    std::vector<double> pending_split_times;

    std::unique_ptr<V4l2MonoSource> source;
    std::unique_ptr<WheelOdomReader> odom;
    std::unique_ptr<SlamBackend> slam;
    std::unique_ptr<AtlasMonitor> monitor;
    SessionWriter writer(paths);

    source = std::make_unique<V4l2MonoSource>(opt.camera, opt.width, opt.height, opt.fps, opt.flip);
    if (!source->Open(&err))
    {
        std::cerr << "[FAIL] 相机启动失败：" << err << std::endl;
        return 3;
    }
    std::cout << "[capture] " << source->Describe() << std::endl;

    if (opt.wheel)
    {
        // 只读监听；失败不致命（尺度校验与区段报告会降级）
        odom = std::make_unique<WheelOdomReader>(opt.iface, 10, 11, 0.025, 0.132);
        if (odom->Open(&err) && odom->Start(&err, paths.wheel_odom))
            std::cout << "[odom] 只读轮速里程计已启动（" << opt.iface << "，从不写总线）" << std::endl;
        else
        {
            std::cout << "[warn] 轮速里程计不可用：" << err << "（尺度校验将缺少轮速侧证据）"
                      << std::endl;
            odom.reset();
        }
    }

    slam = std::make_unique<SlamBackend>(opt.vocab, paths.settings, SensorMode::Mono, false);
    if(!source->Start(&err))
    {
        std::cerr << "[FAIL] 采集启动失败：" << err << std::endl;
        return 3;
    }
    monitor = std::make_unique<AtlasMonitor>(&writer, &tee);
    std::cout << "[slam] ORB-SLAM3 已启动（MONOCULAR, viewer=off）" << std::endl;

    // 用户结束后直接停采集，尺度从已有数据估计，不再要求额外运动
    RunMappingLoop(source.get(), slam.get(), monitor.get(), &writer, 180.0, opt.record_frames,
                   &pending_split_snaps, &pending_split_times, &tee);

    // ---------- 收口：停采集 → 等后台静默 → 导出 → 保存 → 回读验证 ----------
    std::cout << "\n[exit] 停止采集线程" << std::endl;
    source->Stop();
    std::cout << "[exit] 请求后台建图/回环线程结束（有界等待）" << std::endl;
    slam->RequestMappingStop();

    if (odom) odom->Stop(); // 先落盘全部轮速样本，再估尺度
    try
    {
        slam->PrepareSubmapExport();
        slam->PrintSubmapSummary();
        monitor->OnFinish(NowSeconds(), slam->MapCount());
        // 保留原始逐帧轨迹用于诊断，定尺度使用每张子图优化后的关键帧轨迹
        slam->ExportTrajectory("traj_raw.csv");
        // 自动轮速尺度只做诊断，最终米制倍率由回访双点卷尺确定
        const int scale_rc = RunAndEcho(opt.scale_check_cmd + " --session . --no-apply");
        if(scale_rc != 0)
            std::cout << "[warn] 子图自动尺度诊断失败，地图保留原始单位" << std::endl;
        slam->SaveSubmaps(opt.save, false);
    }
    catch(const std::exception& error)
    {
        // 超时或写盘失败必须留存失败状态，不能触发 terminate 或进入回访
        std::cout << "[FAIL] 候选地图保存失败：" << error.what() << std::endl;
        writer.WriteMeta("{\n  \"saved\": false,\n  \"verify_ok\": false,\n"
                         "  \"failure_stage\": \"submap_export\"\n}\n");
        std::cout.flush();
        // 后台可能尚未结束，沿用正常出口的进程回收方式避免析构竞态
        _exit(5);
    }

    int verified = -1;
    int verified_individual = 0;
    int ignored_verify_failures = 0;
    bool verify_ok = false;
    if (opt.verify && opt.save && fs::exists("map.osa"))
    {
        std::vector<std::string> atlases = {"map.osa"};
        for(const auto& entry : fs::directory_iterator("."))
        {
            const std::string name = entry.path().filename().string();
            if(name.rfind("map_", 0) == 0 && entry.path().extension() == ".osa")
                atlases.push_back(name);
        }
        std::sort(atlases.begin() + 1, atlases.end());
        const int primary_id = ReadPrimaryMapId();
        bool primary_seen = false;
        verify_ok = true;
        for(const auto& path : atlases)
        {
            const std::string cmd = ShellQuote(opt.verify_tool) + " --inspect-atlas "
                                    + ShellQuote(path) + " " + ShellQuote(opt.vocab);
            FILE *pipe = ::popen((cmd + " 2>&1").c_str(), "r");
            int maps = -1;
            int rc = -1;
            if(pipe)
            {
                char buf[512];
                while(std::fgets(buf, sizeof(buf), pipe))
                {
                    if(std::string(buf).rfind("INERTIAL_MAP_GATE=", 0) == 0)
                        std::cout << "[verify] 纯单目：惯性初始化检查不适用，不代表地图损坏或尺度失败\n";
                    else std::cout << buf;
                    if(std::string(buf).rfind("ATLAS_READ_OK", 0) == 0)
                        std::sscanf(buf, "ATLAS_READ_OK maps=%d", &maps);
                }
                rc = ::pclose(pipe);
            }
            // 旧检查器对纯单目返回3表示未惯性初始化，不影响Atlas读回成功
            const bool good = rc >= 0 && WIFEXITED(rc)
                && (WEXITSTATUS(rc) == 0 || WEXITSTATUS(rc) == 3)
                && (path == "map.osa" ? maps > 0 : maps == 1);
            const bool is_container = path == "map.osa";
            const bool is_primary = primary_id >= 0 &&
                path == "map_" + std::to_string(primary_id) + ".osa";
            if(good) ++verified_individual;
            if(is_container) {
                if(good) verified = maps;
                verify_ok = verify_ok && good;
            } else if(is_primary) {
                primary_seen = true;
                verify_ok = verify_ok && good;
            } else if(!good) {
                ++ignored_verify_failures;
            }
            std::cout << "[verify] " << path << (good ? " OK" : " FAILED") << std::endl;
        }
        // 小图只保留供诊断；导航只要求完整容器和最大主图读回成功
        if(primary_id >= 0)
            verify_ok = verify_ok && primary_seen;
        else
            verify_ok = verify_ok && verified == verified_individual;
        if(ignored_verify_failures)
            std::cout << "[verify] 未融合小图读回失败 " << ignored_verify_failures
                      << " 个，已忽略，不影响最大主图\n";
    }

    // 会话元数据（供后续分析/报告脚本读取）
    std::ostringstream meta;
    meta << "{\n"
         << "  \"session\": \"" << session_root << "\",\n"
         << "  \"sensor\": \"monocular\",\n"
         << "  \"settings\": \"" << paths.settings << "\",\n"
         << "  \"camera\": \"" << opt.camera << "\",\n"
         << "  \"fps_target\": " << opt.fps << ",\n"
         << "  \"captured\": " << source->Stats().captured << ",\n"
         << "  \"dropped\": " << source->Stats().dropped << ",\n"
         << "  \"read_fail\": " << source->Stats().read_fail << ",\n"
         << "  \"measured_fps\": " << source->Stats().measured_fps << ",\n"
         << "  \"keyframe_images\": " << 0 << ",\n"
         << "  \"raw_frames_saved\": " << writer.RawFramesSaved() << ",\n"
         << "  \"new_map_events\": " << monitor->new_map_events() << ",\n"
         << "  \"merge_events\": " << monitor->merge_events() << ",\n"
         << "  \"verified_maps\": " << verified << ",\n"
         << "  \"verified_individual_maps\": " << verified_individual << ",\n"
         << "  \"ignored_submap_verify_failures\": " << ignored_verify_failures << ",\n"
         << "  \"verify_ok\": " << (verify_ok ? "true" : "false") << ",\n"
         << "  \"saved\": " << (opt.save && fs::exists("map.osa") ? "true" : "false") << "\n"
         << "}\n";
    writer.WriteMeta(meta.str());
    if(g_validate_requested && verify_ok) {
        std::ofstream request("validation_requested"); request << "1\n";
    }

    // 会话报告：轮速路径按子图着色 + 回扫建议（失败不影响主流程）
    std::cout << "[report] 生成子图区段报告…" << std::endl;
    RunAndEcho("python3 " + ShellQuote(self_dir + "/session_report.py") + " --session .");
    std::cout << "[done] 会话产物目录：" << session_root << std::endl;
    std::cout.flush();
    // 轨迹与地图都已落盘，直接退出，避开 ORB-SLAM3 退出阶段的析构崩溃
    _exit(opt.verify && opt.save && !verify_ok ? 4 : 0);
}
