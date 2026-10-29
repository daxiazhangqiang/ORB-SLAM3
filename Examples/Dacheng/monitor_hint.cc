#include "monitor_hint.h"

#include <iomanip>
#include <iostream>
#include <sstream>

// 注意：本文件全部用 std::cout 输出（不要用 printf）——
// LogTee 只接 cout，用 printf 的日志不会进 online.log

namespace dacheng
{

AtlasMonitor::AtlasMonitor(SessionWriter *writer, const LogTee *tee) : writer_(writer), tee_(tee) {}

void AtlasMonitor::Initialize(int state, int map_count, double t)
{
    last_state_ = state;
    last_map_count_ = map_count;
    max_map_count_ = map_count;
    last_summary_t_ = t;
    inited_ = true;
    std::cout << "[init] 跟踪状态=" << TrackingStateName(state) << "  子图数=" << map_count
              << std::endl;
    if (writer_)
        writer_->AppendEvent(t, map_count, "session_start");
}

void AtlasMonitor::OnFrame(int state, int map_count, const SensorStats &sensor, double frame_cost_ms,
                           double t, int extracted, int matched)
{
    if (!inited_)
    {
        Initialize(state, map_count, t);
        return;
    }

    // 使用局部流格式化，避免改变其他线程日志的小数精度
    std::ostringstream output;
    if (state != last_state_)
    {
        output << "[" << std::setw(8) << std::fixed << std::setprecision(1) << t
                  << "s] TRACKING  " << TrackingStateName(last_state_) << " -> "
                  << TrackingStateName(state) << std::endl;
        // 进入 LOST / RECENTLY_LOST 时记一次现场（主循环会存下这一刻前后的图像）
        if (state == 3 || state == 4)
            lost_pending_ = true;
        last_state_ = state;
    }

    if (map_count != last_map_count_)
    {
        const bool grew = map_count > last_map_count_;
        output << "\n[" << t << "s] ⚠ ATLAS 子图数 " << last_map_count_ << " -> " << map_count
                  << (grew ? "（新建了子图）" : "（疑似融合）") << std::endl;
        if (grew && writer_)
        {
            new_map_events_++;
            if (map_split_since_ <= 0.0)
                map_split_since_ = t;
            writer_->AppendEvent(t, map_count, "new_map");
            new_map_pending_ = true;
            output << "            → 已保存 lost_frames/split/lost_split_*.jpg；按低频子图统计决定是否需要补扫\n";
        }
        else if (!grew && writer_)
        {
            writer_->AppendEvent(t, map_count, "map_count_down");
        }
        last_map_count_ = map_count;
        if (map_count > max_map_count_)
            max_map_count_ = map_count;
        if (map_count < 2)
        {
            map_split_since_ = 0.0;
            hinted_60s_ = false;
            hinted_300s_ = false;
        }
    }

    // ORB-SLAM3 的融合/回环标记（来自 stdout tee）
    if (tee_)
    {
        const int md = tee_->merge_detected();
        const int mf = tee_->merge_finished();
        if (mf > merge_finished_seen_)
        {
            const int delta = mf - merge_finished_seen_;
            merge_finished_seen_ = mf;
            merge_seen_ += delta;
            merge_pending_ = true;
            output << "\n[" << t << "s] ✅ 检测到 Merge finished（第 " << mf
                      << " 次）→ 子图已焊接；建议沿主路线完整回访一遍确认不再新增子图"
                      << std::endl;
            if (writer_)
                writer_->AppendEvent(t, map_count, "merge_finished");
        }
        else if (md > 0 && !merge_announced_)
        {
            merge_announced_ = true;
            output << "\n[" << t << "s] 检测到 *Merge detected：正在做跨子图重定位与 Sim3 焊接…"
                      << std::endl;
        }
    }

    // 每5秒一条摘要；单帧耗时长期超预算时提示一次
    frame_cost_sum_ += frame_cost_ms;
    frame_cost_n_++;
    if (t - last_summary_t_ >= 5.0)
    {
        const double avg_cost = frame_cost_n_ > 0 ? frame_cost_sum_ / frame_cost_n_ : 0.0;
        output << "[" << std::setw(8) << std::fixed << std::setprecision(1) << t << "s] 子图 "
                  << std::setw(2) << map_count << "  跟踪 " << std::setw(14)
                  << TrackingStateName(state) << "  输入 " << std::setprecision(1)
                  << sensor.measured_fps << " fps（丢 " << sensor.dropped << "）  单帧 "
                  << std::setprecision(0) << avg_cost << " ms";
        if (extracted >= 0)
            output << "  特征 " << extracted << "  匹配 " << matched;
        output << std::endl;
        if (sensor.measured_fps > 0.0 && avg_cost > 1000.0 / sensor.measured_fps)
        {
            slow_seconds_ += 5;
            if (slow_seconds_ >= 5 && !hinted_slow_)
            {
                hinted_slow_ = true;
                output << "           ⚠ 单帧耗时长期超过实测相机帧间隔预算："
                             "现在按“只取最新帧”丢帧运行（有效帧率下降）\n"
                             "              建议：检查CPU负载、采集延迟及运动模糊；"
                             "保持标定分辨率和角点阈值不变" << std::endl;
            }
        }
        else
        {
            slow_seconds_ = 0;
        }
        last_summary_t_ = t;
        frame_cost_sum_ = 0.0;
        frame_cost_n_ = 0;
    }
    const std::string message = output.str();
    if (!message.empty()) {
        std::cout << message;
        std::cout.flush();
    }
}

void AtlasMonitor::OnFinish(double t, int final_map_count)
{
    // 停机和PreSave后空图可能已被清理，以最终Atlas为准
    last_map_count_ = final_map_count;
    if(tee_) merge_seen_ = tee_->merge_finished();
    std::cout << "\n[" << t << "s] 会话结束：本次共新建子图事件 " << new_map_events_
              << " 次，融合完成 " << merge_seen_ << " 次，最终子图数 " << last_map_count_
              << std::endl;
    if (last_map_count_ > 1)
        std::cout << "           ⚠ 未收敛为单图：本次输出不是一张完整地图；"
                     "各子图独立保存，定位可只加载最大的主图，无需强行融合小图" << std::endl;
    else if (last_map_count_ == 1)
        std::cout << "           ✓ 本次收敛为单图（注意：单图 ≠ 全区域覆盖完整）" << std::endl;
    if (writer_)
        writer_->AppendEvent(t, last_map_count_, "session_end");
    std::cout.flush();
}

} // namespace dacheng
