#include "session_io.h"

#include "realtime_types.h"

#include <opencv2/imgcodecs.hpp>

#include <cstdio>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

namespace dacheng
{

SessionPaths MakeSessionPaths(const std::string &root)
{
    SessionPaths p;
    p.root = root;
    p.log = root + "/online.log";
    p.settings = root + "/settings.yaml";
    p.traj = root + "/traj.csv";
    p.kf_traj = root + "/traj_kf.csv";
    p.events = root + "/submap_events.csv";
    p.wheel_odom = root + "/wheel_odom.csv";
    p.meta = root + "/session_meta.json";
    p.ring_dir = root + "/recent_frames";
    p.lost_dir = root + "/lost_frames";
    p.split_dir = root + "/lost_frames/split";
    p.weak_dir = root + "/lost_frames/quality_weak";
    p.ply = root + "/map_points.ply";
    return p;
}

bool EnsureSessionDirs(const SessionPaths &paths, std::string *err)
{
    try
    {
        fs::create_directories(paths.root);
        fs::create_directories(paths.ring_dir);
        fs::create_directories(paths.lost_dir);
        fs::create_directories(paths.split_dir);
        fs::create_directories(paths.weak_dir);
    }
    catch (const std::exception &e)
    {
        if (err) *err = std::string("创建会话目录失败：") + e.what();
        return false;
    }
    return true;
}

LogTee::LogTee(const std::string &path)
{
    file_.open(path, std::ios::out | std::ios::trunc);
    old_ = std::cout.rdbuf(this);
}

LogTee::~LogTee()
{
    if (old_)
        std::cout.rdbuf(old_);
    if (file_.is_open())
    {
        file_.flush();
        file_.close();
    }
}

int LogTee::merge_detected() const { return merge_detected_; }
int LogTee::merge_finished() const { return merge_finished_; }
int LogTee::loop_detected() const { return loop_detected_; }
int LogTee::new_map_id_counter() const { return new_map_id_counter_; }
int LogTee::new_map_id() const { return new_map_id_; }
int LogTee::change_map_counter() const { return change_map_counter_; }
int LogTee::change_map_id() const { return change_map_id_; }

void LogTee::Feed(const char *s, size_t n)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (file_.is_open())
        file_.write(s, static_cast<std::streamsize>(n));
    if (old_)
        old_->sputn(s, static_cast<std::streamsize>(n));

    // 逐行扫描 ORB-SLAM3 的标记（只在 VERBOSITY_QUIET 打印，恒可见）
    line_buf_.append(s, n);
    size_t pos = 0;
    while ((pos = line_buf_.find('\n')) != std::string::npos)
    {
        const std::string line = line_buf_.substr(0, pos);
        line_buf_.erase(0, pos + 1);
        if (line.find("*Merge detected") != std::string::npos)
            merge_detected_++;
        if (line.find("Merge finished!") != std::string::npos)
            merge_finished_++;
        if (line.find("*Loop detected") != std::string::npos)
            loop_detected_++;
        // 子图创建 / 换图：把 map id 抠出来，供“关键帧按子图归档”使用
        std::size_t pos_new = line.find("Creation of new map with id: ");
        if (pos_new != std::string::npos)
        {
            new_map_id_ = std::atoi(line.c_str() + pos_new + 29);
            new_map_id_counter_++;
        }
        std::size_t pos_chg = line.find("Change to map with id: ");
        if (pos_chg != std::string::npos)
        {
            change_map_id_ = std::atoi(line.c_str() + pos_chg + 23);
            change_map_counter_++;
        }
    }
    if (line_buf_.size() > 4096)   // 防御：异常长行不留内存
        line_buf_.clear();
}

int LogTee::overflow(int ch)
{
    if (ch == EOF)
        return 0;
    const char c = static_cast<char>(ch);
    if (c == '\n')
    {
        Feed("\n", 1);
        return ch;
    }
    Feed(&c, 1);
    return ch;
}

std::streamsize LogTee::xsputn(const char *s, std::streamsize n)
{
    Feed(s, static_cast<size_t>(n));
    return n;
}

int LogTee::sync()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (file_.is_open())
        file_.flush();
    if (old_)
        old_->pubsync();
    return 0;
}

SessionWriter::SessionWriter(const SessionPaths &paths) : paths_(paths)
{
    events_.open(paths_.events, std::ios::out | std::ios::trunc);
    if (events_.is_open())
        events_ << "timestamp_ns,map_id,event\n";
    loss_csv_.open(paths_.root + "/loss_stats.csv", std::ios::out | std::ios::trunc);
    if (loss_csv_.is_open())
        loss_csv_ << "timestamp_ns,event,map_id,extracted_last,matched_last,extracted_mean2s,"
                     "matched_mean2s,cost_ms,delta_matched_pct,verdict\n";
}

void SessionWriter::AppendLossStat(double t, const std::string &event, int map_id,
                                   int extracted_last, int matched_last, double extracted_mean2s,
                                   double matched_mean2s, double cost_ms, double delta_matched_pct,
                                   const std::string &verdict)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (!loss_csv_.is_open())
        return;
    loss_csv_ << static_cast<long long>(t * 1e9) << ',' << event << ',' << map_id << ','
              << extracted_last << ',' << matched_last << ',' << extracted_mean2s << ','
              << matched_mean2s << ',' << cost_ms << ',' << delta_matched_pct << ',' << verdict
              << '\n';
    loss_csv_.flush();
    loss_rows_++;
}

void SessionWriter::SaveRawFrame(const cv::Mat &image, double t)
{
    if (image.empty())
        return;
    const std::string dir = paths_.root + "/frames";
    std::lock_guard<std::mutex> lock(mtx_);
    fs::create_directories(dir);
    char name[512];
    std::snprintf(name, sizeof(name), "%s/%010lld.jpg", dir.c_str(),
                  static_cast<long long>(t * 10));
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 90};
    if (cv::imwrite(name, image, params))
        raw_saved_++;
}

void SessionWriter::AppendEvent(double t, int map_id, const std::string &event)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (!events_.is_open())
        return;
    const long long ns = static_cast<long long>(t * 1e9);
    events_ << ns << ',' << map_id << ',' << event << '\n';
    events_.flush();
    event_rows_++;
}

std::string SessionWriter::SaveSnapshot(const cv::Mat &image, double t, const std::string &tag)
{
    if (image.empty())
        return std::string();
    char name[256];
    const std::string &dir = tag.rfind("lost_split", 0) == 0 ? paths_.split_dir :
                             tag == "quality_weak" ? paths_.weak_dir : paths_.lost_dir;
    // 毫秒级时间戳，便于把“丢失瞬间 / 前 1 s / 前 2 s”三张按时间排序
    std::snprintf(name, sizeof(name), "%s/%s_%013lld.jpg", dir.c_str(), tag.c_str(),
                  static_cast<long long>(t * 1000.0));
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 90};
    if (!cv::imwrite(name, image, params))
        return std::string();
    return std::string(name);
}

bool SessionWriter::RemoveFile(const std::string &path)
{
    if (path.empty())
        return false;
    return std::remove(path.c_str()) == 0;
}

void SessionWriter::PushRingFrame(const cv::Mat &image, double t, double keep_seconds)
{
    if (image.empty())
        return;
    char name[256];
    std::snprintf(name, sizeof(name), "%s/ring_ns_%019lld.jpg", paths_.ring_dir.c_str(),
                  static_cast<long long>(t * 1e9));
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 85};
    if (!cv::imwrite(name, image, params))
        return;

    std::lock_guard<std::mutex> lock(mtx_);
    ring_.emplace_back(t, std::string(name));
    while (!ring_.empty() && t - ring_.front().first > keep_seconds)
    {
        std::remove(ring_.front().second.c_str());
        ring_.erase(ring_.begin());
    }
}

void SessionWriter::WriteMeta(const std::string &json) const
{
    std::ofstream f(paths_.meta, std::ios::out | std::ios::trunc);
    if (f.is_open())
        f << json << '\n';
}

size_t SessionWriter::EventCount() const { return event_rows_; }

} // namespace dacheng
