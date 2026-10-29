#ifndef DACHENG_SESSION_IO_H
#define DACHENG_SESSION_IO_H

// 会话目录与日志：目录布局、stdout tee 到 online.log、事件 CSV、快照、1FPS 抽帧环、会话元数据
// 只做文件 IO，不接触 SLAM 与算法

#include <cstdint>
#include <atomic>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace dacheng
{

struct SessionPaths
{
    std::string root;
    std::string log;         // online.log
    std::string settings;    // settings.yaml（本次实际使用的副本）
    std::string traj;        // traj.csv（最大子图逐帧位姿）
    std::string kf_traj;     // traj_kf.csv
    std::string events;      // submap_events.csv
    std::string wheel_odom;  // wheel_odom.csv
    std::string meta;        // session_meta.json
    std::string ring_dir;    // recent_frames/
    std::string lost_dir;    // lost_frames/（保留给旧调用）
    std::string split_dir;   // lost_frames/split（子图分裂现场）
    std::string weak_dir;    // lost_frames/quality_weak（质量薄弱现场）
    std::string ply;         // map_points.ply
};

SessionPaths MakeSessionPaths(const std::string &root);
bool EnsureSessionDirs(const SessionPaths &paths, std::string *err);

// stdout 复制到 online.log，顺带统计 ORB-SLAM3 的融合/回环标记
class LogTee : public std::streambuf
{
public:
    explicit LogTee(const std::string &path);
    ~LogTee() override;

    bool ok() const { return file_.is_open(); }
    int merge_detected() const;
    int merge_finished() const;
    int loop_detected() const;
    // 子图切换相关标记（用于把关键帧按子图归档）
    int new_map_id_counter() const;      // "Creation of new map with id: X" 出现次数
    int new_map_id() const;              // 最近一次的 X
    int change_map_counter() const;      // "Change to map with id: N" 出现次数（融合/重定位换图）
    int change_map_id() const;           // 最近一次的 N

protected:
    int overflow(int ch) override;
    std::streamsize xsputn(const char *s, std::streamsize n) override;
    int sync() override;

private:
    void Feed(const char *s, size_t n);

    std::streambuf *old_ = nullptr;
    std::ofstream file_;
    std::mutex mtx_;
    std::string line_buf_;
    std::atomic<int> merge_detected_ = 0;
    std::atomic<int> merge_finished_ = 0;
    std::atomic<int> loop_detected_ = 0;
    std::atomic<int> new_map_id_counter_ = 0;
    std::atomic<int> new_map_id_ = -1;
    std::atomic<int> change_map_counter_ = 0;
    std::atomic<int> change_map_id_ = -1;
};

// 会话数据落盘（线程内调用即可，内部自带锁）
class SessionWriter
{
public:
    explicit SessionWriter(const SessionPaths &paths);

    void AppendEvent(double t, int map_id, const std::string &event);
    // 返回落盘路径（失败返回空串），便于后续按“焊接结果”回收快照
    std::string SaveSnapshot(const cv::Mat &image, double t, const std::string &tag);
    // 删除单个文件（用于焊接成功后回收该分裂的快照）
    bool RemoveFile(const std::string &path);
    void PushRingFrame(const cv::Mat &image, double t, double keep_seconds);
    // 丢失点归因统计：每次 LOST/分裂写一行，含断前 2 s 的特征/匹配趋势
    void AppendLossStat(double t, const std::string &event, int map_id, int extracted_last,
                        int matched_last, double extracted_mean2s, double matched_mean2s,
                        double cost_ms, double delta_matched_pct, const std::string &verdict);
    // 全帧落盘（--record-frames：便于离线复跑，磁盘换可复现性）
    void SaveRawFrame(const cv::Mat &image, double t);
    void WriteMeta(const std::string &json) const;
    size_t EventCount() const;
    uint64_t RawFramesSaved() const { return raw_saved_; }

private:
    SessionPaths paths_;
    std::ofstream events_;
    std::ofstream loss_csv_;
    mutable std::mutex mtx_;
    std::vector<std::pair<double, std::string>> ring_;   // (t, 路径)
    size_t event_rows_ = 0;
    size_t loss_rows_ = 0;
    uint64_t raw_saved_ = 0;
};

} // namespace dacheng

#endif // DACHENG_SESSION_IO_H
