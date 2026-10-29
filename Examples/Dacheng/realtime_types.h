#ifndef DACHENG_REALTIME_TYPES_H
#define DACHENG_REALTIME_TYPES_H

// 实时建图封装层的公共类型：帧包、跟踪状态名、采集统计
// 双目扩展点：FramePacket 里 right 为空即单目，填上就是双目

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

namespace dacheng
{

struct FramePacket
{
    double timestamp = 0.0;      // 单调时钟秒（CLOCK_MONOTONIC 口径）
    uint64_t seq = 0;            // 采集序号（从 1 开始）
    cv::Mat left;                // 单目就是这一帧；双目为左目
    cv::Mat right;               // 双目右目，单目为空

    bool Empty() const { return left.empty(); }
    bool IsStereo() const { return !right.empty(); }
};

struct SensorStats
{
    uint64_t captured = 0;       // 采集线程取到的总帧数
    uint64_t dropped = 0;        // 被覆盖掉的旧帧数（SLAM 处理不过来）
    uint64_t read_fail = 0;      // 读帧失败次数
    double last_frame_ts = 0.0;
    double measured_fps = 0.0;   // 滑动窗口实测输入帧率
};

// 单调时钟秒，与 ROS/相机/轮速/子图事件同一时基
inline double NowSeconds()
{
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// Tracking::eTrackingState → 可读字符串
inline const char *TrackingStateName(int state)
{
    switch (state)
    {
    case -1: return "SYSTEM_NOT_READY";
    case 0:  return "NO_IMAGES_YET";
    case 1:  return "NOT_INITIALIZED";
    case 2:  return "OK";
    case 3:  return "RECENTLY_LOST";
    case 4:  return "LOST";
    case 5:  return "OK_KLT";
    default: return "UNKNOWN";
    }
}

// 采集接口：单目/双目/其它来源都实现它，SLAM 主线程只认这个接口
class ISensorSource
{
public:
    virtual ~ISensorSource() = default;

    virtual bool Open(std::string *err) = 0;
    virtual bool Start(std::string *err) = 0;
    // 取“最新一帧”（容量 1，旧帧被覆盖）；timeout_ms 内没新帧返回 false
    virtual bool WaitLatest(FramePacket *out, int timeout_ms) = 0;
    virtual void Stop() = 0;
    virtual SensorStats Stats() const = 0;
    virtual std::string Describe() const = 0;
};

} // namespace dacheng

#endif // DACHENG_REALTIME_TYPES_H
