#ifndef DACHENG_V4L2_SOURCE_H
#define DACHENG_V4L2_SOURCE_H

// 单目 V4L2 采集：独占相机 → 采集线程 → 单槽最新帧（容量 1）
// 只负责取帧与打时间戳；不做保存、不做 SLAM
// 原始帧是左右镜像，按标定约定默认翻正（与离线工具一致）

#include "realtime_types.h"
#include "opencv2/videoio.hpp"

#include <mutex>
#include <condition_variable>
#include <string>
#include <thread>

namespace dacheng
{

class V4l2MonoSource : public ISensorSource
{
public:
    V4l2MonoSource(std::string device, int width, int height, double fps, bool flip);
    ~V4l2MonoSource() override;

    bool Open(std::string *err) override;
    bool Start(std::string *err) override;
    bool WaitLatest(FramePacket *out, int timeout_ms) override;
    void Stop() override;
    SensorStats Stats() const override;
    std::string Describe() const override;

private:
    void CaptureLoop();
    // 内部近似单向队列：只保留最新一帧，覆盖旧帧并计丢帧
    void Publish(const cv::Mat &frame, double ts);

    std::string device_;
    int width_;
    int height_;
    double fps_;
    bool flip_;

    cv::VideoCapture cap_;
    bool file_mode_ = false;     // 视频文件/图像序列（离线复跑用），跳过 V4L2 专有设置
    std::thread thread_;
    std::atomic<bool> stop_{false};
    mutable std::mutex mtx_;
    std::condition_variable ready_;
    cv::Mat latest_;
    double latest_ts_ = 0.0;
    uint64_t latest_seq_ = 0;
    bool has_new_ = false;
    SensorStats stats_;
    double fps_win_start_ = 0.0;
    uint64_t fps_win_frames_ = 0;
};

} // namespace dacheng

#endif // DACHENG_V4L2_SOURCE_H
