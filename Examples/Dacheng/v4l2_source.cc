#include "v4l2_source.h"

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <iostream>

namespace dacheng
{

V4l2MonoSource::V4l2MonoSource(std::string device, int width, int height, double fps, bool flip)
    : device_(std::move(device)), width_(width), height_(height), fps_(fps), flip_(flip)
{
}

V4l2MonoSource::~V4l2MonoSource()
{
    Stop();
}

bool V4l2MonoSource::Open(std::string *err)
{
    // MJPG 才能跑满 640x480@20；YUYV 在 USB2.0 上带不动
    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened())
    {
        // 不是 V4L2 设备就按文件/图像序列打开（例如 /path/seq_%07d.jpg，用于离线复跑验证）
        cap_.open(device_, cv::CAP_ANY);
        file_mode_ = cap_.isOpened();
        if (file_mode_)
            std::cout << "[capture] " << device_ << " 按视频/图像序列打开（离线复跑模式）"
                      << std::endl;
    }
    if (!cap_.isOpened())
    {
        if (err) *err = "打不开相机 " + device_;
        return false;
    }
    if (!file_mode_)
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    if (!file_mode_)
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);   // 只要最新缓冲，减少陈旧帧

    // 实机必须先锁定标定焦点并读回，禁止带错误内参开始建图
    if (!file_mode_)
    {
        const bool auto_ok = cap_.set(cv::CAP_PROP_AUTOFOCUS, 0);
        const bool focus_ok = cap_.set(cv::CAP_PROP_FOCUS, 192);
        const double autofocus = cap_.get(cv::CAP_PROP_AUTOFOCUS);
        const double focus = cap_.get(cv::CAP_PROP_FOCUS);
        const double fourcc = cap_.get(cv::CAP_PROP_FOURCC);
        if (!auto_ok || !focus_ok || !std::isfinite(autofocus) || autofocus != 0 ||
            !std::isfinite(focus) || focus != 192 ||
            fourcc != cv::VideoWriter::fourcc('M', 'J', 'P', 'G') ||
            width_ != 640 || height_ != 480 || !flip_)
        {
            if (err) *err = "标定相机模式校验失败：必须 MJPG、640x480、关闭自动对焦、焦点192、镜像翻正";
            cap_.release();
            return false;
        }
        std::cout << "[camera_gate] autofocus=0 focus=192 MJPG flip=true" << std::endl;
    }

    const int got_w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    const int got_h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double got_fps = cap_.get(cv::CAP_PROP_FPS);
    std::cout << "[capture] " << device_ << " 实际 " << got_w << "x" << got_h
              << " @" << got_fps << " fps（申请 " << width_ << "x" << height_ << "@" << fps_ << "）"
              << std::endl;
    if (got_w != width_ || got_h != height_)
    {
        if (err) *err = "相机实际分辨率与申请不一致（标定失效，必须先解决）";
        return false;
    }
    if (!file_mode_ && (!std::isfinite(got_fps) || got_fps <= 1.0 ||
                        std::abs(got_fps - fps_) / fps_ > 0.15))
    {
        if (err) *err = "相机协商帧率与标定运行配置不匹配，拒绝建图";
        cap_.release();
        return false;
    }
    return true;
}

bool V4l2MonoSource::Start(std::string *err)
{
    if (!cap_.isOpened())
    {
        if (err) *err = "采集线程启动前必须先 Open()";
        return false;
    }
    stop_ = false;
    fps_win_start_ = NowSeconds();
    fps_win_frames_ = 0;
    thread_ = std::thread(&V4l2MonoSource::CaptureLoop, this);
    return true;
}

void V4l2MonoSource::CaptureLoop()
{
    cv::Mat frame;
    double next_due = 0.0;
    while (!stop_.load())
    {
        if (!cap_.read(frame) || frame.empty())
        {
            if (file_mode_)
            {
                // 文件/图像序列读完：停在这里等 Stop()，不要忙等刷屏
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            std::lock_guard<std::mutex> lock(mtx_);
            stats_.read_fail++;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (flip_)
            cv::flip(frame, frame, 1);   // 原始帧左右镜像，按标定约定翻正
        double ts = NowSeconds();
        if (file_mode_)
        {
            // 离线复跑：按标称帧率节流，保证相邻帧的位移与真实采集一致
            const double period = 1.0 / std::max(1.0, fps_);
            const double now = NowSeconds();
            if (next_due <= 0.0 || now > next_due + period)
                next_due = now;
            next_due += period;
            const double sleep_s = next_due - (now + 0.001);
            if (sleep_s > 0)
                std::this_thread::sleep_for(
                    std::chrono::microseconds(static_cast<long>(sleep_s * 1e6)));
            ts = NowSeconds();
        }
        Publish(frame.clone(), ts);
    }
}

void V4l2MonoSource::Publish(const cv::Mat &frame, double ts)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (has_new_)
        stats_.dropped++;               // 上一帧还没被取走就被覆盖
    latest_ = frame;
    latest_ts_ = ts;
    latest_seq_++;
    has_new_ = true;
    ready_.notify_one();
    stats_.captured++;
    stats_.last_frame_ts = ts;

    fps_win_frames_++;
    const double now = ts;
    if (now - fps_win_start_ >= 2.0)
    {
        stats_.measured_fps = fps_win_frames_ / (now - fps_win_start_);
        fps_win_start_ = now;
        fps_win_frames_ = 0;
    }
}

bool V4l2MonoSource::WaitLatest(FramePacket *out, int timeout_ms)
{
    // 有新帧或停止时才唤醒，避免每2毫秒轮询
    std::unique_lock<std::mutex> lock(mtx_);
    if(!ready_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                       [this] { return has_new_ || stop_.load(); }) || stop_.load())
        return false;
    out->timestamp = latest_ts_;
    out->seq = latest_seq_;
    out->left = latest_;
    out->right.release();
    has_new_ = false;
    return true;
}

void V4l2MonoSource::Stop()
{
    { std::lock_guard<std::mutex> lock(mtx_); stop_ = true; }
    ready_.notify_all();
    if (thread_.joinable())
        thread_.join();
    if (cap_.isOpened())
        cap_.release();
}

SensorStats V4l2MonoSource::Stats() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return stats_;
}

std::string V4l2MonoSource::Describe() const
{
    return "v4l2 mono " + device_ + " " + std::to_string(width_) + "x" + std::to_string(height_) +
           "@" + std::to_string(static_cast<int>(fps_)) + (flip_ ? " 已翻正" : " 未翻正");
}

} // namespace dacheng
