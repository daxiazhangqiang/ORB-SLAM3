#ifndef DACHENG_WHEEL_ODOM_H
#define DACHENG_WHEEL_ODOM_H

// 只读轮速里程计：监听 can0 的两轮反馈帧，差速积分出 (x, y, yaw)
// 纪律：只用只读 socket，任何情况下不向 CAN 写帧（写权限归 motion_daemon）
// 用途：退出时直线段判定与尺度恢复、子图区段报告

#include <atomic>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace dacheng
{

struct WheelOdomSample
{
    double t = 0.0;        // 采样时刻（单调时钟秒）
    double x = 0.0, y = 0.0, yaw = 0.0;
    double v = 0.0;        // 车体线速度 m/s
    double omega = 0.0;    // 车体角速度 rad/s
    double v_right = 0.0, v_left = 0.0;   // 车体口径轮速 m/s
};

class WheelOdomReader
{
public:
    WheelOdomReader(std::string iface, int right_id, int left_id, double wheel_radius_m,
                    double track_m);
    ~WheelOdomReader();

    bool Open(std::string *err);
    bool Start(std::string *err, const std::string &csv_path);
    void Stop();

    WheelOdomSample Latest() const;
    uint64_t Frames() const { return frames_; }
    bool Alive(double timeout_s = 1.0) const;

private:
    void Loop();
    void OnFrame(int motor_id, double pos_rad, double vel_rad_s, double recv_t);
    void Advance(double now);

    std::string iface_;
    int right_id_;
    int left_id_;
    double radius_;
    double track_;
    int fd_ = -1;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<uint64_t> frames_{0};
    std::ofstream csv_;

    mutable std::mutex mtx_;
    // 单轮状态
    bool has_[2] = {false, false};        // [0]=right [1]=left
    double last_pos_[2] = {0.0, 0.0};
    double turn_[2] = {0.0, 0.0};         // 累计转角（车体口径，已处理反向与环上解包）
    double last_t_[2] = {0.0, 0.0};
    double last_vel_[2] = {0.0, 0.0};
    double prev_turn_[2] = {0.0, 0.0};
    uint64_t csv_rows_ = 0;
    WheelOdomSample pose_;
};

} // namespace dacheng

#endif // DACHENG_WHEEL_ODOM_H
