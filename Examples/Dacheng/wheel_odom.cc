#include "wheel_odom.h"

#include "realtime_types.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <iostream>

namespace dacheng
{
namespace
{
constexpr uint32_t kCanEffFlag = 0x80000000u;
constexpr int kCommType2 = 0x02;
constexpr int kCommType24 = 0x18;
constexpr double kPosLo = -12.57, kPosHi = 12.57;
constexpr double kVelLo = -50.0, kVelHi = 50.0;

// 无符号整型还原浮点（与 daemon / dacheng_odom 同口径）
double UintToFloat(uint16_t raw, double lo, double hi)
{
    return static_cast<double>(raw) / 65535.0 * (hi - lo) + lo;
}

// 环上最短角差：位置字段只有 ±12.57 rad 窗口
double RingDelta(double a, double b)
{
    const double span = kPosHi - kPosLo;
    const double half = span / 2.0;
    double d = std::fmod(b - a + half, span);
    if (d < 0)
        d += span;
    return d - half;
}
} // namespace

WheelOdomReader::WheelOdomReader(std::string iface, int right_id, int left_id, double wheel_radius_m,
                                 double track_m)
    : iface_(std::move(iface)), right_id_(right_id), left_id_(left_id), radius_(wheel_radius_m),
      track_(track_m)
{
}

WheelOdomReader::~WheelOdomReader()
{
    Stop();
}

bool WheelOdomReader::Open(std::string *err)
{
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0)
    {
        if (err) *err = "创建 CAN 只读 socket 失败";
        return false;
    }
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        if (err) *err = "找不到 CAN 接口 " + iface_;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        if (err) *err = "bind CAN socket 失败（CAN 接口是否 up？）";
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 20000;   // 20 ms 收帧超时，保证线程可退出
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return true;
}

bool WheelOdomReader::Start(std::string *err, const std::string &csv_path)
{
    if (fd_ < 0)
    {
        if (err) *err = "轮速里程计启动前必须先 Open()";
        return false;
    }
    if (!csv_path.empty())
    {
        csv_.open(csv_path, std::ios::out | std::ios::trunc);
        if (!csv_.is_open())
        {
            if (err) *err = "打不开 wheel_odom.csv：" + csv_path;
            return false;
        }
        csv_ << "timestamp_ns,x,y,yaw_rad,v_m_s,omega_rad_s,v_right_m_s,v_left_m_s\n";
    }
    stop_ = false;
    thread_ = std::thread(&WheelOdomReader::Loop, this);
    return true;
}

void WheelOdomReader::Loop()
{
    struct can_frame frame;
    double last_csv_t = -1.0;
    while (!stop_.load())
    {
        const ssize_t n = ::read(fd_, &frame, sizeof(frame));
        if (n < static_cast<ssize_t>(sizeof(frame)))
            continue;   // 超时或短读，直接下一轮
        if (frame.can_id & CAN_ERR_FLAG)
            continue;

        const uint32_t eid = frame.can_id & ~kCanEffFlag;
        const int comm = (eid >> 24) & 0x1F;
        if (comm != kCommType2 && comm != kCommType24)
            continue;
        const int motor_id = (eid >> 8) & 0xFF;
        if (motor_id != right_id_ && motor_id != left_id_)
            continue;
        if (frame.can_dlc < 8)
            continue;

        const uint16_t raw_pos = static_cast<uint16_t>((frame.data[0] << 8) | frame.data[1]);
        const uint16_t raw_vel = static_cast<uint16_t>((frame.data[2] << 8) | frame.data[3]);
        const double pos = UintToFloat(raw_pos, kPosLo, kPosHi);
        const double vel = UintToFloat(raw_vel, kVelLo, kVelHi);
        const double recv_t = NowSeconds();
        OnFrame(motor_id, pos, vel, recv_t);

        // 20 Hz 落一份里程计样本，够做尺度与区段报告
        if (recv_t - last_csv_t >= 0.05)
        {
            last_csv_t = recv_t;
            const WheelOdomSample s = Latest();
            if (csv_.is_open() && s.t > 0.0 && recv_t - s.t < 0.01)
            {
                const long long ns = static_cast<long long>(s.t * 1e9);
                csv_ << ns << ',' << s.x << ',' << s.y << ',' << s.yaw << ',' << s.v << ','
                     << s.omega << ',' << s.v_right << ',' << s.v_left << '\n';
                csv_rows_++;
            }
        }
        frames_++;
    }
}

void WheelOdomReader::OnFrame(int motor_id, double pos_rad, double vel_rad_s, double recv_t)
{
    std::lock_guard<std::mutex> lock(mtx_);
    const int idx = (motor_id == right_id_) ? 0 : 1;
    if (has_[idx] && recv_t > last_t_[idx] && recv_t - last_t_[idx] <= 0.25)
    {
        // 位置法积分：环上解包后累计转角；左轮反向安装 → 取反成“向前为正”
        double delta = RingDelta(last_pos_[idx], pos_rad);
        if (idx == 1)
            delta = -delta;
        turn_[idx] += delta;
    }
    has_[idx] = true;
    last_pos_[idx] = pos_rad;
    last_vel_[idx] = vel_rad_s;
    last_t_[idx] = recv_t;
    Advance(recv_t);
}

void WheelOdomReader::Advance(double now)
{
    if (!has_[0] || !has_[1])
        return;
    // 任一车轮掉数时不混用过期反馈，保留CSV时间间隙供尺度门禁拒绝
    if (now - last_t_[0] > 0.25 || now - last_t_[1] > 0.25)
    {
        prev_turn_[0] = turn_[0];
        prev_turn_[1] = turn_[1];
        return;
    }
    // 车体口径轮速 (m/s)：右轮直接用，左轮反向取反
    const double vr = last_vel_[0] * radius_;
    const double vl = -last_vel_[1] * radius_;
    const double dr = (turn_[0] - prev_turn_[0]) * radius_;
    const double dl = (turn_[1] - prev_turn_[1]) * radius_;
    prev_turn_[0] = turn_[0];
    prev_turn_[1] = turn_[1];

    const double ds = (dr + dl) / 2.0;
    const double dtheta = (dr - dl) / track_;
    pose_.x += ds * std::cos(pose_.yaw + dtheta / 2.0);
    pose_.y += ds * std::sin(pose_.yaw + dtheta / 2.0);
    pose_.yaw += dtheta;
    pose_.v = (vr + vl) / 2.0;
    pose_.omega = (vr - vl) / track_;
    pose_.v_right = vr;
    pose_.v_left = vl;
    pose_.t = now;
}

void WheelOdomReader::Stop()
{
    stop_ = true;
    if (thread_.joinable())
        thread_.join();
    if (csv_.is_open())
    {
        csv_.flush();
        csv_.close();
    }
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

WheelOdomSample WheelOdomReader::Latest() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return pose_;
}

bool WheelOdomReader::Alive(double timeout_s) const
{
    const WheelOdomSample s = Latest();
    return s.t > 0.0 && (NowSeconds() - s.t) < timeout_s;
}

} // namespace dacheng
