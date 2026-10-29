#include "slam_backend.h"

#include "System.h"

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace dacheng
{

SlamBackend::SlamBackend(const std::string &vocab, const std::string &settings, SensorMode sensor,
                         bool viewer)
    : sensor_(sensor)
{
    ORB_SLAM3::System::eSensor mode = ORB_SLAM3::System::MONOCULAR;
    if (sensor_ == SensorMode::Stereo)
        mode = ORB_SLAM3::System::STEREO;
    slam_ = std::make_unique<ORB_SLAM3::System>(vocab, settings, mode, viewer);
}

SlamBackend::~SlamBackend() = default;

void SlamBackend::Track(const FramePacket &packet)
{
    if (sensor_ == SensorMode::Mono)
    {
        slam_->TrackMonocular(packet.left, packet.timestamp);
        return;
    }
    if (packet.right.empty())
        throw std::runtime_error("双目模式收到空右目图像");
    slam_->TrackStereo(packet.left, packet.right, packet.timestamp);
}

int SlamBackend::TrackingState()
{
    return slam_->GetTrackingState();
}

int SlamBackend::MapCount()
{
    return slam_->GetAtlasMapCount();
}

int SlamBackend::KeyFrameCount()
{
    return slam_->GetKeyFrameCount();
}

double SlamBackend::LastKeyFrameTimestamp()
{
    return slam_->GetLastKeyFrameTimestamp();
}

FrameTrackStats SlamBackend::TrackStats()
{
    // 与离线工具同口径：向量长度=提取点数，非空元素个数=关联地图点数
    const std::vector<ORB_SLAM3::MapPoint *> points = slam_->GetTrackedMapPoints();
    FrameTrackStats stats;
    stats.extracted = static_cast<int>(points.size());
    stats.matched = 0;
    for (const ORB_SLAM3::MapPoint *point : points)
        if (point)
            stats.matched++;
    return stats;
}

// 低频只读质量评估不修改地图
ORB_SLAM3::MapQualitySample SlamBackend::Quality(bool inventory)
{
    return slam_->GetMapQuality(inventory);
}

void SlamBackend::ExportTrajectory(const std::string &path)
{
    slam_->SaveTrajectoryEuRoC(path);
}

void SlamBackend::ExportKeyFrameTrajectory(const std::string &path)
{
    slam_->SaveKeyFrameTrajectoryEuRoC(path);
}

void SlamBackend::ExportMapPointsPLY(const std::string &path)
{
    slam_->SaveMapPointsPLY(path);
}

void SlamBackend::ExportMapPointsPLYPerMap(const std::string &prefix)
{
    slam_->SaveMapPointsPLYPerMap(prefix);
}

void SlamBackend::RequestMappingStop()
{
    slam_->RequestMappingStop();
}

// 计数只作参考，是否可结束交给质量评估与回访验证
void SlamBackend::PrintSubmapSummary(int active_id, int state)
{
    const auto stats = slam_->GetSubmapStats();
    size_t total = 0;
    for(const auto& st : stats) total += st.keyframes;
    std::cout << "[submaps]";
    if(active_id >= 0)
        std::cout << " active=map_" << active_id << " state=" << state;
    for(const auto& st : stats)
        std::cout << " map_" << st.id << " KF=" << st.keyframes << " MP=" << st.points;
    std::cout << std::endl;
    std::cout << "[hint] active=当前激活子图；点数和主图占比仅供参考，以质量提示及冻结地图回访验证为准" << std::endl;
}

// 后台停止后才生成定尺度输入
void SlamBackend::PrepareSubmapExport()
{
    slam_->PrepareSubmapExport("map");
}

// 仅消费本次尺度脚本成功生成的主图结果
void SlamBackend::SaveSubmaps(bool save_atlas, bool use_scale)
{
    unsigned long id = static_cast<unsigned long>(-1);
    double scale = 1.0;
    if(use_scale)
    {
        std::ifstream f("primary_scale.txt");
        if(!(f >> id >> scale)) { id = static_cast<unsigned long>(-1); scale = 1.0; }
    }
    std::cout << "[scale] 主图保存倍率=" << scale
              << "（建图保存原始单位；最终尺度由回访双点卷尺确定）" << std::endl;
    slam_->SaveSubmaps("map", id, scale, save_atlas);
}

void SlamBackend::SaveAndShutdown()
{
    slam_->Shutdown();
}

} // namespace dacheng
