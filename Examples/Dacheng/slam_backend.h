#ifndef DACHENG_SLAM_BACKEND_H
#define DACHENG_SLAM_BACKEND_H

// ORB-SLAM3 门面：唯一持有 ORB_SLAM3::System 的地方
// 上层（监控/提示/会话）只通过这里访问 SLAM，不直接碰 Atlas/Tracking
// 双目扩展点：Create() 里按 sensor 字段选 MONOCULAR/STEREO，Track() 分派 TrackMonocular/TrackStereo

#include "realtime_types.h"
#include "MapQuality.h"

#include <memory>
#include <string>

namespace ORB_SLAM3
{
class System;
}

namespace dacheng
{

enum class SensorMode
{
    Mono = 0,
    Stereo = 1,   // 预留：本轮只实现 Mono
};

// 单帧跟踪质量统计（用于丢失点归因）
struct FrameTrackStats
{
    int extracted = -1;   // 本帧提取到的特征点数（-1 = 尚未拿到）
    int matched = -1;     // 关联到本帧的地图点数（已剔除外点/无观测点）
};

class SlamBackend
{
public:
    // vocab/settings 必须已解析成绝对路径；sensor 决定单目/双目
    SlamBackend(const std::string &vocab, const std::string &settings, SensorMode sensor,
                bool viewer);
    ~SlamBackend();

    // 送入一帧（双目时 FramePacket.right 必须非空）
    void Track(const FramePacket &packet);
    int TrackingState();
    int MapCount();
    int KeyFrameCount();
    double LastKeyFrameTimestamp();
    // 取一帧的跟踪质量：一次调用同时拿到“提取点数 / 关联地图点数”
    FrameTrackStats TrackStats();
    // 读取主图稳定点和当前帧几何内点统计
    ORB_SLAM3::MapQualitySample Quality(bool inventory);

    // 只读导出（不停止后台线程）：用于退出前的尺度校验
    void ExportTrajectory(const std::string &path);
    void ExportKeyFrameTrajectory(const std::string &path);
    void ExportMapPointsPLY(const std::string &path);
    void ExportMapPointsPLYPerMap(const std::string &prefix);

    // 停止后台建图/回环线程并有界等待；可重复调用
    void RequestMappingStop();

    // 触发 Atlas 保存（settings 里 SaveAtlasToFile 决定落盘路径）
    void SaveAndShutdown();
    // 低频输出各子图计数，并标记当前激活子图和跟踪状态
    void PrintSubmapSummary(int active_id = -1, int state = -1);
    // 停机后的独立子图导出与尺度应用
    void PrepareSubmapExport();
    void SaveSubmaps(bool save_atlas, bool use_scale);


private:
    std::unique_ptr<ORB_SLAM3::System> slam_;
    SensorMode sensor_;
};

} // namespace dacheng

#endif // DACHENG_SLAM_BACKEND_H
