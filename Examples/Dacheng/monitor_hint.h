#ifndef DACHENG_MONITOR_HINT_H
#define DACHENG_MONITOR_HINT_H

// 观测层：状态迁移 / 子图数变化 / 融合标记 / 每5秒摘要（AtlasMonitor）
// 提示层：什么时候提示、提示什么（HintPolicy）
// 只用公开 API（GetTrackingState / GetAtlasMapCount）与日志标记，不接触算法内部

#include "realtime_types.h"
#include "session_io.h"

#include <string>

namespace dacheng
{

class AtlasMonitor
{
public:
    AtlasMonitor(SessionWriter *writer, const LogTee *tee);

    // 每帧调用一次：打印状态迁移、5 秒摘要，并处理子图数变化
    void OnFrame(int state, int map_count, const SensorStats &sensor, double frame_cost_ms,
                 double t, int extracted = -1, int matched = -1);
    // 会话开始时的基线（避免把初始状态当成一次“迁移”）
    void Initialize(int state, int map_count, double t);
    // 会话结束时收口：残留子图与提示
    void OnFinish(double t, int final_map_count);

    int max_map_count() const { return max_map_count_; }
    int new_map_events() const { return new_map_events_; }
    int merge_events() const { return merge_seen_; }
    int MapCountForLog() const { return last_map_count_; }
    // 一次性事件标志：由主循环取走后调用方负责存现场快照
    bool TakeNewMapPending()
    {
        const bool v = new_map_pending_;
        new_map_pending_ = false;
        return v;
    }
    bool TakeLostPending()
    {
        const bool v = lost_pending_;
        lost_pending_ = false;
        return v;
    }
    // 刚完成一次跨子图焊接（Merge finished）
    bool TakeMergePending()
    {
        const bool v = merge_pending_;
        merge_pending_ = false;
        return v;
    }

private:
    SessionWriter *writer_;
    const LogTee *tee_;
    bool inited_ = false;
    int last_state_ = -999;
    int last_map_count_ = -1;
    int max_map_count_ = 0;
    int new_map_events_ = 0;
    int merge_seen_ = 0;
    int merge_finished_seen_ = 0;
    double last_summary_t_ = 0.0;
    double map_split_since_ = 0.0;     // 子图数 >=2 的起始时刻
    double frame_cost_sum_ = 0.0;
    int frame_cost_n_ = 0;
    int slow_seconds_ = 0;             // 单帧耗时连续超预算的秒数
    bool hinted_slow_ = false;
    bool merge_announced_ = false;
    bool new_map_pending_ = false;     // 刚发生子图分裂，等主循环存快照
    bool lost_pending_ = false;        // 刚进入 LOST/RECENTLY_LOST
    bool merge_pending_ = false;       // 刚完成焊接
    // 分档提示只打一次
    bool hinted_60s_ = false;
    bool hinted_300s_ = false;
    bool log_event_written_ = false;
};

} // namespace dacheng

#endif // DACHENG_MONITOR_HINT_H
