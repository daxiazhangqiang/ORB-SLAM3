#ifndef ORB_MAP_QUALITY_H
#define ORB_MAP_QUALITY_H
#include <vector>
#include <utility>
namespace ORB_SLAM3 {
// 只读质量快照，所有数量均限定在最大主图，不影响原生建图决策
struct MapQualitySample {
    int state = -1, map_id = -1, active_map_id = -1, revision = 0;
    int keyframes = 0, points = 0, tested_points = 0, stable_points = 0;
    int inliers = 0, stable_matches = 0, grid_cells = 0;
    // 未融合子图的点数用于判断主图是否已覆盖主要区域
    std::vector<std::pair<unsigned long, int>> other_submaps;
    std::vector<unsigned long> route_anchors, visible_anchors;
    // 逐个判断其余子图，20%边界允许通过，不使用点数之和
    bool SubmapsWithinLimit() const {
        if (points <= 0) return false;
        for (const auto& submap : other_submaps)
            if (static_cast<long long>(submap.second) * 5 > points) return false;
        return true;
    }
};
}
#endif
