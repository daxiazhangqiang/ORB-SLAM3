#ifndef DACHENG_SCALE_CAPTURE_UI_H
#define DACHENG_SCALE_CAPTURE_UI_H

#include <algorithm>
#include <cmath>

namespace dacheng {

constexpr int kScalePointMinTrackedPoints = 20;
constexpr double kScalePointMaxOrientationDeg = 5.0;

// 尺度端点的实时定位状态
struct ScaleLiveStatus {
    int state = -1;
    int inliers = 0;
    int tracked = 0;
    bool pose_ok = false;
    bool orientation_ok = true;
    double orientation_deg = 0.0;

    bool tracking_ok() const {
        return state == 2 && pose_ok && inliers >= kScalePointMinTrackedPoints &&
            tracked >= kScalePointMinTrackedPoints;
    }

    bool ready() const {
        return tracking_ok() && orientation_ok;
    }
};

// 根据跟踪状态和第一点朝向判断当前位置能否确认
inline ScaleLiveStatus EvaluateScaleLiveStatus(
    int state, int inliers, int tracked, bool pose_ok,
    bool has_orientation_reference = false,
    double orientation_deg = 0.0,
    double max_orientation_deg = kScalePointMaxOrientationDeg) {
    ScaleLiveStatus result;
    result.state = state;
    result.inliers = std::max(0, inliers);
    result.tracked = std::max(0, tracked);
    result.pose_ok = pose_ok;
    result.orientation_deg = orientation_deg;
    result.orientation_ok = !has_orientation_reference ||
        (std::isfinite(orientation_deg) && orientation_deg <= max_orientation_deg);
    return result;
}

}  // namespace dacheng

#endif
