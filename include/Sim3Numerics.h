#ifndef ORB_SLAM3_SIM3_NUMERICS_H
#define ORB_SLAM3_SIM3_NUMERICS_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace ORB_SLAM3 {
// 直接归一化四元数得到旋转矩阵，避免零旋转转换为轴角时除以零
inline bool Sim3QuaternionRotation(const Eigen::Vector4f& coefficients,
                                   Eigen::Matrix3f& rotation) {
    if (!coefficients.allFinite()) return false;
    const float norm = coefficients.norm();
    if (!std::isfinite(norm) || norm <= 0.0f) return false;
    const Eigen::Vector4f q = coefficients / norm;
    rotation = Eigen::Quaternionf(q[0], q[1], q[2], q[3]).toRotationMatrix();
    return rotation.allFinite();
}
}
#endif
