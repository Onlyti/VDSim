#include "vdsim/coordinate.hpp"

#include <cmath>

namespace vdsim {

// ZYX intrinsic Euler -> Quat:
//   q = Rz(yaw) * Ry(pitch) * Rx(roll)
Quat quat_from_euler(const Euler& e) {
    const Eigen::AngleAxisd rz(e.yaw,   Vec3::UnitZ());
    const Eigen::AngleAxisd ry(e.pitch, Vec3::UnitY());
    const Eigen::AngleAxisd rx(e.roll,  Vec3::UnitX());
    return Quat(rz * ry * rx);
}

// Quat -> ZYX intrinsic Euler.
// Implemented directly from quaternion components (avoids Eigen's
// eulerAngles range surprises for small-angle cases).
Euler euler_from_quat(const Quat& q) {
    Euler e;

    // Roll (x-axis rotation)
    const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    e.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1.0) {
        e.pitch = std::copysign(M_PI / 2.0, sinp);   // gimbal
    } else {
        e.pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    e.yaw = std::atan2(siny_cosp, cosy_cosp);

    return e;
}

double yaw_from_quat(const Quat& q) {
    const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(siny_cosp, cosy_cosp);
}

// =============================================================================
// UE5 <-> VDSim conversion
// UE5  : LH, Z up, X fwd, Y right, cm
// VDSim: RH, Z up, X fwd, Y left,  m
// Transform = "flip Y axis" + scale.
//   For quaternions, flipping Y axis maps (w,x,y,z) -> (w,-x,y,-z).
// =============================================================================
namespace ue {

Vec3 from_ue_position(const Vec3& ue_cm) {
    return Vec3(ue_cm.x() * 0.01, -ue_cm.y() * 0.01, ue_cm.z() * 0.01);
}

Vec3 to_ue_position(const Vec3& vd_m) {
    return Vec3(vd_m.x() * 100.0, -vd_m.y() * 100.0, vd_m.z() * 100.0);
}

Quat from_ue_rotation(const Quat& ue_q) {
    return Quat(ue_q.w(), -ue_q.x(), ue_q.y(), -ue_q.z());
}

Quat to_ue_rotation(const Quat& vd_q) {
    return Quat(vd_q.w(), -vd_q.x(), vd_q.y(), -vd_q.z());
}

Vec3 from_ue_velocity(const Vec3& ue_cm_s) {
    return from_ue_position(ue_cm_s);   // same linear transform
}

Vec3 to_ue_velocity(const Vec3& vd_m_s) {
    return to_ue_position(vd_m_s);
}

}  // namespace ue

}  // namespace vdsim
