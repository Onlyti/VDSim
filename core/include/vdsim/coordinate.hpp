#pragma once

#include "vdsim/types.hpp"

namespace vdsim {

// Canonical convention:
//   World: ENU RH (X east, Y north, Z up), meters
//   Body : ISO 8855 RH (X forward, Y left, Z up), meters
//   Quat : body -> world (Eigen default)
//   Euler: ZYX intrinsic (yaw -> pitch -> roll)
struct Euler {
    double roll  {0.0};
    double pitch {0.0};
    double yaw   {0.0};
};

Quat   quat_from_euler(const Euler&);
Euler  euler_from_quat(const Quat&);
double yaw_from_quat(const Quat&);

namespace ue {
// Unreal Engine 5:
//   Z up, X forward, Y right (LH), centimeters.
// Conversion to VDSim:
//   - scale 1/100
//   - Y flip
//   - yaw flips sign (LH <-> RH)
Vec3 from_ue_position(const Vec3& ue_cm);
Vec3 to_ue_position(const Vec3& vd_m);

Quat from_ue_rotation(const Quat& ue_q);
Quat to_ue_rotation(const Quat& vd_q);

Vec3 from_ue_velocity(const Vec3& ue_cm_s);
Vec3 to_ue_velocity(const Vec3& vd_m_s);
}  // namespace ue

}  // namespace vdsim
