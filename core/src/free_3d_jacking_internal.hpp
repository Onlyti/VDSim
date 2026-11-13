// Work order 49a: contact-patch jacking slope of the L5 strut + corner-model (DAE) path.
//
// The strut bushing constrains the wheel particle to the hardpoint travel path, and the
// path tangent's lateral slope turns the tyre lateral force into a vertical (jacking)
// reaction. The lateral force acts at the contact patch, so the slope that sets the
// jacking is that of the contact patch, dy_cp/dz_v, not that of the wheel centre.
// Internal to core/src; tests reach it through the unit-test include path.
#pragma once

#include "vdsim/multibody.hpp"

#include <cmath>

namespace vdsim::detail {

/// Central-difference step [m]; the same step as IHardJointDaeModel::travel_maps.
inline constexpr double kContactSlopeStep = 1e-3;

/**
 * @brief Lateral slope of the tyre contact patch along the corner travel path, dy_cp/dz_v.
 *
 * Contact patch y in the corner (left) frame: y_cp = y_wc - R sin(gamma), gamma = corner
 * model camber (+ = wheel top toward +y), so to first order dy_cp/dz_v = dy_wc/dz_v -
 * R dgamma/dz_v. Both poses come from pose_at_travel at z_v +- kContactSlopeStep with the
 * same rack input and seed as travel_maps, so the wheel-centre part equals travel_maps'
 * w_dq.y bit for bit.
 *
 * @param dae           corner model
 * @param travel_z      vertical wheel travel z_v [m]
 * @param steer_rack_dy rack input in the corner frame [m]
 * @param seed          knuckle warm start (corner state knuckle_aa)
 * @param wheel_radius  tyre radius R [m] (vehicle wheel_radius_nominal)
 * @return dy_cp/dz_v [-] in the corner (left) frame; a right corner mirrors y (negate)
 */
inline double contact_patch_lateral_slope(const mb::IHardJointDaeModel& dae, double travel_z,
                                          double steer_rack_dy, const Vec3& seed,
                                          double wheel_radius) {
    const double h = kContactSlopeStep;
    const mb::WheelPose pp = dae.pose_at_travel(travel_z + h, steer_rack_dy, seed);
    const mb::WheelPose pm = dae.pose_at_travel(travel_z - h, steer_rack_dy, seed);
    const double yp = pp.position_world.y() - wheel_radius * std::sin(pp.camber_rad);
    const double ym = pm.position_world.y() - wheel_radius * std::sin(pm.camber_rad);
    return (yp - ym) / (2.0 * h);
}

}  // namespace vdsim::detail
