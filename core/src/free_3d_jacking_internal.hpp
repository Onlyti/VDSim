// Work order 49a: contact-patch jacking slope of the L5 strut + corner-model (DAE) path.
//
// The strut bushing constrains the wheel particle to the hardpoint travel path, and the
// path tangent's lateral slope turns the tyre lateral force into a vertical (jacking)
// reaction. The lateral force acts at the contact patch, so the slope that sets the
// jacking is that of the contact patch, dy_cp/dz_v, not that of the wheel centre.
// Internal to core/src; tests reach it through the unit/integration test include paths.
#pragma once

#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"

#include <cmath>
#include <functional>

namespace vdsim::detail {

/**
 * @brief One strut-path connection sample (work order 49a gate P10b, diagnostic only).
 *
 * Taken once per corner per derivative evaluation on the L5 strut path. Forces are world
 * frame [N]. Recording never changes the dynamics.
 */
struct JackingPairSample {
    int corner {0};                  ///< 0 FL, 1 FR, 2 RL, 3 RR
    bool path_valid {false};         ///< corner-model travel path active (jacking present)
    Vec3 f_on_body {Vec3::Zero()};   ///< connection force added to the body translation
    Vec3 f_on_body_moment {Vec3::Zero()};  ///< R * F_body[i], the force whose moment the body takes
    Vec3 f_on_wheel {Vec3::Zero()};  ///< connection force on the wheel particle,
                                     ///< m_u a_u - tyre force - m_u g (from the applied a_u)
    double jack_radius {std::nan("")};     ///< radius the contact-patch jacking slope used
                                           ///< (NaN = not used on this path)
    double contact_radius {std::nan("")};  ///< radius the contact query reads
                                           ///< (VehicleParams::wheel_radius_nominal)
    // Work order 49a gate P10c (jacking magnitude). Body frame [N] unless noted.
    Vec3 tyre_force_body {Vec3::Zero()};       ///< tyre force on the wheel particle
    Vec3 unsprung_inertia_body {Vec3::Zero()}; ///< m_u a_u from the applied a_u
    Vec3 unsprung_weight_body {Vec3::Zero()};  ///< m_u g, g = (0, 0, -9.80665) world
    Vec3 conn_on_body_body {Vec3::Zero()};     ///< connection force on the body (F_body[i])
    double strut_force {0.0};                  ///< coilover + damper + stop force along the
                                               ///< strut axis (body up), F_susp
    Vec3 path_tangent_body {Vec3::UnitZ()};    ///< travel path tangent the bushing used
                                               ///< (body frame, unnormalised)
    double cp_slope_body {std::nan("")};       ///< contact_patch_lateral_slope at this outer
                                               ///< step, body sign (right corner mirrored);
                                               ///< NaN = not evaluated
};

/// Diagnostic sink for JackingPairSample; empty = recording off.
using JackingPairProbe = std::function<void(const JackingPairSample&)>;

/**
 * @brief Install (or clear with an empty function) the P10b connection-pair probe.
 * @param dyn   vehicle dynamics; only the L5 free-3D model records
 * @param probe sink called per corner per derivative evaluation on the strut path
 * @return true if dyn is the L5 free-3D model
 */
bool free_3d_set_jacking_probe(IVehicleDynamics& dyn, JackingPairProbe probe);

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
