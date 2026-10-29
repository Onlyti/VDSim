// L5 drive / brake torque reaction helpers (work order 43). Internal to the core:
// free_3d_dynamics.cpp uses them, the unit tests check them directly.
#pragma once

#include "vdsim/multibody.hpp"
#include "vdsim/types.hpp"

#include <optional>

namespace vdsim::detail {

/**
 * @brief Knuckle pitch rate dtheta/dz_v [rad/m] about the body +y axis.
 *
 * Central difference of the knuckle rotation the corner model reports in
 * WheelPose::orientation at z_v +/- dz. The pitch (y) component is the same on
 * the mirrored right side, so one value serves both corners of an axle.
 *
 * @param dae           Corner model.
 * @param travel_z      Vertical wheel travel z_v [m].
 * @param steer_rack_dy Rack input passed to pose_at_travel.
 * @param seed          Warm start of the inner knuckle solve.
 * @param dz            Half step [m].
 * @return The rate, or nullopt when the model reports no knuckle orientation
 *         (identity at both points).
 */
std::optional<double> knuckle_pitch_rate(const mb::IHardJointDaeModel& dae,
                                         double travel_z, double steer_rack_dy,
                                         const Vec3& seed, double dz = 1e-3);

/// Equivalent forces of a knuckle pitch moment on a particle-plus-chassis corner.
struct KnuckleReactionSplit {
    Vec3 force_on_wheel {Vec3::Zero()};   ///< on the wheel particle [N] (body axes)
    Vec3 force_on_body  {Vec3::Zero()};   ///< on the chassis at the mount [N] (body axes)
    Vec3 couple_on_body {Vec3::Zero()};   ///< free couple on the chassis [N m] (body axes)
};

/**
 * @brief Split a moment on the knuckle about body +y into particle / chassis loads.
 *
 * The wheel particle gets F = Q t / |t|^2 with Q = M_pitch * theta_p, so its virtual
 * work along the travel path equals the knuckle's (F.t = Q). The chassis gets -F at
 * the mount plus the couple M_pitch*y - (r_wheel - r_mount) x F, so the whole corner
 * carries exactly M_pitch*y and no net force.
 *
 * @param M_pitch  Moment on the knuckle about body +y [N m].
 * @param theta_p  Knuckle pitch rate dtheta/dz_v [rad/m].
 * @param t_body   Wheel-centre path tangent dw/dz_v (body axes).
 * @param r_wheel  Wheel particle position relative to the CG (body axes) [m].
 * @param r_mount  Strut mount position relative to the CG (body axes) [m].
 * @return The three loads (all zero for a degenerate tangent).
 */
KnuckleReactionSplit split_knuckle_reaction(double M_pitch, double theta_p,
                                            const Vec3& t_body, const Vec3& r_wheel,
                                            const Vec3& r_mount);

}  // namespace vdsim::detail
