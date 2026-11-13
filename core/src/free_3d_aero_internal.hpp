// L5 aero static reference (work order 51). Internal to the core: SimSession runs
// the in-reset settle run and hands the reference to the free-3D model through
// these hooks. No-ops on any other level.
#pragma once

#include "vdsim/interfaces.hpp"

namespace vdsim::detail {

/**
 * @brief True when the L5 model needs a static aero reference: some element has
 *        a non-zero ride slope, or an aero module is attached.
 * @param dyn dynamics (false unless L5 free-3D)
 * @return whether reset() must run the settle run
 */
bool free_3d_aero_needs_settle(const IVehicleDynamics& dyn);

/**
 * @brief Mark the settle run: aero contributes exactly 0, the aero module is not
 *        called (no compute / begin_step) and reset() does not reset it.
 * @param dyn dynamics (no-op unless L5 free-3D)
 * @param on true while the settle run is in progress
 */
void free_3d_set_aero_settling(IVehicleDynamics& dyn, bool on);

/**
 * @brief Set the static reference the ride slopes and the module read.
 * @param dyn dynamics (no-op unless L5 free-3D)
 * @param z body-origin world z [m]
 * @param pitch body pitch [rad] (euler_from_quat, ISO + nose down)
 * @param roll body roll [rad]
 */
void free_3d_set_aero_reference(IVehicleDynamics& dyn, double z, double pitch, double roll);

/**
 * @brief Read-only observation of the L5 aero evaluation (work order 51 B3b-L5i).
 *
 * Holds the inputs and axle loads of the first aero evaluation after
 * free_3d_aero_obs_arm(). Diagnostic only: nothing in the model reads it.
 */
struct AeroObs {
    long   count {0};       ///< aero evaluations since arm
    double fz_front {0.0};  ///< AeroLoads::fz_front of the first evaluation [N]
    double fz_rear {0.0};   ///< AeroLoads::fz_rear of the first evaluation [N]
    double dz {0.0};        ///< static-relative heave seen by the slopes [m]
    double pitch {0.0};     ///< static-relative pitch seen by the slopes [rad]
    double vx {0.0};        ///< body-frame forward speed [m/s]
};

/**
 * @brief Clear the observation; the next aero evaluation is recorded.
 * @param dyn dynamics (no-op unless L5 free-3D)
 */
void free_3d_aero_obs_arm(IVehicleDynamics& dyn);

/**
 * @brief Observation since the last arm.
 * @param dyn dynamics
 * @return the observation (count 0 unless L5 free-3D evaluated aero)
 */
AeroObs free_3d_aero_obs(const IVehicleDynamics& dyn);

}  // namespace vdsim::detail
