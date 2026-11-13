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

}  // namespace vdsim::detail
