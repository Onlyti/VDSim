// Ride states for the aero ride slopes of the L2 model hosted inside L3 / L4
// (work order 51). Internal to the core: the L3 ride model hands its heave / pitch /
// roll to the inner planar model each step so the inner aero (drag, quasi-static
// load of every RK stage, reported tire_Fz) sees the same ride slopes as the L3
// tire-load path. A stand-alone L2 never calls it (ride states 0).
#pragma once

#include "vdsim/interfaces.hpp"

namespace vdsim::detail {

/**
 * @brief Set the ride states the L2 aero reads for the next step.
 * @param dyn inner L2 model (no-op on any other model)
 * @param dz heave relative to the static reference [m], + up
 * @param pitch pitch relative to the static reference [rad], ISO + nose down
 * @param roll roll relative to the static reference [rad]
 */
void seven_dof_set_aero_ride(IVehicleDynamics& dyn, double dz, double pitch, double roll);

}  // namespace vdsim::detail
