// L5 rotor gyroscope (work order 52). Internal to the core: read-only observation of the
// spin inertia the L5 spin equation used, for the V5b2 single-source gate. Declared before
// the implementation commit; reading it never changes the dynamics.
#pragma once

#include "vdsim/interfaces.hpp"

#include <array>

namespace vdsim::detail {

/**
 * @brief Reflected engine / rotor spin inertia per wheel from the last L5 derivative
 *        evaluation (after the ground-contact gate; 0 for an airborne wheel).
 *
 * @param dyn L5 dynamics (create_stunt_dof).
 * @return Per-wheel reflected inertia [kg m^2] in FL, FR, RL, RR order.
 * @throws std::invalid_argument when @p dyn is not the L5 model.
 */
std::array<double, NUM_WHEELS> free_3d_wheel_engine_inertia_obs(const IVehicleDynamics& dyn);

/**
 * @brief Wheel spin inertia per wheel the L5 spin equation uses (wheel_inertia, or
 *        0.5 m_u R^2 when unset), without the reflected part.
 *
 * @param dyn L5 dynamics (create_stunt_dof).
 * @return Per-wheel wheel inertia [kg m^2] in FL, FR, RL, RR order.
 * @throws std::invalid_argument when @p dyn is not the L5 model.
 */
std::array<double, NUM_WHEELS> free_3d_wheel_inertia_obs(const IVehicleDynamics& dyn);

}  // namespace vdsim::detail
