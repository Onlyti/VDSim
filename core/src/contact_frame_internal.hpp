#pragma once

#include "vdsim/types.hpp"

#include <array>

namespace vdsim {

class IVehicleDynamics;

namespace detail {

/** Internal L4 spatial contact kinematics; not part of the public plant API. */
struct L4ContactKinematics {
    bool enabled {false};
    Quat body_to_world {Quat::Identity()};
    Vec3 angular_velocity_body {Vec3::Zero()};
    std::array<double, NUM_WHEELS> wheel_vertical_velocity_world {{0.0, 0.0, 0.0, 0.0}};
};

/** Inject one-step L4 spatial context into the shared SevenDOF core. */
bool set_l4_contact_kinematics(IVehicleDynamics& dynamics,
                               const L4ContactKinematics& context) noexcept;

/** Return per-wheel contact-wrench moment increments not already handled by flat suspension. */
std::array<Vec3, NUM_WHEELS> contact_moment_delta(
    const IVehicleDynamics& dynamics) noexcept;

}  // namespace detail
}  // namespace vdsim
