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

/** Internal per-wheel contact load; public tire-force diagnostics stay tangential-only. */
struct ContactLoad {
    Vec3 full_force_body {Vec3::Zero()};
    Vec3 applied_force_body {Vec3::Zero()};
    Vec3 full_force_world {Vec3::Zero()};
    Vec3 applied_force_world {Vec3::Zero()};
    Vec3 moment_delta_body {Vec3::Zero()};
};

/** Inject one-step L4 spatial context into the shared SevenDOF core. */
bool set_l4_contact_kinematics(IVehicleDynamics& dynamics,
                               const L4ContactKinematics& context) noexcept;

/** Return full loads and non-flat increments without extending the public plant API. */
std::array<ContactLoad, NUM_WHEELS> contact_loads(
    const IVehicleDynamics& dynamics) noexcept;

}  // namespace detail
}  // namespace vdsim
