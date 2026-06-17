#pragma once

#include <array>
#include <cmath>

#include "vdsim/coordinate.hpp"
#include "vdsim/types.hpp"

namespace vdsim {

// State covers all ladder levels (L1/L2 leave L3 fields zero).
struct State {
    // Pose (world frame ENU)
    Vec3 position    {Vec3::Zero()};         // [m]
    Quat orientation {Quat::Identity()};     // body -> world

    // Velocity (body frame, ISO 8855)
    Vec3 velocity         {Vec3::Zero()};    // [m/s]   vx, vy, vz
    Vec3 angular_velocity {Vec3::Zero()};    // [rad/s] roll_rate, pitch_rate, yaw_rate

    // Wheel spin (L2+)
    std::array<double, NUM_WHEELS> wheel_spin {{0.0, 0.0, 0.0, 0.0}};   // [rad/s]

    // Suspension (L3+)
    std::array<double, NUM_WHEELS> susp_compression {{0.0, 0.0, 0.0, 0.0}};  // [m] +: compressed
    std::array<double, NUM_WHEELS> susp_velocity    {{0.0, 0.0, 0.0, 0.0}};  // [m/s]

    // L5 free-3D unsprung point mass (Stunt strut path only; zero/unused elsewhere).
    // Each wheel centre is a genuine inertial particle in world coordinates, connected
    // to the body by an anisotropic two-point bushing (soft along the strut axis = the
    // suspension spring, stiff perpendicular = the rigid links). susp_compression above
    // is the DERIVED strut-axis travel (contract preserved for the DAE / FMI / cosim).
    std::array<Vec3, NUM_WHEELS> unsprung_pos {{Vec3::Zero(), Vec3::Zero(),
                                                Vec3::Zero(), Vec3::Zero()}};  // [m] world
    std::array<Vec3, NUM_WHEELS> unsprung_vel {{Vec3::Zero(), Vec3::Zero(),
                                                Vec3::Zero(), Vec3::Zero()}};  // [m/s] world

    // Steering rack (Dynamic steering mode, Ld3+).
    // rack_travel = rack displacement [m] from neutral; rack_velocity = dxr/dt [m/s].
    // Both are zero when ISteeringSystem uses Kinematic mode (rack position is a constraint).
    double rack_travel   {0.0};   // [m]
    double rack_velocity {0.0};   // [m/s]

    // ---- Convenience accessors ----
    double yaw() const        { return yaw_from_quat(orientation); }
    double yaw_rate() const   { return angular_velocity.z(); }
    double roll_rate() const  { return angular_velocity.x(); }
    double pitch_rate() const { return angular_velocity.y(); }
    double vx() const         { return velocity.x(); }
    double vy() const         { return velocity.y(); }
    double vz() const         { return velocity.z(); }
    double speed_xy() const   { return velocity.head<2>().norm(); }

    // Side-slip angle [rad], referenced to body X axis
    double beta() const {
        const double vfwd = velocity.x();
        if (std::abs(vfwd) < 1e-3) return 0.0;
        return std::atan2(velocity.y(), vfwd);
    }
};

}  // namespace vdsim
