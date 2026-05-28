#pragma once

#include <array>
#include <variant>
#include <vector>

#include "vdsim/types.hpp"

namespace vdsim {

// Steer convention across all levels: wheel angle [rad] (single value, average for steered axle).
// CARLA's normalized [-1,1] is converted at the plugin boundary using steering_ratio.

// L1: per-wheel motor + per-wheel brake + wheel steer
struct CmdL1 {
    std::array<double, NUM_WHEELS> motor_torque {{0.0, 0.0, 0.0, 0.0}};  // [Nm]
    std::array<double, NUM_WHEELS> brake_torque {{0.0, 0.0, 0.0, 0.0}};  // [Nm] (>=0)
    double steer_angle_wheel {0.0};                                       // [rad]
};

// L2: aggregated drive/brake torque + steer
struct CmdL2 {
    double drive_torque {0.0};        // [Nm] (distributed per Drive type)
    double brake_torque {0.0};        // [Nm]
    double steer_angle_wheel {0.0};   // [rad]
};

// L3: longitudinal force at CG + steer
struct CmdL3 {
    double Fx_total {0.0};            // [N], + accel / - brake
    double steer_angle_wheel {0.0};   // [rad]
};

// L4: pedal-level (CARLA-compatible)
struct CmdL4 {
    double throttle {0.0};            // [0, 1]
    double brake    {0.0};            // [0, 1]
    double steer_angle_wheel {0.0};   // [rad] at wheel
    int    gear     {1};              // +1 fwd, -1 rev, 0 N
    bool   handbrake {false};
};

// L5: longitudinal acceleration target
struct CmdL5 {
    double ax_target {0.0};           // [m/s^2]
    double steer_angle_wheel {0.0};   // [rad]
};

// L6: velocity target
struct CmdL6 {
    double v_target {0.0};            // [m/s]
    double steer_angle_wheel {0.0};   // [rad]
};

// L7: velocity + curvature
struct CmdL7 {
    double v_target {0.0};            // [m/s]
    double kappa    {0.0};            // [1/m]
};

// L8: path tracking
struct CmdL8 {
    struct PathPoint {
        double s {0.0};               // arc length [m]
        Vec2   xy {Vec2::Zero()};     // world
        double yaw {0.0};
        double kappa {0.0};
        double v_des {0.0};
    };
    std::vector<PathPoint> path;
    double lookahead_distance {5.0};  // [m]
};

using ControlInput = std::variant<CmdL1, CmdL2, CmdL3, CmdL4, CmdL5, CmdL6, CmdL7, CmdL8>;

}  // namespace vdsim
