#pragma once

#include <array>
#include <variant>
#include <vector>

#include "vdsim/types.hpp"

namespace vdsim {

// Control ladder level identifier (longitudinal and lateral share the same numbering).
// Lower = more physical (closer to actuator), higher = more abstract (closer to planner).
//
//  Lon:  L1=per-wheel torque  L2=axle torque  L3=Fx force  L4=pedal  L5=ax  L6=vx
//  Lat:  L1=steer torque  L2=ang accel  L3=ang vel  L4=angle  L5=ay  L6=r  L7=kappa  L8=path
enum class LcLevel : int {
    L1 = 1, L2 = 2, L3 = 3, L4 = 4,
    L5 = 5, L6 = 6, L7 = 7, L8 = 8
};

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

// Steering actuator command interpretation. Default = Angle (kinematic path).
// Sub-L4 lateral levels (torque/rate/accel) require a Dynamic steering subsystem
// that integrates them via the Rack EOM.
enum class SteerMode { Angle, Torque, AngVel, AngAccel };

// L4: pedal-level (CARLA-compatible)
struct CmdL4 {
    double throttle {0.0};            // [0, 1]
    double brake    {0.0};            // [0, 1]
    double steer_angle_wheel {0.0};   // [rad] at wheel  (used when steer_mode == Angle)
    int    gear     {1};              // +1 fwd, -1 rev, 0 N
    bool   handbrake {false};
    // Sub-L4 steering actuator command (Dynamic steering). When steer_mode != Angle,
    // steer_actuator holds: Torque[Nm] / AngVel[rad/s] / AngAccel[rad/s²] at the wheel.
    SteerMode steer_mode    {SteerMode::Angle};
    double    steer_actuator {0.0};
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

// NOTE: ControlInput is defined at the end of this header, after CmdSplit,
// because std::variant requires complete types.

// ─── Lateral control commands (LcLat) ─────────────────────────────────────
// These carry the lateral component independently from the longitudinal.
// Subsystems can receive any LcLat level and cascade down to their native level.
//
//  Lat L1: steer torque [Nm]    — direct motor/column torque
//  Lat L2: ang accel [rad/s²]   — desired steering angular acceleration
//  Lat L3: ang vel [rad/s]      — desired steering angular velocity
//  Lat L4: steer angle [rad]    — wheel angle reference (native for most subsystems)
//  Lat L5: ay_target [m/s²]     — desired lateral acceleration
//  Lat L6: r_target [rad/s]     — desired yaw rate
//  Lat L7: kappa [1/m]          — desired path curvature
//  Lat L8: waypoints             — path reference (same as CmdL8.path)

struct LcLatL1 { double steer_torque   {0.0}; };                // [Nm]
struct LcLatL2 { double steer_ang_accel{0.0}; };                // [rad/s²]
struct LcLatL3 { double steer_ang_vel  {0.0}; };                // [rad/s]
struct LcLatL4 { double steer_angle    {0.0}; };                // [rad] at wheel
struct LcLatL5 { double ay_target      {0.0}; };                // [m/s²]
struct LcLatL6 { double r_target       {0.0}; };                // [rad/s] yaw rate
struct LcLatL7 { double kappa          {0.0}; };                // [1/m]
struct LcLatL8 { std::vector<CmdL8::PathPoint> path; double lookahead {5.0}; };

using LcLatCmd = std::variant<LcLatL1, LcLatL2, LcLatL3, LcLatL4,
                               LcLatL5, LcLatL6, LcLatL7, LcLatL8>;

// ─── Longitudinal control commands (LcLon) ────────────────────────────────
// Explicit typed commands for the longitudinal axis.
//
//  Lon L1: per-wheel drive/brake torque [Nm]
//  Lon L2: axle drive torque [Nm]
//  Lon L3: total Fx force [N]
//  Lon L4: throttle/brake pedal [0,1]
//  Lon L5: ax_target [m/s²]
//  Lon L6: vx_target [m/s]

struct LcLonL1 { std::array<double, 4> wheel_torque {{0,0,0,0}}; };  // [Nm] (+drive,-brake)
struct LcLonL2 { double axle_torque{0.0}; };                          // [Nm]
struct LcLonL3 { double Fx_total   {0.0}; };                          // [N]
struct LcLonL4 { double throttle{0.0}; double brake{0.0}; int gear{1}; };  // [0,1]
struct LcLonL5 { double ax_target  {0.0}; };                          // [m/s²]
struct LcLonL6 { double vx_target  {0.0}; };                          // [m/s]

using LcLonCmd = std::variant<LcLonL1, LcLonL2, LcLonL3, LcLonL4, LcLonL5, LcLonL6>;

// Helper: extract LcLevel from a LcLatCmd or LcLonCmd.
inline LcLevel lc_lat_level(const LcLatCmd& cmd) {
    return static_cast<LcLevel>(cmd.index() + 1);   // variant index 0=L1 ... 7=L8
}
inline LcLevel lc_lon_level(const LcLonCmd& cmd) {
    return static_cast<LcLevel>(cmd.index() + 1);
}

// ─── Split command: independent longitudinal + lateral levels ──────────────
// Lets the two axes be commanded at *different* abstraction levels, e.g.
// vx-target (lon L6) + yaw-rate-target (lat L6), or pedal (lon L4) + curvature
// (lat L7). The CascadeController converts each axis independently to CmdL4.
struct CmdSplit {
    LcLonCmd lon {LcLonL4{}};
    LcLatCmd lat {LcLatL4{}};
};

// Unified control command: any single-axis ladder level (CmdL1..CmdL8) OR a
// CmdSplit carrying independent lon/lat levels. Defined here (after CmdSplit)
// because std::variant requires complete alternative types.
using ControlInput = std::variant<CmdL1, CmdL2, CmdL3, CmdL4, CmdL5, CmdL6, CmdL7,
                                  CmdL8, CmdSplit>;

}  // namespace vdsim
