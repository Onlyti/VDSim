// Actuator dynamics + sensor delay layer.
//
// Sits between the control ladder (lowered to CmdL4) and the plant: filters the
// commanded throttle/brake/steer into the *realized* command with dead-time,
// first-order lag, rate limit, saturation, steering friction (LuGre), and brake
// dead-zone + temperature fade.  SensorDelay models feedback-path latency.
//
// Plant-non-invasive: callers wrap dyn.step() —
//     CmdL4 realized = act.apply(desired, speed, dt);
//     dyn->step(realized, contacts, dt);
//     State meas = sensor.apply(dyn->state(), dt);   // fed to the controller
//
// All effects default OFF (zero delay/lag/rate/friction) -> identity, so this is
// backward-compatible.  See docs/references/actuator_nonlinearity.md.
#pragma once

#include <deque>
#include <vector>

#include "vdsim/control.hpp"
#include "vdsim/state.hpp"

namespace vdsim {

// Per-channel first-order-plus-dead-time + rate limit + saturation.
struct ChannelActuator {
    double dead_time_s {0.0};      // pure transport delay L [s] (0 = none)
    double tau_s       {0.0};      // first-order lag time constant [s] (0 = none)
    double rate_limit  {0.0};      // max |d(output)/dt| per s (<=0 = disabled)
    double dead_zone   {0.0};      // input dead-band [-]; cmd below it -> 0, then
                                   // rescaled (throttle pedal tip-in / brake pad
                                   // clearance). Applies to one-sided 0..1 channels.
    double out_min     {-1e12};    // saturation
    double out_max     { 1e12};
};

// LuGre dynamic friction (Stribeck + presliding hysteresis + stick-slip).
struct LuGreParams {
    bool   enabled {false};
    double sigma0 {1.0e4};         // bristle stiffness [N m/rad]
    double sigma1 {1.0e2};         // bristle damping
    double sigma2 {0.4};           // viscous coefficient
    double Tc     {1.0};           // Coulomb torque [N m]
    double Ts     {1.6};           // static (breakaway) torque [N m]
    double ws     {0.05};          // Stribeck velocity [rad/s]
};

// Steering actuator: dead-time + (servo+LuGre OR first-order lag) + rate + sat.
struct SteerActuator {
    ChannelActuator ch;            // dead_time, tau (lag mode), rate_limit, sat
    LuGreParams     friction;      // if enabled -> torque-servo model with LuGre
    double inertia  {0.02};        // steering inertia at road-wheel angle [kg m^2]
    double servo_kp {60.0};        // position servo gain (torque per rad)
    double servo_kd {6.0};         // servo damping (torque per rad/s)
};

// Brake actuator: channel (incl. dead-zone for pad clearance) + mu(T) fade.
struct BrakeActuator {
    ChannelActuator ch;            // ch.dead_zone models pad clearance fill
    // Temperature-dependent friction (brake fade). Disabled -> mu scale = 1.
    bool   thermal_enabled {false};
    double heat_coeff {0.0};       // dT += heat_coeff * brake_cmd * |speed| * dt
    double cool_coeff {0.0};       // dT += -cool_coeff * (T - T_ambient) * dt
    double T_ambient  {40.0};      // [degC]
    std::vector<double> mu_T_temp;   // breakpoints [degC] (ascending)
    std::vector<double> mu_T_scale;  // mu/mu_ref at each breakpoint (empty -> 1.0)
};

struct ActuatorParams {
    SteerActuator   steer;
    ChannelActuator throttle;
    BrakeActuator   brake;
};

// Applies actuator dynamics to a CmdL4 command stream.
class ActuatorModel {
public:
    void  initialize(const ActuatorParams& p, double nominal_dt);
    void  reset();
    // desired: commanded CmdL4; speed_mps: vehicle speed (for brake mu(T));
    // returns the realized CmdL4 to feed the plant.
    CmdL4 apply(const CmdL4& desired, double speed_mps, double dt);

    double brake_temperature() const { return brake_T_; }
    double steer_angle()       const { return steer_pos_; }

private:
    ActuatorParams p_{};
    double nominal_dt_ {0.005};

    // Per-channel transport-delay buffers.
    // std::deque: push_back + pop_front are both O(1) — unlike vector::erase(begin,...).
    std::deque<double> steer_buf_, throttle_buf_, brake_buf_;
    // First-order lag states.
    double steer_lag_ {0.0}, throttle_lag_ {0.0}, brake_lag_ {0.0};
    // Previous outputs (for rate limit) and steering servo states.
    double steer_out_ {0.0}, throttle_out_ {0.0}, brake_out_ {0.0};
    double steer_pos_ {0.0}, steer_vel_ {0.0}, lugre_z_ {0.0};
    double brake_T_ {40.0};
    bool   initialized_ {false};

    double push_delay(std::deque<double>& buf, double v, double L, double dt) const;
};

// Feedback-path transport delay: returns the state as seen by the controller
// `delay_s` seconds ago.  Buffers whole State snapshots.
class SensorDelay {
public:
    void  initialize(double delay_s, double nominal_dt);
    void  reset(const State& s);
    State apply(const State& measured, double dt);

private:
    std::vector<State> buf_;
    double delay_s_ {0.0};
    double nominal_dt_ {0.005};
    bool   initialized_ {false};
};

}  // namespace vdsim
