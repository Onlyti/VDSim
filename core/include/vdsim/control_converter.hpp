#pragma once

#include <vector>

#include "vdsim/control.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"

// L5 Longitudinal Acceleration controller (PI + feed-forward).
//
//   input : ax_target [m/s^2] desired longitudinal acceleration
//           ax_meas   [m/s^2] current measured ax (e.g., dyn.ax_body_est())
//           dt        [s]
//   output: (throttle, brake) in [0, 1] each, mutually exclusive sign side.
//
// Mapping convention:
//   u = Kp · e + Ki · integral + Kd · de/dt + Kff · ax_target
//   u > 0  =>  throttle = clamp(u, 0, 1), brake = 0
//   u <= 0 =>  throttle = 0, brake = clamp(-u, 0, 1)
//
// Integral anti-windup: clamp integral at +/- 1.0 / Ki.

#include <utility>

namespace vdsim {

class LongAxController {
public:
    struct Gains {
        double kp  {0.4};
        double ki  {0.6};
        double kd  {0.0};
        double kff {0.10};      // feed-forward ax->throttle gain
        double i_max {2.5};     // anti-windup magnitude on integral
    };

    void initialize(const Gains& g) noexcept;
    void reset() noexcept;

    // Returns (throttle, brake), both in [0, 1].
    std::pair<double, double> update(double ax_target,
                                     double ax_meas,
                                     double dt) noexcept;

    double last_error()    const noexcept { return prev_err_; }
    double integrator()    const noexcept { return integ_; }

private:
    Gains  g_;
    double integ_    {0.0};
    double prev_err_ {0.0};
    bool   first_    {true};
};

// L6 Velocity tracking — cascades vx error to ax_target, then feeds L5 PID.
class LongVxController {
public:
    struct Gains {
        double kp     {0.8};
        double ki     {0.20};
        double i_max  {3.0};
        double ax_clamp {3.5};   // [m/s^2] max output ax_target magnitude
    };

    void initialize(const Gains& g) noexcept;
    void reset() noexcept;
    // Returns desired ax_target [m/s^2] given (v_target, v_meas, dt).
    double update(double v_target, double v_meas, double dt) noexcept;

    double integrator() const noexcept { return integ_; }

private:
    Gains  g_;
    double integ_ {0.0};
    bool   first_ {true};
};

// L7 Pure Pursuit — pick a lookahead point relative to vehicle pose and
// produce a wheel-angle steer command using Ackerman geometry.
class PurePursuitController {
public:
    struct Gains {
        double lookahead_min {1.5};   // [m]
        double lookahead_k   {0.40};  // [s] -- Ld = max(min, k * vx)
        double max_steer     {0.55};  // [rad]
        double wheelbase     {2.7};   // [m]  set from VehicleParams
    };
    void initialize(const Gains& g) noexcept;
    // Inputs: vehicle world position (x, y), heading psi, vx.
    // path_x/y: monotonic in arc length.
    // Returns (steer [rad], curvature [1/m], lookahead point index).
    struct Output { double steer; double curvature; int idx; };
    Output update(double x, double y, double psi, double vx,
                  const double* path_x, const double* path_y, int n_pts,
                  int prev_idx = 0) const noexcept;

private:
    Gains g_;
};

// Human-like driver model — wraps PurePursuit + LongVxController and injects
// reaction time delay + noise on steer / throttle.
class DriverModel {
public:
    struct Gains {
        double lookahead_min   {2.0};
        double lookahead_k     {0.4};
        double wheelbase       {2.7};
        double max_steer       {0.55};
        double reaction_time_s {0.150};   // [s] applied to steer output
        double steer_noise_rms {0.005};   // [rad]
        double thr_noise_rms   {0.02};    // [-]
        // Embedded vx PID gains (default sensible).
        double vx_kp {0.6};
        double vx_ki {0.15};
    };
    struct Output {
        double throttle;
        double brake;
        double steer;
    };

    void initialize(const Gains& g) noexcept;
    void reset() noexcept;
    // Update: returns Output (throttle, brake, steer).
    Output update(double x, double y, double psi, double vx,
                  double v_target,
                  const double* path_x, const double* path_y, int n_pts,
                  double dt,
                  double rand_uniform_01_a = 0.5,
                  double rand_uniform_01_b = 0.5) noexcept;

private:
    Gains g_;
    PurePursuitController pp_;
    LongVxController       vxc_;
    LongAxController       axc_;
    // Steering delay buffer: simple ring with size = round(reaction / dt_nominal).
    std::vector<double> steer_buffer_;
    int                 steer_idx_ {0};
    int                 prev_idx_  {0};   // Pure-Pursuit lookahead progress (monotonic)
    double              prev_dt_   {0.005};
};

// =============================================================================
// CascadeController — converts ANY control-ladder level (CmdL1..CmdL8) to CmdL4.
//
//   L1-L4: stateless lowering (per-wheel/axle/Fx/pedal → throttle/brake/steer).
//   L5    : ax_target  → throttle/brake via LongAxController (needs ax_meas).
//   L6    : vx_target  → ax_target → throttle/brake (LongVx → LongAx cascade).
//   L7    : (v_target, kappa) → speed cascade + steer = atan(kappa·L).
//   L8    : waypoint path → PurePursuit steer + per-point speed cascade.
//
// Stateful (PID integrators + pursuit progress). Lives at the session level,
// fed the measured state each tick. This is the "controller" entity in the
// architecture: User Lc-command → CascadeController → CmdL4 → VehNetwork → actuator.
// =============================================================================
class CascadeController {
public:
    void initialize(const VehicleParams& vp);
    void reset() noexcept;

    // Convert a ladder command to CmdL4 using current feedback.
    //   meas    : measured plant state (pose, vx for cascade)
    //   ax_meas : measured longitudinal accel [m/s²] (for L5/L6 inner loop)
    CmdL4 to_l4(const ControlInput& u, const State& meas, double ax_meas, double dt);

private:
    LongVxController vxc_;
    LongAxController axc_;
    PurePursuitController pp_;
    double wheelbase_ {2.7};
    double max_steer_ {0.5};
    int    pp_idx_    {0};
};

}  // namespace vdsim
