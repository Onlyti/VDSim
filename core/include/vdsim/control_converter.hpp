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
// Direction of travel (update_directed): while reversing (vx < -kReverseVx) a
// positive ax demand slows the car, so the same u maps to
//   u >= 0 =>  brake = clamp(u, 0, 1),   throttle = 0, gear = -1
//   u <  0 =>  throttle = clamp(-u, 0, 1), brake = 0, gear = -1 (drive backward)
// The loop itself (error, integrator, feed-forward) stays in the body frame, so
// the integrator is continuous through a direction change and forward motion is
// the plain update() mapping with gear = +1.
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
    // Swap the gains without touching the integrator (live retune).
    void set_gains(const Gains& g) noexcept { g_ = g; }
    const Gains& gains() const noexcept { return g_; }

    // Returns (throttle, brake), both in [0, 1].
    std::pair<double, double> update(double ax_target,
                                     double ax_meas,
                                     double dt) noexcept;

    /// Pedal command for one step of the acceleration loop.
    struct Pedals {
        double throttle {0.0};   ///< [0, 1]
        double brake    {0.0};   ///< [0, 1]
        int    gear     {1};     ///< +1 forward, -1 reverse
    };

    /// Longitudinal speed [m/s] below which (vx < -kReverseVx) the car counts as
    /// reversing. The band around standstill keeps the forward mapping so a held
    /// stop does not toggle between gears on sensor-level speed noise.
    static constexpr double kReverseVx = 0.05;

    /**
     * @brief One loop step with the pedal mapping chosen by the direction of travel.
     *
     * Forward or standing (vx >= -kReverseVx): identical to update() with gear +1.
     * Reversing (vx < -kReverseVx): the same body-frame command u is mapped so that a
     * positive ax demand (slowing the backward motion) brakes and a negative demand
     * (speeding up backward) drives with gear -1.
     *
     * @param ax_target desired body longitudinal acceleration [m/s^2]
     * @param ax_meas   measured body longitudinal acceleration [m/s^2]
     * @param vx        measured body longitudinal speed [m/s]
     * @param dt        step [s]; non-positive returns zero pedals without updating
     * @return throttle, brake and gear for this step
     */
    Pedals update_directed(double ax_target, double ax_meas, double vx,
                           double dt) noexcept;

    double last_error()    const noexcept { return prev_err_; }
    double integrator()    const noexcept { return integ_; }

    /**
     * @brief Append the controller memory (integrator, previous error, first-step
     *        flag) to a session snapshot blob. Gains are configuration, not state.
     * @param v snapshot blob to append to
     */
    void save_state(std::vector<double>& v) const;
    /**
     * @brief Read back what save_state() wrote, in the same order.
     * @param v snapshot blob
     * @param p read cursor, advanced past the consumed entries
     */
    void restore_state(const std::vector<double>& v, std::size_t& p);

private:
    /// PI-D + feed-forward command u for one step (advances integrator / error memory).
    double command(double ax_target, double ax_meas, double dt) noexcept;

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
    // Swap the gains without touching the integrator (live retune).
    void set_gains(const Gains& g) noexcept { g_ = g; }
    const Gains& gains() const noexcept { return g_; }
    // Returns desired ax_target [m/s^2] given (v_target, v_meas, dt).
    double update(double v_target, double v_meas, double dt) noexcept;

    double integrator() const noexcept { return integ_; }

    /**
     * @brief Append the controller memory (integrator, first-step flag) to a
     *        session snapshot blob.
     * @param v snapshot blob to append to
     */
    void save_state(std::vector<double>& v) const;
    /**
     * @brief Read back what save_state() wrote, in the same order.
     * @param v snapshot blob
     * @param p read cursor, advanced past the consumed entries
     */
    void restore_state(const std::vector<double>& v, std::size_t& p);

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
        int    gear {1};   ///< +1 forward, -1 reversing (LongAxController::update_directed)
    };

    void initialize(const Gains& g) noexcept;
    void reset() noexcept;
    /**
     * @brief One driver step: steer (pure pursuit, delayed) and pedals from the speed loop.
     *
     * The pedal map follows the direction of travel (LongAxController::update_directed):
     * while reversing (vx < -kReverseVx) a demand that speeds the car up backward is
     * throttle with gear -1 and a demand that slows it is brake; forward it is the plain
     * map with gear +1.
     * @return Output (throttle, brake, steer, gear).
     */
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
// Tunable gains of the CascadeController. Defaults are the values the cascade
// used before they became settable, so an untouched session behaves as before.
struct CascadeGains {
    LongVxController::Gains vx {};            // L6 speed loop
    LongAxController::Gains ax {};            // L5 accel loop (+ pedal feed-forward)
    double pp_lookahead_min {1.5};            // [m]  pure-pursuit lookahead floor
    double pp_lookahead_k   {0.40};           // [s]  Ld = max(min, k * vx)
    double yaw_kp {0.15};                     // [rad steer / (rad/s)] yaw-rate PI
    double yaw_ki {0.05};                     // [rad steer / rad]
};

class CascadeController {
public:
    void initialize(const VehicleParams& vp);
    void reset() noexcept;

    // Live retune: keeps every integrator and the pursuit progress.
    void set_gains(const CascadeGains& g) noexcept;
    const CascadeGains& gains() const noexcept { return gains_; }

    // Convert a ladder command to CmdL4 using current feedback.
    //   meas    : measured plant state (pose, vx for cascade)
    //   ax_meas : measured longitudinal accel [m/s²] (for L5/L6 inner loop)
    CmdL4 to_l4(const ControlInput& u, const State& meas, double ax_meas, double dt);

    /**
     * @brief Append every integrator this cascade carries between ticks (LongVx,
     *        LongAx, pure-pursuit progress index, yaw-rate PI) to a snapshot blob.
     * @param v snapshot blob to append to
     */
    void save_state(std::vector<double>& v) const;
    /**
     * @brief Read back what save_state() wrote, in the same order.
     * @param v snapshot blob
     * @param p read cursor, advanced past the consumed entries
     */
    void restore_state(const std::vector<double>& v, std::size_t& p);

private:
    // Independent longitudinal cascade (LcLon → throttle/brake[/gear]).
    void lon_to_pedals(const LcLonCmd& lon, const State& meas, double ax_meas,
                       double dt, CmdL4& out);
    // Independent lateral cascade → fills out.steer_angle_wheel (L4-L8) OR
    // out.steer_mode/steer_actuator (L1-L3 sub-L4, for a Dynamic steering subsystem).
    void lat_to_cmd(const LcLatCmd& lat, const State& meas, double dt, CmdL4& out);

    CascadeGains gains_ {};
    LongVxController vxc_;
    LongAxController axc_;
    PurePursuitController pp_;
    double wheelbase_ {2.7};
    double max_steer_ {0.5};
    int    pp_idx_    {0};
    // Lateral yaw-rate PI (L6 r_target → steer): integral state.
    double r_integ_   {0.0};
    double r_kp_      {0.15};   // [rad steer / (rad/s err)]
    double r_ki_      {0.05};
};

}  // namespace vdsim
