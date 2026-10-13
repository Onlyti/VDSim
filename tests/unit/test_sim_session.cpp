// Unit tests for the SimSession kernel + RealTimeRunner.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>
#include <variant>

#include "vdsim/realtime_runner.hpp"
#include "vdsim/sim_session.hpp"

using namespace vdsim;

// SimSession holds a std::mutex (non-movable), so build it on the heap.
static std::unique_ptr<SimSession> make_session(const SimConfig& cfg = {}) {
    VehicleParams vp; TireParams tp; SolverParams sp;
    auto s = std::make_unique<SimSession>(
        create_seven_dof(), create_flat_ground(0.0, 1.0), vp, tp, sp, cfg);
    State s0; s0.velocity = {20.0, 0.0, 0.0};
    s->reset(s0);
    return s;
}

// External-step (experiment) mode: caller drives tick(); latched input is held.
TEST(SimSession, ExternalStepAdvancesAndLatches) {
    auto s = make_session();
    CmdL4 cmd; cmd.steer_angle_wheel = 0.03;
    s->set_input(cmd);
    for (int i = 0; i < 200; ++i) s->tick(0.005);   // latch held across ticks
    EXPECT_NEAR(s->sim_time(), 1.0, 1e-9);
    EXPECT_GT(s->state().yaw_rate(), 1e-4);          // +steer => +yaw (ISO)
}

// Latch (ZOH): a single set_input persists; straightening reduces yaw.
TEST(SimSession, InputIsHeldUntilReplaced) {
    auto s = make_session();
    CmdL4 turn; turn.steer_angle_wheel = 0.05;
    s->set_input(turn);
    for (int i = 0; i < 100; ++i) s->tick(0.005);
    const double r_turning = s->state().yaw_rate();
    EXPECT_GT(r_turning, 1e-3);
    s->set_input(CmdL4{});                            // straighten
    for (int i = 0; i < 300; ++i) s->tick(0.005);
    EXPECT_LT(std::abs(s->state().yaw_rate()), std::abs(r_turning));
}

// Sensor delay: measured_state lags the true state.
TEST(SimSession, SensorDelayLagsMeasuredState) {
    SimConfig cfg; cfg.sensor_delay_s = 0.05;         // 10-step delay at dt=0.005
    auto s = make_session(cfg);
    CmdL4 cmd; cmd.throttle = 1.0;
    s->set_input(cmd);
    for (int i = 0; i < 60; ++i) s->tick(0.005);
    EXPECT_GT(s->state().vx(), s->measured_state().vx());  // true accelerated more
}

// Actuator layer is wired: a steering first-order lag slows the steered response.
TEST(SimSession, ActuatorLayerIsApplied) {
    SimConfig lag; lag.actuator.steer.ch.tau_s = 0.2;
    auto s_lag = make_session(lag);
    auto s_id  = make_session();
    CmdL4 cmd; cmd.steer_angle_wheel = 0.05;
    s_lag->set_input(cmd); s_id->set_input(cmd);
    for (int i = 0; i < 20; ++i) { s_lag->tick(0.005); s_id->tick(0.005); }
    EXPECT_LT(std::abs(s_lag->state().yaw_rate()), std::abs(s_id->state().yaw_rate()));
}

// RealTimeRunner: paces ticks on its own thread; sim advances; fail-safe brake
// engages when no input arrives, decelerating the vehicle.
TEST(SimSession, RealTimeRunnerPacesAndFailsafe) {
    auto s = make_session();
    RealTimeRunner::Config cfg;
    cfg.dt = 0.002; cfg.cmd_timeout_s = 0.005;        // quick failsafe
    cfg.failsafe = CmdL4{0.0, 0.6, 0.0, 1, false};    // brake
    RealTimeRunner runner(*s, cfg);
    runner.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    runner.stop();
    EXPECT_GT(s->sim_time(), 0.0);                    // advanced in real time
    EXPECT_TRUE(std::isfinite(s->state().vx()));
    EXPECT_LT(s->state().vx(), 20.0);                 // fail-safe brake decelerated
}

// CascadeController: set_input(CmdL6) converts a speed target to pedals each tick.
// Starting at 20 m/s, a 10 m/s target must decelerate the vehicle toward 10.
TEST(SimSession, CascadeL6SpeedTargetTracks) {
    auto s = make_session();   // starts at vx = 20 m/s
    CmdL6 cmd; cmd.v_target = 10.0;
    for (int i = 0; i < int(15.0 / 0.005); ++i) { s->set_input(ControlInput{cmd}); s->tick(0.005); }
    EXPECT_NEAR(s->state().vx(), 10.0, 1.5) << "L6 cruise should converge to target";
}

// CascadeController: set_input(CmdL7) with curvature produces a steady yaw rate.
TEST(SimSession, CascadeL7CurvatureYaws) {
    auto s = make_session();   // 20 m/s
    CmdL7 cmd; cmd.v_target = 12.0; cmd.kappa = 0.02;
    for (int i = 0; i < int(8.0 / 0.005); ++i) { s->set_input(ControlInput{cmd}); s->tick(0.005); }
    EXPECT_GT(std::abs(s->state().yaw_rate()), 0.05) << "L7 curvature should yaw the vehicle";
}

// CmdSplit: independent longitudinal (vx) + lateral (yaw-rate) levels.
TEST(SimSession, CascadeSplitIndependentAxes) {
    auto s = make_session();   // 20 m/s
    CmdSplit cmd;
    cmd.lon = LcLonL6{18.0};       // vx target 18 m/s
    cmd.lat = LcLatL6{0.15};       // yaw-rate target 0.15 rad/s
    for (int i = 0; i < int(8.0 / 0.005); ++i) { s->set_input(ControlInput{cmd}); s->tick(0.005); }
    EXPECT_NEAR(s->state().vx(), 18.0, 1.5)              << "lon axis tracks vx target";
    EXPECT_NEAR(s->state().yaw_rate(), 0.15, 0.08)       << "lat axis tracks yaw-rate target";
}

// Dynamic steering (Rack EOM): LcLat L1 steer torque moves the rack and yaws.
// Build a dynamic-steering session directly (make_session uses defaults).
TEST(SimSession, DynamicSteeringTorqueYaws) {
    VehicleParams vp; vp.steering_dynamic = true;
    TireParams tp; SolverParams sp;
    auto s = std::make_unique<SimSession>(
        create_seven_dof(), create_flat_ground(0.0, 1.0), vp, tp, sp);
    State s0; s0.velocity = {15.0, 0.0, 0.0};
    const double w = 15.0 / 0.33; s0.wheel_spin = {{w, w, w, w}};
    s->reset(s0);
    CmdSplit cmd;
    cmd.lon = LcLonL6{15.0};
    cmd.lat = LcLatL1{1.0};        // +1 Nm column torque
    for (int i = 0; i < int(3.0 / 0.005); ++i) { s->set_input(ControlInput{cmd}); s->tick(0.005); }
    EXPECT_GT(std::abs(s->state().rack_travel), 1e-3)  << "torque should move the rack";
    EXPECT_GT(s->state().yaw_rate(), 0.02)             << "+torque → +rack → +yaw";
    // Steering lock: rack must not exceed max_steer · ratio.
    EXPECT_LE(std::abs(s->state().rack_travel),
              vp.max_steer_angle_wheel * vp.steering_ratio + 1e-6) << "rack clamps at lock";
}

// ---- Sensor lag (SensorNoise::lag_tau_s) -----------------------------------

static SimConfig lag_cfg(double tau) {
    SimConfig cfg;
    cfg.sensors.enabled = true;           // all noise 0 -> measured == (lagged) truth
    cfg.sensors.gnss_vel.lag_tau_s = tau;
    return cfg;
}

// tau = 0 is a pure pass-through; tau > 0 starts primed on the truth (no
// transient from zero) and then trails an accelerating signal.
TEST(SimSession, SensorLagTrailsTruthAndStartsPrimed) {
    auto plain = make_session(lag_cfg(0.0));
    auto lag   = make_session(lag_cfg(0.5));
    CmdL4 cmd; cmd.throttle = 1.0;
    plain->set_input(cmd); lag->set_input(cmd);
    plain->tick(0.005); lag->tick(0.005);
    EXPECT_NEAR(plain->output().sensors.gnss_vx, plain->state().vx(), 1e-9);
    EXPECT_NEAR(lag->output().sensors.gnss_vx, 20.0, 0.1) << "primed on the first sample";
    for (int i = 0; i < 400; ++i) { plain->tick(0.005); lag->tick(0.005); }
    EXPECT_NEAR(plain->output().sensors.gnss_vx, plain->state().vx(), 1e-9);
    EXPECT_LT(lag->output().sensors.gnss_vx, lag->state().vx() - 0.2);
}

// The lag state is part of the sensor snapshot: restore replays bit-identically.
TEST(SimSession, SensorLagSurvivesSnapshotRestore) {
    auto s = make_session(lag_cfg(0.3));
    CmdL4 cmd; cmd.throttle = 0.8;
    s->set_input(cmd);
    for (int i = 0; i < 100; ++i) s->tick(0.005);
    const SessionSnapshot snap = s->snapshot();
    for (int i = 0; i < 100; ++i) s->tick(0.005);
    const double first = s->output().sensors.gnss_vx;
    s->restore(snap);
    for (int i = 0; i < 100; ++i) s->tick(0.005);
    EXPECT_EQ(first, s->output().sensors.gnss_vx);
}

// ---- CascadeGains -----------------------------------------------------------

static double l6_speed_after(SimSession& s, double v_target, double seconds) {
    CmdL6 cmd; cmd.v_target = v_target;
    for (int i = 0; i < int(seconds / 0.005); ++i) { s.set_input(ControlInput{cmd}); s.tick(0.005); }
    return s.state().vx();
}

TEST(SimSession, CascadeGainsDefaultsAreBitIdentical) {
    auto a = make_session();
    auto b = make_session();
    b->set_cascade_gains(CascadeGains{});
    EXPECT_EQ(l6_speed_after(*a, 10.0, 6.0), l6_speed_after(*b, 10.0, 6.0));
}

TEST(SimSession, CascadeGainsRetuneChangesTracking) {
    auto def = make_session();
    auto off = make_session();
    CascadeGains g; g.vx.kp = 0.0; g.vx.ki = 0.0;     // speed loop disabled
    off->set_cascade_gains(g);
    EXPECT_NEAR(l6_speed_after(*def, 10.0, 15.0), 10.0, 1.5);
    EXPECT_GT(l6_speed_after(*off, 10.0, 15.0), 15.0) << "no speed loop -> no braking to target";
}

TEST(SimSession, CascadeGainsPersistAcrossResetAndRandomization) {
    auto s = make_session();
    CascadeGains g; g.vx.kp = 1.7; g.yaw_ki = 0.2; g.pp_lookahead_min = 3.0;
    s->set_cascade_gains(g);
    DomainRandomization d; d.mass_scale = 1.1;
    s->set_randomization(d);
    State s0; s0.velocity = {20.0, 0.0, 0.0};
    s->reset(s0);
    const CascadeGains r = s->cascade_gains();
    EXPECT_EQ(r.vx.kp, 1.7);
    EXPECT_EQ(r.yaw_ki, 0.2);
    EXPECT_EQ(r.pp_lookahead_min, 3.0);
}

TEST(SimSession, CascadeGainsRejectBadValues) {
    auto s = make_session();
    CascadeGains neg; neg.ax.kp = -0.1;
    EXPECT_THROW(s->set_cascade_gains(neg), std::invalid_argument);
    CascadeGains nan_; nan_.yaw_kp = std::nan("");
    EXPECT_THROW(s->set_cascade_gains(nan_), std::invalid_argument);
    EXPECT_EQ(s->cascade_gains().ax.kp, CascadeGains{}.ax.kp) << "rejected set leaves gains unchanged";
}

// The fail-safe is a ControlInput now. Its default must still be the historical
// L4 brake, and latching it through the variant overload must leave the session
// bit-identical to the old CmdL4 overload the runner used to call.
TEST(SimSession, RealTimeRunnerDefaultFailsafeIsBitIdenticalL4) {
    const RealTimeRunner::Config cfg;
    ASSERT_TRUE(std::holds_alternative<CmdL4>(cfg.failsafe));
    const CmdL4 historical{0.0, 0.3, 0.0, 1, false};
    const CmdL4& fs = std::get<CmdL4>(cfg.failsafe);
    EXPECT_EQ(fs.throttle, historical.throttle);
    EXPECT_EQ(fs.brake, historical.brake);
    EXPECT_EQ(fs.steer_angle_wheel, historical.steer_angle_wheel);
    EXPECT_EQ(fs.gear, historical.gear);
    EXPECT_EQ(fs.handbrake, historical.handbrake);

    auto s_old = make_session();
    auto s_new = make_session();
    s_old->set_input(historical);        // pre-change call: CmdL4 overload
    s_new->set_input(cfg.failsafe);      // post-change call: ControlInput overload
    for (int i = 0; i < 400; ++i) { s_old->tick(0.005); s_new->tick(0.005); }
    const SessionSnapshot a = s_old->snapshot();
    const SessionSnapshot b = s_new->snapshot();
    ASSERT_EQ(a.d.size(), b.d.size());
    for (std::size_t i = 0; i < a.d.size(); ++i) EXPECT_EQ(a.d[i], b.d[i]) << "slot " << i;
    EXPECT_EQ(a.rng, b.rng);
}

// max_ticks: the loop stops by itself after exactly that many ticks, reports
// finished(), and stop() still joins it.
TEST(SimSession, RealTimeRunnerMaxTicksFinishesAndCounts) {
    auto s = make_session();
    RealTimeRunner::Config cfg;
    cfg.dt = 0.001; cfg.cmd_timeout_s = 0.0; cfg.max_ticks = 50;
    RealTimeRunner runner(*s, cfg);
    const double t0 = s->sim_time();
    runner.start();
    for (int i = 0; i < 400 && !runner.finished(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_TRUE(runner.finished());
    EXPECT_TRUE(runner.running());                 // still joinable until stop()
    const auto st = runner.stats();
    EXPECT_EQ(st.ticks, 50u);
    EXPECT_NEAR(st.sim_elapsed_s, 50 * 0.001, 1e-12);
    EXPECT_NEAR(s->sim_time() - t0, 50 * 0.001, 1e-12);
    EXPECT_GT(st.wall_elapsed_s, 0.0);
    EXPECT_TRUE(st.error.empty());
    EXPECT_EQ(st.failsafe_latches, 0u);
    runner.stop();
    EXPECT_FALSE(runner.running());
}

// A non-L4 fail-safe (L1 per-wheel brake torque) is latched by the watchdog and
// counted.
TEST(SimSession, RealTimeRunnerL1FailsafeLatchesAndBrakes) {
    auto s = make_session();
    RealTimeRunner::Config cfg;
    cfg.dt = 0.002; cfg.cmd_timeout_s = 0.005;
    CmdL1 brake;
    brake.brake_torque = {{1500.0, 1500.0, 1500.0, 1500.0}};
    cfg.failsafe = brake;
    RealTimeRunner runner(*s, cfg);
    runner.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    runner.stop();
    EXPECT_GE(runner.stats().failsafe_latches, 1u);
    EXPECT_TRUE(std::isfinite(s->state().vx()));
    EXPECT_LT(s->state().vx(), 20.0);
}

// ---- Direct-control level selection (work order 32, B2) --------------------
// Ported from the retired feat/direct-control-dynamics-level branch: its
// dynamics_level option is DirectControlSessionOptions::level here.
static std::unique_ptr<SimSession> make_dc_level(const char* level) {
    VehicleParams vp; TireParams tp; SolverParams sp;
    DirectControlSessionOptions o;
    o.level = level;
    return make_direct_control_session(vp, TireSetup(tp), sp, o);
}

TEST(DirectControlSession, DefaultLevelIsSevenDof) {
    EXPECT_EQ(DirectControlSessionOptions{}.level, "L2");
    VehicleParams vp; TireParams tp; SolverParams sp;
    auto s = make_direct_control_session(vp, TireSetup(tp), sp);
    EXPECT_EQ(s->dynamics().level(), IVehicleDynamics::Level::L2_SevenDOF);
}

TEST(DirectControlSession, L3AndL4CreateAndTickFinite) {
    const std::pair<const char*, IVehicleDynamics::Level> cases[] = {
        {"L3", IVehicleDynamics::Level::L3_FourteenDOF},
        {"L4", IVehicleDynamics::Level::L4_Kinematic}};
    const double dt = DirectControlSessionOptions{}.nominal_dt;
    for (const auto& [name, lvl] : cases) {
        auto s = make_dc_level(name);
        EXPECT_EQ(s->dynamics().level(), lvl) << name;
        EXPECT_TRUE(s->vehicle_params().plant_path) << name;
        State s0; s0.velocity = {20.0, 0.0, 0.0};
        s->reset(s0);
        CmdL1 cmd; cmd.steer_angle_wheel = 0.02;
        for (int i = 0; i < 2000; ++i) { s->set_input(cmd); s->tick(dt); }
        EXPECT_TRUE(std::isfinite(s->state().vx())) << name;
        EXPECT_GT(s->state().yaw_rate(), 1e-4) << name;
    }
}
