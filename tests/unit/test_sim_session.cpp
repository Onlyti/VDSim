// Unit tests for the SimSession kernel + RealTimeRunner.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

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
