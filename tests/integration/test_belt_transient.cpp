// T2.2 — belt transient on the L2 (seven_dof) MF path.
//
// With the belt relaxation on, the slip the tire sees lags the geometric slip
// (tau = sigma/|Vx|), so the early lateral response to a step steer is lower than
// with instant slip; both reach the same steady state.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

double step_steer_ay(bool belt, double t_s, bool lugre = false) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = lugre;          // belt acts on MF (lugre off) or LuGre slip-velocity path
    tp.belt.enabled = belt;
    tp.belt.sigma_lat = 0.6;           // tau = 0.6/20 = 30 ms at 20 m/s
    vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);

    vdsim::State s;
    s.velocity.x() = 20.0;
    const double w = 20.0 / vp.wheel_radius_nominal;
    s.wheel_spin = {{w, w, w, w}};
    dyn->reset(s);

    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.04;   // step steer
    const vdsim::ControlInput u = cmd;
    const int n = static_cast<int>(t_s / 0.001);
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray c;
        ground->query(dyn->state(), vp, c);
        dyn->step(u, c, 0.001);
    }
    return std::abs(dyn->ay_body_est());
}

double l5_step_steer_ay(bool belt, double t_s) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    tp.belt.enabled = belt;
    tp.belt.sigma_lat = 0.6;
    vdsim::SolverParams sp; sp.stunt_physics = true; sp.max_substep_dt = 2e-4; sp.max_substeps = 16;
    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, sp);
    vdsim::State s;
    s.position.z() = vp.cg_height;
    s.velocity.x() = 20.0;
    const double w = 20.0 / vp.wheel_radius_nominal;
    s.wheel_spin = {{w, w, w, w}};
    dyn->reset(s);
    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.04;
    const vdsim::ControlInput u = cmd;
    const int n = static_cast<int>(t_s / 0.001);
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray c;
        ground->query(dyn->state(), vp, c);
        dyn->step(u, c, 0.001);
    }
    return std::abs(dyn->ay_body_est());
}

// L1 (bicycle) has no ay accessor; use the yaw rate as the belt-observable response.
double l1_step_steer_yaw(bool belt, double t_s, bool lugre = false) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = lugre;
    tp.belt.enabled = belt;
    tp.belt.sigma_lat = 0.6;
    vdsim::SolverParams sp;
    auto dyn = vdsim::create_bicycle();
    dyn->initialize(vp, tp, sp);
    vdsim::State s;
    s.velocity.x() = 20.0;
    const double w = 20.0 / vp.wheel_radius_nominal;
    s.wheel_spin = {{w, w, w, w}};
    dyn->reset(s);
    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.04;
    const vdsim::ControlInput u = cmd;
    const int n = static_cast<int>(t_s / 0.001);
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray c;
        ground->query(dyn->state(), vp, c);
        dyn->step(u, c, 0.001);
    }
    return std::abs(dyn->state().yaw_rate());
}

}  // namespace

TEST(BeltTransient, LateralResponseLagsThenConverges) {
    // Early (around one relaxation time tau ~ 30 ms): belt lags -> lower ay.
    const double ay_off_early = step_steer_ay(false, 0.020);
    const double ay_on_early  = step_steer_ay(true,  0.020);
    EXPECT_GT(ay_off_early, 0.1) << "sanity: vehicle responds to the step steer";
    EXPECT_LT(ay_on_early, ay_off_early) << "belt relaxation lags the early response";

    // Steady (many tau later): the two converge.
    const double ay_off_ss = step_steer_ay(false, 0.8);
    const double ay_on_ss  = step_steer_ay(true,  0.8);
    EXPECT_GT(ay_off_ss, 0.5);
    EXPECT_NEAR(ay_on_ss, ay_off_ss, 0.15 * ay_off_ss) << "belt does not change steady state";
}

// T2.3 — belt stacks on the LuGre (default) path: relaxes the slip velocity
// feeding v_r, so the early response lags with belt on and converges by steady.
TEST(BeltTransient, LuGrePathAlsoLagsThenConverges) {
    const double ay_off_early = step_steer_ay(false, 0.020, /*lugre=*/true);
    const double ay_on_early  = step_steer_ay(true,  0.020, /*lugre=*/true);
    EXPECT_GT(ay_off_early, 0.1);
    EXPECT_LT(ay_on_early, ay_off_early) << "belt lags the LuGre early response";

    const double ay_off_ss = step_steer_ay(false, 0.8, true);
    const double ay_on_ss  = step_steer_ay(true,  0.8, true);
    EXPECT_GT(ay_off_ss, 0.5);
    EXPECT_NEAR(ay_on_ss, ay_off_ss, 0.20 * ay_off_ss);
}

// T2.4 — belt also wired into the L5 (free_3d) MF path.
TEST(BeltTransient, L5PathLagsWithBelt) {
    const double off = l5_step_steer_ay(false, 0.020);
    const double on  = l5_step_steer_ay(true,  0.020);
    EXPECT_GT(off, 0.1) << "L5 responds to the step steer";
    EXPECT_LT(on, off)  << "belt lags the L5 early response";
}

// T2 tail — belt wired into the L1 (bicycle) MF path; early yaw response lags,
// steady state unchanged.
TEST(BeltTransient, L1PathLagsWithBelt) {
    const double off_early = l1_step_steer_yaw(false, 0.020);
    const double on_early  = l1_step_steer_yaw(true,  0.020);
    EXPECT_GT(off_early, 1e-3) << "L1 responds to the step steer";
    EXPECT_LT(on_early, off_early) << "belt lags the L1 early yaw response";

    const double off_ss = l1_step_steer_yaw(false, 0.8);
    const double on_ss  = l1_step_steer_yaw(true,  0.8);
    EXPECT_GT(off_ss, 1e-3);
    EXPECT_NEAR(on_ss, off_ss, 0.20 * off_ss) << "belt is transient-only; steady yaw unchanged";
}

// L1 belt also stacks on the LuGre path (relaxed slip velocity).
TEST(BeltTransient, L1LuGrePathLagsWithBelt) {
    const double off = l1_step_steer_yaw(false, 0.020, /*lugre=*/true);
    const double on  = l1_step_steer_yaw(true,  0.020, /*lugre=*/true);
    EXPECT_GT(off, 1e-3) << "L1 LuGre responds to the step steer";
    EXPECT_LT(on, off)   << "belt lags the L1 LuGre early response";
}
