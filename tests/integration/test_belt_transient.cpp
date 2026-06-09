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

double step_steer_ay(bool belt, double t_s) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;          // belt acts on the MF path
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
