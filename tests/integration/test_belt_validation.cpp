// T2 validation — belt transient step-steer response vs the relaxation-length
// analytic. The belt is a first-order slip lag with time constant tau = sigma/|Vx|,
// so at an early time the lateral acceleration is suppressed relative to belt-off,
// and the suppression is stronger at lower speed (larger tau). Steady state is
// unchanged (belt is a transient-only filter). A fixed-time |ay| metric is used
// rather than time-to-fraction, which is corrupted by the underdamped overshoot.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

vdsim::State straight(double Vx) {
    vdsim::State s;
    s.velocity.x() = Vx;
    const double w = Vx / 0.31;               // default wheel radius
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

// |ay| at wall-clock time t_s after a step steer, MF path (LuGre off).
double ay_at_time(bool belt, double sigma, double Vx, double t_s) {
    vdsim::VehicleParams vp;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    tp.belt.enabled = belt;
    tp.belt.sigma_lat = sigma;
    tp.belt.sigma_long = sigma;
    vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(straight(Vx));
    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.03;
    const vdsim::ControlInput u = cmd;
    const int n = static_cast<int>(t_s / 0.001);
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray c; ground->query(dyn->state(), vp, c);
        dyn->step(u, c, 0.001);
    }
    return std::abs(dyn->ay_body_est());
}

}  // namespace

TEST(BeltValidation, SteadyStateUnchanged) {
    const double Vx = 20.0, sigma = 0.6;
    const double ay_off = ay_at_time(false, sigma, Vx, 1.5);
    const double ay_on  = ay_at_time(true,  sigma, Vx, 1.5);
    EXPECT_GT(ay_off, 0.5);
    EXPECT_NEAR(ay_on, ay_off, 0.08 * ay_off) << "belt is transient-only; steady ay unchanged";
}

TEST(BeltValidation, EarlyResponseSuppressedByBelt) {
    const double Vx = 20.0, sigma = 0.6;
    const double tau = sigma / Vx;            // 0.03 s
    const double ay_off = ay_at_time(false, sigma, Vx, tau);
    const double ay_on  = ay_at_time(true,  sigma, Vx, tau);
    EXPECT_GT(ay_off, 0.05) << "vehicle responds to the step";
    EXPECT_LT(ay_on, 0.95 * ay_off) << "belt relaxation suppresses the early response";
    EXPECT_GT(ay_on, 0.15 * ay_off) << "but the tire is still building force";
}

TEST(BeltValidation, MoreLagAtLowerSpeed) {
    // tau = sigma/|Vx|: at a fixed early time the slower car (larger tau) is more
    // suppressed -> smaller belt-on/off ratio.
    const double sigma = 0.6, t0 = 0.025;
    const double r_slow = ay_at_time(true, sigma, 12.0, t0) / ay_at_time(false, sigma, 12.0, t0);
    const double r_fast = ay_at_time(true, sigma, 34.0, t0) / ay_at_time(false, sigma, 34.0, t0);
    EXPECT_LT(r_slow, r_fast) << "relaxation tau = sigma/|Vx| -> more lag at lower speed";
}
