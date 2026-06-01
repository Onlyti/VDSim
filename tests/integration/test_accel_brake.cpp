// Longitudinal scenarios:
//   1. Drag coast — vx0 = 20, throttle/brake = 0  → vx decays per analytical drag-only solution.
//   2. Throttle step — vx0 = 5, throttle = 0.5    → vx increases monotonically, accelerates.
//   3. Brake step    — vx0 = 20, brake = 0.8      → vx decreases monotonically with |a| > 2 m/s^2.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr double RHO_AIR = 1.225;

vdsim::ContactArray flat_contacts(double mu = 1.0) {
    vdsim::ContactArray c;
    for (auto& p : c) { p.is_valid = true; p.normal = vdsim::Vec3::UnitZ();
                       p.mu_long = mu; p.mu_lat = mu; }
    return c;
}

vdsim::State init_state(double vx, double R) {
    vdsim::State s; s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

double drag_decay_analytical(double vx0, double t,
                             const vdsim::VehicleParams& vp) {
    // m dv/dt = -0.5 rho Cd A v^2  =>  v(t) = vx0 / (1 + vx0 k t),
    //   k = 0.5 rho Cd A / m
    const double k = 0.5 * RHO_AIR * vp.aero_drag_coeff * vp.frontal_area / vp.mass;
    return vx0 / (1.0 + vx0 * k * t);
}

}  // namespace

TEST(LongScenarios, DragCoastMatchesAnalytical) {
    vdsim::VehicleParams vp;     // Cd*A active by default
    vdsim::TireParams tp;
    tp.rolling_resistance = 0.0;     // analytical baseline ignores rolling
    const vdsim::SolverParams sp;

    auto dyn = vdsim::create_bicycle();
    dyn->initialize(vp, tp, sp);

    const double vx0 = 20.0;
    dyn->reset(init_state(vx0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd;   // throttle = brake = 0
    vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();

    const double T = 10.0, dt = 0.005;
    const int N = static_cast<int>(std::round(T / dt));
    for (int i = 0; i < N; ++i) dyn->step(u, contacts, dt);

    const double vx_sim = dyn->state().velocity.x();
    const double vx_ana = drag_decay_analytical(vx0, T, vp);

    // Tire rolling drag also bleeds energy in sim, so vx_sim <= vx_ana slightly.
    EXPECT_LT(vx_sim, vx0);
    EXPECT_NEAR(vx_sim, vx_ana, 0.05 * vx_ana)
        << "vx_sim=" << vx_sim << " vx_ana=" << vx_ana;
}

TEST(LongScenarios, RollingResistanceReducesCoastDistance) {
    auto coast_distance = [](double rr) {
        vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
        vdsim::TireParams tp; tp.rolling_resistance = rr;
        const vdsim::SolverParams sp;
        auto dyn = vdsim::create_bicycle();
        dyn->initialize(vp, tp, sp);
        dyn->reset(init_state(20.0, vp.wheel_radius_nominal));
        vdsim::CmdL4 cmd; vdsim::ControlInput u = cmd;
        const auto contacts = flat_contacts();
        for (int i = 0; i < 1000; ++i) dyn->step(u, contacts, 0.005);
        return dyn->state().velocity.x();
    };
    const double vx_no_rr   = coast_distance(0.0);
    const double vx_with_rr = coast_distance(0.015);
    // With RR the coast vx decay is faster.
    EXPECT_LT(vx_with_rr, vx_no_rr);
}

TEST(LongScenarios, ThrottleStepAccelerates) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.rolling_resistance = 0.0;
    const vdsim::SolverParams sp;

    auto dyn = vdsim::create_bicycle();
    dyn->initialize(vp, tp, sp);

    const double vx0 = 5.0;
    dyn->reset(init_state(vx0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.throttle = 0.5;
    vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();

    double prev_vx = vx0;
    int n_decreased = 0;
    for (int i = 0; i < 800; ++i) {     // 4 s
        dyn->step(u, contacts, 0.005);
        const double vx = dyn->state().velocity.x();
        if (vx + 1e-4 < prev_vx) ++n_decreased;
        prev_vx = vx;
    }
    EXPECT_EQ(n_decreased, 0) << "vx must be monotonically non-decreasing";
    EXPECT_GT(dyn->state().velocity.x(), vx0 + 1.0);   // gained > 1 m/s in 4 s
}

TEST(LongScenarios, BrakeStepDeceleratesAtAtLeastTwo) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    const vdsim::TireParams tp;
    const vdsim::SolverParams sp;

    auto dyn = vdsim::create_bicycle();
    dyn->initialize(vp, tp, sp);

    const double vx0 = 20.0;
    dyn->reset(init_state(vx0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.brake = 0.8;
    vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();

    double prev_vx = vx0;
    int n_increased = 0;
    const double T = 2.0, dt = 0.005;
    const int N = static_cast<int>(std::round(T / dt));
    for (int i = 0; i < N; ++i) {
        dyn->step(u, contacts, dt);
        const double vx = dyn->state().velocity.x();
        if (vx > prev_vx + 1e-4) ++n_increased;
        prev_vx = vx;
    }
    EXPECT_EQ(n_increased, 0) << "vx must be monotonically non-increasing under brake";
    const double vx_end = dyn->state().velocity.x();
    const double a_avg = (vx0 - vx_end) / T;
    EXPECT_GE(a_avg, 2.0) << "average decel must be >= 2 m/s^2";
}
