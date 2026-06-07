#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/lugre_tire.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

vdsim::ContactArray graded_contacts(double grade) {
    vdsim::ContactArray c;
    const double cz = std::sqrt(std::max(0.0, 1.0 - grade * grade));
    for (auto& p : c) {
        p.is_valid = true;
        p.normal = vdsim::Vec3(grade, 0.0, cz);
        p.mu_long = 1.0;
        p.mu_lat  = 1.0;
    }
    return c;
}

vdsim::ContactArray flat_contacts(double mu = 1.0) {
    vdsim::ContactArray c;
    for (auto& p : c) {
        p.is_valid = true;
        p.normal = vdsim::Vec3::UnitZ();
        p.mu_long = mu;
        p.mu_lat  = mu;
    }
    return c;
}

vdsim::State init_state(double vx, double R) {
    vdsim::State s;
    s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

}  // namespace

TEST(LuGreTire, LessGradeCreepThanKinematicBlend) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    const vdsim::SolverParams sp;
    const auto contacts = graded_contacts(0.08);

    auto creep = [&](bool lugre_on) {
        vdsim::TireParams tp;
        tp.rolling_resistance = 0.0;
        tp.lugre.enabled = lugre_on;
        tp.lugre.sigma0 = 3.0e5;
        tp.lugre.sigma2 = 120.0;
        auto dyn = vdsim::create_seven_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(init_state(0.0, vp.wheel_radius_nominal));
        vdsim::CmdL4 cmd;
        const vdsim::ControlInput u = cmd;
        for (int i = 0; i < 600; ++i) dyn->step(u, contacts, 0.005);
        return std::abs(dyn->state().velocity.x());
    };

    const double vx_blend = creep(false);
    const double vx_lugre = creep(true);
    EXPECT_LT(vx_lugre, vx_blend);
}

TEST(LuGreTire, HighSpeedLongitudinalNearPacejka) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp_base;
    tp_base.rolling_resistance = 0.0;
    vdsim::TireParams tp_lugre = tp_base;
    tp_lugre.lugre.enabled = true;
    const vdsim::SolverParams sp;
    const auto contacts = flat_contacts();

    auto run_fx = [&](const vdsim::TireParams& tp) {
        auto dyn = vdsim::create_bicycle();
        dyn->initialize(vp, tp, sp);
        dyn->reset(init_state(15.0, vp.wheel_radius_nominal));
        vdsim::CmdL4 cmd;
        cmd.throttle = 0.35;
        const vdsim::ControlInput u = cmd;
        double fx_sum = 0.0;
        for (int i = 0; i < 1200; ++i) {
            dyn->step(u, contacts, 0.005);
            const auto F = dyn->tire_forces_body();
            fx_sum += F[vdsim::WHEEL_FL].x() + F[vdsim::WHEEL_RL].x();
        }
        return fx_sum / 1200.0;
    };

    const double fx_pacejka = run_fx(tp_base);
    const double fx_lugre   = run_fx(tp_lugre);
    EXPECT_GT(std::abs(fx_pacejka), 50.0);
    EXPECT_NEAR(fx_lugre, fx_pacejka, 0.35 * std::abs(fx_pacejka));
}

TEST(LuGreTire, SteadyDeflectionMatchesBreakaway) {
    vdsim::LuGreTireParams p;
    p.sigma0 = 2.0e5;
    p.sigma2 = 0.0;
    const double g = 3200.0;
    const double z_ss = g / p.sigma0;
    const double F = vdsim::lugre_force(z_ss, 0.0, g, p);
    EXPECT_NEAR(F, g, 0.05 * g);
}
