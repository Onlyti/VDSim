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

TEST(LuGreTire, BreakawayFloorMuFzAtZeroSlip) {
    vdsim::ITireModel::Output mf{};
    mf.Fy = 0.0;
    mf.Fx = 0.0;
    const double Fz = 4000.0;
    const double mu = 1.0;
    const double g_lat = vdsim::lugre_breakaway(mf, false, Fz, mu, mu);
    EXPECT_NEAR(g_lat, mu * Fz, 1.0);
    const double g_long = vdsim::lugre_breakaway(mf, true, Fz, mu, mu);
    EXPECT_NEAR(g_long, 1.0, 1.0);
    const double g_lat_turn = vdsim::lugre_breakaway(mf, false, Fz, mu, mu, 0.08);
    EXPECT_LT(g_lat_turn, 0.35 * mu * Fz);
}

TEST(LuGreTire, CoastFlatNoPhantomLongitudinal) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.rolling_resistance = 0.0;
    tp.lugre.enabled = true;
    tp.lugre.sigma0 = 3.0e5;
    tp.lugre.sigma2 = 120.0;
    const vdsim::SolverParams sp;
    const auto contacts = flat_contacts();

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(8.0, vp.wheel_radius_nominal));
    vdsim::CmdL4 cmd;
    const vdsim::ControlInput u = cmd;
    double max_fx = 0.0, max_dom = 0.0;
    for (int i = 0; i < 1200; ++i) {
        dyn->step(u, contacts, 0.005);
        const auto F = dyn->tire_forces_body();
        for (int w = 0; w < vdsim::NUM_WHEELS; ++w)
            max_fx = std::max(max_fx, std::abs(F[w].x()));
        const auto& om = dyn->state().wheel_spin;
        for (int w = 0; w < vdsim::NUM_WHEELS; ++w)
            max_dom = std::max(max_dom, std::abs(om[w]));
    }
    EXPECT_LT(max_fx, 800.0);
    EXPECT_LT(max_dom, 80.0);
}

TEST(LuGreTire, RearLateralDuringSteer) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.rolling_resistance = 0.0;
    tp.lugre.enabled = true;
    tp.lugre.sigma0 = 3.0e5;
    tp.lugre.sigma2 = 120.0;
    const vdsim::SolverParams sp;
    const auto contacts = flat_contacts();

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(12.0, vp.wheel_radius_nominal));
    vdsim::CmdL4 cmd;
    cmd.steer_angle_wheel = 0.08;
    const vdsim::ControlInput u = cmd;
    double max_fy_rear = 0.0;
    for (int i = 0; i < 600; ++i) {
        dyn->step(u, contacts, 0.005);
        const auto F = dyn->tire_forces_body();
        max_fy_rear = std::max(max_fy_rear,
            std::abs(F[vdsim::WHEEL_RL].y()) + std::abs(F[vdsim::WHEEL_RR].y()));
    }
    EXPECT_GT(max_fy_rear, 400.0);
}

TEST(LuGreTire, LateralForceOpposesSlipIso8855) {
    vdsim::TireParams tp;
    tp.lugre.sigma0 = 2.0e5;
    tp.lugre.sigma2 = 0.0;
    auto tire = vdsim::create_pacejka_mf96();
    tire->initialize(tp);

    for (const double alpha : {0.04, -0.04, 0.12}) {
        vdsim::ITireModel::Input in;
        in.Fz = 4000.0;
        in.kappa = 0.0;
        in.alpha = alpha;
        in.mu_long = 1.0;
        in.mu_lat = 1.0;
        in.Vx_wheel = 12.0;
        const auto mf = tire->compute(in);
        ASSERT_GT(std::abs(mf.Fy), 50.0) << "alpha=" << alpha;

        const double v_slip_lat = in.Vx_wheel * std::tan(alpha);
        const double z_lat = 0.5 * vdsim::lugre_breakaway(mf, false, in.Fz, 1.0, 1.0)
                             / tp.lugre.sigma0;
        const auto lug = vdsim::lugre_wheel_forces(
            *tire, tp, 0.0, z_lat, 0.0, v_slip_lat, in);
        EXPECT_GT(mf.Fy * lug.Fy, 0.0) << "alpha=" << alpha;
        const double Fy_max = tp.D_lat * in.Fz * in.mu_lat * vdsim::lugre_mu_eff(tp, in.Fz);
        EXPECT_LE(std::abs(lug.Fy), Fy_max * 1.01) << "alpha=" << alpha;
    }
}

TEST(LuGreTire, LowSpeedYawDoesNotRunaway) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.rolling_resistance = 0.0;
    tp.lugre.enabled = true;
    tp.lugre.sigma0 = 3.0e5;
    tp.lugre.sigma2 = 120.0;
    const vdsim::SolverParams sp;
    const auto contacts = flat_contacts();

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    vdsim::State s0;
    s0.velocity.x() = 0.5;
    s0.angular_velocity.z() = 0.15;
    dyn->reset(s0);
    vdsim::CmdL4 cmd;
    const vdsim::ControlInput u = cmd;
    double max_r = 0.0;
    for (int i = 0; i < 800; ++i) {
        dyn->step(u, contacts, 0.005);
        max_r = std::max(max_r, std::abs(dyn->state().angular_velocity.z()));
    }
    EXPECT_LT(max_r, 2.0);
}

TEST(LuGreTire, LongitudinalWithSmallSteer) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vp.drive_type = vdsim::VehicleParams::Drive::RWD;
    const vdsim::SolverParams sp;
    const auto contacts = flat_contacts();

    auto run_vx = [&](double steer) {
        vdsim::TireParams tp;
        tp.rolling_resistance = 0.01;
        tp.lugre.enabled = true;
        tp.lugre.sigma0 = 3.0e5;
        tp.lugre.sigma2 = 120.0;
        auto dyn = vdsim::create_seven_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(init_state(10.0, vp.wheel_radius_nominal));
        vdsim::CmdL4 cmd;
        cmd.throttle = 0.35;
        cmd.steer_angle_wheel = steer;
        const vdsim::ControlInput u = cmd;
        for (int i = 0; i < 800; ++i) dyn->step(u, contacts, 0.005);
        return dyn->state().velocity.x();
    };

    const double vx_straight = run_vx(0.0);
    const double vx_steer    = run_vx(0.06);
    EXPECT_GT(vx_straight, 12.0);
    EXPECT_GT(vx_steer, 0.65 * vx_straight);
}

TEST(LuGreTire, ThrottleProducesForwardMotion) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vp.drive_type = vdsim::VehicleParams::Drive::RWD;
    vdsim::TireParams tp;
    tp.rolling_resistance = 0.01;
    tp.lugre.enabled = true;
    tp.lugre.sigma0 = 3.0e5;
    tp.lugre.sigma2 = 120.0;
    const vdsim::SolverParams sp;
    const auto contacts = flat_contacts();

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(0.0, vp.wheel_radius_nominal));
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.35;
    const vdsim::ControlInput u = cmd;
    double k_sign_flips = 0.0;
    double prev_k = 0.0;
    for (int i = 0; i < 1200; ++i) {
        dyn->step(u, contacts, 0.005);
        const double k = dyn->wheel_slip_ratio()[vdsim::WHEEL_RL];
        if (i > 0 && prev_k * k < 0.0 && std::abs(k) > 0.05) ++k_sign_flips;
        prev_k = k;
    }
    EXPECT_GT(dyn->state().velocity.x(), 3.0);
    EXPECT_LT(k_sign_flips, 80.0);
}

TEST(LuGreTire, RearSlipBoundedUnderThrottle) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vp.drive_type = vdsim::VehicleParams::Drive::RWD;
    const vdsim::SolverParams sp;
    const auto contacts = flat_contacts();

    auto max_rear_kappa = [&](bool lugre_on) {
        vdsim::TireParams tp;
        tp.rolling_resistance = 0.01;
        tp.lugre.enabled = lugre_on;
        tp.lugre.sigma0 = 3.0e5;
        tp.lugre.sigma2 = 120.0;
        auto dyn = vdsim::create_seven_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(init_state(6.0, vp.wheel_radius_nominal));
        vdsim::CmdL4 cmd;
        cmd.throttle = 0.3;
        const vdsim::ControlInput u = cmd;
        double peak = 0.0;
        for (int i = 0; i < 800; ++i) {
            dyn->step(u, contacts, 0.005);
            const auto k = dyn->wheel_slip_ratio();
            peak = std::max(peak, std::abs(k[vdsim::WHEEL_RL]));
            peak = std::max(peak, std::abs(k[vdsim::WHEEL_RR]));
        }
        return peak;
    };

    const double k_lugre  = max_rear_kappa(true);
    const double k_blend  = max_rear_kappa(false);
    EXPECT_LT(k_lugre, 0.55);
    EXPECT_LT(k_lugre, 2.5 * std::max(k_blend, 0.05));
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
