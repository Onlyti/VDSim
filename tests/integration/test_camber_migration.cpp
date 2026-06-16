// Camber-induced contact-point migration -> overturning moment.
// A cambered tire contacts on the leaning side (dy = crown_radius*sin gamma),
// shifting the vertical load line and producing Mx = Fz*dy. crown_radius=0
// reproduces the legacy centerline contact (Mx=0).

#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/tire_contact.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

vdsim::ContactArray flat_contacts(double mu = 1.0) {
    vdsim::ContactArray c;
    for (auto& p : c) { p.is_valid = true; p.normal = vdsim::Vec3::UnitZ();
                        p.mu_long = mu; p.mu_lat = mu; }
    return c;
}

vdsim::State roll_state(double vx, double R) {
    vdsim::State s; s.velocity.x() = vx;
    const double w = vx / R;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

}  // namespace

TEST(CamberMigration, ContactOffsetFormula) {
    vdsim::TireParams tp; tp.crown_radius = 0.10;
    const double gamma = 0.05;  // ~2.9 deg
    const auto ck = vdsim::tire_contact_kinematics(20.0, 0.0, 62.5, 4000.0, gamma, tp, 0.32);
    EXPECT_NEAR(ck.contact_dy, 0.10 * std::sin(gamma), 1e-12);
    EXPECT_GT(ck.contact_dy, 0.0);
    // crown_radius=0 -> no migration.
    vdsim::TireParams plain;
    const auto ck0 = vdsim::tire_contact_kinematics(20.0, 0.0, 62.5, 4000.0, gamma, plain, 0.32);
    EXPECT_DOUBLE_EQ(ck0.contact_dy, 0.0);
}

TEST(CamberMigration, OverturningMomentTracksFzAndCamber) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; tp.crown_radius = 0.12;
    vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(roll_state(25.0, vp.wheel_radius_nominal));

    const double gamma = 0.04;
    dyn->set_camber_per_wheel({{gamma, gamma, gamma, gamma}});
    const vdsim::ControlInput u = vdsim::CmdL4{};
    dyn->step(u, flat_contacts(), 0.005);

    const auto Fz = dyn->tire_Fz();
    const auto Mx = dyn->wheel_overturning_moment();
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
        EXPECT_NEAR(Mx[i], Fz[i] * tp.crown_radius * std::sin(gamma), 1e-6)
            << "wheel " << i;
        EXPECT_GT(std::abs(Mx[i]), 1.0);  // physically meaningful, not numerical dust
    }
}

// L/R-opposite camber (as roll-induced camber gain produces) must give opposite-sign
// Mx per side, so that summing into the roll DOF naturally cancels for symmetric roll
// camber. This guards the per-wheel Mx sign that the 14-DOF roll coupling relies on.
// (Tested on L2, which honours external camber; L3 drives camber from its own roll.)
TEST(CamberMigration, OppositeCamberGivesOppositeMx) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; tp.crown_radius = 0.12;
    vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(roll_state(25.0, vp.wheel_radius_nominal));

    const double g = 0.05;
    dyn->set_camber_per_wheel({{+g, -g, +g, -g}});  // ISO roll-camber pattern
    dyn->step(vdsim::ControlInput{vdsim::CmdL4{}}, flat_contacts(), 0.005);

    const auto Mx = dyn->wheel_overturning_moment();
    EXPECT_GT(Mx[vdsim::WHEEL_FL], 0.0);
    EXPECT_LT(Mx[vdsim::WHEEL_FR], 0.0);
    EXPECT_NEAR(Mx[vdsim::WHEEL_FL], -Mx[vdsim::WHEEL_FR], 50.0);  // near-cancel L/R
}

TEST(CamberMigration, ZeroCrownGivesZeroOverturning) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;  // crown_radius = 0
    vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(roll_state(25.0, vp.wheel_radius_nominal));
    dyn->set_camber_per_wheel({{0.05, 0.05, 0.05, 0.05}});
    dyn->step(vdsim::ControlInput{vdsim::CmdL4{}}, flat_contacts(), 0.005);
    for (double m : dyn->wheel_overturning_moment()) EXPECT_DOUBLE_EQ(m, 0.0);
}
