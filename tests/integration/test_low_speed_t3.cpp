#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/tire_contact.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

vdsim::ContactArray flat_contacts() {
    vdsim::ContactArray c;
    for (auto& p : c) {
        p.is_valid = true;
        p.normal = vdsim::Vec3::UnitZ();
        p.mu_long = p.mu_lat = 1.0;
    }
    return c;
}

double yaw_rate_after_parking_steer(vdsim::LowSpeedMode mode) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    tp.low_speed_mode = mode;
    vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    vdsim::State s0;
    s0.velocity.x() = 1.0;
    const double w = 1.0 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    dyn->reset(s0);
    vdsim::CmdL4 cmd;
    cmd.steer_angle_wheel = 0.12;
    const auto contacts = flat_contacts();
    for (int i = 0; i < 400; ++i)
        dyn->step(cmd, contacts, 0.01);
    return dyn->state().yaw_rate();
}

}  // namespace

TEST(TireVlow, SlipDenomUsesPerTireEps) {
    vdsim::TireParams tp;
    tp.vlow_speed_eps = 0.5;
    const auto ck = vdsim::tire_contact_kinematics(0.1, 0.0, 10.0, 4000.0, 0.0, tp, 0.32);
    const auto ck_default = vdsim::tire_contact_kinematics(0.1, 0.0, 10.0, 4000.0, 0.0,
                                                           vdsim::TireParams{}, 0.32);
    EXPECT_NE(ck.kappa, ck_default.kappa);
}

TEST(TireVlow, LowSpeedModeYaml) {
    const std::string path = "/tmp/vdsim_tire_vlow_test.yaml";
    vdsim::TireParams tp;
    tp.low_speed_mode = vdsim::LowSpeedMode::TireVlowOnly;
    tp.vlow_speed_eps = 0.25;
    tp.to_yaml(path);
    const auto loaded = vdsim::TireParams::from_yaml(path);
    EXPECT_EQ(loaded.low_speed_mode, vdsim::LowSpeedMode::TireVlowOnly);
    EXPECT_NEAR(loaded.vlow_speed_eps, 0.25, 1e-12);
}

TEST(TireVlow, ParkingSteerDiffersFromKinematicBlend) {
    const double r_blend = yaw_rate_after_parking_steer(vdsim::LowSpeedMode::KinematicBlend);
    const double r_vlow  = yaw_rate_after_parking_steer(vdsim::LowSpeedMode::TireVlowOnly);
    EXPECT_GT(std::abs(r_blend - r_vlow), 1e-4);
}
