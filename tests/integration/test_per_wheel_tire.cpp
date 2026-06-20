#include <gtest/gtest.h>

#include <cmath>

#include "vdsim/interfaces.hpp"

namespace {

vdsim::ContactArray valid_contacts() {
    vdsim::ContactArray c;
    for (auto& p : c) {
        p.is_valid = true;
        p.normal = vdsim::Vec3(0.0, 0.0, 1.0);
        p.mu_long = 1.0;
        p.mu_lat = 1.0;
    }
    return c;
}

vdsim::TireParams grip_tire(double mu, double calpha) {
    vdsim::TireParams tp;
    tp.mu_nominal = mu;
    tp.D_lat = mu;
    tp.D_long = mu;
    tp.cornering_stiffness = calpha;
    tp.lugre.enabled = false;
    return tp;
}

}  // namespace

TEST(PerWheelTire, L2FrontLeftRightDifferentGrip) {
    vdsim::VehicleParams vp;
    vp.mass = 1500.0;
    vp.wheelbase = 2.7;
    vp.cg_to_front = 1.2;
    vp.cg_to_rear = 1.5;
    vp.wheel_radius_nominal = 0.32;

    const vdsim::TireSetup ts(
        grip_tire(1.0, 90000.0),
        grip_tire(0.35, 28000.0),
        grip_tire(0.9, 85000.0),
        grip_tire(0.9, 85000.0));
    vdsim::SolverParams sp;

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, ts, sp);
    vdsim::State s0;
    s0.velocity.x() = 20.0;
    s0.wheel_spin = vdsim::free_roll_wheel_spin(vp, ts, 20.0);
    dyn->reset(s0);

    vdsim::CmdL4 cmd;
    cmd.steer_angle_wheel = 0.08;
    const auto contacts = valid_contacts();
    for (int i = 0; i < 400; ++i)
        dyn->step(cmd, contacts, 0.005);

    const auto F = dyn->tire_forces_body();
    EXPECT_GT(std::abs(F[vdsim::WHEEL_FL].y()),
              std::abs(F[vdsim::WHEEL_FR].y()) * 1.3)
        << "FL should generate more lateral force than FR with higher grip";
}
