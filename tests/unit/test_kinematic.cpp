// Unit tests for the kinematic bicycle model (Lk-Kinematic).
#include <gtest/gtest.h>

#include <cmath>

#include "vdsim/contact.hpp"
#include "vdsim/interfaces.hpp"

using namespace vdsim;

static ContactArray flat() {
    ContactArray c;
    for (auto& p : c) { p.is_valid = true; p.normal = Vec3::UnitZ(); p.mu_long = 1; p.mu_lat = 1; }
    return c;
}

TEST(Kinematic, NoSlipYawRate) {
    VehicleParams vp; TireParams tp; SolverParams sp;
    auto dyn = create_kinematic();
    dyn->initialize(vp, tp, sp);
    State s; s.velocity = {10.0, 0.0, 0.0}; dyn->reset(s);

    CmdL4 cmd; cmd.steer_angle_wheel = 0.1;
    const ControlInput u = cmd;
    const auto c = flat();
    for (int i = 0; i < 200; ++i) dyn->step(u, c, 0.005);

    const auto& st = dyn->state();
    EXPECT_NEAR(st.vy(), 0.0, 1e-9);                          // no sideslip (kinematic)
    EXPECT_NEAR(st.yaw_rate(), st.vx() * std::tan(0.1) / vp.wheelbase, 1e-6);
    EXPECT_NEAR(st.vx(), 10.0, 0.2);                          // no throttle/drag -> ~const
    EXPECT_EQ(dyn->level(), IVehicleDynamics::Level::Lk_Kinematic);
}

TEST(Kinematic, ThrottleAccelerates) {
    VehicleParams vp; TireParams tp; SolverParams sp;
    auto dyn = create_kinematic();
    dyn->initialize(vp, tp, sp);
    State s; s.velocity = {5.0, 0.0, 0.0}; dyn->reset(s);

    CmdL4 cmd; cmd.throttle = 1.0;
    const ControlInput u = cmd;
    const auto c = flat();
    for (int i = 0; i < 200; ++i) dyn->step(u, c, 0.005);
    EXPECT_GT(dyn->state().vx(), 5.0);
}
