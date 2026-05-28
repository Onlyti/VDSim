// Smoke test: ensure all public headers compile and core types instantiate.

#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"
#include "vdsim/types.hpp"
#include "vdsim/version.hpp"

#include <gtest/gtest.h>

TEST(Compile, TypesInstantiate) {
    vdsim::Vec3 v {1.0, 2.0, 3.0};
    vdsim::Quat q = vdsim::Quat::Identity();
    EXPECT_DOUBLE_EQ(v.x(), 1.0);
    EXPECT_DOUBLE_EQ(q.w(), 1.0);
    EXPECT_EQ(vdsim::NUM_WHEELS, 4);
}

TEST(Compile, StateDefaultsZero) {
    vdsim::State s;
    EXPECT_DOUBLE_EQ(s.position.norm(), 0.0);
    EXPECT_DOUBLE_EQ(s.velocity.norm(), 0.0);
    EXPECT_DOUBLE_EQ(s.yaw_rate(), 0.0);
    EXPECT_DOUBLE_EQ(s.speed_xy(), 0.0);
    EXPECT_DOUBLE_EQ(s.beta(), 0.0);
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
        EXPECT_DOUBLE_EQ(s.wheel_spin[i], 0.0);
    }
}

TEST(Compile, ContactDefaults) {
    vdsim::ContactPoint c;
    EXPECT_FALSE(c.is_valid);
    EXPECT_DOUBLE_EQ(c.normal.z(), 1.0);  // default UnitZ
    EXPECT_DOUBLE_EQ(c.mu_long, 1.0);
    vdsim::ContactArray ca;
    EXPECT_EQ(ca.size(), 4u);
}

TEST(Compile, ControlVariantHoldsL4) {
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.5;
    cmd.steer_angle_wheel = 0.1;
    vdsim::ControlInput input = cmd;
    ASSERT_TRUE(std::holds_alternative<vdsim::CmdL4>(input));
    EXPECT_DOUBLE_EQ(std::get<vdsim::CmdL4>(input).throttle, 0.5);
}

TEST(Compile, ParamsDefaultsPositive) {
    vdsim::VehicleParams vp;
    EXPECT_GT(vp.mass, 0.0);
    EXPECT_GT(vp.wheelbase, 0.0);
    EXPECT_DOUBLE_EQ(vp.cg_to_front + vp.cg_to_rear, vp.wheelbase);

    vdsim::TireParams tp;
    EXPECT_GT(tp.mu_nominal, 0.0);
    EXPECT_GT(tp.cornering_stiffness, 0.0);

    vdsim::SolverParams sp;
    EXPECT_EQ(sp.integrator, vdsim::SolverParams::Integrator::RK4);
    EXPECT_GT(sp.max_substep_dt, 0.0);
}

TEST(Compile, EulerStructDefaults) {
    vdsim::Euler e;
    EXPECT_DOUBLE_EQ(e.roll, 0.0);
    EXPECT_DOUBLE_EQ(e.pitch, 0.0);
    EXPECT_DOUBLE_EQ(e.yaw, 0.0);
}
