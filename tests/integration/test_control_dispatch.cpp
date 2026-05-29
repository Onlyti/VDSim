// L1/L2/L3 ControlInput variant dispatch — bicycle and L2 should accept
// each variant and produce non-zero motion when the lowered L4 command is non-zero.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

vdsim::ContactArray flat(double mu = 1.0) {
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

}  // namespace

TEST(ControlDispatch, BicycleHandlesCmdL2Drive) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_bicycle();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(5.0, vp.wheel_radius_nominal));

    vdsim::CmdL2 cmd; cmd.drive_torque = 240.0;   // > 0 -> accel
    vdsim::ControlInput u = cmd;
    for (int i = 0; i < 200; ++i) dyn->step(u, flat(), 0.005);
    EXPECT_GT(dyn->state().velocity.x(), 5.0);
}

TEST(ControlDispatch, BicycleHandlesCmdL3BrakeNegativeFx) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_bicycle();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(20.0, vp.wheel_radius_nominal));

    vdsim::CmdL3 cmd; cmd.Fx_total = -4500.0;     // negative -> brake
    vdsim::ControlInput u = cmd;
    for (int i = 0; i < 100; ++i) dyn->step(u, flat(), 0.005);
    EXPECT_LT(dyn->state().velocity.x(), 20.0);
}

TEST(ControlDispatch, SevenDOFHandlesCmdL1PerWheelTorque) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(5.0, vp.wheel_radius_nominal));

    vdsim::CmdL1 cmd;
    cmd.motor_torque = {{0.0, 0.0, 150.0, 150.0}};   // rear drive
    vdsim::ControlInput u = cmd;
    for (int i = 0; i < 200; ++i) dyn->step(u, flat(), 0.005);
    EXPECT_GT(dyn->state().velocity.x(), 5.0);
}

TEST(ControlDispatch, NaNInputSanitizedNoCrash) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_bicycle();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(10.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd;
    cmd.throttle = std::numeric_limits<double>::quiet_NaN();
    cmd.brake    = std::numeric_limits<double>::infinity();
    cmd.steer_angle_wheel = std::numeric_limits<double>::quiet_NaN();
    vdsim::ControlInput u = cmd;
    for (int i = 0; i < 10; ++i) dyn->step(u, flat(), 0.005);
    EXPECT_TRUE(std::isfinite(dyn->state().velocity.x()));
}

TEST(ControlDispatch, BicycleFallbackOnHigherLevelInput) {
    // L5+ should be treated as zero (no converter present).
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_bicycle();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(15.0, vp.wheel_radius_nominal));

    vdsim::CmdL5 cmd; cmd.ax_target = 3.0;            // ignored
    vdsim::ControlInput u = cmd;
    const double vx0 = dyn->state().velocity.x();
    for (int i = 0; i < 50; ++i) dyn->step(u, flat(), 0.005);
    // Pure drag coast, no acceleration applied.
    EXPECT_LE(dyn->state().velocity.x(), vx0);
}
