#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

constexpr double G = 9.80665;

vdsim::State level_on_flat(double vx, double cg_z, double R) {
    vdsim::State s;
    s.position.z() = cg_z;
    s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

void run_l5_flat(vdsim::IVehicleDynamics& dyn, vdsim::IContactProvider& ground,
                 const vdsim::VehicleParams& vp, const vdsim::CmdL4& cmd,
                 int n, double dt) {
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray contacts;
        ground.query(dyn.state(), vp, contacts);
        dyn.step(u, contacts, dt);
    }
}

struct L5FlatSetup {
    vdsim::VehicleParams vp;
    vdsim::TireParams tp;
    vdsim::SolverParams sp;
    std::unique_ptr<vdsim::IVehicleDynamics> dyn;
    std::unique_ptr<vdsim::IContactProvider> ground;

    explicit L5FlatSetup(bool lugre = false) {
        vp.aero_drag_coeff = 0.0;
        tp.lugre.enabled = lugre;
        sp.stunt_physics = true;
        sp.max_substep_dt = 2e-4;
        sp.max_substeps = 16;
        dyn = vdsim::create_stunt_dof();
        dyn->initialize(vp, tp, sp);
        ground = vdsim::create_flat_ground(0.0, 1.0);
    }

    void reset_level(double vx) {
        dyn->reset(level_on_flat(vx, vp.cg_height, vp.wheel_radius_nominal));
    }
};

}  // namespace

TEST(FlatGround, HubPenetrationNearZeroAtRideHeight) {
    vdsim::VehicleParams vp;
    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::State s = level_on_flat(0.0, vp.cg_height, vp.wheel_radius_nominal);
    vdsim::ContactArray c;
    ground->query(s, vp, c);
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i)
        EXPECT_NEAR(c[i].penetration, 0.0, 0.02) << "wheel " << i;
}

TEST(L5Driving, NoSinkOnFlatCoast) {
    L5FlatSetup h;
    h.reset_level(12.0);
    vdsim::CmdL4 cmd;
    run_l5_flat(*h.dyn, *h.ground, h.vp, cmd, 800, 0.001);
    const double z0 = h.dyn->state().position.z();
    run_l5_flat(*h.dyn, *h.ground, h.vp, cmd, 3200, 0.001);
    EXPECT_NEAR(h.dyn->state().position.z(), z0, 0.03);
    EXPECT_GT(z0, 0.45);
}

TEST(L5Driving, ThrottleAccelerates) {
    L5FlatSetup h;
    h.reset_level(2.0);
    const double vx0 = h.dyn->state().velocity.x();
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.5;
    run_l5_flat(*h.dyn, *h.ground, h.vp, cmd, 3000, 0.001);
    EXPECT_GT(h.dyn->state().velocity.x(), vx0 + 2.0);
}

TEST(L5Driving, BrakeSlows) {
    L5FlatSetup h;
    h.reset_level(15.0);
    const double vx0 = h.dyn->state().velocity.x();
    vdsim::CmdL4 cmd;
    cmd.brake = 0.85;
    run_l5_flat(*h.dyn, *h.ground, h.vp, cmd, 2000, 0.001);
    EXPECT_LT(h.dyn->state().velocity.x(), vx0 * 0.75);
}

TEST(L5Driving, CoastStableAttitude) {
    L5FlatSetup h;
    h.reset_level(12.0);
    vdsim::CmdL4 cmd;
    double max_pitch = 0.0, max_roll = 0.0, max_omega = 0.0;
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < 3000; ++i) {
        vdsim::ContactArray contacts;
        h.ground->query(h.dyn->state(), h.vp, contacts);
        h.dyn->step(u, contacts, 0.001);
        max_pitch = std::max(max_pitch, std::abs(h.dyn->pitch_angle_qs()));
        max_roll  = std::max(max_roll,  std::abs(h.dyn->roll_angle_qs()));
        const auto& om = h.dyn->state().angular_velocity;
        max_omega = std::max(max_omega, om.norm());
    }
    EXPECT_LT(max_pitch, 0.12);
    EXPECT_LT(max_roll, 0.12);
    EXPECT_LT(max_omega, 2.0);
}

TEST(L5Driving, SteerProducesYaw) {
    L5FlatSetup h;
    h.reset_level(10.0);
    vdsim::CmdL4 cmd;
    cmd.steer_angle_wheel = 0.06;
    const double yaw0 = h.dyn->state().yaw();
    run_l5_flat(*h.dyn, *h.ground, h.vp, cmd, 2500, 0.001);
    const double yaw1 = h.dyn->state().yaw();
    const double dyaw = yaw1 - yaw0;
    EXPECT_GT(std::abs(dyaw), 0.08);
    EXPECT_GT(std::abs(h.dyn->state().velocity.y()), 0.10);
}

TEST(L5Driving, SettledVerticalLoadNearWeight) {
    L5FlatSetup h;
    h.reset_level(0.0);
    vdsim::CmdL4 cmd;
    run_l5_flat(*h.dyn, *h.ground, h.vp, cmd, 1500, 0.001);
    const auto fz = h.dyn->tire_Fz();
    const double sum = fz[vdsim::WHEEL_FL] + fz[vdsim::WHEEL_FR]
                     + fz[vdsim::WHEEL_RL] + fz[vdsim::WHEEL_RR];
    EXPECT_NEAR(sum, h.vp.mass * G, 0.15 * h.vp.mass * G);
}

TEST(L5Driving, LuGreFlatDrivingSmoke) {
    L5FlatSetup h(true);
    h.reset_level(8.0);
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.35;
    run_l5_flat(*h.dyn, *h.ground, h.vp, cmd, 2000, 0.001);
    EXPECT_GT(h.dyn->state().velocity.x(), 8.5);
    EXPECT_NEAR(h.dyn->state().position.z(), h.vp.cg_height, 0.06);
}

TEST(L5Driving, UphillCoastSlows) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::SolverParams sp;
    sp.stunt_physics = true;
    sp.max_substep_dt = 2e-4;

    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(level_on_flat(18.0, vp.cg_height, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd;
    run_l5_flat(*dyn, *vdsim::create_flat_ground(0.0, 1.0), vp, cmd, 2500, 0.001);
    const double vx_flat = dyn->state().velocity.x();

    dyn->reset(level_on_flat(18.0, vp.cg_height, vp.wheel_radius_nominal));
    run_l5_flat(*dyn, *vdsim::create_inclined_ground(0.0, 0.08, 0.0, 1.0), vp, cmd, 2500, 0.001);
    EXPECT_LT(dyn->state().velocity.x(), vx_flat * 0.92);
}
