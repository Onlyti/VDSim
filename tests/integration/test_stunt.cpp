#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <algorithm>

namespace {

constexpr double G = 9.80665;

vdsim::State init_on_ground(double x, double vx, double cg_z, double R) {
    vdsim::State s;
    s.position.x() = x;
    s.position.z() = cg_z;
    s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

void run_steps(vdsim::IVehicleDynamics& dyn, vdsim::IContactProvider& ground,
               const vdsim::VehicleParams& vp, const vdsim::CmdL4& cmd,
               int n, double dt) {
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray contacts;
        ground.query(dyn.state(), vp, contacts);
        dyn.step(u, contacts, dt);
    }
}

}  // namespace

TEST(Stunt, GradeL2CoastSlowsOnUphill) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::SolverParams sp;
    vdsim::CmdL4 cmd;

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_on_ground(0.0, 20.0, 0.0, vp.wheel_radius_nominal));
    run_steps(*dyn, *vdsim::create_flat_ground(0.0, 1.0), vp, cmd, 400, 0.005);
    const double vx_flat = dyn->state().velocity.x();

    dyn->reset(init_on_ground(0.0, 20.0, 0.0, vp.wheel_radius_nominal));
    run_steps(*dyn, *vdsim::create_inclined_ground(0.0, 0.08, 0.0, 1.0), vp, cmd, 400, 0.005);
    EXPECT_LT(dyn->state().velocity.x(), vx_flat * 0.95);
}

TEST(Stunt, JumpAirborneInterval) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::SolverParams sp;
    sp.stunt_physics = true;
    sp.max_substep_dt = 2e-4;
    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_on_ground(10.0, 15.0, vp.cg_height, vp.wheel_radius_nominal));

    auto ramp = vdsim::create_ramp_ground(20.0, 24.0, 0.6, 0.4, 1.0);
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.1;

    bool airborne = false;
    double z_peak = vp.cg_height;
    const int n = 4000;
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray contacts;
        ramp->query(dyn->state(), vp, contacts);
        dyn->step(cmd, contacts, 0.001);
        z_peak = std::max(z_peak, dyn->state().position.z());
        const auto Fz = dyn->tire_Fz();
        const double sum = Fz[vdsim::WHEEL_FL] + Fz[vdsim::WHEEL_FR]
                         + Fz[vdsim::WHEEL_RL] + Fz[vdsim::WHEEL_RR];
        if (sum < 50.0) airborne = true;
    }
    EXPECT_TRUE(airborne);
    EXPECT_GT(z_peak, vp.cg_height + 0.2);
}

TEST(Stunt, VerticalLoopCompletesLap) {
    const double R = 10.0;
    const double xc = 50.0;
    const double zc = 15.0;
    const double v_min = std::sqrt(5.0 * G * R);
    const double v0 = 1.35 * v_min;

    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::SolverParams sp;
    sp.stunt_physics = true;
    sp.loop_radius = R;
    sp.loop_center_x = xc;
    sp.loop_center_z = zc;
    sp.max_substep_dt = 2e-4;

    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, sp);
    const double z0 = zc - R + vp.cg_height;
    dyn->reset(init_on_ground(xc, v0, z0, vp.wheel_radius_nominal));

    auto loop = vdsim::create_loop_ground(xc, zc, R, 1.0);
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.4;

    double theta_max = 0.0;
    const int n = 10000;
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray contacts;
        loop->query(dyn->state(), vp, contacts);
        dyn->step(cmd, contacts, 0.001);
        theta_max = std::max(theta_max, dyn->pitch_angle_qs());
    }
    EXPECT_GT(theta_max, 5.8);
}

TEST(Stunt, StuntLevelTag) {
    auto dyn = vdsim::create_stunt_dof();
    EXPECT_EQ(dyn->level(), vdsim::IVehicleDynamics::Level::L5_Stunt);
}
