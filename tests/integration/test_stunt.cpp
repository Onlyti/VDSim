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

TEST(Stunt, L5CoastOnFlatNoSink) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::SolverParams sp;
    sp.stunt_physics = true;
    sp.max_substep_dt = 2e-4;
    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_on_ground(10.0, 12.0, vp.cg_height, vp.wheel_radius_nominal));

    auto ramp = vdsim::create_ramp_ground(200.0, 204.0, 0.6, 0.4, 1.0);
    vdsim::CmdL4 cmd;
    for (int i = 0; i < 2500; ++i) {
        vdsim::ContactArray contacts;
        ramp->query(dyn->state(), vp, contacts);
        dyn->step(cmd, contacts, 0.001);
    }
    const double z0 = dyn->state().position.z();
    for (int i = 0; i < 2000; ++i) {
        vdsim::ContactArray contacts;
        ramp->query(dyn->state(), vp, contacts);
        dyn->step(cmd, contacts, 0.001);
    }
    EXPECT_GT(z0, 0.4);
    EXPECT_NEAR(dyn->state().position.z(), z0, 0.04);
}

TEST(Stunt, JumpLandingNoSink) {
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

    double z_min_late = 1e9;
    const int n = 10000;
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray contacts;
        ramp->query(dyn->state(), vp, contacts);
        dyn->step(cmd, contacts, 0.001);
        if (dyn->state().position.x() > 26.0)
            z_min_late = std::min(z_min_late, dyn->state().position.z());
    }
    EXPECT_LT(z_min_late, 1e8);
    EXPECT_GT(z_min_late, 0.38);
}

TEST(Stunt, FreeLoopCompletesLap) {
    const double R = 10.0;
    const double xc = 50.0;
    const double zc = 15.0;
    const double v_min = std::sqrt(5.0 * G * R);
    const double v0 = 1.15 * v_min;

    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::SolverParams sp;
    sp.stunt_physics = true;
    sp.l5_spatial_suspension = true;
    sp.loop_radius = R;
    sp.loop_center_x = xc;
    sp.loop_center_z = zc;
    sp.max_substep_dt = 2e-4;

    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, sp);
    const double z0 = zc - R + vp.cg_height + 0.005;
    dyn->reset(init_on_ground(xc, v0, z0, vp.wheel_radius_nominal));

    auto loop = vdsim::create_loop_ground(xc, zc, R, 1.2);
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.5;

    double theta_unwrap = 0.0;
    double theta_peak  = 0.0;
    double theta_prev  = 0.0;
    bool have_prev = false;
    const int n = 8000;
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray contacts;
        loop->query(dyn->state(), vp, contacts);
        dyn->step(cmd, contacts, 0.001);
        const auto& p = dyn->state().position;
        const double theta = std::atan2(p.x() - xc, -(p.z() - zc));
        if (have_prev) {
            double d = theta - theta_prev;
            if (d > M_PI) d -= 2.0 * M_PI;
            if (d < -M_PI) d += 2.0 * M_PI;
            theta_unwrap += d;
        } else {
            theta_unwrap = theta;
            have_prev = true;
        }
        theta_prev = theta;
        theta_peak = std::max(theta_peak, theta_unwrap);
    }
    EXPECT_GT(theta_peak, 1.0);
    EXPECT_GT(std::abs(dyn->pitch_angle_qs()), 0.28);
}

TEST(Stunt, StuntLevelTag) {
    auto dyn = vdsim::create_stunt_dof();
    EXPECT_EQ(dyn->level(), vdsim::IVehicleDynamics::Level::L5_Stunt);
}

TEST(Stunt, LoopSlipAngleFrontRearBalanced) {
    const double R = 10.0;
    const double xc = 50.0;
    const double zc = 15.0;
    const double v0 = 1.12 * std::sqrt(5.0 * G * R);

    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::SolverParams sp;
    sp.stunt_physics = true;
    sp.l5_spatial_suspension = true;
    sp.loop_radius = R;
    sp.loop_center_x = xc;
    sp.loop_center_z = zc;
    sp.max_substep_dt = 1e-4;
    sp.max_substeps = 24;

    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, sp);
    const double z0 = zc - R + vp.cg_height + 0.005;
    dyn->reset(init_on_ground(xc, v0, z0, vp.wheel_radius_nominal));

    auto loop = vdsim::create_loop_ground(xc, zc, R, 1.1);
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.5;
    cmd.steer_angle_wheel = 0.05;

    double peak_af = 0.0, peak_ar = 0.0;
    int n = 0;
    for (int i = 0; i < 7000; ++i) {
        vdsim::ContactArray contacts;
        loop->query(dyn->state(), vp, contacts);
        dyn->step(cmd, contacts, 0.001);
        const auto a = dyn->wheel_slip_angle();
        const auto fz = dyn->tire_Fz();
        const double theta = std::atan2(
            dyn->state().position.x() - xc, -(dyn->state().position.z() - zc));
        if (theta < 0.4 || fz[vdsim::WHEEL_FL] < 400.0 || fz[vdsim::WHEEL_RL] < 400.0)
            continue;
        const double af = 0.5 * (std::abs(a[vdsim::WHEEL_FL]) + std::abs(a[vdsim::WHEEL_FR]));
        const double ar = 0.5 * (std::abs(a[vdsim::WHEEL_RL]) + std::abs(a[vdsim::WHEEL_RR]));
        peak_af = std::max(peak_af, af);
        peak_ar = std::max(peak_ar, ar);
        ++n;
    }
    EXPECT_GT(n, 80);
    EXPECT_GT(peak_af, 0.02);
    EXPECT_GT(peak_ar, 0.25 * peak_af);
}

TEST(Stunt, LoopAccelWheelSpinBounded) {
    const double R = 10.0;
    const double xc = 50.0;
    const double zc = 15.0;
    const double v_min = std::sqrt(5.0 * G * R);
    const double v0 = 1.12 * v_min;

    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::SolverParams sp;
    sp.stunt_physics = true;
    sp.l5_spatial_suspension = true;
    sp.loop_radius = R;
    sp.loop_center_x = xc;
    sp.loop_center_z = zc;
    sp.max_substep_dt = 1e-4;
    sp.max_substeps = 24;

    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, sp);
    const double z0 = zc - R + vp.cg_height + 0.005;
    dyn->reset(init_on_ground(xc, v0, z0, vp.wheel_radius_nominal));

    auto loop = vdsim::create_loop_ground(xc, zc, R, 1.1);
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.55;

    const double Rwh = vp.wheel_radius_nominal;
    double max_spin = 0.0;
    double max_v = 0.0;
    for (int i = 0; i < 8000; ++i) {
        vdsim::ContactArray contacts;
        loop->query(dyn->state(), vp, contacts);
        dyn->step(cmd, contacts, 0.001);
        const auto& st = dyn->state();
        const double v = std::hypot(st.velocity.x(), st.velocity.z());
        max_v = std::max(max_v, v);
        for (int w = 0; w < 4; ++w)
            max_spin = std::max(max_spin, std::abs(st.wheel_spin[w]));
    }
    const double spin_cap = std::max(120.0, max_v / Rwh * 2.2 + 15.0);
    EXPECT_LT(max_spin, spin_cap);
}

TEST(Stunt, LoopMidArcKeepsContact) {
    const double R = 10.0;
    const double xc = 50.0;
    const double zc = 15.0;
    const double v_min = std::sqrt(5.0 * G * R);
    const double v0 = 1.15 * v_min;

    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::SolverParams sp;
    sp.stunt_physics = true;
    sp.l5_spatial_suspension = true;
    sp.loop_radius = R;
    sp.loop_center_x = xc;
    sp.loop_center_z = zc;
    sp.max_substep_dt = 1e-4;
    sp.max_substeps = 24;

    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, sp);
    const double z0 = zc - R + vp.cg_height + 0.005;
    dyn->reset(init_on_ground(xc, v0, z0, vp.wheel_radius_nominal));

    auto loop = vdsim::create_loop_ground(xc, zc, R, 1.1);
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.5;

    double theta_peak = 0.0;
    int grounded = 0;
    double r_min = 1e9, r_max = 0.0;
    const int n = 7000;
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray contacts;
        loop->query(dyn->state(), vp, contacts);
        dyn->step(cmd, contacts, 0.001);
        const auto& p = dyn->state().position;
        const double theta = std::atan2(p.x() - xc, -(p.z() - zc));
        theta_peak = std::max(theta_peak, theta);
        const double rad = std::hypot(p.x() - xc, p.z() - zc);
        r_min = std::min(r_min, rad);
        r_max = std::max(r_max, rad);
        const auto Fz = dyn->tire_Fz();
        const double fz = Fz[0] + Fz[1] + Fz[2] + Fz[3];
        if (fz > 500.0) ++grounded;
    }
    EXPECT_GT(theta_peak, 1.35);          // climbs past the 3-o'clock quadrant
    EXPECT_GT(grounded, n / 4);
    // Stays on the loop: the CG circles near the track radius the whole run — it neither
    // collapses toward the centre (falls off) nor bursts outward through the wall.
    EXPECT_GT(r_min, 0.6 * R);
    EXPECT_LT(r_max, R + 0.5);
}
