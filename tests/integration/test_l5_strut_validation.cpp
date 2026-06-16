// L5 spatial-strut phase-C validation (gates phase B). Each test runs the strut
// path (SolverParams::l5_spatial_suspension = true) and checks it against an
// independent reference: a planar model (cross-model), analytic projectile motion
// (jump), the emergent centripetal condition (loop), and the standalone L4 corner
// DAE (suspension camber). Evidence numbers are echoed to stdout.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace {

constexpr double G = 9.80665;

vdsim::State on_flat(double x, double vx, double cg_z, double R) {
    vdsim::State s;
    s.position.x() = x;
    s.position.z() = cg_z;
    s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

vdsim::SolverParams strut_solver() {
    vdsim::SolverParams sp;
    sp.stunt_physics = true;
    sp.l5_spatial_suspension = true;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 16;
    return sp;
}

void run(vdsim::IVehicleDynamics& dyn, vdsim::IContactProvider& ground,
         const vdsim::VehicleParams& vp, const vdsim::CmdL4& cmd, int n, double dt) {
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray c;
        ground.query(dyn.state(), vp, c);
        dyn.step(u, c, dt);
    }
}

std::string kin_path(const char* rel) {
    return std::string(VDSIM_SOURCE_DIR) + "/configs/parts/susp_kinematics/kin/" + rel;
}

}  // namespace

// C1 — Flat-ground cross-model: at low excitation the 6-DOF spatial model must
// reduce to the planar L2 handling (yaw rate, lateral accel) within tolerance.
TEST(L5StrutValidation, FlatCrossModelMatchesL2) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::CmdL4 cmd;
    cmd.steer_angle_wheel = 0.03;
    const double vx = 15.0;

    auto l2 = vdsim::create_seven_dof();
    vdsim::SolverParams sp2;
    l2->initialize(vp, tp, sp2);
    l2->reset(on_flat(0.0, vx, 0.0, vp.wheel_radius_nominal));
    run(*l2, *vdsim::create_flat_ground(0.0, 1.0), vp, cmd, 1500, 0.002);
    const double yr2 = l2->state().yaw_rate();
    const double ay2 = l2->ay_body_est();

    auto l5 = vdsim::create_stunt_dof();
    l5->initialize(vp, tp, strut_solver());
    l5->reset(on_flat(0.0, vx, vp.cg_height, vp.wheel_radius_nominal));
    run(*l5, *vdsim::create_flat_ground(0.0, 1.0), vp, cmd, 1500, 0.002);
    const double yr5 = l5->state().yaw_rate();
    const double ay5 = l5->ay_body_est();

    EXPECT_GT(std::abs(yr2), 0.02);
    EXPECT_NEAR(yr5, yr2, 0.20 * std::abs(yr2));
    EXPECT_NEAR(ay5, ay2, 0.20 * std::abs(ay2) + 0.3);
}

// C2 — Ballistic jump: during the airborne phase the CG must follow projectile
// motion (constant horizontal speed, vertical accel = -g, parabolic z(t)).
TEST(L5StrutValidation, BallisticJumpFollowsProjectile) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, strut_solver());
    dyn->reset(on_flat(10.0, 16.0, vp.cg_height, vp.wheel_radius_nominal));

    auto ramp = vdsim::create_ramp_ground(20.0, 24.0, 0.6, 0.4, 1.0);
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.1;

    // Record the whole run with an airborne flag, then fit only over the LONGEST
    // contiguous airborne segment (the real flight) — landing bounces and the first
    // pre-settle instant also dip Fz briefly, so a naive "all Fz<30" set is polluted.
    std::vector<double> ts, zs, vxs;
    std::vector<char> air;
    double t = 0.0;
    const double dt = 0.001;
    for (int i = 0; i < 5000; ++i) {
        vdsim::ContactArray c;
        ramp->query(dyn->state(), vp, c);
        dyn->step(cmd, c, dt);
        t += dt;
        const auto fz = dyn->tire_Fz();
        const double sum = fz[0] + fz[1] + fz[2] + fz[3];
        const auto& st = dyn->state();
        const vdsim::Vec3 v_world = st.orientation.toRotationMatrix() * st.velocity;
        ts.push_back(t); zs.push_back(st.position.z()); vxs.push_back(v_world.x());
        air.push_back(sum < 30.0 ? 1 : 0);
    }
    std::size_t best_lo = 0, best_len = 0, run_lo = 0, run = 0;
    for (std::size_t i = 0; i < air.size(); ++i) {
        if (air[i]) { if (run == 0) run_lo = i; ++run; if (run > best_len) { best_len = run; best_lo = run_lo; } }
        else run = 0;
    }
    ASSERT_GT(best_len, 200u) << "no sustained airborne phase";

    // Mid-flight window of the longest airborne segment (drop launch/land transients).
    const std::size_t lo = best_lo + best_len / 5, hi = best_lo + 4 * best_len / 5;
    // Least-squares quadratic z(t) = a t^2 + b t + c -> vertical accel 2a = -g.
    double Sx=0,Sx2=0,Sx3=0,Sx4=0,Sz=0,Sxz=0,Sx2z=0; int m=0;
    double vx_min=1e9, vx_max=-1e9;
    for (std::size_t i = lo; i < hi; ++i) {
        const double x = ts[i], z = zs[i];
        Sx+=x; Sx2+=x*x; Sx3+=x*x*x; Sx4+=x*x*x*x;
        Sz+=z; Sxz+=x*z; Sx2z+=x*x*z; ++m;
        vx_min = std::min(vx_min, vxs[i]); vx_max = std::max(vx_max, vxs[i]);
    }
    Eigen::Matrix3d A; A << Sx4,Sx3,Sx2, Sx3,Sx2,Sx, Sx2,Sx,double(m);
    Eigen::Vector3d rhs(Sx2z, Sxz, Sz);
    const Eigen::Vector3d coef = A.colPivHouseholderQr().solve(rhs);
    const double g_fit = -2.0 * coef(0);
    const double vx_drift = (vx_max - vx_min) / std::max(1e-6, std::abs(vx_max));

    EXPECT_NEAR(g_fit, G, 0.05 * G);
    EXPECT_LT(vx_drift, 0.03);
}

// C3 — Loop: the centripetal condition is emergent. Above the critical speed the
// car climbs much further around the vertical loop than below it.
namespace {
double loop_arc(double v0_factor) {
    const double R = 10.0, xc = 50.0, zc = 15.0;
    const double v0 = v0_factor * std::sqrt(5.0 * G * R);
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    auto sp = strut_solver();
    sp.loop_radius = R; sp.loop_center_x = xc; sp.loop_center_z = zc;
    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, sp);
    const double z0 = zc - R - vp.wheel_radius_nominal + vp.cg_height + 0.01;
    dyn->reset(on_flat(xc, v0, z0, vp.wheel_radius_nominal));
    auto loop = vdsim::create_loop_ground(xc, zc, R, 1.2);
    vdsim::CmdL4 cmd; cmd.throttle = 0.5;
    double unwrap = 0.0, peak = 0.0, prev = 0.0; bool have = false;
    for (int i = 0; i < 8000; ++i) {
        vdsim::ContactArray c;
        loop->query(dyn->state(), vp, c);
        dyn->step(cmd, c, 0.001);
        const auto& p = dyn->state().position;
        const double th = std::atan2(p.x() - xc, -(p.z() - zc));
        if (have) {
            double d = th - prev;
            if (d > M_PI) d -= 2.0 * M_PI;
            if (d < -M_PI) d += 2.0 * M_PI;
            unwrap += d;
        } else { unwrap = th; have = true; }
        prev = th;
        peak = std::max(peak, unwrap);
    }
    return peak;
}
}  // namespace

TEST(L5StrutValidation, LoopCriticalSpeedEmergent) {
    const double fast = loop_arc(1.15);
    const double slow = loop_arc(0.70);
    EXPECT_GT(fast, 1.0);             // climbs well past the bottom quadrant
    EXPECT_GT(fast, slow + 0.5);      // and much further than the sub-critical run
}

// C4 — Suspension vs L4: under a brake-dive jounce, the per-wheel camber applied
// by the strut path must match the standalone L4 corner DAE at the observed travel.
TEST(L5StrutValidation, CornerCamberMatchesL4Dae) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, strut_solver());
    auto front = vdsim::mb::SuspensionTopology::from_yaml(kin_path("mp_front_sedan.yaml"));
    auto rear  = vdsim::mb::SuspensionTopology::from_yaml(kin_path("ta_rear_sedan.yaml"));
    vdsim::free_3d_attach_multibody(*dyn, true,  front, true);
    vdsim::free_3d_attach_multibody(*dyn, false, rear,  true);
    dyn->reset(on_flat(0.0, 18.0, vp.cg_height, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd;
    cmd.brake = 0.7;                                  // dive: front compresses, rear extends
    run(*dyn, *vdsim::create_flat_ground(0.0, 1.0), vp, cmd, 1200, 0.001);

    const auto comp = dyn->state().susp_compression;
    const auto cam  = vdsim::free_3d_wheel_camber(*dyn);

    auto dae_camber = [](vdsim::mb::SuspensionTopology& topo, double travel, double sign) {
        auto model = vdsim::mb::create_hard_joint_dae_model(topo);
        vdsim::mb::HardJointCornerState st;
        vdsim::mb::PrescribedCornerMotion mot; mot.travel_z = travel;
        model->initialize(st, mot);
        vdsim::mb::WheelLoad zl;
        for (int i = 0; i < 50; ++i)
            model->step(st, mot, zl, 0.001);          // converge q to the travel
        return sign * model->step(st, mot, zl, 0.0).camber_rad;
    };
    const double cam_fl_ref = dae_camber(front, comp[vdsim::WHEEL_FL], +1.0);
    const double cam_rl_ref = dae_camber(rear,  comp[vdsim::WHEEL_RL], +1.0);

    EXPECT_GT(std::abs(comp[vdsim::WHEEL_FL]), 0.002);   // there was a real jounce
    EXPECT_NEAR(cam[vdsim::WHEEL_FL], cam_fl_ref, 2e-3);
    EXPECT_NEAR(cam[vdsim::WHEEL_RL], cam_rl_ref, 2e-3);
}
