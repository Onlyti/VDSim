#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
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

// ---------------------------------------------------------------------------
// Spatial strut (l5_spatial_suspension) — opt-in B1 path. Isolated from the
// penalty-path tests above; default flag stays off so those remain byte-stable.
// ---------------------------------------------------------------------------
struct L5StrutSetup {
    vdsim::VehicleParams vp;
    vdsim::TireParams tp;
    vdsim::SolverParams sp;
    std::unique_ptr<vdsim::IVehicleDynamics> dyn;
    std::unique_ptr<vdsim::IContactProvider> ground;

    L5StrutSetup() {
        vp.aero_drag_coeff = 0.0;
        tp.lugre.enabled = false;
        sp.stunt_physics = true;
        sp.l5_spatial_suspension = true;
        sp.max_substep_dt = 2e-4;
        sp.max_substeps = 16;
        dyn = vdsim::create_stunt_dof();
        dyn->initialize(vp, tp, sp);
        ground = vdsim::create_flat_ground(0.0, 1.0);
    }
    void reset_level(double vx) {
        dyn->reset(level_on_flat(vx, vp.cg_height, vp.wheel_radius_nominal));
    }
    void run(const vdsim::CmdL4& cmd, int n, double dt) {
        run_l5_flat(*dyn, *ground, vp, cmd, n, dt);
    }
};

TEST(L5Strut, SettlesToStaticEquilibrium) {
    L5StrutSetup h;
    h.reset_level(0.0);
    vdsim::CmdL4 cmd;
    h.run(cmd, 3000, 0.001);
    const auto& s = h.dyn->state();
    // Tire vertical loads sum to the full vehicle weight (tire carries sprung +
    // unsprung; suspension carries only the sprung corner).
    const auto fz = h.dyn->tire_Fz();
    const double sum = fz[0] + fz[1] + fz[2] + fz[3];
    EXPECT_NEAR(sum, h.vp.mass * G, 0.12 * h.vp.mass * G);
    // comp=0 is the static ride position by preload construction.
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i)
        EXPECT_NEAR(s.susp_compression[i], 0.0, 0.012) << "corner " << i;
    // Body sits just below design ride height by the tire static deflection.
    EXPECT_GT(s.position.z(), 0.50);
    EXPECT_LT(s.position.z(), h.vp.cg_height + 0.005);
    EXPECT_LT(std::abs(h.dyn->pitch_angle_qs()), 0.03);
    EXPECT_LT(std::abs(h.dyn->roll_angle_qs()), 0.03);
}

TEST(L5Strut, NoSinkOnFlatCoast) {
    L5StrutSetup h;
    h.reset_level(12.0);
    vdsim::CmdL4 cmd;
    h.run(cmd, 1000, 0.001);
    const double z0 = h.dyn->state().position.z();
    h.run(cmd, 3000, 0.001);
    EXPECT_NEAR(h.dyn->state().position.z(), z0, 0.02);
}

TEST(L5Strut, ThrottleAccelerates) {
    L5StrutSetup h;
    h.reset_level(2.0);
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.5;
    h.run(cmd, 3000, 0.001);
    EXPECT_GT(h.dyn->state().velocity.x(), 4.0);
}

TEST(L5Strut, SteerProducesYaw) {
    L5StrutSetup h;
    h.reset_level(10.0);
    vdsim::CmdL4 cmd;
    cmd.steer_angle_wheel = 0.06;
    const double yaw0 = h.dyn->state().yaw();
    h.run(cmd, 2500, 0.001);
    EXPECT_GT(std::abs(h.dyn->state().yaw() - yaw0), 0.08);
}

// Heave ride frequency: settle, then inject a vertical impulse and measure the
// sprung-mass oscillation. Analytic quarter-car heave uses the tire spring in
// series with the corner spring: k_eff = 4 k_s k_t / (k_s + k_t) over m_sprung.
TEST(L5Strut, HeaveRideFrequencyMatchesQuarterCar) {
    L5StrutSetup h;
    h.reset_level(0.0);
    vdsim::CmdL4 cmd;
    h.run(cmd, 3000, 0.001);                       // reach static equilibrium
    const double z_eq = h.dyn->state().position.z();

    vdsim::State s = h.dyn->state();
    s.velocity.z() = -0.45;                         // downward heave impulse
    h.dyn->reset(s);

    const double dt = 0.0005;
    double t = 0.0, prev = 0.0, t_cross = -1.0;
    for (int i = 0; i < 4000; ++i) {                // 2 s
        vdsim::ContactArray c;
        h.ground->query(h.dyn->state(), h.vp, c);
        h.dyn->step(cmd, c, dt);
        t += dt;
        const double d = h.dyn->state().position.z() - z_eq;
        if (t > 0.05 && prev < 0.0 && d >= 0.0) { t_cross = t; break; }
        prev = d;
    }
    ASSERT_GT(t_cross, 0.0) << "no heave restoring crossing detected";

    const double ks = h.vp.spring_stiffness[0];
    const double kt = h.tp.tire_vertical_stiffness;
    const double k_eff = 4.0 * ks * kt / (ks + kt);
    const double wn = std::sqrt(k_eff / h.vp.mass_sprung);
    const double half_period = M_PI / wn;           // first return-to-eq time
    EXPECT_NEAR(t_cross, half_period, 0.30 * half_period);
}

// ---------------------------------------------------------------------------
// B2 — per-corner L4 corner DAE on the strut path: prescribed strut travel ->
// WheelPose toe/camber -> tire. Inactive unless a DAE is attached.
// ---------------------------------------------------------------------------
namespace {
std::string kin_path(const char* rel) {
    return std::string(VDSIM_SOURCE_DIR) + "/configs/parts/susp_kinematics/kin/" + rel;
}
void attach_corner_dae(vdsim::IVehicleDynamics& dyn) {
    auto front = vdsim::mb::SuspensionTopology::from_yaml(kin_path("mp_front_sedan.yaml"));
    auto rear  = vdsim::mb::SuspensionTopology::from_yaml(kin_path("ta_rear_sedan.yaml"));
    vdsim::free_3d_attach_multibody(dyn, true,  front, true);
    vdsim::free_3d_attach_multibody(dyn, false, rear,  true);
}
}  // namespace

TEST(L5StrutDae, AttachReportsEnabled) {
    L5StrutSetup h;
    attach_corner_dae(*h.dyn);
    EXPECT_TRUE(vdsim::free_3d_mb_dynamics_enabled(*h.dyn, 0));
    EXPECT_TRUE(vdsim::free_3d_mb_dynamics_enabled(*h.dyn, 1));
}

TEST(L5StrutDae, SettleStillHoldsWeight) {
    L5StrutSetup h;
    attach_corner_dae(*h.dyn);
    h.reset_level(0.0);
    vdsim::CmdL4 cmd;
    h.run(cmd, 3000, 0.001);
    const auto fz = h.dyn->tire_Fz();
    const double sum = fz[0] + fz[1] + fz[2] + fz[3];
    EXPECT_NEAR(sum, h.vp.mass * G, 0.12 * h.vp.mass * G);
    EXPECT_TRUE(std::isfinite(h.dyn->state().position.z()));
    EXPECT_LT(std::abs(h.dyn->roll_angle_qs()), 0.05);
}

// The corner kinematics (bump-steer toe + camber gain) must change the handling
// response: a steady steer with the DAE attached diverges from the DAE-off run.
TEST(L5StrutDae, CornerKinematicsAlterHandling) {
    vdsim::CmdL4 cmd;
    cmd.steer_angle_wheel = 0.05;

    L5StrutSetup off;
    off.reset_level(14.0);
    off.run(cmd, 2500, 0.001);
    const double vy_off  = off.dyn->state().velocity.y();
    const double yaw_off = off.dyn->state().yaw();

    L5StrutSetup on;
    on.vp.camber_per_roll = 0.0;
    attach_corner_dae(*on.dyn);
    on.reset_level(14.0);
    on.run(cmd, 2500, 0.001);
    const double vy_on  = on.dyn->state().velocity.y();
    const double yaw_on = on.dyn->state().yaw();

    EXPECT_TRUE(std::isfinite(vy_on) && std::isfinite(yaw_on));
    const double dvy  = std::abs(vy_on - vy_off);
    const double dyaw = std::abs(yaw_on - yaw_off);
    EXPECT_GT(dvy + dyaw, 1e-3) << "DAE toe/camber did not affect handling";
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
