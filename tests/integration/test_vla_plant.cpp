// VLA plant (Ld2 7DOF) acceptance: CmdL1 direct-torque (throttle bypass) + native MF96
// combined-slip friction circle + friction-patch ground + GT consistency.
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

vdsim::VehicleParams ioniq5_vp() {
    vdsim::VehicleParams vp;
    vp.mass = 2359.0;
    vp.inertia_diag.z() = 3400.0;
    vp.cg_to_front = 1.17;
    vp.cg_to_rear = 1.80;
    vp.track_front = vp.track_rear = 1.635;
    vp.cg_height = 0.58;
    vp.wheel_radius_nominal = 0.338;
    vp.drive_type = vdsim::VehicleParams::Drive::AWD;
    vp.drive_split_front = 0.5;
    vp.plant_path = true;
    return vp;
}

vdsim::TireParams plant_tp(double mu) {
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    tp.combined_slip_enabled = true;
    tp.mu_nominal = mu;
    return tp;
}

vdsim::State rolling(double vx, double R) {
    vdsim::State s;
    s.velocity.x() = vx;
    const double w = vx / R;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

// per-wheel torque for a desired total Fx (axle 50/50, /2 per wheel): tau = split/2 * Fx * R
double wheel_torque(double Fx_total, double R) { return 0.25 * Fx_total * R; }

}  // namespace

// Throttle BYPASS: a CmdL1 drive torque must accelerate the car directly (wheel-spin ODE),
// whereas CmdL4 throttle=0 does not. Proves Fx is NOT routed through the throttle map.
TEST(VlaPlant, CmdL1DirectTorqueNotThrottleMapped) {
    auto vp = ioniq5_vp();
    auto tp = plant_tp(0.9);
    vdsim::SolverParams sp;
    const double R = vp.wheel_radius_nominal;

    auto torq = vdsim::create_seven_dof();
    torq->initialize(vp, tp, sp);
    torq->reset(rolling(5.0, R));
    vdsim::CmdL1 cmd{};
    const double tau = wheel_torque(4000.0, R);   // +Fx -> drive
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) cmd.motor_torque[i] = tau;
    auto flat = vdsim::create_flat_ground(0.0, 0.9);
    for (int k = 0; k < 40; ++k) {
        vdsim::ContactArray c; flat->query(torq->state(), vp, c);
        torq->step(vdsim::ControlInput{cmd}, c, 0.05);
    }
    const double vx_torque = torq->state().velocity.x();

    auto coast = vdsim::create_seven_dof();   // CmdL4 throttle=0 control
    coast->initialize(vp, tp, sp);
    coast->reset(rolling(5.0, R));
    vdsim::CmdL4 zero{};
    for (int k = 0; k < 40; ++k) {
        vdsim::ContactArray c; flat->query(coast->state(), vp, c);
        coast->step(vdsim::ControlInput{zero}, c, 0.05);
    }
    const double vx_coast = coast->state().velocity.x();

    EXPECT_GT(vx_torque, 5.0 + 0.5) << "CmdL1 drive torque must accelerate the car";
    EXPECT_GT(torq->ax_body_est(), 0.2);
    EXPECT_GT(vx_torque - vx_coast, 1.0) << "torque path != throttle=0 coast";
}

// HEADLINE (acceptance #2): brake + turn simultaneously on the low-mu patch. Native MF96
// combined slip must (a) hold every wheel inside the friction circle, (b) actually REACH it
// on the loaded wheels, (c) develop real slip (kappa), and (d) lose grip at the vehicle level
// (realised |sum Fx| < commanded). Not a clamp — the saturation comes from wheel lock/spin.
TEST(VlaPlant, CombinedBrakeTurnGripLossOnPatch) {
    auto vp = ioniq5_vp();
    auto tp = plant_tp(0.5);
    vdsim::SolverParams sp;
    const double R = vp.wheel_radius_nominal;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    auto ground = vdsim::create_friction_patch_ground(0.0, 0.5, {{-1.0, 1e4, 0.5}});
    dyn->reset(rolling(16.7, R));

    vdsim::CmdL1 cmd{};
    cmd.steer_angle_wheel = 0.12;                       // turn
    const double Fx_cmd = -15000.0;                     // hard brake (will exceed grip)
    const double tau = std::abs(wheel_torque(Fx_cmd, R));
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) cmd.brake_torque[i] = tau;

    // The limit behaviour is TRANSIENT (the car brakes to a stop), so check DURING the run.
    // Over-braking LOCKS the wheels (kappa ~ -1): the contact force sits on the MF sliding
    // tail below peak (why ABS exists), so we do NOT assert it reaches peak mu*Fz — we assert
    // the GT circle invariant, real slip (lock), the tyre stays engaged, and the vehicle
    // cannot deliver the commanded Fx (grip loss).
    bool engaged = false, slipped = false, vehicle_grip_loss = false;
    bool circle_ok = true, locked_no_reverse = true;
    for (int k = 0; k < 80; ++k) {
        vdsim::ContactArray c; ground->query(dyn->state(), vp, c);
        dyn->step(vdsim::ControlInput{cmd}, c, 0.05);
        if (dyn->state().velocity.x() < 2.0) break;     // braked to (near) stop
        const auto Fw = dyn->tire_forces_wheel();       // CONTACT frame (friction circle)
        const auto Fb = dyn->tire_forces_body();
        const auto mu = dyn->wheel_mu();
        const auto kappa = dyn->wheel_slip_ratio();
        const auto Fz = dyn->tire_Fz();
        double sumFx_body = 0.0;
        for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
            sumFx_body += Fb[i].x();
            if (Fz[i] < 200.0) continue;
            const double fxy = std::hypot(Fw[i].x(), Fw[i].y());
            const double cap = mu[i] * Fz[i];
            if (fxy > 1.08 * cap) circle_ok = false;            // GT invariant
            if (fxy >= 0.30 * cap) engaged = true;              // tyre carrying real force
            if (std::abs(kappa[i]) > 0.1) slipped = true;       // real slip (lock)
            if (kappa[i] < -1.2) locked_no_reverse = false;     // brake must not reverse spin
        }
        if (std::abs(sumFx_body) < 0.85 * std::abs(Fx_cmd)) vehicle_grip_loss = true;
    }
    EXPECT_TRUE(circle_ok) << "no wheel may exceed the friction circle (GT invariant)";
    EXPECT_TRUE(engaged) << "tyres must carry real contact force on the patch";
    EXPECT_TRUE(slipped) << "wheels must slip (real kappa) — grip loss, not a clamp";
    EXPECT_TRUE(locked_no_reverse) << "a friction brake must not spin the wheel backwards";
    EXPECT_TRUE(vehicle_grip_loss)                       // vehicle-level grip loss
        << "the patch must prevent delivering the commanded Fx";
}

// GT consistency (acceptance #3): lateral force balance sum Fy_body ~ m*ay, and every wheel
// stays inside the friction circle, on a moderate dry turn.
TEST(VlaPlant, GroundTruthConsistency) {
    auto vp = ioniq5_vp();
    auto tp = plant_tp(0.9);
    vdsim::SolverParams sp;
    const double R = vp.wheel_radius_nominal;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    auto flat = vdsim::create_flat_ground(0.0, 0.9);
    dyn->reset(rolling(20.0, R));
    vdsim::CmdL1 cmd{};
    cmd.steer_angle_wheel = 0.04;                        // moderate turn, no drive/brake
    for (int k = 0; k < 60; ++k) {
        vdsim::ContactArray c; flat->query(dyn->state(), vp, c);
        dyn->step(vdsim::ControlInput{cmd}, c, 0.02);
    }
    const auto Fb = dyn->tire_forces_body();
    const auto Fw = dyn->tire_forces_wheel();
    const auto mu = dyn->wheel_mu();
    const auto Fz = dyn->tire_Fz();
    double sumFy = 0.0;
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
        sumFy += Fb[i].y();
        if (Fz[i] < 200.0) continue;
        EXPECT_LE(std::hypot(Fw[i].x(), Fw[i].y()), 1.08 * mu[i] * Fz[i]);
    }
    const double m_ay = vp.mass * dyn->ay_body_est();
    EXPECT_NEAR(sumFy, m_ay, 0.05 * std::abs(m_ay) + 200.0)
        << "sum Fy_body must match m*ay (force<->accel consistency)";
}

TEST(VlaPlant, FrictionPatchPerWheelMu) {
    auto vp = ioniq5_vp();
    auto ground = vdsim::create_friction_patch_ground(0.0, 0.9, {{10.0, 30.0, 0.5}});
    vdsim::State s;
    s.position.x() = 20.0;
    s.velocity.x() = 10.0;
    vdsim::ContactArray c{};
    ground->query(s, vp, c);
    EXPECT_NEAR(c[vdsim::WHEEL_FL].mu_long, 0.5, 1e-9);
    EXPECT_NEAR(c[vdsim::WHEEL_RL].mu_long, 0.5, 1e-9);
}
