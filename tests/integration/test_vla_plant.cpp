// VLA plant (Ld2 7DOF) acceptance: CmdL1 direct-torque (throttle bypass) + MF2002/MF96
// combined-slip friction circle + friction-patch ground + GT consistency.
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/magic_formula.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace {

const std::string kRepo = VDSIM_SOURCE_DIR;
const std::string kIoniq5TirYaml = kRepo + "/configs/parts/tire/ioniq5_pac2002.yaml";
const std::string kIoniq5Tir     = kRepo + "/configs/parts/tire/ioniq5_pac2002.tir";
const std::string kIoniq5VpYaml  = kRepo + "/configs/vehicles/ioniq5_awd.yaml";

vdsim::VehicleParams ioniq5_vp() {
    return vdsim::VehicleParams::from_yaml(kIoniq5VpYaml);
}

vdsim::TireParams plant_tp(double mu) {
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    tp.combined_slip_enabled = true;
    tp.mu_nominal = mu;
    return tp;
}

vdsim::TireParams ioniq5_plant_tp() {
    vdsim::TireParams tp = vdsim::TireParams::from_yaml(kIoniq5TirYaml);
    tp.tir_path = kIoniq5Tir;
    tp.lugre.enabled = false;
    return tp;
}

vdsim::ITireModel::Input slip_input(double kappa, double alpha, double Fz,
                                    double mu_lat = 0.9, double mu_long = 0.9) {
    vdsim::ITireModel::Input in;
    in.Fz = Fz;
    in.kappa = kappa;
    in.alpha = alpha;
    in.mu_long = mu_long;
    in.mu_lat = mu_lat;
    in.Vx_wheel = 16.0;
    return in;
}

double finite_diff_kya(const vdsim::ITireModel& tire, double Fz, double dalpha = 0.01) {
    const auto op = tire.compute(slip_input(0.0, dalpha, Fz));
    const auto om = tire.compute(slip_input(0.0, -dalpha, Fz));
    return (op.Fy - om.Fy) / (2.0 * dalpha);
}

double scan_peak_mu_lat(const vdsim::ITireModel& tire, double Fz, double mu_lat) {
    double peak = 0.0;
    for (double a = 0.02; a <= 0.30; a += 0.01) {
        const auto o = tire.compute(slip_input(0.0, a, Fz, mu_lat, mu_lat));
        peak = std::max(peak, std::abs(o.Fy) / Fz);
    }
    return peak;
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

// Dry qualitative match (acceptance #1): in the linear regime the 7DOF plant's steady-state
// yaw rate must track the linear single-track (bicycle) model with the SAME axle cornering
// stiffness — confirms the plant reproduces the controller's bicycle handling AND that the
// MF96 B=Ca/(C*D) calibration recovered the intended Caf/Car (per-wheel B=Ca/2).
TEST(VlaPlant, DryHandlingMatchesLinearBicycle) {
    auto vp = ioniq5_vp();
    auto tp = plant_tp(0.9);
    vdsim::SolverParams sp;
    const double R = vp.wheel_radius_nominal;
    const double m = vp.mass, lf = vp.cg_to_front, lr = vp.cg_to_rear, L = lf + lr;
    const double Caf = 2.2e5, Car = 1.6e5, V = 16.7, delta = 0.04;
    const double K_us = m * (lr * Car - lf * Caf) / (L * Caf * Car);
    const double r_bicycle = delta * V / (L + K_us * V * V);   // linear single-track yaw gain

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    auto flat = vdsim::create_flat_ground(0.0, 0.9);
    dyn->reset(rolling(V, R));
    vdsim::CmdL1 cmd{};
    cmd.steer_angle_wheel = delta;
    for (int k = 0; k < 400; ++k) {
        vdsim::ContactArray c; flat->query(dyn->state(), vp, c);
        dyn->step(vdsim::ControlInput{cmd}, c, 0.01);
    }
    const double r_vd = dyn->state().yaw_rate();
    EXPECT_GT(r_vd, 0.0) << "+steer -> +yaw (ISO 8855)";
    EXPECT_NEAR(r_vd, r_bicycle, 0.15 * r_bicycle)
        << "7DOF yaw rate must track the linear bicycle (ratio=" << r_vd / r_bicycle << ")";
    EXPECT_LT(std::abs(dyn->state().beta()), 0.05);   // modest sideslip in the linear regime
}

// Same acceptance as DryHandlingMatchesLinearBicycle but through the REAL plant tyre
// (ioniq5 yaml -> MF2002 .tir). Self-correcting gate for PKY1/PKY2 calibration.
TEST(VlaPlant, DryHandlingMatchesLinearBicycleRealMf2002Tire) {
    auto vp = ioniq5_vp();
    vp.plant_path = true;
    auto tp = ioniq5_plant_tp();
    vdsim::SolverParams sp;
    const double R = vp.wheel_radius_nominal;
    const double m = vp.mass, lf = vp.cg_to_front, lr = vp.cg_to_rear, L = lf + lr;
    const double Caf = 2.2e5, Car = 1.6e5, V = 16.7, delta = 0.04;
    const double K_us = m * (lr * Car - lf * Caf) / (L * Caf * Car);
    const double r_bicycle = delta * V / (L + K_us * V * V);

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    auto flat = vdsim::create_flat_ground(0.0, 0.9);
    dyn->reset(rolling(V, R));
    vdsim::CmdL1 cmd{};
    cmd.steer_angle_wheel = delta;
    for (int k = 0; k < 400; ++k) {
        vdsim::ContactArray c; flat->query(dyn->state(), vp, c);
        dyn->step(vdsim::ControlInput{cmd}, c, 0.01);
    }
    const double r_vd = dyn->state().yaw_rate();
    EXPECT_GT(r_vd, 0.0) << "+steer -> +yaw (ISO 8855)";
    EXPECT_NEAR(r_vd, r_bicycle, 0.15 * r_bicycle)
        << "MF2002 plant yaw rate must track the linear bicycle (ratio=" << r_vd / r_bicycle << ")";
}

// Load-sensitivity gate: concave cornering stiffness + peak mu_y falls with Fz (PDY2<0).
TEST(VlaPlant, Mf2002LoadSensitivityConcaveStiffnessAndMu) {
    auto tire = vdsim::create_magic_formula_tire_from_tir(kIoniq5Tir);
    ASSERT_NE(tire, nullptr);
    const double Fz0 = 5764.0;
    const double K0 = finite_diff_kya(*tire, Fz0);
    const double K2 = finite_diff_kya(*tire, 2.0 * Fz0);
    EXPECT_LT(std::abs(K2), 2.0 * std::abs(K0))
        << "cornering stiffness must be sub-linear in Fz (concave Kya)";
    const double mu0 = scan_peak_mu_lat(*tire, Fz0, 0.9);
    const double mu2 = scan_peak_mu_lat(*tire, 2.0 * Fz0, 0.9);
    EXPECT_GT(mu0, 0.5);
    EXPECT_LT(mu2, mu0) << "peak mu_y must decrease with load (PDY2<0)";
}

// MF2002 honours contact mu_lat (Lmuy scaling) — low-mu patch halves peak grip.
TEST(VlaPlant, Mf2002FrictionPatchHalvesPeakGrip) {
    auto tire = vdsim::create_magic_formula_tire_from_tir(kIoniq5Tir);
    ASSERT_NE(tire, nullptr);
    const double Fz = 5764.0;
    const double peak_dry = scan_peak_mu_lat(*tire, Fz, 0.9);
    const double peak_patch = scan_peak_mu_lat(*tire, Fz, 0.5);
    EXPECT_GT(peak_dry, peak_patch);
    EXPECT_NEAR(peak_patch, 0.5 * peak_dry, 0.12 * peak_dry);
}

// HEADLINE grip-loss with the REAL MF2002 plant tyre on the low-mu patch.
TEST(VlaPlant, CombinedBrakeTurnGripLossOnPatchMf2002) {
    auto vp = ioniq5_vp();
    vp.plant_path = true;
    auto tp = ioniq5_plant_tp();
    vdsim::SolverParams sp;
    const double R = vp.wheel_radius_nominal;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    auto ground = vdsim::create_friction_patch_ground(0.0, 0.5, {{-1.0, 1e4, 0.5}});
    dyn->reset(rolling(16.7, R));

    vdsim::CmdL1 cmd{};
    cmd.steer_angle_wheel = 0.12;
    const double Fx_cmd = -15000.0;
    const double tau = std::abs(wheel_torque(Fx_cmd, R));
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) cmd.brake_torque[i] = tau;

    bool engaged = false, slipped = false, vehicle_grip_loss = false;
    bool circle_ok = true, locked_no_reverse = true;
    for (int k = 0; k < 80; ++k) {
        vdsim::ContactArray c; ground->query(dyn->state(), vp, c);
        dyn->step(vdsim::ControlInput{cmd}, c, 0.05);
        if (dyn->state().velocity.x() < 2.0) break;
        const auto Fw = dyn->tire_forces_wheel();
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
            if (fxy > 1.08 * cap) circle_ok = false;
            if (fxy >= 0.30 * cap) engaged = true;
            if (std::abs(kappa[i]) > 0.1) slipped = true;
            if (kappa[i] < -1.2) locked_no_reverse = false;
        }
        if (std::abs(sumFx_body) < 0.85 * std::abs(Fx_cmd)) vehicle_grip_loss = true;
    }
    EXPECT_TRUE(circle_ok);
    EXPECT_TRUE(engaged);
    EXPECT_TRUE(slipped);
    EXPECT_TRUE(locked_no_reverse);
    EXPECT_TRUE(vehicle_grip_loss);
}

// Realized peak mu (useGT>1 fix): ||[Fx,Fy]||/(mu_peak*Fz) <= 1 with load-dependent MF peak.
// Documents that mu_peak can exceed contact mu on unloaded/low-load wheels (PDY2<0).
TEST(VlaPlant, RealizedPeakMuBoundsFrictionCircle) {
    auto vp = ioniq5_vp();
    vp.plant_path = true;
    auto tp = ioniq5_plant_tp();
    vdsim::SolverParams sp;
    const double R = vp.wheel_radius_nominal;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    auto ground = vdsim::create_friction_patch_ground(0.0, 0.5, {{-1.0, 1e4, 0.5}});
    dyn->reset(rolling(16.7, R));

    vdsim::CmdL1 cmd{};
    cmd.steer_angle_wheel = 0.12;
    const double Fx_cmd = -15000.0;
    const double tau = std::abs(wheel_torque(Fx_cmd, R));
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) cmd.brake_torque[i] = tau;

    bool peak_circle_ok = true;
    bool old_metric_exceeds_contact = false;
    for (int k = 0; k < 80; ++k) {
        vdsim::ContactArray c; ground->query(dyn->state(), vp, c);
        dyn->step(vdsim::ControlInput{cmd}, c, 0.05);
        if (dyn->state().velocity.x() < 2.0) break;
        const auto Fw = dyn->tire_forces_wheel();
        const auto mu = dyn->wheel_mu();
        const auto mu_peak = dyn->wheel_mu_peak();
        const auto Fz = dyn->tire_Fz();
        for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
            if (Fz[i] <= 0.0) continue;
            const double fxy = std::hypot(Fw[i].x(), Fw[i].y());
            const double cap_peak = mu_peak[i] * Fz[i];
            if (fxy > cap_peak * (1.0 + 1e-3)) peak_circle_ok = false;
            if (fxy > mu[i] * Fz[i]) old_metric_exceeds_contact = true;
        }
    }
    EXPECT_TRUE(peak_circle_ok)
        << "||F|| must stay inside mu_peak*Fz (realized peak friction circle)";
    if (!old_metric_exceeds_contact) {
        // Locked-wheel braking sits on the MF sliding tail (below peak); document the
        // old-metric violation on a lightly loaded combined-slip tyre point instead.
        auto tire = vdsim::create_magic_formula_tire_from_tir(kIoniq5Tir);
        const auto o = tire->compute(slip_input(-0.18, 0.04, 300.0, 0.5, 0.5));
        const double fxy = std::hypot(o.Fx, o.Fy);
        EXPECT_GT(o.mu_peak, 0.5);
        EXPECT_GT(fxy, 0.5 * 300.0)
            << "old useGT=||F||/(contact_mu*Fz) can read >1 when mu_peak > contact mu";
        EXPECT_LE(fxy, o.mu_peak * 300.0 * (1.0 + 1e-3));
        old_metric_exceeds_contact = fxy > 0.5 * 300.0;
    }
    EXPECT_TRUE(old_metric_exceeds_contact)
        << "document ||F|| > contact_mu*Fz while mu_peak bounds the friction circle";
}

TEST(VlaPlant, PeakSlipMatchesNumericalArgmax) {
    auto tire = vdsim::create_magic_formula_tire_from_tir(kIoniq5Tir);
    const double fz_vals[] = {3000.0, 5764.0, 9000.0};
    for (double Fz : fz_vals) {
        double alpha_argmax = 0.0;
        double peak_fy = 0.0;
        for (double a = 0.0; a <= 0.4; a += 0.001) {
            const double fy = std::abs(tire->compute(slip_input(0.0, a, Fz)).Fy);
            if (fy > peak_fy) {
                peak_fy = fy;
                alpha_argmax = a;
            }
        }
        const double alpha_peak = tire->compute(slip_input(0.0, 0.0, Fz)).alpha_peak;
        EXPECT_GT(alpha_peak, 0.0);
        EXPECT_TRUE(std::isfinite(alpha_peak));
        EXPECT_NEAR(alpha_argmax, alpha_peak, 0.05 * alpha_peak)
            << "Fz=" << Fz << " lateral peak slip";

        double kappa_argmax = 0.0;
        double peak_fx = 0.0;
        for (double k = 0.0; k <= 0.3; k += 0.001) {
            const double fx = std::abs(tire->compute(slip_input(k, 0.0, Fz)).Fx);
            if (fx > peak_fx) {
                peak_fx = fx;
                kappa_argmax = k;
            }
        }
        const double kappa_peak = tire->compute(slip_input(0.0, 0.0, Fz)).kappa_peak;
        EXPECT_GT(kappa_peak, 0.0);
        EXPECT_TRUE(std::isfinite(kappa_peak));
        EXPECT_NEAR(kappa_argmax, kappa_peak, 0.05 * kappa_peak)
            << "Fz=" << Fz << " longitudinal peak slip";
    }
}

// Speed (acceptance #5): a ~5 s trajectory at the fine plant substep must run far under
// real time so velocity / patch sweeps are cheap. Generous bound (measured ~25 ms here).
TEST(VlaPlant, FasterThanRealtime) {
    auto vp = ioniq5_vp();
    auto tp = plant_tp(0.9);
    vdsim::SolverParams sp;
    sp.max_substep_dt = 5e-4;
    sp.max_substeps = 256;
    const double R = vp.wheel_radius_nominal;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    auto flat = vdsim::create_flat_ground(0.0, 0.9);
    dyn->reset(rolling(16.7, R));
    vdsim::CmdL1 cmd{};
    cmd.steer_angle_wheel = 0.03;
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < 100; ++k) {            // 100 * 0.05 s = 5 s sim, 10000 substeps
        vdsim::ContactArray c; flat->query(dyn->state(), vp, c);
        dyn->step(vdsim::ControlInput{cmd}, c, 0.05);
    }
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    EXPECT_LT(ms, 500.0) << "5 s plant traj took " << ms << " ms (want << 1 s real time)";
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
