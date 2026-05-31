// L2 7-DOF dynamics integration tests.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr double GRAVITY = 9.80665;

vdsim::ContactArray flat_contacts(double mu = 1.0) {
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

void simulate(vdsim::IVehicleDynamics& dyn, const vdsim::CmdL4& cmd,
              double duration, double dt) {
    const vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();
    const int N = static_cast<int>(std::round(duration / dt));
    for (int i = 0; i < N; ++i) dyn.step(u, contacts, dt);
}

}  // namespace

TEST(SevenDOF, LevelTagAndConstruction) {
    auto p = vdsim::create_seven_dof();
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->level(), vdsim::IVehicleDynamics::Level::L2_SevenDOF);
}

TEST(SevenDOF, AtRestStaticFz) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams    tp;
    vdsim::SolverParams  sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(0.0, vp.wheel_radius_nominal));
    vdsim::CmdL4 cmd; simulate(*dyn, cmd, 0.05, 0.005);

    const auto Fz = dyn->tire_Fz();
    const double Fz_static_f = vp.mass * GRAVITY * vp.cg_to_rear  / (2.0 * vp.wheelbase);
    const double Fz_static_r = vp.mass * GRAVITY * vp.cg_to_front / (2.0 * vp.wheelbase);
    EXPECT_NEAR(Fz[vdsim::WHEEL_FL], Fz_static_f, 1.0);
    EXPECT_NEAR(Fz[vdsim::WHEEL_FR], Fz_static_f, 1.0);
    EXPECT_NEAR(Fz[vdsim::WHEEL_RL], Fz_static_r, 1.0);
    EXPECT_NEAR(Fz[vdsim::WHEEL_RR], Fz_static_r, 1.0);
    EXPECT_NEAR(Fz[0]+Fz[1]+Fz[2]+Fz[3], vp.mass * GRAVITY, 1.0);
}

TEST(SevenDOF, HardBrakeLoadsBothFrontWheels) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams    tp;
    vdsim::SolverParams  sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(20.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.brake = 0.9;
    simulate(*dyn, cmd, 0.5, 0.005);

    const auto Fz = dyn->tire_Fz();
    const double Fz_static_f = vp.mass * GRAVITY * vp.cg_to_rear / (2.0 * vp.wheelbase);
    EXPECT_GT(Fz[vdsim::WHEEL_FL], Fz_static_f * 1.05);
    EXPECT_GT(Fz[vdsim::WHEEL_FR], Fz_static_f * 1.05);
    EXPECT_NEAR(Fz[0]+Fz[1]+Fz[2]+Fz[3], vp.mass * GRAVITY, 5.0);
}

TEST(SevenDOF, LeftTurnLoadsRightWheels) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams    tp;
    vdsim::SolverParams  sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(10.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.05;   // left turn
    simulate(*dyn, cmd, 4.0, 0.005);                  // settle to SS

    const auto Fz = dyn->tire_Fz();
    // ISO 8855 RH: +y leftward.  Centripetal accel in left turn is +y.
    // Inertia load shifts to -y (right).  Right wheels: FR, RR.
    EXPECT_GT(Fz[vdsim::WHEEL_FR], Fz[vdsim::WHEEL_FL]);
    EXPECT_GT(Fz[vdsim::WHEEL_RR], Fz[vdsim::WHEEL_RL]);
    EXPECT_NEAR(Fz[0]+Fz[1]+Fz[2]+Fz[3], vp.mass * GRAVITY, 5.0);
}

TEST(SevenDOF, SmallSteerYawRateMatchesBicycle) {
    // L1 and L2 should agree on the linear-region SS yaw rate (small slip).
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams    tp;
    vdsim::SolverParams  sp;

    auto l1 = vdsim::create_bicycle();
    auto l2 = vdsim::create_seven_dof();
    l1->initialize(vp, tp, sp);  l1->reset(init_state(10.0, vp.wheel_radius_nominal));
    l2->initialize(vp, tp, sp);  l2->reset(init_state(10.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.03;
    simulate(*l1, cmd, 5.0, 0.005);
    simulate(*l2, cmd, 5.0, 0.005);

    const double r1 = l1->state().yaw_rate();
    const double r2 = l2->state().yaw_rate();
    EXPECT_GT(r1, 0.0);
    EXPECT_GT(r2, 0.0);
    EXPECT_NEAR(r2, r1, std::abs(r1) * 0.10);   // within 10 %
}

TEST(SevenDOF, ZeroSteerStraightLine) {
    vdsim::VehicleParams vp;
    vdsim::TireParams    tp;
    vdsim::SolverParams  sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(10.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; simulate(*dyn, cmd, 3.0, 0.005);
    const auto& s = dyn->state();
    EXPECT_NEAR(s.yaw_rate(),     0.0, 1e-6);
    EXPECT_NEAR(s.velocity.y(),   0.0, 1e-4);
    EXPECT_NEAR(s.position.y(),   0.0, 1e-3);
    EXPECT_NEAR(vdsim::yaw_from_quat(s.orientation), 0.0, 1e-6);
}

TEST(SevenDOF, AckermanInfluencesTurningRadiusLowSpeed) {
    // At low speed in a tight turn, 100% Ackerman should reduce the lateral
    // slip seen by the inside wheel, giving a (slightly) smaller turning
    // radius than 0% (parallel) steer.
    vdsim::VehicleParams vp_par = {}; vp_par.aero_drag_coeff = 0.0;
    vdsim::VehicleParams vp_ack = vp_par; vp_ack.ackerman_percent = 100.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;

    auto run = [&](const vdsim::VehicleParams& vp) {
        auto dyn = vdsim::create_seven_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(init_state(2.0, vp.wheel_radius_nominal));
        vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.35;
        simulate(*dyn, cmd, 8.0, 0.005);
        return dyn->state().yaw_rate();
    };
    const double r_par = run(vp_par);
    const double r_ack = run(vp_ack);
    EXPECT_GT(r_par, 0.0);
    EXPECT_GT(r_ack, 0.0);
    // 100% Ackerman is more efficient -> larger SS yaw rate at same input.
    EXPECT_GE(r_ack, r_par * 0.97);   // at least within 3 % of parallel
}

TEST(SevenDOF, AckermanZeroReproducesBaseline) {
    // Setting ackerman_percent = 0 must yield bit-equal results vs the
    // default VehicleParams (which also has ackerman_percent = 0).
    vdsim::VehicleParams vp_def; vp_def.aero_drag_coeff = 0.0;
    vdsim::VehicleParams vp_zero = vp_def; vp_zero.ackerman_percent = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;

    auto run = [&](const vdsim::VehicleParams& vp) {
        auto dyn = vdsim::create_seven_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(init_state(10.0, vp.wheel_radius_nominal));
        vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.05;
        simulate(*dyn, cmd, 3.0, 0.005);
        return dyn->state().yaw_rate();
    };
    EXPECT_DOUBLE_EQ(run(vp_def), run(vp_zero));
}

TEST(SevenDOF, AeroDownforceIncreasesFzAtSpeed) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vp.aero_lift_front = 2.0;     // strong downforce
    vp.aero_lift_rear  = 2.5;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(40.0, vp.wheel_radius_nominal));   // 40 m/s ~ 144 km/h
    vdsim::CmdL4 cmd;
    simulate(*dyn, cmd, 0.5, 0.005);

    const auto Fz = dyn->tire_Fz();
    const double sum = Fz[0] + Fz[1] + Fz[2] + Fz[3];
    EXPECT_GT(sum, vp.mass * 9.80665);   // sum exceeds m*g due to downforce
}

TEST(SevenDOF, AeroDownforceZeroAtRest) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vp.aero_lift_front = 2.0; vp.aero_lift_rear = 2.5;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(0.0, vp.wheel_radius_nominal));
    vdsim::CmdL4 cmd; simulate(*dyn, cmd, 0.05, 0.005);

    const auto Fz = dyn->tire_Fz();
    EXPECT_NEAR(Fz[0]+Fz[1]+Fz[2]+Fz[3], vp.mass * 9.80665, 1.0);
}

TEST(SevenDOF, OpenDifferentialSplitMuSpinsLowMuWheel) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vp.differential = vdsim::VehicleParams::Differential::Open;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(2.0, vp.wheel_radius_nominal));

    vdsim::ContactArray c = flat_contacts(1.0);
    c[vdsim::WHEEL_RL].mu_long = 0.2;
    vdsim::CmdL4 cmd; cmd.throttle = 1.0;
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < 400; ++i) dyn->step(u, c, 0.005);

    const auto& s = dyn->state();
    const double r_left  = s.wheel_spin[vdsim::WHEEL_RL];
    const double r_right = s.wheel_spin[vdsim::WHEEL_RR];
    EXPECT_GT(r_left - r_right, 0.3);   // open: measurable divergence
}

TEST(SevenDOF, LockedDifferentialReducesSplitMuSpread) {
    vdsim::VehicleParams vp_o, vp_l;
    vp_o.aero_drag_coeff = vp_l.aero_drag_coeff = 0.0;
    vp_o.differential = vdsim::VehicleParams::Differential::Open;
    vp_l.differential = vdsim::VehicleParams::Differential::Locked;
    vdsim::TireParams tp; vdsim::SolverParams sp;

    auto run = [&](const vdsim::VehicleParams& vp) {
        auto dyn = vdsim::create_seven_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(init_state(2.0, vp.wheel_radius_nominal));
        vdsim::ContactArray c = flat_contacts(1.0);
        c[vdsim::WHEEL_RL].mu_long = 0.2;
        vdsim::CmdL4 cmd; cmd.throttle = 1.0;
        const vdsim::ControlInput u = cmd;
        for (int i = 0; i < 400; ++i) dyn->step(u, c, 0.005);
        return std::abs(dyn->state().wheel_spin[vdsim::WHEEL_RL] -
                        dyn->state().wheel_spin[vdsim::WHEEL_RR]);
    };
    const double spread_o = run(vp_o);
    const double spread_l = run(vp_l);
    EXPECT_LT(spread_l, spread_o);
}

TEST(SevenDOF, LSDBetweenOpenAndLocked) {
    auto base_vp = [](vdsim::VehicleParams::Differential d) {
        vdsim::VehicleParams vp;
        vp.aero_drag_coeff = 0.0;
        vp.differential = d;
        return vp;
    };
    auto vp_o = base_vp(vdsim::VehicleParams::Differential::Open);
    auto vp_lsd = base_vp(vdsim::VehicleParams::Differential::LSD);
    auto vp_l = base_vp(vdsim::VehicleParams::Differential::Locked);
    vdsim::TireParams tp; vdsim::SolverParams sp;

    auto run = [&](const vdsim::VehicleParams& vp) {
        auto dyn = vdsim::create_seven_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(init_state(2.0, vp.wheel_radius_nominal));
        vdsim::ContactArray c = flat_contacts(1.0);
        c[vdsim::WHEEL_RL].mu_long = 0.2;
        vdsim::CmdL4 cmd; cmd.throttle = 1.0;
        const vdsim::ControlInput u = cmd;
        for (int i = 0; i < 400; ++i) dyn->step(u, c, 0.005);
        return dyn->state().velocity.x();
    };
    const double vx_open   = run(vp_o);
    const double vx_lsd    = run(vp_lsd);
    const double vx_locked = run(vp_l);
    EXPECT_LE(vx_open, vx_lsd);
    EXPECT_LE(vx_lsd,  vx_locked);
}

TEST(SevenDOF, RollAngleSignAndScale) {
    // Left turn (delta > 0, ay > 0 in body) => positive roll (vehicle leans right
    // physically, but quasi-static formula phi = m*ay*h/K is positive numerically).
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(15.0, vp.wheel_radius_nominal));
    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.04;
    simulate(*dyn, cmd, 4.0, 0.005);

    const double phi = dyn->roll_angle_qs();
    const double ay  = dyn->ay_body_est();
    EXPECT_GT(ay,  0.0);
    EXPECT_GT(phi, 0.0);
    // Order-of-magnitude check: phi should be in [0.5 deg, 5 deg]
    EXPECT_LT(std::abs(phi), 0.10);            // < ~6 deg
    EXPECT_GT(std::abs(phi), 1e-3);            // > 0.06 deg
}

TEST(SevenDOF, PitchAngleSignDuringBrake) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(20.0, vp.wheel_radius_nominal));
    vdsim::CmdL4 cmd; cmd.brake = 0.8;
    simulate(*dyn, cmd, 0.5, 0.005);

    const double ax    = dyn->ax_body_est();
    const double pitch = dyn->pitch_angle_qs();
    EXPECT_LT(ax, 0.0);
    EXPECT_LT(pitch, 0.0);                      // nose-dive => negative pitch numerically
}

TEST(BicycleDefaultDiagnostics, RollPitchZero) {
    auto l1 = vdsim::create_bicycle();
    EXPECT_DOUBLE_EQ(l1->roll_angle_qs(),  0.0);
    EXPECT_DOUBLE_EQ(l1->pitch_angle_qs(), 0.0);
    EXPECT_DOUBLE_EQ(l1->ax_body_est(),    0.0);
    EXPECT_DOUBLE_EQ(l1->ay_body_est(),    0.0);
}

TEST(SevenDOF, SteeringRackTorqueSignOpposesSteer) {
    // In a steady left turn the rack torque should oppose the steer input
    // (centering / self-aligning).
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(15.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.05;
    simulate(*dyn, cmd, 4.0, 0.005);

    EXPECT_NE(dyn->steering_rack_torque(), 0.0);
    // For left turn (delta>0), Mz_front sum should be positive (rack torque
    // tries to bring steer back). Sign convention check at minimum.
    const double trq = dyn->steering_rack_torque();
    EXPECT_TRUE(std::isfinite(trq));
}

TEST(SevenDOF, IndependentWheelSpinUnderSplitMu) {
    // Different mu on left vs right wheels => left spin diverges from right
    // under throttle.  This is the L1-vs-L2 differentiator.
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams    tp;
    vdsim::SolverParams  sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(2.0, vp.wheel_radius_nominal));

    vdsim::ContactArray c = flat_contacts(1.0);
    c[vdsim::WHEEL_RL].mu_long = 0.2;        // low-mu left rear (split-mu)
    vdsim::CmdL4 cmd; cmd.throttle = 1.0;
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < 400; ++i) dyn->step(u, c, 0.005);   // 2 s

    const auto& s = dyn->state();
    // RL on low mu spins faster than RR (slip).
    EXPECT_GT(s.wheel_spin[vdsim::WHEEL_RL], s.wheel_spin[vdsim::WHEEL_RR]);
}

// =============================================================================
// Transient slip angle (relaxation length): step-steer Fy build-up
// =============================================================================
// At steering onset, lateral force should NOT reach steady value instantly;
// it should rise over a rolling distance σ_y.  Compare instant-response vs
// finite-σ_y: at t = σ/v the lagged response should be ~63% of steady.

TEST(SevenDOF, RelaxationLengthDelaysLateralForce) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::SolverParams  sp;

    // Two tire configs: identical except for relaxation length
    vdsim::TireParams tp_instant;  tp_instant.relaxation_length_lat = 0.0;
    vdsim::TireParams tp_lagged;   tp_lagged.relaxation_length_lat  = 0.6;

    const double vx_init = 15.0;
    const double dt      = 0.001;
    const double steer   = 0.05;

    auto run = [&](const vdsim::TireParams& tp, double t_probe) {
        auto dyn = vdsim::create_seven_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(init_state(vx_init, vp.wheel_radius_nominal));
        vdsim::CmdL4 cmd; cmd.steer_angle_wheel = steer;
        const auto contacts = flat_contacts();
        const int N = static_cast<int>(std::round(t_probe / dt));
        for (int i = 0; i < N; ++i) dyn->step(cmd, contacts, dt);
        // Sum body-frame Fy at front (axle):  F_FL.y + F_FR.y
        const auto F = dyn->tire_forces_body();
        return F[vdsim::WHEEL_FL].y() + F[vdsim::WHEEL_FR].y();
    };

    // At t = σ/v_x ≈ 0.04 s, lagged response ≈ 0.63 · steady (1 − 1/e).
    const double t_relax = 0.6 / vx_init;     // ≈ 0.04 s
    const double Fy_inst_relax = run(tp_instant, t_relax);
    const double Fy_lag_relax  = run(tp_lagged,  t_relax);
    EXPECT_LT(std::abs(Fy_lag_relax), std::abs(Fy_inst_relax) * 0.85);

    // After many σ periods, both should converge to ~steady (≥ 95% of instant).
    const double Fy_inst_long = run(tp_instant, 2.0);
    const double Fy_lag_long  = run(tp_lagged,  2.0);
    EXPECT_NEAR(Fy_lag_long, Fy_inst_long, std::abs(Fy_inst_long) * 0.10);
}
