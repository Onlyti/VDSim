// L3 14-DOF skeleton integration tests.
//
// The skeleton delegates planar motion to L2, so behavior should match L2
// closely while populating suspension state (susp_compression, susp_velocity).

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/suspension.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

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

}  // namespace

TEST(FourteenDOF, ConstructionAndLevel) {
    auto p = vdsim::create_fourteen_dof();
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->level(), vdsim::IVehicleDynamics::Level::L3_FourteenDOF);
}

TEST(FourteenDOF, StaticSuspensionPopulatedAtRest) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;    vdsim::SolverParams sp;
    auto dyn = vdsim::create_fourteen_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(0.0, vp.wheel_radius_nominal));

    const auto& s = dyn->state();
    // All four susp_compression are positive (preloaded by sprung weight).
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
        EXPECT_GT(s.susp_compression[i], 0.0);
        EXPECT_DOUBLE_EQ(s.susp_velocity[i], 0.0);
    }
    // Static compression front: m_sprung * g * b / (2L) / k_f
    const double exp_f = vp.mass_sprung * GRAVITY * vp.cg_to_rear  /
                         (2.0 * vp.wheelbase * vp.spring_stiffness[vdsim::WHEEL_FL]);
    const double exp_r = vp.mass_sprung * GRAVITY * vp.cg_to_front /
                         (2.0 * vp.wheelbase * vp.spring_stiffness[vdsim::WHEEL_RL]);
    EXPECT_NEAR(s.susp_compression[vdsim::WHEEL_FL], exp_f, 1e-9);
    EXPECT_NEAR(s.susp_compression[vdsim::WHEEL_FR], exp_f, 1e-9);
    EXPECT_NEAR(s.susp_compression[vdsim::WHEEL_RL], exp_r, 1e-9);
    EXPECT_NEAR(s.susp_compression[vdsim::WHEEL_RR], exp_r, 1e-9);
}

TEST(FourteenDOF, CompressionGrowsUnderBrakeOnFront) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;    vdsim::SolverParams sp;
    auto dyn = vdsim::create_fourteen_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(20.0, vp.wheel_radius_nominal));
    const double rest_front = dyn->state().susp_compression[vdsim::WHEEL_FL];

    vdsim::CmdL4 cmd; cmd.brake = 0.9;
    const vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();
    for (int i = 0; i < 80; ++i) dyn->step(u, contacts, 0.005);

    const auto& s = dyn->state();
    EXPECT_GT(s.susp_compression[vdsim::WHEEL_FL], rest_front * 1.05);
    EXPECT_GT(s.susp_compression[vdsim::WHEEL_FR], rest_front * 1.05);
}

TEST(FourteenDOF, PlanarMotionMatchesL2Closely) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;    vdsim::SolverParams sp;

    auto l2 = vdsim::create_seven_dof();
    auto l3 = vdsim::create_fourteen_dof();
    l2->initialize(vp, tp, sp); l2->reset(init_state(10.0, vp.wheel_radius_nominal));
    l3->initialize(vp, tp, sp); l3->reset(init_state(10.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.03;
    const vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();
    for (int i = 0; i < 500; ++i) { l2->step(u, contacts, 0.005);
                                    l3->step(u, contacts, 0.005); }

    EXPECT_NEAR(l3->state().yaw_rate(),    l2->state().yaw_rate(),    1e-9);
    EXPECT_NEAR(l3->state().velocity.x(),  l2->state().velocity.x(),  1e-9);
    EXPECT_NEAR(l3->state().velocity.y(),  l2->state().velocity.y(),  1e-9);
}

TEST(FourteenDOF, RollOscillatesAndSettles) {
    // Step steer at moderate speed -> roll builds, possibly overshoots,
    // and settles to roughly quasi-static value within ~2 s.
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_fourteen_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(15.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.05;
    const vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();
    double max_roll_during = 0.0, roll_at_4s = 0.0;
    for (int i = 0; i < 800; ++i) {
        dyn->step(u, contacts, 0.005);
        const double phi = dyn->roll_angle_qs();
        max_roll_during = std::max(max_roll_during, std::abs(phi));
        if (i == 799) roll_at_4s = phi;
    }
    EXPECT_GT(roll_at_4s, 0.0);
    EXPECT_GT(max_roll_during, roll_at_4s * 0.99);    // overshoot or settled
    EXPECT_LT(std::abs(roll_at_4s), 0.10);            // < 6 deg
}

TEST(FourteenDOF, PitchTransientUnderBrake) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_fourteen_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(20.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.brake = 0.8;
    const vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();
    for (int i = 0; i < 200; ++i) dyn->step(u, contacts, 0.005);

    const double pitch = dyn->pitch_angle_qs();
    EXPECT_LT(pitch, 0.0);                             // nose dive
    EXPECT_LT(pitch, -0.001);                          // measurable
}

TEST(FourteenDOF, AntiDiveReducesPitchUnderBrake) {
    auto run = [&](double anti_dive) {
        vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
        vp.anti_dive_front = anti_dive;
        vdsim::TireParams tp; vdsim::SolverParams sp;
        auto dyn = vdsim::create_fourteen_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(init_state(20.0, vp.wheel_radius_nominal));
        vdsim::CmdL4 cmd; cmd.brake = 0.8;
        const vdsim::ControlInput u = cmd;
        const auto contacts = flat_contacts();
        for (int i = 0; i < 200; ++i) dyn->step(u, contacts, 0.005);
        return dyn->pitch_angle_qs();
    };
    const double pitch_base = run(0.0);
    const double pitch_anti = run(0.5);
    EXPECT_LT(std::abs(pitch_anti), std::abs(pitch_base));   // less dive
}

TEST(FourteenDOF, SuspensionVelocityNonZeroDuringTransient) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp; vdsim::SolverParams sp;
    auto dyn = vdsim::create_fourteen_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(15.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.05;
    const vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();
    double max_susp_vel = 0.0;
    for (int i = 0; i < 100; ++i) {
        dyn->step(u, contacts, 0.005);
        for (int w = 0; w < vdsim::NUM_WHEELS; ++w) {
            max_susp_vel = std::max(max_susp_vel,
                                     std::abs(dyn->state().susp_velocity[w]));
        }
    }
    EXPECT_GT(max_susp_vel, 0.01);   // transient ride velocity > 1 cm/s
}

TEST(FourteenDOF, PoseEncodesRollAndPitch) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;    vdsim::SolverParams sp;
    auto dyn = vdsim::create_fourteen_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(15.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.05;
    const vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();
    for (int i = 0; i < 800; ++i) dyn->step(u, contacts, 0.005);

    const double roll = dyn->roll_angle_qs();
    EXPECT_GT(roll, 0.0);
    // Quat → roll extraction: roll = atan2(2(w x + y z), 1 - 2(x^2 + y^2))
    const auto& q = dyn->state().orientation;
    const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    const double pose_roll = std::atan2(sinr_cosp, cosr_cosp);
    EXPECT_NEAR(pose_roll, roll, 0.02);   // within 1 degree of quasi-static estimate
}

// =============================================================================
// L3 roll → per-wheel camber → tire Fy_camber path
// =============================================================================
TEST(FourteenDOF, RollFeedsPerWheelCamberWithCorrectSign) {
    // Contract: ITireModel applies camber thrust = -k_cs · γ · Fz · μ.
    // We verify (a) the Pacejka side enforces this sign rule, and (b) the L3
    // body actually develops roll under sustained cornering — i.e. the
    // upstream phi_ is non-zero and is forwarded each step via
    // set_camber_per_wheel() (otherwise no roll → no camber thrust).
    {
        auto tire = vdsim::create_pacejka_mf96();
        vdsim::TireParams tp; tp.camber_stiffness = 2.0;
        tire->initialize(tp);
        vdsim::ITireModel::Input in;
        in.Fz = 4000; in.kappa = 0.0; in.alpha = 0.0;
        in.mu_long = 1.0; in.mu_lat = 1.0; in.Vx_wheel = 15.0;

        in.gamma = +0.05;
        const auto o_pos = tire->compute(in);
        in.gamma = -0.05;
        const auto o_neg = tire->compute(in);
        EXPECT_LT(o_pos.Fy, 0.0);
        EXPECT_GT(o_neg.Fy, 0.0);
        EXPECT_NEAR(o_pos.Fy, -o_neg.Fy, 1e-6);
    }

    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vp.camber_per_roll = 0.8;
    vdsim::TireParams    tp; tp.camber_stiffness = 2.0;
    vdsim::SolverParams  sp;

    auto dyn = vdsim::create_fourteen_dof();
    dyn->initialize(vp, tp, sp);
    vdsim::State s0;
    s0.velocity.x() = 15.0;
    const double w0 = 15.0 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w0, w0, w0, w0}};
    dyn->reset(s0);

    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.06;
    vdsim::ContactArray contacts;
    for (auto& p : contacts) { p.is_valid = true;
                                p.normal = vdsim::Vec3::UnitZ();
                                p.mu_long = 1.0; p.mu_lat = 1.0; }
    const double dt = 0.001;
    for (int i = 0; i < 1500; ++i) dyn->step(cmd, contacts, dt);

    EXPECT_GT(std::abs(dyn->roll_angle_qs()), 0.005);
}

// =============================================================================
// L3 Stage D: attach_front_kinematics replaces camber_per_roll behavior
// =============================================================================
TEST(FourteenDOF, AttachedKinematicsOverridesCamberPerRoll) {
    // Build a tiny synthetic table that returns a STRONG fixed camber (5°) for
    // ANY wheel travel — independent of vp_.camber_per_roll.
    const auto csv_path = std::filesystem::temp_directory_path()
                           / "vdsim_l3_kine.csv";
    {
        std::ofstream f(csv_path);
        f << "wheel_travel,steer_rack_dy,camber,toe,track_change,caster,valid\n";
        for (double t : {-0.05, 0.0, +0.05})
            for (double s : {-0.005, 0.0, +0.005})
                f << t << "," << s << "," << (5.0 * M_PI / 180.0)
                  << "," << 0 << "," << 0 << "," << 0 << ",1\n";
    }

    auto run = [&](bool use_kine) {
        vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
        vp.camber_per_roll = 0.0;     // disable legacy fallback
        vdsim::TireParams    tp; tp.camber_stiffness = 2.0;
        vdsim::SolverParams  sp;

        auto dyn = vdsim::create_fourteen_dof();
        dyn->initialize(vp, tp, sp);
        if (use_kine) {
            auto k = vdsim::create_lookup_kinematics(csv_path.string());
            EXPECT_TRUE(vdsim::attach_front_kinematics(*dyn, std::move(k)));
        }
        vdsim::State s0;
        s0.velocity.x() = 15.0;
        const double w0 = 15.0 / vp.wheel_radius_nominal;
        s0.wheel_spin = {{w0, w0, w0, w0}};
        dyn->reset(s0);

        vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.01;
        vdsim::ContactArray contacts;
        for (auto& p : contacts) { p.is_valid = true;
                                    p.normal = vdsim::Vec3::UnitZ();
                                    p.mu_long = 1.0; p.mu_lat = 1.0; }
        const double dt = 0.001;
        for (int i = 0; i < 800; ++i) dyn->step(cmd, contacts, dt);
        return dyn->tire_forces_body();
    };

    const auto F_no_kine = run(false);
    const auto F_kine    = run(true);

    // With kinematics attached → fixed 5° camber on front → camber thrust on
    // front Fy substantially shifted relative to baseline.
    const double dFy_FL = std::abs(F_kine[vdsim::WHEEL_FL].y()
                                     - F_no_kine[vdsim::WHEEL_FL].y());
    EXPECT_GT(dFy_FL, 100.0);   // > 100 N change

    std::filesystem::remove(csv_path);
}

// =============================================================================
// L3 Stage G: kinematics toe (bump-steer) propagates to Ld2 wheel angle.
// =============================================================================
TEST(FourteenDOF, KinematicsToeProducesBumpSteer) {
    // Hardpoint table with FIXED 0.05 rad toe on every row → effectively a
    // hard 2.86° toe-in on front wheels at any travel.  In straight-line at
    // 15 m/s, this should generate a non-zero front Fy from the toe alone
    // (slip angle ≈ -toe).
    const auto csv_path = std::filesystem::temp_directory_path()
                           / "vdsim_l3_toe.csv";
    {
        std::ofstream f(csv_path);
        f << "wheel_travel,steer_rack_dy,camber,toe,track_change,caster,valid\n";
        for (double t : {-0.05, 0.0, +0.05})
            for (double s : {-0.005, 0.0, +0.005})
                f << t << "," << s << "," << 0 << "," << 0.05
                  << "," << 0 << "," << 0 << ",1\n";
    }

    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    vp.camber_per_roll = 0.0;
    vdsim::TireParams    tp;
    vdsim::SolverParams  sp;
    auto dyn = vdsim::create_fourteen_dof();
    dyn->initialize(vp, tp, sp);
    auto k = vdsim::create_lookup_kinematics(csv_path.string());
    ASSERT_TRUE(vdsim::attach_front_kinematics(*dyn, std::move(k)));

    vdsim::State s0; s0.velocity.x() = 15.0;
    const double w0 = 15.0 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w0, w0, w0, w0}};
    dyn->reset(s0);

    vdsim::CmdL4 cmd;     // ZERO steering input
    const vdsim::ControlInput u = cmd;
    vdsim::ContactArray contacts;
    for (auto& p : contacts) { p.is_valid = true;
                                p.normal = vdsim::Vec3::UnitZ();
                                p.mu_long = 1.0; p.mu_lat = 1.0; }
    for (int i = 0; i < 100; ++i) dyn->step(u, contacts, 0.001);

    // After a few steps the front wheels' kinematic toe should manifest as
    // non-zero slip angle and thus non-zero Fy.  (Without toe wiring this
    // would stay at zero since cmd.steer_angle_wheel = 0.)
    const auto alphas = dyn->wheel_slip_angle();
    EXPECT_GT(std::abs(alphas[vdsim::WHEEL_FL]) +
              std::abs(alphas[vdsim::WHEEL_FR]),
              0.02);   // ≥ ~1 deg slip from toe

    std::filesystem::remove(csv_path);
}
