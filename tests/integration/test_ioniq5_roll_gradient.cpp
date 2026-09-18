// Ioniq5 (CarMaker-derived) suspension: conversion provenance + roll-gradient
// analytic cross-check.
//
// configs/vehicles/ioniq5_awd.yaml carries a suspension block converted from the
// IPG CarMaker 12.0.1 dataset "Hyundai_Ioniq_5_CG_based".  Two things are pinned:
//
//  1. The conversion.  Axle roll stiffness must reproduce, from the CarMaker raw
//     numbers rather than from the YAML,
//         K_axle = (k_l + k_r) * (track/2)^2 + c_Stabi * track^2      [N m/rad]
//     with k = spring-table slope * SuspX.Spring.Amplify (motion ratio 1, because
//     SuspX.Kin.0.L.lSpring = 0 -1 0) and c_Stabi = SuspX.Stabi [N/m].  The stabi
//     term follows CarMaker Reference Manual 12.7.3 EQ 58/59: F_l = c*(x_r - x_l),
//     F_r = -F_l, so in pure roll (x_r - x_l = track*phi) the couple is
//     c*track^2*phi.
//
//  2. The roll gradient the L3 (14-DOF) model realises.  Its sprung-mass roll ODE
//     (core/src/fourteen_dof_dynamics.cpp) is
//         Ixx * phi_ddot = M_roll_spring + m_s * ay * h_cg + M_overturn + M_contact
//     and the per-corner springs plus the ARB give M_roll_spring = -K_phi * phi, so
//     in steady state
//         dphi/da_y = m_s * g * h_cg / K_phi_eff       [rad per g]
//     L3 carries the unsprung masses, so the suspension roll rate is in series with
//     the tyre vertical rate on the way to the ground:
//         K_tyre,axle = 2 * Cz * (track/2)^2
//         K_eff,axle  = K_axle * K_tyre,axle / (K_axle + K_tyre,axle)
//     Dropping that series term overstates the roll stiffness by ~27 % here (the
//     springs are stiff enough that 220 kN/m of tyre is not negligible).
//
// Roll centre height does not enter here: L3 takes the full CG height as the roll
// arm.  roll_center_height_* splits geometric vs elastic load transfer on the L2
// (7-DOF) path instead, which this test does not exercise.

#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

namespace {

constexpr double kGravity = 9.80665;
constexpr double kRad2Deg = 57.29577951308232;

const std::string kIoniq5VpYaml =
    std::string(VDSIM_SOURCE_DIR) + "/configs/vehicles/ioniq5_awd.yaml";

// ---- CarMaker Hyundai_Ioniq_5_CG_based, verbatim ------------------------------
constexpr double kCmSpringSlopeFront = 49361.0;   // SuspF.Spring table, N/m
constexpr double kCmSpringSlopeRear  = 54903.0;   // SuspR.Spring table, N/m
constexpr double kCmSpringAmplify    = 0.9;       // SuspX.Spring.Amplify
constexpr double kCmStabiFront       = 16289.0;   // SuspF.Stabi, N/m
constexpr double kCmStabiRear        = 18118.0;   // SuspR.Stabi, N/m
constexpr double kCmTrack            = 1.634;     // 2 * Wheel.fl.pos.y (0.817)

double carmaker_axle_roll_stiffness(double spring_slope, double stabi) {
    const double k = spring_slope * kCmSpringAmplify;               // wheel rate
    const double half = 0.5 * kCmTrack;
    return 2.0 * k * half * half + stabi * kCmTrack * kCmTrack;
}

vdsim::ContactArray flat_contacts(double mu = 1.0) {
    vdsim::ContactArray c;
    for (auto& p : c) {
        p.is_valid = true;
        p.normal   = vdsim::Vec3::UnitZ();
        p.mu_long  = mu;
        p.mu_lat   = mu;
    }
    return c;
}

vdsim::State initial_state(double vx, double wheel_radius) {
    vdsim::State s;
    s.velocity.x() = vx;
    const double w = (wheel_radius > 0.0) ? vx / wheel_radius : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

}  // namespace

TEST(Ioniq5CarMakerSuspension, AxleRollStiffnessMatchesCarMakerConversion) {
    const auto vp = vdsim::VehicleParams::from_yaml(kIoniq5VpYaml);

    const double kf_expected = carmaker_axle_roll_stiffness(kCmSpringSlopeFront, kCmStabiFront);
    const double kr_expected = carmaker_axle_roll_stiffness(kCmSpringSlopeRear,  kCmStabiRear);

    // 0.5 %: the YAML rounds the converted values and uses track 1.635 (CarMaker
    // wheel centres are at +-0.817, i.e. 1.634).
    EXPECT_NEAR(vdsim::axle_roll_stiffness(vp, 0), kf_expected, 0.005 * kf_expected);
    EXPECT_NEAR(vdsim::axle_roll_stiffness(vp, 1), kr_expected, 0.005 * kr_expected);

    // Mass bookkeeping: sprung + unsprung must close on the total.
    const double m_unsprung = vp.unsprung_mass[vdsim::WHEEL_FL] + vp.unsprung_mass[vdsim::WHEEL_FR]
                            + vp.unsprung_mass[vdsim::WHEEL_RL] + vp.unsprung_mass[vdsim::WHEEL_RR];
    EXPECT_NEAR(vp.mass_sprung + m_unsprung, vp.mass, 1.0);
}

TEST(Ioniq5CarMakerSuspension, L3RollGradientMatchesClosedForm) {
    auto vp = vdsim::VehicleParams::from_yaml(kIoniq5VpYaml);
    vp.aero_drag_coeff = 0.0;          // hold vx over the run
    vdsim::TireParams tp;
    tp.lugre.enabled = false;

    auto dyn = vdsim::create_fourteen_dof();
    dyn->initialize(vp, tp, vdsim::SolverParams{});
    dyn->reset(initial_state(20.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd;
    cmd.steer_angle_wheel = 0.05;      // ~2.9 deg left, ay ~ 0.6 g
    const vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();
    for (int i = 0; i < 1600; ++i) {   // 8 s at 200 Hz -> roll transient settled
        dyn->step(u, contacts, 0.005);
    }

    const double ay   = dyn->ay_body_est();
    const double roll = dyn->roll_angle_qs();
    ASSERT_GT(ay, 1.0) << "steer must produce a measurable lateral acceleration";
    EXPECT_GT(roll, 0.0) << "+steer (left) -> +phi (left side up), ISO 8855";

    const double half_f = 0.5 * vp.track_front, half_r = 0.5 * vp.track_rear;
    const double k_tyre_f = 2.0 * tp.tire_vertical_stiffness * half_f * half_f;
    const double k_tyre_r = 2.0 * tp.tire_vertical_stiffness * half_r * half_r;
    const double k_f = vdsim::axle_roll_stiffness(vp, 0);
    const double k_r = vdsim::axle_roll_stiffness(vp, 1);
    const double k_eff = k_f * k_tyre_f / (k_f + k_tyre_f) + k_r * k_tyre_r / (k_r + k_tyre_r);
    const double expected_deg_per_g = vp.mass_sprung * kGravity * vp.cg_height / k_eff * kRad2Deg;
    const double measured_deg_per_g = roll * kRad2Deg / (ay / kGravity);

    EXPECT_NEAR(measured_deg_per_g, expected_deg_per_g, 0.05 * expected_deg_per_g)
        << "roll gradient " << measured_deg_per_g << " deg/g vs closed form "
        << expected_deg_per_g << " deg/g (ay = " << ay << " m/s^2)";
}
