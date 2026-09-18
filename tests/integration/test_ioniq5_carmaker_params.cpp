// Ioniq5 (CarMaker-derived) vehicle block: conversion provenance.
//
// Companion to test_ioniq5_roll_gradient.cpp, which pins the suspension roll
// path.  This file pins the rest of what configs/vehicles/ioniq5_awd.yaml takes
// from the IPG CarMaker 12.0.1 dataset "Hyundai_Ioniq_5_CG_based": mass
// properties, aerodynamics, the BEV driveline and the steering / anti-geometry
// that come out of the suspension kinematics table.
//
// Every expectation is recomputed here from the CarMaker raw numbers, never
// from the YAML, so editing a YAML value without redoing the conversion fails.
// Equation numbers refer to the CarMaker 12.0.1 Reference Manual:
//   EQ 73     Linear2D kinematics, row = [offset  d/dq0  d/dq2],
//             q0 = wheel compression, q2 = steering coordinate
//   EQ 83/84  aerodynamic forces and moments, SAE J1594 frame:
//             cL positive = lift upward, cPM positive = nose up,
//             reference length l = Aero.lReference

#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

const std::string kIoniq5VpYaml =
    std::string(VDSIM_SOURCE_DIR) + "/configs/vehicles/ioniq5_awd.yaml";

// ---- CarMaker Hyundai_Ioniq_5_CG_based, verbatim ------------------------------
// Body / corners
constexpr double kCmBodyMass   = 2100.8;          // Body.mass
constexpr double kCmBodyIxx    = 749.84;          // Body.I
constexpr double kCmBodyIyy    = 2732.2;
constexpr double kCmBodyIzz    = 2900.3;
constexpr double kCmBodyPosX   = 2.635;           // Body.pos
constexpr double kCmBodyPosZ   = 0.654;
constexpr double kCmCarrierMassF = 44.959;        // WheelCarrier.fl.mass
constexpr double kCmCarrierMassR = 29.638;        // WheelCarrier.rl.mass
constexpr double kCmWheelMassF   = 29.161;        // Wheel.fl.mass
constexpr double kCmWheelMassR   = 25.357;        // Wheel.rl.mass
constexpr double kCmWheelPosXF   = 3.79;          // Wheel.fl.pos
constexpr double kCmWheelPosXR   = 0.82;          // Wheel.rl.pos
constexpr double kCmWheelPosZ    = 0.298;         // Wheel.X.pos.z = loaded radius
constexpr double kCmTrack        = 1.634;         // 2 * Wheel.fl.pos.y (0.817)

// Aero
constexpr double kCmAeroAx   = 2.594;             // Aero.Ax
constexpr double kCmAeroLRef = 4.635;             // Aero.lReference
constexpr double kCmCd       = 0.288;             // Aero.Coeff, tau = 0, cD
constexpr double kCmCl       = 0.149;             // Aero.Coeff, tau = 0, cL
constexpr double kCmCpm      = -0.002;            // Aero.Coeff, tau = 0, cPM

// Driveline (PowerTrain.Kind = BEV, nMotor = 2)
constexpr double kCmMotorTrqMax = 255.0;          // PowerTrain.Motor.Mot.Trq_max
constexpr int    kCmNMotor      = 2;              // PowerTrain.nMotor
constexpr double kCmMotorRatio  = 2.263;          // PowerTrain.Motor.Ratio
constexpr double kCmDiffRatio   = 4.706;          // PowerTrain.DL.FDiff.i
constexpr double kCmMotorInertia   = 0.002;       // PowerTrain.Motor.I
constexpr double kCmGearBoxIIn     = 0.001;       // PowerTrain.GearBoxM.I_in
constexpr double kCmGearBoxIOut    = 0.016;       // PowerTrain.GearBoxM.I_out
constexpr double kCmDiffIIn        = 0.001;       // PowerTrain.DL.FDiff.I_in

// Steering / suspension kinematics (SuspF|R.Kin.0.L.*, EQ 73)
constexpr double kCmRack2StWhl = 72.314;          // Steering.Rack2StWhl
constexpr double kCmRzPerRack  = 5.0;             // SuspF.Kin.0.L.rz, d/dq2
constexpr double kCmTxPerTzF   = -0.019;          // SuspF.Kin.0.L.tx, d/dq0
constexpr double kCmTxPerTzR   = -0.003;          // SuspR.Kin.0.L.tx, d/dq0
constexpr double kCmRyPerTzF   = -0.602;          // SuspF.Kin.0.L.ry, d/dq0
constexpr double kCmRxPerTzF   =  0.628;          // SuspF.Kin.0.L.rx, d/dq0
constexpr double kCmRxPerTzR   =  0.314;          // SuspR.Kin.0.L.rx, d/dq0

double corner_mass_front() { return kCmCarrierMassF + kCmWheelMassF; }
double corner_mass_rear()  { return kCmCarrierMassR + kCmWheelMassR; }
double unsprung_total()    { return 2.0 * (corner_mass_front() + corner_mass_rear()); }
double total_mass()        { return kCmBodyMass + unsprung_total(); }

}  // namespace

// The sprung body and the four corners must close on the YAML mass, CG and
// sprung inertia.  inertia_diag is Body.I verbatim: params.hpp defines it as the
// inertia "of sprung", and the sprung CG is where CarMaker states Body.I.
TEST(Ioniq5CarMakerParams, MassPropertiesMatchCarMaker) {
    const auto vp = vdsim::VehicleParams::from_yaml(kIoniq5VpYaml);

    EXPECT_NEAR(vp.mass, total_mass(), 0.05);
    EXPECT_NEAR(vp.mass_sprung, kCmBodyMass, 1e-9);

    EXPECT_NEAR(vp.inertia_diag.x(), kCmBodyIxx, 1e-9);
    EXPECT_NEAR(vp.inertia_diag.y(), kCmBodyIyy, 1e-9);
    EXPECT_NEAR(vp.inertia_diag.z(), kCmBodyIzz, 1e-9);

    // Whole-vehicle CG: body at Body.pos, corners at Wheel.pos.
    const double m = total_mass();
    const double x_cg = (kCmBodyMass * kCmBodyPosX
                         + 2.0 * corner_mass_front() * kCmWheelPosXF
                         + 2.0 * corner_mass_rear()  * kCmWheelPosXR) / m;
    const double z_cg = (kCmBodyMass * kCmBodyPosZ + unsprung_total() * kCmWheelPosZ) / m;

    EXPECT_NEAR(vp.cg_height, z_cg, 0.001);
    EXPECT_NEAR(vp.cg_to_front, kCmWheelPosXF - x_cg, 0.005);
    EXPECT_NEAR(vp.cg_to_rear,  x_cg - kCmWheelPosXR, 0.005);
    EXPECT_NEAR(vp.wheelbase, kCmWheelPosXF - kCmWheelPosXR, 1e-9);
    EXPECT_NEAR(vp.track_front, kCmTrack, 0.002);
}

// Aero.Coeff row tau = 0, split onto the two axles by the pitching moment.
// CarMaker's cL is positive upward; VDSim's aero_lift_* is positive downforce.
TEST(Ioniq5CarMakerParams, AeroMatchesCarMakerCoeff6x1) {
    const auto vp = vdsim::VehicleParams::from_yaml(kIoniq5VpYaml);

    EXPECT_NEAR(vp.aero_drag_coeff, kCmCd, 1e-9);
    EXPECT_NEAR(vp.frontal_area,    kCmAeroAx, 1e-9);

    // F_Lf + F_Lr = F_L and (F_Lf - F_Lr) * L/2 = M_PM  (nose-up positive).
    const double moment_share = (kCmAeroLRef / vp.wheelbase) * kCmCpm;
    const double cl_front = 0.5 * kCmCl + moment_share;
    const double cl_rear  = 0.5 * kCmCl - moment_share;

    EXPECT_NEAR(vp.aero_lift_front, -cl_front, 0.001);
    EXPECT_NEAR(vp.aero_lift_rear,  -cl_rear,  0.001);
    EXPECT_LT(vp.aero_lift_front, 0.0) << "this body produces lift, not downforce";
    EXPECT_LT(vp.aero_lift_rear,  0.0);
}

// Two mapped motors behind a single-speed reduction and an open diff per axle.
TEST(Ioniq5CarMakerParams, DrivelineMatchesCarMakerBev) {
    const auto vp = vdsim::VehicleParams::from_yaml(kIoniq5VpYaml);

    EXPECT_NEAR(vp.max_motor_torque, kCmNMotor * kCmMotorTrqMax, 1e-9);
    EXPECT_NEAR(vp.final_drive_ratio, kCmMotorRatio * kCmDiffRatio, 0.001);
    EXPECT_EQ(vp.differential, vdsim::VehicleParams::Differential::Open);

    // Reflected to the motor node, which is where VDSim applies
    // final_drive_ratio^2 on the way to the wheels.
    const double behind_motor_gear =
        (kCmGearBoxIIn + kCmGearBoxIOut + kCmDiffIIn) / (kCmMotorRatio * kCmMotorRatio);
    const double j_per_motor = kCmMotorInertia + behind_motor_gear;
    EXPECT_NEAR(vp.engine_rotational_inertia, kCmNMotor * j_per_motor, 5e-5);
}

// Steering.Rack2StWhl converts rack travel to handwheel angle; the front
// kinematics table converts the same rack travel to road-wheel steer angle.
TEST(Ioniq5CarMakerParams, SteeringRatioMatchesRackKinematics) {
    const auto vp = vdsim::VehicleParams::from_yaml(kIoniq5VpYaml);

    EXPECT_NEAR(vp.steering_ratio, kCmRack2StWhl / kCmRzPerRack, 0.01);
    // rz is linear in the rack and mirrored left/right: parallel steer.
    EXPECT_NEAR(vp.ackerman_percent, 0.0, 1e-9);
}

// Anti-dive / anti-squat and camber gain out of the Linear2D table (EQ 73).
TEST(Ioniq5CarMakerParams, AntiGeometryMatchesSuspensionKinematics) {
    const auto vp = vdsim::VehicleParams::from_yaml(kIoniq5VpYaml);

    // Front brakes are outboard, so the reaction point is the contact patch: a
    // spindle pitch dry moves a point r_w below the wheel centre by -r_w*dry.
    const double tan_theta_f = kCmTxPerTzF - kCmWheelPosZ * kCmRyPerTzF;
    // Rear drive is reacted by the chassis-mounted diff, so the reaction point
    // is the wheel centre and only the carrier recession counts.
    const double tan_theta_r = kCmTxPerTzR;

    const double lever = vp.wheelbase / vp.cg_height;
    EXPECT_NEAR(vp.anti_dive_front, 0.5 * tan_theta_f * lever, 0.002);
    EXPECT_NEAR(vp.anti_squat_rear, 0.5 * (-tan_theta_r) * lever, 0.002);
    EXPECT_GT(vp.anti_dive_front, vp.anti_squat_rear)
        << "this dataset has real front anti-dive and almost no anti-squat";

    // Pure roll puts (track/2)*phi of travel into each wheel; VDSim carries one
    // lumped scalar, so the YAML holds the two-axle mean.
    const double k_cam_f = kCmRxPerTzF * 0.5 * kCmTrack;
    const double k_cam_r = kCmRxPerTzR * 0.5 * kCmTrack;
    EXPECT_NEAR(vp.camber_per_roll, 0.5 * (k_cam_f + k_cam_r), 0.002);
}
