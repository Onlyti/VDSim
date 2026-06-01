#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace {

constexpr double kTol = 1e-12;

fs::path tmp_path(const char* leaf) {
    static int counter = 0;
    auto p = fs::temp_directory_path() /
             ("vdsim_test_" + std::to_string(::getpid()) + "_" +
              std::to_string(counter++) + "_" + leaf);
    return p;
}

void write_file(const fs::path& path, const std::string& content) {
    std::ofstream ofs(path);
    ASSERT_TRUE(static_cast<bool>(ofs));
    ofs << content;
}

void expect_vehicle_equal(const vdsim::VehicleParams& a, const vdsim::VehicleParams& b) {
    EXPECT_NEAR(a.mass,        b.mass,        kTol);
    EXPECT_NEAR(a.mass_sprung, b.mass_sprung, kTol);
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(a.inertia_diag[i], b.inertia_diag[i], kTol);

    EXPECT_NEAR(a.wheelbase,            b.wheelbase,            kTol);
    EXPECT_NEAR(a.cg_to_front,          b.cg_to_front,          kTol);
    EXPECT_NEAR(a.cg_to_rear,           b.cg_to_rear,           kTol);
    EXPECT_NEAR(a.track_front,          b.track_front,          kTol);
    EXPECT_NEAR(a.track_rear,           b.track_rear,           kTol);
    EXPECT_NEAR(a.cg_height,            b.cg_height,            kTol);
    EXPECT_NEAR(a.wheel_radius_nominal, b.wheel_radius_nominal, kTol);

    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
        EXPECT_NEAR(a.spring_stiffness[i],   b.spring_stiffness[i],   kTol);
        EXPECT_NEAR(a.damper_coefficient[i], b.damper_coefficient[i], kTol);
        EXPECT_NEAR(a.unsprung_mass[i],      b.unsprung_mass[i],      kTol);
    }
    EXPECT_NEAR(a.roll_stiffness_front, b.roll_stiffness_front, kTol);
    EXPECT_NEAR(a.roll_stiffness_rear,  b.roll_stiffness_rear,  kTol);

    EXPECT_EQ(a.drive_type, b.drive_type);
    EXPECT_NEAR(a.max_motor_torque, b.max_motor_torque, kTol);
    EXPECT_NEAR(a.max_brake_torque, b.max_brake_torque, kTol);

    EXPECT_NEAR(a.steering_ratio,        b.steering_ratio,        kTol);
    EXPECT_NEAR(a.max_steer_angle_wheel, b.max_steer_angle_wheel, kTol);

    EXPECT_NEAR(a.aero_drag_coeff, b.aero_drag_coeff, kTol);
    EXPECT_NEAR(a.frontal_area,    b.frontal_area,    kTol);
}

void expect_tire_equal(const vdsim::TireParams& a, const vdsim::TireParams& b) {
    EXPECT_NEAR(a.B_long, b.B_long, kTol);
    EXPECT_NEAR(a.C_long, b.C_long, kTol);
    EXPECT_NEAR(a.D_long, b.D_long, kTol);
    EXPECT_NEAR(a.E_long, b.E_long, kTol);
    EXPECT_NEAR(a.B_lat,  b.B_lat,  kTol);
    EXPECT_NEAR(a.C_lat,  b.C_lat,  kTol);
    EXPECT_NEAR(a.D_lat,  b.D_lat,  kTol);
    EXPECT_NEAR(a.E_lat,  b.E_lat,  kTol);
    EXPECT_NEAR(a.mu_nominal,          b.mu_nominal,          kTol);
    EXPECT_NEAR(a.Fz_nominal,          b.Fz_nominal,          kTol);
    EXPECT_NEAR(a.cornering_stiffness, b.cornering_stiffness, kTol);
    EXPECT_NEAR(a.rolling_resistance,  b.rolling_resistance,  kTol);
}

}  // namespace

// =============================================================================
// VehicleParams
// =============================================================================

TEST(VehicleYaml, RoundtripDefaultsBitwise) {
    const vdsim::VehicleParams a;
    const auto path = tmp_path("vehicle_default.yaml");
    a.to_yaml(path.string());
    const auto b = vdsim::VehicleParams::from_yaml(path.string());
    expect_vehicle_equal(a, b);
    fs::remove(path);
}

TEST(VehicleYaml, BrakeBiasRoundtrip) {
    vdsim::VehicleParams a; a.brake_bias_front = 0.65;
    const auto p = tmp_path("bias.yaml");
    a.to_yaml(p.string());
    const auto b = vdsim::VehicleParams::from_yaml(p.string());
    EXPECT_DOUBLE_EQ(a.brake_bias_front, b.brake_bias_front);
    fs::remove(p);
}

TEST(VehicleYaml, AeroLiftRoundtrip) {
    vdsim::VehicleParams a;
    a.aero_lift_front = 1.20; a.aero_lift_rear = 1.85;
    const auto p = tmp_path("aero.yaml");
    a.to_yaml(p.string());
    const auto b = vdsim::VehicleParams::from_yaml(p.string());
    EXPECT_DOUBLE_EQ(a.aero_lift_front, b.aero_lift_front);
    EXPECT_DOUBLE_EQ(a.aero_lift_rear,  b.aero_lift_rear);
    fs::remove(p);
}

TEST(VehicleYaml, DifferentialRoundtrip) {
    for (auto d : {vdsim::VehicleParams::Differential::Open,
                   vdsim::VehicleParams::Differential::Locked,
                   vdsim::VehicleParams::Differential::LSD}) {
        vdsim::VehicleParams a;
        a.differential = d; a.lsd_preload = 0.18; a.lsd_ramp = 0.40;
        const auto path = tmp_path("diff.yaml");
        a.to_yaml(path.string());
        const auto b = vdsim::VehicleParams::from_yaml(path.string());
        EXPECT_EQ(a.differential, b.differential);
        EXPECT_DOUBLE_EQ(a.lsd_preload, b.lsd_preload);
        EXPECT_DOUBLE_EQ(a.lsd_ramp,    b.lsd_ramp);
        fs::remove(path);
    }
}

TEST(VehicleYaml, AckermanRoundtrip) {
    vdsim::VehicleParams a;
    a.ackerman_percent = 75.0;
    const auto path = tmp_path("vehicle_ackerman.yaml");
    a.to_yaml(path.string());
    const auto b = vdsim::VehicleParams::from_yaml(path.string());
    EXPECT_DOUBLE_EQ(a.ackerman_percent, b.ackerman_percent);
    fs::remove(path);
}

TEST(VehicleYaml, RoundtripCustomValues) {
    vdsim::VehicleParams a;
    a.mass            = 1234.5;
    a.mass_sprung     = 1100.0;
    a.inertia_diag    = vdsim::Vec3(456.0, 1888.0, 2333.0);
    a.wheelbase       = 2.85;
    a.cg_to_front     = 1.30;
    a.cg_to_rear      = 1.55;
    a.track_front     = 1.60;
    a.track_rear      = 1.58;
    a.cg_height       = 0.48;
    a.wheel_radius_nominal = 0.33;
    a.spring_stiffness   = {{31000, 31000, 28000, 28000}};
    a.damper_coefficient = {{3100,  3100,  2800,  2800}};
    a.unsprung_mass      = {{42, 42, 38, 38}};
    a.roll_stiffness_front = 35000.0;
    a.roll_stiffness_rear  = 27000.0;
    a.drive_type = vdsim::VehicleParams::Drive::AWD;
    a.max_motor_torque = 450.0;
    a.max_brake_torque = 2500.0;
    a.steering_ratio        = 14.5;
    a.max_steer_angle_wheel = 0.55;
    a.aero_drag_coeff = 0.28;
    a.frontal_area    = 2.15;

    const auto path = tmp_path("vehicle_custom.yaml");
    a.to_yaml(path.string());
    const auto b = vdsim::VehicleParams::from_yaml(path.string());
    expect_vehicle_equal(a, b);
    fs::remove(path);
}

TEST(VehicleYaml, DriveTypeAllThree) {
    for (auto d : {vdsim::VehicleParams::Drive::FWD,
                   vdsim::VehicleParams::Drive::RWD,
                   vdsim::VehicleParams::Drive::AWD}) {
        vdsim::VehicleParams a;
        a.drive_type = d;
        const auto path = tmp_path("vehicle_drive.yaml");
        a.to_yaml(path.string());
        const auto b = vdsim::VehicleParams::from_yaml(path.string());
        EXPECT_EQ(a.drive_type, b.drive_type);
        fs::remove(path);
    }
}

TEST(VehicleYaml, PartialYamlKeepsDefaults) {
    const auto path = tmp_path("vehicle_partial.yaml");
    write_file(path, "mass: 2000.0\nwheelbase: 3.0\n");

    const auto p = vdsim::VehicleParams::from_yaml(path.string());
    const vdsim::VehicleParams def;

    EXPECT_DOUBLE_EQ(p.mass,      2000.0);
    EXPECT_DOUBLE_EQ(p.wheelbase, 3.0);
    EXPECT_DOUBLE_EQ(p.cg_height, def.cg_height);
    EXPECT_DOUBLE_EQ(p.aero_drag_coeff, def.aero_drag_coeff);
    EXPECT_EQ(p.drive_type, def.drive_type);
    fs::remove(path);
}

TEST(VehicleYaml, UnknownKeysIgnored) {
    const auto path = tmp_path("vehicle_extra.yaml");
    write_file(path,
        "mass: 1700.0\n"
        "unknown_future_key: 42.0\n"
        "experimental:\n"
        "  nested: 1\n"
        "  list: [1, 2, 3]\n");

    EXPECT_NO_THROW({
        const auto p = vdsim::VehicleParams::from_yaml(path.string());
        EXPECT_DOUBLE_EQ(p.mass, 1700.0);
    });
    fs::remove(path);
}

TEST(VehicleYaml, BadDriveStringThrows) {
    const auto path = tmp_path("vehicle_bad_drive.yaml");
    write_file(path, "drive_type: HYBRID\n");
    EXPECT_THROW(vdsim::VehicleParams::from_yaml(path.string()), std::runtime_error);
    fs::remove(path);
}

TEST(VehicleYaml, BadArrayLengthThrows) {
    const auto path = tmp_path("vehicle_bad_array.yaml");
    write_file(path, "spring_stiffness: [1, 2, 3]\n");      // only 3
    EXPECT_THROW(vdsim::VehicleParams::from_yaml(path.string()), std::runtime_error);
    fs::remove(path);
}

TEST(VehicleYaml, MissingFileThrows) {
    EXPECT_THROW(vdsim::VehicleParams::from_yaml("/no/such/path.yaml"),
                 std::runtime_error);
}

TEST(VehicleYaml, GeometryConsistencyWarnsButLoads) {
    // a + b != wheelbase  =>  spdlog::warn but load succeeds, values respected.
    const auto path = tmp_path("vehicle_inconsistent.yaml");
    write_file(path,
        "wheelbase: 2.7\n"
        "cg_to_front: 1.0\n"
        "cg_to_rear: 1.0\n");
    EXPECT_NO_THROW({
        const auto p = vdsim::VehicleParams::from_yaml(path.string());
        EXPECT_DOUBLE_EQ(p.wheelbase, 2.7);
        EXPECT_DOUBLE_EQ(p.cg_to_front, 1.0);
        EXPECT_DOUBLE_EQ(p.cg_to_rear, 1.0);
    });
    fs::remove(path);
}

// =============================================================================
// TireParams
// =============================================================================

TEST(TireYaml, LoadDefaultsKeptOnEmpty) {
    const auto path = tmp_path("tire_empty.yaml");
    write_file(path, "{}\n");
    const auto p = vdsim::TireParams::from_yaml(path.string());
    const vdsim::TireParams def;
    expect_tire_equal(p, def);
    fs::remove(path);
}

TEST(TireYaml, LoadCustomFlat) {
    const auto path = tmp_path("tire_custom.yaml");
    write_file(path,
        "B_long: 12.0\n"
        "C_long: 1.70\n"
        "D_long: 1.05\n"
        "E_long: 0.98\n"
        "B_lat:  9.5\n"
        "C_lat:  1.35\n"
        "D_lat:  1.02\n"
        "E_lat:  -0.95\n"
        "mu_nominal: 1.10\n"
        "Fz_nominal: 4500.0\n"
        "cornering_stiffness: 95000.0\n"
        "rolling_resistance: 0.012\n");
    const auto p = vdsim::TireParams::from_yaml(path.string());
    EXPECT_DOUBLE_EQ(p.B_long, 12.0);
    EXPECT_DOUBLE_EQ(p.C_lat,  1.35);
    EXPECT_DOUBLE_EQ(p.E_lat, -0.95);
    EXPECT_DOUBLE_EQ(p.mu_nominal, 1.10);
    EXPECT_DOUBLE_EQ(p.cornering_stiffness, 95000.0);
    fs::remove(path);
}

TEST(TireYaml, FromTirThrowsNotImplemented) {
    EXPECT_THROW(vdsim::TireParams::from_tir("anything.tir"),
                 std::runtime_error);
}

TEST(TireYaml, RoundtripDefaultsBitwise) {
    const vdsim::TireParams a;
    const auto path = tmp_path("tire_default.yaml");
    a.to_yaml(path.string());
    const auto b = vdsim::TireParams::from_yaml(path.string());
    expect_tire_equal(a, b);
    EXPECT_EQ(a.combined_slip_enabled, b.combined_slip_enabled);
    EXPECT_DOUBLE_EQ(a.pneumatic_trail, b.pneumatic_trail);
    EXPECT_DOUBLE_EQ(a.trail_falloff_alpha, b.trail_falloff_alpha);
    fs::remove(path);
}

TEST(TireYaml, RoundtripCustomFlagsAndTrail) {
    vdsim::TireParams a;
    a.B_long = 11.5; a.C_lat = 1.45; a.mu_nominal = 1.05;
    a.combined_slip_enabled = false;
    a.pneumatic_trail = 0.038;
    a.trail_falloff_alpha = 0.15;
    const auto path = tmp_path("tire_custom_full.yaml");
    a.to_yaml(path.string());
    const auto b = vdsim::TireParams::from_yaml(path.string());
    EXPECT_DOUBLE_EQ(a.B_long, b.B_long);
    EXPECT_DOUBLE_EQ(a.C_lat,  b.C_lat);
    EXPECT_DOUBLE_EQ(a.mu_nominal, b.mu_nominal);
    EXPECT_EQ(a.combined_slip_enabled, b.combined_slip_enabled);
    EXPECT_DOUBLE_EQ(a.pneumatic_trail, b.pneumatic_trail);
    EXPECT_DOUBLE_EQ(a.trail_falloff_alpha, b.trail_falloff_alpha);
    fs::remove(path);
}

// =============================================================================
// SolverParams
// =============================================================================

TEST(SolverYaml, RoundtripDefaults) {
    const vdsim::SolverParams a;
    const auto path = tmp_path("solver_default.yaml");
    a.to_yaml(path.string());
    const auto b = vdsim::SolverParams::from_yaml(path.string());
    EXPECT_EQ(a.integrator, b.integrator);
    EXPECT_DOUBLE_EQ(a.max_substep_dt, b.max_substep_dt);
    EXPECT_EQ(a.max_substeps, b.max_substeps);
    fs::remove(path);
}

TEST(SolverYaml, EulerRoundtrip) {
    vdsim::SolverParams a;
    a.integrator     = vdsim::SolverParams::Integrator::Euler;
    a.max_substep_dt = 5e-4;
    a.max_substeps   = 20;
    const auto path = tmp_path("solver_euler.yaml");
    a.to_yaml(path.string());
    const auto b = vdsim::SolverParams::from_yaml(path.string());
    EXPECT_EQ(b.integrator, vdsim::SolverParams::Integrator::Euler);
    EXPECT_DOUBLE_EQ(b.max_substep_dt, 5e-4);
    EXPECT_EQ(b.max_substeps, 20);
    fs::remove(path);
}

TEST(SolverYaml, BadIntegratorThrows) {
    const auto path = tmp_path("solver_bad.yaml");
    write_file(path, "integrator: ImplicitEuler\n");
    EXPECT_THROW(vdsim::SolverParams::from_yaml(path.string()),
                 std::runtime_error);
    fs::remove(path);
}

TEST(SolverYaml, NegativeDtThrows) {
    const auto path = tmp_path("solver_negdt.yaml");
    write_file(path, "max_substep_dt: -1e-3\n");
    EXPECT_THROW(vdsim::SolverParams::from_yaml(path.string()),
                 std::runtime_error);
    fs::remove(path);
}
