// Per-element aero loads (work order 42, 0.8.0): A1, A2, A5, A7, E2, E3.
// Check vector of the team wiki: rho 1.165, 60 km/h, wheelbase 1.55 m.
#include "vdsim/aero.hpp"
#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

using namespace vdsim;

namespace {

constexpr double kV60 = 60.0 / 3.6;   // [m/s]

const std::string kExample =
    std::string(VDSIM_SOURCE_DIR) + "/configs/vehicles/fs_aero_example.yaml";

double rel(double got, double want) { return std::abs(got - want) / std::abs(want); }

std::string write_tmp(const std::string& stem, const std::string& body) {
    const std::string path = ::testing::TempDir() + stem + ".yaml";
    std::ofstream(path) << body;
    return path;
}

const char* kFourElements =
    "wheelbase: 1.55\ncg_to_front: 0.82\ncg_to_rear: 0.73\n"
    "air_density: 1.165\n"
    "aero:\n  elements:\n"
    "    - {name: front_wing, ClA: 0.70, CdA: 0.12, x:  0.550, z: 0.100}\n"
    "    - {name: rear_wing,  ClA: 1.50, CdA: 0.55, x: -1.520, z: 1.000}\n"
    "    - {name: undertray,  ClA: 0.80, CdA: 0.05, x: -0.750, z: 0.060}\n"
    "    - {name: body,       ClA: 0.00, CdA: 0.75, x: -0.775, z: 0.300}\n";

}  // namespace

/// A1: total downforce, total drag and the downforce-only centre of pressure.
TEST(Aero, A1_WikiVectorTotalsAndCentreOfPressure) {
    const auto vp = VehicleParams::from_yaml(kExample);
    ASSERT_EQ(vp.aero_elements.size(), 4u);
    const auto L = compute_aero_loads(vp, AeroState{kV60});
    EXPECT_LE(rel(L.fz_total, 485.4), 1e-3) << L.fz_total;
    EXPECT_LE(rel(L.fx_total, 237.9), 1e-3) << L.fx_total;
    double m = 0.0;
    for (std::size_t i = 0; i < L.elements.size(); ++i) m += L.elements[i].fz * vp.aero_elements[i].x;
    EXPECT_LE(rel(m / L.fz_total, -0.8317), 1e-3) << m / L.fz_total;
}

/// A2: front-axle share with the drag pitch moment, and the effective axle ClA.
TEST(Aero, A2_FrontShareIncludesDragMoment) {
    const auto vp = VehicleParams::from_yaml(kExample);
    const auto L = compute_aero_loads(vp, AeroState{kV60});
    EXPECT_NEAR(100.0 * L.balance_front, 29.4, 0.1) << L.balance_front;
    const double q = 0.5 * vp.air_density * kV60 * kV60;
    EXPECT_GE(L.fz_front / q, 0.880);
    EXPECT_LE(L.fz_front / q, 0.881);
    EXPECT_GE(L.fz_rear / q, 2.119);
    EXPECT_LE(L.fz_rear / q, 2.120);
    // Wheel splits sum back to the axle reactions.
    EXPECT_NEAR(L.fz_wheel_roll[WHEEL_FL] + L.fz_wheel_roll[WHEEL_FR], L.fz_front, 1e-9);
    EXPECT_NEAR(L.fz_wheel_split[WHEEL_RL] + L.fz_wheel_split[WHEEL_RR], L.fz_rear, 1e-9);
}

/// E4: the roll split uses each axle's own track.
TEST(Aero, E4_RollSplitUsesAxleTrack) {
    VehicleParams vp;
    vp.wheelbase = 2.0; vp.cg_to_front = 1.0; vp.cg_to_rear = 1.0;
    vp.track_front = 1.6; vp.track_rear = 1.2;
    vp.aero_elements = {{"f", 1.0, 0.0, 0.0, 0.2, 0.1}, {"r", 1.0, 0.0, -2.0, 0.2, 0.1}};
    const auto L = compute_aero_loads(vp, AeroState{20.0});
    const double F = 0.5 * vp.air_density * 400.0;   // per element
    EXPECT_NEAR(L.fz_wheel_roll[WHEEL_FL], F * (0.5 + 0.2 / 1.6), 1e-9);
    EXPECT_NEAR(L.fz_wheel_roll[WHEEL_RL], F * (0.5 + 0.2 / 1.2), 1e-9);
    EXPECT_NEAR(L.fz_wheel_split[WHEEL_FL], 0.5 * F, 1e-9);
    // Roll moment on the body wrench: -y * fz about x.
    EXPECT_NEAR(L.torque_body.x(), -0.2 * F * 2.0, 1e-9);
}

/// E2: reversing keeps downforce (vx^2) and flips the drag sign.
TEST(Aero, E2_ReversingKeepsDownforceFlipsDrag) {
    const auto vp = VehicleParams::from_yaml(kExample);
    const auto fwd = compute_aero_loads(vp, AeroState{kV60});
    const auto rev = compute_aero_loads(vp, AeroState{-kV60});
    EXPECT_DOUBLE_EQ(rev.fz_total, fwd.fz_total);
    EXPECT_DOUBLE_EQ(rev.fx_total, -fwd.fx_total);
    EXPECT_GT(rev.force_body.x(), 0.0);   // drag pushes +x when moving -x
    const auto still = compute_aero_loads(vp, AeroState{0.0});
    EXPECT_TRUE(std::isnan(still.ld));
    EXPECT_TRUE(std::isnan(still.balance_front));
}

/// A5: forces scale exactly with air_density; the default is 1.225.
TEST(Aero, A5_AirDensityScalesForces) {
    auto vp = VehicleParams::from_yaml(kExample);
    EXPECT_EQ(VehicleParams{}.air_density, 1.225);
    vp.air_density = 1.225;
    const auto a = compute_aero_loads(vp, AeroState{kV60});
    vp.air_density = 1.165;
    const auto b = compute_aero_loads(vp, AeroState{kV60});
    EXPECT_LE(rel(b.fz_total / a.fz_total, 1.165 / 1.225), 1e-12);
    EXPECT_LE(rel(b.fx_total / a.fx_total, 1.165 / 1.225), 1e-12);
}

/// Four-key path: lift at the axles, drag at the ground (split) and at the CG (wrench).
TEST(Aero, FourKeyPathLegacyPlacement) {
    VehicleParams vp;
    vp.aero_lift_front = 0.4; vp.aero_lift_rear = 0.6;
    EXPECT_FALSE(aero_uses_element_path(vp));
    const double v = 30.0;
    const auto L = compute_aero_loads(vp, AeroState{v});
    const double q = 0.5 * vp.air_density * v * v * vp.frontal_area;
    EXPECT_NEAR(L.fz_front, 0.4 * q, 1e-9);
    EXPECT_NEAR(L.fz_rear, 0.6 * q, 1e-9);
    EXPECT_EQ(L.drag_pitch_moment, 0.0);
    // Wrench: lift moment only, drag through the CG.
    EXPECT_NEAR(L.torque_body.y(), 0.4 * q * vp.cg_to_front - 0.6 * q * vp.cg_to_rear, 1e-6);
    vp.aero_drag_height = 0.5;
    EXPECT_TRUE(aero_uses_element_path(vp));
    const auto H = compute_aero_loads(vp, AeroState{v});
    EXPECT_NEAR(H.fz_front, L.fz_front - H.fx_total * 0.5 / vp.wheelbase, 1e-9);
}

/// A7 (core): every double specification and malformed element is refused.
TEST(Aero, A7_ValidationRefusesInconsistentInput) {
    VehicleParams ok = VehicleParams::from_yaml(kExample);
    EXPECT_NO_THROW(validate_aero_params(ok));
    auto bad = ok; bad.aero_lift_front = 0.1;
    EXPECT_THROW(validate_aero_params(bad), std::invalid_argument);
    bad = ok; bad.aero_drag_height = 0.3;
    EXPECT_THROW(validate_aero_params(bad), std::invalid_argument);
    bad = ok; bad.aero_elements[1].name = "front_wing";
    EXPECT_THROW(validate_aero_params(bad), std::invalid_argument);
    bad = ok; bad.aero_elements[0].CdA = -0.01;
    EXPECT_THROW(validate_aero_params(bad), std::invalid_argument);
    bad = ok; bad.aero_elements[0].z = -0.01;
    EXPECT_THROW(validate_aero_params(bad), std::invalid_argument);
    bad = ok; bad.aero_elements[0].name = "Front-Wing";
    EXPECT_THROW(validate_aero_params(bad), std::invalid_argument);
    bad = ok; bad.air_density = 0.0;
    EXPECT_THROW(validate_aero_params(bad), std::invalid_argument);
}

/// A7 (YAML): the same rules at load, including the four keys by presence.
TEST(Aero, A7_YamlRefusesDoubleSpecification) {
    EXPECT_NO_THROW(VehicleParams::from_yaml(write_tmp("aero_ok", kFourElements)));
    for (const char* extra : {"aero_lift_front: 0.0\n", "aero_lift_rear: 0.5\n",
                              "aero_drag_coeff: 0.3\n", "frontal_area: 1.0\n",
                              "aero_drag_height: 0.3\n"}) {
        const std::string p = write_tmp("aero_dbl", std::string(kFourElements) + extra);
        EXPECT_THROW(VehicleParams::from_yaml(p), std::invalid_argument) << extra;
    }
    const std::string missing_x =
        "aero:\n  elements:\n    - {name: w, ClA: 1.0, CdA: 0.1, z: 0.2}\n";
    EXPECT_THROW(VehicleParams::from_yaml(write_tmp("aero_nox", missing_x)),
                 std::invalid_argument);
}

/// E3: unset drag height and 0.0 stay distinct through a YAML round trip.
TEST(Aero, E3_DragHeightUnsetVersusZeroRoundTrip) {
    VehicleParams a;
    const std::string p0 = ::testing::TempDir() + "aero_unset.yaml";
    a.to_yaml(p0);
    EXPECT_FALSE(VehicleParams::from_yaml(p0).aero_drag_height.has_value());
    a.aero_drag_height = 0.0;
    const std::string p1 = ::testing::TempDir() + "aero_zero.yaml";
    a.to_yaml(p1);
    const auto b = VehicleParams::from_yaml(p1);
    ASSERT_TRUE(b.aero_drag_height.has_value());
    EXPECT_EQ(*b.aero_drag_height, 0.0);
}

/// Elements and air density survive save -> load unchanged.
TEST(Aero, ElementsRoundTrip) {
    auto a = VehicleParams::from_yaml(kExample);
    a.aero_elements[0].y = 0.05;
    const std::string p = ::testing::TempDir() + "aero_rt.yaml";
    a.to_yaml(p);
    const auto b = VehicleParams::from_yaml(p);
    ASSERT_EQ(b.aero_elements.size(), a.aero_elements.size());
    for (std::size_t i = 0; i < a.aero_elements.size(); ++i) {
        EXPECT_EQ(b.aero_elements[i].name, a.aero_elements[i].name);
        EXPECT_DOUBLE_EQ(b.aero_elements[i].ClA, a.aero_elements[i].ClA);
        EXPECT_DOUBLE_EQ(b.aero_elements[i].CdA, a.aero_elements[i].CdA);
        EXPECT_DOUBLE_EQ(b.aero_elements[i].x, a.aero_elements[i].x);
        EXPECT_DOUBLE_EQ(b.aero_elements[i].y, a.aero_elements[i].y);
        EXPECT_DOUBLE_EQ(b.aero_elements[i].z, a.aero_elements[i].z);
    }
    EXPECT_DOUBLE_EQ(b.air_density, 1.165);
}
