#include "vdsim/suspension.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <filesystem>
#include <memory>

namespace {

// Tiny CSV synthetic table: 3 travels × 3 steers,
// camber = +1.0 deg per mm bump,  toe = +0.5 deg per mm rack dy.
std::string make_csv(const std::string& path) {
    std::ofstream f(path);
    f << "wheel_travel,steer_rack_dy,camber,toe,track_change,caster,valid\n";
    const double travels[] = {-0.01, 0.0, +0.01};
    const double steers[]  = {-0.005, 0.0, +0.005};
    for (double t : travels)
        for (double s : steers) {
            const double cam = 1.0 * (t / 0.001) * (M_PI / 180.0);    // 1°/mm
            const double toe = 0.5 * (s / 0.001) * (M_PI / 180.0);    // 0.5°/mm
            const double trk = -0.4 * t;                              // -0.4 m/m
            const double cas = 0.025;                                 // constant
            f << t << "," << s << "," << cam << "," << toe << ","
              << trk << "," << cas << ",1\n";
        }
    return path;
}

}  // namespace

class LookupKinematicsFixture : public ::testing::Test {
protected:
    void SetUp() override {
        path_ = (std::filesystem::temp_directory_path() / "vdsim_lookup_test.csv").string();
        make_csv(path_);
        k_ = vdsim::create_lookup_kinematics(path_);
        ASSERT_NE(k_, nullptr);
    }
    void TearDown() override { std::filesystem::remove(path_); }

    std::string path_;
    std::unique_ptr<vdsim::ISuspensionKinematics> k_;
};

TEST_F(LookupKinematicsFixture, ExactGridPointReturnsExactValues) {
    const auto o = k_->compute(0.0, 0.0);
    EXPECT_NEAR(o.camber, 0.0, 1e-6);
    EXPECT_NEAR(o.toe,    0.0, 1e-6);
}

TEST_F(LookupKinematicsFixture, BilinearInterpAtMidpoint) {
    // Midway between travel=0 and travel=+0.01 (i.e., 5 mm bump), steer=0
    const auto o = k_->compute(0.005, 0.0);
    // Expected camber = 5 deg = 5 * pi/180
    EXPECT_NEAR(o.camber, 5.0 * M_PI / 180.0, 1e-6);
}

TEST_F(LookupKinematicsFixture, SteerInterpolation) {
    // Midway between steer=0 and steer=+0.005 (i.e., +0.0025), travel=0
    const auto o = k_->compute(0.0, 0.0025);
    // Expected toe = 2.5 * 0.5 = 1.25 deg = 1.25*pi/180
    EXPECT_NEAR(o.toe, 1.25 * M_PI / 180.0, 1e-6);
}

TEST_F(LookupKinematicsFixture, ClampOutsideRange) {
    const auto o_lo = k_->compute(-0.10, 0.0);
    const auto o_hi = k_->compute(+0.10, 0.0);
    // Clamped to ±0.01 travel = ±10 deg camber
    EXPECT_NEAR(o_lo.camber, -10.0 * M_PI / 180.0, 1e-6);
    EXPECT_NEAR(o_hi.camber, +10.0 * M_PI / 180.0, 1e-6);
}

TEST_F(LookupKinematicsFixture, TrackChangeLinearInTravel) {
    const auto o = k_->compute(0.005, 0.0);    // 5 mm bump
    EXPECT_NEAR(o.track_change, -0.4 * 0.005, 1e-6);
}

// =============================================================================
// DW native kinematics solver — match lookup at grid points
// =============================================================================
TEST(DWNativeKinematics, MatchesLookupAtGridPoints) {
    const std::string yaml_path =
        std::string(VDSIM_SOURCE_DIR) + "/configs/suspensions/dw_front_sports.yaml";
    const std::string csv_path  =
        std::string(VDSIM_SOURCE_DIR) + "/docs/tasks/T27_ld4_dw/run3d/sweep_3d.csv";

    auto k_native = vdsim::create_dw_native_kinematics(yaml_path);
    auto k_lookup = vdsim::create_lookup_kinematics(csv_path);
    ASSERT_NE(k_native, nullptr);
    ASSERT_NE(k_lookup, nullptr);

    // At grid points from the precomputed sweep (travel ∈ {-50,-25,0,25,50} mm,
    // steer ∈ {-20,0,+20} mm), the lookup returns the exact precomputed value
    // and the native solver returns the same physically-correct answer.
    for (double t : {-0.050, 0.0, +0.050}) {
        for (double s : {-0.020, 0.0, +0.020}) {
            const auto an = k_native->compute(t, s);
            const auto al = k_lookup->compute(t, s);
            EXPECT_NEAR(an.camber, al.camber, 1e-3);
            EXPECT_NEAR(an.toe,    al.toe,    1e-3);
            EXPECT_NEAR(an.track_change, al.track_change, 1e-3);
        }
    }
}

TEST(SuspensionFactory, DispatchesByYamlType) {
    const std::string base = std::string(VDSIM_SOURCE_DIR) + "/configs/suspensions/";
    auto k = vdsim::create_native_kinematics_from_yaml(base + "mp_front_sedan.yaml");
    ASSERT_NE(k, nullptr);
    const auto o = k->compute(0.0, 0.0);
    EXPECT_TRUE(std::isfinite(o.camber));
}

TEST(SuspensionFactory, RejectsTopologyOnlyYaml) {
    const std::string path =
        std::string(VDSIM_SOURCE_DIR) + "/configs/suspensions/double_wishbone.yaml";
    EXPECT_THROW(vdsim::create_native_kinematics_from_yaml(path), std::exception);
}

TEST(DWNativeKinematics, StaticReturnsZero) {
    auto k = vdsim::create_dw_native_kinematics(
        std::string(VDSIM_SOURCE_DIR) + "/configs/suspensions/dw_front_sports.yaml");
    const auto o = k->compute(0.0, 0.0);
    EXPECT_NEAR(o.camber, 0.0, 1e-6);
    EXPECT_NEAR(o.toe,    0.0, 1e-6);
    EXPECT_NEAR(o.track_change, 0.0, 1e-6);
    EXPECT_GT(o.caster, 0.0);   // sample geometry has +caster ≈ 0.025 rad
    EXPECT_LT(o.caster, 0.05);
}

// =============================================================================
// TA / MP / 5-link native kinematics — match Python lookup at grid points
// =============================================================================
TEST(TANativeKinematics, MatchesLookupAtGridPoints) {
    auto k_n = vdsim::create_ta_native_kinematics(
        std::string(VDSIM_SOURCE_DIR) + "/configs/suspensions/ta_rear_sedan.yaml");
    auto k_l = vdsim::create_lookup_kinematics(
        std::string(VDSIM_SOURCE_DIR) + "/docs/tasks/T29_ld4_ta/run01/sweep_3d.csv");
    for (double t : {-0.04, 0.0, +0.04}) {
        const auto on = k_n->compute(t, 0.0);
        const auto ol = k_l->compute(t, 0.0);
        EXPECT_NEAR(on.camber, ol.camber, 1e-3);
        EXPECT_NEAR(on.toe,    ol.toe,    1e-3);
    }
}

TEST(MPNativeKinematics, MatchesLookupAtGridPoints) {
    auto k_n = vdsim::create_mp_native_kinematics(
        std::string(VDSIM_SOURCE_DIR) + "/configs/suspensions/mp_front_sedan.yaml");
    auto k_l = vdsim::create_lookup_kinematics(
        std::string(VDSIM_SOURCE_DIR) + "/docs/tasks/T28_ld4_mp/run01/sweep_3d.csv");
    for (double t : {-0.04, 0.0, +0.04}) {
        const auto on = k_n->compute(t, 0.0);
        const auto ol = k_l->compute(t, 0.0);
        EXPECT_NEAR(on.camber, ol.camber, 5e-3);
        EXPECT_NEAR(on.toe,    ol.toe,    5e-3);
    }
}

TEST(FiveLinkNativeKinematics, MatchesLookupAtGridPoints) {
    auto k_n = vdsim::create_5link_native_kinematics(
        std::string(VDSIM_SOURCE_DIR) + "/configs/suspensions/5link_rear_sports.yaml");
    auto k_l = vdsim::create_lookup_kinematics(
        std::string(VDSIM_SOURCE_DIR) + "/docs/tasks/T30_ld4_5link/run01/sweep_3d.csv");
    for (double t : {-0.025, 0.0, +0.025}) {
        const auto on = k_n->compute(t, 0.0);
        const auto ol = k_l->compute(t, 0.0);
        EXPECT_NEAR(on.camber, ol.camber, 5e-3);
        EXPECT_NEAR(on.toe,    ol.toe,    5e-3);
    }
}
