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
