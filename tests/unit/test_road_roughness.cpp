// ISO 8608 road roughness provider: verify the synthesized profile has the
// standard spatial-PSD scaling -- each road class is 4x the PSD of the previous,
// i.e. 2x the RMS height -- and that magnitudes match published class ranges.

#include "vdsim/interfaces.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

// RMS of the per-wheel road profile (road_dz) sampled along travel for a class.
double profile_rms(int road_class) {
    auto ground = vdsim::create_iso8608_ground(0.0, 1.0, road_class, 1u);
    vdsim::VehicleParams vp;
    vdsim::State s;
    vdsim::ContactArray c;
    double sum = 0.0, sum2 = 0.0;
    const int N = 20000;
    for (int i = 0; i < N; ++i) {
        s.position.x() = 0.05 * i;          // sweep 0..1000 m
        ground->query(s, vp, c);
        const double dz = c[vdsim::WHEEL_FL].road_dz;
        sum += dz; sum2 += dz * dz;
    }
    const double mean = sum / N;
    return std::sqrt(sum2 / N - mean * mean);
}

}  // namespace

TEST(Iso8608, RmsDoublesPerClass) {
    const double a = profile_rms(0);   // class A
    const double b = profile_rms(1);   // class B
    const double c = profile_rms(2);   // class C
    // each class step is +6 dB PSD -> x2 RMS
    EXPECT_NEAR(b / a, 2.0, 0.3);
    EXPECT_NEAR(c / b, 2.0, 0.3);
    EXPECT_NEAR(c / a, 4.0, 0.6);
}

TEST(Iso8608, MagnitudesInPublishedRange) {
    // Class A "very good" ~ few mm; class C "average" ~ 10-20 mm RMS over the band.
    const double a = profile_rms(0);
    const double c = profile_rms(2);
    EXPECT_GT(a, 1e-3);  EXPECT_LT(a, 8e-3);
    EXPECT_GT(c, 8e-3);  EXPECT_LT(c, 25e-3);
}

TEST(Iso8608, LeftRightTracksDiffer) {
    // independent tracks -> left/right road_dz differ (excites roll)
    auto ground = vdsim::create_iso8608_ground(0.0, 1.0, 3, 1u);
    vdsim::VehicleParams vp; vdsim::State s; vdsim::ContactArray c;
    s.position.x() = 123.0;
    ground->query(s, vp, c);
    EXPECT_NE(c[vdsim::WHEEL_FL].road_dz, c[vdsim::WHEEL_FR].road_dz);
}
