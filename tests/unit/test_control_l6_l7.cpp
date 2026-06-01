#include "vdsim/control_converter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

// =============================================================================
// L6 LongVxController
// =============================================================================

TEST(LongVxController, ZeroErrorZeroOutput) {
    vdsim::LongVxController c; c.initialize({});
    EXPECT_DOUBLE_EQ(c.update(10.0, 10.0, 0.005), 0.0);
}

TEST(LongVxController, PositiveErrorPositiveAx) {
    vdsim::LongVxController c; c.initialize({});
    EXPECT_GT(c.update(15.0, 10.0, 0.005), 0.0);
}

TEST(LongVxController, NegativeErrorNegativeAx) {
    vdsim::LongVxController c; c.initialize({});
    EXPECT_LT(c.update(5.0, 10.0, 0.005), 0.0);
}

TEST(LongVxController, OutputClamped) {
    vdsim::LongVxController c; vdsim::LongVxController::Gains g;
    g.kp = 100.0; g.ki = 0.0; g.ax_clamp = 2.5;
    c.initialize(g);
    EXPECT_LE(c.update(100.0, 0.0, 0.005), 2.5 + 1e-9);
    EXPECT_GE(c.update(-100.0, 0.0, 0.005), -2.5 - 1e-9);
}

// =============================================================================
// L7 PurePursuitController
// =============================================================================

TEST(PurePursuit, StraightAheadZeroSteer) {
    vdsim::PurePursuitController pp; vdsim::PurePursuitController::Gains g;
    g.wheelbase = 2.7; pp.initialize(g);
    std::vector<double> px = {0, 10, 20, 30}, py = {0, 0, 0, 0};
    const auto out = pp.update(0.0, 0.0, 0.0, 10.0, px.data(), py.data(), 4);
    EXPECT_NEAR(out.steer, 0.0, 1e-6);
}

TEST(PurePursuit, LeftCircleProducesPositiveSteer) {
    vdsim::PurePursuitController pp; vdsim::PurePursuitController::Gains g;
    g.wheelbase = 2.7; pp.initialize(g);
    // Circular arc of radius R=20, samples
    std::vector<double> px, py;
    for (int i = 0; i < 40; ++i) {
        const double t = i * 0.05;        // arc parameter
        px.push_back(20.0 * std::sin(t));
        py.push_back(20.0 - 20.0 * std::cos(t));   // starts at (0,0), curves left
    }
    const auto out = pp.update(0.0, 0.0, 0.0, 5.0,
                               px.data(), py.data(), (int)px.size());
    EXPECT_GT(out.steer, 0.0);
    EXPECT_GT(out.curvature, 0.0);
}

// =============================================================================
// DriverModel
// =============================================================================

TEST(DriverModel, ReactionTimeDelaysSteer) {
    vdsim::DriverModel d; vdsim::DriverModel::Gains g;
    g.wheelbase = 2.7; g.reaction_time_s = 0.10;
    g.steer_noise_rms = 0.0; g.thr_noise_rms = 0.0;
    d.initialize(g);
    std::vector<double> px = {0,10,20,30}, py = {0,1,2,3};       // gentle curve
    // First few ticks should output zero steer (buffer empty).
    auto out0 = d.update(0,0,0,5.0, 5.0, px.data(), py.data(), 4, 0.01);
    EXPECT_NEAR(out0.steer, 0.0, 1e-6);
    // After fewer than 0.10s the buffer hasn't released the steer yet.
    for (int i = 0; i < 5; ++i)
        d.update(0,0,0,5.0, 5.0, px.data(), py.data(), 4, 0.01);
    auto out_later = d.update(0,0,0,5.0, 5.0, px.data(), py.data(), 4, 0.01);
    // After >10 ticks at 10ms, steer should match the (small) PP output.
    EXPECT_GE(out_later.steer, -0.5);
    EXPECT_LE(out_later.steer,  0.5);
}

TEST(DriverModel, NoiseIsBoundedByRMS) {
    vdsim::DriverModel d; vdsim::DriverModel::Gains g;
    g.wheelbase = 2.7; g.reaction_time_s = 0.001;
    g.steer_noise_rms = 0.005; g.thr_noise_rms = 0.0;
    d.initialize(g);
    std::vector<double> px = {0,100}, py = {0,0};
    double max_steer = 0.0;
    for (int i = 0; i < 200; ++i) {
        const double a = 0.05 + i * 0.001;
        const double b = 0.05 + i * 0.013;
        auto out = d.update(0,0,0,10.0, 10.0, px.data(), py.data(), 2, 0.005,
                            a - std::floor(a), b - std::floor(b));
        max_steer = std::max(max_steer, std::abs(out.steer));
    }
    EXPECT_LE(max_steer, 0.04);   // ~ 8 sigma bound
}

TEST(PurePursuit, MaxSteerClamped) {
    vdsim::PurePursuitController pp; vdsim::PurePursuitController::Gains g;
    g.wheelbase = 2.7; g.max_steer = 0.30; pp.initialize(g);
    // Tight 90-degree turn at 1 m radius
    std::vector<double> px = {0, 0.1, 0.2, 0.3, 0.4};
    std::vector<double> py = {0, 1, 2, 3, 4};
    const auto out = pp.update(0.0, 0.0, 0.0, 2.0,
                               px.data(), py.data(), (int)px.size());
    EXPECT_LE(std::abs(out.steer), 0.30 + 1e-9);
}
