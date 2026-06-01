#include "vdsim/control_converter.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(LongAxController, ZeroErrorZeroOutput) {
    vdsim::LongAxController c;
    c.initialize({});
    const auto [t, b] = c.update(0.0, 0.0, 0.005);
    EXPECT_DOUBLE_EQ(t, 0.0);
    EXPECT_DOUBLE_EQ(b, 0.0);
}

TEST(LongAxController, PositiveTargetUsesThrottle) {
    vdsim::LongAxController c;
    c.initialize({});
    const auto [t, b] = c.update(2.0, 0.0, 0.005);
    EXPECT_GT(t, 0.0);
    EXPECT_DOUBLE_EQ(b, 0.0);
}

TEST(LongAxController, NegativeTargetUsesBrake) {
    vdsim::LongAxController c;
    c.initialize({});
    const auto [t, b] = c.update(-2.0, 0.0, 0.005);
    EXPECT_DOUBLE_EQ(t, 0.0);
    EXPECT_GT(b, 0.0);
}

TEST(LongAxController, IntegratorAccumulates) {
    vdsim::LongAxController c;
    c.initialize({});
    // sustained positive error builds throttle.
    double t0 = 0, t1 = 0;
    for (int i = 0; i < 5; ++i) {
        const auto [t, b] = c.update(1.0, 0.0, 0.01);
        if (i == 0) t0 = t;
        if (i == 4) t1 = t;
    }
    EXPECT_GT(t1, t0);
}

TEST(LongAxController, ResetClearsState) {
    vdsim::LongAxController c;
    c.initialize({});
    for (int i = 0; i < 100; ++i) c.update(1.0, 0.0, 0.005);
    const double i_before = c.integrator();
    EXPECT_GT(i_before, 0.0);
    c.reset();
    EXPECT_DOUBLE_EQ(c.integrator(), 0.0);
    EXPECT_DOUBLE_EQ(c.last_error(), 0.0);
}

TEST(LongAxController, OutputsAreClamped) {
    vdsim::LongAxController c;
    vdsim::LongAxController::Gains g;
    g.kp = 100.0;
    c.initialize(g);
    const auto [t, b] = c.update(10.0, 0.0, 0.005);
    EXPECT_LE(t, 1.0);
    EXPECT_GE(t, 0.0);
}

TEST(LongAxController, IntegratorAntiwindup) {
    vdsim::LongAxController c;
    vdsim::LongAxController::Gains g;
    g.kp = 0.0; g.ki = 1.0; g.kff = 0.0; g.i_max = 0.5;
    c.initialize(g);
    for (int i = 0; i < 1000; ++i) c.update(5.0, 0.0, 0.01);
    EXPECT_LE(std::abs(c.integrator()), 0.5 + 1e-9);
}
