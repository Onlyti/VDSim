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

TEST(LongAxController, DirectedForwardIsUpdateBitForBit) {
    // Forward and the standstill band keep the plain mapping with gear +1.
    for (double vx : {3.0, 0.0, -0.5 * vdsim::LongAxController::kReverseVx}) {
        vdsim::LongAxController a, b;
        a.initialize({});
        b.initialize({});
        for (int i = 0; i < 50; ++i) {
            const double tgt = (i % 7) * 0.4 - 1.2, meas = (i % 5) * 0.3 - 0.6;
            const auto [t, br] = a.update(tgt, meas, 0.005);
            const auto pd = b.update_directed(tgt, meas, vx, 0.005);
            EXPECT_EQ(pd.throttle, t);
            EXPECT_EQ(pd.brake, br);
            EXPECT_EQ(pd.gear, 1);
        }
        EXPECT_EQ(a.integrator(), b.integrator());
    }
}

TEST(LongAxController, DirectedReverseBrakesToSlowAndDrivesBackward) {
    vdsim::LongAxController c;
    c.initialize({});
    // Reversing, positive ax demand slows the backward motion -> brake.
    const auto slow = c.update_directed(1.0, 0.0, -3.0, 0.005);
    EXPECT_EQ(slow.gear, -1);
    EXPECT_EQ(slow.throttle, 0.0);
    EXPECT_GT(slow.brake, 0.0);
    c.reset();
    // Reversing, negative ax demand speeds it up backward -> throttle in reverse.
    const auto fast = c.update_directed(-1.0, 0.0, -3.0, 0.005);
    EXPECT_EQ(fast.gear, -1);
    EXPECT_GT(fast.throttle, 0.0);
    EXPECT_EQ(fast.brake, 0.0);
    // Same body-frame command magnitude as the forward mirror.
    vdsim::LongAxController f;
    f.initialize({});
    const auto [ft, fb] = f.update(-1.0, 0.0, 0.005);
    EXPECT_EQ(slow.brake, fb);
    EXPECT_EQ(ft, 0.0);
}

TEST(LongAxController, DirectedZeroDtReturnsZeroPedals) {
    vdsim::LongAxController c;
    c.initialize({});
    const auto pd = c.update_directed(1.0, 0.0, -3.0, 0.0);
    EXPECT_EQ(pd.throttle, 0.0);
    EXPECT_EQ(pd.brake, 0.0);
    EXPECT_EQ(c.integrator(), 0.0);
}

TEST(LongAxController, IntegratorAntiwindup) {
    vdsim::LongAxController c;
    vdsim::LongAxController::Gains g;
    g.kp = 0.0; g.ki = 1.0; g.kff = 0.0; g.i_max = 0.5;
    c.initialize(g);
    for (int i = 0; i < 1000; ++i) c.update(5.0, 0.0, 0.01);
    EXPECT_LE(std::abs(c.integrator()), 0.5 + 1e-9);
}
