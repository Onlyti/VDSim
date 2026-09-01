// Unit tests for the per-tick warning rate limiter (cosim/warn_throttle.hpp)
// used by the realtime server's comms TX loop.
#include <gtest/gtest.h>

#include "warn_throttle.hpp"

using vdsim::cosim::WarnThrottle;
using vdsim::cosim::warn_due;

TEST(WarnThrottle, FirstWarningAlwaysPasses) {
    WarnThrottle w;
    // Default state must fire whatever the clock reads, including t = 0 and a
    // large steady_clock uptime.
    EXPECT_TRUE(warn_due(0.0, w));
    WarnThrottle w2;
    EXPECT_TRUE(warn_due(1.0e9, w2));
}

TEST(WarnThrottle, SuppressesInsideThePeriodAndFiresAfterIt) {
    WarnThrottle w;
    ASSERT_TRUE(warn_due(100.0, w));
    EXPECT_EQ(w.suppressed, 0u);
    EXPECT_FALSE(warn_due(100.005, w));
    EXPECT_FALSE(warn_due(104.999, w));
    EXPECT_EQ(w.suppressed, 2u);
    EXPECT_TRUE(warn_due(105.0, w));
    // The running total survives the print so the printed line can quote it.
    EXPECT_EQ(w.suppressed, 2u);
    EXPECT_FALSE(warn_due(105.1, w));
    EXPECT_EQ(w.suppressed, 3u);
}

TEST(WarnThrottle, ThrottlesA200HzLoopToOneLinePerPeriod) {
    // The regression this exists for: 3 s of a 200 Hz TX loop used to print 600
    // identical lines per channel.
    WarnThrottle w;
    int printed = 0;
    for (int i = 0; i < 600; ++i)
        if (warn_due(i / 200.0, w)) ++printed;
    EXPECT_EQ(printed, 1);            // t = 0 .. 2.995 s spans one period
    EXPECT_EQ(w.suppressed, 599u);
    for (int i = 600; i < 2000; ++i)
        if (warn_due(i / 200.0, w)) ++printed;
    EXPECT_EQ(printed, 2);            // 10 s total -> a second line at t = 5 s
}

TEST(WarnThrottle, HonoursAnExplicitPeriod) {
    WarnThrottle w;
    ASSERT_TRUE(warn_due(0.0, w, 0.5));
    EXPECT_FALSE(warn_due(0.4, w, 0.5));
    EXPECT_TRUE(warn_due(0.5, w, 0.5));
    // A zero period disables throttling entirely.
    WarnThrottle none;
    EXPECT_TRUE(warn_due(1.0, none, 0.0));
    EXPECT_TRUE(warn_due(1.0, none, 0.0));
    EXPECT_EQ(none.suppressed, 0u);
}

TEST(WarnThrottle, EachCallSiteThrottlesIndependently) {
    WarnThrottle a, b;
    EXPECT_TRUE(warn_due(10.0, a));
    EXPECT_TRUE(warn_due(10.0, b));
    EXPECT_FALSE(warn_due(10.1, a));
    EXPECT_EQ(b.suppressed, 0u);
}
