// T2.1 — belt/carcass first-order slip relaxation primitive.

#include "vdsim/belt_tire.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(BeltTire, InstantWhenSigmaZero) {
    // sigma <= 0 disables the lag -> effective slip == geometric slip.
    EXPECT_DOUBLE_EQ(vdsim::belt_relax(0.0, 0.1, 20.0, 0.0, 0.001), 0.1);
}

TEST(BeltTire, FrozenAtStandstill) {
    // |Vx| = 0 -> decay = 1 -> the state holds (no relaxation rolling stationary).
    EXPECT_DOUBLE_EQ(vdsim::belt_relax(0.03, 0.1, 0.0, 0.5, 0.001), 0.03);
}

TEST(BeltTire, SingleStepMatchesAnalytic) {
    const double Vx = 20.0, sigma = 0.5, dt = 0.01;
    const double got = vdsim::belt_relax(0.0, 0.1, Vx, sigma, dt);
    const double want = 0.1 * (1.0 - std::exp(-Vx * dt / sigma));
    EXPECT_NEAR(got, want, 1e-12);
}

TEST(BeltTire, StepResponseReachesTauAndSteady) {
    const double Vx = 20.0, sigma = 0.5, dt = 1e-4;
    const double tau = sigma / Vx;            // 0.025 s
    const double sg = 0.1;
    double s = 0.0;
    for (double t = 0.0; t < tau - 1e-9; t += dt)
        s = vdsim::belt_relax(s, sg, Vx, sigma, dt);
    EXPECT_NEAR(s, sg * (1.0 - 1.0 / M_E), 0.005);   // ~63.2% of step at t = tau
    for (double t = tau; t < 5.0 * tau; t += dt)
        s = vdsim::belt_relax(s, sg, Vx, sigma, dt);
    EXPECT_NEAR(s, sg, 0.005);                        // ~steady by 5 tau
}

TEST(BeltTire, MonotoneNoOvershoot) {
    const double Vx = 15.0, sigma = 0.3, dt = 1e-3, sg = 0.08;
    double s = 0.0, prev = -1.0;
    for (int i = 0; i < 300; ++i) {
        s = vdsim::belt_relax(s, sg, Vx, sigma, dt);
        EXPECT_GE(s, prev);          // monotone approach
        EXPECT_LE(s, sg + 1e-12);    // never overshoots the geometric slip
        prev = s;
    }
}

TEST(BeltTire, FasterRelaxAtHigherSpeed) {
    // tau = sigma/|Vx|: at higher speed the same dt relaxes more.
    const double sigma = 0.4, dt = 0.005, sg = 0.1;
    const double slow = vdsim::belt_relax(0.0, sg, 5.0, sigma, dt);
    const double fast = vdsim::belt_relax(0.0, sg, 30.0, sigma, dt);
    EXPECT_GT(fast, slow);
}
