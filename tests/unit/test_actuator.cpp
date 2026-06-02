// Unit tests for the actuator dynamics + sensor delay layer.
// Uses synthetic parameters only (no measured/confidential data).
#include <gtest/gtest.h>

#include <cmath>

#include "vdsim/actuator.hpp"

using namespace vdsim;

static CmdL4 cmd(double thr, double brk, double steer) {
    CmdL4 c; c.throttle = thr; c.brake = brk; c.steer_angle_wheel = steer; return c;
}

// Default params -> identity passthrough (backward compatible).
TEST(Actuator, DefaultIsIdentity) {
    ActuatorModel a; a.initialize(ActuatorParams{}, 0.005);
    auto out = a.apply(cmd(0.4, 0.0, 0.1), 20.0, 0.005);
    EXPECT_DOUBLE_EQ(out.throttle, 0.4);
    EXPECT_DOUBLE_EQ(out.steer_angle_wheel, 0.1);
}

// Pure dead-time: step appears only after L seconds (N steps).
TEST(Actuator, DeadTimeDelaysStep) {
    ActuatorParams p;
    p.throttle.dead_time_s = 0.05;          // 10 steps at dt=0.005
    ActuatorModel a; a.initialize(p, 0.005);
    double last = 0.0;
    int first_nonzero = -1;
    for (int k = 0; k < 30; ++k) {
        auto out = a.apply(cmd(1.0, 0, 0), 0.0, 0.005);
        if (first_nonzero < 0 && out.throttle > 1e-6) first_nonzero = k;
        last = out.throttle;
    }
    EXPECT_GE(first_nonzero, 9);             // ~10-step delay
    EXPECT_NEAR(last, 1.0, 1e-9);            // eventually reaches command
}

// First-order lag: monotonic rise toward command, ~63% at t=tau.
TEST(Actuator, FirstOrderLagRises) {
    ActuatorParams p;
    p.throttle.tau_s = 0.1;
    ActuatorModel a; a.initialize(p, 0.005);
    double y = 0.0;
    for (int k = 0; k < 20; ++k) y = a.apply(cmd(1.0, 0, 0), 0.0, 0.005).throttle;  // t=0.1=tau
    EXPECT_GT(y, 0.55);
    EXPECT_LT(y, 0.72);                      // ~0.632
}

// Rate limit caps the slew of the output.
TEST(Actuator, RateLimitCapsSlew) {
    ActuatorParams p;
    p.steer.ch.rate_limit = 2.0;            // rad/s
    ActuatorModel a; a.initialize(p, 0.005);
    auto out = a.apply(cmd(0, 0, 1.0), 0.0, 0.005);  // big step
    EXPECT_NEAR(out.steer_angle_wheel, 2.0 * 0.005, 1e-9);
}

// Saturation clamps output.
TEST(Actuator, SaturationClamps) {
    ActuatorParams p;
    p.steer.ch.out_max = 0.3;
    ActuatorModel a; a.initialize(p, 0.005);
    double y = 0;
    for (int k = 0; k < 50; ++k) y = a.apply(cmd(0, 0, 1.0), 0.0, 0.005).steer_angle_wheel;
    EXPECT_LE(y, 0.3 + 1e-9);
}

// Brake dead-zone: small command below threshold yields zero brake.
TEST(Actuator, BrakeDeadZone) {
    ActuatorParams p;
    p.brake.dead_zone = 0.1;
    ActuatorModel a; a.initialize(p, 0.005);
    double y_small = 0, y_big = 0;
    for (int k = 0; k < 50; ++k) y_small = a.apply(cmd(0, 0.05, 0), 0.0, 0.005).brake;
    a.reset();
    for (int k = 0; k < 50; ++k) y_big = a.apply(cmd(0, 0.5, 0), 0.0, 0.005).brake;
    EXPECT_NEAR(y_small, 0.0, 1e-9);
    EXPECT_GT(y_big, 0.0);
}

// Brake mu(T) fade: effectiveness drops as temperature rises with hard braking.
TEST(Actuator, BrakeThermalFade) {
    ActuatorParams p;
    p.brake.thermal_enabled = true;
    p.brake.heat_coeff = 1.0;
    p.brake.cool_coeff = 0.0;               // no cooling -> monotone heating
    p.brake.T_ambient = 40.0;
    p.brake.mu_T_temp  = {40.0, 200.0, 400.0};
    p.brake.mu_T_scale = {1.0, 1.0, 0.5};   // fade above 200 degC
    ActuatorModel a; a.initialize(p, 0.005);
    double y_cold = a.apply(cmd(0, 1.0, 0), 30.0, 0.005).brake;
    for (int k = 0; k < 20000; ++k) a.apply(cmd(0, 1.0, 0), 30.0, 0.005);  // heat up
    double y_hot = a.apply(cmd(0, 1.0, 0), 30.0, 0.005).brake;
    EXPECT_GT(a.brake_temperature(), 200.0);
    EXPECT_LT(y_hot, y_cold);               // faded
}

// LuGre steering: output tracks command and shows stiction (lags small inputs).
TEST(Actuator, LuGreSteeringTracksAndIsFinite) {
    ActuatorParams p;
    p.steer.friction.enabled = true;
    ActuatorModel a; a.initialize(p, 0.005);
    double y = 0;
    for (int k = 0; k < 400; ++k) y = a.apply(cmd(0, 0, 0.2), 0.0, 0.005).steer_angle_wheel;
    EXPECT_TRUE(std::isfinite(y));
    EXPECT_NEAR(y, 0.2, 0.05);              // converges near command
}

// Sensor delay returns an older state snapshot.
TEST(Actuator, SensorDelayReturnsPast) {
    SensorDelay s; s.initialize(0.05, 0.005);  // 10-step delay
    State st;
    double seen_vx = -1;
    for (int k = 0; k < 30; ++k) {
        st.velocity = Vec3(static_cast<double>(k), 0, 0);
        State out = s.apply(st, 0.005);
        if (k == 20) seen_vx = out.vx();
    }
    EXPECT_NEAR(seen_vx, 10.0, 1.0 + 1e-9);  // ~10 steps behind k=20
}
