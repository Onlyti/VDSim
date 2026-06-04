// Sensor models: turn the true plant state into measured signals with
// configurable white noise, constant bias, and random-walk bias drift.
//
// Groups: IMU accel + gyro, per-wheel speed, steering angle, GNSS position +
// (ground/ENU) velocity. All effects default OFF (enabled=false) -> measured
// equals ground truth (identity), so this is backward-compatible.
//
// Sample-rate downsampling and dropouts are out of scope for this version; the
// transport delay on the feedback path is handled separately by SensorDelay.
#pragma once

#include <array>
#include <random>
#include <string>

#include "vdsim/state.hpp"

namespace vdsim {

// White Gaussian noise (1-sigma per sample) + constant bias + random-walk drift.
struct SensorNoise {
    double noise_std {0.0};   // measurement white noise 1-sigma
    double bias      {0.0};   // constant offset
    double bias_rw   {0.0};   // random-walk std rate: bias += N(0,1)*bias_rw*sqrt(dt)
};

struct SensorParams {
    bool     enabled {false}; // false -> measured == truth (identity)
    unsigned seed    {1};
    SensorNoise imu_accel;    // ax, ay, az  [m/s^2] (specific force)
    SensorNoise imu_gyro;     // wx, wy, wz  [rad/s]
    SensorNoise wheel_speed;  // per-wheel spin [rad/s]
    SensorNoise steer;        // road-wheel steer angle [rad]
    SensorNoise gnss_pos;     // world x, y [m]
    SensorNoise gnss_vel;     // world (ENU) vx, vy [m/s]

    static SensorParams from_yaml(const std::string& path);
    void to_yaml(const std::string& path) const;
};

// Measured sensor bundle (alongside ground truth in SimOutput).
struct SensorMeas {
    double ax {0.0}, ay {0.0}, az {0.0};
    double wx {0.0}, wy {0.0}, wz {0.0};
    std::array<double, NUM_WHEELS> wheel_speed {{0.0, 0.0, 0.0, 0.0}};
    double steer {0.0};
    double gnss_x {0.0}, gnss_y {0.0};
    double gnss_vx {0.0}, gnss_vy {0.0};
};

class SensorModel {
public:
    void initialize(const SensorParams& p);
    void reset();
    // s: true plant state; ax/ay: body accel diagnostics; steer_true: realized
    // road-wheel angle. Returns the measured bundle.
    SensorMeas apply(const State& s, double ax, double ay, double steer_true, double dt);

private:
    SensorParams p_{};
    std::mt19937 rng_{1};
    std::normal_distribution<double> nd_{0.0, 1.0};
    // random-walk bias integrator states
    double b_ax_{0}, b_ay_{0}, b_az_{0}, b_wx_{0}, b_wy_{0}, b_wz_{0};
    std::array<double, NUM_WHEELS> b_w_{{0, 0, 0, 0}};
    double b_st_{0}, b_gx_{0}, b_gy_{0}, b_gvx_{0}, b_gvy_{0};
};

}  // namespace vdsim
