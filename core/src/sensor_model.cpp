// SensorModel implementation. See sensors.hpp.
#include "vdsim/sensors.hpp"

#include <cmath>

#include "vdsim/coordinate.hpp"

namespace vdsim {
namespace {
constexpr double kGravity = 9.81;
}

void SensorModel::initialize(const SensorParams& p) {
    p_ = p;
    rng_.seed(p_.seed);
    reset();
}

void SensorModel::reset() {
    b_ax_ = b_ay_ = b_az_ = b_wx_ = b_wy_ = b_wz_ = 0.0;
    b_w_ = {{0.0, 0.0, 0.0, 0.0}};
    b_st_ = b_gx_ = b_gy_ = b_gvx_ = b_gvy_ = 0.0;
}

SensorMeas SensorModel::apply(const State& s, double ax, double ay,
                              double steer_true, double dt) {
    const double yaw = yaw_from_quat(s.orientation);
    const double vx = s.velocity.x(), vy = s.velocity.y();
    // World (ENU) ground velocity from body-frame vx, vy.
    const double vE = vx * std::cos(yaw) - vy * std::sin(yaw);
    const double vN = vx * std::sin(yaw) + vy * std::cos(yaw);

    SensorMeas m;
    if (!p_.enabled) {                       // identity: measured == truth
        m.ax = ax; m.ay = ay; m.az = kGravity;
        m.wx = s.angular_velocity.x(); m.wy = s.angular_velocity.y();
        m.wz = s.angular_velocity.z();
        for (int i = 0; i < NUM_WHEELS; ++i) m.wheel_speed[i] = s.wheel_spin[i];
        m.steer = steer_true;
        m.gnss_x = s.position.x(); m.gnss_y = s.position.y();
        m.gnss_vx = vE; m.gnss_vy = vN;
        return m;
    }

    const double sdt = std::sqrt(std::max(0.0, dt));
    auto meas = [&](double truth, const SensorNoise& n, double& rwbias) {
        if (n.bias_rw > 0.0) rwbias += nd_(rng_) * n.bias_rw * sdt;
        const double w = (n.noise_std > 0.0) ? nd_(rng_) * n.noise_std : 0.0;
        return truth + n.bias + rwbias + w;
    };

    m.ax = meas(ax, p_.imu_accel, b_ax_);
    m.ay = meas(ay, p_.imu_accel, b_ay_);
    m.az = meas(kGravity, p_.imu_accel, b_az_);
    m.wx = meas(s.angular_velocity.x(), p_.imu_gyro, b_wx_);
    m.wy = meas(s.angular_velocity.y(), p_.imu_gyro, b_wy_);
    m.wz = meas(s.angular_velocity.z(), p_.imu_gyro, b_wz_);
    for (int i = 0; i < NUM_WHEELS; ++i)
        m.wheel_speed[i] = meas(s.wheel_spin[i], p_.wheel_speed, b_w_[i]);
    m.steer = meas(steer_true, p_.steer, b_st_);
    m.gnss_x = meas(s.position.x(), p_.gnss_pos, b_gx_);
    m.gnss_y = meas(s.position.y(), p_.gnss_pos, b_gy_);
    m.gnss_vx = meas(vE, p_.gnss_vel, b_gvx_);
    m.gnss_vy = meas(vN, p_.gnss_vel, b_gvy_);
    return m;
}

}  // namespace vdsim
