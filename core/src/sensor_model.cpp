// SensorModel implementation. See sensors.hpp.
#include <sstream>

#include "vdsim/snapshot.hpp"
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
    // normal_distribution caches the second Box-Muller value; without this the
    // noise stream depends on where the previous episode stopped.
    nd_.reset();
    b_ax_ = b_ay_ = b_az_ = b_wx_ = b_wy_ = b_wz_ = 0.0;
    b_w_ = {{0.0, 0.0, 0.0, 0.0}};
    b_st_ = b_gx_ = b_gy_ = b_gvx_ = b_gvy_ = 0.0;
    lag_.fill(0.0);
    lag_primed_ = false;
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
    auto meas = [&](double truth, const SensorNoise& n, double& rwbias, int li) {
        if (n.lag_tau_s > 0.0) {
            if (!lag_primed_) lag_[li] = truth;
            else if (dt > 0.0) lag_[li] += (1.0 - std::exp(-dt / n.lag_tau_s)) * (truth - lag_[li]);
            truth = lag_[li];
        }
        if (n.bias_rw > 0.0) rwbias += nd_(rng_) * n.bias_rw * sdt;
        const double w = (n.noise_std > 0.0) ? nd_(rng_) * n.noise_std : 0.0;
        return truth + n.bias + rwbias + w;
    };

    m.ax = meas(ax, p_.imu_accel, b_ax_, 0);
    m.ay = meas(ay, p_.imu_accel, b_ay_, 1);
    m.az = meas(kGravity, p_.imu_accel, b_az_, 2);
    m.wx = meas(s.angular_velocity.x(), p_.imu_gyro, b_wx_, 3);
    m.wy = meas(s.angular_velocity.y(), p_.imu_gyro, b_wy_, 4);
    m.wz = meas(s.angular_velocity.z(), p_.imu_gyro, b_wz_, 5);
    for (int i = 0; i < NUM_WHEELS; ++i)
        m.wheel_speed[i] = meas(s.wheel_spin[i], p_.wheel_speed, b_w_[i], 6 + i);
    m.steer = meas(steer_true, p_.steer, b_st_, 10);
    m.gnss_x = meas(s.position.x(), p_.gnss_pos, b_gx_, 11);
    m.gnss_y = meas(s.position.y(), p_.gnss_pos, b_gy_, 12);
    m.gnss_vx = meas(vE, p_.gnss_vel, b_gvx_, 13);
    m.gnss_vy = meas(vN, p_.gnss_vel, b_gvy_, 14);
    lag_primed_ = true;
    nd_.reset();   // R8: leave no un-snapshottable cache
    return m;
}

// ---- R8 snapshot ----------------------------------------------------------

void SensorModel::save_aux(std::vector<double>& v, std::string& rng) const {
    v.push_back(b_ax_); v.push_back(b_ay_); v.push_back(b_az_);
    v.push_back(b_wx_); v.push_back(b_wy_); v.push_back(b_wz_);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(b_w_[i]);
    v.push_back(b_st_); v.push_back(b_gx_); v.push_back(b_gy_);
    v.push_back(b_gvx_); v.push_back(b_gvy_);
    for (double x : lag_) v.push_back(x);
    v.push_back(lag_primed_ ? 1.0 : 0.0);
    std::ostringstream os;
    os << rng_;
    rng = os.str();
}

void SensorModel::restore_aux(const std::vector<double>& v, std::size_t& p,
                              const std::string& rng) {
    b_ax_ = snap::get(v, p); b_ay_ = snap::get(v, p); b_az_ = snap::get(v, p);
    b_wx_ = snap::get(v, p); b_wy_ = snap::get(v, p); b_wz_ = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) b_w_[i] = snap::get(v, p);
    b_st_ = snap::get(v, p); b_gx_ = snap::get(v, p); b_gy_ = snap::get(v, p);
    b_gvx_ = snap::get(v, p); b_gvy_ = snap::get(v, p);
    for (double& x : lag_) x = snap::get(v, p);
    lag_primed_ = snap::get(v, p) != 0.0;
    if (!rng.empty()) {
        std::istringstream is(rng);
        is >> rng_;
    }
    nd_.reset();
}

}  // namespace vdsim
