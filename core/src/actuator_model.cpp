// Actuator dynamics + sensor delay implementation. See actuator.hpp.
#include "vdsim/actuator.hpp"

#include <algorithm>
#include <cmath>

namespace vdsim {
namespace {

inline double saturate(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// Slew-rate limit: move `out` toward `target` by at most rate*dt.
inline double rate_limit(double out, double target, double rate, double dt) {
    if (rate <= 0.0) return target;            // disabled
    const double dmax = rate * dt;
    const double d = target - out;
    if (d >  dmax) return out + dmax;
    if (d < -dmax) return out - dmax;
    return target;
}

// First-order lag: tau*dy/dt + y = u  (exact discrete update over dt).
inline double first_order(double y, double u, double tau, double dt) {
    if (tau <= 1e-9) return u;                  // no lag
    const double a = std::exp(-dt / tau);
    return a * y + (1.0 - a) * u;
}

// Piecewise-linear lookup (ascending x). Empty table -> default.
double lerp_table(const std::vector<double>& xs, const std::vector<double>& ys,
                  double x, double def) {
    const size_t n = std::min(xs.size(), ys.size());
    if (n == 0) return def;
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    for (size_t i = 1; i < n; ++i) {
        if (x <= xs[i]) {
            const double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
            return ys[i - 1] + t * (ys[i] - ys[i - 1]);
        }
    }
    return ys[n - 1];
}

}  // namespace

// ---- transport delay with fractional interpolation -----------------------
// buf holds the value history (oldest at front). Returns value L seconds ago.
double ActuatorModel::push_delay(std::vector<double>& buf, double v,
                                 double L, double dt) const {
    if (L <= 0.0) return v;                      // no delay
    const double fidx = L / dt;                  // delay in samples
    const int N = static_cast<int>(std::floor(fidx));
    const double f = fidx - N;                   // fractional part
    buf.push_back(v);
    const int maxlen = N + 3;
    if (static_cast<int>(buf.size()) > maxlen)
        buf.erase(buf.begin(), buf.end() - maxlen);
    const int last = static_cast<int>(buf.size()) - 1;
    // u[j] = 0 for j before the buffer existed (pre-command steady state), so a
    // step does not appear before the delay has elapsed (warmup correctness).
    auto val = [&](int idx) -> double { return (idx >= 0) ? buf[idx] : 0.0; };
    return (1.0 - f) * val(last - N) + f * val(last - N - 1);
}

void ActuatorModel::initialize(const ActuatorParams& p, double nominal_dt) {
    p_ = p;
    nominal_dt_ = (nominal_dt > 1e-6) ? nominal_dt : 0.005;
    reset();                                   // sets brake_T_ = T_ambient
    initialized_ = true;
}

void ActuatorModel::reset() {
    steer_buf_.clear(); throttle_buf_.clear(); brake_buf_.clear();
    steer_lag_ = throttle_lag_ = brake_lag_ = 0.0;
    steer_out_ = throttle_out_ = brake_out_ = 0.0;
    steer_pos_ = steer_vel_ = lugre_z_ = 0.0;
    brake_T_ = p_.brake.T_ambient;
}

CmdL4 ActuatorModel::apply(const CmdL4& desired, double speed_mps, double dt) {
    CmdL4 out = desired;
    if (!initialized_ || dt <= 0.0) return out;

    // ---------------- Steering ----------------
    double s_cmd = push_delay(steer_buf_, desired.steer_angle_wheel,
                              p_.steer.ch.dead_time_s, dt);
    if (p_.steer.friction.enabled) {
        // Torque-servo steering with LuGre friction (substepped).
        const auto& L = p_.steer.friction;
        const int nsub = std::max(1, static_cast<int>(std::ceil(dt / 1.0e-4)));
        const double h = dt / nsub;
        for (int i = 0; i < nsub; ++i) {
            const double w = steer_vel_;
            const double g = L.Tc + (L.Ts - L.Tc) * std::exp(-(w / L.ws) * (w / L.ws));
            // Semi-implicit bristle update: unconditionally stable for the
            // stiff sigma0 term. z_{k+1} = (z + h w)/(1 + h sigma0 |w|/g).
            const double z_new = (lugre_z_ + h * w)
                / (1.0 + h * L.sigma0 * std::fabs(w) / std::max(1e-6, g));
            const double dz = (z_new - lugre_z_) / h;            // dz/dt
            const double Tf = L.sigma0 * z_new + L.sigma1 * dz + L.sigma2 * w;
            const double Tservo = p_.steer.servo_kp * (s_cmd - steer_pos_)
                                  - p_.steer.servo_kd * w;
            const double acc = (Tservo - Tf) / std::max(1e-6, p_.steer.inertia);
            lugre_z_   = z_new;
            steer_vel_ += acc * h;
            steer_pos_ += steer_vel_ * h;
            // Hard travel stop with anti-windup: clamp the true servo position
            // and kill velocity into the stop (do not corrupt with an external
            // clip — steer_pos_ stays the physical integrator state).
            if (steer_pos_ > p_.steer.ch.out_max) {
                steer_pos_ = p_.steer.ch.out_max;
                if (steer_vel_ > 0.0) steer_vel_ = 0.0;
            } else if (steer_pos_ < p_.steer.ch.out_min) {
                steer_pos_ = p_.steer.ch.out_min;
                if (steer_vel_ < 0.0) steer_vel_ = 0.0;
            }
        }
        // Optional external slew cap on the realized output; servo state
        // (steer_pos_) is left intact so the integrator never desyncs.
        steer_out_ = (p_.steer.ch.rate_limit > 0.0)
            ? rate_limit(steer_out_, steer_pos_, p_.steer.ch.rate_limit, dt)
            : steer_pos_;
    } else {
        double y = first_order(steer_lag_, s_cmd, p_.steer.ch.tau_s, dt);
        steer_lag_ = y;
        y = rate_limit(steer_out_, y, p_.steer.ch.rate_limit, dt);
        steer_out_ = saturate(y, p_.steer.ch.out_min, p_.steer.ch.out_max);
        steer_pos_ = steer_out_;
    }
    out.steer_angle_wheel = steer_out_;

    // ---------------- Throttle ----------------
    {
        double t_in = desired.throttle;
        // Dead-zone (pedal tip-in / motor torque threshold): below it -> no effect.
        const double tdz = p_.throttle.dead_zone;
        if (tdz > 0.0) {
            t_in = (t_in > tdz) ? (t_in - tdz) / std::max(1e-6, 1.0 - tdz) : 0.0;
        }
        double t_cmd = push_delay(throttle_buf_, t_in,
                                  p_.throttle.dead_time_s, dt);
        double y = first_order(throttle_lag_, t_cmd, p_.throttle.tau_s, dt);
        throttle_lag_ = y;
        y = rate_limit(throttle_out_, y, p_.throttle.rate_limit, dt);
        const double lo = std::max(0.0, p_.throttle.out_min);
        const double hi = std::min(1.0, p_.throttle.out_max);
        throttle_out_ = saturate(y, lo, hi);
        out.throttle = throttle_out_;
    }

    // ---------------- Brake ----------------
    {
        double b_in = desired.brake;
        // Dead-zone (pad clearance fill): below threshold -> no effect.
        const double dz = p_.brake.ch.dead_zone;
        if (dz > 0.0) {
            b_in = (b_in > dz) ? (b_in - dz) / std::max(1e-6, 1.0 - dz) : 0.0;
        }
        double b_cmd = push_delay(brake_buf_, b_in, p_.brake.ch.dead_time_s, dt);
        double y = first_order(brake_lag_, b_cmd, p_.brake.ch.tau_s, dt);
        brake_lag_ = y;
        y = rate_limit(brake_out_, y, p_.brake.ch.rate_limit, dt);

        // Temperature fade: integrate brake temperature, scale effectiveness.
        double mu_scale = 1.0;
        if (p_.brake.thermal_enabled) {
            const double heat = p_.brake.heat_coeff * std::fabs(y) * std::fabs(speed_mps);
            const double cool = p_.brake.cool_coeff * (brake_T_ - p_.brake.T_ambient);
            brake_T_ += (heat - cool) * dt;
            mu_scale = lerp_table(p_.brake.mu_T_temp, p_.brake.mu_T_scale, brake_T_, 1.0);
        }
        const double lo = std::max(0.0, p_.brake.ch.out_min);
        const double hi = std::min(1.0, p_.brake.ch.out_max);
        brake_out_ = saturate(y, lo, hi);
        out.brake = saturate(brake_out_ * mu_scale, 0.0, 1.0);
    }

    return out;
}

// ---------------- SensorDelay ----------------
void SensorDelay::initialize(double delay_s, double nominal_dt) {
    delay_s_ = std::max(0.0, delay_s);
    nominal_dt_ = (nominal_dt > 1e-6) ? nominal_dt : 0.005;
    buf_.clear();
    initialized_ = true;
}

void SensorDelay::reset(const State& s) {
    buf_.assign(1, s);
}

State SensorDelay::apply(const State& measured, double dt) {
    if (!initialized_ || delay_s_ <= 0.0 || dt <= 0.0) return measured;
    buf_.push_back(measured);
    // Whole-State snapshots cannot be interpolated, so the sensor delay is
    // quantized to the nearest sample (round), unlike the scalar command delay
    // which interpolates fractionally. Intentional.
    const int N = static_cast<int>(std::round(delay_s_ / dt));
    const int maxlen = N + 2;
    if (static_cast<int>(buf_.size()) > maxlen)
        buf_.erase(buf_.begin(), buf_.end() - maxlen);
    const int idx = static_cast<int>(buf_.size()) - 1 - N;
    return buf_[std::max(0, idx)];
}

}  // namespace vdsim
