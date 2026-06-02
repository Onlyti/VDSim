#include "vdsim/control_converter.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace vdsim {

void LongAxController::initialize(const Gains& g) noexcept {
    g_ = g;
    reset();
}

void LongAxController::reset() noexcept {
    integ_    = 0.0;
    prev_err_ = 0.0;
    first_    = true;
}

std::pair<double, double> LongAxController::update(double ax_target,
                                                    double ax_meas,
                                                    double dt) noexcept {
    if (!(dt > 0.0)) return {0.0, 0.0};
    const double e = ax_target - ax_meas;
    integ_ += e * dt;
    if (g_.ki > 0.0) {
        const double cap = g_.i_max;
        integ_ = std::clamp(integ_, -cap, cap);
    }
    double de_dt = 0.0;
    if (!first_) de_dt = (e - prev_err_) / dt;
    prev_err_ = e;
    first_ = false;

    const double u = g_.kp * e + g_.ki * integ_ + g_.kd * de_dt + g_.kff * ax_target;
    if (u >= 0.0) {
        return {std::clamp(u, 0.0, 1.0), 0.0};
    }
    return {0.0, std::clamp(-u, 0.0, 1.0)};
}

// =============================================================================
// L6 LongVxController
// =============================================================================

void LongVxController::initialize(const Gains& g) noexcept {
    g_ = g; reset();
}
void LongVxController::reset() noexcept {
    integ_ = 0.0; first_ = true;
}
double LongVxController::update(double v_target, double v_meas, double dt) noexcept {
    if (!(dt > 0.0)) return 0.0;
    const double e = v_target - v_meas;
    integ_ += e * dt;
    if (g_.ki > 0.0) integ_ = std::clamp(integ_, -g_.i_max, g_.i_max);
    first_ = false;
    const double ax = g_.kp * e + g_.ki * integ_;
    return std::clamp(ax, -g_.ax_clamp, g_.ax_clamp);
}

// =============================================================================
// L7 PurePursuitController
// =============================================================================

void PurePursuitController::initialize(const Gains& g) noexcept { g_ = g; }

PurePursuitController::Output PurePursuitController::update(
    double x, double y, double psi, double vx,
    const double* px, const double* py, int n,
    int prev_idx) const noexcept {
    Output out{0.0, 0.0, prev_idx};
    if (n <= 0 || !px || !py) return out;

    const double Ld = std::max(g_.lookahead_min, g_.lookahead_k * std::max(vx, 1.0));

    // Find first point along the path past arc-length Ld from (x, y), starting from prev_idx.
    int idx = std::clamp(prev_idx, 0, n - 1);
    for (; idx < n; ++idx) {
        const double dx = px[idx] - x;
        const double dy = py[idx] - y;
        if (std::sqrt(dx * dx + dy * dy) >= Ld) break;
    }
    if (idx >= n) idx = n - 1;
    out.idx = idx;

    // Transform lookahead point to vehicle body frame.
    const double cp = std::cos(psi), sp = std::sin(psi);
    const double dx =  cp * (px[idx] - x) + sp * (py[idx] - y);
    const double dy = -sp * (px[idx] - x) + cp * (py[idx] - y);
    const double l2 = dx * dx + dy * dy;
    if (l2 < 1e-6) return out;
    const double kappa = 2.0 * dy / l2;                    // curvature
    const double steer = std::atan(kappa * g_.wheelbase);
    out.steer    = std::clamp(steer, -g_.max_steer, g_.max_steer);
    out.curvature = kappa;
    return out;
}

// =============================================================================
// DriverModel
// =============================================================================

void DriverModel::initialize(const Gains& g) noexcept {
    g_ = g;
    PurePursuitController::Gains pg;
    pg.lookahead_min = g.lookahead_min;
    pg.lookahead_k   = g.lookahead_k;
    pg.wheelbase     = g.wheelbase;
    pg.max_steer     = g.max_steer;
    pp_.initialize(pg);
    LongVxController::Gains vxg;
    vxg.kp = g.vx_kp; vxg.ki = g.vx_ki;
    vxc_.initialize(vxg);
    axc_.initialize({});
    reset();
}

void DriverModel::reset() noexcept {
    steer_buffer_.assign(8, 0.0);
    steer_idx_ = 0;
    prev_idx_  = 0;
    vxc_.reset();
    axc_.reset();
}

DriverModel::Output DriverModel::update(double x, double y, double psi, double vx,
                                         double v_target,
                                         const double* px, const double* py, int n,
                                         double dt,
                                         double rand_a, double rand_b) noexcept {
    Output out{0.0, 0.0, 0.0};
    if (!(dt > 0.0) || n <= 0) return out;
    prev_dt_ = dt;

    // L7 raw steer. Resume the lookahead search from the last index so progress
    // is monotonic (avoids locking onto an earlier near-start segment on loops).
    const auto pp_out = pp_.update(x, y, psi, vx, px, py, n, prev_idx_);
    prev_idx_ = pp_out.idx;

    // Reaction time delay via ring buffer
    const int buf_size = std::max(1, (int)std::round(g_.reaction_time_s / dt));
    if ((int)steer_buffer_.size() != buf_size) steer_buffer_.assign(buf_size, 0.0);
    steer_buffer_[steer_idx_] = pp_out.steer;
    steer_idx_ = (steer_idx_ + 1) % buf_size;
    double delayed_steer = steer_buffer_[steer_idx_];

    // Two independent normals from one Box-Muller draw (cos + sin): z1 for steer,
    // z2 for the pedals, so steering and pedal noise are decorrelated.
    const double u1 = std::clamp(rand_a, 1e-6, 1.0 - 1e-6);
    const double u2 = rand_b;
    const double mag = std::sqrt(-2.0 * std::log(u1));
    const double z1  = mag * std::cos(2.0 * 3.141592653589793 * u2);
    const double z2  = mag * std::sin(2.0 * 3.141592653589793 * u2);
    delayed_steer += g_.steer_noise_rms * z1;
    delayed_steer = std::clamp(delayed_steer, -g_.max_steer, g_.max_steer);

    // L6 vx -> L5 ax -> throttle/brake. Noise applied only to the active pedal so
    // it cannot spawn phantom simultaneous throttle+brake.
    const double ax_tgt = vxc_.update(v_target, vx, dt);
    const auto [thr, brk] = axc_.update(ax_tgt, 0.0, dt);   // No ax_meas feedback (simplified)
    if (brk > 0.0) {
        out.brake    = std::clamp(brk + g_.thr_noise_rms * z2, 0.0, 1.0);
        out.throttle = 0.0;
    } else {
        out.throttle = std::clamp(thr + g_.thr_noise_rms * z2, 0.0, 1.0);
        out.brake    = 0.0;
    }
    out.steer = delayed_steer;
    return out;
}

}  // namespace vdsim
