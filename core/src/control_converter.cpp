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

// =============================================================================
// CascadeController
// =============================================================================

void CascadeController::initialize(const VehicleParams& vp) {
    wheelbase_ = vp.wheelbase > 1e-3 ? vp.wheelbase : 2.7;
    max_steer_ = vp.max_steer_angle_wheel > 1e-3 ? vp.max_steer_angle_wheel : 0.5;
    vxc_.initialize({});
    axc_.initialize({});
    PurePursuitController::Gains pg;
    pg.wheelbase = wheelbase_;
    pg.max_steer = max_steer_;
    pp_.initialize(pg);
    reset();
}

void CascadeController::reset() noexcept {
    vxc_.reset();
    axc_.reset();
    pp_idx_ = 0;
    r_integ_ = 0.0;
}

// ---- Independent longitudinal cascade: LcLon → throttle/brake/gear ----
void CascadeController::lon_to_pedals(const LcLonCmd& lon, const State& meas,
                                      double ax_meas, double dt, CmdL4& out) {
    const double vx = meas.vx();
    std::visit([&](const auto& c) {
        using T = std::decay_t<decltype(c)>;
        if constexpr (std::is_same_v<T, LcLonL1>) {
            const double Td = c.wheel_torque[0] + c.wheel_torque[1]
                            + c.wheel_torque[2] + c.wheel_torque[3];
            out.throttle = std::clamp(Td / 600.0, 0.0, 1.0);
            out.brake    = std::clamp(-Td / 4000.0, 0.0, 1.0);
        } else if constexpr (std::is_same_v<T, LcLonL2>) {
            out.throttle = std::clamp(c.axle_torque / 600.0, 0.0, 1.0);
            out.brake    = std::clamp(-c.axle_torque / 4000.0, 0.0, 1.0);
        } else if constexpr (std::is_same_v<T, LcLonL3>) {
            const double s = c.Fx_total / (1500.0 * 5.0);
            out.throttle = std::clamp(s, 0.0, 1.0);
            out.brake    = std::clamp(-s, 0.0, 1.0);
        } else if constexpr (std::is_same_v<T, LcLonL4>) {
            out.throttle = c.throttle; out.brake = c.brake; out.gear = c.gear;
        } else if constexpr (std::is_same_v<T, LcLonL5>) {
            const auto [thr, brk] = axc_.update(c.ax_target, ax_meas, dt);
            out.throttle = thr; out.brake = brk;
        } else if constexpr (std::is_same_v<T, LcLonL6>) {
            const double ax_tgt = vxc_.update(c.vx_target, vx, dt);
            const auto [thr, brk] = axc_.update(ax_tgt, ax_meas, dt);
            out.throttle = thr; out.brake = brk;
        }
    }, lon);
}

// ---- Independent lateral cascade ----
// L4-L8 → steer angle (out.steer_angle_wheel, Angle mode).
// L1-L3 → sub-L4 actuator command (out.steer_mode + steer_actuator) passed through
//         to a Dynamic steering subsystem (the cascade can't lower these to an angle
//         without integrating, which is the subsystem's Rack EOM job).
void CascadeController::lat_to_cmd(const LcLatCmd& lat, const State& meas, double dt,
                                   CmdL4& out) {
    const double vx  = meas.vx();
    const double px  = meas.position.x();
    const double py  = meas.position.y();
    const double yaw = meas.yaw();
    std::visit([&](const auto& c) {
        using T = std::decay_t<decltype(c)>;
        if constexpr (std::is_same_v<T, LcLatL1>) {
            out.steer_mode = SteerMode::Torque;   out.steer_actuator = c.steer_torque;
        } else if constexpr (std::is_same_v<T, LcLatL2>) {
            out.steer_mode = SteerMode::AngAccel; out.steer_actuator = c.steer_ang_accel;
        } else if constexpr (std::is_same_v<T, LcLatL3>) {
            out.steer_mode = SteerMode::AngVel;   out.steer_actuator = c.steer_ang_vel;
        } else if constexpr (std::is_same_v<T, LcLatL4>) {
            out.steer_angle_wheel = c.steer_angle;
        } else if constexpr (std::is_same_v<T, LcLatL5>) {
            const double r_tgt = c.ay_target / std::max(std::abs(vx), 1.0);
            const double e = r_tgt - meas.yaw_rate();
            r_integ_ = std::clamp(r_integ_ + e * dt, -2.0, 2.0);
            out.steer_angle_wheel = std::clamp(r_kp_ * e + r_ki_ * r_integ_, -max_steer_, max_steer_);
        } else if constexpr (std::is_same_v<T, LcLatL6>) {
            const double e = c.r_target - meas.yaw_rate();
            r_integ_ = std::clamp(r_integ_ + e * dt, -2.0, 2.0);
            out.steer_angle_wheel = std::clamp(r_kp_ * e + r_ki_ * r_integ_, -max_steer_, max_steer_);
        } else if constexpr (std::is_same_v<T, LcLatL7>) {
            out.steer_angle_wheel = std::clamp(std::atan(c.kappa * wheelbase_), -max_steer_, max_steer_);
        } else if constexpr (std::is_same_v<T, LcLatL8>) {
            const int n = static_cast<int>(c.path.size());
            if (n > 0) {
                std::vector<double> xs(n), ys(n);
                for (int i = 0; i < n; ++i) { xs[i] = c.path[i].xy.x(); ys[i] = c.path[i].xy.y(); }
                const auto o = pp_.update(px, py, yaw, vx, xs.data(), ys.data(), n, pp_idx_);
                pp_idx_ = o.idx;
                out.steer_angle_wheel = o.steer;
            }
        }
    }, lat);
}

CmdL4 CascadeController::to_l4(const ControlInput& u, const State& meas,
                              double ax_meas, double dt) {
    const double vx  = meas.vx();
    const double px  = meas.position.x();
    const double py  = meas.position.y();
    const double yaw = meas.yaw();

    return std::visit([&](const auto& cmd) -> CmdL4 {
        using T = std::decay_t<decltype(cmd)>;
        CmdL4 out;
        // ---- L1-L4: stateless lowering (no feedback) ----
        if constexpr (std::is_same_v<T, CmdL1>) {
            const double T_drive = cmd.motor_torque[0] + cmd.motor_torque[1]
                                 + cmd.motor_torque[2] + cmd.motor_torque[3];
            const double T_brake = cmd.brake_torque[0] + cmd.brake_torque[1]
                                 + cmd.brake_torque[2] + cmd.brake_torque[3];
            out.throttle = std::clamp(T_drive / 600.0, 0.0, 1.0);
            out.brake    = std::clamp(T_brake / 4000.0 - std::min(0.0, T_drive) / 4000.0,
                                       0.0, 1.0);
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL2>) {
            out.throttle = std::clamp(cmd.drive_torque / 600.0, 0.0, 1.0);
            out.brake    = std::clamp(cmd.brake_torque / 4000.0
                                       - std::min(0.0, cmd.drive_torque) / 4000.0, 0.0, 1.0);
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL3>) {
            const double scale = cmd.Fx_total / (1500.0 * 5.0);
            out.throttle = std::clamp(scale, 0.0, 1.0);
            out.brake    = std::clamp(-scale, 0.0, 1.0);
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL4>) {
            out = cmd;
        }
        // ---- L5: ax_target → throttle/brake (LongAx, needs ax_meas) ----
        else if constexpr (std::is_same_v<T, CmdL5>) {
            const auto [thr, brk] = axc_.update(cmd.ax_target, ax_meas, dt);
            out.throttle = thr; out.brake = brk;
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        }
        // ---- L6: vx_target → ax_target → throttle/brake (LongVx → LongAx) ----
        else if constexpr (std::is_same_v<T, CmdL6>) {
            const double ax_tgt = vxc_.update(cmd.v_target, vx, dt);
            const auto [thr, brk] = axc_.update(ax_tgt, ax_meas, dt);
            out.throttle = thr; out.brake = brk;
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        }
        // ---- L7: (v_target, kappa) → speed cascade + steer = atan(kappa·L) ----
        else if constexpr (std::is_same_v<T, CmdL7>) {
            const double ax_tgt = vxc_.update(cmd.v_target, vx, dt);
            const auto [thr, brk] = axc_.update(ax_tgt, ax_meas, dt);
            out.throttle = thr; out.brake = brk;
            out.steer_angle_wheel = std::clamp(std::atan(cmd.kappa * wheelbase_),
                                               -max_steer_, max_steer_);
        }
        // ---- L8: waypoint path → PurePursuit steer + per-point speed cascade ----
        else if constexpr (std::is_same_v<T, CmdL8>) {
            const int n = static_cast<int>(cmd.path.size());
            if (n > 0) {
                std::vector<double> xs(n), ys(n);
                for (int i = 0; i < n; ++i) { xs[i] = cmd.path[i].xy.x(); ys[i] = cmd.path[i].xy.y(); }
                const auto pp_out = pp_.update(px, py, yaw, vx, xs.data(), ys.data(), n, pp_idx_);
                pp_idx_ = pp_out.idx;
                out.steer_angle_wheel = pp_out.steer;
                const double v_des = cmd.path[pp_out.idx].v_des;
                const double ax_tgt = vxc_.update(v_des, vx, dt);
                const auto [thr, brk] = axc_.update(ax_tgt, ax_meas, dt);
                out.throttle = thr; out.brake = brk;
            }
        }
        // ---- CmdSplit: independent lon + lat cascades ----
        else if constexpr (std::is_same_v<T, CmdSplit>) {
            lon_to_pedals(cmd.lon, meas, ax_meas, dt, out);
            lat_to_cmd(cmd.lat, meas, dt, out);
        }
        return out;
    }, u);
}

}  // namespace vdsim
