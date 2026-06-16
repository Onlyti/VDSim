#include "vdsim/belt_tire.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/default_subsystems.hpp"
#include "vdsim/drivetrain_inertia.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/lugre_tire.hpp"
#include "vdsim/subsystems.hpp"
#include "vdsim/tire_contact.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <numeric>
#include <type_traits>
#include <variant>

namespace vdsim {

namespace {

inline CmdL4 lower_to_l4(const ControlInput& u) {
    return std::visit([](const auto& cmd) -> CmdL4 {
        using T = std::decay_t<decltype(cmd)>;
        CmdL4 out;
        if constexpr (std::is_same_v<T, CmdL1>) {
            const double T_drive = std::accumulate(cmd.motor_torque.begin(),
                                                    cmd.motor_torque.end(), 0.0);
            const double T_brake = std::accumulate(cmd.brake_torque.begin(),
                                                    cmd.brake_torque.end(), 0.0);
            out.throttle = std::clamp(T_drive / 600.0, 0.0, 1.0);
            out.brake    = std::clamp(T_brake / 4000.0 - std::min(0.0, T_drive) / 4000.0,
                                       0.0, 1.0);
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL2>) {
            out.throttle = std::clamp(cmd.drive_torque / 600.0, 0.0, 1.0);
            out.brake    = std::clamp(cmd.brake_torque / 4000.0
                                       - std::min(0.0, cmd.drive_torque) / 4000.0,
                                       0.0, 1.0);
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL3>) {
            const double scale = cmd.Fx_total / (1500.0 * 5.0);
            out.throttle = std::clamp(scale, 0.0, 1.0);
            out.brake    = std::clamp(-scale, 0.0, 1.0);
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL4>) {
            out = cmd;
        }
        return out;
    }, u);
}

constexpr double kAirDensity = 1.225;
constexpr double kGravity    = 9.80665;
constexpr double kSpeedEps     = 0.15;
constexpr double kStickBlend   = 3.0;
constexpr double kStickC       = 6.0e4;
constexpr double kWheelSpinDrag = 2.5;

class Free3DDynamics final : public IVehicleDynamics {
public:
    Free3DDynamics() : tire_(create_pacejka_mf96()) {}

    Level level() const noexcept override { return Level::L5_Stunt; }

    void initialize(const VehicleParams& vp,
                    const TireParams& tp,
                    const SolverParams& sp) override {
        vp_ = vp;
        tp_ = tp;
        sp_ = sp;
        if (tp.backend != "mf96" && !tp.backend.empty())
            tire_ = create_tire_from_params(tp);
        tire_->initialize(tp);
        const double R = vp.wheel_radius_nominal;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            if (vp.wheel_inertia[i] > 0.0) {
                I_wheel_[i] = vp.wheel_inertia[i];
            } else {
                const double m_w = vp.unsprung_mass[i] > 0.0 ? vp.unsprung_mass[i] : 25.0;
                I_wheel_[i] = std::max(0.01, 0.5 * m_w * R * R);
            }
        }
        const double a = vp.cg_to_front, b = vp.cg_to_rear;
        const double tw_f = vp.track_front * 0.5, tw_r = vp.track_rear * 0.5;
        const double hz  = -(vp.cg_height - R);
        r_body_[WHEEL_FL] = Vec3(a,  tw_f, hz);
        r_body_[WHEEL_FR] = Vec3(a, -tw_f, hz);
        r_body_[WHEEL_RL] = Vec3(-b,  tw_r, hz);
        r_body_[WHEEL_RR] = Vec3(-b, -tw_r, hz);
        I_body_inv_ = Vec3(
            1.0 / std::max(1.0, vp.inertia_diag.x()),
            1.0 / std::max(1.0, vp.inertia_diag.y()),
            1.0 / std::max(1.0, vp.inertia_diag.z()));
        drivetrain_ = make_default_drivetrain(vp_, vp_.drive_deadtime_s);
        brake_      = make_default_brake(vp_, vp_.brake_deadtime_s);
        steering_   = make_default_steering(vp_, vp_.steer_deadtime_s);
        spdlog::debug("[L5 free-3D] init: mass={:.0f} kg, I=({:.0f},{:.0f},{:.0f})",
                      vp.mass, vp.inertia_diag.x(), vp.inertia_diag.y(), vp.inertia_diag.z());
    }

    void reset(const State& s) noexcept override {
        state_ = s;
        state_.orientation.normalize();
        ax_prev_ = 0.0;
        ay_prev_ = 0.0;
        belt_kappa_.fill(0.0);
        belt_alpha_.fill(0.0);
        belt_vlong_.fill(0.0);
        belt_vlat_.fill(0.0);
        kappa_geom_last_.fill(0.0);
        alpha_geom_last_.fill(0.0);
        v_slip_long_last_.fill(0.0);
        v_slip_lat_last_.fill(0.0);
        vx_belt_last_.fill(0.0);
        lugre_z_long_.fill(0.0);
        lugre_z_lat_.fill(0.0);
        drivetrain_ = make_default_drivetrain(vp_, vp_.drive_deadtime_s);
        brake_      = make_default_brake(vp_, vp_.brake_deadtime_s);
        steering_   = make_default_steering(vp_, vp_.steer_deadtime_s);
        tire_F_.fill(Vec3::Zero());
        tire_Fz_.fill(0.0);
        slip_ratio_.fill(0.0);
        slip_angle_.fill(0.0);
        mz_front_sum_ = 0.0;
    }

    void step(const ControlInput& u,
              const ContactArray& contacts,
              double dt) noexcept override {
        const CmdL4 cmd = lower_to_l4(u);
        if (!(dt > 0.0)) return;

        const DriverCmd driver_cmd{
            cmd.steer_angle_wheel, cmd.throttle, cmd.brake, cmd.gear, cmd.handbrake};
        SubsystemContext ctx{state_, driver_cmd, dt};
        brake_->begin_step(ctx, dt);
        drivetrain_->begin_step(ctx, dt);
        steering_->begin_step(ctx, dt);

        const int N = std::max(1,
                       std::min(sp_.max_substeps,
                                static_cast<int>(std::ceil(dt / sp_.max_substep_dt))));
        const double h = dt / static_cast<double>(N);
        for (int i = 0; i < N; ++i) substep(cmd, contacts, h);
    }

    const State& state() const noexcept override { return state_; }

    // User-defined modules. L5 carries the planar actuator subsystems (no vertical
    // suspension/ARB objects), so brake/steering/drivetrain can be replaced.
    bool set_brake_module(std::shared_ptr<IBrakeSystem> m) override {
        if (!m) return false;
        brake_ = std::move(m);
        return true;
    }
    bool set_steering_module(std::shared_ptr<ISteeringSystem> m) override {
        if (!m) return false;
        steering_ = std::move(m);
        return true;
    }
    bool set_drivetrain_module(std::shared_ptr<IDrivetrain> m) override {
        if (!m) return false;
        drivetrain_ = std::move(m);
        return true;
    }

    std::array<Vec3, NUM_WHEELS>   tire_forces_body() const override { return tire_F_; }
    std::array<double, NUM_WHEELS> tire_Fz()           const override { return tire_Fz_; }
    std::array<double, NUM_WHEELS> wheel_slip_ratio()  const override { return slip_ratio_; }
    std::array<double, NUM_WHEELS> wheel_slip_angle()  const override { return slip_angle_; }

    double roll_angle_qs()  const override {
        return euler_from_quat(state_.orientation).roll;
    }
    double pitch_angle_qs() const override {
        return euler_from_quat(state_.orientation).pitch;
    }
    double ax_body_est() const override { return ax_prev_; }
    double ay_body_est() const override { return ay_prev_; }
    double steering_rack_torque() const override {
        return mz_front_sum_ / std::max(1e-6, vp_.steering_ratio);
    }

private:
    struct Deriv {
        Vec3 dp_world {Vec3::Zero()};
        Vec3 d_v_body {Vec3::Zero()};
        Vec3 d_omega  {Vec3::Zero()};
        double ax_body {0.0};
        double ay_body {0.0};
        std::array<double, NUM_WHEELS> domega {{0.0, 0.0, 0.0, 0.0}};
    };

    static Vec3 wheel_tangent_frame(const Vec3& n_in, const Vec3& body_fwd,
                                    double steer, Vec3& t_lat_out) {
        Vec3 n = n_in.normalized();
        Vec3 t_long = body_fwd - n * body_fwd.dot(n);
        if (t_long.squaredNorm() < 1e-12) {
            Vec3 lat = n.cross(Vec3::UnitX());
            if (lat.squaredNorm() < 1e-12) lat = n.cross(Vec3::UnitY());
            t_long = lat.cross(n);
        }
        t_long.normalize();
        if (std::abs(steer) > 1e-12) {
            const Quat q(Eigen::AngleAxisd(steer, n));
            t_long = q * t_long;
        }
        t_lat_out = n.cross(t_long).normalized();
        return t_long;
    }

    static Vec3 loop_track_tangent(const Vec3& hub_world, const Vec3& v_hub_world,
                                   double xc, double zc) {
        const double dx = hub_world.x() - xc;
        const double dz = hub_world.z() - zc;
        const double th = std::atan2(dx, -dz);
        Vec3 t(std::cos(th), 0.0, std::sin(th));
        if (v_hub_world.dot(t) < 0.0) t = -t;
        return t;
    }

    Deriv derivatives(const State& s,
                    const CmdL4& cmd,
                    const ContactArray& contacts) {
        const Mat3 R = s.orientation.toRotationMatrix();
        const Vec3 v_body = s.velocity;
        const Vec3 omega  = s.angular_velocity;
        const double m    = vp_.mass;
        const double Rwh  = vp_.wheel_radius_nominal;
        const bool stunt_loop = sp_.stunt_physics && sp_.loop_radius > 1.0;
        double k_tire = std::min(80000.0, std::max(1.0, tp_.tire_vertical_stiffness));
        if (stunt_loop) k_tire *= 0.5;
        const double L = vp_.wheelbase;
        const double Tw_f = vp_.track_front;

        const DriverCmd driver_cmd{
            cmd.steer_angle_wheel, cmd.throttle, cmd.brake, cmd.gear, cmd.handbrake};
        SubsystemContext ctx{s, driver_cmd, 0.0};

        const double Fz_cap = (stunt_loop ? 18.0 : 6.0) * m * kGravity
                              / static_cast<double>(NUM_WHEELS);
        constexpr double kLoopPenCap = 0.016;
        double c_vert = 2.0 * std::sqrt(k_tire * m / static_cast<double>(NUM_WHEELS));
        if (stunt_loop) c_vert *= 1.4;
        std::array<double, NUM_WHEELS> Fz {};
        for (int i = 0; i < NUM_WHEELS; ++i) {
            if (!contacts[i].is_valid || contacts[i].penetration <= 0.0) {
                Fz[i] = 0.0;
                continue;
            }
            const Vec3& rb = r_body_[i];
            const Vec3 v_hub = R * (v_body + omega.cross(rb));
            const Vec3 n = contacts[i].normal.normalized();
            const double pen_raw = contacts[i].penetration;
            const double pen = stunt_loop ? std::min(pen_raw, kLoopPenCap) : pen_raw;
            const double vn  = -v_hub.dot(n);
            Fz[i] = std::clamp(k_tire * pen + c_vert * vn, 0.0, Fz_cap);
        }
        ctx.Fz = Fz;
        const double d = steering_->apply(ctx).roadwheel_angle;

        double d_FL = d, d_FR = d;
        if (std::abs(d) > 1e-6 && vp_.ackerman_percent > 1e-9) {
            const double frac = std::clamp(vp_.ackerman_percent / 100.0, 0.0, 1.0);
            const double half = Tw_f * 0.5;
            const double sgn  = (d >= 0.0) ? 1.0 : -1.0;
            const double td_a = std::tan(std::abs(d));
            const double d_inner = sgn * std::atan(td_a / (1.0 - td_a * half / L));
            const double d_outer = sgn * std::atan(td_a / (1.0 + td_a * half / L));
            const double d_FL_ack = (d > 0.0) ? d_inner : d_outer;
            const double d_FR_ack = (d > 0.0) ? d_outer : d_inner;
            d_FL = d + frac * (d_FL_ack - d);
            d_FR = d + frac * (d_FR_ack - d);
        }
        const std::array<double, NUM_WHEELS> d_wheel = {{
            d_FL, d_FR, 0.0, 0.0}};

        const Vec3 body_fwd = R * Vec3::UnitX();
        const bool lugre_on = tp_.lugre.enabled;
        const double speed = (R * v_body).norm();
        double lambda = lugre_on ? 1.0 : std::clamp(speed / kStickBlend, 0.0, 1.0);
        if (!lugre_on) lambda = lambda * lambda * (3.0 - 2.0 * lambda);

        Vec3 F_total_world = Vec3(0.0, 0.0, -m * kGravity);
        Vec3 tau_body = Vec3::Zero();
        std::array<Vec3, NUM_WHEELS> F_body {};
        std::array<double, NUM_WHEELS> kappa {}, alpha {};
        std::array<double, NUM_WHEELS> mz_wheel {{0.0, 0.0, 0.0, 0.0}};
        std::array<double, NUM_WHEELS> fx_kin {{0.0, 0.0, 0.0, 0.0}};

        for (int i = 0; i < NUM_WHEELS; ++i) {
            if (!contacts[i].is_valid || contacts[i].penetration <= 0.0) {
                F_body[i] = Vec3::Zero();
                continue;
            }
            const Vec3& rb = r_body_[i];
            const Vec3 hub_world = s.position + R * rb;
            const Vec3 v_hub_world = R * (v_body + omega.cross(rb));
            Vec3 n = contacts[i].normal.normalized();
            Vec3 t_lat;
            Vec3 t_long = wheel_tangent_frame(n, body_fwd, d_wheel[i], t_lat);
            const double v_long_w = v_hub_world.dot(t_long);
            const double v_lat = v_hub_world.dot(t_lat);
            double v_long_k = v_long_w;
            if (stunt_loop && contacts[i].surface_id == 2) {
                const Vec3 t_track = loop_track_tangent(
                    hub_world, v_hub_world, sp_.loop_center_x, sp_.loop_center_z);
                const double v_track = v_hub_world.dot(t_track);
                const double spd = v_hub_world.norm();
                if (std::abs(v_long_w) < 0.2 * std::max(spd, kSpeedEps)) {
                    const double sign = (t_long.dot(t_track) >= 0.0) ? 1.0 : -1.0;
                    v_long_k = sign * std::max(std::abs(v_track), kSpeedEps);
                }
            }
            const double a_slip = std::atan2(v_lat, v_long_w);
            double denom = std::max(std::abs(v_long_k), kSpeedEps);
            if (stunt_loop && contacts[i].surface_id == 2)
                denom = std::max(denom, 0.05 * v_hub_world.norm());
            // Re + slip velocity from the shared tire-contact module; free_3d keeps
            // its own denom (stunt-loop specialisation) for the slip ratio.
            const auto ck = tire_contact_kinematics(v_long_k, v_lat, s.wheel_spin[i], Fz[i], 0.0, tp_, Rwh);
            const double k_slip = ck.vsx / denom;
            kappa[i] = k_slip;
            alpha[i] = a_slip;

            if (Fz[i] < 1.0) {
                F_body[i] = Vec3::Zero();
                continue;
            }

            // Belt transient (T2): feed relaxed slip on the MF path, relaxed slip
            // velocity on the LuGre path. Geometric values stored for the per-
            // substep belt advance; held frozen within RK4.
            const bool belt_mf = tp_.belt.enabled && !lugre_on;
            ITireModel::Input in;
            in.Fz = Fz[i];
            in.kappa = belt_mf ? belt_kappa_[i] : k_slip;
            in.alpha = belt_mf ? belt_alpha_[i] : a_slip;
            in.mu_long = contacts[i].mu_long;
            in.mu_lat  = contacts[i].mu_lat;
            in.Vx_wheel = v_long_k;
            kappa_geom_last_[i] = k_slip;
            alpha_geom_last_[i] = a_slip;
            vx_belt_last_[i]    = v_long_k;

            const double muFz = std::min(in.mu_long, in.mu_lat) * std::max(0.0, Fz[i]);
            double Fx_w = 0.0, Fy_w = 0.0;
            if (lugre_on) {
                const double v_slip_long_geom = ck.vsx;
                v_slip_long_last_[i] = v_slip_long_geom;
                v_slip_lat_last_[i]  = v_lat;
                const double v_slip_long = tp_.belt.enabled ? belt_vlong_[i] : v_slip_long_geom;
                const double v_slip_lat  = tp_.belt.enabled ? belt_vlat_[i]  : v_lat;
                const auto lugre = lugre_wheel_forces(
                    *tire_, tp_, lugre_z_long_[i], lugre_z_lat_[i],
                    v_slip_long, v_slip_lat, in);
                Fx_w = lugre.Fx;
                Fy_w = lugre.Fy;
                mz_wheel[i] = lugre.Mz;
            } else {
                const auto out = tire_->compute(in);
                const double hold_gate = (1.0 - lambda) *
                    std::clamp(cmd.brake - cmd.throttle, 0.0, 1.0);
                double Fx_hold = -kStickC * (R * v_body).dot(t_long) * hold_gate;
                if (std::abs(Fx_hold) > muFz) Fx_hold = std::copysign(muFz, Fx_hold);
                Fx_w = out.Fx + Fx_hold;
                Fy_w = lambda * out.Fy;
                fx_kin[i] = out.Fx;
                mz_wheel[i] = out.Mz * lambda;
            }
            const double Fmag = std::hypot(Fx_w, Fy_w);
            if (!tp_.model_provides_combined_slip() && Fmag > muFz && Fmag > 1e-9) {
                const double c = muFz / Fmag;
                Fx_w *= c;
                Fy_w *= c;
            }
            Vec3 F_w = Fx_w * t_long + Fy_w * t_lat + Fz[i] * n;
            F_total_world += F_w;
            F_body[i] = R.transpose() * F_w;
            tau_body += rb.cross(F_body[i]);
            tau_body.z() += mz_wheel[i];
        }

        const double vx_fwd = v_body.x();
        const double F_aero = 0.5 * kAirDensity * vp_.aero_drag_coeff *
                              vp_.frontal_area * vx_fwd * std::abs(vx_fwd);
        F_total_world -= R * Vec3(F_aero, 0.0, 0.0);
        if (stunt_loop && !sp_.loop_rail_guide && sp_.loop_radius > 1.0) {
            const double xc = sp_.loop_center_x;
            const double zc = sp_.loop_center_z;
            const double Rloop = sp_.loop_radius;
            const double target = Rloop + Rwh;
            constexpr double kRad = 175000.0;
            constexpr double cRad = 18000.0;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                const Vec3 hub = s.position + R * r_body_[i];
                const double dx = hub.x() - xc;
                const double dz = hub.z() - zc;
                const double hr = std::hypot(dx, dz);
                if (hr < 0.5 * Rloop || hr > target + 0.55) continue;
                const double err = hr - target;
                if (std::abs(err) > 0.5) continue;
                const Vec3 radial(dx / hr, 0.0, dz / hr);
                const Vec3 v_hub = R * (v_body + omega.cross(r_body_[i]));
                const double vr = v_hub.dot(radial);
                double Fr = -kRad * err - cRad * vr;
                if (err > 0.0) {
                    Fr = std::clamp(Fr, -3.5 * Fz_cap, 0.0);
                } else {
                    Fr = std::clamp(Fr, 0.0, 0.4 * Fz_cap);
                }
                const Vec3 Fw = Fr * radial;
                F_total_world += Fw;
                tau_body += r_body_[i].cross(R.transpose() * Fw);
            }
        }

        const double Fz_sum = Fz[0] + Fz[1] + Fz[2] + Fz[3];
        const double F_rr = tp_.rolling_resistance * Fz_sum * std::tanh(vx_fwd / 0.5);
        F_total_world -= R * Vec3(F_rr, 0.0, 0.0);

        const auto dt_out = drivetrain_->apply(ctx);
        const std::array<double, NUM_WHEELS> Td = dt_out.wheel_torque;
        const double effective_brake = driver_cmd.brake
                                       * (1.0 - std::clamp(dt_out.brake_absorbed, 0.0, 1.0));
        const DriverCmd cmd_residual{driver_cmd.handwheel_angle,
                                     driver_cmd.throttle, effective_brake,
                                     driver_cmd.gear, driver_cmd.handbrake};
        const SubsystemContext ctx_brake{s, cmd_residual, ctx.dt, ctx.Fz};
        const std::array<double, NUM_WHEELS> Tb = brake_->wheel_torque(ctx_brake);

        const Vec3 a_body = R.transpose() * (F_total_world / m) - omega.cross(v_body);
        const Vec3 Iw(
            vp_.inertia_diag.x() * omega.x(),
            vp_.inertia_diag.y() * omega.y(),
            vp_.inertia_diag.z() * omega.z());
        const Vec3 alpha_body = Vec3(
            I_body_inv_.x() * (tau_body.x() - omega.y() * Iw.z() + omega.z() * Iw.y()),
            I_body_inv_.y() * (tau_body.y() - omega.z() * Iw.x() + omega.x() * Iw.z()),
            I_body_inv_.z() * (tau_body.z() - omega.x() * Iw.y() + omega.y() * Iw.x()));

        Deriv d_out;
        d_out.dp_world = R * v_body;
        d_out.d_v_body = a_body;
        d_out.d_omega  = alpha_body;
        d_out.ax_body  = a_body.x();
        d_out.ay_body  = a_body.y();

        const bool open_diff = vp_.differential == VehicleParams::Differential::Open;
        // Effective per-wheel reflected engine inertia: gear-dependent from the
        // drivetrain when provided (sentinel <0 -> legacy reflection). The
        // open-diff carrier inertia is the sum of the axle's pair, consistent
        // with the per-wheel divisor (legacy path == axle_reflected_shares).
        auto eff_wheel_I = [&](int i) {
            const double e = drivetrain_->wheel_engine_inertia(i);
            return (e < 0.0) ? wheel_engine_inertia_share(vp_, i) : e;
        };
        // Net wheel torque + per-wheel reflected inertia, gated by ground contact
        // (an airborne wheel gets no drive and no engine coupling — clutch/diff open).
        std::array<double, NUM_WHEELS> T_net{};
        std::array<double, NUM_WHEELS> I_eng_w{};
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const bool grounded = contacts[i].is_valid && contacts[i].penetration > 0.0
                                  && Fz[i] >= 1.0;
            const double T_drive = grounded ? Td[i] : 0.0;
            const double T_react = grounded
                ? fx_kin[i] * effective_rolling_radius(tp_, Rwh, Fz[i])
                : kWheelSpinDrag * s.wheel_spin[i];
            T_net[i]   = T_drive + Tb[i] - T_react;
            I_eng_w[i] = grounded ? eff_wheel_I(i) : 0.0;
        }
        if (open_diff) {
            open_axle_spin_accel(d_out.domega[WHEEL_FL], d_out.domega[WHEEL_FR],
                                 T_net[WHEEL_FL], T_net[WHEEL_FR],
                                 I_wheel_[WHEEL_FL], I_wheel_[WHEEL_FR],
                                 I_eng_w[WHEEL_FL] + I_eng_w[WHEEL_FR]);
            open_axle_spin_accel(d_out.domega[WHEEL_RL], d_out.domega[WHEEL_RR],
                                 T_net[WHEEL_RL], T_net[WHEEL_RR],
                                 I_wheel_[WHEEL_RL], I_wheel_[WHEEL_RR],
                                 I_eng_w[WHEEL_RL] + I_eng_w[WHEEL_RR]);
        } else {
            for (int i = 0; i < NUM_WHEELS; ++i)
                d_out.domega[i] = T_net[i] / (I_wheel_[i] + I_eng_w[i]);
        }

        tire_F_   = F_body;
        tire_Fz_  = Fz;
        slip_ratio_ = kappa;
        slip_angle_ = alpha;
        mz_front_sum_ = mz_wheel[WHEEL_FL] + mz_wheel[WHEEL_FR];
        return d_out;
    }

    static State apply(const State& s0, const Deriv& d, double h) {
        State s = s0;
        s.position += d.dp_world * h;
        s.velocity += d.d_v_body * h;
        s.angular_velocity += d.d_omega * h;
        const Vec3 dw = s.angular_velocity * h;
        const double ang = dw.norm();
        if (ang > 1e-14) {
            const Quat dq(Eigen::AngleAxisd(ang, dw.normalized()));
            s.orientation = (s0.orientation * dq).normalized();
        }
        for (int i = 0; i < NUM_WHEELS; ++i)
            s.wheel_spin[i] = s0.wheel_spin[i] + d.domega[i] * h;
        return s;
    }

    void substep(const CmdL4& cmd, const ContactArray& contacts, double h) {
        const State s0 = state_;
        if (sp_.integrator == SolverParams::Integrator::Euler) {
            const Deriv k = derivatives(s0, cmd, contacts);
            state_ = apply(s0, k, h);
            ax_prev_ = k.ax_body;
            ay_prev_ = k.ay_body;
            advance_belt(h);
            return;
        }
        const Deriv k1 = derivatives(s0, cmd, contacts);
        const Deriv k2 = derivatives(apply(s0, k1, 0.5 * h), cmd, contacts);
        const Deriv k3 = derivatives(apply(s0, k2, 0.5 * h), cmd, contacts);
        const Deriv k4 = derivatives(apply(s0, k3, h), cmd, contacts);
        Deriv k;
        k.dp_world = (k1.dp_world + 2*k2.dp_world + 2*k3.dp_world + k4.dp_world) / 6.0;
        k.d_v_body = (k1.d_v_body + 2*k2.d_v_body + 2*k3.d_v_body + k4.d_v_body) / 6.0;
        k.d_omega  = (k1.d_omega  + 2*k2.d_omega  + 2*k3.d_omega  + k4.d_omega) / 6.0;
        k.ax_body  = (k1.ax_body  + 2*k2.ax_body  + 2*k3.ax_body  + k4.ax_body) / 6.0;
        k.ay_body  = (k1.ay_body  + 2*k2.ay_body  + 2*k3.ay_body  + k4.ay_body) / 6.0;
        for (int i = 0; i < NUM_WHEELS; ++i)
            k.domega[i] = (k1.domega[i] + 2*k2.domega[i] + 2*k3.domega[i] + k4.domega[i]) / 6.0;
        state_ = apply(s0, k, h);
        ax_prev_ = k.ax_body;
        ay_prev_ = k.ay_body;
        advance_belt(h);
    }

    // Belt transient (T2): relax slip (MF) or slip velocity (LuGre) per substep.
    void advance_belt(double h) {
        if (!tp_.belt.enabled) return;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            if (tp_.lugre.enabled) {
                belt_vlong_[i] = belt_relax(belt_vlong_[i], v_slip_long_last_[i],
                                            vx_belt_last_[i], tp_.belt.sigma_long, h);
                belt_vlat_[i]  = belt_relax(belt_vlat_[i], v_slip_lat_last_[i],
                                            vx_belt_last_[i], tp_.belt.sigma_lat, h);
            } else {
                belt_kappa_[i] = belt_relax(belt_kappa_[i], kappa_geom_last_[i],
                                            vx_belt_last_[i], tp_.belt.sigma_long, h);
                belt_alpha_[i] = belt_relax(belt_alpha_[i], alpha_geom_last_[i],
                                            vx_belt_last_[i], tp_.belt.sigma_lat, h);
            }
        }
    }

    VehicleParams vp_;
    TireParams    tp_;
    SolverParams  sp_;
    std::unique_ptr<ITireModel> tire_;
    std::shared_ptr<IDrivetrain> drivetrain_;
    std::shared_ptr<IBrakeSystem> brake_;
    std::shared_ptr<ISteeringSystem> steering_;
    std::array<Vec3, NUM_WHEELS> r_body_ {};
    std::array<double, NUM_WHEELS> I_wheel_ {};
    Vec3 I_body_inv_ {Vec3::Zero()};
    State state_;
    double ax_prev_ {0.0};
    double ay_prev_ {0.0};
    std::array<Vec3, NUM_WHEELS> tire_F_ {};
    std::array<double, NUM_WHEELS> tire_Fz_ {};
    std::array<double, NUM_WHEELS> slip_ratio_ {};
    std::array<double, NUM_WHEELS> slip_angle_ {};
    // Belt transient (T2): relaxed slip (MF) + slip velocity (LuGre) states.
    std::array<double, NUM_WHEELS> belt_kappa_       {};
    std::array<double, NUM_WHEELS> belt_alpha_       {};
    std::array<double, NUM_WHEELS> belt_vlong_       {};
    std::array<double, NUM_WHEELS> belt_vlat_        {};
    std::array<double, NUM_WHEELS> kappa_geom_last_  {};
    std::array<double, NUM_WHEELS> alpha_geom_last_  {};
    std::array<double, NUM_WHEELS> v_slip_long_last_ {};
    std::array<double, NUM_WHEELS> v_slip_lat_last_  {};
    std::array<double, NUM_WHEELS> vx_belt_last_     {};
    std::array<double, NUM_WHEELS> lugre_z_long_ {};
    std::array<double, NUM_WHEELS> lugre_z_lat_ {};
    double mz_front_sum_ {0.0};
};

}  // namespace

std::unique_ptr<IVehicleDynamics> create_stunt_dof() {
    return std::make_unique<Free3DDynamics>();
}

}  // namespace vdsim
