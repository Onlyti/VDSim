// L2 7-DOF planar vehicle dynamics.
//
//   State derivative: [x_w, y_w, yaw, vx, vy, r, omega_FL, omega_FR, omega_RL, omega_RR]
//   DOF count        : 3 (planar pose) + 4 wheel spin = 7
//
// Differences from L1 bicycle:
//   - Per-wheel velocity / slip / Pacejka call (4 tires instead of 2 axles).
//   - Per-tire Fz with longitudinal + lateral weight transfer
//     using 1-step lag on ax, ay.  Roll-stiffness ratio splits lateral
//     transfer between front / rear axles.
//
// Limitations (L3 territory):
//   - No suspension dynamics (springs/dampers/unsprung).
//   - No roll / pitch angle in pose; quasi-static transfer only.
//   - Static Fz_z balance only; no aerodynamic lift, no anti-dive geometry.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

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
            out.brake    = std::clamp(T_brake / 4000.0, 0.0, 1.0);
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL2>) {
            out.throttle = std::clamp(cmd.drive_torque / 600.0, 0.0, 1.0);
            out.brake    = std::clamp(cmd.brake_torque / 4000.0, 0.0, 1.0);
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
constexpr double kSpeedEps   = 0.5;
constexpr double kBrakeWidth = 1.0;

inline double smooth_sign(double x, double w) { return std::tanh(x / w); }

class SevenDOFDynamics final : public IVehicleDynamics {
public:
    explicit SevenDOFDynamics(std::unique_ptr<ITireModel> tire)
        : tire_(std::move(tire)) {}

    Level level() const noexcept override { return Level::L2_SevenDOF; }

    void initialize(const VehicleParams& vp,
                    const TireParams& tp,
                    const SolverParams& sp) override {
        vp_ = vp;
        tp_ = tp;
        sp_ = sp;
        tire_->initialize(tp);
        const double m_wheel = vp.unsprung_mass[WHEEL_FL] > 0.0
                              ? vp.unsprung_mass[WHEEL_FL] : 25.0;
        const double R = vp.wheel_radius_nominal;
        I_wheel_ = std::max(0.01, 0.5 * m_wheel * R * R);
        spdlog::debug("[L2 7-DOF] init: mass={:.0f} kg, L={:.2f} m, Tw_f={:.2f} m, "
                      "diff={}, ackerman={:.0f}%, EBD={}",
                      vp.mass, vp.wheelbase, vp.track_front,
                      static_cast<int>(vp.differential), vp.ackerman_percent,
                      vp.brake_ebd_enabled);
    }

    void reset(const State& s) noexcept override {
        state_   = s;
        ax_prev_ = 0.0;
        ay_prev_ = 0.0;
        alpha_dyn_.fill(0.0);
        alpha_geom_last_.fill(0.0);
    }

    void step(const ControlInput& u,
              const ContactArray& contacts,
              double dt) noexcept override {
        const CmdL4 cmd = lower_to_l4(u);
        if (!(dt > 0.0)) return;

        const int N = std::max(1,
                       std::min(sp_.max_substeps,
                                static_cast<int>(std::ceil(dt / sp_.max_substep_dt))));
        const double h = dt / static_cast<double>(N);
        for (int i = 0; i < N; ++i) substep(cmd, contacts, h);
    }

    const State& state() const noexcept override { return state_; }

    std::array<Vec3, NUM_WHEELS>   tire_forces_body() const override { return tire_F_; }
    std::array<double, NUM_WHEELS> tire_Fz()           const override { return tire_Fz_; }
    std::array<double, NUM_WHEELS> wheel_slip_ratio()  const override { return slip_ratio_; }
    std::array<double, NUM_WHEELS> wheel_slip_angle()  const override { return slip_angle_; }

    // Quasi-static roll: K_phi_total * phi = m * ay * h_cg
    double roll_angle_qs() const override {
        const double K = vp_.roll_stiffness_front + vp_.roll_stiffness_rear;
        return (K > 1e-6) ? (vp_.mass * ay_prev_ * vp_.cg_height / K) : 0.0;
    }
    // Quasi-static pitch: anti-dive omitted; effective pitch stiffness
    // approximated by per-axle spring k * (a^2 + b^2) about CG.
    double pitch_angle_qs() const override {
        const double k_avg = 0.25 * (vp_.spring_stiffness[WHEEL_FL] +
                                     vp_.spring_stiffness[WHEEL_FR] +
                                     vp_.spring_stiffness[WHEEL_RL] +
                                     vp_.spring_stiffness[WHEEL_RR]);
        const double K_pitch = k_avg * (vp_.cg_to_front * vp_.cg_to_front +
                                        vp_.cg_to_rear  * vp_.cg_to_rear) * 2.0;
        return (K_pitch > 1e-6)
               ? (vp_.mass * ax_prev_ * vp_.cg_height / K_pitch) : 0.0;
    }
    double ax_body_est() const override { return ax_prev_; }
    double ay_body_est() const override { return ay_prev_; }
    // Steering rack torque: front-wheel Mz summed and divided by steering_ratio.
    double steering_rack_torque() const override {
        return (mz_front_sum_ * vp_.steering_ratio);
    }

    void set_camber_per_wheel(
        const std::array<double, NUM_WHEELS>& gamma) noexcept override {
        camber_ext_ = gamma;
    }

private:
    struct Deriv {
        double dx_world {0.0};
        double dy_world {0.0};
        double dyaw     {0.0};
        double dvx      {0.0};
        double dvy      {0.0};
        double dr       {0.0};
        double ax_body  {0.0};   // Fx_total / m  (kinematic body-x accel)
        double ay_body  {0.0};   // Fy_total / m  (kinematic body-y accel)
        std::array<double, NUM_WHEELS> domega {{0.0, 0.0, 0.0, 0.0}};
    };

    static int axle_of(int i) { return (i < 2) ? 0 : 1; }   // 0 = front, 1 = rear
    static int side_of(int i) { return i % 2; }              // 0 = left (FL/RL), 1 = right (FR/RR)

    Deriv derivatives(const State& s,
                      const CmdL4& cmd,
                      const ContactArray& contacts) {
        const double d   = cmd.steer_angle_wheel;
        const double vx  = s.velocity.x();
        const double vy  = s.velocity.y();
        const double r   = s.angular_velocity.z();
        const double yaw = yaw_from_quat(s.orientation);

        const double m   = vp_.mass;
        const double Izz = vp_.inertia_diag.z();
        const double a   = vp_.cg_to_front;
        const double b   = vp_.cg_to_rear;
        const double L   = vp_.wheelbase;
        const double Tw_f = vp_.track_front;
        const double Tw_r = vp_.track_rear;
        const double R   = vp_.wheel_radius_nominal;
        const double h_cg = vp_.cg_height;

        // ---- Per-tire Fz with 1-step lag weight transfer + aero downforce ----
        const double q_aero  = 0.5 * kAirDensity * vp_.frontal_area * vx * std::abs(vx);
        const double Fz_aero_f_per = 0.5 * vp_.aero_lift_front * q_aero;
        const double Fz_aero_r_per = 0.5 * vp_.aero_lift_rear  * q_aero;
        const double Fz_static_f = m * kGravity * b / (2.0 * L) + Fz_aero_f_per;
        const double Fz_static_r = m * kGravity * a / (2.0 * L) + Fz_aero_r_per;
        const double dFz_long_total = m * ax_prev_ * h_cg / L;      // moves to rear if ax>0
        const double dFz_long_half  = dFz_long_total * 0.5;

        const double k_phi_f = vp_.roll_stiffness_front;
        const double k_phi_r = vp_.roll_stiffness_rear;
        const double k_phi_sum = std::max(1e-6, k_phi_f + k_phi_r);
        const double share_f = k_phi_f / k_phi_sum;
        const double share_r = k_phi_r / k_phi_sum;

        const double dFz_lat_f = (Tw_f > 1e-3)
            ? m * ay_prev_ * h_cg / Tw_f * share_f : 0.0;
        const double dFz_lat_r = (Tw_r > 1e-3)
            ? m * ay_prev_ * h_cg / Tw_r * share_r : 0.0;
        // Sign: ay > 0 (+y, left-turn centripetal) loads right side.
        //   left  tire -= |dFz_lat|, right tire += |dFz_lat|
        // Because in ISO 8855 RH, +y is leftward, and inertia load shifts to -y (right).
        // The numeric sign of dFz_lat already encodes sign(ay) but the lateral
        // transfer goes to the right side regardless of sign convention check.
        // Equivalent expression: Fz_FL -= dFz_lat_f, Fz_FR += dFz_lat_f.

        std::array<double, NUM_WHEELS> Fz;
        Fz[WHEEL_FL] = Fz_static_f - dFz_long_half - dFz_lat_f;
        Fz[WHEEL_FR] = Fz_static_f - dFz_long_half + dFz_lat_f;
        Fz[WHEEL_RL] = Fz_static_r + dFz_long_half - dFz_lat_r;
        Fz[WHEEL_RR] = Fz_static_r + dFz_long_half + dFz_lat_r;
        for (auto& v : Fz) if (v < 0.0) v = 0.0;

        // ---- Per-wheel steer angle with Ackerman correction ----
        // Average steer d -> per-axle inner/outer split.  Ackerman 0% = parallel,
        // 100% = perfect kinematic Ackerman (low-speed turning).
        double d_FL = d, d_FR = d;
        if (std::abs(d) > 1e-6 && vp_.ackerman_percent > 1e-9) {
            const double frac = std::clamp(vp_.ackerman_percent / 100.0, 0.0, 1.0);
            const double td   = std::tan(d);
            const double half = Tw_f * 0.5;
            // R = L / tan(d), Ackerman: delta_in = atan(L / (R - half)),
            //                          delta_out = atan(L / (R + half))
            const double denom_in  = 1.0 - td * half / L;
            const double denom_out = 1.0 + td * half / L;
            const double d_in  = std::atan(td / denom_in);
            const double d_out = std::atan(td / denom_out);
            // d > 0 = left turn => inside = left = FL
            const double d_FL_ack = (d > 0.0) ? d_in  : d_out;
            const double d_FR_ack = (d > 0.0) ? d_out : d_in;
            d_FL = d + frac * (d_FL_ack - d);
            d_FR = d + frac * (d_FR_ack - d);
        }

        // ---- Per-tire velocity, slip, force ----
        std::array<Vec3,   NUM_WHEELS> F_body;
        std::array<double, NUM_WHEELS> kappa, alpha;
        std::array<double, NUM_WHEELS> mz_wheel {{0.0, 0.0, 0.0, 0.0}};

        // Wheel position offsets in body frame (ax, ay relative to CG):
        const std::array<double, NUM_WHEELS> r_x = {{ +a,      +a,      -b,      -b      }};
        const std::array<double, NUM_WHEELS> r_y = {{ +Tw_f*0.5, -Tw_f*0.5, +Tw_r*0.5, -Tw_r*0.5 }};
        const std::array<double, NUM_WHEELS> d_wheel = {{ d_FL, d_FR, 0.0, 0.0 }};

        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double v_x_body = vx - r * r_y[i];
            const double v_y_body = vy + r * r_x[i];

            const double di = d_wheel[i];
            const bool steered = (i == WHEEL_FL || i == WHEEL_FR);
            const double cd_i = std::cos(di), sd_i = std::sin(di);
            double v_x_wheel = v_x_body;
            double v_y_wheel = v_y_body;
            if (steered) {
                v_x_wheel =  v_x_body * cd_i + v_y_body * sd_i;
                v_y_wheel = -v_x_body * sd_i + v_y_body * cd_i;
            }
            const double a_slip = std::atan2(v_y_wheel, v_x_wheel);
            const double denom  = std::max(std::abs(v_x_wheel), kSpeedEps);
            const double k_slip = (R * s.wheel_spin[i] - v_x_wheel) / denom;

            const double mu_long_i = contacts[i].mu_long;
            const double mu_lat_i  = contacts[i].mu_lat;

            ITireModel::Input in;
            in.Fz = Fz[i];
            in.kappa = k_slip;
            // Use the transient α_dyn_ if relaxation length is enabled; the
            // host substep() advances α_dyn_ between substeps.  Within RK4 we
            // hold it frozen so all 4 stages see the same Pacejka linearization.
            in.alpha = (tp_.relaxation_length_lat > 1e-6) ? alpha_dyn_[i] : a_slip;
            in.mu_long = mu_long_i; in.mu_lat = mu_lat_i; in.Vx_wheel = v_x_wheel;
            // Camber input: set by Ld3 (roll-driven) or by external caller via
            // set_camber_per_wheel().  Stand-alone Ld2 leaves it at zero.
            in.gamma = camber_ext_[i];
            const auto out = tire_->compute(in);
            alpha_geom_last_[i] = a_slip;
            v_x_wheel_last_[i]  = v_x_wheel;

            double Fx_b = out.Fx, Fy_b = out.Fy;
            if (steered) {
                Fx_b = out.Fx * cd_i - out.Fy * sd_i;
                Fy_b = out.Fx * sd_i + out.Fy * cd_i;
            }
            F_body[i] = Vec3(Fx_b, Fy_b, 0.0);
            kappa[i]  = k_slip;
            alpha[i]  = a_slip;
            mz_wheel[i] = out.Mz;
        }

        // ---- Drive / brake torques ----
        std::array<double, NUM_WHEELS> Td {{0.0, 0.0, 0.0, 0.0}};
        const double Tmot = cmd.throttle * vp_.max_motor_torque;
        // Distribute per-axle then split L/R via differential model.
        double T_front_axle = 0.0, T_rear_axle = 0.0;
        switch (vp_.drive_type) {
            case VehicleParams::Drive::FWD: T_front_axle = Tmot; break;
            case VehicleParams::Drive::RWD: T_rear_axle  = Tmot; break;
            case VehicleParams::Drive::AWD:
                T_front_axle = 0.5 * Tmot; T_rear_axle = 0.5 * Tmot; break;
        }
        auto split_axle = [&](double T_axle, double omega_L, double omega_R,
                              double& T_L, double& T_R) {
            switch (vp_.differential) {
                case VehicleParams::Differential::Open:
                    T_L = T_R = 0.5 * T_axle;
                    break;
                case VehicleParams::Differential::Locked: {
                    const double dw = omega_L - omega_R;
                    // Bias to slower wheel: dw>0 -> right slower -> bias R (positive bias)
                    const double bias = 0.45 * std::tanh(2.0 * dw);
                    T_L = (0.5 - bias) * T_axle;
                    T_R = (0.5 + bias) * T_axle;
                    break;
                }
                case VehicleParams::Differential::LSD: {
                    const double dw = omega_L - omega_R;
                    const double mag = std::clamp(
                        vp_.lsd_preload + vp_.lsd_ramp * std::abs(dw),
                        0.0, 0.45);
                    const double bias = mag * std::tanh(2.0 * dw);
                    T_L = (0.5 - bias) * T_axle;
                    T_R = (0.5 + bias) * T_axle;
                    break;
                }
            }
        };
        split_axle(T_front_axle,
                   state_.wheel_spin[WHEEL_FL], state_.wheel_spin[WHEEL_FR],
                   Td[WHEEL_FL], Td[WHEEL_FR]);
        split_axle(T_rear_axle,
                   state_.wheel_spin[WHEEL_RL], state_.wheel_spin[WHEEL_RR],
                   Td[WHEEL_RL], Td[WHEEL_RR]);
        if (cmd.gear < 0) for (auto& x : Td) x = -x;
        double bias = std::clamp(vp_.brake_bias_front, 0.0, 1.0);
        if (vp_.brake_ebd_enabled) {
            const double Fz_f = Fz[WHEEL_FL] + Fz[WHEEL_FR];
            const double Fz_r = Fz[WHEEL_RL] + Fz[WHEEL_RR];
            const double total = Fz_f + Fz_r;
            if (total > 1.0) bias = std::clamp(Fz_f / total, 0.05, 0.95);
        }
        const double Tbrk_front_axle =       bias  * cmd.brake * vp_.max_brake_torque;
        const double Tbrk_rear_axle  = (1.0-bias) * cmd.brake * vp_.max_brake_torque;
        std::array<double, NUM_WHEELS> Tb;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double axle_T = (i == WHEEL_FL || i == WHEEL_FR)
                                  ? Tbrk_front_axle : Tbrk_rear_axle;
            Tb[i] = -smooth_sign(s.wheel_spin[i], kBrakeWidth) * (0.5 * axle_T);
        }

        // ---- Body equations ----
        double Fx_total = 0.0, Fy_total = 0.0, Mz_total = 0.0;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            Fx_total += F_body[i].x();
            Fy_total += F_body[i].y();
            Mz_total += r_x[i] * F_body[i].y() - r_y[i] * F_body[i].x();
        }
        // Per-tire Mz_wheel adds directly to body z (parallel axes, camber=0).
        for (int i = 0; i < NUM_WHEELS; ++i) Mz_total += mz_wheel[i];
        mz_front_sum_ = mz_wheel[WHEEL_FL] + mz_wheel[WHEEL_FR];
        const double F_aero = 0.5 * kAirDensity * vp_.aero_drag_coeff *
                              vp_.frontal_area * vx * std::abs(vx);
        const double Fz_total_now = Fz[0] + Fz[1] + Fz[2] + Fz[3];
        const double F_rr = tp_.rolling_resistance * Fz_total_now *
                            std::tanh(vx / 0.5);
        Fx_total -= F_aero;
        Fx_total -= F_rr;

        Deriv d_out;
        d_out.dx_world = vx * std::cos(yaw) - vy * std::sin(yaw);
        d_out.dy_world = vx * std::sin(yaw) + vy * std::cos(yaw);
        d_out.dyaw     = r;
        d_out.dvx      = Fx_total / m + vy * r;
        d_out.dvy      = Fy_total / m - vx * r;
        d_out.dr       = Mz_total / Izz;
        d_out.ax_body  = Fx_total / m;
        d_out.ay_body  = Fy_total / m;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            // Body-frame Fx already applied to body translation. For wheel
            // spin, use wheel-frame Fx (i.e., the raw tire output along wheel-x).
            // For un-steered wheels Fx_body == Fx_wheel; for steered we recover
            // by inverse rotation -> but we have Fx_body and original F.Fx;
            // simpler: re-rotate body force back into wheel x.
            double Fx_wheel = F_body[i].x();
            if (i == WHEEL_FL || i == WHEEL_FR) {
                const double cd_i = std::cos(d_wheel[i]);
                const double sd_i = std::sin(d_wheel[i]);
                Fx_wheel =  F_body[i].x() * cd_i + F_body[i].y() * sd_i;
            }
            d_out.domega[i] = (Td[i] + Tb[i] - Fx_wheel * R) / I_wheel_;
        }

        // ---- Diagnostics ----
        tire_F_   = F_body;
        tire_Fz_  = Fz;
        slip_ratio_ = kappa;
        slip_angle_ = alpha;

        return d_out;
    }

    State apply(const State& s0, const Deriv& d, double h) {
        State s = s0;
        s.position.x() = s0.position.x() + d.dx_world * h;
        s.position.y() = s0.position.y() + d.dy_world * h;
        const double new_yaw = yaw_from_quat(s0.orientation) + d.dyaw * h;
        s.orientation = quat_from_euler({0.0, 0.0, new_yaw});
        s.velocity.x() = s0.velocity.x() + d.dvx * h;
        s.velocity.y() = s0.velocity.y() + d.dvy * h;
        s.angular_velocity.z() = s0.angular_velocity.z() + d.dr * h;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            s.wheel_spin[i] = s0.wheel_spin[i] + d.domega[i] * h;
        }
        return s;
    }

    void substep(const CmdL4& cmd, const ContactArray& contacts, double h) {
        const State s0 = state_;
        if (sp_.integrator == SolverParams::Integrator::Euler) {
            const Deriv k = derivatives(s0, cmd, contacts);
            state_   = apply(s0, k, h);
            ax_prev_ = k.ax_body;
            ay_prev_ = k.ay_body;
            return;
        }
        const Deriv k1 = derivatives(s0,                       cmd, contacts);
        const Deriv k2 = derivatives(apply(s0, k1, 0.5 * h),  cmd, contacts);
        const Deriv k3 = derivatives(apply(s0, k2, 0.5 * h),  cmd, contacts);
        const Deriv k4 = derivatives(apply(s0, k3, h),        cmd, contacts);

        Deriv k;
        k.dx_world = (k1.dx_world + 2*k2.dx_world + 2*k3.dx_world + k4.dx_world) / 6.0;
        k.dy_world = (k1.dy_world + 2*k2.dy_world + 2*k3.dy_world + k4.dy_world) / 6.0;
        k.dyaw     = (k1.dyaw     + 2*k2.dyaw     + 2*k3.dyaw     + k4.dyaw)     / 6.0;
        k.dvx      = (k1.dvx      + 2*k2.dvx      + 2*k3.dvx      + k4.dvx)      / 6.0;
        k.dvy      = (k1.dvy      + 2*k2.dvy      + 2*k3.dvy      + k4.dvy)      / 6.0;
        k.dr       = (k1.dr       + 2*k2.dr       + 2*k3.dr       + k4.dr)       / 6.0;
        k.ax_body  = (k1.ax_body  + 2*k2.ax_body  + 2*k3.ax_body  + k4.ax_body)  / 6.0;
        k.ay_body  = (k1.ay_body  + 2*k2.ay_body  + 2*k3.ay_body  + k4.ay_body)  / 6.0;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            k.domega[i] = (k1.domega[i] + 2*k2.domega[i] + 2*k3.domega[i] + k4.domega[i]) / 6.0;
        }
        state_   = apply(s0, k, h);
        ax_prev_ = k.ax_body;
        ay_prev_ = k.ay_body;
        // Advance the per-wheel transient slip angle exponentially:
        //   α_dyn(t+h) = α_geom + (α_dyn(t) − α_geom) · exp(−|v|·h/σ)
        if (tp_.relaxation_length_lat > 1e-6) {
            const double sigma = tp_.relaxation_length_lat;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                const double v_safe = std::max(std::abs(v_x_wheel_last_[i]),
                                               kSpeedEps);
                const double decay = std::exp(-v_safe * h / sigma);
                alpha_dyn_[i] = alpha_geom_last_[i]
                              + (alpha_dyn_[i] - alpha_geom_last_[i]) * decay;
            }
        }
    }

    VehicleParams vp_;
    TireParams    tp_;
    SolverParams  sp_;
    std::unique_ptr<ITireModel> tire_;
    double I_wheel_ {1.0};

    State state_;
    double ax_prev_ {0.0};
    double ay_prev_ {0.0};
    double mz_front_sum_ {0.0};
    std::array<Vec3,   NUM_WHEELS> tire_F_     {};
    std::array<double, NUM_WHEELS> tire_Fz_    {};
    std::array<double, NUM_WHEELS> slip_ratio_ {};
    std::array<double, NUM_WHEELS> slip_angle_ {};
    // Transient slip-angle state (relaxation length).  Updated between substeps.
    std::array<double, NUM_WHEELS> alpha_dyn_       {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, NUM_WHEELS> alpha_geom_last_ {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, NUM_WHEELS> v_x_wheel_last_  {{0.0, 0.0, 0.0, 0.0}};
    // External per-wheel camber input (set by Ld3 or caller before step).
    std::array<double, NUM_WHEELS> camber_ext_      {{0.0, 0.0, 0.0, 0.0}};
};

}  // namespace

std::unique_ptr<IVehicleDynamics> create_seven_dof() {
    return std::make_unique<SevenDOFDynamics>(create_pacejka_mf96());
}

}  // namespace vdsim
