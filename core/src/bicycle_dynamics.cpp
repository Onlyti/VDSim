// L1 Bicycle (single-track) vehicle dynamics.
//
// State vector for integration: [x_w, y_w, yaw, vx, vy, r, omega_f, omega_r]
//   x_w, y_w : world position [m]
//   yaw      : heading [rad], ISO 8855 RH (CCW positive)
//   vx, vy   : body-frame velocity [m/s]
//   r        : yaw rate [rad/s]
//   omega_*  : wheel spin [rad/s] (front avg, rear avg)
//
// Tire forces from Pacejka MF96 with per-axle Fz.
// Static Fz only (no longitudinal weight transfer in PoC bicycle).
// Slip angle in ISO 8855 RH:  alpha_f = atan2(v_wheel_y, v_wheel_x)
//                              alpha_r = atan2(vy - b*r, vx)
// Note: this differs from Rajamani's "delta - atan(...)" which assumes
// SAE Y-right convention.

#include "vdsim/coordinate.hpp"
#include "vdsim/drivetrain_inertia.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/low_speed.hpp"
#include "vdsim/powertrain.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <type_traits>
#include <variant>

namespace vdsim {

namespace {

// Lower L1/L2/L3 commands to a unified L4 (throttle/brake/steer) for the
// downstream dynamics path.  Higher-level inputs (L5..L8) fall back to zero
// — they should be handled by an external converter cascade.
inline CmdL4 lower_to_l4(const ControlInput& u) {
    return std::visit([](const auto& cmd) -> CmdL4 {
        using T = std::decay_t<decltype(cmd)>;
        CmdL4 out;
        if constexpr (std::is_same_v<T, CmdL1>) {
            const double T_drive = std::accumulate(cmd.motor_torque.begin(),
                                                    cmd.motor_torque.end(), 0.0);
            const double T_brake = std::accumulate(cmd.brake_torque.begin(),
                                                    cmd.brake_torque.end(), 0.0);
            // Normalize against typical max sums (300 Nm motor * 1 axle, etc.)
            out.throttle = std::clamp(T_drive / 600.0, -1.0, 1.0);
            out.brake    = std::clamp(T_brake / 4000.0, 0.0, 1.0);
            if (out.throttle < 0.0) { out.brake = std::max(out.brake, -out.throttle); out.throttle = 0.0; }
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL2>) {
            out.throttle = std::clamp(cmd.drive_torque / 600.0, -1.0, 1.0);
            out.brake    = std::clamp(cmd.brake_torque / 4000.0, 0.0, 1.0);
            if (out.throttle < 0.0) { out.brake = std::max(out.brake, -out.throttle); out.throttle = 0.0; }
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL3>) {
            const double scale = cmd.Fx_total / (1500.0 * 5.0);   // m * a_ref
            out.throttle = std::clamp(scale, 0.0, 1.0);
            out.brake    = std::clamp(-scale, 0.0, 1.0);
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL4>) {
            out = cmd;
        }
        return out;
    }, u);
}

constexpr double kAirDensity   = 1.225;   // [kg/m^3]
constexpr double kGravity      = 9.80665; // [m/s^2]
constexpr double kBrakeWidth   = 1.0;     // [rad/s] tanh transition width
// kStickBlend / kStickC / kKinTau: shared low-speed constants in vdsim/low_speed.hpp

inline double smooth_sign(double x, double width) {
    return std::tanh(x / width);
}

class BicycleDynamics final : public IVehicleDynamics {
public:
    explicit BicycleDynamics(std::unique_ptr<ITireModel> tire) : tire_(std::move(tire)) {}

    Level level() const noexcept override { return Level::L1_Bicycle; }

    void initialize(const VehicleParams& vp,
                    const TireParams& tp,
                    const SolverParams& sp) override {
        vp_ = vp;
        tp_ = tp;
        sp_ = sp;
        if (tp.backend != "mf96" && !tp.backend.empty())
            tire_ = create_tire_from_params(tp);
        tire_->initialize(tp);
        spdlog::debug("[L1 Bicycle] init: mass={:.0f} kg, L={:.2f} m, wr={:.3f} m, "
                      "RWD={}, ackerman={:.0f}%",
                      vp.mass, vp.wheelbase, vp.wheel_radius_nominal,
                      vp.drive_type == VehicleParams::Drive::RWD, vp.ackerman_percent);

        // Wheel rotational inertia per axle. Use the explicit wheel_inertia param
        // when set (>0); otherwise the solid-disk approximation 0.5 * m * R^2.
        const double R = vp.wheel_radius_nominal;
        auto I_axle = [&](int w) {
            if (vp.wheel_inertia[w] > 0.0) return vp.wheel_inertia[w];
            const double m_w = vp.unsprung_mass[w] > 0.0 ? vp.unsprung_mass[w] : 25.0;
            return std::max(0.01, 0.5 * m_w * R * R);
        };
        I_wheel_f_ = I_axle(WHEEL_FL);
        I_wheel_r_ = I_axle(WHEEL_RL);
        // Opt-in powertrain (drivetrain v2): a 2D engine map + N-speed gearbox
        // replaces the flat (max_motor_torque x final_drive) torque. The engine
        // is advanced once per substep and its reflected inertia is gear-dependent,
        // so the spin-inertia base here is wheel-only; the gear-dependent share is
        // added per substep. Default off -> legacy flat path, ISO baseline unchanged.
        has_powertrain_ = vp.powertrain.enabled;
        if (has_powertrain_) {
            engine_gb_.configure(vp.powertrain);
            I_spin_f_ = I_wheel_f_;
            I_spin_r_ = I_wheel_r_;
        } else {
            double I_axle_f = 0.0, I_axle_r = 0.0;
            axle_reflected_shares(vp, I_axle_f, I_axle_r);
            I_spin_f_ = I_wheel_f_ + I_axle_f;
            I_spin_r_ = I_wheel_r_ + I_axle_r;
        }
    }

    void reset(const State& s) noexcept override {
        state_   = s;
        ax_prev_ = 0.0;
        if (has_powertrain_) engine_gb_.reset();
        pt_T_front_ = pt_T_rear_ = 0.0;
        pt_I_axle_f_ = pt_I_axle_r_ = 0.0;
        transient_.fill(ITireModel::Transient{});
        ci_.fill(ITireModel::ContactInput{});
        // Clear diagnostics so accessors don't return stale values before step().
        tire_F_.fill(Vec3::Zero());
        tire_Fz_.fill(0.0);
        slip_ratio_.fill(0.0);
        slip_angle_.fill(0.0);
    }

    void step(const ControlInput& u,
              const ContactArray& contacts,
              double dt) noexcept override {
        // L1-L4 direct dispatch via std::visit. Higher levels fall back to zero.
        CmdL4 cmd = lower_to_l4(u);

        // NaN/Inf guard on inputs — log once then sanitize.
        auto sanitize = [](double& v, double lo, double hi) {
            if (!std::isfinite(v)) { v = 0.0; return true; }
            if (v < lo) v = lo; if (v > hi) v = hi; return false;
        };
        bool bad = false;
        bad |= sanitize(cmd.throttle, 0.0, 1.0);
        bad |= sanitize(cmd.brake,    0.0, 1.0);
        bad |= sanitize(cmd.steer_angle_wheel, -1.5, 1.5);
        if (bad) spdlog::warn("[L1] non-finite CmdL4 sanitized");

        if (!(dt > 0.0) || !std::isfinite(dt)) return;

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

    double engine_rpm()   const override { return has_powertrain_ ? engine_gb_.engine_rpm() : 0.0; }
    int    current_gear() const override { return has_powertrain_ ? engine_gb_.current_gear() : 0; }
    bool   set_shift_policy(ShiftPolicy fn) override {
        if (!has_powertrain_) return false;
        engine_gb_.set_shift_policy(std::move(fn));
        return true;
    }

private:
    struct Deriv {
        double dx_world {0.0};
        double dy_world {0.0};
        double dyaw     {0.0};
        double dvx      {0.0};
        double dvy      {0.0};
        double dr       {0.0};
        double domega_f {0.0};
        double domega_r {0.0};
        double ax_body  {0.0};   // pure longitudinal accel Fx/m (weight-transfer lag)
    };

    Deriv derivatives(const State& s,
                      const CmdL4& cmd,
                      const ContactArray& contacts) {
        const double d   = cmd.steer_angle_wheel;
        const double vx  = s.velocity.x();
        const double vy  = s.velocity.y();
        const double r   = s.angular_velocity.z();
        const double of  = s.wheel_spin[WHEEL_FL];
        const double or_ = s.wheel_spin[WHEEL_RL];
        const double yaw = yaw_from_quat(s.orientation);

        const double m   = vp_.mass;
        const double Izz = vp_.inertia_diag.z();
        const double a   = vp_.cg_to_front;
        const double b   = vp_.cg_to_rear;
        const double L   = vp_.wheelbase;
        const double R   = vp_.wheel_radius_nominal;
        const double h_cg = vp_.cg_height;

        // Road slope from contact normals (flat -> no effect): normal load scales
        // by cos(slope); gravity tangential to the road is felt as a body force.
        Vec3 n_road = contacts[WHEEL_FL].normal + contacts[WHEEL_FR].normal
                    + contacts[WHEEL_RL].normal + contacts[WHEEL_RR].normal;
        const double nn = n_road.norm();
        if (nn > 1e-9) n_road /= nn; else n_road = Vec3::UnitZ();
        const double cos_slope = std::max(0.1, n_road.z());
        const double gdotn = -kGravity * n_road.z();
        const double gtx = -gdotn * n_road.x();
        const double gty = -gdotn * n_road.y();
        const double gx_b =  std::cos(yaw) * gtx + std::sin(yaw) * gty;
        const double gy_b = -std::sin(yaw) * gtx + std::cos(yaw) * gty;

        // Aerodynamic downforce per axle (positive Cl -> Fz increase).
        const double q_aero = 0.5 * kAirDensity * vp_.frontal_area * vx * std::abs(vx);
        const double Fz_aero_f = vp_.aero_lift_front * q_aero;
        const double Fz_aero_r = vp_.aero_lift_rear  * q_aero;

        // Quasi-static longitudinal weight transfer (1-step lag on ax to
        // avoid self-reference).  ax > 0 (accel) -> rear loaded.
        const double dFz_long = m * ax_prev_ * h_cg / L;
        double Fz_f = m * kGravity * cos_slope * b / L + Fz_aero_f - dFz_long;
        double Fz_r = m * kGravity * cos_slope * a / L + Fz_aero_r + dFz_long;
        // Clamp to non-negative (extreme braking can lift rear).
        if (Fz_f < 0.0) Fz_f = 0.0;
        if (Fz_r < 0.0) Fz_r = 0.0;

        // Average mu from front/rear contact pairs.
        auto avg = [](double p, double q) { return 0.5 * (p + q); };
        const double mu_long_f = avg(contacts[WHEEL_FL].mu_long, contacts[WHEEL_FR].mu_long);
        const double mu_lat_f  = avg(contacts[WHEEL_FL].mu_lat,  contacts[WHEEL_FR].mu_lat);
        const double mu_long_r = avg(contacts[WHEEL_RL].mu_long, contacts[WHEEL_RR].mu_long);
        const double mu_lat_r  = avg(contacts[WHEEL_RL].mu_lat,  contacts[WHEEL_RR].mu_lat);

        // ---- Velocities ----
        // Wheel positions in body: front (a, 0), rear (-b, 0).
        // Body-frame velocity at wheel positions:
        const double v_fx_body = vx;
        const double v_fy_body = vy + a * r;
        const double v_rx_body = vx;
        const double v_ry_body = vy - b * r;

        // Rotate front-wheel velocity into wheel frame (wheel rotated by d from body):
        const double cd = std::cos(d);
        const double sd = std::sin(d);
        const double v_fx_wheel =  v_fx_body * cd + v_fy_body * sd;
        const double v_fy_wheel = -v_fx_body * sd + v_fy_body * cd;
        // Rear wheel: no steering, wheel frame == body frame.

        const bool lugre_on = tp_.lugre.enabled;

        // Per-tire individual contact: each axle is two identical tires (single-track,
        // no lateral transfer) at half the axle load. Evaluate ONE tire at the per-wheel
        // load (ci.Fz = 0.5*Fz, so the force law AND Re see a consistent load -- the
        // load-sensitive mu is applied at the true per-tire load) and double the wrench
        // for the axle. The tire owns slip / Re / transient (advanced per stage/substep
        // below); only the integrator stabilization stays here.
        ITireModel::ContactInput ci_f;
        ci_f.Fz = 0.5 * Fz_f; ci_f.Vx = v_fx_wheel; ci_f.Vy = v_fy_wheel;
        ci_f.omega = of; ci_f.gamma = 0.0;
        ci_f.mu_long = mu_long_f; ci_f.mu_lat = mu_lat_f; ci_f.R0 = R;
        ITireModel::ContactInput ci_r;
        ci_r.Fz = 0.5 * Fz_r; ci_r.Vx = v_rx_body; ci_r.Vy = v_ry_body;
        ci_r.omega = or_; ci_r.gamma = 0.0;
        ci_r.mu_long = mu_long_r; ci_r.mu_lat = mu_lat_r; ci_r.R0 = R;
        ci_[0] = ci_f; ci_[1] = ci_r;

        const ITireModel::Wrench wf = tire_->evaluate(ci_f, transient_[0]);
        const ITireModel::Wrench wr = tire_->evaluate(ci_r, transient_[1]);
        const double Re_f = wf.Re, Re_r = wr.Re;
        const double kappa_f = wf.kappa, kappa_r = wr.kappa;
        const double alpha_f = wf.alpha, alpha_r = wr.alpha;
        // Axle force = two per-tire wrenches.
        ITireModel::Output F_f, F_r;
        F_f.Fx = 2.0 * wf.Fx; F_f.Fy = 2.0 * wf.Fy; F_f.Mz = 2.0 * wf.Mz;
        F_r.Fx = 2.0 * wr.Fx; F_r.Fy = 2.0 * wr.Fy; F_r.Mz = 2.0 * wr.Mz;

        const double speed = std::hypot(vx, vy);
        double lambda = lugre_on ? 1.0 : std::clamp(speed / kStickBlend, 0.0, 1.0);
        if (!lugre_on) lambda = lambda * lambda * (3.0 - 2.0 * lambda);

        const double muFz_f = std::min(mu_long_f, mu_lat_f) * std::max(0.0, Fz_f);
        const double muFz_r = std::min(mu_long_r, mu_lat_r) * std::max(0.0, Fz_r);

        double Fxh_f = 0.0, Fxh_r = 0.0;
        double Fx_f_wheel = 0.0, Fy_f_wheel = 0.0;
        double Fx_r_axle = 0.0, Fy_r_axle = 0.0;
        double fx_kin_f = 0.0, fx_kin_r = 0.0;
        if (lugre_on) {
            Fx_f_wheel = F_f.Fx; Fy_f_wheel = F_f.Fy;
            Fx_r_axle  = F_r.Fx; Fy_r_axle  = F_r.Fy;
        } else {
            const double hold_gate = (1.0 - lambda) *
                std::clamp(cmd.brake - cmd.throttle, 0.0, 1.0);
            const double c_hold = 2.0 * kStickC;
            Fxh_f = -c_hold * v_fx_body * hold_gate;
            Fxh_r = -c_hold * v_rx_body * hold_gate;
            if (std::abs(Fxh_f) > muFz_f) Fxh_f = std::copysign(muFz_f, Fxh_f);
            if (std::abs(Fxh_r) > muFz_r) Fxh_r = std::copysign(muFz_r, Fxh_r);
            Fx_f_wheel = F_f.Fx + Fxh_f; Fy_f_wheel = lambda * F_f.Fy;
            Fx_r_axle  = F_r.Fx + Fxh_r; Fy_r_axle  = lambda * F_r.Fy;
            fx_kin_f = F_f.Fx; fx_kin_r = F_r.Fx;
        }
        const double Fmag_f = std::hypot(Fx_f_wheel, Fy_f_wheel);
        if (!tp_.model_provides_combined_slip() && Fmag_f > muFz_f && Fmag_f > 1e-9) {
            const double c = muFz_f / Fmag_f; Fx_f_wheel *= c; Fy_f_wheel *= c;
        }
        const double Fmag_r = std::hypot(Fx_r_axle, Fy_r_axle);
        if (!tp_.model_provides_combined_slip() && Fmag_r > muFz_r && Fmag_r > 1e-9) {
            const double c = muFz_r / Fmag_r; Fx_r_axle *= c; Fy_r_axle *= c;
        }
        if (lugre_on) { fx_kin_f = Fx_f_wheel; fx_kin_r = Fx_r_axle; }
        // Rotate wheel-frame front forces back into body frame; rear is body frame.
        const double Fx_body_f = Fx_f_wheel * cd - Fy_f_wheel * sd;
        const double Fy_body_f = Fx_f_wheel * sd + Fy_f_wheel * cd;
        const double Fx_body_r = Fx_r_axle;
        const double Fy_body_r = Fy_r_axle;

        // Aero drag (opposes body-X velocity).
        const double F_aero = 0.5 * kAirDensity * vp_.aero_drag_coeff *
                              vp_.frontal_area * vx * std::abs(vx);
        // Rolling resistance: f_rr * (Fz_f + Fz_r) * tanh(vx / 0.5)
        const double F_rr = tp_.rolling_resistance * (Fz_f + Fz_r) *
                            std::tanh(vx / 0.5);

        // ---- Drive / brake torques ----
        double Td_f = 0.0, Td_r = 0.0;
        if (has_powertrain_) {
            // Engine map + gearbox torque, advanced once per substep (held across
            // RK4 stages). Reverse sign and reflected inertia are baked in upstream.
            Td_f = pt_T_front_;
            Td_r = pt_T_rear_;
        } else {
            const double Tmot = cmd.throttle * vp_.max_motor_torque * vp_.final_drive_ratio;
            switch (vp_.drive_type) {
                case VehicleParams::Drive::FWD:
                    Td_f = Tmot;
                    break;
                case VehicleParams::Drive::RWD:
                    Td_r = Tmot;
                    break;
                case VehicleParams::Drive::AWD:
                    Td_f = 0.5 * Tmot;
                    Td_r = 0.5 * Tmot;
                    break;
            }
            if (cmd.gear < 0) {                   // reverse
                Td_f = -Td_f; Td_r = -Td_r;
            }
        }
        double bias = std::clamp(vp_.brake_bias_front, 0.0, 1.0);
        if (vp_.brake_ebd_enabled) {
            const double total = Fz_f + Fz_r;
            if (total > 1.0) bias = std::clamp(Fz_f / total, 0.05, 0.95);
        }
        const double Tb_f_mag =  bias        * cmd.brake * vp_.max_brake_torque;
        const double Tb_r_mag = (1.0 - bias) * cmd.brake * vp_.max_brake_torque;
        const double Tb_f = -smooth_sign(of,  kBrakeWidth) * Tb_f_mag;
        const double Tb_r = -smooth_sign(or_, kBrakeWidth) * Tb_r_mag;

        // ---- Body equations of motion (ISO 8855) ----
        // m*(vx_dot - vy*r) = Fx_body  =>  vx_dot = Fx/m + vy*r
        // m*(vy_dot + vx*r) = Fy_body  =>  vy_dot = Fy/m - vx*r
        const double Fx_total = Fx_body_f + Fx_body_r - F_aero - F_rr + m * gx_b;
        const double Fy_total = Fy_body_f + Fy_body_r + m * gy_b;
        // Wheel-z and body-z are parallel (camber=0 assumed); Mz adds directly.
        const double mz_scale = lugre_on ? 1.0 : lambda;
        const double Mz_total = a * Fy_body_f - b * Fy_body_r + mz_scale * (F_f.Mz + F_r.Mz);

        Deriv d_out;
        d_out.dx_world = vx * std::cos(yaw) - vy * std::sin(yaw);
        d_out.dy_world = vx * std::sin(yaw) + vy * std::cos(yaw);
        d_out.dyaw     = r;
        d_out.dvx      = Fx_total / m + vy * r;
        d_out.ax_body  = Fx_total / m;   // longitudinal accel (no vy*r) for Fz lag
        // Kinematic-dynamic blend on the lateral states: the lateral force is
        // already faded by lambda (above), so the dynamic part dies on its own as
        // Vx->0; add the (1-lambda) kinematic relaxation onto the slip-free
        // bicycle (r = vx·tan(delta)/L, vy = r·b). Pure kinematic at rest, pure
        // validated dynamics at lambda=1.
        const double dvy_dyn = Fy_total / m - vx * r;
        const double dr_dyn  = Mz_total / Izz;
        const double r_kin   = (L > 1e-6) ? vx * std::tan(d) / L : 0.0;
        const double vy_kin  = r_kin * b;
        if (lugre_on) {
            d_out.dvy = dvy_dyn;
            d_out.dr  = dr_dyn;
        } else {
            d_out.dvy = dvy_dyn + (1.0 - lambda) * (vy_kin - vy) / kKinTau;
            d_out.dr  = dr_dyn  + (1.0 - lambda) * (r_kin  - r ) / kKinTau;
        }
        // Gear-dependent reflected engine inertia (powertrain) is added to the
        // wheel-only base; zero when the flat-torque path is active.
        d_out.domega_f = (Td_f + Tb_f - fx_kin_f * Re_f) / (I_spin_f_ + pt_I_axle_f_);
        d_out.domega_r = (Td_r + Tb_r - fx_kin_r * Re_r) / (I_spin_r_ + pt_I_axle_r_);

        // ---- Diagnostics ----
        // Reported force: fade the raw slip force by lambda and let the smooth
        // brake-hold term carry the low speed (matches the 7-DOF), so the rendered
        // arrows don't chatter from the slip singularity as Vx->0. Display only.
        auto disp = [](double Fx_slip, double Fxh, double Fy_slip, double lam,
                       double muFz, double cd_, double sd_, double& bx, double& by) {
            double fx = lam * Fx_slip + Fxh, fy = lam * Fy_slip;
            const double mg = std::hypot(fx, fy);
            if (mg > muFz && mg > 1e-9) { const double c = muFz / mg; fx *= c; fy *= c; }
            bx = fx * cd_ - fy * sd_; by = fx * sd_ + fy * cd_;
        };
        double dfxf, dfyf, dfxr, dfyr;
        if (lugre_on) {
            dfxf = Fx_f_wheel * cd - Fy_f_wheel * sd;
            dfyf = Fx_f_wheel * sd + Fy_f_wheel * cd;
            dfxr = Fx_r_axle; dfyr = Fy_r_axle;
        } else {
            disp(F_f.Fx, Fxh_f, F_f.Fy, lambda, muFz_f, cd, sd, dfxf, dfyf);
            disp(F_r.Fx, Fxh_r, F_r.Fy, lambda, muFz_r, 1.0, 0.0, dfxr, dfyr);
        }
        // Split axle force evenly across the two co-axial tires.
        tire_F_[WHEEL_FL] = tire_F_[WHEEL_FR] = Vec3(0.5 * dfxf, 0.5 * dfyf, 0.0);
        tire_F_[WHEEL_RL] = tire_F_[WHEEL_RR] = Vec3(0.5 * dfxr, 0.5 * dfyr, 0.0);
        tire_Fz_[WHEEL_FL] = tire_Fz_[WHEEL_FR] = 0.5 * Fz_f;
        tire_Fz_[WHEEL_RL] = tire_Fz_[WHEEL_RR] = 0.5 * Fz_r;
        slip_ratio_[WHEEL_FL] = slip_ratio_[WHEEL_FR] = kappa_f;
        slip_ratio_[WHEEL_RL] = slip_ratio_[WHEEL_RR] = kappa_r;
        slip_angle_[WHEEL_FL] = slip_angle_[WHEEL_FR] = alpha_f;
        slip_angle_[WHEEL_RL] = slip_angle_[WHEEL_RR] = alpha_r;

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
        const double new_of = s0.wheel_spin[WHEEL_FL] + d.domega_f * h;
        const double new_or = s0.wheel_spin[WHEEL_RL] + d.domega_r * h;
        s.wheel_spin = {{new_of, new_of, new_or, new_or}};
        return s;
    }

    // LuGre bristle z — per RK4 stage (per-axle: front=index 0, rear=index 1).
    void advance_bristle_(double dt) {
        if (!tp_.lugre.enabled) return;
        transient_[0] = tire_->advance_bristle(ci_[0], transient_[0], dt);
        transient_[1] = tire_->advance_bristle(ci_[1], transient_[1], dt);
    }
    // Carcass/belt + relaxation-length lag — once per substep.
    void advance_relaxation_(double dt) {
        transient_[0] = tire_->advance_relaxation(ci_[0], transient_[0], dt);
        transient_[1] = tire_->advance_relaxation(ci_[1], transient_[1], dt);
    }

    void substep(const CmdL4& cmd, const ContactArray& contacts, double h) {
        const State s0 = state_;
        // Powertrain (opt-in) is stateful (rpm / gear / shift timer): advance it
        // once per substep with dt=h and hold the axle drive torque + gear-dependent
        // reflected inertia across the RK4 stages (the spin EOM in derivatives()
        // reads the cached pt_* values). Mirrors the once-per-step begin_step in
        // the 7-DOF EngineGearboxDrivetrain.
        if (has_powertrain_) {
            const double of  = s0.wheel_spin[WHEEL_FL];
            const double or_ = s0.wheel_spin[WHEEL_RL];
            double driven_omega = 0.0;
            switch (vp_.drive_type) {
                case VehicleParams::Drive::FWD: driven_omega = of; break;
                case VehicleParams::Drive::RWD: driven_omega = or_; break;
                case VehicleParams::Drive::AWD: driven_omega = 0.5 * (of + or_); break;
            }
            double T = engine_gb_.axle_torque(cmd.throttle, cmd.brake, driven_omega,
                                              s0.velocity.x(), cmd.gear, h);
            if (engine_gb_.current_gear() < 0) T = -T;   // reverse drives backward
            const double Iax = engine_gb_.reflected_inertia();
            switch (vp_.drive_type) {
                case VehicleParams::Drive::FWD:
                    pt_T_front_ = T;       pt_T_rear_ = 0.0;
                    pt_I_axle_f_ = Iax;    pt_I_axle_r_ = 0.0;       break;
                case VehicleParams::Drive::RWD:
                    pt_T_front_ = 0.0;     pt_T_rear_ = T;
                    pt_I_axle_f_ = 0.0;    pt_I_axle_r_ = Iax;       break;
                case VehicleParams::Drive::AWD:
                    pt_T_front_ = 0.5 * T; pt_T_rear_ = 0.5 * T;
                    pt_I_axle_f_ = 0.5 * Iax; pt_I_axle_r_ = 0.5 * Iax; break;
            }
        }
        const bool lugre_on = tp_.lugre.enabled;
        const double hz = 0.25 * h;
        if (sp_.integrator == SolverParams::Integrator::Euler) {
            const Deriv k = derivatives(s0, cmd, contacts);
            state_   = apply(s0, k, h);
            ax_prev_ = k.ax_body;
            if (lugre_on) advance_bristle_(h);
            return;
        }
        const Deriv k1 = derivatives(s0,                       cmd, contacts);
        if (lugre_on) advance_bristle_(hz);
        const Deriv k2 = derivatives(apply(s0, k1, 0.5 * h),  cmd, contacts);
        if (lugre_on) advance_bristle_(hz);
        const Deriv k3 = derivatives(apply(s0, k2, 0.5 * h),  cmd, contacts);
        if (lugre_on) advance_bristle_(hz);
        const Deriv k4 = derivatives(apply(s0, k3, h),        cmd, contacts);
        if (lugre_on) advance_bristle_(hz);

        Deriv k;
        k.dx_world = (k1.dx_world + 2.0 * k2.dx_world + 2.0 * k3.dx_world + k4.dx_world) / 6.0;
        k.dy_world = (k1.dy_world + 2.0 * k2.dy_world + 2.0 * k3.dy_world + k4.dy_world) / 6.0;
        k.dyaw     = (k1.dyaw     + 2.0 * k2.dyaw     + 2.0 * k3.dyaw     + k4.dyaw)     / 6.0;
        k.dvx      = (k1.dvx      + 2.0 * k2.dvx      + 2.0 * k3.dvx      + k4.dvx)      / 6.0;
        k.dvy      = (k1.dvy      + 2.0 * k2.dvy      + 2.0 * k3.dvy      + k4.dvy)      / 6.0;
        k.dr       = (k1.dr       + 2.0 * k2.dr       + 2.0 * k3.dr       + k4.dr)       / 6.0;
        k.domega_f = (k1.domega_f + 2.0 * k2.domega_f + 2.0 * k3.domega_f + k4.domega_f) / 6.0;
        k.domega_r = (k1.domega_r + 2.0 * k2.domega_r + 2.0 * k3.domega_r + k4.domega_r) / 6.0;
        k.ax_body  = (k1.ax_body  + 2.0 * k2.ax_body  + 2.0 * k3.ax_body  + k4.ax_body)  / 6.0;

        state_   = apply(s0, k, h);
        ax_prev_ = k.ax_body;
        // Carcass relaxation + relaxation-length lag, once per substep (tire-owned).
        advance_relaxation_(h);
    }

    VehicleParams vp_;
    TireParams    tp_;
    SolverParams  sp_;
    std::unique_ptr<ITireModel> tire_;
    double I_wheel_f_ {1.0};
    double I_wheel_r_ {1.0};
    double I_spin_f_  {1.0};
    double I_spin_r_  {1.0};

    // Opt-in powertrain (drivetrain v2). Engine state is advanced once per substep;
    // the axle drive torque + gear-dependent reflected inertia are cached and held
    // across the RK4 stages.
    bool          has_powertrain_ {false};
    EngineGearbox engine_gb_;
    double pt_T_front_  {0.0};
    double pt_T_rear_   {0.0};
    double pt_I_axle_f_ {0.0};
    double pt_I_axle_r_ {0.0};

    State state_;
    double ax_prev_ {0.0};
    std::array<Vec3, NUM_WHEELS>   tire_F_    {};
    std::array<double, NUM_WHEELS> tire_Fz_   {};
    std::array<double, NUM_WHEELS> slip_ratio_ {};
    std::array<double, NUM_WHEELS> slip_angle_ {};
    // Per-axle (index 0 = front, 1 = rear) tire transient + the contact kinematics the
    // tire was last evaluated at, feeding the per-stage / per-substep advance calls.
    std::array<ITireModel::Transient, 2> transient_ {};
    std::array<ITireModel::ContactInput, 2> ci_ {};
};

}  // namespace

std::unique_ptr<IVehicleDynamics> create_bicycle() {
    return std::make_unique<BicycleDynamics>(create_pacejka_mf96());
}

std::unique_ptr<IVehicleDynamics> create_bicycle(std::unique_ptr<ITireModel> tire) {
    return std::make_unique<BicycleDynamics>(std::move(tire));
}

}  // namespace vdsim
