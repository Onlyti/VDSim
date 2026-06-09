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
#include "vdsim/default_subsystems.hpp"
#include "vdsim/drivetrain_inertia.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/low_speed.hpp"
#include "vdsim/lugre_tire.hpp"
#include "vdsim/subsystems.hpp"

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
            // Negative aggregate drive (regen/engine braking) maps to brake.
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
constexpr double kSpeedEps    = 0.15;
// kStickBlend / kStickC / kKinTau: shared low-speed constants in vdsim/low_speed.hpp

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
        const double R = vp.wheel_radius_nominal;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            // Explicit wheel_inertia overrides the solid-disk approximation.
            if (vp.wheel_inertia[i] > 0.0) {
                I_wheel_[i] = vp.wheel_inertia[i];
            } else {
                const double m_w = vp.unsprung_mass[i] > 0.0 ? vp.unsprung_mass[i] : 25.0;
                I_wheel_[i] = std::max(0.01, 0.5 * m_w * R * R);
            }
        }
        drivetrain_ = make_default_drivetrain(vp_, vp_.drive_deadtime_s);
        brake_      = make_default_brake(vp_, vp_.brake_deadtime_s);
        steering_   = make_default_steering(vp_, vp_.steer_deadtime_s);
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
        lugre_z_long_.fill(0.0);
        lugre_z_lat_.fill(0.0);
        drivetrain_ = make_default_drivetrain(vp_, vp_.drive_deadtime_s);
        brake_      = make_default_brake(vp_, vp_.brake_deadtime_s);
        steering_   = make_default_steering(vp_, vp_.steer_deadtime_s);
        // Clear diagnostics so accessors don't return stale values before step().
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
            cmd.steer_angle_wheel,
            cmd.throttle,
            cmd.brake,
            cmd.gear,
            cmd.handbrake,
        };
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

    std::array<Vec3, NUM_WHEELS>   tire_forces_body() const override { return tire_F_; }
    std::array<double, NUM_WHEELS> tire_Fz()           const override { return tire_Fz_; }
    std::array<double, NUM_WHEELS> wheel_slip_ratio()  const override { return slip_ratio_; }
    std::array<double, NUM_WHEELS> wheel_slip_angle()  const override { return slip_angle_; }

    // Quasi-static roll/pitch incl. the CG-migration (jacking) feedback —
    // computed in step() (needs cos_slope) and stored. See the weight-transfer
    // block for the K_phi*phi = M_roll/(1-eps) derivation.
    double roll_angle_qs()  const override { return roll_qs_;  }
    double pitch_angle_qs() const override { return pitch_qs_; }
    double ax_body_est() const override { return ax_prev_; }
    double ay_body_est() const override { return ay_prev_; }
    // Hand-wheel feedback torque: front-axle aligning moment reduced through the
    // steering ratio (large ratio -> small torque at the hand wheel).
    double steering_rack_torque() const override {
        return mz_front_sum_ / std::max(1e-6, vp_.steering_ratio);
    }

    void set_camber_per_wheel(
        const std::array<double, NUM_WHEELS>& gamma) noexcept override {
        camber_ext_ = gamma;
    }

    void set_toe_per_wheel(
        const std::array<double, NUM_WHEELS>& toe) noexcept override {
        toe_ext_ = toe;
    }

    void set_external_fz(
        const std::array<double, NUM_WHEELS>& fz) noexcept override {
        ext_fz_ = fz; use_ext_fz_ = true;
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

        // ---- Road slope from contact normals (flat -> no effect) ----
        // Average unit normal (world). Normal load scales by cos(slope)=n_z; the
        // gravity component tangential to the road is felt as a body force.
        Vec3 n_road = contacts[0].normal + contacts[1].normal
                    + contacts[2].normal + contacts[3].normal;
        const double nn = n_road.norm();
        if (nn > 1e-9) n_road /= nn; else n_road = Vec3::UnitZ();
        const double cos_slope = std::max(0.1, n_road.z());
        const double gdotn = -kGravity * n_road.z();          // (0,0,-g) . n
        const double gtx = -gdotn * n_road.x();               // tangential gravity (world)
        const double gty = -gdotn * n_road.y();
        const double gx_b =  std::cos(yaw) * gtx + std::sin(yaw) * gty;   // -> body
        const double gy_b = -std::sin(yaw) * gtx + std::cos(yaw) * gty;

        // Specific force the chassis feels in the road-tangent plane = inertial
        // accel minus the tangential gravity the tires already react. On a banked
        // straight ay_kin->0 but ay_felt = -gy_b != 0, so the bank/slope produces
        // quasi-static load transfer (and the reported roll/pitch). Flat: ==0.
        const double ax_felt = ax_prev_ - gx_b;
        const double ay_felt = ay_prev_ - gy_b;
        ax_felt_ = ax_felt; ay_felt_ = ay_felt;

        // ---- Per-tire Fz with 1-step lag weight transfer + aero downforce ----
        const double q_aero  = 0.5 * kAirDensity * vp_.frontal_area * vx * std::abs(vx);
        const double Fz_aero_f_per = 0.5 * vp_.aero_lift_front * q_aero;
        const double Fz_aero_r_per = 0.5 * vp_.aero_lift_rear  * q_aero;
        const double Fz_static_f = m * kGravity * cos_slope * b / (2.0 * L) + Fz_aero_f_per;
        const double Fz_static_r = m * kGravity * cos_slope * a / (2.0 * L) + Fz_aero_r_per;
        const double g_perp = kGravity * cos_slope;               // gravity normal to road

        // Longitudinal transfer + pitch, with CG-migration (jacking): a pitched
        // body shifts the CG by h_cg*sin(theta), adding a gravity pitch moment ->
        // effective pitch stiffness drops by m*g_perp*h_cg (amplifies by 1/(1-eps)).
        const double k_avg = 0.25 * (vp_.spring_stiffness[WHEEL_FL] + vp_.spring_stiffness[WHEEL_FR]
                                   + vp_.spring_stiffness[WHEEL_RL] + vp_.spring_stiffness[WHEEL_RR]);
        const double K_pitch = std::max(1.0, k_avg * (a * a + b * b) * 2.0);
        const double eps_pitch = std::clamp(m * g_perp * h_cg / K_pitch, 0.0, 0.8);
        const double pitch_gain = 1.0 / (1.0 - eps_pitch);
        const double dFz_long_total = m * ax_felt * h_cg / L * pitch_gain;  // incl. grade + jacking
        const double dFz_long_half  = dFz_long_total * 0.5;
        pitch_qs_ = -m * ax_felt * h_cg / K_pitch * pitch_gain;   // ISO: nose-down +

        // ---- Lateral load transfer = geometric (jacking through the roll center)
        //      + elastic (roll about the roll axis, split by axle roll stiffness)
        //      + unsprung. Roll stiffness is derived from springs + ARB.
        const double Kf   = axle_roll_stiffness(vp_, 0);
        const double Kr   = axle_roll_stiffness(vp_, 1);
        const double Ktot = std::max(1e-6, Kf + Kr);
        const double hrc_f = vp_.roll_center_height_front;
        const double hrc_r = vp_.roll_center_height_rear;
        const double h_ra  = hrc_f * (b / L) + hrc_r * (a / L);   // roll axis under CG
        const double m_s   = vp_.mass_sprung;
        const double Fys   = m_s * ay_felt;                      // sprung lateral force (incl. bank)
        // CG-migration: a rolled body shifts the CG by (h_cg-h_ra)*sin(phi), adding
        // a gravity roll moment -> effective roll stiffness drops (amplify 1/(1-eps)).
        const double arm = h_cg - h_ra;
        const double eps_roll = std::clamp(m_s * g_perp * arm / Ktot, 0.0, 0.8);
        const double Mroll = Fys * arm / (1.0 - eps_roll);        // roll moment incl. jacking
        roll_qs_ = Mroll / Ktot;
        const double m_uf  = vp_.unsprung_mass[WHEEL_FL] + vp_.unsprung_mass[WHEEL_FR];
        const double m_ur  = vp_.unsprung_mass[WHEEL_RL] + vp_.unsprung_mass[WHEEL_RR];
        const double dFz_lat_f = (Tw_f > 1e-3)
            ? (Fys * (b / L) * hrc_f + Mroll * (Kf / Ktot) + m_uf * ay_felt * R) / Tw_f
            : 0.0;
        const double dFz_lat_r = (Tw_r > 1e-3)
            ? (Fys * (a / L) * hrc_r + Mroll * (Kr / Ktot) + m_ur * ay_felt * R) / Tw_r
            : 0.0;
        // Sign: ay > 0 (+y, left) shifts load to the right (-y); applied below as
        //   Fz_FL -= dFz_lat_f, Fz_FR += dFz_lat_f (rear analogously).

        std::array<double, NUM_WHEELS> Fz;
        Fz[WHEEL_FL] = Fz_static_f - dFz_long_half - dFz_lat_f;
        Fz[WHEEL_FR] = Fz_static_f - dFz_long_half + dFz_lat_f;
        Fz[WHEEL_RL] = Fz_static_r + dFz_long_half - dFz_lat_r;
        Fz[WHEEL_RR] = Fz_static_r + dFz_long_half + dFz_lat_r;
        for (auto& v : Fz) if (v < 0.0) v = 0.0;
        for (int i = 0; i < NUM_WHEELS; ++i)
            if (!contacts[i].is_valid) Fz[i] = 0.0;
        // L3 may supply a dynamic (ride/road-coupled) tire load for grip in place
        // of this quasi-static Fz. One-shot: consumed here, re-set each L3 step.
        if (use_ext_fz_) { Fz = ext_fz_; use_ext_fz_ = false; }

        const DriverCmd driver_cmd{
            cmd.steer_angle_wheel,
            cmd.throttle,
            cmd.brake,
            cmd.gear,
            cmd.handbrake,
        };
        SubsystemContext ctx{s, driver_cmd, 0.0};
        ctx.Fz = Fz;
        const double d = steering_->apply(ctx).roadwheel_angle;

        // ---- Per-wheel steer angle with Ackerman correction ----
        // Average steer d -> per-axle inner/outer split.  Ackerman 0% = parallel,
        // 100% = perfect kinematic Ackerman (low-speed turning).
        double d_FL = d, d_FR = d;
        if (std::abs(d) > 1e-6 && vp_.ackerman_percent > 1e-9) {
            const double frac = std::clamp(vp_.ackerman_percent / 100.0, 0.0, 1.0);
            const double half = Tw_f * 0.5;
            const double sgn  = (d >= 0.0) ? 1.0 : -1.0;
            const double td_a = std::tan(std::abs(d));
            // R = L/tan(d): inner = atan(|tan d|/(1-|tan d|·half/L)), outer = +half term.
            const double d_inner = sgn * std::atan(td_a / (1.0 - td_a * half / L));
            const double d_outer = sgn * std::atan(td_a / (1.0 + td_a * half / L));
            const double d_FL_ack = (d > 0.0) ? d_inner : d_outer;
            const double d_FR_ack = (d > 0.0) ? d_outer : d_inner;
            d_FL = d + frac * (d_FL_ack - d);
            d_FR = d + frac * (d_FR_ack - d);
        }

        // ---- Per-tire velocity, slip, force ----
        std::array<Vec3,   NUM_WHEELS> F_body;
        std::array<Vec3,   NUM_WHEELS> tire_F_disp;   // reported force (lateral faded by lambda)
        std::array<double, NUM_WHEELS> kappa, alpha;
        std::array<double, NUM_WHEELS> mz_wheel {{0.0, 0.0, 0.0, 0.0}};

        // Wheel position offsets in body frame (ax, ay relative to CG):
        const std::array<double, NUM_WHEELS> r_x = {{ +a,      +a,      -b,      -b      }};
        const std::array<double, NUM_WHEELS> r_y = {{ +Tw_f*0.5, -Tw_f*0.5, +Tw_r*0.5, -Tw_r*0.5 }};
        // Per-wheel total wheel-angle = Ackerman-corrected steer (front) +
        // bump-steer / kinematic toe from Ld4 (any wheel).
        const std::array<double, NUM_WHEELS> d_wheel = {{
            d_FL + toe_ext_[WHEEL_FL],
            d_FR + toe_ext_[WHEEL_FR],
            0.0  + toe_ext_[WHEEL_RL],
            0.0  + toe_ext_[WHEEL_RR],
        }};

        const bool lugre_on = tp_.lugre.enabled;
        // Low-speed blend factor: 0 at rest (stick + kinematic govern) -> 1 above
        // kStickBlend (validated dynamic model). Skipped when LuGre is active.
        const double speed = std::hypot(vx, vy);
        double lambda = lugre_on ? 1.0
            : std::clamp(speed / kStickBlend, 0.0, 1.0);
        if (!lugre_on) lambda = lambda * lambda * (3.0 - 2.0 * lambda);
        std::array<double, NUM_WHEELS> fx_kin {{0.0, 0.0, 0.0, 0.0}};

        for (int i = 0; i < NUM_WHEELS; ++i) {
            if (!contacts[i].is_valid) {
                F_body[i]      = Vec3::Zero();
                tire_F_disp[i] = Vec3::Zero();
                kappa[i]       = 0.0;
                alpha[i]       = 0.0;
                mz_wheel[i]    = 0.0;
                fx_kin[i]      = 0.0;
                continue;
            }
            const double v_x_body = vx - r * r_y[i];
            const double v_y_body = vy + r * r_x[i];

            const double di = d_wheel[i];
            // A wheel is "steered" if its angle is non-trivial (front or any
            // wheel with non-zero toe input).  We just always do the rotation
            // — it's a cheap cos/sin per wheel.
            const double cd_i = std::cos(di), sd_i = std::sin(di);
            double v_x_wheel = v_x_body * cd_i + v_y_body * sd_i;
            double v_y_wheel = -v_x_body * sd_i + v_y_body * cd_i;
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
            in.alpha = (lugre_on || tp_.relaxation_length_lat <= 1e-6)
                ? a_slip : alpha_dyn_[i];
            in.mu_long = mu_long_i; in.mu_lat = mu_lat_i; in.Vx_wheel = v_x_wheel;
            // Camber input: set by Ld3 (roll-driven) or by external caller via
            // set_camber_per_wheel().  Stand-alone Ld2 leaves it at zero.
            in.gamma = camber_ext_[i];
            alpha_geom_last_[i] = a_slip;
            v_x_wheel_last_[i]  = v_x_wheel;
            v_y_wheel_last_[i]  = v_y_wheel;
            wheel_spin_last_[i] = s.wheel_spin[i];

            const double muFz = std::min(mu_long_i, mu_lat_i) * std::max(0.0, Fz[i]);
            const double v_slip_long = R * s.wheel_spin[i] - v_x_wheel;
            const double v_slip_lat  = v_y_wheel;

            double Fx_w = 0.0, Fy_w = 0.0;
            double Fxd = 0.0, Fyd = 0.0;
            if (lugre_on) {
                const auto lugre = lugre_wheel_forces(
                    *tire_, tp_, lugre_z_long_[i], lugre_z_lat_[i],
                    v_slip_long, v_slip_lat, in);
                Fx_w = lugre.Fx;
                Fy_w = lugre.Fy;
                mz_wheel[i] = lugre.Mz;
                Fxd = Fx_w;
                Fyd = Fy_w;
            } else {
                const auto out = tire_->compute(in);
                const double hold_gate = (1.0 - lambda) *
                    std::clamp(cmd.brake - cmd.throttle, 0.0, 1.0);
                double Fx_hold = -kStickC * v_x_body * hold_gate;
                if (std::abs(Fx_hold) > muFz) Fx_hold = std::copysign(muFz, Fx_hold);
                Fx_w = out.Fx + Fx_hold;
                Fy_w = lambda * out.Fy;
                fx_kin[i] = out.Fx;
                mz_wheel[i] = out.Mz * lambda;
                Fxd = lambda * out.Fx + Fx_hold;
                Fyd = Fy_w;
            }
            const double Fmag = std::hypot(Fx_w, Fy_w);
            if (!(lugre_on && tp_.combined_slip_enabled) && Fmag > muFz && Fmag > 1e-9) {
                const double c = muFz / Fmag;
                Fx_w *= c;
                Fy_w *= c;
            }
            if (lugre_on) fx_kin[i] = Fx_w;
            const double Fx_b = Fx_w * cd_i - Fy_w * sd_i;
            const double Fy_b = Fx_w * sd_i + Fy_w * cd_i;
            F_body[i] = Vec3(Fx_b, Fy_b, 0.0);
            const double Fdm = std::hypot(Fxd, Fyd);
            if (!(lugre_on && tp_.combined_slip_enabled) && Fdm > muFz && Fdm > 1e-9) {
                const double c = muFz / Fdm;
                Fxd *= c;
                Fyd *= c;
            }
            tire_F_disp[i] = Vec3(Fxd * cd_i - Fyd * sd_i, Fxd * sd_i + Fyd * cd_i, 0.0);
            kappa[i]  = k_slip;
            alpha[i]  = a_slip;
        }

        const auto dt_out = drivetrain_->apply(ctx);
        const std::array<double, NUM_WHEELS> Td = dt_out.wheel_torque;
        const std::array<double, NUM_WHEELS> Tb = brake_->wheel_torque(ctx);

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
        Fx_total += m * gx_b;   // gravity tangential to the road (0 on flat)
        Fy_total += m * gy_b;

        Deriv d_out;
        d_out.dx_world = vx * std::cos(yaw) - vy * std::sin(yaw);
        d_out.dy_world = vx * std::sin(yaw) + vy * std::cos(yaw);
        d_out.dyaw     = r;
        d_out.dvx      = Fx_total / m + vy * r;
        // Kinematic-dynamic blend (Kong 2015 / Polack 2017): the *lateral* states
        // (vy, r) cross-fade from pure dynamic at speed to pure slip-free kinematic
        // at rest. The lateral *force* is already faded by lambda (above), so the
        // dynamic part dies on its own as Vx->0; we just add the (1-lambda)
        // kinematic relaxation that pulls (vy, r) onto the geometric constraint
        // r = vx·tan(delta)/L. At rest this is pure kinematic (no force-induced
        // yaw -> no spin / stop oscillation); at lambda=1 it is the validated
        // dynamic model. Longitudinal (vx) stays fully dynamic.
        const double dvy_dyn = Fy_total / m - vx * r;
        const double dr_dyn  = Mz_total / Izz;
        const double delta_f = 0.5 * (d_wheel[WHEEL_FL] + d_wheel[WHEEL_FR]);
        const double r_kin   = (L > 1e-6) ? vx * std::tan(delta_f) / L : 0.0;
        const double vy_kin  = r_kin * b;
        if (lugre_on) {
            d_out.dvy = dvy_dyn;
            d_out.dr  = dr_dyn;
        } else {
            d_out.dvy = dvy_dyn + (1.0 - lambda) * (vy_kin - vy) / kKinTau;
            d_out.dr  = dr_dyn  + (1.0 - lambda) * (r_kin  - r ) / kKinTau;
        }
        d_out.ax_body  = Fx_total / m;
        d_out.ay_body  = Fy_total / m;
        const bool open_diff = vp_.differential == VehicleParams::Differential::Open;
        double I_axle_f = 0.0, I_axle_r = 0.0;
        axle_reflected_shares(vp_, I_axle_f, I_axle_r);
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double T_net = Td[i] + Tb[i] - fx_kin[i] * R;
            const double I_eng = wheel_engine_inertia_share(vp_, i);
            if (open_diff && I_eng > 0.0) {
                d_out.domega[i] = T_net / I_wheel_[i];
            } else {
                d_out.domega[i] = T_net / (I_wheel_[i] + I_eng);
            }
        }
        if (open_diff) {
            if (I_axle_f > 0.0) {
                couple_open_axle_spin(d_out.domega[WHEEL_FL], d_out.domega[WHEEL_FR],
                                      I_wheel_[WHEEL_FL], I_wheel_[WHEEL_FR], I_axle_f);
            }
            if (I_axle_r > 0.0) {
                couple_open_axle_spin(d_out.domega[WHEEL_RL], d_out.domega[WHEEL_RR],
                                      I_wheel_[WHEEL_RL], I_wheel_[WHEEL_RR], I_axle_r);
            }
        }

        // ---- Diagnostics ----
        tire_F_   = tire_F_disp;
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

    void advance_lugre_states_(const ContactArray& contacts, double h) {
        if (!tp_.lugre.enabled) return;
        const double sigma0 = std::max(1.0, tp_.lugre.sigma0);
        const double Rloc = vp_.wheel_radius_nominal;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double v_slip_long = Rloc * wheel_spin_last_[i] - v_x_wheel_last_[i];
            const double v_slip_lat  = v_y_wheel_last_[i];
            ITireModel::Input in;
            in.Fz = tire_Fz_[i];
            in.kappa = slip_ratio_[i];
            in.alpha = slip_angle_[i];
            in.mu_long = contacts[i].mu_long;
            in.mu_lat  = contacts[i].mu_lat;
            in.Vx_wheel = v_x_wheel_last_[i];
            const auto mf = tire_->compute(in);
            lugre_z_long_[i] = lugre_advance_z(
                lugre_z_long_[i], v_slip_long,
                lugre_breakaway(mf, true, in.Fz, in.mu_long, in.mu_lat), sigma0, h);
            lugre_z_lat_[i] = lugre_advance_z(
                lugre_z_lat_[i], v_slip_lat,
                lugre_breakaway(mf, false, in.Fz, in.mu_long, in.mu_lat, in.alpha), sigma0, h);
        }
    }

    void substep(const CmdL4& cmd, const ContactArray& contacts, double h) {
        const State s0 = state_;
        const bool lugre_on = tp_.lugre.enabled;
        const double hz = 0.25 * h;
        if (sp_.integrator == SolverParams::Integrator::Euler) {
            const Deriv k = derivatives(s0, cmd, contacts);
            state_   = apply(s0, k, h);
            ax_prev_ = k.ax_body;
            ay_prev_ = k.ay_body;
            if (lugre_on) advance_lugre_states_(contacts, h);
            return;
        }
        const Deriv k1 = derivatives(s0,                       cmd, contacts);
        if (lugre_on) advance_lugre_states_(contacts, hz);
        const Deriv k2 = derivatives(apply(s0, k1, 0.5 * h),  cmd, contacts);
        if (lugre_on) advance_lugre_states_(contacts, hz);
        const Deriv k3 = derivatives(apply(s0, k2, 0.5 * h),  cmd, contacts);
        if (lugre_on) advance_lugre_states_(contacts, hz);
        const Deriv k4 = derivatives(apply(s0, k3, h),        cmd, contacts);
        if (lugre_on) advance_lugre_states_(contacts, hz);

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
        if (!tp_.lugre.enabled && tp_.relaxation_length_lat > 1e-6) {
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
    std::unique_ptr<IDrivetrain>     drivetrain_;
    std::unique_ptr<IBrakeSystem>    brake_;
    std::unique_ptr<ISteeringSystem> steering_;
    std::array<double, NUM_WHEELS> I_wheel_ {{1.0, 1.0, 1.0, 1.0}};

    State state_;
    double ax_prev_ {0.0};
    double ay_prev_ {0.0};
    double ax_felt_ {0.0};   // specific force (accel - tangential gravity) for transfer/attitude
    double ay_felt_ {0.0};
    double roll_qs_ {0.0};   // quasi-static roll/pitch incl. CG-migration (set in step)
    double pitch_qs_ {0.0};
    double mz_front_sum_ {0.0};
    std::array<Vec3,   NUM_WHEELS> tire_F_     {};
    std::array<double, NUM_WHEELS> tire_Fz_    {};
    std::array<double, NUM_WHEELS> slip_ratio_ {};
    std::array<double, NUM_WHEELS> slip_angle_ {};
    // Transient slip-angle state (relaxation length).  Updated between substeps.
    std::array<double, NUM_WHEELS> alpha_dyn_       {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, NUM_WHEELS> alpha_geom_last_ {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, NUM_WHEELS> v_x_wheel_last_  {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, NUM_WHEELS> v_y_wheel_last_  {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, NUM_WHEELS> wheel_spin_last_ {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, NUM_WHEELS> lugre_z_long_   {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, NUM_WHEELS> lugre_z_lat_    {{0.0, 0.0, 0.0, 0.0}};
    // External per-wheel camber input (set by Ld3 or caller before step).
    std::array<double, NUM_WHEELS> camber_ext_      {{0.0, 0.0, 0.0, 0.0}};
    // External per-wheel toe input (additive to Ackerman steer angle).
    std::array<double, NUM_WHEELS> toe_ext_         {{0.0, 0.0, 0.0, 0.0}};
    // External per-wheel Fz for grip (L3 dynamic load), one-shot per step.
    std::array<double, NUM_WHEELS> ext_fz_          {{0.0, 0.0, 0.0, 0.0}};
    bool use_ext_fz_ {false};
};

}  // namespace

std::unique_ptr<IVehicleDynamics> create_seven_dof() {
    return std::make_unique<SevenDOFDynamics>(create_pacejka_mf96());
}

std::unique_ptr<IVehicleDynamics> create_seven_dof(std::unique_ptr<ITireModel> tire) {
    return std::make_unique<SevenDOFDynamics>(std::move(tire));
}

}  // namespace vdsim
