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
#include "vdsim/contact_frame.hpp"
#include "vdsim/default_subsystems.hpp"
#include "vdsim/drivetrain_inertia.hpp"
#include "vdsim/steering_kinematics.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/ladder_lowering.hpp"
#include "vdsim/low_speed.hpp"
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
            motor_brake_torque_to_pedal(T_drive, T_brake, out.throttle, out.brake);
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL2>) {
            axle_torque_to_pedal(cmd.drive_torque, cmd.brake_torque,
                                 out.throttle, out.brake);
            out.steer_angle_wheel = cmd.steer_angle_wheel;
        } else if constexpr (std::is_same_v<T, CmdL3>) {
            const double scale = fx_total_pedal_scale(cmd.Fx_total);
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
// kStickBlend / kStickC / kKinTau: shared low-speed constants in vdsim/low_speed.hpp

class SevenDOFDynamics final : public IVehicleDynamics {
public:
    SevenDOFDynamics() = default;
    explicit SevenDOFDynamics(std::unique_ptr<ITireModel> tire)
        : inj_tire_(std::move(tire)) {}

    Level level() const noexcept override { return Level::L2_SevenDOF; }

    void initialize(const VehicleParams& vp,
                    const TireSetup& ts,
                    const SolverParams& sp) override {
        vp_ = vp;
        ts_ = ts;
        sp_ = sp;
        init_wheel_tire_models(tire_, ts_, &inj_tire_);
        const double R = vp.wheel_radius_nominal;
        Re_w_.fill(R);   // until the first force loop writes Re(Fz); keeps the
                         // first-substep LuGre slip velocity on the nominal radius.
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
        steering_   = vp_.steering_dynamic
                          ? std::unique_ptr<ISteeringSystem>(std::make_unique<DynamicSteering>(vp_))
                          : make_default_steering(vp_, vp_.steer_deadtime_s);
        steer_kin_  = make_ratio_steering_kinematics(
                          std::max(1e-6, vp_.steering_ratio));
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
        Re_w_.fill(vp_.wheel_radius_nominal > 1e-6 ? vp_.wheel_radius_nominal : 0.32);
        contact_dy_.fill(0.0);
        mx_w_.fill(0.0);
        transient_.fill(ITireModel::Transient{});
        ci_.fill(ITireModel::ContactInput{});
        drivetrain_ = make_default_drivetrain(vp_, vp_.drive_deadtime_s);
        brake_      = make_default_brake(vp_, vp_.brake_deadtime_s);
        steering_   = vp_.steering_dynamic
                          ? std::unique_ptr<ISteeringSystem>(std::make_unique<DynamicSteering>(vp_))
                          : make_default_steering(vp_, vp_.steer_deadtime_s);
        steer_kin_  = make_ratio_steering_kinematics(
                          std::max(1e-6, vp_.steering_ratio));
        // Clear diagnostics so accessors don't return stale values before step().
        tire_F_.fill(Vec3::Zero());
        tire_F_wheel_.fill(Vec3::Zero());
        tire_Fz_.fill(0.0);
        slip_ratio_.fill(0.0);
        slip_angle_.fill(0.0);
        wheel_mu_.fill(0.0);
        wheel_mu_peak_.fill(0.0);
        wheel_alpha_peak_.fill(0.0);
        wheel_kappa_peak_.fill(0.0);
        mz_front_sum_ = 0.0;
        direct_l1_ = false;
    }

    void step(const ControlInput& u,
              const ContactArray& contacts,
              double dt) noexcept override {
        direct_l1_ = false;
        if (std::holds_alternative<CmdL1>(u)) {
            direct_l1_ = true;
            cmd_l1_    = std::get<CmdL1>(u);
        } else {
            cmd_l4_ = lower_to_l4(u);
        }
        if (!(dt > 0.0)) return;

        if (!direct_l1_) {
            const DriverCmd driver_cmd{
                cmd_l4_.steer_angle_wheel,
                cmd_l4_.throttle,
                cmd_l4_.brake,
                cmd_l4_.gear,
                cmd_l4_.handbrake,
                cmd_l4_.steer_mode,
                cmd_l4_.steer_actuator,
            };
            SubsystemContext ctx{state_, driver_cmd, dt};
            brake_->begin_step(ctx, dt);
            drivetrain_->begin_step(ctx, dt);
            steering_->begin_step(ctx, dt);
        } else {
            const DriverCmd driver_cmd{
                cmd_l1_.steer_angle_wheel, 0.0, 0.0, 1, false,
                SteerMode::Angle, 0.0,
            };
            SubsystemContext ctx{state_, driver_cmd, dt};
            steering_->begin_step(ctx, dt);
        }

        const int N = std::max(1,
                       std::min(sp_.max_substeps,
                                static_cast<int>(std::ceil(dt / sp_.max_substep_dt))));
        const double h = dt / static_cast<double>(N);
        for (int i = 0; i < N; ++i) substep(contacts, h);
    }

    const State& state() const noexcept override { return state_; }

    std::array<Vec3, NUM_WHEELS>   tire_forces_body() const override { return tire_F_; }
    std::array<Vec3, NUM_WHEELS>   tire_forces_wheel() const override { return tire_F_wheel_; }
    double engine_rpm()   const override { return drivetrain_->engine_rpm(); }
    int    current_gear() const override { return drivetrain_->current_gear(); }
    bool   set_shift_policy(ShiftPolicy fn) override {
        return drivetrain_->set_shift_policy(std::move(fn)); }
    bool   set_brake_module(std::shared_ptr<IBrakeSystem> m) override {
        if (!m) return false;
        brake_ = std::move(m);
        return true;
    }
    bool   set_steering_module(std::shared_ptr<ISteeringSystem> m) override {
        if (!m) return false;
        steering_ = std::move(m);
        return true;
    }
    bool   set_drivetrain_module(std::shared_ptr<IDrivetrain> m) override {
        if (!m) return false;
        drivetrain_ = std::move(m);
        return true;
    }
    std::array<double, NUM_WHEELS> tire_Fz()           const override { return tire_Fz_; }
    std::array<double, NUM_WHEELS> wheel_slip_ratio()  const override { return slip_ratio_; }
    std::array<double, NUM_WHEELS> wheel_slip_angle()  const override { return slip_angle_; }
    std::array<double, NUM_WHEELS> wheel_mu()          const override { return wheel_mu_; }
    std::array<double, NUM_WHEELS> wheel_mu_peak()     const override { return wheel_mu_peak_; }
    std::array<double, NUM_WHEELS> wheel_alpha_peak()  const override { return wheel_alpha_peak_; }
    std::array<double, NUM_WHEELS> wheel_kappa_peak()  const override { return wheel_kappa_peak_; }
    const ITireModel* tire(int wheel) const override {
        if (wheel < 0 || wheel >= NUM_WHEELS) return nullptr;
        return tire_[wheel].get();
    }
    std::array<double, NUM_WHEELS> wheel_overturning_moment() const override { return mx_w_; }

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
        // Rack EOM (Dynamic steering mode): drack_travel = rack_velocity,
        // drack_velocity = (F_motor - F_tire_fb - c_rack * rack_vel) / m_rack
        double drack_travel   {0.0};
        double drack_velocity {0.0};
    };

    static int axle_of(int i) { return (i < 2) ? 0 : 1; }   // 0 = front, 1 = rear
    static int side_of(int i) { return i % 2; }              // 0 = left (FL/RL), 1 = right (FR/RR)

    Deriv derivatives(const State& s,
                      const ContactArray& contacts) {
        const CmdL4& cmd = cmd_l4_;
        const double steer_cmd = direct_l1_ ? cmd_l1_.steer_angle_wheel
                                            : cmd.steer_angle_wheel;
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
        Vec3 n_road = Vec3::Zero();
        for (const auto& contact : contacts) {
            if (!contact.is_valid || !contact.normal.allFinite()) continue;
            const double normal_norm = contact.normal.norm();
            if (normal_norm > 1e-12) n_road += contact.normal / normal_norm;
        }
        const double nn = n_road.norm();
        const bool has_road_normal = nn > 1e-9;
        if (has_road_normal) n_road /= nn;
        const double cos_slope = has_road_normal
            ? std::max(0.1, n_road.z()) : 0.0;
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
        // `is_valid=false` is non-contact, including on the L3/L4 external-Fz
        // path.  Re-mask after the override so no normal load, rolling
        // resistance, tire wrench, or transient update leaks through.
        for (int i = 0; i < NUM_WHEELS; ++i)
            if (!contacts[i].is_valid) Fz[i] = 0.0;

        const DriverCmd driver_cmd{
            steer_cmd,
            direct_l1_ ? 0.0 : cmd.throttle,
            direct_l1_ ? 0.0 : cmd.brake,
            direct_l1_ ? 1   : cmd.gear,
            direct_l1_ ? false : cmd.handbrake,
            direct_l1_ ? SteerMode::Angle : cmd.steer_mode,
            direct_l1_ ? 0.0 : cmd.steer_actuator,
        };
        SubsystemContext ctx{s, driver_cmd, 0.0};
        ctx.Fz = Fz;
        const SteeringOutput steer_out = steering_->apply(ctx);

        // Steer angle from rack_travel.
        // Dynamic mode: rack_travel from State (RK4-integrated via Rack EOM below).
        // Kinematic mode: rack_travel from steer_out (subsystem sets it directly).
        // Legacy subsystems set roadwheel_angle only (rack_travel=0) → fall through.
        const double rack_for_kin = (steer_out.mode == SteeringOutput::Mode::Dynamic)
            ? s.rack_travel : steer_out.rack_travel;
        const double d = (steer_kin_ && std::abs(rack_for_kin) > 1e-9)
            ? steer_kin_->compute(rack_for_kin).angle_avg()
            : steer_out.roadwheel_angle;

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
        std::array<Vec3,   NUM_WHEELS> tire_F_wheel_disp;  // same, wheel frame (pre steer rotation)
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

        const bool plant    = vp_.plant_path;
        const double speed = std::hypot(vx, vy);
        double lambda_body = 1.0;
        if (!plant) {
            double lam_min = 1.0;
            for (int wi = 0; wi < NUM_WHEELS; ++wi) {
                if (ts_.for_wheel(wi).lugre.enabled) continue;
                double lam = std::clamp(speed / kStickBlend, 0.0, 1.0);
                lam = lam * lam * (3.0 - 2.0 * lam);
                lam_min = std::min(lam_min, lam);
            }
            lambda_body = lam_min;
        }
        std::array<double, NUM_WHEELS> fx_kin {{0.0, 0.0, 0.0, 0.0}};

        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double di = d_wheel[i];
            const double cd_i = std::cos(di), sd_i = std::sin(di);
            // Contact normals are injected in world ENU.  Resolve them into
            // the planar body frame, then construct one shared exact contact
            // basis for both L3 and L4 (which delegate through this path).
            const Vec3& nw = contacts[i].normal;
            const Vec3 normal_body(
                std::cos(yaw) * nw.x() + std::sin(yaw) * nw.y(),
               -std::sin(yaw) * nw.x() + std::cos(yaw) * nw.y(),
                nw.z());
            const ContactFrame contact_frame = make_contact_frame(
                normal_body, Vec3(cd_i, sd_i, 0.0));
            if (!contacts[i].is_valid || !contact_frame.valid) {
                F_body[i]      = Vec3::Zero();
                tire_F_disp[i] = Vec3::Zero();
                tire_F_wheel_disp[i] = Vec3::Zero();
                kappa[i]       = 0.0;
                alpha[i]       = 0.0;
                mz_wheel[i]    = 0.0;
                mx_w_[i]       = 0.0;   // airborne: no overturning (don't leak stale Mx to roll)
                contact_dy_[i] = 0.0;
                fx_kin[i]      = 0.0;
                wheel_mu_[i]       = 0.0;
                wheel_mu_peak_[i]  = 0.0;
                wheel_alpha_peak_[i] = 0.0;
                wheel_kappa_peak_[i] = 0.0;
                ci_[i]         = ITireModel::ContactInput{};  // neutral: no bristle/relax evolution
                continue;
            }
            const double v_x_body = vx - r * r_y[i];
            const double v_y_body = vy + r * r_x[i];
            const Vec3 contact_velocity = contact_frame.resolve_velocity(
                Vec3(v_x_body, v_y_body, 0.0));

            const double mu_long_i = contacts[i].mu_long;
            const double mu_lat_i  = contacts[i].mu_lat;
            wheel_mu_[i] = std::min(mu_long_i, mu_lat_i);

            const bool lugre_i = ts_.for_wheel(i).lugre.enabled;
            const double lambda_i = (lugre_i || plant) ? 1.0 : lambda_body;

            // Raw contact kinematics + load handed to the tire; the tire owns slip /
            // Re / camber migration / transient. Stored per wheel so the per-stage
            // advance_bristle() / per-substep advance_relaxation() see this stage's
            // kinematics (mirrors the old *_last_ scratch).
            ITireModel::ContactInput ci;
            ci.Fz = Fz[i];
            ci.Vx = contact_velocity.x();
            ci.Vy = contact_velocity.y();
            ci.omega = s.wheel_spin[i]; ci.gamma = camber_ext_[i];
            ci.mu_long = mu_long_i; ci.mu_lat = mu_lat_i; ci.R0 = R;
            ci_[i] = ci;

            // Frozen per-RK4-stage force: slip / Re / camber / transient all live in
            // the tire now. The transient_ state is held frozen within the stage and
            // advanced between stages/substeps below.
            const ITireModel::Wrench w = wheel_tire_model(tire_, i)
                                             .evaluate(ci, transient_[i]);
            wheel_mu_peak_[i] = w.mu_peak;
            wheel_alpha_peak_[i] = w.alpha_peak;
            wheel_kappa_peak_[i] = w.kappa_peak;
            Re_w_[i]       = w.Re;
            contact_dy_[i] = w.contact_dy;
            mx_w_[i]       = w.Mx;        // camber migration (+ carcass Mx if any)
            const double k_slip = w.kappa;
            const double a_slip = w.alpha;

            const double muFz = std::min(mu_long_i, mu_lat_i) * std::max(0.0, Fz[i]);

            // Vehicle-side low-speed shaping stays here (integrator stabilization, not
            // tire physics): stick-blend lambda, Fx_hold creep, combined-slip clamp.
            double Fx_w = 0.0, Fy_w = 0.0;
            double Fxd = 0.0, Fyd = 0.0;
            if (lugre_i) {
                Fx_w = w.Fx;
                Fy_w = w.Fy;
                mz_wheel[i] = w.Mz;
                Fxd = Fx_w;
                Fyd = Fy_w;
            } else {
                const double hold_gate = plant ? 0.0
                    : (1.0 - lambda_i) * std::clamp(cmd.brake - cmd.throttle, 0.0, 1.0);
                double Fx_hold = -kStickC * contact_velocity.x() * hold_gate;
                if (std::abs(Fx_hold) > muFz) Fx_hold = std::copysign(muFz, Fx_hold);
                Fx_w = w.Fx + Fx_hold;
                Fy_w = lambda_i * w.Fy;
                fx_kin[i] = w.Fx;
                mz_wheel[i] = w.Mz * lambda_i;
                Fxd = lambda_i * w.Fx + Fx_hold;
                Fyd = Fy_w;
            }
            const double Fmag = std::hypot(Fx_w, Fy_w);
            if (!ts_.for_wheel(i).model_provides_combined_slip() && Fmag > muFz && Fmag > 1e-9) {
                const double c = muFz / Fmag;
                Fx_w *= c;
                Fy_w *= c;
            }
            if (lugre_i) fx_kin[i] = Fx_w;
            F_body[i] = contact_frame.tangential_force(Fx_w, Fy_w);
            const double Fdm = std::hypot(Fxd, Fyd);
            if (!ts_.for_wheel(i).model_provides_combined_slip() && Fdm > muFz && Fdm > 1e-9) {
                const double c = muFz / Fdm;
                Fxd *= c;
                Fyd *= c;
            }
            tire_F_disp[i] = contact_frame.tangential_force(Fxd, Fyd);
            tire_F_wheel_disp[i] = Vec3(Fxd, Fyd, 0.0);
            kappa[i]  = k_slip;
            alpha[i]  = a_slip;
        }

        const auto dt_out = direct_l1_ ? DrivetrainOutput{}
                                       : drivetrain_->apply(ctx);
        const std::array<double, NUM_WHEELS> Td = dt_out.wheel_torque;
        std::array<double, NUM_WHEELS> Tb {{0.0, 0.0, 0.0, 0.0}};
        if (!direct_l1_) {
            const double effective_brake = driver_cmd.brake
                                           * (1.0 - std::clamp(dt_out.brake_absorbed, 0.0, 1.0));
            const DriverCmd cmd_residual{driver_cmd.handwheel_angle,
                                         driver_cmd.throttle, effective_brake,
                                         driver_cmd.gear, driver_cmd.handbrake};
            const SubsystemContext ctx_brake{s, cmd_residual, ctx.dt, ctx.Fz};
            Tb = brake_->wheel_torque(ctx_brake);
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
        double F_rr = 0.0;
        for (int wi = 0; wi < NUM_WHEELS; ++wi)
            F_rr += ts_.for_wheel(wi).rolling_resistance * Fz[wi];
        F_rr *= std::tanh(vx / 0.5);
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
        bool lugre_all = true;
        for (int wi = 0; wi < NUM_WHEELS; ++wi)
            lugre_all = lugre_all && ts_.for_wheel(wi).lugre.enabled;
        if (lugre_all || plant) {
            d_out.dvy = dvy_dyn;
            d_out.dr  = dr_dyn;
        } else {
            d_out.dvy = dvy_dyn + (1.0 - lambda_body) * (vy_kin - vy) / kKinTau;
            d_out.dr  = dr_dyn  + (1.0 - lambda_body) * (r_kin  - r ) / kKinTau;
        }
        d_out.ax_body  = Fx_total / m;
        d_out.ay_body  = Fy_total / m;
        const bool open_diff = vp_.differential == VehicleParams::Differential::Open;
        // Effective per-wheel reflected engine inertia: gear-dependent from the
        // powertrain when it provides one (sentinel <0 -> legacy final-drive
        // reflection). The open-diff carrier inertia is derived from the SAME
        // per-wheel shares (axle = sum of its pair) so the divisor and the
        // coupling never disagree. For the legacy path this reproduces
        // axle_reflected_shares exactly (per-wheel share = 0.5 * axle).
        std::array<double, NUM_WHEELS> I_eng_w{};
        for (int i = 0; i < NUM_WHEELS; ++i) {
            double ie = drivetrain_->wheel_engine_inertia(i);
            I_eng_w[i] = (ie < 0.0) ? wheel_engine_inertia_share(vp_, i) : ie;
        }
        const double I_axle_f = I_eng_w[WHEEL_FL] + I_eng_w[WHEEL_FR];
        const double I_axle_r = I_eng_w[WHEEL_RL] + I_eng_w[WHEEL_RR];
        // Tire-force reaction torque uses the same effective rolling radius as the
        // slip definition (energetically consistent: free roll → kappa=0 → Fx=0).
        std::array<double, NUM_WHEELS> T_net{};
        if (direct_l1_) {
            // Brake opposes wheel rotation and must NOT drive it backwards (a friction brake
            // can only resist, not propel). tanh(omega/eps) gives the signed, vanishing-at-rest
            // brake direction, so a locked wheel settles at kappa~-1 instead of spinning
            // backward to kappa->-inf under a constant over-brake torque.
            constexpr double kBrakeOmegaEps = 2.0;  // [rad/s]
            for (int i = 0; i < NUM_WHEELS; ++i) {
                const double brake = cmd_l1_.brake_torque[i]
                    * std::tanh(s.wheel_spin[i] / kBrakeOmegaEps);
                T_net[i] = cmd_l1_.motor_torque[i] - brake - fx_kin[i] * Re_w_[i];
            }
        } else {
            for (int i = 0; i < NUM_WHEELS; ++i)
                T_net[i] = Td[i] + Tb[i] - fx_kin[i] * Re_w_[i];
        }
        if (open_diff) {
            // Open diff: each axle's engine/carrier inertia couples its wheel pair
            // (felt under symmetric accel, transparent to wheel-to-wheel differences).
            open_axle_spin_accel(d_out.domega[WHEEL_FL], d_out.domega[WHEEL_FR],
                                 T_net[WHEEL_FL], T_net[WHEEL_FR],
                                 I_wheel_[WHEEL_FL], I_wheel_[WHEEL_FR], I_axle_f);
            open_axle_spin_accel(d_out.domega[WHEEL_RL], d_out.domega[WHEEL_RR],
                                 T_net[WHEEL_RL], T_net[WHEEL_RR],
                                 I_wheel_[WHEEL_RL], I_wheel_[WHEEL_RR], I_axle_r);
        } else {
            // Locked / LSD: each wheel rigidly carries its half of the axle inertia.
            for (int i = 0; i < NUM_WHEELS; ++i)
                d_out.domega[i] = T_net[i] / (I_wheel_[i] + I_eng_w[i]);
        }

        // ---- Rack EOM (Dynamic steering mode only) ----
        // m_rack * xr'' = F_motor - F_tire_fb - c_rack * xr'
        //   F_tire_fb = sum(Mz_front)/R_pinion + sum(Fy_front)*caster_trail
        // Kinematic mode leaves drack_* = 0 (rack position is a constraint, not a state).
        if (steer_out.mode == SteeringOutput::Mode::Dynamic) {
            const double m_rack   = std::max(1e-3, vp_.rack_mass);
            const double c_rack   = vp_.rack_damping;
            const double R_pinion = std::max(1e-4, vp_.pinion_radius);
            const double trail    = vp_.caster_trail;
            const double Mz_front = mz_wheel[WHEEL_FL] + mz_wheel[WHEEL_FR];
            const double Fy_front = F_body[WHEEL_FL].y() + F_body[WHEEL_FR].y();
            const double F_tire_fb = Mz_front / R_pinion + Fy_front * trail;
            d_out.drack_travel   = s.rack_velocity;
            d_out.drack_velocity = (steer_out.motor_force - F_tire_fb
                                    - c_rack * s.rack_velocity) / m_rack;
        }

        // ---- Diagnostics ----
        tire_F_       = tire_F_disp;
        tire_F_wheel_ = tire_F_wheel_disp;
        tire_Fz_  = Fz;  // scalar normal-load magnitude along each contact z_c
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
        // Rack EOM (Dynamic steering mode only; zero when Kinematic).
        s.rack_travel   = s0.rack_travel   + d.drack_travel   * h;
        s.rack_velocity = s0.rack_velocity + d.drack_velocity * h;
        // Steering lock: clamp rack travel to ±(max_steer · ratio); kill velocity
        // into the stop so a constant motor torque rests at the lock (no runaway).
        const double rack_lock = vp_.max_steer_angle_wheel * std::max(1e-6, vp_.steering_ratio);
        if (s.rack_travel >  rack_lock) { s.rack_travel =  rack_lock; if (s.rack_velocity > 0.0) s.rack_velocity = 0.0; }
        if (s.rack_travel < -rack_lock) { s.rack_travel = -rack_lock; if (s.rack_velocity < 0.0) s.rack_velocity = 0.0; }
        return s;
    }

    // LuGre bristle z — advanced once per RK4 stage at the sub-substep dt, against the
    // kinematics of the stage just evaluated (stored in ci_).
    void advance_bristle_(double dt) {
        for (int i = 0; i < NUM_WHEELS; ++i) {
            if (!ts_.for_wheel(i).lugre.enabled) continue;
            transient_[i] = wheel_tire_model(tire_, i)
                                .advance_bristle(ci_[i], transient_[i], dt);
        }
    }
    // Carcass/belt + relaxation-length lag — advanced once per substep at the full dt,
    void advance_relaxation_(double dt) {
        for (int i = 0; i < NUM_WHEELS; ++i)
            transient_[i] = wheel_tire_model(tire_, i)
                                .advance_relaxation(ci_[i], transient_[i], dt);
    }

    void substep(const ContactArray& contacts, double h) {
        const State s0 = state_;
        const bool lugre_on = [&]() {
            for (int wi = 0; wi < NUM_WHEELS; ++wi)
                if (ts_.for_wheel(wi).lugre.enabled) return true;
            return false;
        }();
        const double hz = 0.25 * h;
        if (sp_.integrator == SolverParams::Integrator::Euler) {
            const Deriv k = derivatives(s0, contacts);
            state_   = apply(s0, k, h);
            ax_prev_ = k.ax_body;
            ay_prev_ = k.ay_body;
            if (lugre_on) advance_bristle_(h);
            return;
        }
        const Deriv k1 = derivatives(s0,                       contacts);
        if (lugre_on) advance_bristle_(hz);
        const Deriv k2 = derivatives(apply(s0, k1, 0.5 * h),  contacts);
        if (lugre_on) advance_bristle_(hz);
        const Deriv k3 = derivatives(apply(s0, k2, 0.5 * h),  contacts);
        if (lugre_on) advance_bristle_(hz);
        const Deriv k4 = derivatives(apply(s0, k3, h),        contacts);
        if (lugre_on) advance_bristle_(hz);

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
        k.drack_travel   = (k1.drack_travel   + 2*k2.drack_travel   + 2*k3.drack_travel   + k4.drack_travel)   / 6.0;
        k.drack_velocity = (k1.drack_velocity + 2*k2.drack_velocity + 2*k3.drack_velocity + k4.drack_velocity) / 6.0;
        state_   = apply(s0, k, h);
        ax_prev_ = k.ax_body;
        ay_prev_ = k.ay_body;
        // Carcass relaxation + relaxation-length lag, once per substep against the final
        // stage's geometric slip. The tire owns the update; the cadence (per-substep at h,
        // not per-stage) is preserved here. See advance_relaxation_.
        advance_relaxation_(h);
    }

    VehicleParams vp_;
    TireSetup     ts_;
    SolverParams  sp_;
    std::unique_ptr<ITireModel> inj_tire_;
    std::array<std::unique_ptr<ITireModel>, NUM_WHEELS> tire_ {};
    std::shared_ptr<IDrivetrain>         drivetrain_;
    std::shared_ptr<IBrakeSystem>        brake_;
    std::shared_ptr<ISteeringSystem>     steering_;
    std::unique_ptr<ISteeringKinematics> steer_kin_;  // rack_travel → FL/FR angles
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
    std::array<Vec3,   NUM_WHEELS> tire_F_wheel_ {};
    std::array<double, NUM_WHEELS> tire_Fz_    {};
    std::array<double, NUM_WHEELS> slip_ratio_ {};
    std::array<double, NUM_WHEELS> slip_angle_ {};
    std::array<double, NUM_WHEELS> wheel_mu_       {};
    std::array<double, NUM_WHEELS> wheel_mu_peak_ {};
    std::array<double, NUM_WHEELS> wheel_alpha_peak_ {};
    std::array<double, NUM_WHEELS> wheel_kappa_peak_ {};
    CmdL4 cmd_l4_ {};
    CmdL1 cmd_l1_ {};
    bool  direct_l1_ {false};
    // Per-wheel tire transient (belt / relaxation / LuGre bristle z), OWNED here so the
    // RK4 integrator can freeze/advance it; the tire backend integrates it via
    // advance_bristle()/advance_relaxation(). ci_ holds the contact kinematics the tire
    // was last evaluated at (per stage), feeding those advance calls.
    std::array<ITireModel::Transient, NUM_WHEELS> transient_ {};
    mutable std::array<ITireModel::ContactInput, NUM_WHEELS> ci_ {};
    mutable std::array<double, NUM_WHEELS> Re_w_ {{0.31, 0.31, 0.31, 0.31}};  // effective rolling radius per wheel
    mutable std::array<double, NUM_WHEELS> contact_dy_ {{0.0, 0.0, 0.0, 0.0}};  // camber lateral contact offset
    mutable std::array<double, NUM_WHEELS> mx_w_ {{0.0, 0.0, 0.0, 0.0}};  // per-wheel overturning moment
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
    return std::make_unique<SevenDOFDynamics>();
}

std::unique_ptr<IVehicleDynamics> create_seven_dof(std::unique_ptr<ITireModel> tire) {
    return std::make_unique<SevenDOFDynamics>(std::move(tire));
}

}  // namespace vdsim
