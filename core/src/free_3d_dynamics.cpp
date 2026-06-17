#include "vdsim/coordinate.hpp"
#include "vdsim/default_subsystems.hpp"
#include "vdsim/drivetrain_inertia.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/subsystems.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
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
constexpr double kStickBlend   = 3.0;
constexpr double kStickC       = 6.0e4;
constexpr double kWheelSpinDrag = 2.5;
constexpr double kTireDampRatio = 0.08;   // tire-spring damping ratio (strut path)
constexpr double kRackPerRad    = 0.08;   // tie-rod rack travel per roadwheel rad (mirrors L4)
constexpr double kStopStiffness = 2.0e6;  // bump/rebound stop rate [N/m] (strut path)
// Free-3D unsprung: stiff perpendicular bushing standing in for the rigid suspension
// links (the wheel is located horizontally by the A-arms). Soft along the strut axis
// (the coilover); stiff perpendicular so the link is effectively rigid. At 1e8 N/m the
// cornering-load compliance is ~40 um (vs 0.4 mm at 1e7) — a hard link for all practical
// purposes — while the unsprung natural frequency (~250 Hz on 40 kg, critically damped)
// stays comfortably RK4-stable at 2e-4 s. A TRUE hard constraint is not used: in the
// k->inf limit the wheel collapses onto the strut line (x_u = mount + s*u_hat), which
// makes the spring deflection the independent coordinate again -> the reduced-coordinate
// loss of quasi-static roll stiffness. The stiff penalty IS that limit minus a few um and
// already delivers the link reaction to the body, so it keeps the roll stiffness. Verified
// stable up to 1e9 (4 um); 1e10 is RK4-unstable (omega*dt > 2.8). Env: VDSIM_KLINK.
constexpr double kLinkStiffness = 1.0e8;   // [N/m] perpendicular (rigid-link penalty)

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
        // Spatial-strut static preload: each strut carries the sprung corner load so
        // comp=0 is the static ride position (sum F_preload = m_sprung·g). Droop stop
        // at the spring's free length (F_susp=0); bump room above static.
        const double ms = vp.mass_sprung > 1.0 ? vp.mass_sprung : vp.mass;
        const double Lwb = vp.wheelbase > 1e-6 ? vp.wheelbase : 2.7;
        const double Fs_f = 0.5 * ms * kGravity * vp.cg_to_rear  / Lwb;
        const double Fs_r = 0.5 * ms * kGravity * vp.cg_to_front / Lwb;
        F_preload_ = {{Fs_f, Fs_f, Fs_r, Fs_r}};
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double ks = std::max(1.0, vp.spring_stiffness[i]);
            comp_min_[i] = -F_preload_[i] / ks;   // rebound stop engages (spring free length)
            comp_max_[i] =  0.10;                 // bump stop engages [m] above static
        }
        drivetrain_ = make_default_drivetrain(vp_, vp_.drive_deadtime_s);
        brake_      = make_default_brake(vp_, vp_.brake_deadtime_s);
        steering_   = make_default_steering(vp_, vp_.steer_deadtime_s);
        if (const char* kl = std::getenv("VDSIM_KLINK")) k_link_ = std::atof(kl);
        spdlog::debug("[L5 free-3D] init: mass={:.0f} kg, I=({:.0f},{:.0f},{:.0f})",
                      vp.mass, vp.inertia_diag.x(), vp.inertia_diag.y(), vp.inertia_diag.z());
    }

    void reset(const State& s) noexcept override {
        state_ = s;
        state_.orientation.normalize();
        // Seed the wheel-centre world position/velocity at the static mount (rides with the
        // body). The strut path then integrates these as free-3D particles; the penalty path
        // keeps them rigid. Maintained on every step (see apply), so always the true wheel
        // centre — no path-dependent special-casing.
        {
            const Mat3 R = state_.orientation.toRotationMatrix();
            for (int i = 0; i < NUM_WHEELS; ++i) {
                state_.unsprung_pos[i] = state_.position + R * r_body_[i];
                state_.unsprung_vel[i] =
                    R * (state_.velocity + state_.angular_velocity.cross(r_body_[i]));
                state_.susp_compression[i] = 0.0;
                state_.susp_velocity[i]    = 0.0;
                mount_query_[i] = state_.unsprung_pos[i];
            }
        }
        ax_prev_ = 0.0;
        ay_prev_ = 0.0;
        transient_.fill(ITireModel::Transient{});
        ci_.fill(ITireModel::ContactInput{});
        Re_w_.fill(vp_.wheel_radius_nominal > 1e-6 ? vp_.wheel_radius_nominal : 0.32);
        drivetrain_ = make_default_drivetrain(vp_, vp_.drive_deadtime_s);
        brake_      = make_default_brake(vp_, vp_.brake_deadtime_s);
        steering_   = make_default_steering(vp_, vp_.steer_deadtime_s);
        tire_F_.fill(Vec3::Zero());
        tire_Fz_.fill(0.0);
        slip_ratio_.fill(0.0);
        slip_angle_.fill(0.0);
        mz_front_sum_ = 0.0;
        gamma_dae_.fill(0.0);
        toe_dae_.fill(0.0);
        const mb::PrescribedCornerMotion mot {};
        if (mb_dae_front_) {
            mb_state_[WHEEL_FL] = {}; mb_state_[WHEEL_FR] = {};
            mb_dae_front_->initialize(mb_state_[WHEEL_FL], mot);
            mb_dae_front_->initialize(mb_state_[WHEEL_FR], mot);
        }
        if (mb_dae_rear_) {
            mb_state_[WHEEL_RL] = {}; mb_state_[WHEEL_RR] = {};
            mb_dae_rear_->initialize(mb_state_[WHEEL_RL], mot);
            mb_dae_rear_->initialize(mb_state_[WHEEL_RR], mot);
        }
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

        // Free-3D strut: freeze the wheel-particle position the (once-per-step) contact
        // query evaluated penetration at (the provider now samples the surface at the actual
        // x_u, not the rigid hub), so the substep correction only tracks the SMALL wheel
        // motion within the step — exact on curved surfaces, no planar-extrapolation false
        // contact when the wheel leaves a loop/bank.
        if (sp_.l5_spatial_suspension) {
            for (int i = 0; i < NUM_WHEELS; ++i)
                mount_query_[i] = state_.unsprung_pos[i];
        }

        // Advance the corner suspension kinematics once per outer step (toe/camber from
        // the prescribed strut travel), held constant through the substeps below — like
        // L4's once-per-step DAE update. No-op unless strut path + a corner DAE attached.
        update_corner_dae(cmd, dt);

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

    // B2: attach the L4 corner DAE for an axle (FL/FR if front, else RL/RR). The same
    // topology drives both corners; each keeps its own DAE state fed by its strut travel.
    void attach_mb_corner(bool front, mb::SuspensionTopology topo, bool enable) {
        mb::PrescribedCornerMotion mot {};
        if (front) {
            mb_topo_front_   = std::move(topo);
            mb_enabled_front_ = enable;
            mb_dae_front_ = enable ? mb::create_hard_joint_dae_model(mb_topo_front_) : nullptr;
            if (mb_dae_front_) {
                mb_dae_front_->initialize(mb_state_[WHEEL_FL], mot);
                mb_dae_front_->initialize(mb_state_[WHEEL_FR], mot);
            }
        } else {
            mb_topo_rear_   = std::move(topo);
            mb_enabled_rear_ = enable;
            mb_dae_rear_ = enable ? mb::create_hard_joint_dae_model(mb_topo_rear_) : nullptr;
            if (mb_dae_rear_) {
                mb_dae_rear_->initialize(mb_state_[WHEEL_RL], mot);
                mb_dae_rear_->initialize(mb_state_[WHEEL_RR], mot);
            }
        }
    }
    bool mb_enabled(int axle) const noexcept {
        return axle == 0 ? mb_enabled_front_ : mb_enabled_rear_;
    }
    void set_contact_provider(IContactProvider* p) noexcept { provider_ = p; }
    std::array<double, NUM_WHEELS> wheel_camber() const noexcept { return gamma_dae_; }
    std::array<double, NUM_WHEELS> wheel_toe()    const noexcept { return toe_dae_; }

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
        // Free-3D unsprung point-mass DOF (zero on the penalty path): d/dt of the
        // world-frame wheel-centre position (= unsprung velocity) and velocity (= accel).
        std::array<Vec3, NUM_WHEELS> d_u_pos {{Vec3::Zero(), Vec3::Zero(),
                                               Vec3::Zero(), Vec3::Zero()}};
        std::array<Vec3, NUM_WHEELS> d_u_vel {{Vec3::Zero(), Vec3::Zero(),
                                               Vec3::Zero(), Vec3::Zero()}};
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


    Deriv derivatives(const State& s,
                    const CmdL4& cmd,
                    const ContactArray& contacts) {
        const Mat3 R = s.orientation.toRotationMatrix();
        const Vec3 v_body = s.velocity;
        const Vec3 omega  = s.angular_velocity;
        const double m    = vp_.mass;
        const double Rwh  = vp_.wheel_radius_nominal;
        const double k_tire = std::min(80000.0, std::max(1.0, tp_.tire_vertical_stiffness));
        // Spatial-strut path: the 6-DOF body is the sprung mass; the (unclamped) tire
        // stiffness acts on the per-corner unsprung mass. Penalty path leaves m_body==m,
        // so its arithmetic is byte-identical to before.
        const bool strut = sp_.l5_spatial_suspension;
        // Body = sprung mass only: total minus the four unsprung particles, so the whole
        // vehicle sums back to the total mass (keeps flat handling matched to L2/L3).
        double m_unsprung_sum = 0.0;
        for (int i = 0; i < NUM_WHEELS; ++i)
            m_unsprung_sum += std::max(1.0, vp_.unsprung_mass[i]);
        const double m_body = strut ? std::max(1.0, m - m_unsprung_sum) : m;
        const double k_tire_strut = std::max(1.0, tp_.tire_vertical_stiffness);
        const double L = vp_.wheelbase;
        const double Tw_f = vp_.track_front;

        const DriverCmd driver_cmd{
            cmd.steer_angle_wheel, cmd.throttle, cmd.brake, cmd.gear, cmd.handbrake};
        SubsystemContext ctx{s, driver_cmd, 0.0};

        // Penalty-path contact cap (stability of the rigid-glued contact). The loop is now
        // a general surface (LoopGround penalty contact), so there is no loop-specific cap.
        const double Fz_cap = 6.0 * m * kGravity / static_cast<double>(NUM_WHEELS);
        const double c_vert = 2.0 * std::sqrt(k_tire * m / static_cast<double>(NUM_WHEELS));
        const Vec3 ez_world = R.col(2);   // body up axis in world (strut travel axis)
        std::array<double, NUM_WHEELS> Fz {};
        for (int i = 0; i < NUM_WHEELS; ++i) {
            if (!contacts[i].is_valid) { Fz[i] = 0.0; continue; }
            const Vec3& rb = r_body_[i];
            const Vec3 n = contacts[i].normal.normalized();
            if (strut) {
                // Free-3D unsprung: the wheel centre is the inertial particle x_u. The
                // contact provider reported penetration for the RIGID hub frozen at the
                // step-start mount (mount_query_); the real penetration corrects for the
                // wheel's displacement from that frozen hub along the contact normal.
                const Vec3 x_u = s.unsprung_pos[i];
                const Vec3 v_u = s.unsprung_vel[i];
                // The provider sampled the surface AT the wheel particle (wheel_world_positions
                // uses unsprung_pos), so penetration<=0 means the wheel is genuinely off the
                // surface -> no force. (Don't let the within-step correction below manufacture
                // contact from the clamped-to-zero gap, which gave false contact when the wheel
                // left a loop/bank.) When in contact, the correction tracks the wheel's small
                // within-step motion relative to the once-per-step contact query.
                const double pen_rigid = contacts[i].penetration;
                if (pen_rigid <= 0.0) { Fz[i] = 0.0; continue; }
                const double pen = pen_rigid - (x_u - mount_query_[i]).dot(n);
                if (pen <= 0.0) { Fz[i] = 0.0; continue; }
                const double pen_dot = -v_u.dot(n);   // road frozen -> rate is wheel only
                const double m_u = std::max(1.0, vp_.unsprung_mass[i]);
                const double c_t = kTireDampRatio * 2.0 * std::sqrt(k_tire_strut * m_u);
                // No artificial cap: the tire deforms, so Fz keeps rising with the
                // (penetration) deformation. The stunt Fz_cap is a penalty-path-only hack.
                Fz[i] = std::max(0.0, k_tire_strut * pen + c_t * pen_dot);
                continue;
            }
            if (contacts[i].penetration <= 0.0) { Fz[i] = 0.0; continue; }
            const Vec3 v_hub = R * (v_body + omega.cross(rb));
            const double pen = contacts[i].penetration;
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

        Vec3 F_total_world = Vec3(0.0, 0.0, -m_body * kGravity);
        Vec3 tau_body = Vec3::Zero();
        std::array<Vec3, NUM_WHEELS> F_body {};
        std::array<double, NUM_WHEELS> kappa {}, alpha {};
        std::array<double, NUM_WHEELS> mz_wheel {{0.0, 0.0, 0.0, 0.0}};
        std::array<double, NUM_WHEELS> fx_kin {{0.0, 0.0, 0.0, 0.0}};
        // Free-3D unsprung derivatives (zero on the penalty path): d(x_u)/dt = v_u and
        // d(v_u)/dt = a_u, both world frame.
        std::array<Vec3, NUM_WHEELS> d_upos {{Vec3::Zero(), Vec3::Zero(),
                                              Vec3::Zero(), Vec3::Zero()}};
        std::array<Vec3, NUM_WHEELS> d_uvel {{Vec3::Zero(), Vec3::Zero(),
                                              Vec3::Zero(), Vec3::Zero()}};

        for (int i = 0; i < NUM_WHEELS; ++i) {
            if (strut) {
                // Free-3D unsprung point mass x_u, connected to the body mount by a
                // two-point bushing: SOFT along the strut axis u_hat (= body up) is the
                // coilover; STIFF perpendicular stands in for the rigid links. The tire
                // force acts on the wheel (arbitrary-surface: Fz along the contact normal
                // n, slip from the real wheel-centre velocity v_u). The body feels ONLY
                // the connection reaction at the mount (+ its moment) — the tire reaches
                // the chassis through the bushing, so every body<->wheel force is a genuine
                // two-point element and the strut roll moment is delivered directly.
                const Vec3& rb = r_body_[i];
                const Vec3 mount   = s.position + R * rb;             // current mount, world
                const Vec3 v_mount = R * (v_body + omega.cross(rb));  // its world velocity
                const Vec3 u_hat   = ez_world;                        // strut axis (body up)
                const Vec3 x_u = s.unsprung_pos[i];
                const Vec3 v_u = s.unsprung_vel[i];
                const double m_u = std::max(1.0, vp_.unsprung_mass[i]);
                const Vec3 rel  = x_u - mount;
                const Vec3 vrel = v_u - v_mount;
                const double comp     = rel.dot(u_hat);   // strut compression (+ toward body)
                const double comp_dot = vrel.dot(u_hat);
                const double ks  = std::max(1.0, vp_.spring_stiffness[i]);
                const double cs  = std::max(0.0, vp_.damper_coefficient[i]);
                // Strut force along +u_hat (preload so comp=0 is the static ride; no tension)
                // + damper + bump/rebound stops as stiff FORCE elements.
                double F_spring = F_preload_[i] + ks * comp;
                if (F_spring < 0.0) F_spring = 0.0;          // coilover cannot pull
                double F_stop = 0.0;
                if (comp > comp_max_[i])
                    F_stop = kStopStiffness * (comp - comp_max_[i]);
                else if (comp < comp_min_[i])
                    F_stop = kStopStiffness * (comp - comp_min_[i]);
                const double F_susp = F_spring + cs * comp_dot + F_stop;
                // Perpendicular bushing (rigid-link penalty), critically damped.
                const Vec3 rel_perp  = rel  - comp     * u_hat;
                const Vec3 vrel_perp = vrel - comp_dot * u_hat;
                const double cb = 2.0 * std::sqrt(k_link_ * m_u);
                const Vec3 F_bush = -k_link_ * rel_perp - cb * vrel_perp;  // on wheel

                Vec3 tire_total = Vec3::Zero();
                if (Fz[i] > 1.0) {
                    Vec3 n = contacts[i].normal.normalized();
                    Vec3 t_lat;
                    // DAE toe adds to the steer angle (bump-steer); camber flows via gamma.
                    Vec3 t_long = wheel_tangent_frame(n, body_fwd,
                                                      d_wheel[i] + toe_dae_[i], t_lat);
                    const double v_long_k = v_u.dot(t_long);
                    const double v_lat    = v_u.dot(t_lat);
                    ITireModel::ContactInput ci;
                    ci.Fz = Fz[i]; ci.Vx = v_long_k; ci.Vy = v_lat;
                    ci.omega = s.wheel_spin[i]; ci.gamma = gamma_dae_[i];
                    ci.mu_long = contacts[i].mu_long; ci.mu_lat = contacts[i].mu_lat;
                    ci.R0 = Rwh;
                    ci_[i] = ci;
                    const ITireModel::Wrench w = tire_->evaluate(ci, transient_[i]);
                    Re_w_[i] = w.Re; kappa[i] = w.kappa; alpha[i] = w.alpha;

                    const double muFz = std::min(contacts[i].mu_long, contacts[i].mu_lat)
                                        * std::max(0.0, Fz[i]);
                    double Fx_w = 0.0, Fy_w = 0.0;
                    if (lugre_on) {
                        Fx_w = w.Fx; Fy_w = w.Fy; mz_wheel[i] = w.Mz;
                    } else {
                        const double hold_gate = (1.0 - lambda) *
                            std::clamp(cmd.brake - cmd.throttle, 0.0, 1.0);
                        double Fx_hold = -kStickC * (R * v_body).dot(t_long) * hold_gate;
                        if (std::abs(Fx_hold) > muFz) Fx_hold = std::copysign(muFz, Fx_hold);
                        Fx_w = w.Fx + Fx_hold;
                        Fy_w = lambda * w.Fy;
                        fx_kin[i] = w.Fx;
                        mz_wheel[i] = w.Mz * lambda;
                    }
                    const double Fmag = std::hypot(Fx_w, Fy_w);
                    if (!tp_.model_provides_combined_slip() && Fmag > muFz && Fmag > 1e-9) {
                        const double c = muFz / Fmag; Fx_w *= c; Fy_w *= c;
                    }
                    tire_total = Fx_w * t_long + Fy_w * t_lat + Fz[i] * n;
                } else {
                    ci_[i] = ITireModel::ContactInput{};   // neutral: no transient evolution
                }
                // Unsprung Newton (world): tire + strut (-F_susp along u_hat) + bushing +
                // gravity. No frame-coupling shortcut — x_u is a genuine inertial particle.
                const Vec3 F_strut_on_u = -F_susp * u_hat;
                const Vec3 a_u = (tire_total + F_strut_on_u + F_bush
                                  + Vec3(0.0, 0.0, -m_u * kGravity)) / m_u;
                d_upos[i] = v_u;
                d_uvel[i] = a_u;
                // Body feels the equal-opposite connection reaction at the mount.
                const Vec3 F_conn = -(F_strut_on_u + F_bush);   // = F_susp*u_hat - F_bush
                F_total_world += F_conn;
                F_body[i] = R.transpose() * F_conn;
                tau_body += rb.cross(F_body[i]);
                tau_body.z() += mz_wheel[i];
                continue;
            }
            if (!contacts[i].is_valid || contacts[i].penetration <= 0.0) {
                F_body[i] = Vec3::Zero();
                ci_[i] = ITireModel::ContactInput{};   // neutral: no transient evolution
                continue;
            }
            const Vec3& rb = r_body_[i];
            const Vec3 v_hub_world = R * (v_body + omega.cross(rb));
            Vec3 n = contacts[i].normal.normalized();
            Vec3 t_lat;
            Vec3 t_long = wheel_tangent_frame(n, body_fwd, d_wheel[i], t_lat);
            // General contact-frame slip on any surface (flat / banked / loop): the
            // wheel-heading projection onto the contact tangent plane is the longitudinal
            // direction everywhere, no loop-specific tangent fallback.
            const double v_long_k = v_hub_world.dot(t_long);
            const double v_lat = v_hub_world.dot(t_lat);

            // Inverted tire: kinematics in (contact-frame), wrench out. The tire owns slip /
            // Re / transient; this model keeps only the integrator stabilization below.
            ITireModel::ContactInput ci;
            ci.Fz = Fz[i]; ci.Vx = v_long_k; ci.Vy = v_lat;
            ci.omega = s.wheel_spin[i]; ci.gamma = 0.0;
            ci.mu_long = contacts[i].mu_long; ci.mu_lat = contacts[i].mu_lat; ci.R0 = Rwh;
            ci_[i] = ci;
            const ITireModel::Wrench w = tire_->evaluate(ci, transient_[i]);
            Re_w_[i] = w.Re;
            kappa[i] = w.kappa;
            alpha[i] = w.alpha;

            if (Fz[i] < 1.0) {
                F_body[i] = Vec3::Zero();
                continue;
            }

            const double muFz = std::min(contacts[i].mu_long, contacts[i].mu_lat)
                                * std::max(0.0, Fz[i]);
            double Fx_w = 0.0, Fy_w = 0.0;
            if (lugre_on) {
                Fx_w = w.Fx;
                Fy_w = w.Fy;
                mz_wheel[i] = w.Mz;
            } else {
                const double hold_gate = (1.0 - lambda) *
                    std::clamp(cmd.brake - cmd.throttle, 0.0, 1.0);
                double Fx_hold = -kStickC * (R * v_body).dot(t_long) * hold_gate;
                if (std::abs(Fx_hold) > muFz) Fx_hold = std::copysign(muFz, Fx_hold);
                Fx_w = w.Fx + Fx_hold;
                Fy_w = lambda * w.Fy;
                fx_kin[i] = w.Fx;
                mz_wheel[i] = w.Mz * lambda;
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

        // Free-3D strut path: the body is purely the sprung mass (m_body = mass - sum
        // unsprung); each unsprung is its own inertial particle, so there is no mass split
        // and no frame-coupling shortcut — the body feels only the mount connection forces.
        // The whole vehicle (body + 4 unsprung) sums to the total mass, so flat-ground
        // handling still matches the planar L2/L3 models. Penalty path: m_body == mass.
        const Vec3 a_world = F_total_world / m_body;
        const Vec3 a_body = R.transpose() * a_world - omega.cross(v_body);
        // Reported ax/ay are the body-frame specific force (gravity removed), matching
        // the planar L2/L3 convention (Fy/m) — the accelerometer/load-transfer signal,
        // not the body-velocity derivative (which is ~0 in a steady turn).
        const Vec3 accel_sf_body =
            R.transpose() * (a_world + Vec3(0.0, 0.0, kGravity));
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
        d_out.ax_body  = accel_sf_body.x();
        d_out.ay_body  = accel_sf_body.y();
        d_out.d_u_pos = d_upos;
        d_out.d_u_vel = d_uvel;

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
                ? fx_kin[i] * Re_w_[i]
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

    State apply(const State& s0, const Deriv& d, double h) const {
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
        // Wheel-centre world position: ALWAYS maintained (the contact provider samples the
        // surface there). Strut path: integrate the free-3D inertial particle and DERIVE the
        // strut-axis travel (susp_compression/velocity) for the L4 DAE / FMI / cosim contract
        // ("suspension travel is the real vertical wheel travel"). Penalty path: the wheel
        // rides rigidly with the body. Either way unsprung_pos is the true wheel centre, so
        // it never goes stale and needs no path-dependent guard elsewhere.
        const Mat3 Rn = s.orientation.toRotationMatrix();
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const Vec3 mount   = s.position + Rn * r_body_[i];
            const Vec3 v_mount = Rn * (s.velocity + s.angular_velocity.cross(r_body_[i]));
            if (sp_.l5_spatial_suspension) {
                s.unsprung_pos[i] = s0.unsprung_pos[i] + d.d_u_pos[i] * h;
                s.unsprung_vel[i] = s0.unsprung_vel[i] + d.d_u_vel[i] * h;
                const Vec3 u_hat = Rn.col(2);
                s.susp_compression[i] = (s.unsprung_pos[i] - mount).dot(u_hat);
                s.susp_velocity[i]    = (s.unsprung_vel[i] - v_mount).dot(u_hat);
            } else {
                s.unsprung_pos[i] = mount;       // rigid wheel centre rides with the body
                s.unsprung_vel[i] = v_mount;
            }
        }
        return s;
    }

    // B2: advance each attached corner DAE with the prescribed strut travel (comp + rate)
    // and the last corner tire load, reading back per-wheel toe/camber (L/R sign flip).
    void update_corner_dae(const CmdL4& cmd, double dt) {
        if (!sp_.l5_spatial_suspension) return;
        if (!(mb_enabled_front_ || mb_enabled_rear_)) return;
        const DriverCmd dc{cmd.steer_angle_wheel, cmd.throttle, cmd.brake,
                           cmd.gear, cmd.handbrake};
        SubsystemContext ctx{state_, dc, 0.0};
        const double steer_rad = steering_->apply(ctx).roadwheel_angle;
        const Mat3 R = state_.orientation.toRotationMatrix();
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const bool front = (i == WHEEL_FL || i == WHEEL_FR);
            mb::IHardJointDaeModel* dae = front ? mb_dae_front_.get() : mb_dae_rear_.get();
            if (!dae) continue;
            mb::PrescribedCornerMotion mot;
            mot.travel_z      = state_.susp_compression[i];
            mot.travel_z_dot  = state_.susp_velocity[i];
            mot.steer_rack_dy = front ? steer_rad * kRackPerRad : 0.0;
            mb::WheelLoad wl;
            wl.force_world = R * tire_F_[i];
            const mb::WheelPose wp = dae->step(mb_state_[i], mot, wl, dt);
            const double s = (i == WHEEL_FR || i == WHEEL_RR) ? -1.0 : 1.0;
            gamma_dae_[i] = s * wp.camber_rad;
            toe_dae_[i]   = s * wp.toe_rad;
        }
    }

    void substep(const CmdL4& cmd, const ContactArray& contacts_in, double h) {
        const State s0 = state_;
        // Per-substep contact re-query (free-3D only): if a provider is attached, refresh
        // the contact (normal + penetration + the frozen rigid-hub reference) at this
        // substep's pose instead of holding the once-per-outer-step query. Shrinks the
        // frozen-contact O(dt) energy term on curved surfaces (loop / bank). Backward
        // compatible: null provider -> use the ContactArray the caller passed in.
        ContactArray local;
        const ContactArray* cptr = &contacts_in;
        if (provider_ && sp_.l5_spatial_suspension) {
            provider_->query(s0, vp_, local);
            cptr = &local;
            for (int i = 0; i < NUM_WHEELS; ++i)
                mount_query_[i] = s0.unsprung_pos[i];   // wheel particle the query sampled at
        }
        const ContactArray& contacts = *cptr;
        const bool lugre_on = tp_.lugre.enabled;
        const double hz = 0.25 * h;
        if (sp_.integrator == SolverParams::Integrator::Euler) {
            const Deriv k = derivatives(s0, cmd, contacts);
            state_ = apply(s0, k, h);
            ax_prev_ = k.ax_body;
            ay_prev_ = k.ay_body;
            if (lugre_on) advance_bristle_(h);
            return;
        }
        const Deriv k1 = derivatives(s0, cmd, contacts);
        if (lugre_on) advance_bristle_(hz);
        const Deriv k2 = derivatives(apply(s0, k1, 0.5 * h), cmd, contacts);
        if (lugre_on) advance_bristle_(hz);
        const Deriv k3 = derivatives(apply(s0, k2, 0.5 * h), cmd, contacts);
        if (lugre_on) advance_bristle_(hz);
        const Deriv k4 = derivatives(apply(s0, k3, h), cmd, contacts);
        if (lugre_on) advance_bristle_(hz);
        Deriv k;
        k.dp_world = (k1.dp_world + 2*k2.dp_world + 2*k3.dp_world + k4.dp_world) / 6.0;
        k.d_v_body = (k1.d_v_body + 2*k2.d_v_body + 2*k3.d_v_body + k4.d_v_body) / 6.0;
        k.d_omega  = (k1.d_omega  + 2*k2.d_omega  + 2*k3.d_omega  + k4.d_omega) / 6.0;
        k.ax_body  = (k1.ax_body  + 2*k2.ax_body  + 2*k3.ax_body  + k4.ax_body) / 6.0;
        k.ay_body  = (k1.ay_body  + 2*k2.ay_body  + 2*k3.ay_body  + k4.ay_body) / 6.0;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            k.domega[i] = (k1.domega[i] + 2*k2.domega[i] + 2*k3.domega[i] + k4.domega[i]) / 6.0;
            k.d_u_pos[i] = (k1.d_u_pos[i] + 2*k2.d_u_pos[i]
                          + 2*k3.d_u_pos[i] + k4.d_u_pos[i]) / 6.0;
            k.d_u_vel[i] = (k1.d_u_vel[i] + 2*k2.d_u_vel[i]
                          + 2*k3.d_u_vel[i] + k4.d_u_vel[i]) / 6.0;
        }
        state_ = apply(s0, k, h);
        ax_prev_ = k.ax_body;
        ay_prev_ = k.ay_body;
        advance_relaxation_(h);
    }

    // LuGre bristle z — per RK4 stage (tire-owned).
    void advance_bristle_(double dt) {
        if (!tp_.lugre.enabled) return;
        for (int i = 0; i < NUM_WHEELS; ++i)
            transient_[i] = tire_->advance_bristle(ci_[i], transient_[i], dt);
    }
    // Carcass/belt + relaxation-length lag — once per substep (tire-owned).
    void advance_relaxation_(double dt) {
        for (int i = 0; i < NUM_WHEELS; ++i)
            transient_[i] = tire_->advance_relaxation(ci_[i], transient_[i], dt);
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
    // Spatial strut (l5_spatial_suspension): per-corner sprung static load (preload so
    // comp=0 is the static ride position) and travel limits (bump / droop stops).
    std::array<double, NUM_WHEELS> F_preload_ {};
    std::array<double, NUM_WHEELS> comp_min_ {};   // droop limit [m] (spring topped out)
    std::array<double, NUM_WHEELS> comp_max_ {};   // bump limit [m]
    // Free-3D: rigid-hub world position the step-start contact query assumed (frozen
    // through the substeps so the tire penetration tracks the wheel particle vs the road).
    std::array<Vec3, NUM_WHEELS> mount_query_ {{Vec3::Zero(), Vec3::Zero(),
                                                Vec3::Zero(), Vec3::Zero()}};
    // Perpendicular link bushing rate (env-overridable for the hard-constraint sweep).
    double k_link_ {kLinkStiffness};
    // Optional contact provider for per-substep re-query (shrinks the once-per-step
    // frozen-contact O(dt) term). Non-owning; null -> use the frozen ContactArray passed in.
    IContactProvider* provider_ {nullptr};
    // Per-corner L4 hard-joint corner DAE (B2): one model per axle (front/rear) shared by
    // its two corners, each with its own state, fed the corner strut travel. Produces the
    // per-wheel toe/camber applied to the tire. Inactive unless attached AND strut path on.
    mb::SuspensionTopology mb_topo_front_ {}, mb_topo_rear_ {};
    std::unique_ptr<mb::IHardJointDaeModel> mb_dae_front_, mb_dae_rear_;
    std::array<mb::HardJointCornerState, NUM_WHEELS> mb_state_ {};
    bool mb_enabled_front_ {false}, mb_enabled_rear_ {false};
    std::array<double, NUM_WHEELS> gamma_dae_ {};   // per-wheel camber [rad] from the DAE
    std::array<double, NUM_WHEELS> toe_dae_ {};     // per-wheel toe [rad] from the DAE
    State state_;
    double ax_prev_ {0.0};
    double ay_prev_ {0.0};
    std::array<Vec3, NUM_WHEELS> tire_F_ {};
    std::array<double, NUM_WHEELS> tire_Fz_ {};
    std::array<double, NUM_WHEELS> slip_ratio_ {};
    std::array<double, NUM_WHEELS> slip_angle_ {};
    // Per-wheel tire transient + the contact kinematics the tire was last evaluated at,
    // feeding the per-stage (advance_bristle) / per-substep (advance_relaxation) calls.
    std::array<ITireModel::Transient, NUM_WHEELS> transient_ {};
    std::array<ITireModel::ContactInput, NUM_WHEELS> ci_ {};
    std::array<double, NUM_WHEELS> Re_w_ {{0.32, 0.32, 0.32, 0.32}};  // effective rolling radius
    double mz_front_sum_ {0.0};
};

}  // namespace

std::unique_ptr<IVehicleDynamics> create_stunt_dof() {
    return std::make_unique<Free3DDynamics>();
}

bool free_3d_attach_multibody(IVehicleDynamics& dyn,
                              bool front_axle,
                              const mb::SuspensionTopology& topo,
                              bool enable_dynamics) {
    auto* p = dynamic_cast<Free3DDynamics*>(&dyn);
    if (!p) return false;
    p->attach_mb_corner(front_axle, topo, enable_dynamics);
    return true;
}

bool free_3d_mb_dynamics_enabled(const IVehicleDynamics& dyn, int axle) {
    const auto* p = dynamic_cast<const Free3DDynamics*>(&dyn);
    if (!p) return false;
    return p->mb_enabled(axle);
}

bool free_3d_attach_contact_provider(IVehicleDynamics& dyn, IContactProvider* provider) {
    auto* p = dynamic_cast<Free3DDynamics*>(&dyn);
    if (!p) return false;
    p->set_contact_provider(provider);
    return true;
}

std::array<double, NUM_WHEELS> free_3d_wheel_camber(const IVehicleDynamics& dyn) {
    const auto* p = dynamic_cast<const Free3DDynamics*>(&dyn);
    return p ? p->wheel_camber() : std::array<double, NUM_WHEELS>{};
}

std::array<double, NUM_WHEELS> free_3d_wheel_toe(const IVehicleDynamics& dyn) {
    const auto* p = dynamic_cast<const Free3DDynamics*>(&dyn);
    return p ? p->wheel_toe() : std::array<double, NUM_WHEELS>{};
}

}  // namespace vdsim
