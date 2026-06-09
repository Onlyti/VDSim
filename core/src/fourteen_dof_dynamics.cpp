// L3 14-DOF dynamics — full suspension implementation.
//
// Planar motion (x, y, yaw, 4 wheel spin = 7 DOF) is delegated to L2.  On top
// of L2 we integrate **seven additional vertical DOFs**:
//   - 3 sprung-body modes: heave (z_s), roll (phi), pitch (theta)
//   - 4 unsprung-mass modes: z_u_i  (per corner i)
// Combined first-order state vector = 14 (with each second-order DOF split
// into position + velocity), matching D8's nominal 14-DOF count.
//
// Per-corner spring + damper acts between sprung corner and unsprung mass.
// Tire vertical spring acts between unsprung mass and ground (z=0).
//
// Equations of motion (about sprung CG, body frame, small-angle linearized):
//
//   per corner i (positions (a, Tw/2) etc. in body x,y):
//      δ_corner_i   = z_s + ry_i · phi  − rx_i · theta
//      v_corner_i   = ż_s + ry_i · ϕ̇   − rx_i · θ̇
//      F_spring_i   = − k_i · δ_corner_i              (restoring)
//      F_damper_i   = − c_i · v_corner_i
//      F_total_i    = F_spring_i + F_damper_i
//
//   m_s · z̈_s     = Σ F_total_i  + m_s · g_z_effective
//   I_xx · ϕ̈     = Σ ry_i · F_total_i + m_s · ay · h_cg     (lateral inertia)
//   I_yy · θ̈     = − Σ rx_i · F_total_i − m_s · ax · h_cg   (long. inertia)
//
// where g_z_effective ≈ 0 (we work in z relative to static eq).
// Sign convention (ISO 8855): +phi = lean (left-up under left turn), +theta =
// nose-down. Braking (ax<0) -> +theta -> front compresses (dive). The pose
// quaternion uses these directly; susp_compression is reported as a magnitude
// (larger = more compressed).
//
// Steady state of small linear system:
//   z_s    → 0
//   phi    →   m_s · ay · h_cg / K_phi_total
//   theta  → − m_s · ax · h_cg / K_pitch_total
//
// matches the Task 22 quasi-static formulas.  Damper gives transient ringing.

#include "vdsim/control.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/default_subsystems.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/subsystems.hpp"
#include "vdsim/suspension.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

namespace vdsim {

namespace {

constexpr double kGravity = 9.80665;
constexpr double kRackPerRad = 0.08;

double steer_wheel_rad(const ControlInput& u) noexcept {
    return std::visit([](const auto& cmd) -> double {
        using T = std::decay_t<decltype(cmd)>;
        if constexpr (std::is_same_v<T, CmdL1> || std::is_same_v<T, CmdL2>
                   || std::is_same_v<T, CmdL3> || std::is_same_v<T, CmdL4>
                   || std::is_same_v<T, CmdL5> || std::is_same_v<T, CmdL6>)
            return cmd.steer_angle_wheel;
        return 0.0;
    }, u);
}

class FourteenDOFDynamics : public IVehicleDynamics {
public:
    FourteenDOFDynamics() : inner_(create_seven_dof()) {}
    explicit FourteenDOFDynamics(std::unique_ptr<ITireModel> tire)
        : inner_(create_seven_dof(std::move(tire))) {}

    Level level() const noexcept override { return Level::L3_FourteenDOF; }

    void initialize(const VehicleParams& vp,
                    const TireParams& tp,
                    const SolverParams& sp) override {
        vp_ = vp; tp_ = tp; sp_ = sp;
        inner_->initialize(vp, tp, sp);
        suspension_ = make_default_suspension(vp);
        arb_front_  = make_default_antirollbar(vp, 0);
        arb_rear_   = make_default_antirollbar(vp, 1);

        const double a = vp.cg_to_front, b = vp.cg_to_rear;
        const double tw_f = vp.track_front * 0.5, tw_r = vp.track_rear * 0.5;
        rx_ = {{ +a, +a, -b, -b }};
        ry_ = {{ +tw_f, -tw_f, +tw_r, -tw_r }};

        const double Fz_static_f = vp.mass_sprung * kGravity * b / (2.0 * vp.wheelbase);
        const double Fz_static_r = vp.mass_sprung * kGravity * a / (2.0 * vp.wheelbase);
        Fz_static_[WHEEL_FL] = Fz_static_[WHEEL_FR] = Fz_static_f;
        Fz_static_[WHEEL_RL] = Fz_static_[WHEEL_RR] = Fz_static_r;

        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double k = std::max(1.0, vp.spring_stiffness[i]);
            static_compression_[i] = Fz_static_[i] / k;
        }
        spdlog::debug("[L3 14-DOF] init: mass_sprung={:.0f} kg, K_phi_spring={:.0f} N m/rad, "
                      "K_arb={:.0f} N m/rad, k_tire={:.0f} N/m, anti_dive={:.2f}",
                      vp.mass_sprung,
                      axle_roll_stiffness(vp, 0) + axle_roll_stiffness(vp, 1)
                          - vp.arb_stiffness_front - vp.arb_stiffness_rear,
                      vp.arb_stiffness_front + vp.arb_stiffness_rear,
                      tp.tire_vertical_stiffness, vp.anti_dive_front);
    }

    void reset(const State& s) noexcept override {
        state_ = s;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            state_.susp_compression[i] = static_compression_[i];
            state_.susp_velocity[i]    = 0.0;
        }
        z_s_ = 0.0; z_s_dot_ = 0.0;
        inner_->reset(state_);
        phi_ = 0.0; phi_dot_ = 0.0;
        th_  = 0.0; th_dot_  = 0.0;
        z_u_     = {{0.0, 0.0, 0.0, 0.0}};
        z_u_dot_ = {{0.0, 0.0, 0.0, 0.0}};
        if (!suspension_) suspension_ = make_default_suspension(vp_);
        if (!arb_front_)  arb_front_  = make_default_antirollbar(vp_, 0);
        if (!arb_rear_)   arb_rear_   = make_default_antirollbar(vp_, 1);
    }

    void step(const ControlInput& u,
              const ContactArray& contacts,
              double dt) noexcept override {
        // Per-wheel camber AND toe input to inner Ld2.  Two sources:
        //   1. Hardpoint kinematics (front and/or rear) if attached — uses
        //      per-wheel travel from sprung-corner z minus unsprung world z.
        //   2. Fallback: phenomenological vp_.camber_per_roll · phi  (legacy);
        //      no toe contribution in fallback (matches pre-Ld4 behavior).
        std::array<double, NUM_WHEELS> gamma {{0.0, 0.0, 0.0, 0.0}};
        std::array<double, NUM_WHEELS> toe   {{0.0, 0.0, 0.0, 0.0}};
        const double steer_rad = steer_wheel_rad(u);
        if (mb_dyn_front_ && mb_dae_front_) {
            apply_hard_joint_axle_pose(0, steer_rad, gamma, toe);
        }
        if (mb_dyn_rear_ && mb_dae_rear_) {
            apply_hard_joint_axle_pose(2, steer_rad, gamma, toe);
        }
        if (kine_front_ || kine_rear_) {
            for (int i = 0; i < NUM_WHEELS; ++i) {
                if ((i < 2 && mb_dyn_front_ && mb_dae_front_)
                    || (i >= 2 && mb_dyn_rear_ && mb_dae_rear_))
                    continue;
                const double z_corner_s = z_s_ + ry_[i] * std::sin(phi_)
                                                - rx_[i] * std::sin(th_);
                const double wheel_travel = z_u_[i] - z_corner_s;
                ISuspensionKinematics* k = (i < 2) ? kine_front_.get()
                                                   : kine_rear_.get();
                if (k) {
                    const double rack_dy = (i < 2) ? steer_rad * kRackPerRad : 0.0;
                    const auto o = k->compute(wheel_travel, rack_dy);
                    const bool right = (i == WHEEL_FR || i == WHEEL_RR);
                    const double s = right ? -1.0 : 1.0;
                    gamma[i] = s * o.camber;
                    toe[i]   = s * o.toe;
                } else {
                    gamma[i] = (i == WHEEL_FR || i == WHEEL_RR)
                                ? -vp_.camber_per_roll * phi_
                                : +vp_.camber_per_roll * phi_;
                }
            }
        } else if (!mb_dyn_front_ && !mb_dyn_rear_) {
            const double k_cam = vp_.camber_per_roll;
            gamma = {{ +k_cam * phi_, -k_cam * phi_,
                       +k_cam * phi_, -k_cam * phi_ }};
        }
        inner_->set_camber_per_wheel(gamma);
        inner_->set_toe_per_wheel(toe);

        {
            constexpr double kAirDensity = 1.225;
            const double k_tire = std::max(1.0, tp_.tire_vertical_stiffness);
            const double Lwb  = vp_.wheelbase;
            const double vx   = state_.velocity.x();
            const double q_aero = 0.5 * kAirDensity * vp_.frontal_area * vx * std::abs(vx);
            const double aero_f = 0.5 * vp_.aero_lift_front * q_aero;
            const double aero_r = 0.5 * vp_.aero_lift_rear  * q_aero;
            const double st_f = vp_.mass * kGravity * vp_.cg_to_rear  / (2.0 * Lwb);
            const double st_r = vp_.mass * kGravity * vp_.cg_to_front / (2.0 * Lwb);
            std::array<double, NUM_WHEELS> fz_dyn;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                const double cos_slope = std::max(0.1, contacts[i].normal.z());
                const double st = ((i < 2) ? st_f : st_r) * cos_slope
                                + ((i < 2) ? aero_f : aero_r);
                fz_dyn[i] = std::max(0.0, st + k_tire * (contacts[i].road_dz - z_u_[i]));
            }
            inner_->set_external_fz(fz_dyn);
        }

        inner_->step(u, contacts, dt);
        state_ = inner_->state();
        if (dt > 0.0 && (mb_dyn_front_ || mb_dyn_rear_)) {
            const auto F = inner_->tire_forces_body();
            if (mb_dyn_front_ && mb_dae_front_) {
                mb::WheelLoad wl;
                wl.force_world = 0.5 * (F[WHEEL_FL] + F[WHEEL_FR]);
                mb::step_hard_joint_dae(mb_state_front_, *mb_dae_front_, mb_topo_front_,
                                        axle_prescribed_motion(0, steer_rad), wl, dt);
            }
            if (mb_dyn_rear_ && mb_dae_rear_) {
                mb::WheelLoad wl;
                wl.force_world = 0.5 * (F[WHEEL_RL] + F[WHEEL_RR]);
                mb::step_hard_joint_dae(mb_state_rear_, *mb_dae_rear_, mb_topo_rear_,
                                        axle_prescribed_motion(2, steer_rad), wl, dt);
            }
        }
        for (int i = 0; i < NUM_WHEELS; ++i) road_dz_[i] = contacts[i].road_dz;
        if (dt > 0.0) integrate_vertical(dt);
        write_pose_and_suspension();
    }

    // Attach an ISuspensionKinematics for the front (L) or rear (R) axle.
    // nullptr restores the legacy camber_per_roll fallback.
    void set_kinematics_front(std::unique_ptr<ISuspensionKinematics> k) {
        kine_front_ = std::move(k);
    }
    void set_kinematics_rear(std::unique_ptr<ISuspensionKinematics> k) {
        kine_rear_ = std::move(k);
    }

    void attach_mb_corner(bool front, mb::SuspensionTopology topo, bool enable) {
        if (!mb_solver_) mb_solver_ = mb::create_kinematic_solver();
        mb::PrescribedCornerMotion mot {};
        if (front) {
            mb_topo_front_ = std::move(topo);
            mb_dyn_front_  = enable;
            mb_state_front_ = {};
            mb_dae_front_ = enable ? mb::create_hard_joint_dae_model(mb_topo_front_) : nullptr;
            if (mb_dae_front_) mb_dae_front_->initialize(mb_state_front_, mot);
        } else {
            mb_topo_rear_ = std::move(topo);
            mb_dyn_rear_  = enable;
            mb_state_rear_ = {};
            mb_dae_rear_ = enable ? mb::create_hard_joint_dae_model(mb_topo_rear_) : nullptr;
            if (mb_dae_rear_) mb_dae_rear_->initialize(mb_state_rear_, mot);
        }
    }

    double compliance_toe_rad(int /*axle*/) const noexcept override { return 0.0; }

    mb::PrescribedCornerMotion axle_prescribed_motion(int wheel_base,
                                                      double steer_rad) const {
        mb::PrescribedCornerMotion mot;
        double travel = 0.0;
        double travel_dot = 0.0;
        for (int j = 0; j < 2; ++j) {
            const int i = wheel_base + j;
            const double z_corner_s = z_s_ + ry_[i] * std::sin(phi_)
                                            - rx_[i] * std::sin(th_);
            const double v_corner_s = z_s_dot_ + ry_[i] * std::cos(phi_) * phi_dot_
                                    - rx_[i] * std::cos(th_) * th_dot_;
            travel += 0.5 * (z_u_[i] - z_corner_s);
            travel_dot += 0.5 * (z_u_dot_[i] - v_corner_s);
        }
        mot.travel_z = travel;
        mot.travel_z_dot = travel_dot;
        mot.steer_rack_dy = (wheel_base == 0) ? steer_rad * kRackPerRad : 0.0;
        return mot;
    }

    void apply_hard_joint_axle_pose(int wheel_base, double steer_rad,
                                    std::array<double, NUM_WHEELS>& gamma,
                                    std::array<double, NUM_WHEELS>& toe) {
        mb::IHardJointDaeModel* dae = (wheel_base == 0) ? mb_dae_front_.get()
                                                        : mb_dae_rear_.get();
        mb::HardJointCornerState* st = (wheel_base == 0) ? &mb_state_front_
                                                         : &mb_state_rear_;
        if (!dae || !st) return;
        mb::WheelLoad zl {};
        const auto mot = axle_prescribed_motion(wheel_base, steer_rad);
        const auto wp = dae->step(*st, mot, zl, 0.0);
        for (int j = 0; j < 2; ++j) {
            const int i = wheel_base + j;
            const bool right = (i == WHEEL_FR || i == WHEEL_RR);
            const double s = right ? -1.0 : 1.0;
            gamma[i] = s * wp.camber_rad;
            toe[i]   = s * wp.toe_rad;
        }
    }

    bool mb_dyn_front_enabled() const noexcept { return mb_dyn_front_; }
    bool mb_dyn_rear_enabled()  const noexcept { return mb_dyn_rear_;  }

    const State& state() const noexcept override { return state_; }

    std::array<Vec3,   NUM_WHEELS> tire_forces_body() const override
        { return inner_->tire_forces_body(); }
    std::array<double, NUM_WHEELS> tire_Fz() const override
        { return inner_->tire_Fz(); }
    std::array<double, NUM_WHEELS> wheel_slip_ratio() const override
        { return inner_->wheel_slip_ratio(); }
    std::array<double, NUM_WHEELS> wheel_slip_angle() const override
        { return inner_->wheel_slip_angle(); }

    double roll_angle_qs()  const override { return phi_; }
    double pitch_angle_qs() const override { return th_;  }
    double ax_body_est()    const override { return inner_->ax_body_est(); }
    double ay_body_est()    const override { return inner_->ay_body_est(); }

private:
    struct Deriv6 {
        double dz {0.0};
        double dz_dot {0.0};
        double dphi {0.0};
        double dphi_dot {0.0};
        double dth {0.0};
        double dth_dot {0.0};
        std::array<double, NUM_WHEELS> dz_u     {{0.0, 0.0, 0.0, 0.0}};
        std::array<double, NUM_WHEELS> dz_u_dot {{0.0, 0.0, 0.0, 0.0}};
    };

    Deriv6 derivatives_vertical(double z, double z_dot,
                                double phi, double phi_dot,
                                double th, double th_dot,
                                const std::array<double, NUM_WHEELS>& zu,
                                const std::array<double, NUM_WHEELS>& zu_dot,
                                double ax, double ay) const {
        const double m_s = vp_.mass_sprung;
        const double h   = vp_.cg_height;
        const double Ixx = vp_.inertia_diag.x();
        const double Iyy = vp_.inertia_diag.y();

        // Heave + pitch via per-corner spring/damper between sprung corner
        // and unsprung mass.
        // Corner deflection couples heave + pitch + roll (delta = z + ry*phi - rx*th),
        // so the per-corner springs/dampers produce roll stiffness and roll damping
        // naturally -- no separate lumped K_phi/C_phi. The ARB adds a roll-only term.
        std::array<double, NUM_WHEELS> F_susp{};
        std::array<double, NUM_WHEELS> delta{}, vel{};
        // Per-corner suspension force from the pluggable ISuspension module (default
        // LinearSuspension == -k*delta - c*vel). ctx is threaded so a custom module
        // can read vehicle state (default ignores it).
        const SubsystemContext susp_ctx{state_, susp_ctx_cmd_, 0.0};
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double z_corner = z + ry_[i] * phi - rx_[i] * th;
            const double v_corner = z_dot + ry_[i] * phi_dot - rx_[i] * th_dot;
            delta[i] = z_corner - zu[i];        // relative compression deviation
            vel[i]   = v_corner - zu_dot[i];
            F_susp[i] = suspension_->force(susp_ctx, CornerInput{i, delta[i], vel[i], 1.0});
        }
        // Anti-roll bar as a per-wheel force pair (per axle), added to the corner
        // forces so it feeds heave/pitch/roll AND the unsprung dynamics — the
        // physical ARB. For pure roll (delta_L-delta_R = track*phi) this reproduces
        // the previous lumped -K_arb*phi roll moment; it additionally reacts to
        // asymmetric (one-wheel) inputs, which the lumped form could not.
        {
            const auto Ff = arb_front_->force(susp_ctx,
                AxleDefl{delta[WHEEL_FL], delta[WHEEL_FR], vel[WHEEL_FL], vel[WHEEL_FR]});
            const auto Fr = arb_rear_->force(susp_ctx,
                AxleDefl{delta[WHEEL_RL], delta[WHEEL_RR], vel[WHEEL_RL], vel[WHEEL_RR]});
            F_susp[WHEEL_FL] += Ff.first;  F_susp[WHEEL_FR] += Ff.second;
            F_susp[WHEEL_RL] += Fr.first;  F_susp[WHEEL_RR] += Fr.second;
        }
        double Fz_sum = 0.0, M_pitch_spring = 0.0, M_roll_spring = 0.0;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            Fz_sum         += F_susp[i];
            M_pitch_spring -= rx_[i] * F_susp[i];
            M_roll_spring  += ry_[i] * F_susp[i];   // includes the ARB roll contribution
        }

        // Anti-dive / anti-squat reduces the longitudinal inertia moment fed
        // into pitch.  For braking (ax<0) the front anti_dive_front fraction
        // is bypassed; for accel (ax>0) the rear anti_squat_rear fraction is
        // bypassed.  Effective scaling on inertial pitch moment:
        // Not clamped to [0,1]: >1 (over-100% anti, lifts under braking) or <0
        // (pro-dive) are valid suspension-geometry design choices.
        const double anti = (ax < 0.0) ? vp_.anti_dive_front : vp_.anti_squat_rear;
        // ISO sign: +theta = nose-down. Braking (ax<0) must dive (front compress),
        // i.e. theta>0, so the inertial pitch moment is -m_s*ax*h.
        const double M_inertia_pitch = -m_s * ax * h * (1.0 - anti);

        Deriv6 d;
        d.dz       = z_dot;
        d.dz_dot   = Fz_sum / std::max(1.0, m_s);
        d.dphi     = phi_dot;
        d.dphi_dot = (M_roll_spring + m_s * ay * h) / std::max(1e-3, Ixx);
        d.dth      = th_dot;
        d.dth_dot  = (M_pitch_spring + M_inertia_pitch) / std::max(1e-3, Iyy);

        // Unsprung mass per corner: m_u · z̈_u = -F_susp(on sprung) - k_tire · z_u
        // = +F_susp_on_unsprung - k_tire · z_u
        // F_susp_on_unsprung = -F_susp(on sprung) (Newton III)
        const double k_tire = std::max(1.0, tp_.tire_vertical_stiffness);
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double m_u = std::max(1.0, vp_.unsprung_mass[i]);
            d.dz_u[i] = zu_dot[i];
            d.dz_u_dot[i] = (-F_susp[i] - k_tire * (zu[i] - road_dz_[i])) / m_u;
        }
        return d;
    }

    void integrate_vertical(double dt) noexcept {
        const int N = std::max(1,
                       std::min(sp_.max_substeps,
                                static_cast<int>(std::ceil(dt / sp_.max_substep_dt))));
        const double h = dt / static_cast<double>(N);
        const double ax = inner_->ax_body_est();
        const double ay = inner_->ay_body_est();

        auto shift_zu = [](const std::array<double, NUM_WHEELS>& zu,
                            const std::array<double, NUM_WHEELS>& d,
                            double s) {
            std::array<double, NUM_WHEELS> r{};
            for (int i = 0; i < NUM_WHEELS; ++i) r[i] = zu[i] + s * d[i];
            return r;
        };

        for (int it = 0; it < N; ++it) {
            const auto k1 = derivatives_vertical(z_s_, z_s_dot_, phi_, phi_dot_, th_, th_dot_,
                                                  z_u_, z_u_dot_, ax, ay);
            const auto k2 = derivatives_vertical(z_s_ + 0.5*h*k1.dz,
                                                  z_s_dot_ + 0.5*h*k1.dz_dot,
                                                  phi_ + 0.5*h*k1.dphi,
                                                  phi_dot_ + 0.5*h*k1.dphi_dot,
                                                  th_  + 0.5*h*k1.dth,
                                                  th_dot_  + 0.5*h*k1.dth_dot,
                                                  shift_zu(z_u_, k1.dz_u, 0.5*h),
                                                  shift_zu(z_u_dot_, k1.dz_u_dot, 0.5*h),
                                                  ax, ay);
            const auto k3 = derivatives_vertical(z_s_ + 0.5*h*k2.dz,
                                                  z_s_dot_ + 0.5*h*k2.dz_dot,
                                                  phi_ + 0.5*h*k2.dphi,
                                                  phi_dot_ + 0.5*h*k2.dphi_dot,
                                                  th_  + 0.5*h*k2.dth,
                                                  th_dot_  + 0.5*h*k2.dth_dot,
                                                  shift_zu(z_u_, k2.dz_u, 0.5*h),
                                                  shift_zu(z_u_dot_, k2.dz_u_dot, 0.5*h),
                                                  ax, ay);
            const auto k4 = derivatives_vertical(z_s_ + h*k3.dz,
                                                  z_s_dot_ + h*k3.dz_dot,
                                                  phi_ + h*k3.dphi,
                                                  phi_dot_ + h*k3.dphi_dot,
                                                  th_  + h*k3.dth,
                                                  th_dot_  + h*k3.dth_dot,
                                                  shift_zu(z_u_, k3.dz_u, h),
                                                  shift_zu(z_u_dot_, k3.dz_u_dot, h),
                                                  ax, ay);

            z_s_     += h * (k1.dz       + 2*k2.dz       + 2*k3.dz       + k4.dz)       / 6.0;
            z_s_dot_ += h * (k1.dz_dot   + 2*k2.dz_dot   + 2*k3.dz_dot   + k4.dz_dot)   / 6.0;
            phi_     += h * (k1.dphi     + 2*k2.dphi     + 2*k3.dphi     + k4.dphi)     / 6.0;
            phi_dot_ += h * (k1.dphi_dot + 2*k2.dphi_dot + 2*k3.dphi_dot + k4.dphi_dot) / 6.0;
            th_      += h * (k1.dth      + 2*k2.dth      + 2*k3.dth      + k4.dth)      / 6.0;
            th_dot_  += h * (k1.dth_dot  + 2*k2.dth_dot  + 2*k3.dth_dot  + k4.dth_dot)  / 6.0;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                z_u_[i]     += h * (k1.dz_u[i]     + 2*k2.dz_u[i]     + 2*k3.dz_u[i]     + k4.dz_u[i])     / 6.0;
                z_u_dot_[i] += h * (k1.dz_u_dot[i] + 2*k2.dz_u_dot[i] + 2*k3.dz_u_dot[i] + k4.dz_u_dot[i]) / 6.0;
            }
        }
    }

    void write_pose_and_suspension() {
        const double yaw = yaw_from_quat(state_.orientation);
        state_.orientation = quat_from_euler({phi_, th_, yaw});
        // Roll/pitch rates live in the L3 vertical model; publish them into the
        // body angular velocity so the gyro/IMU sees them (z = yaw rate from inner).
        state_.angular_velocity.x() = phi_dot_;
        state_.angular_velocity.y() = th_dot_;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double z_corner = z_s_ + ry_[i] * phi_ - rx_[i] * th_;
            const double v_corner = z_s_dot_ + ry_[i] * phi_dot_ - rx_[i] * th_dot_;
            const double delta    = z_corner - z_u_[i];
            const double rel_vel  = v_corner - z_u_dot_[i];
            // Report compression as a magnitude (larger = more compressed). The
            // sprung corner moving toward the unsprung (delta<0) is more
            // compression, hence the minus. susp_velocity is its time rate.
            state_.susp_compression[i] = static_compression_[i] - delta;
            state_.susp_velocity[i]    = -rel_vel;
        }
    }

    VehicleParams vp_;
    TireParams    tp_;
    SolverParams  sp_;
    std::unique_ptr<IVehicleDynamics> inner_;
    std::unique_ptr<ISuspension> suspension_;
    std::unique_ptr<IAntiRollBar> arb_front_;
    std::unique_ptr<IAntiRollBar> arb_rear_;
    DriverCmd susp_ctx_cmd_ {};   // threaded into the suspension/ARB ctx (default modules ignore it)
    std::array<double, NUM_WHEELS> Fz_static_           {};
    std::array<double, NUM_WHEELS> static_compression_  {};
    std::array<double, NUM_WHEELS> rx_                  {};
    std::array<double, NUM_WHEELS> ry_                  {};

    State state_;
    double z_s_ {0.0}, z_s_dot_ {0.0};
    double phi_ {0.0}, phi_dot_ {0.0};
    double th_  {0.0}, th_dot_  {0.0};
    std::array<double, NUM_WHEELS> z_u_     {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, NUM_WHEELS> z_u_dot_ {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, NUM_WHEELS> road_dz_ {{0.0, 0.0, 0.0, 0.0}};

    // Optional hardpoint kinematics (Ld4 Stage D).  When attached, replaces
    // the lumped vp_.camber_per_roll · phi heuristic for the corresponding axle.
    std::unique_ptr<ISuspensionKinematics> kine_front_;
    std::unique_ptr<ISuspensionKinematics> kine_rear_;

    std::unique_ptr<mb::IMultibodySolver> mb_solver_;
    bool mb_dyn_front_ {false};
    bool mb_dyn_rear_  {false};
    mb::SuspensionTopology mb_topo_front_;
    mb::SuspensionTopology mb_topo_rear_;
    mb::HardJointCornerState mb_state_front_;
    mb::HardJointCornerState mb_state_rear_;
    std::unique_ptr<mb::IHardJointDaeModel> mb_dae_front_;
    std::unique_ptr<mb::IHardJointDaeModel> mb_dae_rear_;
};

class KinematicFourteenDOFDynamics : public FourteenDOFDynamics {
public:
    using FourteenDOFDynamics::FourteenDOFDynamics;
    Level level() const noexcept override { return Level::L4_Kinematic; }
};

}  // namespace

std::unique_ptr<IVehicleDynamics> create_fourteen_dof() {
    return std::make_unique<FourteenDOFDynamics>();
}

std::unique_ptr<IVehicleDynamics> create_fourteen_dof(std::unique_ptr<ITireModel> tire) {
    return std::make_unique<FourteenDOFDynamics>(std::move(tire));
}

std::unique_ptr<IVehicleDynamics> create_fourteen_dof_kinematic() {
    return std::make_unique<KinematicFourteenDOFDynamics>();
}

std::unique_ptr<IVehicleDynamics> create_fourteen_dof_kinematic(
    std::unique_ptr<ITireModel> tire) {
    return std::make_unique<KinematicFourteenDOFDynamics>(std::move(tire));
}

// Public attach helpers — callers can build the kinematics object (e.g. via
// create_lookup_kinematics) and hand it to the Ld3 instance.  Returns true on
// success (i.e. dyn really is a FourteenDOFDynamics).
bool attach_front_kinematics(IVehicleDynamics& dyn,
                              std::unique_ptr<ISuspensionKinematics> k) {
    auto* p = dynamic_cast<FourteenDOFDynamics*>(&dyn);
    if (!p) return false;
    p->set_kinematics_front(std::move(k));
    return true;
}
bool attach_rear_kinematics(IVehicleDynamics& dyn,
                             std::unique_ptr<ISuspensionKinematics> k) {
    auto* p = dynamic_cast<FourteenDOFDynamics*>(&dyn);
    if (!p) return false;
    p->set_kinematics_rear(std::move(k));
    return true;
}

bool fourteen_dof_attach_multibody(IVehicleDynamics& dyn,
                                   bool front_axle,
                                   const mb::SuspensionTopology& topo,
                                   bool enable_dynamics) {
    auto* p = dynamic_cast<FourteenDOFDynamics*>(&dyn);
    if (!p) return false;
    p->attach_mb_corner(front_axle, topo, enable_dynamics);
    return true;
}

bool fourteen_dof_mb_dynamics_enabled(const IVehicleDynamics& dyn, int axle) {
    const auto* p = dynamic_cast<const FourteenDOFDynamics*>(&dyn);
    if (!p) return false;
    return (axle == 0) ? p->mb_dyn_front_enabled() : p->mb_dyn_rear_enabled();
}

}  // namespace vdsim
