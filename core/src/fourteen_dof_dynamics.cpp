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

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/suspension.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

namespace vdsim {

namespace {

constexpr double kGravity = 9.80665;

class FourteenDOFDynamics final : public IVehicleDynamics {
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
        inner_->reset(state_);
        z_s_ = 0.0; z_s_dot_ = 0.0;
        phi_ = 0.0; phi_dot_ = 0.0;
        th_  = 0.0; th_dot_  = 0.0;
        z_u_     = {{0.0, 0.0, 0.0, 0.0}};
        z_u_dot_ = {{0.0, 0.0, 0.0, 0.0}};
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
        if (kine_front_ || kine_rear_) {
            for (int i = 0; i < NUM_WHEELS; ++i) {
                const double z_corner_s = z_s_ + ry_[i] * std::sin(phi_)
                                                - rx_[i] * std::sin(th_);
                const double wheel_travel = z_u_[i] - z_corner_s;
                ISuspensionKinematics* k = (i < 2) ? kine_front_.get()
                                                   : kine_rear_.get();
                if (k) {
                    const auto o = k->compute(wheel_travel, 0.0);
                    const bool right = (i == WHEEL_FR || i == WHEEL_RR);
                    gamma[i] = right ? -o.camber : o.camber;
                    toe[i]   = right ? -o.toe    : o.toe;
                } else {
                    gamma[i] = (i == WHEEL_FR || i == WHEEL_RR)
                                ? -vp_.camber_per_roll * phi_
                                : +vp_.camber_per_roll * phi_;
                }
            }
        } else {
            const double k_cam = vp_.camber_per_roll;
            gamma = {{ +k_cam * phi_, -k_cam * phi_,
                       +k_cam * phi_, -k_cam * phi_ }};
        }
        inner_->set_camber_per_wheel(gamma);
        inner_->set_toe_per_wheel(toe);

        // Dynamic per-corner tire load from the 14-DOF ride state (suspension
        // transfer via z_u + road/terrain via road_dz), fed to the inner grip
        // calc in place of its quasi-static Fz. z_u_ is the previous step's value
        // (1-step lag, consistent with the inner's lagged transfer). Static is the
        // full corner share scaled by the per-wheel contact-normal cos(slope) plus
        // aero downforce, matching the inner's static (only the unsprung lateral
        // transfer term is omitted).
        {
            constexpr double kAirDensity = 1.225;
            const double k_tire = std::max(1.0, tp_.tire_vertical_stiffness);
            const double Lwb = vp_.wheelbase;
            const double vx  = state_.velocity.x();
            const double q_aero = 0.5 * kAirDensity * vp_.frontal_area * vx * std::abs(vx);
            const double aero_f = 0.5 * vp_.aero_lift_front * q_aero;
            const double aero_r = 0.5 * vp_.aero_lift_rear  * q_aero;
            const double st_f = vp_.mass * kGravity * vp_.cg_to_rear  / (2.0 * Lwb);
            const double st_r = vp_.mass * kGravity * vp_.cg_to_front / (2.0 * Lwb);
            std::array<double, NUM_WHEELS> fz_dyn;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                const double cos_slope = std::max(0.1, contacts[i].normal.z());
                const double st   = ((i < 2) ? st_f : st_r) * cos_slope + ((i < 2) ? aero_f : aero_r);
                fz_dyn[i] = std::max(0.0, st + k_tire * (contacts[i].road_dz - z_u_[i]));
            }
            inner_->set_external_fz(fz_dyn);
        }

        inner_->step(u, contacts, dt);
        state_ = inner_->state();
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
        double Fz_sum = 0.0, M_pitch_spring = 0.0, M_roll_spring = 0.0;
        std::array<double, NUM_WHEELS> F_susp{};
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double z_corner = z + ry_[i] * phi - rx_[i] * th;
            const double v_corner = z_dot + ry_[i] * phi_dot - rx_[i] * th_dot;
            const double delta    = z_corner - zu[i];        // relative compression deviation
            const double vel      = v_corner - zu_dot[i];
            const double k        = std::max(1.0, vp_.spring_stiffness[i]);
            const double c        = std::max(0.0, vp_.damper_coefficient[i]);
            const double F        = -k * delta - c * vel;     // force on sprung (positive = up)
            F_susp[i]      = F;
            Fz_sum        += F;
            M_pitch_spring -= rx_[i] * F;
            M_roll_spring  += ry_[i] * F;
        }
        // Anti-roll bar: roll-only restoring moment (undamped), added on top of the
        // spring-derived roll stiffness above.
        const double K_arb = std::max(0.0, vp_.arb_stiffness_front)
                           + std::max(0.0, vp_.arb_stiffness_rear);

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
        d.dphi_dot = (M_roll_spring - K_arb * phi + m_s * ay * h) / std::max(1e-3, Ixx);
        d.dth      = th_dot;
        d.dth_dot  = (M_pitch_spring + M_inertia_pitch) / std::max(1e-3, Iyy);

        // Unsprung mass per corner: m_u · z̈_u = -F_susp(on sprung) - k_tire · z_u
        // = +F_susp_on_unsprung - k_tire · z_u
        // F_susp_on_unsprung = -F_susp(on sprung) (Newton III)
        const double k_tire = std::max(1.0, tp_.tire_vertical_stiffness);
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double m_u = std::max(1.0, vp_.unsprung_mass[i]);
            d.dz_u[i]     = zu_dot[i];
            d.dz_u_dot[i] = (- F_susp[i] - k_tire * (zu[i] - road_dz_[i])) / m_u;
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
    std::array<double, NUM_WHEELS> road_dz_ {{0.0, 0.0, 0.0, 0.0}};  // road roughness input

    // Optional hardpoint kinematics (Ld4 Stage D).  When attached, replaces
    // the lumped vp_.camber_per_roll · phi heuristic for the corresponding axle.
    std::unique_ptr<ISuspensionKinematics> kine_front_;
    std::unique_ptr<ISuspensionKinematics> kine_rear_;
};

}  // namespace

std::unique_ptr<IVehicleDynamics> create_fourteen_dof() {
    return std::make_unique<FourteenDOFDynamics>();
}

std::unique_ptr<IVehicleDynamics> create_fourteen_dof(std::unique_ptr<ITireModel> tire) {
    return std::make_unique<FourteenDOFDynamics>(std::move(tire));
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

}  // namespace vdsim
