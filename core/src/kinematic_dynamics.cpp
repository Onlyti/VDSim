// Lk-Kinematic — kinematic bicycle model (no tire forces, no slip).
//
//   v̇   = a               (a from pedals, capped by torque/mass)
//   ψ̇   = v·tan(δ)/L      (kinematic steering, no sideslip)
//   ẋ_w = v·cosψ,  ẏ_w = v·sinψ
//
// Useful for path-planning and kinematic-MPC, and as the simplest ladder rung.
// ISO 8855 frame; vy = 0 by construction.
#include "vdsim/interfaces.hpp"

#include <algorithm>
#include <cmath>
#include <variant>

#include "vdsim/coordinate.hpp"

namespace vdsim {
namespace {

constexpr double kGravity = 9.81;

inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Reduce any control input to pedal-level (throttle/brake/steer).
CmdL4 to_l4(const ControlInput& u) {
    return std::visit([](const auto& cmd) -> CmdL4 {
        using T = std::decay_t<decltype(cmd)>;
        CmdL4 out;
        if constexpr (std::is_same_v<T, CmdL4>) {
            out = cmd;
        } else if constexpr (std::is_same_v<T, CmdL1> || std::is_same_v<T, CmdL2> ||
                             std::is_same_v<T, CmdL3> || std::is_same_v<T, CmdL5> ||
                             std::is_same_v<T, CmdL6>) {
            out.steer_angle_wheel = cmd.steer_angle_wheel;   // longitudinal best-effort 0
        }
        return out;
    }, u);
}

class KinematicBicycle final : public IVehicleDynamics {
public:
    Level level() const noexcept override { return Level::Lk_Kinematic; }

    void initialize(const VehicleParams& vp, const TireSetup&,
                    const SolverParams& sp) override {
        vp_ = vp; sp_ = sp;
    }

    void reset(const State& s) noexcept override {
        state_ = s; ax_ = 0.0; ay_ = 0.0;
    }

    void step(const ControlInput& u, const ContactArray&, double dt) noexcept override {
        if (!(dt > 0.0)) return;
        const CmdL4 c = to_l4(u);
        const double m  = std::max(1.0, vp_.mass);
        const double L  = std::max(1e-3, vp_.wheelbase);
        const double R  = std::max(1e-3, vp_.wheel_radius_nominal);
        const double A  = vp_.max_motor_torque / (R * m);     // accel cap [m/s^2]
        const double B  = vp_.max_brake_torque / (R * m);     // decel cap
        const double thr = clampd(c.throttle, 0.0, 1.0);
        const double brk = clampd(c.brake, 0.0, 1.0);
        const double steer = clampd(c.steer_angle_wheel,
                                    -vp_.max_steer_angle_wheel, vp_.max_steer_angle_wheel);
        const int gear = (c.gear != 0) ? c.gear : 1;

        const int N = std::max(1, std::min(sp_.max_substeps,
                        static_cast<int>(std::ceil(dt / std::max(1e-6, sp_.max_substep_dt)))));
        const double h = dt / static_cast<double>(N);

        double v   = state_.velocity.x();
        double yaw = yaw_from_quat(state_.orientation);
        double x   = state_.position.x();
        double y   = state_.position.y();
        double yawrate = 0.0;
        double ax = thr * A * (gear >= 0 ? 1.0 : -1.0) - brk * B * (v > 0 ? 1.0 : -1.0);

        for (int i = 0; i < N; ++i) {
            v += ax * h;
            if (gear >= 0) v = std::max(0.0, v);      // no creep backwards on a fwd gear
            else           v = std::min(0.0, v);
            yawrate = v * std::tan(steer) / L;
            yaw += yawrate * h;
            x   += v * std::cos(yaw) * h;
            y   += v * std::sin(yaw) * h;
        }

        state_.position.x() = x;
        state_.position.y() = y;
        state_.velocity     = Vec3(v, 0.0, 0.0);
        state_.angular_velocity = Vec3(0.0, 0.0, yawrate);
        state_.orientation  = quat_from_euler({0.0, 0.0, yaw});
        const double w = v / R;
        state_.wheel_spin = {{w, w, w, w}};
        ax_ = ax;
        ay_ = v * yawrate;     // centripetal
    }

    const State& state() const noexcept override { return state_; }

    std::array<Vec3, NUM_WHEELS> tire_forces_body() const override { return {}; }
    std::array<double, NUM_WHEELS> tire_Fz() const override {
        const double L = std::max(1e-3, vp_.wheelbase);
        const double Ff = vp_.mass * kGravity * vp_.cg_to_rear  / (2.0 * L);
        const double Fr = vp_.mass * kGravity * vp_.cg_to_front / (2.0 * L);
        return {{Ff, Ff, Fr, Fr}};
    }
    std::array<double, NUM_WHEELS> wheel_slip_ratio() const override { return {}; }
    std::array<double, NUM_WHEELS> wheel_slip_angle() const override { return {}; }
    double ax_body_est() const override { return ax_; }
    double ay_body_est() const override { return ay_; }

private:
    VehicleParams vp_{};
    SolverParams  sp_{};
    State         state_{};
    double        ax_{0.0}, ay_{0.0};
};

}  // namespace

std::unique_ptr<IVehicleDynamics> create_kinematic() {
    return std::make_unique<KinematicBicycle>();
}

}  // namespace vdsim
