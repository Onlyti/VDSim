#include "vdsim/default_subsystems.hpp"

#include <algorithm>
#include <cmath>

namespace vdsim {
namespace {

constexpr double kGravity    = 9.80665;
constexpr double kBrakeWidth = 1.0;

inline double smooth_sign(double x, double w) { return std::tanh(x / w); }

void split_axle(const VehicleParams& vp, double T_axle,
                double omega_L, double omega_R, double& T_L, double& T_R) {
    switch (vp.differential) {
        case VehicleParams::Differential::Open:
            T_L = T_R = 0.5 * T_axle;
            break;
        case VehicleParams::Differential::Locked: {
            const double dw = omega_L - omega_R;
            const double bias = 0.45 * std::tanh(2.0 * dw);
            T_L = (0.5 - bias) * T_axle;
            T_R = (0.5 + bias) * T_axle;
            break;
        }
        case VehicleParams::Differential::LSD: {
            const double dw = omega_L - omega_R;
            const double mag = std::clamp(
                vp.lsd_preload + vp.lsd_ramp * std::abs(dw), 0.0, 0.45);
            const double bias = mag * std::tanh(2.0 * dw);
            T_L = (0.5 - bias) * T_axle;
            T_R = (0.5 + bias) * T_axle;
            break;
        }
    }
}

double ebd_front_bias(const VehicleParams& vp,
                      const std::array<double, NUM_WHEELS>& Fz) {
    double bias = std::clamp(vp.brake_bias_front, 0.0, 1.0);
    if (!vp.brake_ebd_enabled) return bias;

    const double Fz_f = Fz[WHEEL_FL] + Fz[WHEEL_FR];
    const double Fz_r = Fz[WHEEL_RL] + Fz[WHEEL_RR];
    const double total = Fz_f + Fz_r;
    if (total > 1.0) {
        bias = std::clamp(Fz_f / total, 0.05, 0.95);
    } else {
        const double L = vp.wheelbase;
        const double Fz_f_static = vp.mass * kGravity * vp.cg_to_rear / L;
        const double Fz_r_static = vp.mass * kGravity * vp.cg_to_front / L;
        const double total_static = Fz_f_static + Fz_r_static;
        if (total_static > 1.0) {
            bias = std::clamp(Fz_f_static / total_static, 0.05, 0.95);
        }
    }
    return bias;
}

}  // namespace

ProportionalBrake::ProportionalBrake(const VehicleParams& vp, double /*deadtime_s*/)
    : vp_(vp) {}

std::array<double, NUM_WHEELS> ProportionalBrake::wheel_torque(const SubsystemContext& ctx) {
    const double brake = ctx.cmd.brake;   // already realized by ActuatorModel
    const double bias = ebd_front_bias(vp_, ctx.Fz);
    const double Tbrk_front_axle =       bias  * brake * vp_.max_brake_torque;
    const double Tbrk_rear_axle  = (1.0 - bias) * brake * vp_.max_brake_torque;

    std::array<double, NUM_WHEELS> Tb{};
    for (int i = 0; i < NUM_WHEELS; ++i) {
        const double axle_T = (i == WHEEL_FL || i == WHEEL_FR)
                              ? Tbrk_front_axle : Tbrk_rear_axle;
        Tb[i] = -smooth_sign(ctx.state.wheel_spin[i], kBrakeWidth) * (0.5 * axle_T);
    }
    return Tb;
}

// ── SimpleABS ───────────────────────────────────────────────────────────────
SimpleABS::SimpleABS(const VehicleParams& vp, double slip_target, double slip_release)
    : base_(vp), vp_(vp), slip_target_(slip_target), slip_release_(slip_release) {}

std::array<double, NUM_WHEELS> SimpleABS::wheel_torque(const SubsystemContext& ctx) {
    const auto Tb = base_.wheel_torque(ctx);   // proportional + EBD base demand
    const double vx = ctx.state.vx();
    const double R  = std::max(1e-3, vp_.wheel_radius_nominal);
    const double v  = std::max(std::abs(vx), 1.0);   // avoid /0 at low speed

    std::array<double, NUM_WHEELS> out{};
    for (int i = 0; i < NUM_WHEELS; ++i) {
        // Longitudinal slip ratio (negative under braking = wheel slower than ground).
        const double slip = (ctx.state.wheel_spin[i] * R - vx) / v;
        const double lock = -slip;   // positive when the wheel is locking up

        // Release valve: above the release threshold, ramp the modulation down
        // toward zero; recover (re-apply) below the target. First-order toward goal.
        double goal = 1.0;
        if (lock > slip_release_) {
            goal = 0.0;                                   // dump pressure
        } else if (lock > slip_target_) {
            goal = (slip_release_ - lock) / (slip_release_ - slip_target_);  // taper
        }
        // Smooth the valve so it cycles rather than chatters every step.
        mod_[i] += 0.4 * (goal - mod_[i]);
        out[i] = Tb[i] * std::clamp(mod_[i], 0.0, 1.0);
    }
    return out;
}

BasicDrivetrain::BasicDrivetrain(const VehicleParams& vp, double /*deadtime_s*/)
    : vp_(vp) {}

DrivetrainOutput BasicDrivetrain::apply(const SubsystemContext& ctx) {
    const double throttle = ctx.cmd.throttle;   // already realized by ActuatorModel
    const double Tmot = throttle * vp_.max_motor_torque * vp_.final_drive_ratio;

    double T_front_axle = 0.0, T_rear_axle = 0.0;
    switch (vp_.drive_type) {
        case VehicleParams::Drive::FWD: T_front_axle = Tmot; break;
        case VehicleParams::Drive::RWD: T_rear_axle  = Tmot; break;
        case VehicleParams::Drive::AWD:
            T_front_axle = 0.5 * Tmot;
            T_rear_axle  = 0.5 * Tmot;
            break;
    }

    DrivetrainOutput out{};
    split_axle(vp_, T_front_axle,
               ctx.state.wheel_spin[WHEEL_FL], ctx.state.wheel_spin[WHEEL_FR],
               out.wheel_torque[WHEEL_FL], out.wheel_torque[WHEEL_FR]);
    split_axle(vp_, T_rear_axle,
               ctx.state.wheel_spin[WHEEL_RL], ctx.state.wheel_spin[WHEEL_RR],
               out.wheel_torque[WHEEL_RL], out.wheel_torque[WHEEL_RR]);
    if (ctx.cmd.gear < 0) {
        for (auto& x : out.wheel_torque) x = -x;
    }
    return out;
}

RatioSteering::RatioSteering(const VehicleParams& vp, double /*deadtime_s*/)
    : steering_ratio_(std::max(1e-6, vp.steering_ratio)) {}

RatioSteering::RatioSteering(double steering_ratio, double /*deadtime_s*/)
    : steering_ratio_(std::max(1e-6, steering_ratio)) {}

SteeringOutput RatioSteering::apply(const SubsystemContext& ctx) {
    const double handwheel = ctx.cmd.handwheel_angle;   // already realized by ActuatorModel
    SteeringOutput out;
    out.roadwheel_angle = handwheel / steering_ratio_;
    out.rack_travel     = out.roadwheel_angle;
    return out;
}

UnitySteering::UnitySteering(double /*deadtime_s*/) {}

SteeringOutput UnitySteering::apply(const SubsystemContext& ctx) {
    SteeringOutput out;
    out.roadwheel_angle = ctx.cmd.handwheel_angle;   // already realized by ActuatorModel
    // rack_travel left at 0: UnitySteering is a kinematic simplification that
    // maps handwheel_angle directly to wheel angle without rack geometry.
    return out;
}

// ── DynamicSteering ─────────────────────────────────────────────────────────
DynamicSteering::DynamicSteering(const VehicleParams& vp)
    : ratio_(std::max(1e-6, vp.steering_ratio)),
      pinion_radius_(std::max(1e-4, vp.pinion_radius)),
      m_rack_(std::max(1e-3, vp.rack_mass)),
      c_rack_(vp.rack_damping) {}

SteeringOutput DynamicSteering::apply(const SubsystemContext& ctx) {
    SteeringOutput out;
    out.mode = SteeringOutput::Mode::Dynamic;
    const double rack     = ctx.state.rack_travel;     // generalized steering DOF
    const double rack_vel = ctx.state.rack_velocity;

    switch (ctx.cmd.steer_mode) {
        case SteerMode::Torque: {
            // Column/motor torque [Nm] → rack force [N] via pinion radius.
            out.motor_force = ctx.cmd.steer_actuator / pinion_radius_;
            break;
        }
        case SteerMode::AngVel: {
            // Target wheel angular rate → target rack velocity (rack = angle·ratio).
            const double rack_vel_tgt = ctx.cmd.steer_actuator * ratio_;
            out.motor_force = vel_kv_ * (rack_vel_tgt - rack_vel);
            break;
        }
        case SteerMode::AngAccel: {
            // Feed-forward: F = m·a_rack + c·v_rack (a_rack = wheel ang-accel · ratio).
            const double rack_acc_tgt = ctx.cmd.steer_actuator * ratio_;
            out.motor_force = m_rack_ * rack_acc_tgt + c_rack_ * rack_vel;
            break;
        }
        case SteerMode::Angle:
        default: {
            // Position servo: drive rack toward the commanded angle (× ratio).
            const double rack_tgt = ctx.cmd.handwheel_angle * ratio_;
            out.motor_force = pos_kp_ * (rack_tgt - rack) - pos_kd_ * rack_vel;
            break;
        }
    }
    return out;
}

LinearSuspension::LinearSuspension(const VehicleParams& vp) : vp_(vp) {}

double LinearSuspension::force(const SubsystemContext&, const CornerInput& corner) {
    const int i = corner.corner;
    const double k = std::max(1.0, vp_.spring_stiffness[i]);
    const double c = std::max(0.0, vp_.damper_coefficient[i]);
    return -k * corner.defl - c * corner.damping_scale * corner.defl_rate;
}

LinearARB::LinearARB(const VehicleParams& vp, int axle) {
    K_arb_ = std::max(0.0, axle ? vp.arb_stiffness_rear : vp.arb_stiffness_front);
    track_ = std::max(1e-3, axle ? vp.track_rear : vp.track_front);
}

std::pair<double, double> LinearARB::force(const SubsystemContext&, const AxleDefl& defl) {
    const double diff = defl.defl_left - defl.defl_right;
    const double k    = K_arb_ / (track_ * track_);
    const double F    = k * diff;
    return {-F, F};
}

std::unique_ptr<IBrakeSystem> make_default_brake(const VehicleParams& vp, double deadtime_s) {
    return std::make_unique<ProportionalBrake>(vp, deadtime_s);
}

std::unique_ptr<ISteeringSystem> make_default_steering(const VehicleParams& vp, double deadtime_s) {
    (void)vp;
    return std::make_unique<UnitySteering>(deadtime_s);
}

// --- Drivetrain v2: engine torque map + gearbox + shift policy -------------------
EngineGearboxDrivetrain::EngineGearboxDrivetrain(const VehicleParams& vp, double /*deadtime_s*/)
    : vp_(vp) {
    eg_.configure(vp.powertrain);
}

void EngineGearboxDrivetrain::begin_step(const SubsystemContext& ctx, double dt) {
    const double throttle_eff_ = ctx.cmd.throttle;   // already realized by ActuatorModel
    const auto& w = ctx.state.wheel_spin;
    double driven_omega = 0.0;
    switch (vp_.drive_type) {
        case VehicleParams::Drive::FWD: driven_omega = 0.5 * (w[WHEEL_FL] + w[WHEEL_FR]); break;
        case VehicleParams::Drive::RWD: driven_omega = 0.5 * (w[WHEEL_RL] + w[WHEEL_RR]); break;
        case VehicleParams::Drive::AWD:
            driven_omega = 0.25 * (w[WHEEL_FL] + w[WHEEL_FR] + w[WHEEL_RL] + w[WHEEL_RR]); break;
    }
    double T = eg_.axle_torque(throttle_eff_, ctx.cmd.brake, driven_omega,
                               ctx.state.vx(), ctx.cmd.gear, dt);
    if (eg_.current_gear() < 0) T = -T;   // reverse gear drives backward
    switch (vp_.drive_type) {
        case VehicleParams::Drive::FWD: T_front_ = T;       T_rear_ = 0.0;     break;
        case VehicleParams::Drive::RWD: T_front_ = 0.0;     T_rear_ = T;       break;
        case VehicleParams::Drive::AWD: T_front_ = 0.5 * T; T_rear_ = 0.5 * T; break;
    }
}

DrivetrainOutput EngineGearboxDrivetrain::apply(const SubsystemContext& ctx) {
    DrivetrainOutput out{};
    split_axle(vp_, T_front_,
               ctx.state.wheel_spin[WHEEL_FL], ctx.state.wheel_spin[WHEEL_FR],
               out.wheel_torque[WHEEL_FL], out.wheel_torque[WHEEL_FR]);
    split_axle(vp_, T_rear_,
               ctx.state.wheel_spin[WHEEL_RL], ctx.state.wheel_spin[WHEEL_RR],
               out.wheel_torque[WHEEL_RL], out.wheel_torque[WHEEL_RR]);
    return out;
}

void EngineGearboxDrivetrain::reset() {
    eg_.reset();
    T_front_ = 0.0;
    T_rear_  = 0.0;
}

double EngineGearboxDrivetrain::wheel_engine_inertia(int wheel) const {
    const double I_axle = eg_.reflected_inertia();
    const bool front = (wheel == WHEEL_FL || wheel == WHEEL_FR);
    switch (vp_.drive_type) {
        case VehicleParams::Drive::FWD: return front ? 0.5 * I_axle : 0.0;
        case VehicleParams::Drive::RWD: return front ? 0.0 : 0.5 * I_axle;
        case VehicleParams::Drive::AWD: return 0.25 * I_axle;
    }
    return 0.0;
}

std::unique_ptr<IDrivetrain> make_default_drivetrain(const VehicleParams& vp, double deadtime_s) {
    if (vp.powertrain.enabled)
        return std::make_unique<EngineGearboxDrivetrain>(vp, deadtime_s);
    return std::make_unique<BasicDrivetrain>(vp, deadtime_s);
}

std::unique_ptr<ISuspension> make_default_suspension(const VehicleParams& vp) {
    return std::make_unique<LinearSuspension>(vp);
}

std::unique_ptr<IAntiRollBar> make_default_antirollbar(const VehicleParams& vp, int axle) {
    return std::make_unique<LinearARB>(vp, axle);
}

}  // namespace vdsim
