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

ProportionalBrake::ProportionalBrake(const VehicleParams& vp, double deadtime_s)
    : vp_(vp), brake_delay_(deadtime_s) {}

void ProportionalBrake::begin_step(const SubsystemContext& ctx, double dt) {
    brake_eff_ = brake_delay_.step(ctx.cmd.brake, dt);
}

std::array<double, NUM_WHEELS> ProportionalBrake::wheel_torque(const SubsystemContext& ctx) {
    const double brake = brake_eff_;
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

void ProportionalBrake::reset() {
    brake_eff_ = 0.0;
    brake_delay_.reset();
}

BasicDrivetrain::BasicDrivetrain(const VehicleParams& vp, double deadtime_s)
    : vp_(vp), throttle_delay_(deadtime_s) {}

void BasicDrivetrain::begin_step(const SubsystemContext& ctx, double dt) {
    throttle_eff_ = throttle_delay_.step(ctx.cmd.throttle, dt);
}

DrivetrainOutput BasicDrivetrain::apply(const SubsystemContext& ctx) {
    const double throttle = throttle_eff_;
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

void BasicDrivetrain::reset() {
    throttle_eff_ = 0.0;
    throttle_delay_.reset();
}

RatioSteering::RatioSteering(const VehicleParams& vp, double deadtime_s)
    : steering_ratio_(std::max(1e-6, vp.steering_ratio)), handwheel_delay_(deadtime_s) {}

RatioSteering::RatioSteering(double steering_ratio, double deadtime_s)
    : steering_ratio_(std::max(1e-6, steering_ratio)), handwheel_delay_(deadtime_s) {}

void RatioSteering::begin_step(const SubsystemContext& ctx, double dt) {
    handwheel_eff_ = handwheel_delay_.step(ctx.cmd.handwheel_angle, dt);
}

SteeringOutput RatioSteering::apply(const SubsystemContext& ctx) {
    const double handwheel = handwheel_eff_;
    SteeringOutput out;
    out.roadwheel_angle = handwheel / steering_ratio_;
    out.rack_travel     = out.roadwheel_angle;
    return out;
}

void RatioSteering::reset() {
    handwheel_eff_ = 0.0;
    handwheel_delay_.reset();
}

UnitySteering::UnitySteering(double deadtime_s) : handwheel_delay_(deadtime_s) {}

void UnitySteering::begin_step(const SubsystemContext& ctx, double dt) {
    handwheel_eff_ = handwheel_delay_.step(ctx.cmd.handwheel_angle, dt);
}

SteeringOutput UnitySteering::apply(const SubsystemContext& ctx) {
    SteeringOutput out;
    out.roadwheel_angle = handwheel_eff_;
    out.rack_travel     = out.roadwheel_angle;
    return out;
}

void UnitySteering::reset() {
    handwheel_eff_ = 0.0;
    handwheel_delay_.reset();
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

std::unique_ptr<IDrivetrain> make_default_drivetrain(const VehicleParams& vp, double deadtime_s) {
    return std::make_unique<BasicDrivetrain>(vp, deadtime_s);
}

std::unique_ptr<ISuspension> make_default_suspension(const VehicleParams& vp) {
    return std::make_unique<LinearSuspension>(vp);
}

std::unique_ptr<IAntiRollBar> make_default_antirollbar(const VehicleParams& vp, int axle) {
    return std::make_unique<LinearARB>(vp, axle);
}

}  // namespace vdsim
