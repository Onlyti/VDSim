#pragma once

#include <memory>

#include "vdsim/params.hpp"
#include "vdsim/subsystems.hpp"

namespace vdsim {

// Delay lives exclusively in ActuatorModel (SimSession level) so subsystem modules
// receive already-realized commands via ctx.cmd. The deadtime_s constructor parameter
// is kept for API compatibility but is no longer used inside these classes.

class ProportionalBrake final : public IBrakeSystem {
public:
    explicit ProportionalBrake(const VehicleParams& vp, double /*deadtime_s*/ = 0.0);

    std::array<double, NUM_WHEELS> wheel_torque(const SubsystemContext& ctx) override;
    void reset() override {}   // no state to reset

private:
    VehicleParams vp_;
};

// SimpleABS — reference anti-lock brake module (swappable; not default).
// Wraps the proportional/EBD base brake, then per wheel reduces brake torque
// when the wheel slip ratio exceeds the lock threshold (Bosch-style slip
// regulation around the longitudinal grip peak). A first reference algorithm
// developers can replace via set_brake_module() with their own ABS/ESC.
//
//   slip_i = (omega_i·R − vx) / max(vx, eps)   (negative = braking/locking)
//   if |slip_i| > slip_release: scale brake torque down toward slip_target
class SimpleABS final : public IBrakeSystem {
public:
    explicit SimpleABS(const VehicleParams& vp,
                       double slip_target = 0.12,   // peak-grip longitudinal slip
                       double slip_release = 0.18);  // release above this magnitude

    std::array<double, NUM_WHEELS> wheel_torque(const SubsystemContext& ctx) override;
    void reset() override { mod_.fill(1.0); }

private:
    ProportionalBrake base_;
    VehicleParams     vp_;
    double            slip_target_;
    double            slip_release_;
    std::array<double, NUM_WHEELS> mod_ {{1.0, 1.0, 1.0, 1.0}};  // per-wheel torque scale
};

class BasicDrivetrain final : public IDrivetrain {
public:
    explicit BasicDrivetrain(const VehicleParams& vp, double /*deadtime_s*/ = 0.0);

    DrivetrainOutput apply(const SubsystemContext& ctx) override;
    void reset() override {}

private:
    VehicleParams vp_;
};

// Drivetrain v2: 2D engine torque map + gearbox + shift policy (opt-in). The stateful
// EngineGearbox is advanced once per step in begin_step(); apply() splits the frozen
// axle torque to the wheels (so it is safe to call inside the RK4 stages).
class EngineGearboxDrivetrain final : public IDrivetrain {
public:
    explicit EngineGearboxDrivetrain(const VehicleParams& vp, double /*deadtime_s*/ = 0.0);

    DrivetrainOutput apply(const SubsystemContext& ctx) override;
    void begin_step(const SubsystemContext& ctx, double dt) override;
    void reset() override;
    double engine_rpm()   const override { return eg_.engine_rpm(); }
    int    current_gear() const override { return eg_.current_gear(); }
    double wheel_engine_inertia(int wheel) const override;
    bool   set_shift_policy(ShiftPolicy fn) override { eg_.set_shift_policy(std::move(fn)); return true; }

private:
    VehicleParams vp_;
    EngineGearbox eg_;
    double        T_front_ {0.0};
    double        T_rear_  {0.0};
};

class RatioSteering final : public ISteeringSystem {
public:
    explicit RatioSteering(const VehicleParams& vp, double /*deadtime_s*/ = 0.0);
    RatioSteering(double steering_ratio, double /*deadtime_s*/ = 0.0);

    SteeringOutput apply(const SubsystemContext& ctx) override;
    void reset() override {}

private:
    double steering_ratio_;
};

class UnitySteering final : public ISteeringSystem {
public:
    explicit UnitySteering(double /*deadtime_s*/ = 0.0);

    SteeringOutput apply(const SubsystemContext& ctx) override;
    void reset() override {}
};

// Dynamic steering: emits a motor force into the Rack EOM (integrated by the
// dynamics). Accepts sub-L4 lateral commands (steer torque / ang-vel / ang-accel)
// as well as an angle reference (position servo). This is the EPS/MDPS path —
// tire aligning-moment kickback feeds back through the Rack EOM.
//
//   SteerMode::Torque   : motor_force = steer_torque / pinion_radius (column → rack)
//   SteerMode::AngVel   : velocity servo  Kv·(rack_vel_target − rack_vel)
//   SteerMode::AngAccel : feed-forward    m_rack·rack_accel_target + c_rack·rack_vel
//   SteerMode::Angle    : position servo  Kp·(rack_target − rack) − Kd·rack_vel
class DynamicSteering final : public ISteeringSystem {
public:
    explicit DynamicSteering(const VehicleParams& vp);

    // Accepts L1 (torque) through L4 (angle) — the cascade handles L5-L8 → angle.
    LcLevel min_lat_level() const override { return LcLevel::L1; }
    LcLevel max_lat_level() const override { return LcLevel::L4; }

    SteeringOutput apply(const SubsystemContext& ctx) override;
    void reset() override {}

private:
    double ratio_;          // rack travel ↔ wheel angle (rack = angle·ratio)
    double pinion_radius_;
    double m_rack_;
    double c_rack_;
    double pos_kp_ {4.0e4};  // position-servo gain (Angle mode)
    double pos_kd_ {2.0e3};
    double vel_kv_ {1.0e4};  // velocity-servo gain (AngVel mode)
};

class LinearSuspension final : public ISuspension {
public:
    explicit LinearSuspension(const VehicleParams& vp);

    double force(const SubsystemContext&, const CornerInput& corner) override;

private:
    VehicleParams vp_;
};

class LinearARB final : public IAntiRollBar {
public:
    LinearARB(const VehicleParams& vp, int axle);

    std::pair<double, double> force(const SubsystemContext&, const AxleDefl& defl) override;

private:
    double K_arb_  {0.0};
    double track_  {1.0};
};

std::unique_ptr<IBrakeSystem>    make_default_brake(const VehicleParams& vp,
                                                    double deadtime_s = 0.0);
std::unique_ptr<ISteeringSystem> make_default_steering(const VehicleParams& vp,
                                                       double deadtime_s = 0.0);
std::unique_ptr<IDrivetrain>     make_default_drivetrain(const VehicleParams& vp,
                                                         double deadtime_s = 0.0);
std::unique_ptr<ISuspension>     make_default_suspension(const VehicleParams& vp);
std::unique_ptr<IAntiRollBar>    make_default_antirollbar(const VehicleParams& vp, int axle);

}  // namespace vdsim
