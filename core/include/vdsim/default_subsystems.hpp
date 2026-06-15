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
