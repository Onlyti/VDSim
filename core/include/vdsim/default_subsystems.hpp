#pragma once

#include <memory>

#include "vdsim/delay_line.hpp"
#include "vdsim/params.hpp"
#include "vdsim/subsystems.hpp"

namespace vdsim {

class ProportionalBrake final : public IBrakeSystem {
public:
    explicit ProportionalBrake(const VehicleParams& vp, double deadtime_s = 0.0);

    std::array<double, NUM_WHEELS> wheel_torque(const SubsystemContext& ctx) override;
    void reset() override;

private:
    VehicleParams vp_;
    DelayLine     brake_delay_;
};

class BasicDrivetrain final : public IDrivetrain {
public:
    explicit BasicDrivetrain(const VehicleParams& vp, double deadtime_s = 0.0);

    DrivetrainOutput apply(const SubsystemContext& ctx) override;
    void reset() override;

private:
    VehicleParams vp_;
    DelayLine     throttle_delay_;
};

class RatioSteering final : public ISteeringSystem {
public:
    explicit RatioSteering(const VehicleParams& vp, double deadtime_s = 0.0);
    RatioSteering(double steering_ratio, double deadtime_s = 0.0);

    SteeringOutput apply(const SubsystemContext& ctx) override;
    void reset() override;

private:
    double    steering_ratio_;
    DelayLine handwheel_delay_;
};

class UnitySteering final : public ISteeringSystem {
public:
    explicit UnitySteering(double deadtime_s = 0.0);

    SteeringOutput apply(const SubsystemContext& ctx) override;
    void reset() override;

private:
    DelayLine handwheel_delay_;
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
