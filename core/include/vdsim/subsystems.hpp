#pragma once

#include <array>
#include <utility>

#include "vdsim/powertrain.hpp"
#include "vdsim/state.hpp"
#include "vdsim/types.hpp"

namespace vdsim {

struct DriverCmd {
    double handwheel_angle {0.0};
    double throttle        {0.0};
    double brake           {0.0};
    int    gear            {1};
    bool   handbrake       {false};
};

struct SubsystemContext {
    const State&     state;
    const DriverCmd& cmd;
    double           dt;
    std::array<double, NUM_WHEELS> Fz {{0.0, 0.0, 0.0, 0.0}};
};

struct SteeringOutput {
    double roadwheel_angle {0.0};
    double rack_travel     {0.0};
};

struct CornerInput {
    int    corner        {0};
    double defl          {0.0};
    double defl_rate     {0.0};
    double damping_scale {1.0};
};

struct AxleDefl {
    double defl_left  {0.0};
    double defl_right {0.0};
    double rate_left  {0.0};
    double rate_right {0.0};
};

struct DrivetrainOutput {
    std::array<double, NUM_WHEELS> wheel_torque {{0.0, 0.0, 0.0, 0.0}};
};

struct IBrakeSystem {
    virtual std::array<double, NUM_WHEELS> wheel_torque(const SubsystemContext&) = 0;
    virtual void begin_step(const SubsystemContext&, double /*dt*/) {}
    virtual void reset() {}
    virtual ~IBrakeSystem() = default;
};

struct ISteeringSystem {
    virtual SteeringOutput apply(const SubsystemContext&) = 0;
    virtual void begin_step(const SubsystemContext&, double /*dt*/) {}
    virtual void reset() {}
    virtual ~ISteeringSystem() = default;
};

struct IDrivetrain {
    virtual DrivetrainOutput apply(const SubsystemContext&) = 0;
    virtual void begin_step(const SubsystemContext&, double /*dt*/) {}
    virtual void reset() {}
    // Engine + gearbox introspection (0 / 0 for the legacy flat drivetrain).
    virtual double engine_rpm()   const { return 0.0; }
    virtual int    current_gear() const { return 0; }
    // Per-wheel engine inertia reflected through the current gear [kg m^2];
    // negative -> "no override, host uses its legacy final-drive reflection".
    virtual double wheel_engine_inertia(int /*wheel*/) const { return -1.0; }
    // Install a programmatic shift policy (e.g. a Python callable). Returns false
    // if the drivetrain has no gearbox. Declared here to avoid a downcast.
    virtual bool set_shift_policy(ShiftPolicy /*fn*/) { return false; }
    virtual ~IDrivetrain() = default;
};

struct ISuspension {
    virtual double force(const SubsystemContext&, const CornerInput&) = 0;
    virtual void reset() {}
    virtual ~ISuspension() = default;
};

struct IAntiRollBar {
    virtual std::pair<double, double> force(const SubsystemContext&, const AxleDefl&) = 0;
    virtual void reset() {}
    virtual ~IAntiRollBar() = default;
};

}  // namespace vdsim
