#pragma once

#include <array>
#include <stdexcept>
#include <string>
#include <utility>

#include "vdsim/control.hpp"
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
    enum class Mode { Kinematic, Dynamic } mode {Mode::Kinematic};
    // Kinematic: ISteeringSystem directly sets rack position (no tire feedback)
    double rack_travel   {0.0};   // [m]  — used when mode == Kinematic
    // Dynamic: ISteeringSystem outputs motor force; Rack EOM integrated in IVehicleDynamics
    double motor_force   {0.0};   // [N at rack] — used when mode == Dynamic
    // Backward-compat alias: roadwheel_angle derived from rack_travel by ISteeringKinematics
    double roadwheel_angle {0.0}; // [rad] filled by ISteeringKinematics after apply()
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
    // Fraction of brake demand this drivetrain handled (e.g. EV regen).
    // IBrakeSystem receives: effective_brake = cmd.brake * (1 - brake_absorbed).
    // ICE = 0.0, full regen EV = up to 1.0, hybrid = intermediate.
    double brake_absorbed {0.0};  // [0, 1]
};

// Helper: throw LcLevel mismatch error with a clear message.
inline void check_lc_level(LcLevel input, LcLevel lo, LcLevel hi,
                            const char* subsystem) {
    if (input < lo || input > hi) {
        throw std::runtime_error(
            std::string("LcLevel mismatch in ") + subsystem
            + ": input L" + std::to_string(static_cast<int>(input))
            + " outside supported range [L" + std::to_string(static_cast<int>(lo))
            + ", L" + std::to_string(static_cast<int>(hi)) + "]"
            + " — implement a subsystem that handles this level, or raise/lower the input.");
    }
}

struct IBrakeSystem {
    // LcLon range this subsystem natively accepts. Default: L4 (pedal only).
    virtual LcLevel min_lon_level() const { return LcLevel::L4; }
    virtual LcLevel max_lon_level() const { return LcLevel::L4; }

    virtual std::array<double, NUM_WHEELS> wheel_torque(const SubsystemContext&) = 0;
    virtual void begin_step(const SubsystemContext&, double /*dt*/) {}
    virtual void reset() {}
    virtual ~IBrakeSystem() = default;
};

struct ISteeringSystem {
    // LcLat range this subsystem natively accepts. Default: L4 (wheel angle only).
    virtual LcLevel min_lat_level() const { return LcLevel::L4; }
    virtual LcLevel max_lat_level() const { return LcLevel::L4; }

    virtual SteeringOutput apply(const SubsystemContext&) = 0;
    virtual void begin_step(const SubsystemContext&, double /*dt*/) {}
    virtual void reset() {}
    virtual ~ISteeringSystem() = default;
};

struct IDrivetrain {
    // LcLon range this subsystem natively accepts. Default: L4 (pedal only).
    virtual LcLevel min_lon_level() const { return LcLevel::L4; }
    virtual LcLevel max_lon_level() const { return LcLevel::L4; }

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
