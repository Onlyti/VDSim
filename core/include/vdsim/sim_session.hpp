// SimSession — mode-agnostic simulation kernel.
//
// Owns the dynamics + ground + actuator + sensor-delay layers and one tick of
// the loop: latched input -> actuator -> step -> sensor. It carries no clock and
// no I/O, so every run mode is a thin wiring on top of the same kernel:
//   - experiment / external-step: caller calls set_input()+tick() in its own loop
//   - real-time free-run:        a RealTimeRunner paces tick() to the wall clock
//   - scenario replay:           a loop feeds set_input() from a file
//
// set_input() latches the command (ZOH) so an asynchronous command stream
// (e.g. UDP) is decoupled from the fixed-dt tick, exactly like an ECU/actuator
// holding the last command. set_input()/state() are mutex-guarded so a producer
// thread and the sim thread can run concurrently.
#pragma once

#include <chrono>
#include <memory>
#include <mutex>

#include "vdsim/actuator.hpp"
#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"

namespace vdsim {

struct SimConfig {
    ActuatorParams actuator     {};      // default: all effects off (identity)
    double         sensor_delay_s {0.0}; // feedback transport delay (0 = none)
    double         nominal_dt   {0.005}; // for delay-buffer sizing
};

class SimSession {
public:
    SimSession(std::unique_ptr<IVehicleDynamics> dyn,
               std::unique_ptr<IContactProvider> ground,
               const VehicleParams& vp, const TireParams& tp,
               const SolverParams& sp, const SimConfig& cfg = {});

    void reset(const State& s0);

    // Latch the command (thread-safe). Subsequent ticks use it until replaced.
    void set_input(const CmdL4& u);

    // Advance one fixed-dt step: latched cmd -> actuator -> step -> sensor.
    void tick(double dt);

    State  state() const;             // true plant state (pull)
    State  measured_state() const;    // sensor-delayed state (controller feedback)
    double sim_time() const;
    double seconds_since_last_input() const;  // for failsafe (wall clock)

    const VehicleParams& vehicle_params() const { return vp_; }

private:
    std::unique_ptr<IVehicleDynamics> dyn_;
    std::unique_ptr<IContactProvider> ground_;
    ActuatorModel actuator_;
    SensorDelay   sensor_;
    VehicleParams vp_;

    mutable std::mutex mtx_;
    CmdL4  latched_ {};
    State  true_state_ {};
    State  meas_state_ {};
    double sim_time_ {0.0};
    std::chrono::steady_clock::time_point last_input_tp_ {std::chrono::steady_clock::now()};
};

}  // namespace vdsim
