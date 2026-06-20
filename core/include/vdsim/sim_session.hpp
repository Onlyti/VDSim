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
#include <tuple>
#include <vector>

#include "vdsim/actuator.hpp"
#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/control_converter.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sensors.hpp"
#include "vdsim/state.hpp"
#include "vdsim/veh_network.hpp"

namespace vdsim {

enum class SessionKind {
    Standard,       // L4 cascade + default low-speed kinematic blend on L2
    DirectControl,  // latched CmdL1 direct to dynamics; plant low-speed shaping
};

struct SimConfig {
    ActuatorParams    actuator      {};      // physical actuator (lag, rate, sat)
    SensorParams      sensors       {};      // sensor noise model
    VehNetworkParams  veh_network   {};      // ECU/CAN deadtime + drop (default: identity)
    double            sensor_delay_s {0.0}; // feedback transport delay (0 = none)
    double            nominal_dt    {0.005}; // for delay-buffer sizing
    SessionKind       session_kind  {SessionKind::Standard};
};

// Thread-safe snapshot of one tick's result (true + measured state plus the
// dynamics diagnostics needed for logging / the co-sim STATE packet).
struct SimOutput {
    State  state    {};                  // true plant state
    State  measured {};                  // sensor-delayed (controller feedback)
    double sim_time {0.0};
    double ax {0.0}, ay {0.0};           // body accel (ax_body_est / ay_body_est)
    double roll {0.0}, pitch {0.0};      // roll_angle_qs / pitch_angle_qs
    std::array<double, NUM_WHEELS> Fz {{0,0,0,0}};
    std::array<Vec3, NUM_WHEELS>   tire_forces {};       // body-frame per wheel [N]
    std::array<Vec3, NUM_WHEELS>   tire_forces_wheel {}; // contact / wheel frame [N]
    std::array<double, NUM_WHEELS> slip_ratio {{0,0,0,0}};
    std::array<double, NUM_WHEELS> slip_angle {{0,0,0,0}};
    std::array<double, NUM_WHEELS> wheel_mu {{0,0,0,0}};
    std::array<double, NUM_WHEELS> wheel_mu_peak {{0,0,0,0}};
    std::array<double, NUM_WHEELS> wheel_alpha_peak {{0,0,0,0}};
    std::array<double, NUM_WHEELS> wheel_kappa_peak {{0,0,0,0}};
    double rack_torque {0.0};
    double steer_applied {0.0};     // realized steer after actuator [rad]
    double throttle_applied {0.0};  // realized throttle after actuator [0,1]
    double brake_applied {0.0};     // realized brake after actuator [0,1]
    SensorMeas sensors {};        // noisy/biased measured signals (identity if disabled)
};

class SimSession {
public:
    SimSession(std::unique_ptr<IVehicleDynamics> dyn,
               std::unique_ptr<IContactProvider> ground,
               const VehicleParams& vp, const TireSetup& ts,
               const SolverParams& sp, const SimConfig& cfg = {});
    SimSession(std::unique_ptr<IVehicleDynamics> dyn,
               std::unique_ptr<IContactProvider> ground,
               const VehicleParams& vp, const TireParams& tp,
               const SolverParams& sp, const SimConfig& cfg = {})
        : SimSession(std::move(dyn), std::move(ground), vp, TireSetup(tp), sp, cfg) {}

    void reset(const State& s0);

    // Latch the command (thread-safe). Subsequent ticks use it until replaced.
    void set_input(const CmdL4& u);
    // Latch any ladder-level command (CmdL1..CmdL8). Levels above L4 are cascaded
    // to CmdL4 each tick by the CascadeController using the measured-state feedback.
    void set_input(const ControlInput& u);

    // Advance one fixed-dt step: latched cmd -> actuator -> step -> sensor.
    void tick(double dt);

    State  state() const;             // true plant state (pull)
    State  measured_state() const;    // sensor-delayed state (controller feedback)
    SimOutput output() const;         // full thread-safe snapshot (state + diagnostics)
    double sim_time() const;
    double seconds_since_last_input() const;  // for failsafe (wall clock)

    const VehicleParams& vehicle_params() const { return vp_; }

    // Access the (initialized) plant — e.g. to install user module plugins post-construct.
    IVehicleDynamics& dynamics() { return *dyn_; }

private:
    std::unique_ptr<IVehicleDynamics> dyn_;
    std::unique_ptr<IContactProvider> ground_;
    std::unique_ptr<IVehNetwork>      network_;   // ECU/CAN network (deadtime + drop)
    ActuatorModel     actuator_;
    SensorDelay       sensor_;
    SensorModel       sensors_;
    CascadeController cascade_;   // Lc5-L8 → CmdL4 (with measured-state feedback)
    VehicleParams vp_;

    mutable std::mutex mtx_;
    ControlInput latched_ {CmdL4{}};
    State  true_state_ {};
    State  meas_state_ {};
    double ax_ {0.0}, ay_ {0.0}, roll_ {0.0}, pitch_ {0.0}, rack_ {0.0};
    double steer_applied_ {0.0}, throttle_applied_ {0.0}, brake_applied_ {0.0};
    std::array<double, NUM_WHEELS> Fz_ {{0,0,0,0}};
    std::array<Vec3, NUM_WHEELS>   tire_forces_ {};
    std::array<Vec3, NUM_WHEELS>   tire_forces_wheel_ {};
    std::array<double, NUM_WHEELS> slip_ratio_ {{0,0,0,0}};
    std::array<double, NUM_WHEELS> slip_angle_ {{0,0,0,0}};
    std::array<double, NUM_WHEELS> wheel_mu_ {{0,0,0,0}};
    std::array<double, NUM_WHEELS> wheel_mu_peak_ {{0,0,0,0}};
    std::array<double, NUM_WHEELS> wheel_alpha_peak_ {{0,0,0,0}};
    std::array<double, NUM_WHEELS> wheel_kappa_peak_ {{0,0,0,0}};
    SensorMeas sensors_meas_ {};
    double sim_time_ {0.0};
    SessionKind session_kind_ {SessionKind::Standard};
    std::chrono::steady_clock::time_point last_input_tp_ {std::chrono::steady_clock::now()};
};

struct FrictionMapConfig {
    double z {0.0};
    double base_mu {1.0};
    std::vector<std::tuple<double, double, double>> x_bands;
    std::vector<PolygonMuPatch> polygons;
    double blend_distance {1.0};
};

std::unique_ptr<IContactProvider> make_friction_ground(const FrictionMapConfig& cfg);

struct DirectControlSessionOptions {
    FrictionMapConfig friction;
    double nominal_dt {0.001};
};

std::unique_ptr<SimSession> make_direct_control_session(
    const VehicleParams& vp,
    const TireSetup& ts,
    const SolverParams& sp,
    const DirectControlSessionOptions& opts = {});

}  // namespace vdsim
