// SimSession implementation. See sim_session.hpp.
#include "vdsim/sim_session.hpp"

namespace vdsim {

SimSession::SimSession(std::unique_ptr<IVehicleDynamics> dyn,
                       std::unique_ptr<IContactProvider> ground,
                       const VehicleParams& vp, const TireParams& tp,
                       const SolverParams& sp, const SimConfig& cfg)
    : dyn_(std::move(dyn)), ground_(std::move(ground)), vp_(vp) {
    dyn_->initialize(vp, tp, sp);
    network_ = make_default_veh_network(cfg.veh_network);
    actuator_.initialize(cfg.actuator, cfg.nominal_dt);
    sensor_.initialize(cfg.sensor_delay_s, cfg.nominal_dt);
    sensors_.initialize(cfg.sensors);
}

void SimSession::reset(const State& s0) {
    std::lock_guard<std::mutex> lk(mtx_);
    dyn_->reset(s0);
    network_->reset();
    actuator_.reset();
    sensor_.reset(s0);
    sensors_.reset();
    true_state_ = s0;
    meas_state_ = s0;
    latched_    = CmdL4{};
    sim_time_   = 0.0;
    last_input_tp_ = std::chrono::steady_clock::now();
}

void SimSession::set_input(const CmdL4& u) {
    std::lock_guard<std::mutex> lk(mtx_);
    latched_ = u;
    last_input_tp_ = std::chrono::steady_clock::now();
}

void SimSession::tick(double dt) {
    if (!(dt > 0.0)) return;

    CmdL4 cmd;
    State s;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        cmd = latched_;
        s   = true_state_;
    }

    ContactArray contacts;
    ground_->query(s, vp_, contacts);

    // 1. VehNetwork: stochastic deadtime + packet drop (ECU/CAN layer)
    const ControlInput net_cmd = network_->apply(ControlInput{cmd}, dt);
    // VehNetwork preserves the Lc level; SimSession operates on CmdL4 (Lc4 is the
    // SimSession native boundary). Higher-level Lc variants are lowered by the
    // dynamics' own lower_to_l4 inside dyn_->step(). Here we extract CmdL4 directly
    // since set_input currently accepts only CmdL4 (set_input(const CmdL4&)).
    const CmdL4 net_l4 = std::holds_alternative<CmdL4>(net_cmd)
                         ? std::get<CmdL4>(net_cmd) : cmd;

    // 2. Actuator: physical lag, rate-limit, saturation
    const double speed = s.speed_xy();
    const CmdL4 realized = actuator_.apply(net_l4, speed, dt);

    dyn_->step(ControlInput{realized}, contacts, dt);
    const State next = dyn_->state();
    const State meas = sensor_.apply(next, dt);

    // Snapshot diagnostics while we hold the (single) sim thread.
    const double ax = dyn_->ax_body_est();
    const double ay = dyn_->ay_body_est();
    const double roll = dyn_->roll_angle_qs();
    const double pitch = dyn_->pitch_angle_qs();
    const double rack = dyn_->steering_rack_torque();
    const auto Fz = dyn_->tire_Fz();
    const auto Ft = dyn_->tire_forces_body();
    const auto kappa = dyn_->wheel_slip_ratio();
    const auto alpha = dyn_->wheel_slip_angle();
    const SensorMeas sm = sensors_.apply(next, ax, ay, realized.steer_angle_wheel, dt);

    {
        std::lock_guard<std::mutex> lk(mtx_);
        true_state_ = next;
        meas_state_ = meas;
        ax_ = ax; ay_ = ay; roll_ = roll; pitch_ = pitch; rack_ = rack;
        steer_applied_ = realized.steer_angle_wheel;
        throttle_applied_ = realized.throttle;
        brake_applied_ = realized.brake;
        Fz_ = Fz; tire_forces_ = Ft;
        slip_ratio_ = kappa; slip_angle_ = alpha;
        sensors_meas_ = sm;
        sim_time_  += dt;
    }
}

SimOutput SimSession::output() const {
    std::lock_guard<std::mutex> lk(mtx_);
    SimOutput o;
    o.state = true_state_;
    o.measured = meas_state_;
    o.sim_time = sim_time_;
    o.ax = ax_; o.ay = ay_; o.roll = roll_; o.pitch = pitch_;
    o.rack_torque = rack_;
    o.steer_applied = steer_applied_;
    o.throttle_applied = throttle_applied_;
    o.brake_applied = brake_applied_;
    o.Fz = Fz_;
    o.tire_forces = tire_forces_;
    o.slip_ratio = slip_ratio_;
    o.slip_angle = slip_angle_;
    o.sensors = sensors_meas_;
    return o;
}

State SimSession::state() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return true_state_;
}

State SimSession::measured_state() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return meas_state_;
}

double SimSession::sim_time() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return sim_time_;
}

double SimSession::seconds_since_last_input() const {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - last_input_tp_).count();
}

}  // namespace vdsim
