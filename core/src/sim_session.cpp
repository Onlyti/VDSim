// SimSession implementation. See sim_session.hpp.
#include "vdsim/sim_session.hpp"

namespace vdsim {

SimSession::SimSession(std::unique_ptr<IVehicleDynamics> dyn,
                       std::unique_ptr<IContactProvider> ground,
                       const VehicleParams& vp, const TireParams& tp,
                       const SolverParams& sp, const SimConfig& cfg)
    : dyn_(std::move(dyn)), ground_(std::move(ground)), vp_(vp) {
    dyn_->initialize(vp, tp, sp);
    actuator_.initialize(cfg.actuator, cfg.nominal_dt);
    sensor_.initialize(cfg.sensor_delay_s, cfg.nominal_dt);
}

void SimSession::reset(const State& s0) {
    std::lock_guard<std::mutex> lk(mtx_);
    dyn_->reset(s0);
    actuator_.reset();
    sensor_.reset(s0);
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

    const double speed = s.speed_xy();
    const CmdL4 realized = actuator_.apply(cmd, speed, dt);

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

    {
        std::lock_guard<std::mutex> lk(mtx_);
        true_state_ = next;
        meas_state_ = meas;
        ax_ = ax; ay_ = ay; roll_ = roll; pitch_ = pitch; rack_ = rack;
        Fz_ = Fz;
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
    o.Fz = Fz_;
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
