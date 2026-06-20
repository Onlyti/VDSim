// SimSession implementation. See sim_session.hpp.
#include "vdsim/sim_session.hpp"

#include <cmath>

namespace vdsim {

namespace {

void apply_vehicle_steer_deadtime(SimConfig& cfg, const VehicleParams& vp) {
    if (vp.steer_deadtime_s > 1e-9 && cfg.actuator.steer.ch.tau_s <= 1e-9)
        cfg.actuator.steer.ch.tau_s = vp.steer_deadtime_s;
}

void snapshot_dynamics(IVehicleDynamics& dyn,
                       std::array<Vec3, NUM_WHEELS>& tire_forces,
                       std::array<Vec3, NUM_WHEELS>& tire_forces_wheel,
                       std::array<double, NUM_WHEELS>& slip_ratio,
                       std::array<double, NUM_WHEELS>& slip_angle,
                       std::array<double, NUM_WHEELS>& wheel_mu,
                       std::array<double, NUM_WHEELS>& wheel_mu_peak,
                       std::array<double, NUM_WHEELS>& wheel_alpha_peak,
                       std::array<double, NUM_WHEELS>& wheel_kappa_peak) {
    tire_forces       = dyn.tire_forces_body();
    tire_forces_wheel = dyn.tire_forces_wheel();
    slip_ratio        = dyn.wheel_slip_ratio();
    slip_angle        = dyn.wheel_slip_angle();
    wheel_mu          = dyn.wheel_mu();
    wheel_mu_peak     = dyn.wheel_mu_peak();
    wheel_alpha_peak  = dyn.wheel_alpha_peak();
    wheel_kappa_peak  = dyn.wheel_kappa_peak();
}

}  // namespace

SimSession::SimSession(std::unique_ptr<IVehicleDynamics> dyn,
                       std::unique_ptr<IContactProvider> ground,
                       const VehicleParams& vp, const TireSetup& ts,
                       const SolverParams& sp, const SimConfig& cfg)
    : dyn_(std::move(dyn)), ground_(std::move(ground)), vp_(vp),
      session_kind_(cfg.session_kind) {
    dyn_->initialize(vp, ts, sp);
    free_3d_attach_contact_provider(*dyn_, ground_.get());
    network_ = make_default_veh_network(cfg.veh_network);
    SimConfig cfg_act = cfg;
    apply_vehicle_steer_deadtime(cfg_act, vp);
    actuator_.initialize(cfg_act.actuator, cfg_act.nominal_dt);
    sensor_.initialize(cfg.sensor_delay_s, cfg.nominal_dt);
    sensors_.initialize(cfg.sensors);
    cascade_.initialize(vp);
}

void SimSession::reset(const State& s0) {
    std::lock_guard<std::mutex> lk(mtx_);
    dyn_->reset(s0);
    network_->reset();
    actuator_.reset();
    sensor_.reset(s0);
    sensors_.reset();
    cascade_.reset();
    true_state_ = s0;
    meas_state_ = s0;
    latched_    = ControlInput{CmdL4{}};
    sim_time_   = 0.0;
    last_input_tp_ = std::chrono::steady_clock::now();
}

void SimSession::set_input(const CmdL4& u) {
    std::lock_guard<std::mutex> lk(mtx_);
    latched_ = ControlInput{u};
    last_input_tp_ = std::chrono::steady_clock::now();
}

void SimSession::set_input(const ControlInput& u) {
    std::lock_guard<std::mutex> lk(mtx_);
    latched_ = u;
    last_input_tp_ = std::chrono::steady_clock::now();
}

namespace {

double steer_from_input(const ControlInput& u) {
    return std::visit([](const auto& cmd) -> double {
        using T = std::decay_t<decltype(cmd)>;
        if constexpr (std::is_same_v<T, CmdSplit>) {
            return std::visit([](const auto& lat) -> double {
                using LT = std::decay_t<decltype(lat)>;
                if constexpr (std::is_same_v<LT, LcLatL4>) return lat.steer_angle;
                return 0.0;
            }, cmd.lat);
        } else if constexpr (std::is_same_v<T, CmdL7> || std::is_same_v<T, CmdL8>) {
            return 0.0;
        } else {
            return cmd.steer_angle_wheel;
        }
    }, u);
}

ControlInput with_steer_angle(const ControlInput& u, double steer) {
    return std::visit([&](const auto& cmd) -> ControlInput {
        using T = std::decay_t<decltype(cmd)>;
        if constexpr (std::is_same_v<T, CmdSplit>) {
            return u;
        } else if constexpr (std::is_same_v<T, CmdL7> || std::is_same_v<T, CmdL8>) {
            return u;
        } else {
            auto c = cmd;
            c.steer_angle_wheel = steer;
            return ControlInput{c};
        }
    }, u);
}

}  // namespace

void SimSession::tick(double dt) {
    if (!(dt > 0.0)) return;

    ControlInput cmd;
    State s;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        cmd = latched_;
        s   = true_state_;
    }

    ContactArray contacts;
    ground_->query(s, vp_, contacts);

    if (session_kind_ == SessionKind::DirectControl) {
        const double speed = s.speed_xy();
        CmdL4 steer_desired{};
        steer_desired.steer_angle_wheel = steer_from_input(cmd);
        const CmdL4 steer_realized = actuator_.apply(steer_desired, speed, dt);
        const ControlInput dyn_cmd = with_steer_angle(cmd, steer_realized.steer_angle_wheel);

        dyn_->step(dyn_cmd, contacts, dt);
        const State next = dyn_->state();
        const State meas = sensor_.apply(next, dt);

        const double ax = dyn_->ax_body_est();
        const double ay = dyn_->ay_body_est();
        const double roll = dyn_->roll_angle_qs();
        const double pitch = dyn_->pitch_angle_qs();
        const double rack = dyn_->steering_rack_torque();
        const auto Fz = dyn_->tire_Fz();
        const SensorMeas sm = sensors_.apply(
            next, ax, ay, steer_realized.steer_angle_wheel, dt);

        {
            std::lock_guard<std::mutex> lk(mtx_);
            true_state_ = next;
            meas_state_ = meas;
            ax_ = ax; ay_ = ay; roll_ = roll; pitch_ = pitch; rack_ = rack;
            steer_applied_ = steer_realized.steer_angle_wheel;
            throttle_applied_ = 0.0;
            brake_applied_ = 0.0;
            Fz_ = Fz;
            snapshot_dynamics(*dyn_, tire_forces_, tire_forces_wheel_,
                              slip_ratio_, slip_angle_, wheel_mu_, wheel_mu_peak_,
                              wheel_alpha_peak_, wheel_kappa_peak_);
            sensors_meas_ = sm;
            sim_time_  += dt;
        }
        return;
    }

    const double ax_meas = dyn_->ax_body_est();
    const CmdL4 ctrl_l4 = cascade_.to_l4(cmd, meas_state_, ax_meas, dt);

    const ControlInput net_cmd = network_->apply(ControlInput{ctrl_l4}, dt);
    const CmdL4 net_l4 = std::holds_alternative<CmdL4>(net_cmd)
                         ? std::get<CmdL4>(net_cmd) : ctrl_l4;

    const double speed = s.speed_xy();
    const CmdL4 realized = actuator_.apply(net_l4, speed, dt);

    dyn_->step(ControlInput{realized}, contacts, dt);
    const State next = dyn_->state();
    const State meas = sensor_.apply(next, dt);

    const double ax = dyn_->ax_body_est();
    const double ay = dyn_->ay_body_est();
    const double roll = dyn_->roll_angle_qs();
    const double pitch = dyn_->pitch_angle_qs();
    const double rack = dyn_->steering_rack_torque();
    const auto Fz = dyn_->tire_Fz();
    const SensorMeas sm = sensors_.apply(next, ax, ay, realized.steer_angle_wheel, dt);

    {
        std::lock_guard<std::mutex> lk(mtx_);
        true_state_ = next;
        meas_state_ = meas;
        ax_ = ax; ay_ = ay; roll_ = roll; pitch_ = pitch; rack_ = rack;
        steer_applied_ = realized.steer_angle_wheel;
        throttle_applied_ = realized.throttle;
        brake_applied_ = realized.brake;
        Fz_ = Fz;
        snapshot_dynamics(*dyn_, tire_forces_, tire_forces_wheel_,
                          slip_ratio_, slip_angle_, wheel_mu_, wheel_mu_peak_,
                          wheel_alpha_peak_, wheel_kappa_peak_);
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
    o.tire_forces_wheel = tire_forces_wheel_;
    o.slip_ratio = slip_ratio_;
    o.slip_angle = slip_angle_;
    o.wheel_mu = wheel_mu_;
    o.wheel_mu_peak = wheel_mu_peak_;
    o.wheel_alpha_peak = wheel_alpha_peak_;
    o.wheel_kappa_peak = wheel_kappa_peak_;
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

std::unique_ptr<IContactProvider> make_friction_ground(const FrictionMapConfig& cfg) {
    std::vector<PolygonMuPatch> poly = cfg.polygons;
    for (const auto& [x0, x1, mu] : cfg.x_bands) {
        PolygonMuPatch p;
        p.polygon = {{x0, -1.0e4}, {x1, -1.0e4}, {x1, 1.0e4}, {x0, 1.0e4}};
        p.mu = mu;
        poly.push_back(std::move(p));
    }
    if (poly.empty())
        return create_flat_ground(cfg.z, cfg.base_mu);
    return create_polygon_friction_ground(
        cfg.z, cfg.base_mu, poly, cfg.blend_distance);
}

std::unique_ptr<SimSession> make_direct_control_session(
    const VehicleParams& vp,
    const TireSetup& ts,
    const SolverParams& sp,
    const DirectControlSessionOptions& opts) {
    SimConfig cfg;
    cfg.session_kind = SessionKind::DirectControl;
    cfg.nominal_dt = opts.nominal_dt;
    apply_vehicle_steer_deadtime(cfg, vp);
    VehicleParams vp_dc = vp;
    vp_dc.plant_path = true;
    auto ground = make_friction_ground(opts.friction);
    return std::make_unique<SimSession>(
        create_seven_dof(), std::move(ground), vp_dc, ts, sp, cfg);
}

}  // namespace vdsim
