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
    base_vp_ = vp;
    base_ts_ = ts;
    base_sp_ = sp;
    base_sensor_delay_s_ = cfg.sensor_delay_s;
    base_sensors_ = cfg.sensors;
    nominal_dt_ = cfg.nominal_dt;
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


// ---- R8: snapshot / restore ----------------------------------------------

SessionSnapshot SimSession::snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    SessionSnapshot s;
    auto& v = s.d;
    snap::put_state(v, true_state_);
    snap::put_state(v, meas_state_);
    v.push_back(sim_time_);

    // Latched command.  The RL / direct path latches CmdL4; a higher ladder
    // level is flagged (tag 0) and restored as the default CmdL4 rather than
    // silently pretending it round-tripped.
    if (std::holds_alternative<CmdL4>(latched_)) {
        const CmdL4& u = std::get<CmdL4>(latched_);
        v.push_back(1.0);
        v.push_back(u.throttle); v.push_back(u.brake);
        v.push_back(u.steer_angle_wheel);
        v.push_back(static_cast<double>(u.gear));
        v.push_back(u.handbrake ? 1.0 : 0.0);
        v.push_back(static_cast<double>(static_cast<int>(u.steer_mode)));
        v.push_back(u.steer_actuator);
    } else {
        v.push_back(0.0);
        for (int k = 0; k < 7; ++k) v.push_back(0.0);
    }

    v.push_back(ax_); v.push_back(ay_); v.push_back(roll_);
    v.push_back(pitch_); v.push_back(rack_);
    v.push_back(az_); v.push_back(heave_z_);
    v.push_back(steer_applied_); v.push_back(throttle_applied_);
    v.push_back(brake_applied_);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(Fz_[i]);
    for (int i = 0; i < NUM_WHEELS; ++i)
        for (int k = 0; k < 3; ++k) v.push_back(tire_forces_[i][k]);
    for (int i = 0; i < NUM_WHEELS; ++i)
        for (int k = 0; k < 3; ++k) v.push_back(tire_forces_wheel_[i][k]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(slip_ratio_[i]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(slip_angle_[i]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(wheel_mu_[i]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(wheel_mu_peak_[i]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(wheel_alpha_peak_[i]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(wheel_kappa_peak_[i]);

    const SensorMeas& m = sensors_meas_;
    v.push_back(m.ax); v.push_back(m.ay); v.push_back(m.az);
    v.push_back(m.wx); v.push_back(m.wy); v.push_back(m.wz);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(m.wheel_speed[i]);
    v.push_back(m.steer);
    v.push_back(m.gnss_x); v.push_back(m.gnss_y);
    v.push_back(m.gnss_vx); v.push_back(m.gnss_vy);

    actuator_.save_aux(v);
    sensor_.save_aux(v);
    sensors_.save_aux(v, s.rng);
    dyn_->save_aux(v);
    return s;
}

void SimSession::restore(const SessionSnapshot& s) {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto& v = s.d;
    std::size_t p = 0;
    true_state_ = snap::get_state(v, p);
    meas_state_ = snap::get_state(v, p);
    sim_time_   = snap::get(v, p);

    const double tag = snap::get(v, p);
    CmdL4 u;
    u.throttle          = snap::get(v, p);
    u.brake             = snap::get(v, p);
    u.steer_angle_wheel = snap::get(v, p);
    u.gear              = static_cast<int>(snap::get(v, p));
    u.handbrake         = snap::get(v, p) != 0.0;
    u.steer_mode        = static_cast<SteerMode>(static_cast<int>(snap::get(v, p)));
    u.steer_actuator    = snap::get(v, p);
    latched_ = (tag != 0.0) ? ControlInput{u} : ControlInput{CmdL4{}};

    ax_ = snap::get(v, p); ay_ = snap::get(v, p); roll_ = snap::get(v, p);
    pitch_ = snap::get(v, p); rack_ = snap::get(v, p);
    az_ = snap::get(v, p); heave_z_ = snap::get(v, p);
    steer_applied_ = snap::get(v, p); throttle_applied_ = snap::get(v, p);
    brake_applied_ = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) Fz_[i] = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i)
        for (int k = 0; k < 3; ++k) tire_forces_[i][k] = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i)
        for (int k = 0; k < 3; ++k) tire_forces_wheel_[i][k] = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) slip_ratio_[i] = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) slip_angle_[i] = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) wheel_mu_[i] = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) wheel_mu_peak_[i] = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) wheel_alpha_peak_[i] = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) wheel_kappa_peak_[i] = snap::get(v, p);

    SensorMeas& m = sensors_meas_;
    m.ax = snap::get(v, p); m.ay = snap::get(v, p); m.az = snap::get(v, p);
    m.wx = snap::get(v, p); m.wy = snap::get(v, p); m.wz = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) m.wheel_speed[i] = snap::get(v, p);
    m.steer = snap::get(v, p);
    m.gnss_x = snap::get(v, p); m.gnss_y = snap::get(v, p);
    m.gnss_vx = snap::get(v, p); m.gnss_vy = snap::get(v, p);

    actuator_.restore_aux(v, p);
    sensor_.restore_aux(v, p);
    sensors_.restore_aux(v, p, s.rng);
    // Order matters: reset() loads the model's State and clears its tire
    // transients, so the aux block is replayed after it, not before.
    dyn_->reset(true_state_);
    dyn_->restore_aux(v, p);
}

void SimSession::settle_on_ground(State& s) {
    std::lock_guard<std::mutex> lk(mtx_);
    settle_spawn_on_ground(*ground_, vp_, s);
}

State SimSession::reset_settled(const State& s0) {
    State s = s0;
    settle_on_ground(s);          // takes and releases mtx_
    reset(s);
    return s;
}

void SimSession::set_seed(unsigned seed) {
    std::lock_guard<std::mutex> lk(mtx_);
    seed_ = seed;
    has_seed_ = true;
}

void SimSession::set_randomization(const DomainRandomization& d) {
    std::lock_guard<std::mutex> lk(mtx_);
    rand_ = d;
    rand_dirty_ = true;
    mu_scale_ = (d.mu_scale > 0.0) ? d.mu_scale : 1.0;
}

DomainRandomization SimSession::randomization() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return rand_;
}

// Rebuild the plant parameters from the as-built baseline and re-initialize in
// place.  Called under mtx_ from reset().
void SimSession::apply_randomization_locked() {
    VehicleParams vp = base_vp_;
    if (rand_.mass_scale > 0.0) {
        vp.mass        *= rand_.mass_scale;
        vp.mass_sprung *= rand_.mass_scale;
        vp.inertia_diag = vp.inertia_diag * rand_.mass_scale;
    }
    TireSetup ts = base_ts_;
    for (int w = 0; w < NUM_WHEELS; ++w) {
        TireParams& tp = ts.wheel[w];
        if (rand_.tire_stiffness_scale > 0.0) {
            tp.B_long              *= rand_.tire_stiffness_scale;
            tp.B_lat               *= rand_.tire_stiffness_scale;
            tp.cornering_stiffness *= rand_.tire_stiffness_scale;
        }
        if (rand_.tire_mu_scale > 0.0) tp.mu_nominal *= rand_.tire_mu_scale;
    }
    vp_ = vp;
    dyn_->initialize(vp, ts, base_sp_);
    cascade_.initialize(vp);
    const double delay = (rand_.sensor_delay_s >= 0.0) ? rand_.sensor_delay_s
                                                       : base_sensor_delay_s_;
    sensor_.initialize(delay, nominal_dt_);
}

void SimSession::reset(const State& s0) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (rand_dirty_) {
        apply_randomization_locked();
        rand_dirty_ = false;
    }
    if (has_seed_) {                      // R6: re-arm the noise stream
        SensorParams sp = base_sensors_;
        sp.seed = seed_;
        sensors_.initialize(sp);
    }
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
    // R6: the diagnostic snapshot is part of the observable session state, so
    // a reset session must read the same as a freshly built one -- otherwise
    // the first observation of episode k+1 still carries episode k's accel,
    // slip and load.
    ax_ = ay_ = roll_ = pitch_ = rack_ = 0.0;
    az_ = heave_z_ = 0.0;
    contacts_ = ContactArray{};
    steer_applied_ = throttle_applied_ = brake_applied_ = 0.0;
    Fz_.fill(0.0);
    slip_ratio_.fill(0.0);
    slip_angle_.fill(0.0);
    wheel_mu_.fill(0.0);
    wheel_mu_peak_.fill(0.0);
    wheel_alpha_peak_.fill(0.0);
    wheel_kappa_peak_.fill(0.0);
    tire_forces_.fill(Vec3::Zero());
    tire_forces_wheel_.fill(Vec3::Zero());
    sensors_meas_ = SensorMeas{};
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
    // R5: scale the reported friction without rebuilding the provider.  The L5
    // free-3D per-substep re-query talks to the provider directly and is not
    // covered by this scaling.
    if (mu_scale_ != 1.0) {
        for (auto& cp : contacts) {
            cp.mu_long *= mu_scale_;
            cp.mu_lat  *= mu_scale_;
        }
    }

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
        const double az = dyn_->az_body_est();
        const double heave = dyn_->heave_z();
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
            az_ = az; heave_z_ = heave; contacts_ = contacts;
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
    const double az = dyn_->az_body_est();
    const double heave = dyn_->heave_z();
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
        az_ = az; heave_z_ = heave; contacts_ = contacts;
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
    o.az = az_; o.heave_z = heave_z_; o.contacts = contacts_;
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
    auto dyn =
        (opts.level == "K" || opts.level == "L0") ? create_kinematic()
        : (opts.level == "L1") ? create_bicycle()
        : (opts.level == "L3") ? create_fourteen_dof()
        : (opts.level == "L4") ? create_fourteen_dof_kinematic()
                               : create_seven_dof();
    return std::make_unique<SimSession>(
        std::move(dyn), std::move(ground), vp_dc, ts, sp, cfg);
}

}  // namespace vdsim
