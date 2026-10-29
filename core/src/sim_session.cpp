// SimSession implementation. See sim_session.hpp.
#include "vdsim/sim_session.hpp"

#include "vdsim/aero.hpp"

#include <spdlog/spdlog.h>

#include <atomic>

#include <cmath>
#include <stdexcept>

namespace vdsim {

namespace {

void apply_vehicle_steer_deadtime(SimConfig& cfg, const VehicleParams& vp) {
    if (vp.steer_deadtime_s > 1e-9 && cfg.actuator.steer.ch.tau_s <= 1e-9)
        cfg.actuator.steer.ch.tau_s = vp.steer_deadtime_s;
}

// True when a throttle / brake channel does anything; otherwise the DirectControl
// torque path is left untouched so the default stays bit-identical.
bool channel_engaged(const ChannelActuator& c) {
    return c.dead_time_s > 0.0 || c.tau_s > 0.0 || c.rate_limit > 0.0 ||
           c.dead_zone > 0.0 || c.out_min > 0.0 || c.out_max < 1.0 ||
           c.backlash > 0.0 || c.user != nullptr;
}

bool longitudinal_engaged(const ActuatorParams& p) {
    return channel_engaged(p.throttle) || channel_engaged(p.brake.ch) ||
           p.brake.thermal_enabled;
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
    validate_aero_params(vp);
    dyn_->initialize(vp, ts, sp);
    {
        // L1/L2 have no roll split for aero: an element y offset is ignored.
        const auto lvl = dyn_->level();
        if (lvl == IVehicleDynamics::Level::L1_Bicycle ||
            lvl == IVehicleDynamics::Level::L2_SevenDOF) {
            static std::atomic<bool> warned{false};   // once per process
            for (const auto& e : vp.aero_elements) {
                if (e.y != 0.0 && !warned.exchange(true)) {
                    spdlog::warn("[vdsim aero] element '{}' y={} is ignored on L1/L2 "
                                 "(lateral offset is modelled on L3-L5 only)", e.name, e.y);
                    break;
                }
            }
        }
    }
    {
        // Torque-reaction targets are an L5 model; L1-L4 read them and ignore them.
        if (dyn_->level() != IVehicleDynamics::Level::L5_Stunt &&
            (vp.drive_reaction_front || vp.drive_reaction_rear ||
             vp.brake_reaction_front || vp.brake_reaction_rear)) {
            static std::atomic<bool> warned{false};   // once per process
            if (!warned.exchange(true))
                spdlog::warn("[vdsim] drive_reaction_* / brake_reaction_* are ignored "
                             "below L5 (torque reactions are modelled on L5 only)");
        }
    }
    free_3d_attach_contact_provider(*dyn_, ground_.get());
    network_ = make_default_veh_network(cfg.veh_network);
    SimConfig cfg_act = cfg;
    apply_vehicle_steer_deadtime(cfg_act, vp);
    base_actuator_ = cfg_act.actuator;
    actuator_.initialize(cfg_act.actuator, cfg_act.nominal_dt);
    sensor_.initialize(cfg.sensor_delay_s, cfg.nominal_dt);
    sensors_.initialize(cfg.sensors);
    cascade_.initialize(vp);

    drive_ref_ = (cfg.drive_torque_ref > 0.0) ? cfg.drive_torque_ref
                                              : vp.max_motor_torque * vp.final_drive_ratio;
    brake_ref_ = (cfg.brake_torque_ref > 0.0) ? cfg.brake_torque_ref
                                              : vp.max_brake_torque;
    const double sf = vp.drive_split_front;
    w0_ = {{0.5 * sf, 0.5 * sf, 0.5 * (1.0 - sf), 0.5 * (1.0 - sf)}};
    w_drive_ = w_brake_ = w0_;
}


// ---- R8: snapshot / restore ----------------------------------------------

SessionSnapshot SimSession::snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    SessionSnapshot s;
    auto& v = s.d;
    snap::put_header(v);
    snap::put_state(v, true_state_);
    snap::put_state(v, meas_state_);
    v.push_back(sim_time_);

    // Latched command.  Tag 1 = CmdL4 (RL pedal / direct path), tag 2 = CmdL5
    // (RL accel path, first two slots); any other ladder level is flagged (tag 0)
    // and restored as the default CmdL4 rather than silently pretending it
    // round-tripped.
    if (std::holds_alternative<CmdL5>(latched_)) {
        const CmdL5& u = std::get<CmdL5>(latched_);
        v.push_back(2.0);
        v.push_back(u.ax_target); v.push_back(u.steer_angle_wheel);
        for (int k = 0; k < 5; ++k) v.push_back(0.0);
    } else if (std::holds_alternative<CmdL4>(latched_)) {
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
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(w_drive_[i]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(w_brake_[i]);
    cascade_.save_state(v);   // keep last: tests/rl/test_rl_env.py addresses it as the tail
    return s;
}

void SimSession::restore(const SessionSnapshot& s) {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto& v = s.d;
    std::size_t p = 0;
    snap::check_header(v, p);
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
    if (tag == 2.0) {
        CmdL5 u5;
        u5.ax_target         = u.throttle;           // same slots, see snapshot()
        u5.steer_angle_wheel = u.brake;
        latched_ = ControlInput{u5};
    } else {
        latched_ = (tag != 0.0) ? ControlInput{u} : ControlInput{CmdL4{}};
    }

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
    for (int i = 0; i < NUM_WHEELS; ++i) w_drive_[i] = snap::get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) w_brake_[i] = snap::get(v, p);
    cascade_.restore_state(v, p);
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

void SimSession::set_actuator(const ActuatorParams& p) {
    std::lock_guard<std::mutex> lk(mtx_);
    actuator_.set_params(p);
}

ActuatorParams SimSession::actuator() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return actuator_.params();
}

void SimSession::set_cascade_gains(const CascadeGains& g) {
    const double v[] = {g.vx.kp, g.vx.ki, g.vx.i_max, g.vx.ax_clamp,
                        g.ax.kp, g.ax.ki, g.ax.kd, g.ax.kff, g.ax.i_max,
                        g.pp_lookahead_min, g.pp_lookahead_k, g.yaw_kp, g.yaw_ki};
    for (double x : v)
        if (!std::isfinite(x) || x < 0.0)
            throw std::invalid_argument(
                "SimSession.set_cascade_gains: every gain must be finite and >= 0");
    std::lock_guard<std::mutex> lk(mtx_);
    cascade_.set_gains(g);
}

CascadeGains SimSession::cascade_gains() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return cascade_.gains();
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

    // Actuator channels: rebuild from the as-built params so repeated resets do
    // not compound the scales.  set_params() rather than initialize() -- the
    // memory is cleared by actuator_.reset() in reset() right after this.
    ActuatorParams ap = base_actuator_;
    if (rand_.steer_delay_scale > 0.0) ap.steer.ch.dead_time_s *= rand_.steer_delay_scale;
    if (rand_.steer_tau_scale   > 0.0) ap.steer.ch.tau_s       *= rand_.steer_tau_scale;
    if (rand_.rate_limit_scale  > 0.0) {
        ap.steer.ch.rate_limit *= rand_.rate_limit_scale;
        ap.throttle.rate_limit *= rand_.rate_limit_scale;
        ap.brake.ch.rate_limit *= rand_.rate_limit_scale;
    }
    ap.brake.ch.dead_zone += rand_.brake_deadzone_add;
    actuator_.set_params(ap);
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
    w_drive_ = w_brake_ = w0_;
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
        // Acceleration command: the cascade the RL accel action goes through
        // (LongAxController, this session's gains) turns ax_target into
        // pedals; the pedals become wheel torque on the linear references and
        // the default shares, then follow the CmdL1 path below.
        if (const CmdL5* l5 = std::get_if<CmdL5>(&cmd)) {
            const CmdL4 pedal = cascade_.to_l4(cmd, meas_state_, dyn_->ax_body_est(), dt);
            CmdL1 c;
            c.steer_angle_wheel = l5->steer_angle_wheel;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                c.motor_torque[i] = pedal.throttle * drive_ref_ * w0_[i];
                c.brake_torque[i] = pedal.brake * brake_ref_ * w0_[i];
            }
            cmd = ControlInput{c};
        }
        CmdL4 steer_desired{};
        steer_desired.steer_angle_wheel = steer_from_input(cmd);

        // Longitudinal channel: total drive / brake wheel torque -> normalised
        // throttle / brake -> actuator -> redistributed with the commanded
        // per-wheel shares.  Off (and bit-identical) unless a channel is set.
        const CmdL1* l1 = std::get_if<CmdL1>(&cmd);
        const bool long_on = l1 != nullptr && drive_ref_ > 0.0 && brake_ref_ > 0.0 &&
                             longitudinal_engaged(actuator_.params());
        if (long_on) {
            double drive_sum = 0.0, brake_sum = 0.0;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                drive_sum += std::max(l1->motor_torque[i], 0.0);
                brake_sum += l1->brake_torque[i];
            }
            if (drive_sum > 1e-9)
                for (int i = 0; i < NUM_WHEELS; ++i)
                    w_drive_[i] = std::max(l1->motor_torque[i], 0.0) / drive_sum;
            if (brake_sum > 1e-9)
                for (int i = 0; i < NUM_WHEELS; ++i)
                    w_brake_[i] = l1->brake_torque[i] / brake_sum;
            steer_desired.throttle = drive_sum / drive_ref_;
            steer_desired.brake    = brake_sum / brake_ref_;
        }

        const CmdL4 steer_realized = actuator_.apply(steer_desired, speed, dt);
        ControlInput dyn_cmd = with_steer_angle(cmd, steer_realized.steer_angle_wheel);
        if (long_on) {
            CmdL1 c = *l1;
            c.steer_angle_wheel = steer_realized.steer_angle_wheel;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                // Negative motor torque (regen) is not a pedal: it passes through.
                const double regen = std::min(l1->motor_torque[i], 0.0);
                c.motor_torque[i] = regen + steer_realized.throttle * drive_ref_ * w_drive_[i];
                c.brake_torque[i] = steer_realized.brake * brake_ref_ * w_brake_[i];
            }
            dyn_cmd = ControlInput{c};
        }

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
            throttle_applied_ = long_on ? steer_realized.throttle : 0.0;
            brake_applied_    = long_on ? steer_realized.brake : 0.0;
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

std::unique_ptr<IVehicleDynamics> create_dynamics_for_level(const std::string& level) {
    if (level == "K" || level == "L0") return create_kinematic();
    if (level == "L1") return create_bicycle();
    if (level == "L2") return create_seven_dof();
    if (level == "L3") return create_fourteen_dof();
    if (level == "L4") return create_fourteen_dof_kinematic();
    if (level == "L5") return create_stunt_dof();
    throw std::invalid_argument(
        "unknown dynamics level \"" + level +
        "\"; expected one of K, L0, L1, L2, L3, L4, L5 (case-sensitive)");
}

std::unique_ptr<SimSession> make_direct_control_session(
    const VehicleParams& vp,
    const TireSetup& ts,
    const SolverParams& sp,
    const DirectControlSessionOptions& opts) {
    SimConfig cfg;
    cfg.session_kind = SessionKind::DirectControl;
    cfg.nominal_dt = opts.nominal_dt;
    cfg.actuator = opts.actuator;
    cfg.drive_torque_ref = opts.drive_torque_ref;
    cfg.brake_torque_ref = opts.brake_torque_ref;
    apply_vehicle_steer_deadtime(cfg, vp);
    VehicleParams vp_dc = vp;
    vp_dc.plant_path = true;
    auto dyn = create_dynamics_for_level(opts.level);
    auto ground = make_friction_ground(opts.friction);
    return std::make_unique<SimSession>(
        std::move(dyn), std::move(ground), vp_dc, ts, sp, cfg);
}

}  // namespace vdsim
