#include "vdsim/vec_session.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace vdsim {
namespace {

// Field groups: <100 = scalar (index unused), >=100 = per-wheel (index 0..3,
// FL FR RL RR).  Kept as ints so the hot path is a plain switch.
enum : int {
    F_X = 0, F_Y, F_Z, F_YAW, F_ROLL, F_PITCH,
    F_VX, F_VY, F_VZ, F_SPEED, F_BETA,
    F_YAW_RATE, F_ROLL_RATE, F_PITCH_RATE,
    F_AX, F_AY, F_SIM_TIME,
    F_STEER_APPLIED, F_THROTTLE_APPLIED, F_BRAKE_APPLIED,
    F_RACK_TRAVEL, F_RACK_VELOCITY,
    F_WHEEL_SPIN = 100, F_SLIP_RATIO, F_SLIP_ANGLE, F_FZ,
    F_SUSP_COMP, F_SUSP_VEL, F_WHEEL_MU, F_TIRE_FX, F_TIRE_FY,
};

const std::map<std::string, int>& field_table() {
    static const std::map<std::string, int> t = {
        {"x", F_X}, {"y", F_Y}, {"z", F_Z},
        {"yaw", F_YAW}, {"roll", F_ROLL}, {"pitch", F_PITCH},
        {"vx", F_VX}, {"vy", F_VY}, {"vz", F_VZ},
        {"speed", F_SPEED}, {"beta", F_BETA},
        {"yaw_rate", F_YAW_RATE}, {"roll_rate", F_ROLL_RATE},
        {"pitch_rate", F_PITCH_RATE},
        {"ax", F_AX}, {"ay", F_AY}, {"sim_time", F_SIM_TIME},
        {"steer_applied", F_STEER_APPLIED},
        {"throttle_applied", F_THROTTLE_APPLIED},
        {"brake_applied", F_BRAKE_APPLIED},
        {"rack_travel", F_RACK_TRAVEL}, {"rack_velocity", F_RACK_VELOCITY},
        {"wheel_spin", F_WHEEL_SPIN}, {"slip_ratio", F_SLIP_RATIO},
        {"slip_angle", F_SLIP_ANGLE}, {"fz", F_FZ},
        {"susp_compression", F_SUSP_COMP}, {"susp_velocity", F_SUSP_VEL},
        {"wheel_mu", F_WHEEL_MU}, {"tire_fx", F_TIRE_FX}, {"tire_fy", F_TIRE_FY},
    };
    return t;
}

bool finite3(const Vec3& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

}  // namespace

std::vector<std::string> obs_field_names() {
    std::vector<std::string> out;
    for (const auto& kv : field_table()) out.push_back(kv.first);
    return out;
}

std::vector<ObsField> parse_obs_fields(const std::vector<std::string>& names) {
    std::vector<ObsField> out;
    for (const std::string& raw : names) {
        std::string name = raw;
        int wheel = -1;
        const std::size_t dot = raw.rfind('.');
        if (dot != std::string::npos && raw.size() == dot + 2) {
            const char c = raw[dot + 1];
            if (c >= '0' && c <= '3') {
                name  = raw.substr(0, dot);
                wheel = c - '0';
            }
        }
        const auto it = field_table().find(name);
        if (it == field_table().end())
            throw std::invalid_argument("unknown observation field: " + raw);
        if (it->second < 100) {
            if (wheel >= 0)
                throw std::invalid_argument("field is not per-wheel: " + raw);
            out.push_back(ObsField{it->second, 0});
        } else if (wheel >= 0) {
            out.push_back(ObsField{it->second, wheel});
        } else {                       // bare array name -> all four wheels
            for (int w = 0; w < NUM_WHEELS; ++w)
                out.push_back(ObsField{it->second, w});
        }
    }
    return out;
}

VecSession::VecSession(std::vector<std::unique_ptr<SimSession>> sessions,
                       int threads)
    : sessions_(std::move(sessions)) {
    if (sessions_.empty())
        throw std::invalid_argument("VecSession: needs at least one session");
    for (const auto& s : sessions_)
        if (!s) throw std::invalid_argument("VecSession: null session");

    int hw = static_cast<int>(std::thread::hardware_concurrency());
    if (hw <= 0) hw = 1;
    int n = (threads > 0) ? threads : hw;
    n = std::min<int>(n, static_cast<int>(sessions_.size()));
    if (n < 1) n = 1;

    // The calling thread drains too, so spawn n-1 workers.
    workers_.reserve(static_cast<std::size_t>(n - 1));
    for (int i = 0; i < n - 1; ++i)
        workers_.emplace_back([this] { worker_loop(); });
}

VecSession::~VecSession() {
    {
        std::lock_guard<std::mutex> lk(m_);
        stop_ = true;
    }
    cv_.notify_all();
    for (auto& t : workers_)
        if (t.joinable()) t.join();
}

SimSession& VecSession::at(std::size_t i) {
    if (i >= sessions_.size())
        throw std::out_of_range("VecSession: index out of range");
    return *sessions_[i];
}

// ---------------- worker pool ----------------

void VecSession::worker_loop() {
    std::uint64_t seen = 0;
    for (;;) {
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait(lk, [&] { return stop_ || gen_ != seen; });
            if (stop_) return;
            seen = gen_;
        }
        drain();
        if (done_.fetch_add(1, std::memory_order_acq_rel) + 1 ==
            static_cast<int>(workers_.size())) {
            std::lock_guard<std::mutex> lk(m_);
            finished_ = true;
            cv_done_.notify_all();
        }
    }
}

void VecSession::drain() {
    const std::size_t n = sessions_.size();
    for (;;) {
        const std::size_t i = next_.fetch_add(1, std::memory_order_relaxed);
        if (i >= n) return;
        try {
            job_(i);
        } catch (...) {
            std::lock_guard<std::mutex> lk(err_m_);
            if (!err_) err_ = std::current_exception();
        }
    }
}

void VecSession::run_parallel(std::function<void(std::size_t)> fn) {
    job_ = std::move(fn);
    next_.store(0, std::memory_order_relaxed);
    err_ = nullptr;

    if (workers_.empty()) {          // single-threaded: no handshake at all
        drain();
    } else {
        {
            std::lock_guard<std::mutex> lk(m_);
            done_.store(0, std::memory_order_relaxed);
            finished_ = false;
            ++gen_;
        }
        cv_.notify_all();
        drain();                     // the caller is a worker too
        std::unique_lock<std::mutex> lk(m_);
        cv_done_.wait(lk, [&] { return finished_; });
    }

    if (err_) {
        std::exception_ptr e = err_;
        err_ = nullptr;
        std::rethrow_exception(e);
    }
}

// ---------------- command / reset ----------------

void VecSession::set_inputs(const std::vector<CmdL4>& u) {
    if (u.size() != sessions_.size() && u.size() != 1)
        throw std::invalid_argument("VecSession.set_inputs: expected 1 or size() commands");
    const bool bcast = (u.size() == 1);
    for (std::size_t i = 0; i < sessions_.size(); ++i)
        sessions_[i]->set_input(bcast ? u[0] : u[i]);
}

void VecSession::set_input_all(const CmdL4& u) {
    for (auto& s : sessions_) s->set_input(u);
}

void VecSession::reset_all(const std::vector<State>& s0, bool settle) {
    if (s0.size() != sessions_.size() && s0.size() != 1)
        throw std::invalid_argument("VecSession.reset_all: expected 1 or size() states");
    const bool bcast = (s0.size() == 1);
    for (std::size_t i = 0; i < sessions_.size(); ++i) {
        const State& s = bcast ? s0[0] : s0[i];
        if (settle) sessions_[i]->reset_settled(s);
        else        sessions_[i]->reset(s);
    }
}

void VecSession::reset_subset(const std::vector<std::size_t>& idx,
                              const std::vector<State>& s0, bool settle) {
    if (s0.size() != idx.size() && s0.size() != 1)
        throw std::invalid_argument("VecSession.reset_subset: expected 1 or len(idx) states");
    const bool bcast = (s0.size() == 1);
    for (std::size_t k = 0; k < idx.size(); ++k) {
        if (idx[k] >= sessions_.size())
            throw std::out_of_range("VecSession.reset_subset: index out of range");
        const State& s = bcast ? s0[0] : s0[k];
        if (settle) sessions_[idx[k]]->reset_settled(s);
        else        sessions_[idx[k]]->reset(s);
    }
}

void VecSession::set_randomizations(const std::vector<DomainRandomization>& d) {
    if (d.size() != sessions_.size() && d.size() != 1)
        throw std::invalid_argument("VecSession.set_randomizations: expected 1 or size() specs");
    const bool bcast = (d.size() == 1);
    for (std::size_t i = 0; i < sessions_.size(); ++i)
        sessions_[i]->set_randomization(bcast ? d[0] : d[i]);
}

void VecSession::set_seeds(const std::vector<unsigned>& seeds) {
    if (seeds.size() != sessions_.size() && seeds.size() != 1)
        throw std::invalid_argument("VecSession.set_seeds: expected 1 or size() seeds");
    const bool bcast = (seeds.size() == 1);
    for (std::size_t i = 0; i < sessions_.size(); ++i)
        sessions_[i]->set_seed(bcast ? seeds[0] + static_cast<unsigned>(i) : seeds[i]);
}

// ---------------- observation / termination ----------------

void VecSession::set_obs_fields(const std::vector<std::string>& names) {
    obs_fields_ = parse_obs_fields(names);
}

void VecSession::write_obs_row(const SimSession& s, float* row) const {
    const SimOutput o = s.output();
    const State& st = o.state;
    for (std::size_t k = 0; k < obs_fields_.size(); ++k) {
        const ObsField f = obs_fields_[k];
        const int w = f.index;
        double v = 0.0;
        switch (f.group) {
            case F_X:  v = st.position.x(); break;
            case F_Y:  v = st.position.y(); break;
            case F_Z:  v = st.position.z(); break;
            case F_YAW:   v = st.yaw();  break;
            // L2 carries roll/pitch as a quasi-static report, not in the
            // quaternion, so read the session's value (valid on every level).
            case F_ROLL:  v = o.roll;  break;
            case F_PITCH: v = o.pitch; break;
            case F_VX: v = st.vx(); break;
            case F_VY: v = st.vy(); break;
            case F_VZ: v = st.vz(); break;
            case F_SPEED: v = st.speed_xy(); break;
            case F_BETA:  v = st.beta();     break;
            case F_YAW_RATE:   v = st.yaw_rate();   break;
            case F_ROLL_RATE:  v = st.roll_rate();  break;
            case F_PITCH_RATE: v = st.pitch_rate(); break;
            case F_AX: v = o.ax; break;
            case F_AY: v = o.ay; break;
            case F_SIM_TIME: v = o.sim_time; break;
            case F_STEER_APPLIED:    v = o.steer_applied;    break;
            case F_THROTTLE_APPLIED: v = o.throttle_applied; break;
            case F_BRAKE_APPLIED:    v = o.brake_applied;    break;
            case F_RACK_TRAVEL:   v = st.rack_travel;   break;
            case F_RACK_VELOCITY: v = st.rack_velocity; break;
            case F_WHEEL_SPIN: v = st.wheel_spin[w];       break;
            case F_SLIP_RATIO: v = o.slip_ratio[w];        break;
            case F_SLIP_ANGLE: v = o.slip_angle[w];        break;
            case F_FZ:         v = o.Fz[w];                break;
            case F_SUSP_COMP:  v = st.susp_compression[w]; break;
            case F_SUSP_VEL:   v = st.susp_velocity[w];    break;
            case F_WHEEL_MU:   v = o.wheel_mu[w];          break;
            case F_TIRE_FX:    v = o.tire_forces_wheel[w].x(); break;
            case F_TIRE_FY:    v = o.tire_forces_wheel[w].y(); break;
            default: v = 0.0; break;
        }
        row[k] = static_cast<float>(v);
    }
}

std::int32_t VecSession::check_term(const SimSession& s) const {
    const SimOutput o = s.output();
    const State& st = o.state;
    if (term_.check_nan) {
        bool ok = finite3(st.position) && finite3(st.velocity) &&
                  finite3(st.angular_velocity);
        for (int w = 0; w < NUM_WHEELS && ok; ++w)
            ok = std::isfinite(st.wheel_spin[w]);
        if (!ok) return TERM_NAN_STATE;
    }
    if (term_.max_roll > 0.0 && std::abs(o.roll) > term_.max_roll)
        return TERM_ROLLOVER;
    if (term_.max_lateral > 0.0 &&
        std::abs(st.position.y() - term_.lane_y) > term_.max_lateral)
        return TERM_OFF_TRACK;
    if (term_.max_beta > 0.0 && std::abs(st.beta()) > term_.max_beta)
        return TERM_SPIN_OUT;
    if (term_.max_yaw_rate > 0.0 && std::abs(st.yaw_rate()) > term_.max_yaw_rate)
        return TERM_SPIN_OUT;
    if (term_.min_speed > 0.0 && st.speed_xy() < term_.min_speed)
        return TERM_STALL;
    if (term_.time_limit_s > 0.0 && o.sim_time >= term_.time_limit_s)
        return TERM_TIME_LIMIT;
    return TERM_NONE;
}

void VecSession::advance(double dt, int repeat, float* obs, std::int32_t* term) {
    if (repeat < 1)
        throw std::invalid_argument("VecSession.advance: repeat must be >= 1");
    const std::size_t dim = obs_fields_.size();
    if (obs && dim == 0)
        throw std::invalid_argument("VecSession.advance: no obs fields declared");
    run_parallel([this, dt, repeat, obs, term, dim](std::size_t i) {
        SimSession& s = *sessions_[i];
        std::int32_t code = TERM_NONE;
        for (int k = 0; k < repeat; ++k) {
            s.tick(dt);
            code = check_term(s);
            if (code != TERM_NONE) break;   // freeze the env at the event
        }
        if (term) term[i] = code;
        if (obs)  write_obs_row(s, obs + i * dim);
    });
}

void VecSession::observe(float* obs) const {
    if (!obs) return;
    if (obs_fields_.empty())
        throw std::invalid_argument("VecSession.observe: no obs fields declared");
    const std::size_t dim = obs_fields_.size();
    for (std::size_t i = 0; i < sessions_.size(); ++i)
        write_obs_row(*sessions_[i], obs + i * dim);
}

void VecSession::tick_all(double dt, int repeat) {
    if (repeat < 1)
        throw std::invalid_argument("VecSession.tick_all: repeat must be >= 1");
    run_parallel([this, dt, repeat](std::size_t i) {
        SimSession& s = *sessions_[i];
        for (int k = 0; k < repeat; ++k) s.tick(dt);
    });
}

std::vector<SessionSnapshot> VecSession::snapshots() const {
    std::vector<SessionSnapshot> out;
    out.reserve(sessions_.size());
    for (const auto& s : sessions_) out.push_back(s->snapshot());
    return out;
}

void VecSession::restore(const std::vector<SessionSnapshot>& s) {
    if (s.size() != sessions_.size() && s.size() != 1)
        throw std::invalid_argument("VecSession.restore: expected 1 or size() snapshots");
    const bool bcast = (s.size() == 1);
    for (std::size_t i = 0; i < sessions_.size(); ++i)
        sessions_[i]->restore(bcast ? s[0] : s[i]);
}

void VecSession::restore_subset(const std::vector<std::size_t>& idx,
                                const std::vector<SessionSnapshot>& s) {
    if (s.size() != idx.size() && s.size() != 1)
        throw std::invalid_argument("VecSession.restore_subset: expected 1 or len(idx) snapshots");
    const bool bcast = (s.size() == 1);
    for (std::size_t k = 0; k < idx.size(); ++k) {
        if (idx[k] >= sessions_.size())
            throw std::out_of_range("VecSession.restore_subset: index out of range");
        sessions_[idx[k]]->restore(bcast ? s[0] : s[k]);
    }
}

std::vector<State> VecSession::states() const {
    std::vector<State> out;
    out.reserve(sessions_.size());
    for (const auto& s : sessions_) out.push_back(s->state());
    return out;
}

std::vector<SimOutput> VecSession::outputs() const {
    std::vector<SimOutput> out;
    out.reserve(sessions_.size());
    for (const auto& s : sessions_) out.push_back(s->output());
    return out;
}

}  // namespace vdsim
