#pragma once
// N independent SimSessions advanced in parallel by a persistent worker pool,
// plus the two things an RL rollout needs in the same pass: a flat float32
// observation buffer and per-env termination codes.
//
// Each SimSession owns its dynamics, contact provider, actuator, sensor and
// controller state, so sessions share no mutable data: ticking session i on
// thread t is safe as long as no two threads touch the same session.  This
// class owns the sessions and enforces that, and keeps the threads alive
// across steps (an RL rollout does ~1e6 barrier syncs; thread creation per
// step would dominate).
//
// Intended caller: the Python RL vector environment, which releases the GIL
// around advance()/tick_all() so the pool actually scales with cores.
#include <cstddef>
#include <cstdint>
#include <array>
#include <atomic>
#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "vdsim/control.hpp"
#include "vdsim/sim_session.hpp"
#include "vdsim/snapshot.hpp"
#include "vdsim/state.hpp"

namespace vdsim {

// ---- R4: observation field spec -------------------------------------------
// A field is resolved once from its name, so the hot path is a switch on an
// int instead of a string compare.  Array fields ("slip_ratio") expand to the
// four wheels in FL, FR, RL, RR order; "slip_ratio.2" picks one wheel.
struct ObsField {
    int group {0};
    int index {0};
};

// Throws std::invalid_argument on an unknown name.
std::vector<ObsField> parse_obs_fields(const std::vector<std::string>& names);
// Names accepted by parse_obs_fields(), for error messages and docs.
std::vector<std::string> obs_field_names();

// ---- R3: termination spec + reason codes ----------------------------------
enum TermCode : std::int32_t {
    TERM_NONE       = 0,
    TERM_OFF_TRACK  = 1,   // |y - lane_y| > max_lateral
    TERM_ROLLOVER   = 2,   // |roll| > max_roll
    TERM_SPIN_OUT   = 3,   // |beta| > max_beta or |yaw_rate| > max_yaw_rate
    TERM_NAN_STATE  = 4,   // non-finite position / velocity / wheel spin
    TERM_TIME_LIMIT = 5,   // sim_time >= time_limit_s (truncation, not failure)
    TERM_STALL      = 6,   // vx < min_speed
};

struct TermSpec {
    double lane_y       {0.0};
    double max_lateral  {-1.0};   // <0 = off
    double max_roll     {-1.0};   // [rad], <0 = off
    double max_beta     {-1.0};   // [rad], <0 = off
    double max_yaw_rate {-1.0};   // [rad/s], <0 = off
    double time_limit_s {-1.0};   // [s], <0 = off
    double min_speed    {-1.0};   // [m/s], <0 = off
    bool   check_nan    {true};
};

class VecSession {
public:
    // threads <= 0 -> hardware_concurrency(), capped at the session count.
    // threads == 1 -> run on the calling thread (no workers spawned).
    explicit VecSession(std::vector<std::unique_ptr<SimSession>> sessions,
                        int threads = 0);
    ~VecSession();

    VecSession(const VecSession&)            = delete;
    VecSession& operator=(const VecSession&) = delete;

    std::size_t size()    const { return sessions_.size(); }
    // Workers + the calling thread, which also drains the queue.
    int         threads() const { return static_cast<int>(workers_.size()) + 1; }

    SimSession& at(std::size_t i);

    // Per-env command latch.  u.size() must be 1 (broadcast) or size().
    void set_inputs(const std::vector<CmdL4>& u);
    void set_input_all(const CmdL4& u);

    // Per-env reset.  s0.size() must be 1 (broadcast) or size().
    // settle = R7: drop each spawn pose onto that env's ground first (needed
    // for a respawn on non-flat terrain, a no-op cost on flat ground).
    void reset_all(const std::vector<State>& s0, bool settle = false);
    // Reset only the envs whose index is listed (RL auto-reset after done).
    void reset_subset(const std::vector<std::size_t>& idx,
                      const std::vector<State>& s0, bool settle = false);

    // R5: per-env domain randomization (size 1 broadcasts).  Each spec takes
    // effect at that env's next reset; mu_scale applies from the next tick.
    void set_randomizations(const std::vector<DomainRandomization>& d);

    // R6: one independent stochastic stream per env (size 1 = seed, seed+1, ...).
    void set_seeds(const std::vector<unsigned>& seeds);

    // Advance every session by `repeat` ticks of `dt`.  repeat > 1 is the RL
    // action-hold (frame skip): it stays inside the worker, so one barrier
    // covers the whole control interval instead of one per physics tick.
    void tick_all(double dt, int repeat = 1);

    // Observation layout (R4).  Empty = observations disabled.
    void set_obs_fields(const std::vector<std::string>& names);
    std::size_t obs_dim() const { return obs_fields_.size(); }

    // Termination spec (R3).  Default = everything off.
    void set_term_spec(const TermSpec& t) { term_ = t; }
    const TermSpec& term_spec() const { return term_; }

    // One parallel pass: tick `repeat` times (stopping an env early once it
    // terminates), then write that env's observation row and reason code.
    // obs may be null (skip); term may be null (skip).  Both are caller-owned
    // buffers of size N*obs_dim() and N.
    void advance(double dt, int repeat, float* obs, std::int32_t* term);

    // Fill the observation buffer without stepping (used right after reset).
    void observe(float* obs) const;

    // R8: per-env snapshot / restore (branching rollouts, MCTS-style search).
    std::vector<SessionSnapshot> snapshots() const;
    void restore(const std::vector<SessionSnapshot>& s);
    void restore_subset(const std::vector<std::size_t>& idx,
                        const std::vector<SessionSnapshot>& s);

    std::vector<State>     states()  const;
    std::vector<SimOutput> outputs() const;

private:
    void worker_loop();
    void drain();
    void run_parallel(std::function<void(std::size_t)> fn);
    void write_obs_row(const SimSession& s, float* row) const;
    std::int32_t check_term(const SimSession& s) const;

    std::vector<std::unique_ptr<SimSession>> sessions_;
    std::vector<std::thread>                 workers_;

    std::vector<ObsField> obs_fields_;
    TermSpec              term_ {};

    // --- pool handshake (generation counter + work-stealing index) ---
    std::mutex              m_;
    std::condition_variable cv_;        // wakes workers on a new generation
    std::condition_variable cv_done_;   // wakes the caller when workers drained
    std::uint64_t           gen_      {0};
    bool                    stop_     {false};
    bool                    finished_ {false};
    std::atomic<int>        done_     {0};
    std::atomic<std::size_t> next_    {0};

    std::function<void(std::size_t)> job_;

    std::mutex         err_m_;
    std::exception_ptr err_;
};

}  // namespace vdsim
