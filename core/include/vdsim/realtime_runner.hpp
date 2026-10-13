// RealTimeRunner — paces a SimSession to the wall clock on its own thread.
//
// This is the thin "real-time / free-run" wiring on top of the mode-agnostic
// SimSession kernel. The kernel itself has no clock; experiment / batch modes
// drive tick() directly and never touch this class.
//
// A producer thread (e.g. a UDP receiver) calls session.set_input(); this runner
// ticks at fixed dt and, if no fresh command arrived within cmd_timeout_s,
// latches a fail-safe command (ECU-style actuator watchdog).
//
// The loop never touches Python, so from the Python bindings it runs with the
// GIL released: a Python controller thread that blocks or computes does not
// stall the plant.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <string>
#include <thread>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

#include "vdsim/control.hpp"
#include "vdsim/sim_session.hpp"

namespace vdsim {

class RealTimeRunner {
public:
    struct Config {
        double dt            {0.005};   // fixed sim step [s]
        double cmd_timeout_s {0.1};     // no input for this long -> fail-safe (<=0 disables)
        // Any ladder command. The default is the historical L4 brake (throttle 0,
        // moderate brake); a direct-control session (CmdL1) passes an L1 fail-safe.
        ControlInput  failsafe {CmdL4{0.0, 0.3, 0.0, 1, false}};
        std::uint64_t max_ticks {0};    // stop ticking after this many (0 = until stop())
        int           cpu       {-1};   // pin the sim thread to this core (Linux; -1 = inherit)
    };

    // Counters of one start()..finish/stop() run. Read while running for a
    // live view; wall_elapsed_s / sim_elapsed_s are final once finished.
    struct Stats {
        std::uint64_t ticks            {0};    // ticks performed
        std::uint64_t late_ticks       {0};    // ticks that ended after their slot
        std::uint64_t failsafe_latches {0};    // watchdog fail-safe latches
        double        max_lag_s        {0.0};  // worst end-of-tick lateness [s]
        double        wall_elapsed_s   {0.0};  // wall time since start() [s]
        double        sim_elapsed_s    {0.0};  // session time advanced since start() [s]
        bool          finished         {false};// loop has exited (max_ticks or error)
        bool          affinity_applied {false};// cfg.cpu was honoured
        std::string   error;                   // what() of an exception from tick()
    };

    RealTimeRunner(SimSession& session, const Config& cfg)
        : session_(session), cfg_(cfg) {}
    ~RealTimeRunner() { stop(); }

    RealTimeRunner(const RealTimeRunner&) = delete;
    RealTimeRunner& operator=(const RealTimeRunner&) = delete;

    /// @brief Start the pacing thread; no-op if already running.
    void start() {
        if (running_.exchange(true)) return;
        ticks_.store(0);
        late_.store(0);
        failsafe_.store(0);
        max_lag_s_.store(0.0);
        affinity_applied_.store(false);
        finished_.store(false);
        error_.clear();
        sim_t0_    = session_.sim_time();
        wall_t0_   = std::chrono::steady_clock::now();
        wall_end_  = wall_t0_;
        sim_end_   = sim_t0_;
        thread_ = std::thread([this] { loop(); });
    }

    /// @brief Ask the loop to exit and join it. Safe after the loop finished by itself.
    void stop() {
        if (!running_.exchange(false)) return;
        if (thread_.joinable()) thread_.join();
    }

    /// @brief True between start() and stop() (also after the loop finished by itself).
    bool running() const { return running_.load(); }

    /// @brief True once the loop exited on its own (max_ticks reached or tick() threw).
    bool finished() const { return finished_.load(std::memory_order_acquire); }

    /// @brief Ticks performed since start() (live, lock-free).
    std::uint64_t ticks() const { return ticks_.load(); }

    /// @brief Snapshot of the run counters.
    Stats stats() const {
        Stats s;
        s.ticks            = ticks_.load();
        s.late_ticks       = late_.load();
        s.failsafe_latches = failsafe_.load();
        s.max_lag_s        = max_lag_s_.load();
        s.affinity_applied = affinity_applied_.load();
        s.finished         = finished();
        if (s.finished) {
            s.wall_elapsed_s = std::chrono::duration<double>(wall_end_ - wall_t0_).count();
            s.sim_elapsed_s  = sim_end_ - sim_t0_;
            s.error          = error_;
        } else {
            s.wall_elapsed_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - wall_t0_).count();
            s.sim_elapsed_s  = session_.sim_time() - sim_t0_;
        }
        return s;
    }

private:
    /// @brief Pin the calling thread to cfg_.cpu; false if not requested or not supported.
    bool pin_thread() const {
#if defined(__linux__)
        if (cfg_.cpu < 0 || cfg_.cpu >= CPU_SETSIZE) return false;
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(cfg_.cpu, &set);
        return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
#else
        return false;
#endif
    }

    void loop() {
        using clock = std::chrono::steady_clock;
        affinity_applied_.store(pin_thread());
        const auto step = std::chrono::duration<double>(cfg_.dt);
        auto next = wall_t0_;
        try {
            while (running_.load()) {
                if (cfg_.max_ticks > 0 && ticks_.load() >= cfg_.max_ticks) break;
                if (cfg_.cmd_timeout_s > 0.0 &&
                    session_.seconds_since_last_input() > cfg_.cmd_timeout_s) {
                    session_.set_input(cfg_.failsafe);   // actuator watchdog
                    failsafe_.fetch_add(1);
                }
                session_.tick(cfg_.dt);
                ticks_.fetch_add(1);
                next += std::chrono::duration_cast<clock::duration>(step);
                const auto now = clock::now();
                if (now > next) {
                    late_.fetch_add(1);
                    const double lag = std::chrono::duration<double>(now - next).count();
                    if (lag > max_lag_s_.load()) max_lag_s_.store(lag);
                }
                std::this_thread::sleep_until(next);
            }
        } catch (const std::exception& e) {
            error_ = e.what();
        } catch (...) {
            error_ = "unknown exception in SimSession::tick";
        }
        wall_end_ = clock::now();
        sim_end_  = session_.sim_time();
        finished_.store(true, std::memory_order_release);
    }

    SimSession&       session_;
    Config            cfg_;
    std::atomic<bool> running_ {false};
    std::thread       thread_;

    std::atomic<std::uint64_t> ticks_    {0};
    std::atomic<std::uint64_t> late_     {0};
    std::atomic<std::uint64_t> failsafe_ {0};
    std::atomic<double>        max_lag_s_ {0.0};
    std::atomic<bool>          affinity_applied_ {false};
    std::atomic<bool>          finished_ {false};
    // Written by the loop before finished_ is released; read only after acquiring it.
    std::string                error_;
    double                     sim_t0_ {0.0};
    double                     sim_end_ {0.0};
    std::chrono::steady_clock::time_point wall_t0_ {};
    std::chrono::steady_clock::time_point wall_end_ {};
};

}  // namespace vdsim
