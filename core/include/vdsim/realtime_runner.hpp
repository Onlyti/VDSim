// RealTimeRunner — paces a SimSession to the wall clock on its own thread.
//
// This is the thin "real-time / free-run" wiring on top of the mode-agnostic
// SimSession kernel. The kernel itself has no clock; experiment / batch modes
// drive tick() directly and never touch this class.
//
// A producer thread (e.g. a UDP receiver) calls session.set_input(); this runner
// ticks at fixed dt and, if no fresh command arrived within cmd_timeout_s,
// latches a fail-safe command (ECU-style actuator watchdog).
#pragma once

#include <atomic>
#include <chrono>
#include <thread>

#include "vdsim/control.hpp"
#include "vdsim/sim_session.hpp"

namespace vdsim {

class RealTimeRunner {
public:
    struct Config {
        double dt            {0.005};   // fixed sim step [s]
        double cmd_timeout_s {0.1};     // no input for this long -> fail-safe (<=0 disables)
        CmdL4  failsafe      {0.0, 0.3, 0.0, 1, false};  // throttle 0, moderate brake
    };

    RealTimeRunner(SimSession& session, const Config& cfg)
        : session_(session), cfg_(cfg) {}
    ~RealTimeRunner() { stop(); }

    RealTimeRunner(const RealTimeRunner&) = delete;
    RealTimeRunner& operator=(const RealTimeRunner&) = delete;

    void start() {
        if (running_.exchange(true)) return;
        thread_ = std::thread([this] { loop(); });
    }

    void stop() {
        if (!running_.exchange(false)) return;
        if (thread_.joinable()) thread_.join();
    }

    bool running() const { return running_.load(); }

private:
    void loop() {
        using clock = std::chrono::steady_clock;
        const auto step = std::chrono::duration<double>(cfg_.dt);
        auto next = clock::now();
        while (running_.load()) {
            if (cfg_.cmd_timeout_s > 0.0 &&
                session_.seconds_since_last_input() > cfg_.cmd_timeout_s) {
                session_.set_input(cfg_.failsafe);   // actuator watchdog
            }
            session_.tick(cfg_.dt);
            next += std::chrono::duration_cast<clock::duration>(step);
            std::this_thread::sleep_until(next);
        }
    }

    SimSession&       session_;
    Config            cfg_;
    std::atomic<bool> running_ {false};
    std::thread       thread_;
};

}  // namespace vdsim
