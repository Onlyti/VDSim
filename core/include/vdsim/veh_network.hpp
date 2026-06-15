#pragma once

// IVehNetwork — Vehicle Network Module.
//
// Models the physical effects of the in-vehicle communication network
// (CAN bus, Automotive Ethernet) on control command signals.
//
// Physical effects modelled:
//   - Stochastic deadtime: CAN arbitration + ECU computation delay ~ N(mean, std)
//   - Packet drop:         Bernoulli loss at rate p_drop
//
// NOT modelled here (belong elsewhere):
//   - Quantisation: negligible at 16-bit+ resolution
//   - Signal lag / rate-limit / saturation: → Subsystem (physical actuator)
//
// The module is purely computational — no mass, no inertia, no force output.

#include "vdsim/control.hpp"

#include <array>
#include <cmath>
#include <random>

namespace vdsim {

// ─── Parameters ────────────────────────────────────────────────────────────
struct VehNetworkParams {
    double deadtime_mean {0.005};    // [s] mean ECU + CAN latency
    double deadtime_std  {0.0};      // [s] std-dev; 0 = deterministic
    double drop_rate     {0.0};      // [0,1] packet loss probability
    enum class OnDrop { HoldLast, Zero, Failsafe } on_drop {OnDrop::HoldLast};
    unsigned seed        {42};
};

// ─── Interface ─────────────────────────────────────────────────────────────
struct IVehNetwork {
    // Returns the (potentially delayed / dropped) command to send to subsystems.
    virtual ControlInput apply(const ControlInput& cmd, double dt) = 0;
    virtual void reset() {}
    virtual ~IVehNetwork() = default;
};

// ─── DefaultVehNetwork ─────────────────────────────────────────────────────
// Stochastic deadtime (Gaussian-jittered ring buffer) + Bernoulli drop.
class DefaultVehNetwork final : public IVehNetwork {
public:
    explicit DefaultVehNetwork(const VehNetworkParams& p = {})
        : p_(p), rng_(p.seed), normal_(p.deadtime_mean, std::max(p.deadtime_std, 0.0)),
          uniform_(0.0, 1.0) {}

    ControlInput apply(const ControlInput& cmd, double dt) override {
        // 1. Packet drop check
        if (p_.drop_rate > 0.0 && uniform_(rng_) < p_.drop_rate) {
            return on_drop(cmd);
        }

        // 2. Stochastic deadtime
        const double delay = (p_.deadtime_std > 0.0)
            ? std::max(0.0, normal_(rng_))
            : p_.deadtime_mean;

        if (delay <= 0.0 || dt <= 0.0) {
            last_ = cmd; return cmd;
        }

        const int depth = std::max(1, static_cast<int>(std::round(delay / dt)));
        if (static_cast<int>(buf_.size()) != depth)
            buf_.assign(depth, cmd);   // re-size with current cmd as fill

        const ControlInput out = buf_[static_cast<size_t>(idx_)];
        buf_[static_cast<size_t>(idx_)] = cmd;
        idx_ = (idx_ + 1) % depth;
        last_ = out;
        return out;
    }

    void reset() override {
        buf_.clear(); idx_ = 0; last_ = ControlInput{CmdL4{}};
    }

private:
    ControlInput on_drop(const ControlInput& cmd) {
        switch (p_.on_drop) {
            case VehNetworkParams::OnDrop::HoldLast:  return last_;
            case VehNetworkParams::OnDrop::Zero:       return ControlInput{CmdL4{}};
            case VehNetworkParams::OnDrop::Failsafe:   return ControlInput{CmdL4{}};
        }
        return last_;
    }

    VehNetworkParams             p_;
    std::mt19937                 rng_;
    std::normal_distribution<>   normal_;
    std::uniform_real_distribution<> uniform_;
    std::vector<ControlInput>    buf_;
    int                          idx_  {0};
    ControlInput                 last_ {CmdL4{}};
};

// Factory
inline std::unique_ptr<IVehNetwork>
make_default_veh_network(const VehNetworkParams& p = {}) {
    return std::make_unique<DefaultVehNetwork>(p);
}

}  // namespace vdsim
