#pragma once
// R8: full session snapshot / restore.
//
// An RL rollout needs to branch: save a session, run a candidate action
// sequence, rewind, run another.  State alone is not enough -- the actuator
// carries transport-delay rings, lag and servo memory, the feedback path
// carries a delay line of past States, the sensor model carries bias walks and
// its RNG stream, and the tire carries per-wheel relaxation (belt) transients.
// Restoring only State silently continues from the wrong actuator/tire memory.
//
// The snapshot is an opaque blob: a flat double vector plus the sensor RNG
// stream, written and read in a fixed order by the components themselves.
//
// Covered: State, session diagnostics, sim_time, the latched CmdL4, the
// actuator, the sensor delay line, the sensor noise model and the dynamics
// tire transients.
// NOT covered: the CascadeController integrators (only carry state for L5+
// ladder commands; the RL path latches CmdL4) and a non-identity ECU/CAN
// network buffer.  Both are documented rather than silently approximated.
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "vdsim/state.hpp"

namespace vdsim {

struct SessionSnapshot {
    std::vector<double> d;      // numeric state, written in a fixed order
    std::string         rng;    // sensor RNG stream (std::mt19937 text form)

    std::size_t size() const { return d.size(); }
};

namespace snap {

inline void put(std::vector<double>& v, double x) { v.push_back(x); }

inline double get(const std::vector<double>& v, std::size_t& p) {
    if (p >= v.size())
        throw std::runtime_error("SessionSnapshot: truncated blob");
    return v[p++];
}

void  put_state(std::vector<double>& v, const State& s);
State get_state(const std::vector<double>& v, std::size_t& p);

}  // namespace snap
}  // namespace vdsim
