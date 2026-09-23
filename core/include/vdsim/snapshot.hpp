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
// Covered: State, session diagnostics, sim_time, the latched CmdL4 or CmdL5,
// the actuator, the sensor delay line, the sensor noise model, the dynamics
// tire transients and the CascadeController integrators (LongVx, LongAx,
// pure-pursuit index, yaw-rate PI) that L5+ ladder commands run through.
// NOT covered: a latched CmdL1-L3 / CmdL6-L8 / CmdSplit (restored as the
// default CmdL4) and a non-identity ECU/CAN network buffer.  Both are
// documented rather than silently approximated.
//
// Format version: the blob opens with {kFormatMagic, kFormatVersion}.  A blob
// written by another layout is refused with the two versions named, never
// read field by field into the wrong slots.  Any change to the write order
// in SimSession::snapshot() or a component's save_aux() must bump
// kFormatVersion.  Version 1 is the unversioned layout before the header.
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

// A headerless (v1) blob starts with a State position; no physical state
// takes this value, so v1 is told apart from a versioned blob by slot 0.
constexpr double kFormatMagic   = -1.0e300;
constexpr int    kFormatVersion = 2;

/// @brief Write the format header {magic, version} at the start of a blob.
/// @param v  Blob being written; must be empty.
void put_header(std::vector<double>& v);

/// @brief Read and check the format header; advances p past it.
/// @param v  Blob being read.
/// @param p  Read cursor, expected 0.
/// @throws std::runtime_error naming the blob's format version and the
///         version this build reads when they differ (a headerless blob is
///         reported as format v1).
void check_header(const std::vector<double>& v, std::size_t& p);

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
