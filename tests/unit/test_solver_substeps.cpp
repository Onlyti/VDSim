// The substep clamp must be visible (N5).
//
// solver_substeps() returns N = clamp(ceil(dt/max_substep_dt), 1, max_substeps).
// When the ceil term wins, the integrator runs with h = dt/N > max_substep_dt --
// the caller asked for a resolution it is not getting. That used to be silent;
// these tests pin the counter that now makes it observable.
#include <gtest/gtest.h>

#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

#include <cmath>
#include <limits>
#include <memory>

using vdsim::SolverParams;
using vdsim::solver_substeps;
using vdsim::solver_substep_clamp_count;
using vdsim::reset_solver_substep_clamp_count;

namespace {

SolverParams sp(double h_max, int cap) {
    SolverParams s;
    s.max_substep_dt = h_max;
    s.max_substeps   = cap;
    return s;
}

}  // namespace

TEST(SolverSubsteps, HonoursTheRequestWhenItFits) {
    reset_solver_substep_clamp_count();
    // 5 ms outer step at 1 ms resolution needs 5 substeps, cap is 10.
    EXPECT_EQ(solver_substeps(sp(1e-3, 10), 5e-3), 5);
    EXPECT_EQ(solver_substep_clamp_count(), 0u);
    // Exactly at the cap is still not a clamp.
    EXPECT_EQ(solver_substeps(sp(1e-3, 10), 10e-3), 10);
    EXPECT_EQ(solver_substep_clamp_count(), 0u);
}

TEST(SolverSubsteps, CountsTheClampAndReportsTheCoarserStep) {
    reset_solver_substep_clamp_count();
    // An RL control interval (20 ms) against the default cap of 10: 20 substeps
    // are needed, 10 are allowed, so h = 2 ms -- twice what was asked for.
    const SolverParams p = sp(1e-3, 10);
    const int n = solver_substeps(p, 20e-3);
    EXPECT_EQ(n, 10);
    EXPECT_GT(20e-3 / n, p.max_substep_dt);
    EXPECT_EQ(solver_substep_clamp_count(), 1u);
    // 50 ms is worse still, and each occurrence counts.
    EXPECT_EQ(solver_substeps(sp(1e-3, 10), 50e-3), 10);
    EXPECT_EQ(solver_substep_clamp_count(), 2u);
}

TEST(SolverSubsteps, ResetZeroesTheCounter) {
    reset_solver_substep_clamp_count();
    solver_substeps(sp(1e-4, 4), 20e-3);
    EXPECT_GT(solver_substep_clamp_count(), 0u);
    reset_solver_substep_clamp_count();
    EXPECT_EQ(solver_substep_clamp_count(), 0u);
}

TEST(SolverSubsteps, DegenerateInputsAreSafe) {
    reset_solver_substep_clamp_count();
    EXPECT_EQ(solver_substeps(sp(1e-3, 10), 0.0), 1);
    EXPECT_EQ(solver_substeps(sp(1e-3, 10), -1.0), 1);
    EXPECT_EQ(solver_substeps(sp(1e-3, 10),
                              std::numeric_limits<double>::quiet_NaN()), 1);
    EXPECT_EQ(solver_substep_clamp_count(), 0u);
    // A zero / negative resolution must not divide by zero or overflow the int
    // cast; it falls back to 1 us and then clamps like any other request.
    EXPECT_EQ(solver_substeps(sp(0.0, 8), 5e-3), 8);
    EXPECT_EQ(solver_substeps(sp(-1.0, 8), 5e-3), 8);
    EXPECT_EQ(solver_substep_clamp_count(), 2u);
}

TEST(SolverSubsteps, SessionAtAnRlControlRateReportsTheClamp) {
    // The original symptom, end to end: an L2 session ticked at a 20 ms RL
    // control interval with the shipped default cap of 10.
    vdsim::VehicleParams vp;
    vdsim::TireParams    tp;
    vdsim::SimConfig     cfg;

    auto make = [&](const SolverParams& solver) {
        auto s = std::make_unique<vdsim::SimSession>(
            vdsim::create_seven_dof(), vdsim::create_flat_ground(0.0, 1.0),
            vp, tp, solver, cfg);
        vdsim::State s0;
        s0.velocity = {20.0, 0.0, 0.0};
        s->reset(s0);
        return s;
    };

    auto clamped = make(sp(1e-3, 10));          // shipped default cap
    reset_solver_substep_clamp_count();
    clamped->tick(0.02);
    EXPECT_GT(solver_substep_clamp_count(), 0u)
        << "a session that cannot meet max_substep_dt must say so";

    // Raising the cap to what the ratio needs makes the run quiet again.
    auto ok = make(sp(1e-3, 32));
    reset_solver_substep_clamp_count();
    ok->tick(0.02);
    EXPECT_EQ(solver_substep_clamp_count(), 0u);
}
