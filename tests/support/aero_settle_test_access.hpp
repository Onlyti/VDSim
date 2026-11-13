// Test hook of the L5 aero settle run (work order 51 B1a). Not part of the API:
// SimSession grants this struct access so a test can force the settle run on a
// car without ride slopes or aero module, and shorten its limit.
#pragma once

#include "vdsim/sim_session.hpp"

namespace vdsim::detail {

struct AeroSettleTestAccess {
    /**
     * @brief Run the settle run at every reset even without slope / module.
     * @param s session
     * @param on true to force it
     */
    static void force(SimSession& s, bool on) { s.aero_settle_force_ = on; }
    /**
     * @brief Settle-run limit [s] (default 8 s, the 48b limit).
     * @param s session
     * @param t limit [s]
     */
    static void limit(SimSession& s, double t) { s.aero_settle_limit_s_ = t; }
};

}  // namespace vdsim::detail
