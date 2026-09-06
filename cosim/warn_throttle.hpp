// Rate limiter for warnings emitted from a per-tick loop.
//
// The realtime server's TX loop runs at the sim rate (200 Hz in the shipped
// scenes), so any warning printed from inside it repeats 200 times a second per
// channel for as long as the condition holds. Header-only, like the rest of
// cosim/, so the policy is unit-testable without linking the server.
#pragma once

#include <cstdint>

namespace vdsim::cosim {

// Minimum gap between two prints from the same call site [s].
constexpr double kWarnThrottlePeriodS = 5.0;

// Per-call-site state for warn_due(). Default-constructed state always lets the
// first warning through, however early in the run it happens.
struct WarnThrottle {
    double   last_s     {-1.0e300};   // clock value of the last print [s]
    uint64_t suppressed {0};          // prints skipped since the run started
};

/// @brief Decide whether a repeating warning may be printed now.
/// @param now Current clock reading [s]; any monotonic source works (the
///        realtime server passes its steady_clock uptime).
/// @param w In/out throttle state for one call site. On a true result @p w
///        .last_s advances to @p now; on a false result @p w.suppressed is
///        incremented so the next printed line can say how much was hidden.
/// @param period_s Minimum gap between prints [s].
/// @return true if the caller should print, false if it should stay silent.
/// @note suppressed is a running total for the whole run, not a since-last-print
///       count: the caller prints it *after* this returns true, so resetting it
///       here would always report zero.
inline bool warn_due(double now, WarnThrottle& w,
                     double period_s = kWarnThrottlePeriodS) {
    if (now - w.last_s < period_s) { ++w.suppressed; return false; }
    w.last_s = now;
    return true;
}

}  // namespace vdsim::cosim
