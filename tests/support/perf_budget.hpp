#pragma once

/**
 * @file perf_budget.hpp
 * @brief Single source of truth for the wall-clock performance budgets used by
 *        the C++ test suite.
 *
 * A speed assertion measures the machine and the build configuration as much as
 * it measures the code. An unoptimised Debug build runs the same plant 3-6x
 * slower than Release, which says nothing about a performance regression, so the
 * caller that produced the build states the budget that belongs to it.
 *
 * The override is read from the environment variable `VDSIM_PERF_BUDGET_S`
 * (seconds). The same variable name and the same parsing rule are used by the
 * Python suite (`tests/scripts/test_vla_plant.py`); keep the two in step. The
 * default passed by each call site stays the Release contract, so a build that
 * sets nothing keeps full regression-detection strength.
 */

#include <cstdlib>
#include <string>

namespace vdsim {
namespace testing {

/// @brief Name of the environment variable that overrides a speed budget.
inline const char* perf_budget_env_name() { return "VDSIM_PERF_BUDGET_S"; }

/**
 * @brief Wall-clock budget in seconds, taken from the environment if set.
 * @param default_s Budget to use when the variable is unset, empty or unparsable.
 *                  This is the Release contract for the call site.
 * @return The overriding value when it parses to a finite positive number,
 *         @p default_s otherwise.
 *
 * Unset, empty and malformed are all treated as "not stated" rather than as an
 * error: a broken CI expression must not silently disable the assertion, and it
 * must not turn a physics test into a configuration test either.
 */
inline double perf_budget_s(double default_s) {
    const char* raw = std::getenv(perf_budget_env_name());
    if (raw == nullptr || raw[0] == '\0') {
        return default_s;
    }
    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(std::string(raw), &consumed);
        if (consumed == 0 || !(parsed > 0.0)) {
            return default_s;
        }
        return parsed;
    } catch (...) {
        return default_s;
    }
}

/**
 * @brief Same budget expressed in milliseconds.
 * @param default_ms Budget in milliseconds to use when the variable is unset.
 * @return The effective budget in milliseconds.
 */
inline double perf_budget_ms(double default_ms) {
    return perf_budget_s(default_ms / 1000.0) * 1000.0;
}

}  // namespace testing
}  // namespace vdsim
