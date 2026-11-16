#pragma once

// User module fault boundary (work order 51b).
//
// A user module (brake, steering, drivetrain, suspension, anti-roll bar, aero) installed
// with IVehicleDynamics::set_*_module() runs inside the noexcept dynamics step. The model
// wraps it so an exception or a non-finite output field is recorded (first fault only)
// instead of terminating the process; SimSession::tick() then throws
// std::invalid_argument naming the module kind, the module name, the tick and the cause,
// and refuses further ticks until reset() / restore().
//
// The name in that message is the module's class name. Bindings that know a better name
// (the Python class of a Python module) register it before installing the module.

#include <string>

namespace vdsim {

/**
 * @brief Give a module object a display name for fault messages.
 * @param module Address of the module object (the pointer passed to set_*_module).
 * @param name   Display name (e.g. the Python class name).
 *
 * The model reads and drops the entry when the module is installed.
 */
void register_module_name(const void* module, std::string name);

/**
 * @brief Drop a registered display name that was not consumed (module refused).
 * @param module Address used in register_module_name().
 */
void forget_module_name(const void* module);

}  // namespace vdsim
