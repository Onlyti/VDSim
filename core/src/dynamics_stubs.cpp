// Stubs for higher-fidelity dynamics models (Phase 1 W9-W10 / W11-W12).
// They throw on construction so callers get a clear error if they
// accidentally request an unimplemented level.

#include "vdsim/interfaces.hpp"

#include <stdexcept>

namespace vdsim {

std::unique_ptr<IVehicleDynamics> create_seven_dof() {
    throw std::runtime_error(
        "vdsim::create_seven_dof: L2 7-DOF not yet implemented (planned W9-W10)");
}

std::unique_ptr<IVehicleDynamics> create_fourteen_dof() {
    throw std::runtime_error(
        "vdsim::create_fourteen_dof: L3 14-DOF not yet implemented (planned W11-W12)");
}

}  // namespace vdsim
