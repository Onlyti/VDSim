// Stubs for higher-fidelity dynamics models (Phase 1 W9-W10 / W11-W12).
// They throw on construction so callers get a clear error if they
// accidentally request an unimplemented level.

#include "vdsim/interfaces.hpp"

#include <stdexcept>

namespace vdsim {

// L2 (create_seven_dof) provided by core/src/seven_dof_dynamics.cpp.
// L3 (create_fourteen_dof) provided by core/src/fourteen_dof_dynamics.cpp.

}  // namespace vdsim
