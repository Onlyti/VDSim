// VDSim user SUSPENSION module template (L3 ride model).
//
// Subclass ISuspension. force(ctx, corner) returns the vertical corner force from the
// spring+damper given the current deflection. It is evaluated EVERY RK4 stage and must be
// a memoryless function of the deflection (no persistent state):
//   corner.defl       suspension deflection [m] (your compression-sign convention)
//   corner.defl_rate  deflection rate [m/s]
//   corner.damping_scale  per-corner damper multiplier (e.g. adaptive dampers)

#include "vdsim/module_plugin.hpp"

using namespace vdsim;

class MySuspension : public ISuspension {
public:
    double force(const SubsystemContext& /*ctx*/, const CornerInput& c) override {
        const double k = 30000.0;  // spring rate [N/m]
        const double d = 3000.0;   // damping coefficient [N s/m]
        return -(k * c.defl + d * c.defl_rate * c.damping_scale);
    }
};

VDSIM_REGISTER_SUSPENSION_MODULE(MySuspension, "my_suspension")
