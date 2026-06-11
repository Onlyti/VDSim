// VDSim user ANTI-ROLL-BAR module template (L3 ride model).
//
// Subclass IAntiRollBar. force(ctx, axle) returns the (left, right) wheel forces the ARB
// applies on one axle, given the axle's left/right deflections and rates. Evaluated every
// RK4 stage; must be memoryless. A linear ARB reacts the roll (left-right difference):
//   axle.defl_left / defl_right   wheel deflections [m]
//   axle.rate_left / rate_right   deflection rates [m/s]

#include <utility>

#include "vdsim/module_plugin.hpp"

using namespace vdsim;

class MyAntiRollBar : public IAntiRollBar {
public:
    std::pair<double, double> force(const SubsystemContext& /*ctx*/, const AxleDefl& a) override {
        const double k = 20000.0;  // roll stiffness at the wheel [N/m]
        const double anti = 0.5 * k * (a.defl_left - a.defl_right);
        return {-anti, +anti};     // opposes the roll deflection
    }
};

VDSIM_REGISTER_ANTIROLLBAR_MODULE(MyAntiRollBar, "my_antirollbar")
