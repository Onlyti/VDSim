// VDSim user STEERING module template.
//
// Subclass ISteeringSystem. apply(ctx) maps the hand-wheel command to a road-wheel angle.
//   apply(ctx) -> SteeringOutput{roadwheel_angle [rad], rack_travel [m]} (per RK4 stage).
//   begin_step(ctx, dt) once per step() for step-coherent state (optional).
//   ctx.cmd.handwheel_angle is the driver input; ctx.state for speed-dependent ratios.

#include "vdsim/module_plugin.hpp"

using namespace vdsim;

class MySteering : public ISteeringSystem {
public:
    SteeringOutput apply(const SubsystemContext& ctx) override {
        const double ratio = 15.0;  // hand-wheel : road-wheel
        SteeringOutput o;
        o.roadwheel_angle = ctx.cmd.handwheel_angle / ratio;
        o.rack_travel     = 0.0;
        return o;
    }
};

VDSIM_REGISTER_STEERING_MODULE(MySteering, "my_steering")
