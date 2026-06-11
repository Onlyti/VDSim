// VDSim user DRIVETRAIN module template.
//
// Subclass IDrivetrain. apply(ctx) returns the drive torque delivered to each wheel.
//   apply(ctx) -> DrivetrainOutput{wheel_torque[FL,FR,RL,RR]} in N m (per RK4 stage).
//   begin_step(ctx, dt) once per step() for step-coherent state (e.g. an engine model).
//   Drive torque is applied to the wheel-spin DOF; the tire/road convert it to traction.

#include "vdsim/module_plugin.hpp"

using namespace vdsim;

class MyDrivetrain : public IDrivetrain {
public:
    DrivetrainOutput apply(const SubsystemContext& ctx) override {
        DrivetrainOutput o{};
        const double max_axle_torque = 600.0;            // [N m] at wide-open throttle
        const double axle = ctx.cmd.throttle * max_axle_torque;
        // RWD example: split equally across the rear wheels (an open diff).
        o.wheel_torque[WHEEL_RL] = 0.5 * axle;
        o.wheel_torque[WHEEL_RR] = 0.5 * axle;
        return o;
    }
};

VDSIM_REGISTER_DRIVETRAIN_MODULE(MyDrivetrain, "my_drivetrain")
