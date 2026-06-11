// VDSim user BRAKE module template.
//
// Subclass IBrakeSystem, edit the body, keep the registration macro at the bottom.
// Build + contract-check + register via the GUI Module Workshop (or tools/module_workshop.py).
//
// Contract (see theory ch.23 / ch.24):
//   wheel_torque(ctx) -> SIGNED torque per wheel [FL,FR,RL,RR] in N m that OPPOSES wheel
//   spin (returning a positive magnitude would drive the wheel and accelerate the car).
//   begin_step(ctx, dt) runs once per step()  -> put any step-coherent state here.
//   wheel_torque() runs once per RK4 stage     -> treat it as a pure function of ctx.
//   ctx.state (vx(), wheel_spin[i], ...), ctx.cmd (brake, throttle, ...), ctx.Fz[i].

#include <array>

#include "vdsim/module_plugin.hpp"

using namespace vdsim;

class MyBrake : public IBrakeSystem {
public:
    void begin_step(const SubsystemContext& ctx, double /*dt*/) override {
        pedal_ = ctx.cmd.brake;  // one pedal read per step
    }

    std::array<double, NUM_WHEELS> wheel_torque(const SubsystemContext& ctx) override {
        std::array<double, NUM_WHEELS> T{};
        const double max_torque  = 3000.0;   // peak brake torque per wheel [N m]
        const double bias_front  = 0.6;      // front share of brake torque
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double share = (i == WHEEL_FL || i == WHEEL_FR) ? bias_front : (1.0 - bias_front);
            const double spin  = ctx.state.wheel_spin[i];
            const double sign  = (spin >= 0.0) ? 1.0 : -1.0;
            T[i] = -sign * 0.5 * share * pedal_ * max_torque;  // signed, opposes spin
        }
        return T;
    }

    void reset() override { pedal_ = 0.0; }

private:
    double pedal_ {0.0};
};

// Register exactly one module per file. The name is what the catalog part will be labelled.
VDSIM_REGISTER_BRAKE_MODULE(MyBrake, "my_brake")
