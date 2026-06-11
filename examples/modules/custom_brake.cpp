// Sample user brake module (built into a .so for the module-plugin tests and as a worked
// example). A strong front-biased brake: signed torque opposing wheel spin. See theory ch.24.

#include <array>

#include "vdsim/module_plugin.hpp"

using namespace vdsim;

class CustomBrake : public IBrakeSystem {
public:
    void begin_step(const SubsystemContext& ctx, double /*dt*/) override {
        pedal_ = ctx.cmd.brake;
    }

    std::array<double, NUM_WHEELS> wheel_torque(const SubsystemContext& ctx) override {
        std::array<double, NUM_WHEELS> T{};
        const double max_torque = 4000.0;   // strong brake
        const double bias_front = 0.75;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double share = (i == WHEEL_FL || i == WHEEL_FR) ? bias_front : (1.0 - bias_front);
            const double spin  = ctx.state.wheel_spin[i];
            const double sign  = (spin >= 0.0) ? 1.0 : -1.0;
            T[i] = -sign * 0.5 * share * pedal_ * max_torque;
        }
        return T;
    }

    void reset() override { pedal_ = 0.0; }

private:
    double pedal_ {0.0};
};

VDSIM_REGISTER_BRAKE_MODULE(CustomBrake, "custom_brake")
