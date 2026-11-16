#pragma once

// Internal side of the user module fault boundary (work order 51b): the per-model fault
// latch and the guard wrappers installed by the dynamics models' set_*_module().

#include "vdsim/aero.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/subsystems.hpp"

#include <memory>
#include <string>

namespace vdsim::detail {

/// First user-module fault of one dynamics model since the last clear().
struct ModuleFaultLatch {
    bool set {false};
    std::string kind;    ///< brake | steering | drivetrain | suspension | antirollbar | aero
    std::string name;    ///< module display name
    std::string cause;   ///< "exception: <what>" or "non-finite output <field>"
    int installed {0};   ///< guarded user modules installed on this model

    /** @brief Keep the first fault; later ones are ignored until clear(). */
    void record(const std::string& k, const std::string& n, std::string c) {
        if (set) return;
        set = true;
        kind = k;
        name = n;
        cause = std::move(c);
    }
    /** @brief Forget the fault (reset / restore). */
    void clear() {
        set = false;
        kind.clear();
        name.clear();
        cause.clear();
    }
};

/// Implemented by every dynamics model that accepts user modules.
class ModuleGuardHost {
public:
    virtual ~ModuleGuardHost() = default;
    /** @brief The latch the model's guarded modules write to. */
    virtual ModuleFaultLatch& module_latch() = 0;
};

/** @brief Latch of a model, or nullptr when it takes no user modules. */
ModuleFaultLatch* module_latch_of(IVehicleDynamics& dyn);

/**
 * @brief Wrap a user module so exceptions and non-finite outputs go to `latch`.
 *
 * Each returns nullptr for nullptr and the module itself when it is already guarded.
 * After a fault the wrapper no longer calls the module and returns a neutral output
 * (zero torque / force, kinematic rack 0, no aero force) for the rest of the step.
 * @param m     User module.
 * @param latch Model latch (outlives the wrapper: both belong to the model).
 * @return Guarded module.
 */
std::shared_ptr<IBrakeSystem> guard_brake(std::shared_ptr<IBrakeSystem> m, ModuleFaultLatch* latch);
/** @copydoc guard_brake */
std::shared_ptr<ISteeringSystem> guard_steering(std::shared_ptr<ISteeringSystem> m,
                                                ModuleFaultLatch* latch);
/** @copydoc guard_brake */
std::shared_ptr<IDrivetrain> guard_drivetrain(std::shared_ptr<IDrivetrain> m, ModuleFaultLatch* latch);
/** @copydoc guard_brake */
std::shared_ptr<ISuspension> guard_suspension(std::shared_ptr<ISuspension> m, ModuleFaultLatch* latch);
/** @copydoc guard_brake */
std::shared_ptr<IAntiRollBar> guard_antirollbar(std::shared_ptr<IAntiRollBar> m,
                                                ModuleFaultLatch* latch);
/** @copydoc guard_brake */
std::shared_ptr<IAeroModel> guard_aero(std::shared_ptr<IAeroModel> m, ModuleFaultLatch* latch);

}  // namespace vdsim::detail
