#pragma once

// User-defined subsystem modules shipped as a runtime-loadable plugin (.so).
//
// A plugin source file subclasses ONE subsystem interface (vdsim/subsystems.hpp) and emits
// a tiny C ABI via the VDSIM_REGISTER_*_MODULE macro below. `load_module_plugin()` dlopens
// the .so, validates the ABI version and kind, instantiates the module, and returns a
// shared_ptr to the matching interface. `install_module()` then attaches it to a model via
// the existing IVehicleDynamics::set_*_module() hooks. See theory ch.24.

#include <memory>
#include <string>

#include "vdsim/subsystems.hpp"

namespace vdsim {

class IVehicleDynamics;  // forward decl (install_module is defined in the loader TU)

// Bump when the C ABI or the probed interface contract changes incompatibly.
constexpr int kModuleAbiVersion = 1;

enum class ModuleKind { Brake, Steering, Drivetrain, Suspension, AntiRollBar, Unknown };

const char* to_string(ModuleKind k) noexcept;
ModuleKind  module_kind_from_string(const std::string& s) noexcept;

// Exact symbol names a plugin .so exports (the macros define these).
inline constexpr const char* kSymAbiVersion = "vdsim_module_abi_version";
inline constexpr const char* kSymKind       = "vdsim_module_kind";
inline constexpr const char* kSymName       = "vdsim_module_name";
inline constexpr const char* kSymCreate     = "vdsim_module_create";

struct LoadedModule {
    bool        ok {false};
    ModuleKind  kind {ModuleKind::Unknown};
    std::string name;
    std::string error;                       // populated when ok == false
    std::shared_ptr<IBrakeSystem>    brake;
    std::shared_ptr<ISteeringSystem> steering;
    std::shared_ptr<IDrivetrain>     drivetrain;
    std::shared_ptr<ISuspension>     suspension;
    std::shared_ptr<IAntiRollBar>    antirollbar;
};

// dlopen + validate (ABI version, known kind) + instantiate. Never throws; on any failure
// returns ok == false with a human-readable `error`. The handle is intentionally leaked
// (kept open) so the instance's code stays mapped for the process lifetime.
LoadedModule load_module_plugin(const std::string& so_path);

// Attach a loaded module to a model via the matching set_*_module(). Returns false if the
// load failed or the host level does not accept that module kind. `arb_axle` (0 front /
// 1 rear) is used only for anti-roll-bar modules.
bool install_module(IVehicleDynamics& dyn, const LoadedModule& m, int arb_axle = 0);

// Read a resolved vehicle YAML's `module_plugins:` list (a sequence of
// {kind, plugin_so, [axle]}), load each .so, and install it on the model. Call AFTER
// dyn.initialize(). Returns the number successfully installed; bad entries are skipped
// (and logged). A vehicle without the key installs nothing and returns 0.
int install_module_plugins_from_yaml(IVehicleDynamics& dyn, const std::string& vehicle_yaml_path);

}  // namespace vdsim

// ---------------------------------------------------------------------------
// Plugin-side registration. Put exactly one of these at file scope in a plugin .cpp.
// The create function upcasts to the interface pointer BEFORE void* so the loader's
// reverse cast yields a correctly-adjusted interface pointer.
// ---------------------------------------------------------------------------
#define VDSIM_MODULE_EXPORT_(KIND_STR, NAME_STR, IFACE, CTOR_EXPR)                    \
    extern "C" int         vdsim_module_abi_version() { return ::vdsim::kModuleAbiVersion; } \
    extern "C" const char* vdsim_module_kind()        { return KIND_STR; }            \
    extern "C" const char* vdsim_module_name()        { return NAME_STR; }            \
    extern "C" void*       vdsim_module_create()                                      \
        { return static_cast<void*>(static_cast<IFACE*>(CTOR_EXPR)); }

#define VDSIM_REGISTER_BRAKE_MODULE(CLASS, NAME) \
    VDSIM_MODULE_EXPORT_("brake", NAME, ::vdsim::IBrakeSystem, new CLASS())
#define VDSIM_REGISTER_STEERING_MODULE(CLASS, NAME) \
    VDSIM_MODULE_EXPORT_("steering", NAME, ::vdsim::ISteeringSystem, new CLASS())
#define VDSIM_REGISTER_DRIVETRAIN_MODULE(CLASS, NAME) \
    VDSIM_MODULE_EXPORT_("drivetrain", NAME, ::vdsim::IDrivetrain, new CLASS())
#define VDSIM_REGISTER_SUSPENSION_MODULE(CLASS, NAME) \
    VDSIM_MODULE_EXPORT_("suspension", NAME, ::vdsim::ISuspension, new CLASS())
#define VDSIM_REGISTER_ANTIROLLBAR_MODULE(CLASS, NAME) \
    VDSIM_MODULE_EXPORT_("antirollbar", NAME, ::vdsim::IAntiRollBar, new CLASS())
