# Ch.24 — User module plugins (build / check / register)

[Ch.23](23_user_modules.md) showed how to install a C++ subclass of a subsystem interface
in-process. This chapter is how a user ships such a module **without forking the build**: a
plugin `.so` with a tiny C ABI, a loader, and a build/contract-check/register workflow. The
authoring is C++ only — the user writes the `.cpp` in their own editor; the toolchain (CLI
or GUI Module Workshop) detects the file, builds it, checks the contract, and on success
registers it as a catalog part.

## 1. The plugin ABI

A module source subclasses one interface and emits four `extern "C"` symbols via a macro
(`vdsim/module_plugin.hpp`):

```cpp
#include "vdsim/module_plugin.hpp"
class MyBrake : public vdsim::IBrakeSystem { /* ... */ };
VDSIM_REGISTER_BRAKE_MODULE(MyBrake, "my_brake")
```

The macro defines `vdsim_module_abi_version()`, `vdsim_module_kind()` (`"brake"` …),
`vdsim_module_name()`, and `vdsim_module_create()` (which upcasts to the interface pointer
before returning it as `void*`). One module per `.so`. Templates for all five kinds are in
`templates/modules/`.

## 2. Loader

`load_module_plugin(so_path)` (`module_plugin_loader.cpp`) `dlopen`s the `.so`, checks the
ABI version, dispatches on the kind, instantiates, and returns a `shared_ptr` to the right
interface (or `ok=false` with an `error`). `install_module(dyn, loaded)` attaches it via the
existing `IVehicleDynamics::set_*_module()` (ch.23). The handle stays mapped for the
instance's lifetime.

## 3. Build / check / register

`tools/module_workshop.py` (and the GUI Module Workshop) drive three steps:

1. **build** — `g++ -std=c++17 -shared -fPIC -I core/include -I <eigen> <name>.cpp
   -L build/lib -lvdsim_core -o <name>.so`. A compile failure returns the compiler
   diagnostics as the **cause**.
2. **check** — `vdsim_module_check <so> <kind>` loads the `.so` and probes it with a
   synthetic `SubsystemContext`, verifying: ABI matches, kind equals the requested category,
   the instance is non-null, and the output is finite and correctly shaped (4 torques for
   brake/drivetrain, a finite scalar for suspension, a finite pair for ARB). Reports **적격
   (pass)** or **문제 (fail)** with the specific cause.
3. **register** — on pass, writes a `module_plugin_v1` catalog part under the `gui_custom`
   user package (`body: {kind, plugin_so, abi}`), so the module is selectable like any part.

## 4. Contract (recap)

Same as ch.23: `begin_step` runs once per step (step-coherent state), `apply`/`wheel_torque`
/`force` per RK4 stage (pure); brake/drivetrain `wheel_torque` is a **signed** torque that
opposes wheel spin. A module that violates the *shape* contract (non-finite, wrong size) is
rejected by the checker; physical correctness (signs, magnitudes) is the author's
responsibility.

## 5. Notes

- The `.so` is a per-machine build artifact (it links the exact core); the registered part's
  `plugin_so` is a local absolute path. Rebuild the module if core changes (the ABI version
  guards the C contract, not the C++ vtable).
- For real-time / HIL the runtime loads the registered plugin after `initialize()` and
  installs it via `set_*_module()`. Python authoring is intentionally out of scope.
