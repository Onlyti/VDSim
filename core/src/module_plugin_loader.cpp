#include "vdsim/module_plugin.hpp"

#include <dlfcn.h>

#include <string>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "vdsim/interfaces.hpp"

namespace vdsim {

const char* to_string(ModuleKind k) noexcept {
    switch (k) {
        case ModuleKind::Brake:       return "brake";
        case ModuleKind::Steering:    return "steering";
        case ModuleKind::Drivetrain:  return "drivetrain";
        case ModuleKind::Suspension:  return "suspension";
        case ModuleKind::AntiRollBar: return "antirollbar";
        default:                      return "unknown";
    }
}

ModuleKind module_kind_from_string(const std::string& s) noexcept {
    if (s == "brake")       return ModuleKind::Brake;
    if (s == "steering")    return ModuleKind::Steering;
    if (s == "drivetrain")  return ModuleKind::Drivetrain;
    if (s == "suspension")  return ModuleKind::Suspension;
    if (s == "antirollbar") return ModuleKind::AntiRollBar;
    return ModuleKind::Unknown;
}

LoadedModule load_module_plugin(const std::string& so_path) {
    LoadedModule out;

    dlerror();
    void* h = dlopen(so_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!h) {
        const char* e = dlerror();
        out.error = std::string("dlopen failed: ") + (e ? e : "unknown error");
        return out;
    }

    auto abi_fn    = reinterpret_cast<int (*)()>(dlsym(h, kSymAbiVersion));
    auto kind_fn   = reinterpret_cast<const char* (*)()>(dlsym(h, kSymKind));
    auto name_fn   = reinterpret_cast<const char* (*)()>(dlsym(h, kSymName));
    auto create_fn = reinterpret_cast<void* (*)()>(dlsym(h, kSymCreate));
    if (!abi_fn || !kind_fn || !name_fn || !create_fn) {
        out.error = "missing plugin symbol(s) — the .so must use a VDSIM_REGISTER_*_MODULE macro";
        dlclose(h);
        return out;
    }

    const int abi = abi_fn();
    if (abi != kModuleAbiVersion) {
        out.error = "ABI mismatch: plugin abi=" + std::to_string(abi) +
                    " expected=" + std::to_string(kModuleAbiVersion) +
                    " (rebuild the module against the current core)";
        dlclose(h);
        return out;
    }

    const char* kind_s = kind_fn();
    const char* name_s = name_fn();
    out.kind = module_kind_from_string(kind_s ? kind_s : "");
    out.name = name_s ? name_s : "";
    if (out.kind == ModuleKind::Unknown) {
        out.error = std::string("unknown module kind: '") + (kind_s ? kind_s : "") +
                    "' (expected brake/steering/drivetrain/suspension/antirollbar)";
        dlclose(h);
        return out;
    }

    void* raw = create_fn();
    if (!raw) {
        out.error = "vdsim_module_create() returned null";
        dlclose(h);
        return out;
    }

    switch (out.kind) {
        case ModuleKind::Brake:       out.brake.reset(static_cast<IBrakeSystem*>(raw)); break;
        case ModuleKind::Steering:    out.steering.reset(static_cast<ISteeringSystem*>(raw)); break;
        case ModuleKind::Drivetrain:  out.drivetrain.reset(static_cast<IDrivetrain*>(raw)); break;
        case ModuleKind::Suspension:  out.suspension.reset(static_cast<ISuspension*>(raw)); break;
        case ModuleKind::AntiRollBar: out.antirollbar.reset(static_cast<IAntiRollBar*>(raw)); break;
        default: break;
    }

    // Keep the handle open on success: the instance's code must stay mapped for its lifetime.
    out.ok = true;
    return out;
}

bool install_module(IVehicleDynamics& dyn, const LoadedModule& m, int arb_axle) {
    if (!m.ok) return false;
    switch (m.kind) {
        case ModuleKind::Brake:       return dyn.set_brake_module(m.brake);
        case ModuleKind::Steering:    return dyn.set_steering_module(m.steering);
        case ModuleKind::Drivetrain:  return dyn.set_drivetrain_module(m.drivetrain);
        case ModuleKind::Suspension:  return dyn.set_suspension_module(m.suspension);
        case ModuleKind::AntiRollBar: return dyn.set_antirollbar_module(arb_axle, m.antirollbar);
        default:                      return false;
    }
}

int install_module_plugins_from_yaml(IVehicleDynamics& dyn, const std::string& vehicle_yaml_path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(vehicle_yaml_path);
    } catch (const std::exception& e) {
        spdlog::warn("module plugins: cannot read {}: {}", vehicle_yaml_path, e.what());
        return 0;
    }
    const auto list = root["module_plugins"];
    if (!list || !list.IsSequence()) return 0;

    int installed = 0;
    for (const auto& e : list) {
        const std::string so = e["plugin_so"] ? e["plugin_so"].as<std::string>() : "";
        const int axle = e["axle"] ? e["axle"].as<int>() : 0;
        if (so.empty()) {
            spdlog::warn("module plugins: entry without plugin_so, skipped");
            continue;
        }
        LoadedModule m = load_module_plugin(so);
        if (!m.ok) {
            spdlog::warn("module plugins: load failed for {}: {}", so, m.error);
            continue;
        }
        if (!install_module(dyn, m, axle)) {
            spdlog::warn("module plugins: {} ({}) not accepted by this model level",
                         so, to_string(m.kind));
            continue;
        }
        ++installed;
    }
    return installed;
}

}  // namespace vdsim
