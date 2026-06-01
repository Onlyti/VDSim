// VDSim L2 (Seven-DOF) packaged as an FMI 2.0 Co-Simulation FMU.
//
// Variable map (ValueReferences):
//   Inputs (3):
//     1  steer_angle_wheel  [rad]
//     2  throttle           [0..1]
//     3  brake              [0..1]
//   Outputs (8):
//    10  x_world            [m]
//    11  y_world            [m]
//    12  yaw                [rad]
//    13  vx                 [m/s]
//    14  vy                 [m/s]
//    15  yaw_rate           [rad/s]
//    16  ax_body            [m/s²]
//    17  ay_body            [m/s²]
//
// The FMU reads vehicle.yaml + tire.yaml from <resourceLocation>/configs/
// at fmi2EnterInitializationMode.  Surface contacts default to flat ground,
// μ = 1.0 for all four wheels.

#include "fmi2/fmi2Functions.h"

#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"
#include "vdsim/control.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

struct VDSimL2 {
    std::unique_ptr<vdsim::IVehicleDynamics> dyn;
    vdsim::VehicleParams vp;
    vdsim::TireParams    tp;
    vdsim::SolverParams  sp;
    vdsim::ContactArray  contacts;
    double in_steer {0.0};
    double in_throttle {0.0};
    double in_brake {0.0};
    double t {0.0};
    std::string resource_location;
    std::string instance_name;
    fmi2CallbackLogger logger {nullptr};
    fmi2ComponentEnvironment cenv {nullptr};
    bool logging_on {false};
};

namespace {

void log(VDSimL2* m, fmi2Status s, const char* category, const char* msg) {
    if (m && m->logger && m->logging_on)
        m->logger(m->cenv, m->instance_name.c_str(), s, category, msg);
}

vdsim::ContactArray flat_contacts(double mu = 1.0) {
    vdsim::ContactArray c;
    for (auto& p : c) {
        p.is_valid = true;
        p.normal = vdsim::Vec3(0.0, 0.0, 1.0);
        p.mu_long = mu; p.mu_lat = mu;
    }
    return c;
}

}  // namespace

extern "C" {

const char* fmi2GetVersion(void) { return fmi2Version; }
const char* fmi2GetTypesPlatform(void) { return fmi2TypesPlatform; }

fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn,
                                 size_t, const fmi2String[]) {
    if (!c) return fmi2Error;
    static_cast<VDSimL2*>(c)->logging_on = (loggingOn != 0);
    return fmi2OK;
}

fmi2Component fmi2Instantiate(fmi2String instanceName, fmi2Type fmuType,
                                fmi2String /*fmuGUID*/, fmi2String resourceLocation,
                                const fmi2CallbackFunctions* functions,
                                fmi2Boolean /*visible*/, fmi2Boolean loggingOn) {
    if (fmuType != fmi2CoSimulation) return nullptr;
    auto* m = new VDSimL2;
    m->instance_name = instanceName ? instanceName : "vdsim_l2";
    m->logger = functions ? functions->logger : nullptr;
    m->cenv   = functions ? functions->componentEnvironment : nullptr;
    m->logging_on = (loggingOn != 0);
    // FMI passes resourceLocation as e.g. "file:///path/to/resources" or just a path
    if (resourceLocation) {
        std::string loc = resourceLocation;
        const std::string prefix = "file://";
        if (loc.find(prefix) == 0) loc = loc.substr(prefix.size());
        // Some implementations include /// — strip the leading slashes if path
        // becomes empty / non-existent.  Just keep as-is.
        m->resource_location = loc;
    }
    m->contacts = flat_contacts();
    return reinterpret_cast<fmi2Component>(m);
}

void fmi2FreeInstance(fmi2Component c) {
    if (!c) return;
    delete static_cast<VDSimL2*>(c);
}

fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean /*tol*/,
                                 fmi2Real /*tol_val*/, fmi2Real /*startTime*/,
                                 fmi2Boolean /*stop*/, fmi2Real /*stopTime*/) {
    if (!c) return fmi2Error;
    return fmi2OK;
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c) {
    if (!c) return fmi2Error;
    auto* m = static_cast<VDSimL2*>(c);
    try {
        // Look for resources/configs/{vehicle,tire,solver}.yaml
        const std::string base = m->resource_location.empty()
            ? std::string("resources")
            : m->resource_location;
        m->vp = vdsim::VehicleParams::from_yaml(base + "/configs/vehicle.yaml");
        m->tp = vdsim::TireParams::from_yaml   (base + "/configs/tire.yaml");
        try {
            m->sp = vdsim::SolverParams::from_yaml(base + "/configs/solver.yaml");
        } catch (...) {
            m->sp = vdsim::SolverParams();   // default if absent
        }
        m->dyn = vdsim::create_seven_dof();
        m->dyn->initialize(m->vp, m->tp, m->sp);
        // Initial state — zero velocity
        vdsim::State s0;
        m->dyn->reset(s0);
        m->t = 0.0;
        log(m, fmi2OK, "init", "VDSim L2 FMU initialized from YAML resources");
    } catch (const std::exception& e) {
        log(m, fmi2Error, "init", e.what());
        return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
    return c ? fmi2OK : fmi2Error;
}

fmi2Status fmi2Reset(fmi2Component c) {
    if (!c) return fmi2Error;
    auto* m = static_cast<VDSimL2*>(c);
    if (m->dyn) {
        vdsim::State s0;
        m->dyn->reset(s0);
    }
    m->t = 0.0;
    return fmi2OK;
}

fmi2Status fmi2Terminate(fmi2Component) { return fmi2OK; }

// ---- Get / Set ----
fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[],
                         size_t n, const fmi2Real v[]) {
    if (!c) return fmi2Error;
    auto* m = static_cast<VDSimL2*>(c);
    for (size_t i = 0; i < n; ++i) {
        switch (vr[i]) {
            case 1: m->in_steer    = v[i]; break;
            case 2: m->in_throttle = v[i]; break;
            case 3: m->in_brake    = v[i]; break;
            default: return fmi2Warning;     // unknown vr (read-only or unknown)
        }
    }
    return fmi2OK;
}

fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[],
                         size_t n, fmi2Real v[]) {
    if (!c) return fmi2Error;
    auto* m = static_cast<VDSimL2*>(c);
    if (!m->dyn) {
        for (size_t i = 0; i < n; ++i) v[i] = 0.0;
        return fmi2OK;
    }
    const vdsim::State& s = m->dyn->state();
    for (size_t i = 0; i < n; ++i) {
        switch (vr[i]) {
            case 1:  v[i] = m->in_steer; break;
            case 2:  v[i] = m->in_throttle; break;
            case 3:  v[i] = m->in_brake; break;
            case 10: v[i] = s.position.x(); break;
            case 11: v[i] = s.position.y(); break;
            case 12: v[i] = s.yaw(); break;
            case 13: v[i] = s.vx(); break;
            case 14: v[i] = s.vy(); break;
            case 15: v[i] = s.yaw_rate(); break;
            case 16: v[i] = m->dyn->ax_body_est(); break;
            case 17: v[i] = m->dyn->ay_body_est(); break;
            default: v[i] = 0.0; break;
        }
    }
    return fmi2OK;
}

// Unimplemented stubs (return OK no-op)
fmi2Status fmi2GetInteger(fmi2Component, const fmi2ValueReference[], size_t, fmi2Integer[]) { return fmi2OK; }
fmi2Status fmi2GetBoolean(fmi2Component, const fmi2ValueReference[], size_t, fmi2Boolean[]) { return fmi2OK; }
fmi2Status fmi2GetString (fmi2Component, const fmi2ValueReference[], size_t, fmi2String[])  { return fmi2OK; }
fmi2Status fmi2SetInteger(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Integer[]) { return fmi2OK; }
fmi2Status fmi2SetBoolean(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Boolean[]) { return fmi2OK; }
fmi2Status fmi2SetString (fmi2Component, const fmi2ValueReference[], size_t, const fmi2String[])  { return fmi2OK; }

// ---- Co-Simulation ----
fmi2Status fmi2DoStep(fmi2Component c, fmi2Real currentCommunicationPoint,
                       fmi2Real communicationStepSize, fmi2Boolean) {
    if (!c) return fmi2Error;
    auto* m = static_cast<VDSimL2*>(c);
    if (!m->dyn) return fmi2Error;

    vdsim::CmdL4 cmd;
    cmd.steer_angle_wheel = m->in_steer;
    cmd.throttle = m->in_throttle;
    cmd.brake = m->in_brake;
    cmd.gear = 1;

    vdsim::ControlInput u = cmd;
    m->dyn->step(u, m->contacts, communicationStepSize);
    m->t = currentCommunicationPoint + communicationStepSize;
    return fmi2OK;
}

fmi2Status fmi2CancelStep(fmi2Component) { return fmi2OK; }
fmi2Status fmi2SetRealInputDerivatives(fmi2Component, const fmi2ValueReference[],
    size_t, const fmi2Integer[], const fmi2Real[]) { return fmi2Discard; }
fmi2Status fmi2GetRealOutputDerivatives(fmi2Component, const fmi2ValueReference[],
    size_t, const fmi2Integer[], fmi2Real[]) { return fmi2Discard; }
fmi2Status fmi2GetStatus(fmi2Component, fmi2StatusKind, fmi2Status* v)   { if(v) *v = fmi2OK; return fmi2OK; }
fmi2Status fmi2GetRealStatus(fmi2Component, fmi2StatusKind, fmi2Real* v) { if(v) *v = 0; return fmi2OK; }
fmi2Status fmi2GetIntegerStatus(fmi2Component, fmi2StatusKind, fmi2Integer* v) { if(v) *v = 0; return fmi2OK; }
fmi2Status fmi2GetBooleanStatus(fmi2Component, fmi2StatusKind, fmi2Boolean* v) { if(v) *v = 0; return fmi2OK; }
fmi2Status fmi2GetStringStatus(fmi2Component, fmi2StatusKind, fmi2String* v)   { if(v) *v = ""; return fmi2OK; }

}  // extern "C"
