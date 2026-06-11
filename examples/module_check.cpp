// vdsim_module_check <plugin.so> <expected_kind>
//
// Contract checker for a user subsystem-module plugin. Loads the .so, validates the ABI and
// that the module's kind matches the requested category, then probes it with a synthetic
// SubsystemContext and verifies the output is finite and correctly shaped. Prints a single
// JSON line {status:"pass"|"fail", kind, name[, cause]} and exits 0 (pass) / 1 (fail).

#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>

#include "vdsim/module_plugin.hpp"
#include "vdsim/state.hpp"
#include "vdsim/subsystems.hpp"

using namespace vdsim;

namespace {

std::string sanitize(const std::string& s) {
    std::string o;
    o.reserve(s.size());
    for (char c : s) {
        if (c == '"' || c == '\\') o += '\'';
        else if (c == '\n' || c == '\r' || c == '\t') o += ' ';
        else o += c;
    }
    return o;
}

bool fin(double x) { return std::isfinite(x); }

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("{\"status\":\"fail\",\"cause\":\"usage: vdsim_module_check <so> <kind>\"}\n");
        return 2;
    }
    const std::string so_path = argv[1];
    const std::string want    = argv[2];

    LoadedModule m = load_module_plugin(so_path);

    auto fail = [&](const std::string& cause) {
        std::printf("{\"status\":\"fail\",\"kind\":\"%s\",\"name\":\"%s\",\"cause\":\"%s\"}\n",
                    to_string(m.kind), sanitize(m.name).c_str(), sanitize(cause).c_str());
        return 1;
    };

    if (!m.ok) return fail(m.error);
    if (std::string(to_string(m.kind)) != want) {
        return fail("kind mismatch: module is '" + std::string(to_string(m.kind)) +
                    "' but the requested category is '" + want + "'");
    }

    // Synthetic probe state: rolling at 20 m/s, mild brake/throttle/steer, static-ish loads.
    State st;
    st.velocity.x() = 20.0;
    for (int i = 0; i < NUM_WHEELS; ++i) st.wheel_spin[i] = 20.0 / 0.32;
    DriverCmd cmd;
    cmd.handwheel_angle = 0.1;
    cmd.throttle        = 0.3;
    cmd.brake           = 0.5;
    cmd.gear            = 2;
    SubsystemContext ctx{st, cmd, 0.001};
    ctx.Fz = {{4000.0, 4000.0, 3500.0, 3500.0}};

    std::string bad;
    try {
        switch (m.kind) {
            case ModuleKind::Brake: {
                m.brake->reset();
                m.brake->begin_step(ctx, 0.001);
                const auto T = m.brake->wheel_torque(ctx);
                for (int i = 0; i < NUM_WHEELS; ++i)
                    if (!fin(T[i])) bad = "wheel_torque[" + std::to_string(i) + "] is not finite";
                break;
            }
            case ModuleKind::Steering: {
                m.steering->reset();
                m.steering->begin_step(ctx, 0.001);
                const auto o = m.steering->apply(ctx);
                if (!fin(o.roadwheel_angle) || !fin(o.rack_travel)) bad = "SteeringOutput is not finite";
                break;
            }
            case ModuleKind::Drivetrain: {
                m.drivetrain->reset();
                m.drivetrain->begin_step(ctx, 0.001);
                const auto o = m.drivetrain->apply(ctx);
                for (int i = 0; i < NUM_WHEELS; ++i)
                    if (!fin(o.wheel_torque[i])) bad = "wheel_torque[" + std::to_string(i) + "] is not finite";
                break;
            }
            case ModuleKind::Suspension: {
                m.suspension->reset();
                const CornerInput ci{0, 0.02, 0.1, 1.0};
                if (!fin(m.suspension->force(ctx, ci))) bad = "force is not finite";
                break;
            }
            case ModuleKind::AntiRollBar: {
                m.antirollbar->reset();
                const AxleDefl d{0.02, -0.01, 0.1, -0.05};
                const auto pr = m.antirollbar->force(ctx, d);
                if (!fin(pr.first) || !fin(pr.second)) bad = "ARB force is not finite";
                break;
            }
            default: bad = "unknown kind"; break;
        }
    } catch (const std::exception& e) {
        return fail(std::string("module threw during probe: ") + e.what());
    } catch (...) {
        return fail("module threw an unknown exception during probe");
    }

    if (!bad.empty()) return fail("probe produced invalid output: " + bad);

    std::printf("{\"status\":\"pass\",\"kind\":\"%s\",\"name\":\"%s\"}\n",
                to_string(m.kind), sanitize(m.name).c_str());
    return 0;
}
