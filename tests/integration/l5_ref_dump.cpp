// Work order 44 E4: C++ reference trajectories for the Python L5 entry points.
//
// Usage: vdsim_l5_ref_dump <source_dir> <out_dir>
//
// Builds the L5 model directly (create_stunt_dof + SimSession, and the C++
// make_direct_control_session) with the solver settings the Python high-level
// entry points pick for L5 (vdsim_plant.level_solver_params), drives a 3 s step
// steer / drive input and writes one line per tick, %.17g, to
// <out_dir>/<case>.txt. tests/scripts/test_level_parse.py runs the same cases
// through the Python bindings and requires the lines to be identical.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

namespace {

using namespace vdsim;

/**
 * @brief L5 solver settings, mirroring vdsim_plant.level_solver_params.
 * @param sp base settings (adjusted copy is returned).
 * @param outer_dt [s] outer tick length.
 * @return sp with stunt_physics on, max_substep_dt <= 2e-4 and enough substeps.
 */
SolverParams l5_solver(SolverParams sp, double outer_dt) {
    sp.stunt_physics = true;
    sp.max_substep_dt = std::min(sp.max_substep_dt, 2e-4);
    sp.max_substeps = std::max(sp.max_substeps,
                               static_cast<int>(std::ceil(outer_dt / sp.max_substep_dt)));
    return sp;
}

/**
 * @brief Initial state as vdsim.make_init_state(vp, tp, v=v) builds it.
 * @param vp vehicle.  @param tp tire.  @param v [m/s] forward speed.
 * @return pose at the origin, yaw 0, free-rolling wheels.
 */
State init_state(const VehicleParams& vp, const TireParams& tp, double v) {
    State s;
    s.position = Vec3(0.0, 0.0, 0.0);
    s.orientation = quat_from_euler(Euler{0.0, 0.0, 0.0});
    s.velocity = Vec3(v, 0.0, 0.0);
    s.wheel_spin = free_roll_wheel_spin(vp, tp, v);
    return s;
}

/**
 * @brief Append one output row (22 columns) to f.
 * @param f open text file.  @param o session output after the tick.
 */
void write_row(FILE* f, const SimOutput& o) {
    const State& st = o.state;
    const double row[22] = {
        st.position.x(), st.position.y(), st.position.z(),
        st.velocity.x(), st.velocity.y(), st.velocity.z(),
        st.angular_velocity.x(), st.angular_velocity.y(), st.angular_velocity.z(),
        o.roll, o.pitch, st.yaw(),
        o.Fz[0], o.Fz[1], o.Fz[2], o.Fz[3], o.ax, o.ay,
        st.wheel_spin[0], st.wheel_spin[1], st.wheel_spin[2], st.wheel_spin[3]};
    for (int i = 0; i < 22; ++i) std::fprintf(f, i ? " %.17g" : "%.17g", row[i]);
    std::fputc('\n', f);
}

/**
 * @brief Flat-ground session case (Python: vdsim.make_sim_session(level="L5")).
 * @return 0 on success.
 */
int dump_session(const std::string& src, const std::string& out, const char* veh) {
    const auto vp = VehicleParams::from_yaml(src + "/configs/vehicles/" + veh + ".yaml");
    const auto tp = TireParams::from_yaml(src + "/configs/parts/tire/generic_pacejka.yaml");
    const double dt = 0.005;
    SimConfig cfg;
    cfg.nominal_dt = dt;
    SimSession s(create_stunt_dof(), create_flat_ground(0.0, 1.0), vp, TireSetup(tp),
                 l5_solver(SolverParams{}, dt), cfg);
    s.reset(init_state(vp, tp, 10.0));
    FILE* f = std::fopen((out + "/session_" + veh + ".txt").c_str(), "w");
    if (!f) return 1;
    for (int k = 0; k < 600; ++k) {
        CmdL4 u;
        u.throttle = k < 100 ? 0.3 : 0.2;
        u.steer_angle_wheel = k < 100 ? 0.0 : 0.05;
        s.set_input(u);
        s.tick(dt);
        write_row(f, s.output());
    }
    std::fclose(f);
    return 0;
}

/**
 * @brief Direct-control case (Python: vdsim.make_direct_control_session, level "L5"),
 *        solver as VDSimPlant builds it for substep_dt = 5e-4.
 * @return 0 on success.
 */
int dump_direct(const std::string& src, const std::string& out, const char* veh) {
    const auto vp = VehicleParams::from_yaml(src + "/configs/vehicles/" + veh + ".yaml");
    const auto tp = TireParams::from_yaml(src + "/configs/parts/tire/generic_pacejka.yaml");
    const double dt = 5e-4;
    SolverParams sp;
    sp.integrator = SolverParams::Integrator::RK4;
    sp.max_substep_dt = std::min(dt, sp.max_substep_dt);
    DirectControlSessionOptions opts;
    opts.level = "L5";
    opts.nominal_dt = dt;
    auto s = make_direct_control_session(vp, TireSetup(tp), l5_solver(sp, dt), opts);
    s->reset(init_state(vp, tp, 10.0));
    FILE* f = std::fopen((out + "/direct_" + veh + ".txt").c_str(), "w");
    if (!f) return 1;
    for (int k = 0; k < 6000; ++k) {
        CmdL1 u;
        const double tq = k < 1000 ? 150.0 : 100.0;
        u.motor_torque = {{tq, tq, tq, tq}};
        u.steer_angle_wheel = k < 1000 ? 0.0 : 0.05;
        s->set_input(u);
        s->tick(dt);
        if (k % 10 == 9) write_row(f, s->output());
    }
    std::fclose(f);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <source_dir> <out_dir>\n", argv[0]);
        return 2;
    }
    const std::string src = argv[1], out = argv[2];
    int rc = 0;
    for (const char* veh : {"generic_sedan", "fs_aero_example"}) {
        rc |= dump_session(src, out, veh);
        rc |= dump_direct(src, out, veh);
    }
    std::printf("solver_substep_clamp_count %llu\n",
                static_cast<unsigned long long>(solver_substep_clamp_count()));
    return rc;
}
