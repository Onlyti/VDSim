// Benchmark one dynamics level via SimSession::tick (ioniq5_awd, dt, N steps).
// Used when a level is not exposed in pybind (L5) or for cross-checks.
//
// Usage: vdsim_bench_levels --level=L5 [--dt=5e-4] [--steps=500] [--warmup=50]
// Prints one line: ms_per_step (float).
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

namespace {

#ifndef VDSIM_SOURCE_DIR
#define VDSIM_SOURCE_DIR "."
#endif

const char* kRepo = VDSIM_SOURCE_DIR;

double opt_d(int argc, char** argv, const char* key, double def) {
    const size_t n = std::strlen(key);
    for (int i = 1; i < argc; ++i) {
        if (std::strncmp(argv[i], key, n) == 0)
            return std::atof(argv[i] + n);
    }
    return def;
}

std::string opt_s(int argc, char** argv, const char* key, const std::string& def) {
    const size_t n = std::strlen(key);
    for (int i = 1; i < argc; ++i) {
        if (std::strncmp(argv[i], key, n) == 0)
            return std::string(argv[i] + n);
    }
    return def;
}

vdsim::VehicleParams ioniq5_vp() {
    return vdsim::VehicleParams::from_yaml(
        std::string(kRepo) + "/configs/vehicles/ioniq5_awd.yaml");
}

vdsim::TireParams ioniq5_tp() {
    vdsim::TireParams tp = vdsim::TireParams::from_yaml(
        std::string(kRepo) + "/configs/parts/tire/ioniq5_pac2002.yaml");
    tp.tir_path = std::string(kRepo) + "/configs/parts/tire/ioniq5_pac2002.tir";
    tp.lugre.enabled = false;
    return tp;
}

std::unique_ptr<vdsim::IVehicleDynamics> make_dyn(const std::string& level) {
    if (level == "K" || level == "L0") return vdsim::create_kinematic();
    if (level == "L1") return vdsim::create_bicycle();
    if (level == "L3") return vdsim::create_fourteen_dof();
    if (level == "L4") return vdsim::create_fourteen_dof_kinematic();
    if (level == "L5") return vdsim::create_stunt_dof();
    return vdsim::create_seven_dof();
}

vdsim::State init_state(const vdsim::VehicleParams& vp, const vdsim::TireParams& tp,
                        const std::string& level, double v) {
    vdsim::State s;
    s.position = vdsim::Vec3(0.0, 0.0, 0.0);
    if (level == "L5") s.position.z() = vp.cg_height;
    s.orientation = vdsim::quat_from_euler(vdsim::Euler{0.0, 0.0, 0.0});
    s.velocity = vdsim::Vec3(v, 0.0, 0.0);
    s.wheel_spin = vdsim::free_roll_wheel_spin(vp, tp, v);
    return s;
}

double bench(const std::string& level, double dt, int steps, int warmup, double v0) {
    const auto vp = ioniq5_vp();
    const auto tp = ioniq5_tp();
    vdsim::SolverParams sp;
    if (level == "L5") sp.stunt_physics = true;

    vdsim::SimConfig cfg;
    cfg.nominal_dt = dt;
    vdsim::SimSession sess(make_dyn(level),
                           vdsim::create_flat_ground(0.0, tp.mu_nominal),
                           vp, tp, sp, cfg);
    sess.reset(init_state(vp, tp, level, v0));

    vdsim::CmdL4 cmd;
    cmd.steer_angle_wheel = 0.02;
    for (int i = 0; i < warmup; ++i) {
        sess.set_input(vdsim::ControlInput{cmd});
        sess.tick(dt);
    }

    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; ++i) {
        sess.set_input(vdsim::ControlInput{cmd});
        sess.tick(dt);
    }
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    return ms / static_cast<double>(steps);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string level = opt_s(argc, argv, "--level=", "L5");
    const double dt = opt_d(argc, argv, "--dt=", 5e-4);
    const int steps = static_cast<int>(opt_d(argc, argv, "--steps=", 500));
    const int warmup = static_cast<int>(opt_d(argc, argv, "--warmup=", 50));
    const double v0 = opt_d(argc, argv, "--v0=", 20.0);

    if (steps <= 0 || warmup < 0 || !(dt > 0.0)) {
        std::fprintf(stderr, "invalid bench args\n");
        return 2;
    }

    try {
        const double ms = bench(level, dt, steps, warmup, v0);
        std::printf("%.4f\n", ms);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "bench %s failed: %s\n", level.c_str(), e.what());
        return 1;
    }
}
