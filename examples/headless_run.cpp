// vdsim_headless_run — experiment / batch mode runner on the SimSession kernel.
//
// Drives a SimSession as fast as possible (no real-time pacing), feeding a
// YAML-defined command scenario through the actuator/sensor layers, and writes a
// CSV trace (viewer-compatible). This is the "external-step" mode: the loop here
// is the clock. The same kernel is paced by RealTimeRunner for the UDP co-sim.
//
// Usage:
//   vdsim_headless_run <vehicle.yaml> <tire.yaml> <scenario.yaml> <out.csv>
//                      [solver.yaml] [--level=L1|L2|L3]
//                      [--sensor-delay=S] [--steer-tau=S] [--brake-deadtime=S]
//
// The optional actuator/sensor flags exercise the layer; omit for identity.
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/scenario.hpp"
#include "vdsim/sim_session.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {

double opt(int argc, char** argv, const char* key, double def) {
    const size_t klen = std::strlen(key);
    for (int i = 1; i < argc; ++i) {
        if (std::strncmp(argv[i], key, klen) == 0)
            return std::atof(argv[i] + klen);
    }
    return def;
}

std::string opt_str(int argc, char** argv, const char* key, const std::string& def) {
    const size_t klen = std::strlen(key);
    for (int i = 1; i < argc; ++i) {
        if (std::strncmp(argv[i], key, klen) == 0) return std::string(argv[i] + klen);
    }
    return def;
}

std::unique_ptr<vdsim::IVehicleDynamics> make_dyn(const std::string& level) {
    if (level == "L1") return vdsim::create_bicycle();
    if (level == "L3") return vdsim::create_fourteen_dof();
    return vdsim::create_seven_dof();   // default L2
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 5) {
        std::fprintf(stderr,
            "usage: %s <vehicle.yaml> <tire.yaml> <scenario.yaml> <out.csv> "
            "[solver.yaml] [--level=L1|L2|L3] [--sensor-delay=S] [--steer-tau=S] "
            "[--brake-deadtime=S]\n", argv[0]);
        return 2;
    }
    const std::string vp_path = argv[1], tp_path = argv[2];
    const std::string sc_path = argv[3], out_path = argv[4];

    const auto vp = vdsim::VehicleParams::from_yaml(vp_path);
    const auto tp = vdsim::TireParams::from_yaml(tp_path);
    const auto sc = vdsim::Scenario::from_yaml(sc_path);
    vdsim::SolverParams sp;
    // 5th positional (not starting with '-') = solver yaml.
    if (argc >= 6 && argv[5][0] != '-') sp = vdsim::SolverParams::from_yaml(argv[5]);

    const std::string level = opt_str(argc, argv, "--level=", "L2");

    vdsim::SimConfig cfg;
    cfg.nominal_dt        = sc.dt;
    cfg.sensor_delay_s    = opt(argc, argv, "--sensor-delay=", 0.0);
    cfg.actuator.steer.ch.tau_s    = opt(argc, argv, "--steer-tau=", 0.0);
    cfg.actuator.brake.ch.dead_time_s = opt(argc, argv, "--brake-deadtime=", 0.0);

    vdsim::SimSession sim(make_dyn(level), vdsim::create_flat_ground(0.0, sc.mu),
                          vp, tp, sp, cfg);
    vdsim::State s0; s0.velocity.x() = sc.initial_vx;
    s0.wheel_spin = vdsim::free_roll_wheel_spin(vp, tp, sc.initial_vx);
    sim.reset(s0);

    std::ofstream csv(out_path);
    if (!csv) throw std::runtime_error("cannot open output CSV: " + out_path);
    csv << "t,x,y,yaw,vx,vy,r,throttle,brake,steer,"
           "Fz_FL,Fz_FR,Fz_RL,Fz_RR,ax,ay,roll,pitch\n";

    const int N = static_cast<int>(std::round(sc.duration / sc.dt));
    for (int i = 0; i <= N; ++i) {
        const double t = i * sc.dt;
        const auto cs = sc.sample(t);
        vdsim::CmdL4 cmd;
        cmd.throttle = cs.throttle; cmd.brake = cs.brake;
        cmd.steer_angle_wheel = cs.steer; cmd.gear = cs.gear;
        sim.set_input(cmd);

        const auto o = sim.output();
        const auto& s = o.state;
        csv << t << ','
            << s.position.x() << ',' << s.position.y() << ','
            << vdsim::yaw_from_quat(s.orientation) << ','
            << s.velocity.x() << ',' << s.velocity.y() << ',' << s.angular_velocity.z() << ','
            << cs.throttle << ',' << cs.brake << ',' << cs.steer << ','
            << o.Fz[0] << ',' << o.Fz[1] << ',' << o.Fz[2] << ',' << o.Fz[3] << ','
            << o.ax << ',' << o.ay << ',' << o.roll << ',' << o.pitch << '\n';

        if (i < N) sim.tick(sc.dt);
    }

    std::fprintf(stderr,
        "[vdsim_headless_run] %s (%s): vx %.3f -> %.3f m/s, r %.4f rad/s, %d samples\n",
        sc.name.c_str(), level.c_str(), sc.initial_vx,
        sim.state().velocity.x(), sim.state().yaw_rate(), N + 1);
    return 0;
}
