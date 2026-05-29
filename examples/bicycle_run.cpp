// CLI example: load vehicle + tire YAML, run a named scenario, dump a time-series CSV.
//
// Usage:
//   vdsim_bicycle_run <vehicle.yaml> <tire.yaml> <scenario> <out.csv> [solver.yaml]
//
// Supported <scenario> names:
//   step_steer     vx0 = 10 m/s, delta = 0.05 rad, T = 5 s
//   throttle_step  vx0 = 5 m/s,  throttle = 0.5,  T = 4 s
//   brake_step     vx0 = 20 m/s, brake = 0.8,     T = 2 s
//
// CSV columns: t,x,y,yaw,vx,vy,r,omega_f,omega_r,Fz_FL,Fz_FR,Fz_RL,Fz_RR

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

vdsim::ContactArray flat_contacts(double mu = 1.0) {
    vdsim::ContactArray c;
    for (auto& p : c) {
        p.is_valid = true;
        p.normal   = vdsim::Vec3::UnitZ();
        p.mu_long  = mu;
        p.mu_lat   = mu;
    }
    return c;
}

vdsim::State init_state(double vx, double R) {
    vdsim::State s;
    s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

struct Scenario {
    double vx0;
    double duration;
    double dt;
    double throttle;
    double brake;
    double steer;
};

Scenario parse_scenario(std::string_view name) {
    if (name == "step_steer")    return {10.0, 5.0, 0.005, 0.0, 0.0, 0.05};
    if (name == "throttle_step") return {5.0,  4.0, 0.005, 0.5, 0.0, 0.0 };
    if (name == "brake_step")    return {20.0, 2.0, 0.005, 0.0, 0.8, 0.0 };
    throw std::runtime_error("unknown scenario: " + std::string(name));
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 5) {
        std::fprintf(stderr,
            "usage: %s <vehicle.yaml> <tire.yaml> <scenario> <out.csv>\n"
            "scenario: step_steer | throttle_step | brake_step\n",
            argv[0]);
        return 2;
    }

    const std::string vehicle_path = argv[1];
    const std::string tire_path    = argv[2];
    const std::string scen_name    = argv[3];
    const std::string out_path     = argv[4];

    const auto vp = vdsim::VehicleParams::from_yaml(vehicle_path);
    const auto tp = vdsim::TireParams::from_yaml(tire_path);
    vdsim::SolverParams sp;
    if (argc >= 6) {
        sp = vdsim::SolverParams::from_yaml(argv[5]);
    }

    const auto scen = parse_scenario(scen_name);

    auto dyn = vdsim::create_bicycle();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(scen.vx0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd;
    cmd.throttle           = scen.throttle;
    cmd.brake              = scen.brake;
    cmd.steer_angle_wheel  = scen.steer;
    const vdsim::ControlInput u = cmd;
    const auto contacts = flat_contacts();

    std::ofstream csv(out_path);
    if (!csv) throw std::runtime_error("cannot open output CSV: " + out_path);
    csv << "t,x,y,yaw,vx,vy,r,omega_f,omega_r,Fz_FL,Fz_FR,Fz_RL,Fz_RR\n";

    const int N = static_cast<int>(std::round(scen.duration / scen.dt));
    for (int i = 0; i <= N; ++i) {
        const auto& s = dyn->state();
        const auto Fz = dyn->tire_Fz();
        csv << (i * scen.dt) << ','
            << s.position.x() << ',' << s.position.y() << ','
            << vdsim::yaw_from_quat(s.orientation) << ','
            << s.velocity.x() << ',' << s.velocity.y() << ','
            << s.angular_velocity.z() << ','
            << s.wheel_spin[vdsim::WHEEL_FL] << ',' << s.wheel_spin[vdsim::WHEEL_RL] << ','
            << Fz[0] << ',' << Fz[1] << ',' << Fz[2] << ',' << Fz[3] << '\n';
        if (i < N) dyn->step(u, contacts, scen.dt);
    }

    std::fprintf(stderr,
        "[vdsim_bicycle_run] %s on %s: vx %.3f -> %.3f m/s, r %.4f rad/s, %d samples\n",
        scen_name.c_str(),
        vehicle_path.c_str(),
        scen.vx0, dyn->state().velocity.x(), dyn->state().yaw_rate(),
        N + 1);
    return 0;
}
