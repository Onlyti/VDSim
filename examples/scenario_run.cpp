// CLI: run a YAML-defined scenario.
//
// Usage:
//   vdsim_scenario_run <vehicle.yaml> <tire.yaml> <scenario.yaml> <out.csv>
//                      [solver.yaml]
//
// CSV columns: t,x,y,yaw,vx,vy,r,throttle,brake,steer,
//              Fz_FL,Fz_FR,Fz_RL,Fz_RR

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/scenario.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

vdsim::ContactArray flat_contacts(double mu) {
    vdsim::ContactArray c;
    for (auto& p : c) {
        p.is_valid = true; p.normal = vdsim::Vec3::UnitZ();
        p.mu_long  = mu;   p.mu_lat  = mu;
    }
    return c;
}

vdsim::State init_state(double vx, double R) {
    vdsim::State s; s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 5) {
        std::fprintf(stderr,
            "usage: %s <vehicle.yaml> <tire.yaml> <scenario.yaml> <out.csv> [solver.yaml]\n",
            argv[0]);
        return 2;
    }
    const std::string vp_path = argv[1];
    const std::string tp_path = argv[2];
    const std::string sc_path = argv[3];
    const std::string out_path = argv[4];

    const auto vp = vdsim::VehicleParams::from_yaml(vp_path);
    const auto tp = vdsim::TireParams::from_yaml(tp_path);
    const auto sc = vdsim::Scenario::from_yaml(sc_path);
    vdsim::SolverParams sp;
    if (argc >= 6) sp = vdsim::SolverParams::from_yaml(argv[5]);

    auto dyn = vdsim::create_bicycle();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(sc.initial_vx, vp.wheel_radius_nominal));

    auto contacts = flat_contacts(sc.mu);

    std::ofstream csv(out_path);
    if (!csv) throw std::runtime_error("cannot open output CSV: " + out_path);
    csv << "t,x,y,yaw,vx,vy,r,throttle,brake,steer,"
           "Fz_FL,Fz_FR,Fz_RL,Fz_RR,ax,ay,roll,pitch\n";

    const int N = static_cast<int>(std::round(sc.duration / sc.dt));
    for (int i = 0; i <= N; ++i) {
        const double t = i * sc.dt;
        const auto cs = sc.sample(t);
        vdsim::CmdL4 cmd;
        cmd.throttle = cs.throttle;
        cmd.brake    = cs.brake;
        cmd.steer_angle_wheel = cs.steer;
        cmd.gear = cs.gear;
        const vdsim::ControlInput u = cmd;

        const auto& s = dyn->state();
        const auto Fz = dyn->tire_Fz();
        csv << t << ','
            << s.position.x() << ',' << s.position.y() << ','
            << vdsim::yaw_from_quat(s.orientation) << ','
            << s.velocity.x() << ',' << s.velocity.y() << ','
            << s.angular_velocity.z() << ','
            << cs.throttle << ',' << cs.brake << ',' << cs.steer << ','
            << Fz[0] << ',' << Fz[1] << ',' << Fz[2] << ',' << Fz[3] << ','
            << dyn->ax_body_est() << ',' << dyn->ay_body_est() << ','
            << dyn->roll_angle_qs() << ',' << dyn->pitch_angle_qs() << '\n';

        if (i < N) {
            const double mu_now = sc.sample_mu(t + 0.5 * sc.dt);
            for (auto& p : contacts) { p.mu_long = mu_now; p.mu_lat = mu_now; }
            dyn->step(u, contacts, sc.dt);
        }
    }

    std::fprintf(stderr,
        "[vdsim_scenario_run] %s: vx %.3f -> %.3f m/s, r %.4f rad/s, %d samples\n",
        sc.name.c_str(),
        sc.initial_vx, dyn->state().velocity.x(), dyn->state().yaw_rate(),
        N + 1);
    return 0;
}
