// L3 suspension transient demo: dump roll/pitch/heave trace.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/scenario.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

namespace {

vdsim::ContactArray flat_contacts(double mu) {
    vdsim::ContactArray c;
    for (auto& p : c) { p.is_valid = true; p.normal = vdsim::Vec3::UnitZ();
                       p.mu_long = mu; p.mu_lat = mu; }
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
            "usage: %s <vehicle.yaml> <tire.yaml> <scenario.yaml> <out.csv>\n", argv[0]);
        return 2;
    }
    const auto vp = vdsim::VehicleParams::from_yaml(argv[1]);
    const auto tp = vdsim::TireParams::from_yaml(argv[2]);
    const auto sc = vdsim::Scenario::from_yaml(argv[3]);
    const vdsim::SolverParams sp;

    auto dyn = vdsim::create_fourteen_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(sc.initial_vx, vp.wheel_radius_nominal));

    std::ofstream csv(argv[4]);
    csv << "t,vx,r,ax,ay,roll,pitch,susp_FL,susp_FR,susp_RL,susp_RR,"
           "susp_vel_FL,susp_vel_FR,susp_vel_RL,susp_vel_RR\n";

    const auto contacts = flat_contacts(sc.mu);
    const int N = static_cast<int>(std::round(sc.duration / sc.dt));
    for (int i = 0; i <= N; ++i) {
        const double t = i * sc.dt;
        const auto cs = sc.sample(t);
        vdsim::CmdL4 cmd; cmd.throttle = cs.throttle; cmd.brake = cs.brake;
        cmd.steer_angle_wheel = cs.steer; cmd.gear = cs.gear;
        const vdsim::ControlInput u = cmd;
        const auto& s = dyn->state();
        csv << t << ',' << s.velocity.x() << ',' << s.angular_velocity.z() << ','
            << dyn->ax_body_est() << ',' << dyn->ay_body_est() << ','
            << dyn->roll_angle_qs() << ',' << dyn->pitch_angle_qs() << ','
            << s.susp_compression[0] << ',' << s.susp_compression[1] << ','
            << s.susp_compression[2] << ',' << s.susp_compression[3] << ','
            << s.susp_velocity[0] << ',' << s.susp_velocity[1] << ','
            << s.susp_velocity[2] << ',' << s.susp_velocity[3] << '\n';
        if (i < N) dyn->step(u, contacts, sc.dt);
    }
    std::fprintf(stderr,
        "[l3_suspension_demo] %s: max |roll|=%.4f rad, max |pitch|=%.4f rad\n",
        sc.name.c_str(), std::abs(dyn->roll_angle_qs()), std::abs(dyn->pitch_angle_qs()));
    return 0;
}
