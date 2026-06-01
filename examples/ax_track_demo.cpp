// Closed-loop ax tracking demo: PID controller drives throttle/brake to follow
// a piecewise-constant ax_target on the L2 7-DOF dynamics.

#include "vdsim/control_converter.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

namespace {

vdsim::ContactArray flat_contacts() {
    vdsim::ContactArray c;
    for (auto& p : c) { p.is_valid = true; p.normal = vdsim::Vec3::UnitZ();
                       p.mu_long = 1.0; p.mu_lat = 1.0; }
    return c;
}
vdsim::State init_state(double vx, double R) {
    vdsim::State s; s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

double target_profile(double t) {
    if (t < 2.0)  return  2.0;       // accel
    if (t < 5.0)  return  0.0;       // coast
    if (t < 8.0)  return -3.0;       // brake
    return 0.0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr,
            "usage: %s <vehicle.yaml> <tire.yaml> <out.csv>\n", argv[0]);
        return 2;
    }
    const auto vp = vdsim::VehicleParams::from_yaml(argv[1]);
    const auto tp = vdsim::TireParams::from_yaml(argv[2]);
    const vdsim::SolverParams sp;

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(2.0, vp.wheel_radius_nominal));

    vdsim::LongAxController ctrl;
    ctrl.initialize({});

    std::ofstream csv(argv[3]);
    csv << "t,ax_target,ax_meas,throttle,brake,vx\n";

    const double T = 10.0, dt = 0.005;
    const int N = static_cast<int>(std::round(T / dt));
    const auto contacts = flat_contacts();
    for (int i = 0; i <= N; ++i) {
        const double t = i * dt;
        const double a_tgt = target_profile(t);
        const double a_meas = dyn->ax_body_est();
        const auto [thr, brk] = ctrl.update(a_tgt, a_meas, dt);

        csv << t << ',' << a_tgt << ',' << a_meas << ','
            << thr << ',' << brk << ',' << dyn->state().velocity.x() << '\n';

        if (i < N) {
            vdsim::CmdL4 cmd; cmd.throttle = thr; cmd.brake = brk;
            const vdsim::ControlInput u = cmd;
            dyn->step(u, contacts, dt);
        }
    }
    std::fprintf(stderr, "[vdsim_ax_track_demo] wrote %d samples to %s\n",
                 N + 1, argv[3]);
    return 0;
}
