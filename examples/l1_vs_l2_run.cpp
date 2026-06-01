// Run the same scenario on both L1 and L2 dynamics and emit two CSVs.
//
// Usage:
//   vdsim_l1_vs_l2 <vehicle.yaml> <tire.yaml> <scenario.yaml> <out_dir>

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/scenario.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
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

void run_one(vdsim::IVehicleDynamics& dyn, const vdsim::Scenario& sc,
             const vdsim::VehicleParams& vp, const std::string& out) {
    std::ofstream csv(out);
    csv << "t,x,y,yaw,vx,vy,r,Fz_FL,Fz_FR,Fz_RL,Fz_RR,ax,ay,roll,pitch\n";
    dyn.reset(init_state(sc.initial_vx, vp.wheel_radius_nominal));
    const auto contacts = flat_contacts(sc.mu);
    const int N = static_cast<int>(std::round(sc.duration / sc.dt));
    for (int i = 0; i <= N; ++i) {
        const double t = i * sc.dt;
        const auto cs = sc.sample(t);
        vdsim::CmdL4 cmd; cmd.throttle = cs.throttle; cmd.brake = cs.brake;
        cmd.steer_angle_wheel = cs.steer; cmd.gear = cs.gear;
        const vdsim::ControlInput u = cmd;
        const auto& s = dyn.state();
        const auto Fz = dyn.tire_Fz();
        csv << t << ',' << s.position.x() << ',' << s.position.y() << ','
            << vdsim::yaw_from_quat(s.orientation) << ','
            << s.velocity.x() << ',' << s.velocity.y() << ','
            << s.angular_velocity.z() << ','
            << Fz[0] << ',' << Fz[1] << ',' << Fz[2] << ',' << Fz[3] << ','
            << dyn.ax_body_est() << ',' << dyn.ay_body_est() << ','
            << dyn.roll_angle_qs() << ',' << dyn.pitch_angle_qs() << '\n';
        if (i < N) dyn.step(u, contacts, sc.dt);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 5) {
        std::fprintf(stderr,
            "usage: %s <vehicle.yaml> <tire.yaml> <scenario.yaml> <out_dir>\n",
            argv[0]);
        return 2;
    }
    const auto vp = vdsim::VehicleParams::from_yaml(argv[1]);
    const auto tp = vdsim::TireParams::from_yaml(argv[2]);
    const auto sc = vdsim::Scenario::from_yaml(argv[3]);
    const std::string out_dir = argv[4];
    const vdsim::SolverParams sp;

    auto l1 = vdsim::create_bicycle();
    auto l2 = vdsim::create_seven_dof();
    l1->initialize(vp, tp, sp);
    l2->initialize(vp, tp, sp);

    run_one(*l1, sc, vp, out_dir + "/" + sc.name + "_L1.csv");
    run_one(*l2, sc, vp, out_dir + "/" + sc.name + "_L2.csv");

    std::fprintf(stderr,
        "[vdsim_l1_vs_l2] %s: L1 vx_end=%.3f r_end=%.4f  L2 vx_end=%.3f r_end=%.4f\n",
        sc.name.c_str(),
        l1->state().velocity.x(), l1->state().yaw_rate(),
        l2->state().velocity.x(), l2->state().yaw_rate());
    return 0;
}
