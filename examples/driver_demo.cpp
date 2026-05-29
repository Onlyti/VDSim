// Closed-loop driver-model demo on a figure-eight path.

#include "vdsim/control_converter.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

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

void make_figure_eight(double R, int n_per_loop,
                       std::vector<double>& px, std::vector<double>& py) {
    for (int i = 0; i < n_per_loop; ++i) {
        const double t = 2.0 * kPi * (double)i / (double)n_per_loop;
        px.push_back(R - R * std::cos(t));
        py.push_back(R * std::sin(t));
    }
    for (int i = 0; i < n_per_loop; ++i) {
        const double t = 2.0 * kPi * (double)i / (double)n_per_loop;
        px.push_back(-R + R * std::cos(t));
        py.push_back(R * std::sin(t));
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr,
            "usage: %s <vehicle.yaml> <tire.yaml> <out.csv> [v_target=8] [reaction_s=0.15]\n",
            argv[0]);
        return 2;
    }
    const auto vp = vdsim::VehicleParams::from_yaml(argv[1]);
    const auto tp = vdsim::TireParams::from_yaml(argv[2]);
    const double v_tgt = (argc >= 5) ? std::atof(argv[4]) : 8.0;
    const double reaction = (argc >= 6) ? std::atof(argv[5]) : 0.15;
    const vdsim::SolverParams sp;

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(v_tgt, vp.wheel_radius_nominal));

    vdsim::DriverModel drv;
    vdsim::DriverModel::Gains g;
    g.wheelbase       = vp.wheelbase;
    g.max_steer       = vp.max_steer_angle_wheel;
    g.lookahead_min   = 2.0; g.lookahead_k = 0.45;
    g.reaction_time_s = reaction;
    g.steer_noise_rms = 0.005;
    g.thr_noise_rms   = 0.02;
    drv.initialize(g);

    std::vector<double> px, py;
    make_figure_eight(20.0, 80, px, py);

    std::ofstream csv(argv[3]);
    csv << "t,x,y,yaw,vx,steer,throttle,brake\n";

    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    const double T = 25.0, dt = 0.005;
    const int N = static_cast<int>(std::round(T / dt));
    const auto contacts = flat_contacts();
    for (int i = 0; i <= N; ++i) {
        const auto& s = dyn->state();
        const double x = s.position.x(), y = s.position.y();
        const double yaw = vdsim::yaw_from_quat(s.orientation);
        const double vx  = s.velocity.x();
        const double a = uni(rng), b = uni(rng);
        const auto out = drv.update(x, y, yaw, vx, v_tgt,
                                     px.data(), py.data(), (int)px.size(), dt, a, b);
        csv << (i*dt) << ',' << x << ',' << y << ',' << yaw << ',' << vx << ','
            << out.steer << ',' << out.throttle << ',' << out.brake << '\n';
        if (i < N) {
            vdsim::CmdL4 cmd;
            cmd.steer_angle_wheel = out.steer;
            cmd.throttle = out.throttle; cmd.brake = out.brake;
            const vdsim::ControlInput u = cmd;
            dyn->step(u, contacts, dt);
        }
    }
    std::fprintf(stderr, "[driver_demo] v_target=%.1f, reaction=%.0fms, final vx=%.2f\n",
                 v_tgt, reaction*1000.0, dyn->state().velocity.x());
    return 0;
}
