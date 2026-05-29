// L8 closed-loop path tracking demo.
//
// Pipeline:
//   waypoints (figure-eight or other) -> PurePursuit -> steer
//   v_target constant                 -> L6 vx PID    -> ax_target
//   ax_target                         -> L5 ax PID    -> throttle/brake
//   -> L2 7-DOF dynamics
//
// Usage:
//   vdsim_path_tracking <vehicle.yaml> <tire.yaml> <out.csv> [v_target]

#include "vdsim/control_converter.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
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

// Figure-eight path: two circles tangent at origin.
void make_figure_eight(double R, int n_per_loop,
                       std::vector<double>& px, std::vector<double>& py) {
    // First loop: center (R, 0), goes counter-clockwise from (0, 0).
    for (int i = 0; i < n_per_loop; ++i) {
        const double t = 2.0 * kPi * (double)i / (double)n_per_loop;
        px.push_back(R - R * std::cos(t));
        py.push_back(R * std::sin(t));
    }
    // Second loop: center (-R, 0), clockwise.
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
            "usage: %s <vehicle.yaml> <tire.yaml> <out.csv> [v_target=8]\n", argv[0]);
        return 2;
    }
    const auto vp = vdsim::VehicleParams::from_yaml(argv[1]);
    const auto tp = vdsim::TireParams::from_yaml(argv[2]);
    const double v_tgt = (argc >= 5) ? std::atof(argv[4]) : 8.0;
    const vdsim::SolverParams sp;

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(v_tgt, vp.wheel_radius_nominal));

    vdsim::PurePursuitController pp;
    vdsim::PurePursuitController::Gains pg;
    pg.wheelbase = vp.wheelbase; pg.max_steer = vp.max_steer_angle_wheel;
    pg.lookahead_min = 2.0; pg.lookahead_k = 0.45;
    pp.initialize(pg);

    vdsim::LongVxController vxc; vxc.initialize({});
    vdsim::LongAxController  axc; axc.initialize({});

    std::vector<double> px, py;
    make_figure_eight(20.0, 80, px, py);          // R=20 m, 160 pts total

    std::ofstream csv(argv[3]);
    csv << "t,x,y,yaw,vx,target_x,target_y,steer_cmd,ax_target,throttle,brake,xtrack_err\n";

    const double T = 25.0, dt = 0.005;
    const int N = static_cast<int>(std::round(T / dt));
    const auto contacts = flat_contacts();
    int prev_idx = 0;

    for (int i = 0; i <= N; ++i) {
        const auto& s = dyn->state();
        const double x   = s.position.x();
        const double y   = s.position.y();
        const double yaw = vdsim::yaw_from_quat(s.orientation);
        const double vx  = s.velocity.x();

        // L7 Pure Pursuit
        const auto pp_out = pp.update(x, y, yaw, vx,
                                       px.data(), py.data(), (int)px.size(), prev_idx);
        prev_idx = pp_out.idx;

        // L6 v_target -> ax_target
        const double ax_tgt = vxc.update(v_tgt, vx, dt);

        // L5 ax_target -> throttle / brake
        const auto [thr, brk] = axc.update(ax_tgt, dyn->ax_body_est(), dt);

        // Compute cross-track error (distance from vehicle to current target point)
        const double xtrack = std::sqrt(
            (px[pp_out.idx] - x) * (px[pp_out.idx] - x) +
            (py[pp_out.idx] - y) * (py[pp_out.idx] - y));

        csv << (i*dt) << ',' << x << ',' << y << ',' << yaw << ',' << vx << ','
            << px[pp_out.idx] << ',' << py[pp_out.idx] << ','
            << pp_out.steer << ',' << ax_tgt << ',' << thr << ',' << brk << ','
            << xtrack << '\n';

        if (i < N) {
            vdsim::CmdL4 cmd;
            cmd.throttle = thr; cmd.brake = brk;
            cmd.steer_angle_wheel = pp_out.steer;
            const vdsim::ControlInput u = cmd;
            dyn->step(u, contacts, dt);
        }
    }
    std::fprintf(stderr,
        "[path_tracking] v_target=%.1f, final vx=%.2f, %d samples\n",
        v_tgt, dyn->state().velocity.x(), N + 1);
    return 0;
}
