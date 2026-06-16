// Local-only demo dumper: runs the L5 spatial-strut path (C++ core) through the
// phase-C jump and loop scenarios and writes telemetry CSV for animation.
//   usage: strut_demo_dump jump|loop > out.csv
// Not a committed artifact pipeline; companion to apps/jump_demo/animate_strut.py.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

using namespace vdsim;

namespace {
constexpr double G = 9.80665;

State on_flat(double x, double vx, double cg_z, double R) {
    State s;
    s.position.x() = x;
    s.position.z() = cg_z;
    s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

SolverParams strut_solver() {
    SolverParams sp;
    sp.stunt_physics = true;
    sp.l5_spatial_suspension = true;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 16;
    return sp;
}

void emit(double t, const IVehicleDynamics& dyn) {
    const auto& st = dyn.state();
    const Mat3 R = st.orientation.toRotationMatrix();
    const Vec3 vw = R * st.velocity;
    const Vec3 fwd = R.col(0);   // body forward axis in world (continuous heading)
    const auto fz = dyn.tire_Fz();
    const auto& cp = st.susp_compression;   // strut travel [m], +compressed
    const double fsum = fz[0] + fz[1] + fz[2] + fz[3];
    const double wspin = 0.25 * (st.wheel_spin[0] + st.wheel_spin[1]
                               + st.wheel_spin[2] + st.wheel_spin[3]);
    // t,x,z,pitch,vx,vz,fz_sum,fwd_x,fwd_z, comp[FL,FR,RL,RR], fz[FL,FR,RL,RR], ax,ay, wspin
    std::printf("%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.1f,%.5f,%.5f,"
                "%.6f,%.6f,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f\n",
                t, st.position.x(), st.position.z(), dyn.pitch_angle_qs(),
                vw.x(), vw.z(), fsum, fwd.x(), fwd.z(),
                cp[0], cp[1], cp[2], cp[3], fz[0], fz[1], fz[2], fz[3],
                dyn.ax_body_est(), dyn.ay_body_est(), wspin);
}
}  // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "jump";
    VehicleParams vp; vp.aero_drag_coeff = 0.0;
    TireParams tp; tp.lugre.enabled = false;
    SolverParams sp = strut_solver();

    std::printf("t,x,z,pitch,vx,vz,fz_sum,fwd_x,fwd_z,"
                "comp_fl,comp_fr,comp_rl,comp_rr,"
                "fz_fl,fz_fr,fz_rl,fz_rr,ax,ay\n");

    if (mode == "loop" || mode == "loopfail") {
        const double R = 10.0, xc = 50.0, zc = 15.0;
        sp.loop_radius = R; sp.loop_center_x = xc; sp.loop_center_z = zc;
        auto dyn = create_stunt_dof();
        dyn->initialize(vp, tp, sp);
        // "loop": above the critical speed (completes). "loopfail": below it + no drive,
        // so the car climbs partway, the contact normal can no longer hold it, and it
        // detaches and falls — the emergent loss of the loop.
        const bool fail = (mode == "loopfail");
        const double v0 = (fail ? 0.62 : 1.15) * std::sqrt(5.0 * G * R);
        const double z0 = zc - R + vp.cg_height + 0.005;
        dyn->reset(on_flat(xc, v0, z0, vp.wheel_radius_nominal));
        auto loop = create_loop_ground(xc, zc, R, 1.2);
        CmdL4 cmd; cmd.throttle = fail ? 0.0 : 0.5;
        double t = 0.0; const double dt = 0.001;
        for (int i = 0; i < 8000; ++i) {
            ContactArray c; loop->query(dyn->state(), vp, c);
            dyn->step(cmd, c, dt); t += dt;
            if (i % 10 == 0) emit(t, *dyn);
        }
    } else if (mode == "flatsteer") {
        auto dyn = create_stunt_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(on_flat(0.0, 15.0, vp.cg_height, vp.wheel_radius_nominal));
        auto ground = create_flat_ground(0.0, 1.0);
        CmdL4 cmd; cmd.steer_angle_wheel = 0.03;
        const double dt = 0.002;
        std::printf("t,yaw,yawrate,vy,roll,pitch,z\n");
        for (int i = 0; i < 1500; ++i) {
            ContactArray c; ground->query(dyn->state(), vp, c);
            dyn->step(cmd, c, dt);
            if (i % 20 == 0) {
                const auto& st = dyn->state();
                std::printf("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                            i * dt, st.yaw(), st.angular_velocity.z(), st.velocity.y(),
                            dyn->roll_angle_qs(), dyn->pitch_angle_qs(), st.position.z());
            }
        }
        return 0;
    } else {
        auto dyn = create_stunt_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(on_flat(10.0, 16.0, vp.cg_height, vp.wheel_radius_nominal));
        auto ramp = create_ramp_ground(20.0, 24.0, 0.6, 0.4, 1.0);
        CmdL4 cmd; cmd.throttle = 0.1;
        double t = 0.0; const double dt = 0.001;
        for (int i = 0; i < 5000; ++i) {
            ContactArray c; ramp->query(dyn->state(), vp, c);
            dyn->step(cmd, c, dt); t += dt;
            if (i % 10 == 0) emit(t, *dyn);
        }
    }
    return 0;
}
