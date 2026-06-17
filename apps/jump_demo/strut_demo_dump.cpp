// Local-only demo dumper: runs the L5 spatial-strut path (C++ core) through the
// phase-C jump and loop scenarios and writes telemetry CSV for animation.
//   usage: strut_demo_dump jump|loop > out.csv
// Not a committed artifact pipeline; companion to apps/jump_demo/animate_strut.py.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/params.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
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
    sp.l5_spatial_suspension = std::getenv("VDSIM_PENALTY") ? false : true;
    sp.max_substep_dt = std::getenv("VDSIM_SUBDT") ? std::stod(std::getenv("VDSIM_SUBDT")) : 2e-4;
    sp.max_substeps = 64;
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
    // t,x,z,pitch,vx,vz,fz_sum,fwd_x,fwd_z, comp[FL,FR,RL,RR], fz[FL,FR,RL,RR], ax,ay, wspin,
    // then the 4 unsprung (wheel-centre) world x,z [FL,FR,RL,RR] for rendering.
    const auto& up = st.unsprung_pos;
    std::printf("%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.1f,%.5f,%.5f,"
                "%.6f,%.6f,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                t, st.position.x(), st.position.z(), dyn.pitch_angle_qs(),
                vw.x(), vw.z(), fsum, fwd.x(), fwd.z(),
                cp[0], cp[1], cp[2], cp[3], fz[0], fz[1], fz[2], fz[3],
                dyn.ax_body_est(), dyn.ay_body_est(), wspin,
                up[0].x(), up[0].z(), up[1].x(), up[1].z(),
                up[2].x(), up[2].z(), up[3].x(), up[3].z());
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
        const double v0fac = std::getenv("VDSIM_V0") ? std::stod(std::getenv("VDSIM_V0"))
                                                     : (fail ? 0.62 : 1.15);
        const double v0 = v0fac * std::sqrt(5.0 * G * R);
        const double z0 = zc - R + vp.cg_height + 0.005;
        dyn->reset(on_flat(xc, v0, z0, vp.wheel_radius_nominal));
        auto loop = create_loop_ground(xc, zc, R, 1.2);
        if (std::getenv("VDSIM_REQ")) free_3d_attach_contact_provider(*dyn, loop.get());
        CmdL4 cmd; cmd.throttle = std::getenv("VDSIM_THR") ? std::stod(std::getenv("VDSIM_THR"))
                                                           : (fail ? 0.0 : 0.5);
        double t = 0.0;
        const double dt = std::getenv("VDSIM_DT") ? std::stod(std::getenv("VDSIM_DT")) : 0.001;
        const bool edump = std::getenv("VDSIM_EDUMP") != nullptr;
        if (edump) std::printf("t,arc,z,speed,sumFz_over_W,KEt,KEr,PEg,PEs,Etot\n");
        double unwrap = 0.0, prev = 0.0; bool have = false;
        for (int i = 0; i < 8000; ++i) {
            ContactArray c; loop->query(dyn->state(), vp, c);
            dyn->step(cmd, c, dt); t += dt;
            const auto& st = dyn->state();
            const double th = std::atan2(st.position.x() - xc, -(st.position.z() - zc));
            if (have) { double d = th - prev; if (d > M_PI) d -= 2*M_PI; if (d < -M_PI) d += 2*M_PI; unwrap += d; }
            else { unwrap = th; have = true; }
            prev = th;
            if (edump && i % 10 == 0) {
                const Mat3 Rm = st.orientation.toRotationMatrix();
                const Vec3 vw = Rm * st.velocity;
                const Vec3 w = st.angular_velocity;
                // Full ledger: body = sprung mass; each unsprung is its own particle.
                double mu_sum = 0.0; for (int k=0;k<4;++k) mu_sum += vp.unsprung_mass[k];
                const double m_body = vp.mass - mu_sum;
                double KEt = 0.5 * m_body * vw.squaredNorm();
                double PEg = m_body * G * st.position.z();
                for (int k = 0; k < 4; ++k) {
                    KEt += 0.5 * vp.unsprung_mass[k] * st.unsprung_vel[k].squaredNorm();
                    PEg += vp.unsprung_mass[k] * G * st.unsprung_pos[k].z();
                }
                const double KEr = 0.5 * (vp.inertia_diag.x()*w.x()*w.x()
                                        + vp.inertia_diag.y()*w.y()*w.y()
                                        + vp.inertia_diag.z()*w.z()*w.z());
                // Spring PE from natural (unloaded) length: F=preload+k*c=k*(c-c_nat).
                const double ms = vp.mass_sprung > 1.0 ? vp.mass_sprung : vp.mass;
                const double Lwb = vp.wheelbase > 1e-6 ? vp.wheelbase : 2.7;
                const double Fpre_f = 0.5*ms*G*vp.cg_to_rear/Lwb, Fpre_r = 0.5*ms*G*vp.cg_to_front/Lwb;
                double PEs = 0.0;
                for (int k = 0; k < 4; ++k) {
                    const double Fpre = (k<2)?Fpre_f:Fpre_r;
                    const double cnat = -Fpre / std::max(1.0, vp.spring_stiffness[k]);
                    const double dc = st.susp_compression[k] - cnat;
                    PEs += 0.5 * vp.spring_stiffness[k] * dc * dc;
                }
                const auto fz = dyn->tire_Fz();
                const double sumFz = fz[0] + fz[1] + fz[2] + fz[3];
                std::printf("%.4f,%.4f,%.4f,%.3f,%.4f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                            t, unwrap, st.position.z(), vw.norm(), sumFz/(vp.mass*G),
                            KEt, KEr, PEg, PEs, KEt+KEr+PEg+PEs);
            } else if (!edump && i % 10 == 0) emit(t, *dyn);
        }
    } else if (mode == "airspin" || mode == "airspin0") {
        // Free-floating rotating body, NO contact (high above ground). Tests whether the
        // coupled M/b inertial assembly conserves angular momentum / energy on its own.
        if (mode == "airspin0") {            // tiny unsprung mass: pure sprung-body Euler
            vp.unsprung_mass = {{0.01, 0.01, 0.01, 0.01}};
        }
        auto dyn = create_stunt_dof();
        dyn->initialize(vp, tp, sp);
        State s0;
        s0.position = Vec3(0.0, 0.0, 100.0);
        const double wx = argc > 2 ? std::stod(argv[2]) : 1.0;
        const double wy = argc > 3 ? std::stod(argv[3]) : 0.3;
        const double wz = argc > 4 ? std::stod(argv[4]) : 0.5;
        s0.angular_velocity = Vec3(wx, wy, wz);   // roll/pitch/yaw rate
        dyn->reset(s0);
        auto ground = create_flat_ground(0.0, 1.0);
        CmdL4 cmd;
        const double dt = 0.002;
        std::printf("t,wx,wy,wz,wmag,z,vz\n");
        for (int i = 0; i < 1500; ++i) {
            ContactArray c; ground->query(dyn->state(), vp, c);
            dyn->step(cmd, c, dt);
            if (i % 20 == 0) {
                const auto& st = dyn->state();
                const Vec3 w = st.angular_velocity;
                std::printf("%.3f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f\n",
                            i * dt, w.x(), w.y(), w.z(), w.norm(),
                            st.position.z(), st.velocity.z());
            }
        }
        return 0;
    } else if (mode == "rolltest") {
        // Settle straight, then inject a pure body roll-rate and watch whether it decays
        // (positive roll damping) or grows (NEGATIVE roll damping = the bug). No steer.
        auto dyn = create_stunt_dof();
        dyn->initialize(vp, tp, sp);
        dyn->reset(on_flat(0.0, 15.0, vp.cg_height, vp.wheel_radius_nominal));
        auto ground = create_flat_ground(0.0, 1.0);
        CmdL4 cmd;
        cmd.steer_angle_wheel = std::getenv("VDSIM_STEER") ? std::stod(std::getenv("VDSIM_STEER")) : 0.0;
        const double dt = 0.001;
        for (int i = 0; i < 800; ++i) {   // settle 0.8 s into steady turn
            ContactArray c; ground->query(dyn->state(), vp, c);
            dyn->step(cmd, c, dt);
        }
        State s = dyn->state();
        s.angular_velocity.x() += 0.5;    // inject roll rate on top of steady turn
        dyn->reset(s);
        const double W = vp.mass * G;
        std::printf("t,roll,rollrate,sumFz_over_W,sumComp,cFL,cFR,cRL,cRR\n");
        for (int i = 0; i < 400; ++i) {
            ContactArray c; ground->query(dyn->state(), vp, c);
            dyn->step(cmd, c, dt);
            if (i % 2 == 0) {
                const auto& st = dyn->state();
                const auto fz = dyn->tire_Fz();
                const auto& cp = st.susp_compression;
                const double sumFz = fz[0] + fz[1] + fz[2] + fz[3];
                const double sumC = cp[0] + cp[1] + cp[2] + cp[3];
                std::printf("%.4f,%.5f,%.5f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                            i * dt, dyn->roll_angle_qs(), st.angular_velocity.x(),
                            sumFz / W, sumC, cp[0], cp[1], cp[2], cp[3]);
            }
        }
        return 0;
    } else if (mode == "flatsteer") {
        auto dyn = create_stunt_dof();
        dyn->initialize(vp, tp, sp);
        if (std::getenv("VDSIM_DAE")) {
            const std::string base = "/home/ailab-12/git/VDSim/configs/parts/susp_kinematics/kin/";
            auto front = mb::SuspensionTopology::from_yaml(base + "mp_front_sedan.yaml");
            auto rear  = mb::SuspensionTopology::from_yaml(base + "ta_rear_sedan.yaml");
            free_3d_attach_multibody(*dyn, true,  front, true);
            free_3d_attach_multibody(*dyn, false, rear,  true);
        }
        dyn->reset(on_flat(0.0, 15.0, vp.cg_height, vp.wheel_radius_nominal));
        auto ground = create_flat_ground(0.0, 1.0);
        CmdL4 cmd;
        cmd.steer_angle_wheel = std::getenv("VDSIM_STEER") ? std::stod(std::getenv("VDSIM_STEER")) : 0.03;
        const double dt = 0.002;
        std::printf("t,roll,z,speed,KE,PEg,PEs,Etot,ay\n");
        for (int i = 0; i < 1500; ++i) {
            ContactArray c; ground->query(dyn->state(), vp, c);
            dyn->step(cmd, c, dt);
            if (i % 5 == 0) {
                const auto& st = dyn->state();
                const Mat3 R = st.orientation.toRotationMatrix();
                const Vec3 vw = R * st.velocity;
                const Vec3 w = st.angular_velocity;
                const double KEt = 0.5 * vp.mass * vw.squaredNorm();
                const double KEr = 0.5 * (vp.inertia_diag.x()*w.x()*w.x()
                                        + vp.inertia_diag.y()*w.y()*w.y()
                                        + vp.inertia_diag.z()*w.z()*w.z());
                const double PEg = vp.mass * G * st.position.z();
                double PEs = 0.0;
                for (int k = 0; k < 4; ++k) {
                    const double cmp = st.susp_compression[k];
                    PEs += 0.5 * vp.spring_stiffness[k] * cmp * cmp;
                }
                const double KE = KEt + KEr;
                std::printf("%.3f,%.5f,%.5f,%.3f,%.1f,%.1f,%.1f,%.1f,%.4f\n",
                            i * dt, dyn->roll_angle_qs(), st.position.z(), vw.norm(),
                            KE, PEg, PEs, KE + PEg + PEs, dyn->ay_body_est());
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
