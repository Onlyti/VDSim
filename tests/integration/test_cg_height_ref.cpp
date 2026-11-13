// cg_height = whole-vehicle CG on the L5 strut / DAE path (work order 47, 0.8.2): G2, G5.
//
// Gates fixed before measurement (work order 47 sec. 3, PO 1st verdict item 2):
//   G2  static height: the car at rest on flat ground, mean over [1, 2) s. Mass-weighted
//       system CG height z_sys = ((m - sum m_u) z_body + sum m_u z_u,i) / m must satisfy
//       |z_sys - (cg_height - delta)| <= 1 mm, delta = mean over the four wheels of the
//       static tire penetration R - z_u,i (flat ground at z = 0).
//   G5  drag height: on the strut / DAE body the aero drag acts at cg_height above ground
//       (four-key path) or at aero_drag_height when that key is set. Measured as a pitch
//       moment: the car in the air (no contact), one 1 us step with and without drag
//       from the same state; tau_y = Iyy * d(omega_y) / dt is the drag moment about the
//       body origin, z_d = -tau_y / F_d its arm, and the ground height of the drag line is
//       z_d + (R - z_mount) with z_mount the body-frame wheel-centre height read from the
//       reset state (the body origin is R - z_mount above ground at design attitude).
//       Gate |height - want| <= 1 mm, i.e. a pitch-moment error <= F_d * 1 mm.
// m_u = max(1, unsprung_mass[i]) as in the core. Cars: generic_sedan and fs_aero_example
// (aero emptied for G2). DAE path: test-only DW front sample + TA rear, as in R2.
#include "vdsim/aero.hpp"
#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;

enum class Car { Sedan, FS };

const char* car_name(Car c) { return c == Car::Sedan ? "sedan" : "fs"; }

VehicleParams car(Car c) {
    VehicleParams vp = VehicleParams::from_yaml(
        kSrc + (c == Car::Sedan ? "/configs/vehicles/generic_sedan.yaml"
                                : "/configs/vehicles/fs_aero_example.yaml"));
    vp.aero_elements.clear();
    vp.aero_drag_coeff = 0.0;
    vp.aero_lift_front = vp.aero_lift_rear = 0.0;
    vp.aero_drag_height.reset();
    return vp;
}

TireParams tire() {
    return TireParams::from_yaml(kSrc + "/configs/parts/tire/generic_pacejka.yaml");
}

SolverParams strut_solver() {
    SolverParams sp;
    sp.stunt_physics = true;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 16;
    sp.l5_spatial_suspension = true;
    return sp;
}

/// Attach the R2 corner models (DW front sample, TA rear) to an L5 strut model.
void attach_dae(IVehicleDynamics& dyn) {
    free_3d_attach_multibody(dyn, true,
        mb::SuspensionTopology::from_yaml(kSrc + "/tests/fixtures/dw_front_antidive_sample.yaml"),
        true);
    free_3d_attach_multibody(dyn, false,
        mb::SuspensionTopology::from_yaml(kSrc +
                                          "/configs/parts/susp_kinematics/kin/ta_rear_sedan.yaml"),
        true);
}

double m_u(const VehicleParams& vp, int i) { return std::max(1.0, vp.unsprung_mass[i]); }

struct Static {
    double z_sys {0.0};    // mass-weighted system CG height [m]
    double delta {0.0};    // mean static tire penetration [m]
    bool finite {true};
};

/// G2: the car at rest for 2 s at 1 ms ticks; means over [1, 2) s.
Static at_rest(const VehicleParams& vp, bool dae) {
    SimConfig cfg;
    cfg.nominal_dt = 1e-3;
    SimSession s(create_stunt_dof(), create_flat_ground(0.0, 1.0), vp, TireSetup(tire()),
                 strut_solver(), cfg);
    if (dae) attach_dae(s.dynamics());
    State s0;
    s0.position.z() = vp.cg_height;
    s.reset_settled(s0);
    double sum_mu = 0.0;
    for (int i = 0; i < NUM_WHEELS; ++i) sum_mu += m_u(vp, i);
    const double m_body = std::max(1.0, vp.mass - sum_mu);
    const double R = vp.wheel_radius_nominal;
    Static r;
    int n = 0;
    for (int k = 0; k < 2000; ++k) {
        s.set_input(CmdL4{});
        s.tick(1e-3);
        if (k < 1000) continue;
        const auto o = s.output();
        double zm = m_body * o.state.position.z(), pen = 0.0;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            zm += m_u(vp, i) * o.state.unsprung_pos[i].z();
            pen += R - o.state.unsprung_pos[i].z();
        }
        r.z_sys += zm / (m_body + sum_mu);
        r.delta += 0.25 * pen;
        r.finite = r.finite && std::isfinite(zm);
        ++n;
    }
    r.z_sys /= n;
    r.delta /= n;
    return r;
}

void expect_g2(Car c, bool dae) {
    const auto vp = car(c);
    const auto r = at_rest(vp, dae);
    ASSERT_TRUE(r.finite);
    const double want = vp.cg_height - r.delta;
    std::printf("[G2 %s %s] z_sys %.6f m, cg_height - delta %.6f m (delta %.6f m), "
                "diff %+.4f mm, h_s %.6f m\n",
                car_name(c), dae ? "dae" : "strut", r.z_sys, want, r.delta,
                1e3 * (r.z_sys - want), l5_strut_body_height(vp));
    EXPECT_NEAR(r.z_sys, want, 1e-3);
}

constexpr double kAirDt = 1e-6;   // [s] one step, below one substep
constexpr double kAirV  = 30.0;   // [m/s]

/// Body-frame pitch rate after one in-air step from rest attitude at kAirV.
/// Also returns the body-frame wheel-centre height (z of FL relative to the origin).
double air_pitch_rate(const VehicleParams& vp, bool dae, double* z_mount) {
    auto dyn = create_stunt_dof();
    dyn->initialize(vp, TireSetup(tire()), strut_solver());
    if (dae) attach_dae(*dyn);
    State s;
    s.position.z() = 5.0;                       // far above the (absent) ground
    s.velocity.x() = kAirV;
    const double w = kAirV / vp.wheel_radius_nominal;
    s.wheel_spin = {{w, w, w, w}};
    dyn->reset(s);
    if (z_mount) *z_mount = dyn->state().unsprung_pos[WHEEL_FL].z() - dyn->state().position.z();
    ContactArray none {};
    dyn->step(ControlInput{CmdL4{}}, none, kAirDt);
    return dyn->state().angular_velocity.y();
}

/// G5: ground height of the drag line on the strut / DAE body [m].
double drag_height(const VehicleParams& with, bool dae) {
    VehicleParams without = with;
    without.aero_elements.clear();
    without.aero_drag_coeff = 0.0;
    without.aero_lift_front = without.aero_lift_rear = 0.0;
    double z_mount = 0.0;
    const double w1 = air_pitch_rate(with, dae, &z_mount);
    const double w0 = air_pitch_rate(without, dae, nullptr);
    const double tau = with.inertia_diag.y() * (w1 - w0) / kAirDt;
    const double fd = compute_aero_loads(with, AeroState{kAirV}).fx_total;
    return -tau / fd + (with.wheel_radius_nominal - z_mount);
}

void expect_g5(Car c, bool dae, bool drag_key) {
    auto vp = car(c);
    vp.aero_drag_coeff = 1.0;
    vp.frontal_area = 1.0;
    double want = vp.cg_height;
    if (drag_key) {
        vp.aero_drag_height = 0.40;
        want = 0.40;
    }
    const double h = drag_height(vp, dae);
    std::printf("[G5 %s %s %s] drag height %.6f m (want %.6f), diff %+.4f mm, h_s %.6f m\n",
                car_name(c), dae ? "dae" : "strut", drag_key ? "aero_drag_height" : "four-key",
                h, want, 1e3 * (h - want), l5_strut_body_height(vp));
    EXPECT_NEAR(h, want, 1e-3);
}

}  // namespace

// ---- G2: static system CG height on the strut / DAE path ----
TEST(CgHeightRef, G2_StaticHeightSedanStrut) { expect_g2(Car::Sedan, false); }
TEST(CgHeightRef, G2_StaticHeightSedanDae)   { expect_g2(Car::Sedan, true); }
TEST(CgHeightRef, G2_StaticHeightFSStrut)    { expect_g2(Car::FS, false); }
TEST(CgHeightRef, G2_StaticHeightFSDae)      { expect_g2(Car::FS, true); }

// ---- G5: drag acts at cg_height (or aero_drag_height) above ground ----
TEST(CgHeightRef, G5_FourKeyDragHeightStrut) {
    expect_g5(Car::Sedan, false, false);
    expect_g5(Car::FS, false, false);
}
TEST(CgHeightRef, G5_FourKeyDragHeightDae) {
    expect_g5(Car::Sedan, true, false);
    expect_g5(Car::FS, true, false);
}
TEST(CgHeightRef, G5_DragHeightKeyStrut) {
    expect_g5(Car::Sedan, false, true);
    expect_g5(Car::FS, false, true);
}
TEST(CgHeightRef, G5_DragHeightKeyDae) {
    expect_g5(Car::Sedan, true, true);
    expect_g5(Car::FS, true, true);
}
