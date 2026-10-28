// Per-element aero in the dynamics (work order 42, 0.8.0): A3, A4, A6, D1-c.
//
// Steady state: the wiki example car holds 60 km/h under CmdL6 for 12 s on
// flat ground (1 ms ticks); axle loads are averaged over 8-11 s. The
// "no aero" reference is the same car with aero.elements emptied. Expected
// axle increases come from the rigid-axle split incl. the drag pitch moment
// (front 0.294, rear 0.706 of 485.4 N); the gate is +/-2 % per axle.
#include "vdsim/aero.hpp"
#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

using namespace vdsim;

namespace {

constexpr double kV60 = 60.0 / 3.6;
constexpr double kDownTotal = 485.4;   // wiki vector, N

const std::string kSrc = VDSIM_SOURCE_DIR;

VehicleParams example_car() {
    return VehicleParams::from_yaml(kSrc + "/configs/vehicles/fs_aero_example.yaml");
}

TireParams example_tire() {
    return TireParams::from_yaml(kSrc + "/configs/parts/tire/generic_pacejka.yaml");
}

struct Steady {
    double front {0.0}, rear {0.0}, ax {0.0}, vx {0.0};
    bool finite {true};
};

std::unique_ptr<IVehicleDynamics> make_dyn(const std::string& level) {
    if (level == "L1") return create_bicycle();
    if (level == "L3") return create_fourteen_dof();
    if (level == "L4") return create_fourteen_dof_kinematic();
    if (level == "L5") return create_stunt_dof();
    return create_seven_dof();
}

/// Hold kV60 with CmdL6 for 12 s; mean axle Fz, ax and vx over 8-11 s.
Steady run_steady(const std::string& level, const VehicleParams& vp) {
    SolverParams sp;
    if (level == "L5") {
        sp.stunt_physics = true;
        sp.max_substep_dt = 2e-4;
        sp.max_substeps = 16;
    }
    SimConfig cfg;
    cfg.nominal_dt = 1e-3;
    SimSession s(make_dyn(level), create_flat_ground(0.0, 1.0), vp,
                 TireSetup(example_tire()), sp, cfg);
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity.x() = kV60;
    const double w = kV60 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    s.reset_settled(s0);
    CmdL6 cmd;
    cmd.v_target = kV60;
    Steady r;
    int n = 0;
    for (int k = 0; k < 12000; ++k) {
        s.set_input(cmd);
        s.tick(1e-3);
        if (k < 8000 || k >= 11000) continue;
        const auto o = s.output();
        r.front += o.Fz[WHEEL_FL] + o.Fz[WHEEL_FR];
        r.rear  += o.Fz[WHEEL_RL] + o.Fz[WHEEL_RR];
        r.ax += o.ax;
        r.vx += o.state.vx();
        r.finite = r.finite && std::isfinite(o.Fz[0] + o.Fz[1] + o.Fz[2] + o.Fz[3]);
        ++n;
    }
    r.front /= n; r.rear /= n; r.ax /= n; r.vx /= n;
    return r;
}

VehicleParams without_elements(VehicleParams vp) {
    vp.aero_elements.clear();
    vp.aero_drag_coeff = 0.0;   // no aero at all: the reference of A3
    return vp;
}

/// A3 on one level: axle increase vs the rigid split, +/-2 %.
void check_a3(const std::string& level) {
    const auto vp = example_car();
    const auto with = run_steady(level, vp);
    const auto none = run_steady(level, without_elements(vp));
    ASSERT_TRUE(with.finite && none.finite);
    EXPECT_LE(std::abs(with.ax), 0.02) << level;
    EXPECT_LE(std::abs(none.ax), 0.02) << level;
    const double df = with.front - none.front;
    const double dr = with.rear - none.rear;
    std::printf("[A3 %s] vx %.4f m/s ax %.5f | dFz front %.2f N (want %.2f) rear %.2f N "
                "(want %.2f) | total %.2f N\n",
                level.c_str(), with.vx, with.ax, df, 0.294 * kDownTotal, dr,
                0.706 * kDownTotal, df + dr);
    EXPECT_NEAR(df, 0.294 * kDownTotal, 0.02 * 0.294 * kDownTotal) << level;
    EXPECT_NEAR(dr, 0.706 * kDownTotal, 0.02 * 0.706 * kDownTotal) << level;
}

}  // namespace

TEST(AeroElements, A3_SteadyAxleLoadL1) { check_a3("L1"); }
TEST(AeroElements, A3_SteadyAxleLoadL2) { check_a3("L2"); }
TEST(AeroElements, A3_SteadyAxleLoadL3) { check_a3("L3"); }
// The original A3 on L5 (all elements vs no aero) is recorded as FAILED in
// the 0.8.0 acceptance: L5 applies tyre longitudinal forces at the wheel
// centre without the drive/brake torque reaction on the body, so the drive
// force that balances the drag adds a load transfer of F_d * R / L that the
// no-aero reference does not have. The cause is the L5 core, not the aero
// wrench; A3-L5a and A3-L5b below replace it (gate unchanged). Kept runnable
// with --gtest_also_run_disabled_tests to reproduce the recorded failure.
TEST(AeroElements, DISABLED_A3_SteadyAxleLoadL5) { check_a3("L5"); }

/// A3 L4: same loads as L3 (no hardpoints attached -> the L3 equations).
TEST(AeroElements, A3_L4MatchesL3) {
    const auto vp = example_car();
    const auto l3 = run_steady("L3", vp);
    const auto l4 = run_steady("L4", vp);
    EXPECT_NEAR(l4.front, l3.front, 1e-6 * l3.front);
    EXPECT_NEAR(l4.rear, l3.rear, 1e-6 * l3.rear);
}

/// A4: L5 total vertical reaction rises by the total downforce.
TEST(AeroElements, A4_L5TotalReactionCarriesDownforce) {
    const auto vp = example_car();
    const auto with = run_steady("L5", vp);
    const auto none = run_steady("L5", without_elements(vp));
    const double d = (with.front + with.rear) - (none.front + none.rear);
    std::printf("[A4 L5] total dFz %.2f N (want %.1f)\n", d, kDownTotal);
    EXPECT_NEAR(d, kDownTotal, 0.02 * kDownTotal);
}

namespace {

/// Wiki approximation on the four-key path: A = 1, Cd 1.47, Cl_f 0.881, Cl_r 2.119.
VehicleParams four_key_car(bool with_drag_height) {
    auto vp = example_car();
    vp.aero_elements.clear();
    vp.frontal_area = 1.0;
    vp.aero_drag_coeff = 1.47;
    vp.aero_lift_front = 0.881;
    vp.aero_lift_rear = 2.119;
    if (with_drag_height) vp.aero_drag_height = 0.0;
    return vp;
}

}  // namespace

/// A6: four keys + aero_drag_height and elements give the same axle loads (L1-L4).
TEST(AeroElements, A6_FourKeyPathMatchesElements) {
    for (const char* level : {"L1", "L2", "L3", "L4"}) {
        const auto el = run_steady(level, example_car());
        const auto fk = run_steady(level, four_key_car(true));
        std::printf("[A6 %s] front %.2f vs %.2f N, rear %.2f vs %.2f N\n", level,
                    el.front, fk.front, el.rear, fk.rear);
        EXPECT_LE(std::abs(el.front - fk.front), 0.005 * el.front) << level;
        EXPECT_LE(std::abs(el.rear - fk.rear), 0.005 * el.rear) << level;
    }
}

/// D1-c: the four lift keys on L5 (axle points, drag at the CG) load the axles
/// like L3, +/-2 %. Reference: the same car with the lift keys at 0.
TEST(AeroElements, D1c_FourKeyLiftOnL5MatchesL3) {
    double d[2][2];
    const char* levels[2] = {"L3", "L5"};
    for (int k = 0; k < 2; ++k) {
        const auto vp = four_key_car(false);
        auto ref = vp;
        ref.aero_lift_front = ref.aero_lift_rear = 0.0;
        const auto with = run_steady(levels[k], vp);
        const auto none = run_steady(levels[k], ref);
        d[k][0] = with.front - none.front;
        d[k][1] = with.rear - none.rear;
        std::printf("[D1-c %s] dFz front %.2f N rear %.2f N\n", levels[k], d[k][0], d[k][1]);
    }
    EXPECT_NEAR(d[1][0], d[0][0], 0.02 * std::abs(d[0][0]));
    EXPECT_NEAR(d[1][1], d[0][1], 0.02 * std::abs(d[0][1]));
}

// ---------------------------------------------------------------------------
// A3-L5a / A3-L5b (added after the A3 L5 measurement, PO 42 stage-1 verdict).
// Both gates are fixed in this file before they are run.
// ---------------------------------------------------------------------------
namespace {

/// The example car with every element's ClA set to 0 (CdA, x, z kept).
VehicleParams drag_only(VehicleParams vp) {
    for (auto& e : vp.aero_elements) e.ClA = 0.0;
    return vp;
}

/// The example car with every element's CdA set to 0 (downforce only).
VehicleParams downforce_only(VehicleParams vp) {
    for (auto& e : vp.aero_elements) e.CdA = 0.0;
    return vp;
}

}  // namespace

/// A3-L5a: same drag in both runs (reference = ClA 0, CdA/x/z unchanged), so the
/// drive force is the same and only the downforce differs. Each axle increase
/// must match the downforce-only rigid split (CoP -0.8317 m, total 485.4 N) from
/// compute_aero_loads(), +/-2 %.
TEST(AeroElements, A3_L5a_DownforceSplitSameDrag) {
    const auto vp = example_car();
    const auto want = compute_aero_loads(downforce_only(vp), AeroState{kV60});
    // Hand calculation (wiki vector): q = 0.5 * 1.165 * (60/3.6)^2 = 161.80 Pa,
    // sum ClA = 3.0 m^2 -> 485.4 N, CoP x = -2.495 / 3.0 = -0.8317 m,
    // front share (x + L) / L = 0.4634 -> front 224.95 N, rear 260.45 N.
    const auto with = run_steady("L5", vp);
    const auto ref  = run_steady("L5", drag_only(vp));
    ASSERT_TRUE(with.finite && ref.finite);
    EXPECT_LE(std::abs(with.ax), 0.02);
    EXPECT_LE(std::abs(ref.ax), 0.02);
    const double df = with.front - ref.front;
    const double dr = with.rear - ref.rear;
    std::printf("[A3-L5a] vx %.4f m/s | dFz front %.2f N (want %.2f, hand 224.95) "
                "rear %.2f N (want %.2f, hand 260.45) | total %.2f N (want %.2f)\n",
                with.vx, df, want.fz_front, dr, want.fz_rear, df + dr, want.fz_total);
    EXPECT_NEAR(want.fz_total, kDownTotal, 1e-3 * kDownTotal);
    EXPECT_NEAR(df, want.fz_front, 0.02 * want.fz_front);
    EXPECT_NEAR(dr, want.fz_rear, 0.02 * want.fz_rear);
}

/// A3-L5b: quantify the L5 wheel-centre limitation. Drag-only elements (ClA 0,
/// wiki CdA and z) vs no aero. L5 front increase must match the L5 model
/// expectation -sum(F_d z)/L + sum(F_d) R/L (R = wheel_radius_nominal), +/-5 %;
/// the same pair on L1 must match -sum(F_d z)/L, +/-2 %.
TEST(AeroElements, A3_L5b_DragMomentWheelCentreBias) {
    const auto vp = example_car();
    const auto dvp = drag_only(vp);
    const auto loads = compute_aero_loads(dvp, AeroState{kV60});
    const double L = vp.wheelbase;
    const double R = vp.wheel_radius_nominal;
    const double fd = loads.fx_total;
    const double m_drag = loads.drag_pitch_moment;   // sum F_d,i * z_i [N m]
    const double want_l1 = -m_drag / L;
    const double want_l5 = -m_drag / L + fd * R / L;
    double d[2];
    const char* levels[2] = {"L1", "L5"};
    for (int k = 0; k < 2; ++k) {
        const auto with = run_steady(levels[k], dvp);
        const auto none = run_steady(levels[k], without_elements(vp));
        ASSERT_TRUE(with.finite && none.finite) << levels[k];
        EXPECT_LE(std::abs(with.ax), 0.02) << levels[k];
        d[k] = with.front - none.front;
    }
    std::printf("[A3-L5b] sum F_d %.2f N, sum F_d z %.3f N m, R %.3f m, L %.3f m | "
                "L1 dFz front %.2f N (want %.2f) | L5 dFz front %.2f N (want %.2f, "
                "R term %.2f)\n",
                fd, m_drag, R, L, d[0], want_l1, d[1], want_l5, fd * R / L);
    EXPECT_NEAR(d[0], want_l1, 0.02 * std::abs(want_l1));
    EXPECT_NEAR(d[1], want_l5, 0.05 * std::abs(want_l5));
}
