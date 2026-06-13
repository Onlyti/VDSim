// ISO baseline regression gate — locks the nonlinear step-steer signature of the
// shipped catalog default preset (sedan chassis + default_pacejka LuGre tire, L2)
// so that any future change moving tire/dynamics forces is caught in CI.
//
// The companion human-facing report is apps/validation (Python, not in CI). This
// C++ gate mirrors that harness (coast step-steer, steady state = last 20%) on the
// exact YAML configs and bands the four force-sensitive metrics. Golden values were
// measured on the v0.5.1 default preset; bands are wide enough for integrator/
// platform jitter but tight enough to flag a real force-level regression.
//
// If a future change *intentionally* moves these numbers, re-run
// apps/validation/run_validation.py, update VALIDATION.md, and rebaseline here.

#include "vdsim/control.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace {

constexpr double GRAVITY = 9.80665;
const std::string kRepo = VDSIM_SOURCE_DIR;

vdsim::ContactArray flat_contacts(double mu = 1.0) {
    vdsim::ContactArray c;
    for (auto& p : c) {
        p.is_valid = true; p.normal = vdsim::Vec3::UnitZ();
        p.mu_long = mu; p.mu_lat = mu;
    }
    return c;
}

struct StepMetrics {
    double psi_dot_ss;    // [rad/s] steady-state yaw rate (mean of last 20%)
    double psi_dot_peak;  // [rad/s]
    double overshoot;     // peak / ss
    double ay_ss;         // [m/s^2] steady-state lateral accel
    double vx_ss;         // [m/s] residual speed at end (coast)
};

// Coast step-steer on the catalog default preset, mirroring apps/doe scenarios.
StepMetrics step_steer_default(double v_target, double steer_rad) {
    vdsim::VehicleParams vp = vdsim::VehicleParams::from_yaml(
        kRepo + "/configs/parts/body/sedan.yaml");
    vdsim::TireParams tp = vdsim::TireParams::from_yaml(
        kRepo + "/configs/parts/tire/default_pacejka.yaml");
    vdsim::SolverParams sp;

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);

    vdsim::State s0;
    s0.velocity.x() = v_target;
    const double w = v_target / 0.33;
    s0.wheel_spin = {{w, w, w, w}};
    dyn->reset(s0);

    const auto contacts = flat_contacts(1.0);
    const double dt = 0.002;
    const int n_pre = static_cast<int>(0.5 / dt);
    const int n_post = static_cast<int>(6.0 / dt);

    vdsim::CmdL4 cmd;  // throttle = brake = 0 -> coast (no speed hold, matches Python)
    vdsim::ControlInput u = cmd;
    for (int i = 0; i < n_pre; ++i) {
        auto c = contacts;
        dyn->step(u, c, dt);
    }

    cmd.steer_angle_wheel = steer_rad;
    u = cmd;
    const int n_tail = n_post / 5;  // last 20%
    double psi_peak = 0.0, psi_acc = 0.0, ay_acc = 0.0, vx_acc = 0.0;
    for (int i = 0; i < n_post; ++i) {
        auto c = contacts;
        dyn->step(u, c, dt);
        const double r = dyn->state().yaw_rate();
        if (std::abs(r) > std::abs(psi_peak)) psi_peak = r;
        if (i >= n_post - n_tail) {
            psi_acc += r;
            ay_acc += dyn->ay_body_est();
            vx_acc += dyn->state().vx();
        }
    }
    StepMetrics m;
    m.psi_dot_ss = psi_acc / n_tail;
    m.psi_dot_peak = psi_peak;
    m.overshoot = m.psi_dot_peak / m.psi_dot_ss;
    m.ay_ss = ay_acc / n_tail;
    m.vx_ss = vx_acc / n_tail;
    return m;
}

}  // namespace

// 80 km/h, 6 deg road-wheel step. Default preset is mildly understeering and
// underdamped; these bands lock the v0.5.1 force level.
TEST(IsoBaseline, SedanL2StepSteerSignature) {
    const double v = 80.0 / 3.6;            // 22.22 m/s
    const double delta = 6.0 * M_PI / 180.0;  // 0.10472 rad
    const StepMetrics m = step_steer_default(v, delta);

    // --- measurement echo (visible with --output-on-failure / -V) ---
    RecordProperty("psi_dot_ss_radps", m.psi_dot_ss);
    RecordProperty("psi_dot_peak_radps", m.psi_dot_peak);
    RecordProperty("overshoot", m.overshoot);
    RecordProperty("ay_ss_mps2", m.ay_ss);
    RecordProperty("vx_ss_mps", m.vx_ss);

    // Kinematic (neutral-steer) yaw rate ceiling: V*delta/L. The steady yaw must
    // sit below it (understeer) but not collapse (still a meaningful response).
    const double r_kin = v * delta / 2.7;   // sedan wheelbase

    EXPECT_GT(m.psi_dot_ss, 0.40)
        << "steady yaw rate collapsed -> grip/force regression";
    EXPECT_LT(m.psi_dot_ss, r_kin)
        << "steady yaw exceeds kinematic neutral -> oversteer, unexpected for sedan";
    EXPECT_NEAR(m.psi_dot_ss, 0.528, 0.060)
        << "psi_dot_ss off v0.5.1 baseline (0.528 rad/s ~= 30.3 deg/s)";

    EXPECT_NEAR(m.overshoot, 1.16, 0.12)
        << "transient overshoot off v0.5.1 baseline (~16%)";

    EXPECT_NEAR(m.ay_ss, 8.34, 0.90)
        << "steady lateral accel off v0.5.1 baseline (~0.85 g)";
}
