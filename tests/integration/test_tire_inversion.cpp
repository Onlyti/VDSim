// Phase 2 tire-interface inversion equivalence (pacejka_mf96).
//
// The inverted interface ITireModel::evaluate()/advance() must reproduce, bit for
// bit, the raw per-wheel tire block the seven_dof dynamics computes today (the force
// BEFORE the vehicle-side low-speed shaping — stick-blend lambda, Fx_hold creep and
// the combined-slip clamp — which stay in the dynamics). The oracle below re-derives
// the reference straight from the same primitives (tire_contact_kinematics +
// compute / lugre_wheel_forces) so this test locks the encapsulation: when the
// dynamics models later switch to evaluate()/advance(), this is the contract they
// are proven against.

#include "vdsim/belt_tire.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/lugre_tire.hpp"
#include "vdsim/params.hpp"
#include "vdsim/tire_contact.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

using vdsim::ITireModel;

ITireModel::ContactInput make_ci() {
    ITireModel::ContactInput ci;
    ci.Fz = 4000.0;
    ci.Vx = 22.0;
    ci.Vy = -0.9;                 // ~2.3 deg slip angle
    ci.omega = (22.0 / 0.32) * 1.04;  // ~4% slip ratio at R0 = 0.32
    ci.gamma = 0.03;
    ci.mu_long = 1.0;
    ci.mu_lat  = 1.0;
    ci.R0 = 0.32;
    return ci;
}

// Independent oracle: the raw wrench the seven_dof tire block produces for these
// kinematics + transient, using the same shared primitives.
ITireModel::Wrench oracle(const ITireModel& tire, const vdsim::TireParams& tp,
                          const ITireModel::ContactInput& ci,
                          const ITireModel::Transient& tr) {
    const auto ck = vdsim::tire_contact_kinematics(
        ci.Vx, ci.Vy, ci.omega, ci.Fz, ci.gamma, tp, ci.R0);
    ITireModel::Wrench w;
    w.Re = ck.Re; w.kappa = ck.kappa; w.alpha = ck.alpha; w.contact_dy = ck.contact_dy;
    w.Mx = ci.Fz * ck.contact_dy;

    const bool lugre_on   = tp.lugre.enabled;
    const bool belt_on    = tp.belt.enabled && !lugre_on;
    const bool belt_lugre = tp.belt.enabled && lugre_on;

    ITireModel::Input in;
    in.Fz = ci.Fz;
    in.kappa = belt_on ? tr.belt_kappa : ck.kappa;
    in.alpha = belt_on ? tr.belt_alpha
        : ((lugre_on || tp.relaxation_length_lat <= 1e-6) ? ck.alpha : tr.alpha_dyn);
    in.mu_long = ci.mu_long; in.mu_lat = ci.mu_lat; in.Vx_wheel = ci.Vx; in.gamma = ci.gamma;

    if (lugre_on) {
        const double vsl = belt_lugre ? tr.belt_vlong : ck.vsx;
        const double vst = belt_lugre ? tr.belt_vlat  : ck.vsy;
        const auto l = vdsim::lugre_wheel_forces(tire, tp, tr.lugre_z_long, tr.lugre_z_lat,
                                                 vsl, vst, in);
        w.Fx = l.Fx; w.Fy = l.Fy; w.Mz = l.Mz;
    } else {
        const auto out = tire.compute(in);
        w.Fx = out.Fx; w.Fy = out.Fy; w.Mz = out.Mz; w.Mx += out.Mx;
    }
    return w;
}

void expect_wrench_eq(const ITireModel::Wrench& a, const ITireModel::Wrench& b) {
    EXPECT_DOUBLE_EQ(a.Fx, b.Fx);
    EXPECT_DOUBLE_EQ(a.Fy, b.Fy);
    EXPECT_DOUBLE_EQ(a.Mz, b.Mz);
    EXPECT_DOUBLE_EQ(a.Mx, b.Mx);
    EXPECT_DOUBLE_EQ(a.Re, b.Re);
    EXPECT_DOUBLE_EQ(a.kappa, b.kappa);
    EXPECT_DOUBLE_EQ(a.alpha, b.alpha);
    EXPECT_DOUBLE_EQ(a.contact_dy, b.contact_dy);
}

}  // namespace

TEST(TireInversion, EvaluateSteadyMatchesOracle) {
    vdsim::TireParams tp;            // steady MF path
    tp.lugre.enabled = false;        // C++ default is LuGre-on; exercise the MF force law
    tp.crown_radius = 0.10;          // exercise camber-migration Mx
    auto tire = vdsim::create_pacejka_mf96();
    tire->initialize(tp);

    const auto ci = make_ci();
    const ITireModel::Transient tr;  // unused on the steady path
    const auto w = tire->evaluate(ci, tr);
    expect_wrench_eq(w, oracle(*tire, tp, ci, tr));

    // Sanity: nonzero forces and a camber-migration overturning moment.
    EXPECT_GT(std::abs(w.Fx), 1.0);
    EXPECT_GT(std::abs(w.Fy), 1.0);
    EXPECT_NEAR(w.Mx, ci.Fz * 0.10 * std::sin(ci.gamma), 1e-9);
    EXPECT_DOUBLE_EQ(w.Re, ci.R0);   // reff_*=0 -> Re == R0
}

TEST(TireInversion, EvaluateRelaxationUsesAlphaDyn) {
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    tp.relaxation_length_lat = 0.5;
    auto tire = vdsim::create_pacejka_mf96();
    tire->initialize(tp);

    const auto ci = make_ci();
    ITireModel::Transient tr;
    tr.alpha_dyn = 0.005;            // lagged slip angle differs from geometric
    const auto w = tire->evaluate(ci, tr);
    expect_wrench_eq(w, oracle(*tire, tp, ci, tr));
    // The lateral force must reflect alpha_dyn, not the (larger) geometric alpha.
    ITireModel::Transient tr0;       // alpha_dyn = 0
    EXPECT_NE(w.Fy, tire->evaluate(ci, tr0).Fy);
}

TEST(TireInversion, EvaluateBeltMFUsesRelaxedSlip) {
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    tp.belt.enabled = true;
    tp.belt.sigma_long = 0.3;
    tp.belt.sigma_lat  = 0.5;
    auto tire = vdsim::create_pacejka_mf96();
    tire->initialize(tp);

    const auto ci = make_ci();
    ITireModel::Transient tr;
    tr.belt_kappa = 0.01;
    tr.belt_alpha = 0.006;
    const auto w = tire->evaluate(ci, tr);
    expect_wrench_eq(w, oracle(*tire, tp, ci, tr));
}

TEST(TireInversion, EvaluateLuGreMatchesOracle) {
    vdsim::TireParams tp;
    tp.lugre.enabled = true;
    auto tire = vdsim::create_pacejka_mf96();
    tire->initialize(tp);

    const auto ci = make_ci();
    ITireModel::Transient tr;
    tr.lugre_z_long = 1e-4;
    tr.lugre_z_lat  = -5e-5;
    const auto w = tire->evaluate(ci, tr);
    expect_wrench_eq(w, oracle(*tire, tp, ci, tr));
    EXPECT_GT(std::abs(w.Fx), 1.0);
}

TEST(TireInversion, EvaluateLuGreBeltUsesRelaxedVelocity) {
    vdsim::TireParams tp;
    tp.lugre.enabled = true;
    tp.belt.enabled  = true;
    tp.belt.sigma_long = 0.3;
    tp.belt.sigma_lat  = 0.5;
    auto tire = vdsim::create_pacejka_mf96();
    tire->initialize(tp);

    const auto ci = make_ci();
    ITireModel::Transient tr;
    tr.lugre_z_long = 1e-4;
    tr.lugre_z_lat  = -5e-5;
    tr.belt_vlong = 0.4;             // relaxed slip velocities differ from geometric
    tr.belt_vlat  = -0.3;
    const auto w = tire->evaluate(ci, tr);
    expect_wrench_eq(w, oracle(*tire, tp, ci, tr));
}

TEST(TireInversion, AdvanceRelaxationMatchesExponentialLag) {
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    tp.relaxation_length_lat = 0.5;
    auto tire = vdsim::create_pacejka_mf96();
    tire->initialize(tp);

    const auto ci = make_ci();
    ITireModel::Transient tr; tr.alpha_dyn = 0.002;
    const double dt = 0.002;
    const auto out = tire->advance_relaxation(ci, tr, dt);

    const auto ck = vdsim::tire_contact_kinematics(
        ci.Vx, ci.Vy, ci.omega, ci.Fz, ci.gamma, tp, ci.R0);
    const double v_safe = std::max(std::abs(ci.Vx), vdsim::kTireSpeedEps);
    const double decay  = std::exp(-v_safe * dt / tp.relaxation_length_lat);
    const double ref = ck.alpha + (tr.alpha_dyn - ck.alpha) * decay;
    EXPECT_DOUBLE_EQ(out.alpha_dyn, ref);
}

TEST(TireInversion, AdvanceLuGreMatchesAdvanceZ) {
    vdsim::TireParams tp;
    tp.lugre.enabled = true;
    auto tire = vdsim::create_pacejka_mf96();
    tire->initialize(tp);

    const auto ci = make_ci();
    ITireModel::Transient tr; tr.lugre_z_long = 1e-4; tr.lugre_z_lat = -5e-5;
    const double dt = 0.002;
    const auto out = tire->advance_bristle(ci, tr, dt);

    const auto ck = vdsim::tire_contact_kinematics(
        ci.Vx, ci.Vy, ci.omega, ci.Fz, ci.gamma, tp, ci.R0);
    ITireModel::Input in;
    in.Fz = ci.Fz; in.kappa = ck.kappa; in.alpha = ck.alpha;
    in.mu_long = ci.mu_long; in.mu_lat = ci.mu_lat; in.Vx_wheel = ci.Vx;
    const auto mf = tire->compute(in);
    const double sigma0 = std::max(1.0, tp.lugre.sigma0);
    const double zl = vdsim::lugre_advance_z(
        tr.lugre_z_long, ck.vsx,
        vdsim::lugre_breakaway(mf, true, in.Fz, in.mu_long, in.mu_lat), sigma0, dt);
    const double zt = vdsim::lugre_advance_z(
        tr.lugre_z_lat, ck.vsy,
        vdsim::lugre_breakaway(mf, false, in.Fz, in.mu_long, in.mu_lat, in.alpha),
        sigma0, dt);
    EXPECT_DOUBLE_EQ(out.lugre_z_long, zl);
    EXPECT_DOUBLE_EQ(out.lugre_z_lat,  zt);
}

TEST(TireInversion, AdvanceBeltMFMatchesBeltRelax) {
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    tp.belt.enabled = true;
    tp.belt.sigma_long = 0.3;
    tp.belt.sigma_lat  = 0.5;
    auto tire = vdsim::create_pacejka_mf96();
    tire->initialize(tp);

    const auto ci = make_ci();
    ITireModel::Transient tr; tr.belt_kappa = 0.01; tr.belt_alpha = 0.006;
    const double dt = 0.002;
    const auto out = tire->advance_relaxation(ci, tr, dt);

    const auto ck = vdsim::tire_contact_kinematics(
        ci.Vx, ci.Vy, ci.omega, ci.Fz, ci.gamma, tp, ci.R0);
    EXPECT_DOUBLE_EQ(out.belt_kappa,
                     vdsim::belt_relax(tr.belt_kappa, ck.kappa, ci.Vx, tp.belt.sigma_long, dt));
    EXPECT_DOUBLE_EQ(out.belt_alpha,
                     vdsim::belt_relax(tr.belt_alpha, ck.alpha, ci.Vx, tp.belt.sigma_lat, dt));
}

// The inverted interface is defined once in the ITireModel base and dispatches the
// force law through the virtual compute(), so EVERY backend gets evaluate()/advance()
// (not just pacejka). This guards that the linear backend evaluates its own linear law
// — i.e. injecting a non-pacejka tire into a dynamics model still produces real force.
TEST(TireInversion, LinearBackendEvaluatesViaBase) {
    vdsim::TireParams tp;
    tp.lugre.enabled = false;            // exercise the steady linear force law
    tp.cornering_stiffness = 60000.0;
    auto tire = vdsim::create_linear_tire();
    tire->initialize(tp);

    const auto ci = make_ci();
    const auto w = tire->evaluate(ci, ITireModel::Transient{});
    // Oracle: linear compute() fed the geometric slip from the shared kinematics.
    const auto ck = vdsim::tire_contact_kinematics(
        ci.Vx, ci.Vy, ci.omega, ci.Fz, ci.gamma, tp, ci.R0);
    ITireModel::Input in;
    in.Fz = ci.Fz; in.kappa = ck.kappa; in.alpha = ck.alpha;
    in.mu_long = ci.mu_long; in.mu_lat = ci.mu_lat; in.gamma = ci.gamma;
    const auto ref = tire->compute(in);
    EXPECT_DOUBLE_EQ(w.Fx, ref.Fx);
    EXPECT_DOUBLE_EQ(w.Fy, ref.Fy);
    EXPECT_GT(std::abs(w.Fy), 1.0);      // real force, not a zero stub
}
