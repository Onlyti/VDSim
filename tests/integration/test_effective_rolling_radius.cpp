// Effective rolling radius (Re) consistency: a tire initialised at the
// Re-consistent free-roll spin must report slip~0 and produce no phantom
// longitudinal force at the first step. Locks the fix where slip used Re(Fz)
// but wheel_spin was initialised with the unloaded radius (a ~4 kN phantom).

#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

constexpr double GRAVITY = 9.80665;

vdsim::ContactArray flat_contacts(double mu = 1.0) {
    vdsim::ContactArray c;
    for (auto& p : c) { p.is_valid = true; p.normal = vdsim::Vec3::UnitZ();
                        p.mu_long = mu; p.mu_lat = mu; }
    return c;
}

vdsim::TireParams reff_tire() {
    vdsim::TireParams tp;
    tp.tire_vertical_stiffness = 220000.0;
    tp.Fz_nominal = 4000.0;
    tp.reff_breff = 8.4;
    tp.reff_dreff = 0.27;
    tp.reff_freff = 0.07;
    return tp;
}

vdsim::State free_roll(const vdsim::VehicleParams& vp,
                       const vdsim::TireParams& tp, double vx) {
    vdsim::State s; s.velocity.x() = vx;
    s.wheel_spin = vdsim::free_roll_wheel_spin(vp, tp, vx);
    return s;
}

}  // namespace

TEST(EffectiveRollingRadius, FallsBackToR0WhenNoCoeffs) {
    vdsim::TireParams tp;  // reff_* = 0
    EXPECT_DOUBLE_EQ(vdsim::effective_rolling_radius(tp, 0.32, 4000.0), 0.32);
}

TEST(EffectiveRollingRadius, ShrinksWithLoad) {
    const auto tp = reff_tire();
    const double Re_light = vdsim::effective_rolling_radius(tp, 0.32, 2000.0);
    const double Re_heavy = vdsim::effective_rolling_radius(tp, 0.32, 6000.0);
    EXPECT_LT(Re_light, 0.32);          // loaded tire rolls on a smaller radius
    EXPECT_LT(Re_heavy, Re_light);      // heavier -> smaller Re
}

TEST(EffectiveRollingRadius, FreeRollSpinIsPerWheelAndAxleSplit) {
    vdsim::VehicleParams vp;
    const auto tp = reff_tire();
    const auto w = vdsim::free_roll_wheel_spin(vp, tp, 20.0);
    EXPECT_DOUBLE_EQ(w[vdsim::WHEEL_FL], w[vdsim::WHEEL_FR]);
    EXPECT_DOUBLE_EQ(w[vdsim::WHEEL_RL], w[vdsim::WHEEL_RR]);
    // Front axle carries more static load -> smaller Re -> higher spin (default a<b).
    EXPECT_GT(w[vdsim::WHEEL_FL], w[vdsim::WHEEL_RL]);
    // reff_*=0 collapses to vx/R0 on every wheel.
    vdsim::TireParams plain;
    const auto w0 = vdsim::free_roll_wheel_spin(vp, plain, 20.0);
    for (double wi : w0) EXPECT_NEAR(wi, 20.0 / vp.wheel_radius_nominal, 1e-9);
}

// The core regression: no phantom longitudinal force when free-rolling with Re.
class NoPhantomForce : public ::testing::TestWithParam<vdsim::IVehicleDynamics::Level> {};

TEST_P(NoPhantomForce, FirstStepIsPureRollingResistance) {
    vdsim::VehicleParams vp; vp.aero_drag_coeff = 0.0;
    const auto tp = reff_tire();
    vdsim::SolverParams sp;
    std::unique_ptr<vdsim::IVehicleDynamics> dyn;
    switch (GetParam()) {
        case vdsim::IVehicleDynamics::Level::L1_Bicycle: dyn = vdsim::create_bicycle(); break;
        case vdsim::IVehicleDynamics::Level::L3_FourteenDOF: dyn = vdsim::create_fourteen_dof(); break;
        default: dyn = vdsim::create_seven_dof(); break;
    }
    dyn->initialize(vp, tp, sp);
    dyn->reset(free_roll(vp, tp, 20.0));

    const vdsim::ControlInput u = vdsim::CmdL4{};   // coast: no throttle/brake
    dyn->step(u, flat_contacts(), 0.005);

    double sumFx = 0.0;
    const auto F = dyn->tire_forces_body();
    for (const auto& f : F) sumFx += f.x();
    for (double k : dyn->wheel_slip_ratio()) EXPECT_NEAR(k, 0.0, 5e-4);
    // Rolling resistance on a 1500 kg car is ~220 N; a phantom would be ~kN.
    EXPECT_LT(std::abs(sumFx), 60.0);
}

INSTANTIATE_TEST_SUITE_P(AllLevels, NoPhantomForce,
    ::testing::Values(vdsim::IVehicleDynamics::Level::L1_Bicycle,
                      vdsim::IVehicleDynamics::Level::L2_SevenDOF,
                      vdsim::IVehicleDynamics::Level::L3_FourteenDOF));
