// D4 — engine torque map + gearbox + shift policy driving the L2 plant.
//
// A full-throttle acceleration run should: accelerate, auto-upshift through the gears,
// keep engine RPM within [idle, redline], and drop RPM after each upshift (the sawtooth
// signature). A programmatic shift policy must override the built-in one.

#include "vdsim/control.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

vdsim::VehicleParams powertrain_vp() {
    vdsim::VehicleParams vp;            // default sedan-ish, RWD
    auto& pt = vp.powertrain;
    pt.enabled = true;
    pt.engine.idle_rpm = 800; pt.engine.redline_rpm = 6500; pt.engine.inertia = 0.20;
    pt.engine.map.rpm_breaks      = {1000, 2500, 4000, 5500, 6500};
    pt.engine.map.throttle_breaks = {0.0, 1.0};
    pt.engine.map.torque = {
        {-15, -20, -25, -30, -35},
        {180, 300, 330, 300, 250},
    };
    pt.gearbox.gear_ratios = {3.4, 2.0, 1.4, 1.0, 0.8};
    pt.gearbox.final_drive = 4.1; pt.gearbox.efficiency = 0.92; pt.gearbox.shift_time = 0.2;
    pt.shift_mode   = vdsim::PowertrainParams::ShiftMode::AutoRpmThreshold;
    pt.upshift_rpm  = 6000; pt.downshift_rpm = 2000; pt.start_gear = 1;
    return vp;
}

vdsim::State rolling(double Vx) {
    vdsim::State s;
    s.velocity.x() = Vx;
    const double w = Vx / 0.32;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

}  // namespace

TEST(DrivetrainV2, AccelUpshiftsAndRpmStaysBounded) {
    const auto vp = powertrain_vp();
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, vdsim::TireParams{}, vdsim::SolverParams{});
    dyn->reset(rolling(4.0));
    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::CmdL4 cmd; cmd.throttle = 1.0; cmd.gear = 1;
    const vdsim::ControlInput u = cmd;

    int max_gear = dyn->current_gear();
    int upshifts = 0;
    double rpm_before_shift = 0.0;
    bool saw_rpm_drop = false;
    const double v0 = dyn->state().vx();

    for (int i = 0; i < 10000; ++i) {          // 10 s
        vdsim::ContactArray c; ground->query(dyn->state(), vp, c);
        const double rpm_pre = dyn->engine_rpm();
        const int gear_pre = dyn->current_gear();
        dyn->step(u, c, 1e-3);
        const double rpm = dyn->engine_rpm();
        const int gear = dyn->current_gear();

        EXPECT_GE(rpm, 800.0 - 1.0) << "rpm below idle";
        EXPECT_LE(rpm, 6500.0 + 1.0) << "rpm above redline";

        if (gear > gear_pre) {                 // an upshift occurred this step
            ++upshifts;
            rpm_before_shift = rpm_pre;
        }
        if (rpm_before_shift > 0.0 && rpm < rpm_before_shift - 200.0) saw_rpm_drop = true;
        max_gear = std::max(max_gear, gear);
    }

    EXPECT_GT(dyn->state().vx(), v0 + 5.0) << "full throttle should accelerate the car";
    EXPECT_GE(upshifts, 1) << "auto gearbox should upshift at least once";
    EXPECT_GE(max_gear, 2);
    EXPECT_TRUE(saw_rpm_drop) << "rpm should drop after an upshift (sawtooth)";
}

TEST(DrivetrainV2, ProgrammaticShiftPolicyOverrides) {
    const auto vp = powertrain_vp();
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, vdsim::TireParams{}, vdsim::SolverParams{});
    dyn->reset(rolling(20.0));

    bool called = false;
    const bool ok = dyn->set_shift_policy([&](const vdsim::ShiftContext& c) {
        called = true;
        EXPECT_GT(c.engine_rpm, 0.0);
        return 3;                              // force 3rd gear
    });
    ASSERT_TRUE(ok) << "L2 with powertrain must accept a shift policy";

    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::CmdL4 cmd; cmd.throttle = 0.5; cmd.gear = 1;
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < 500; ++i) {
        vdsim::ContactArray c; ground->query(dyn->state(), vp, c);
        dyn->step(u, c, 1e-3);
    }
    EXPECT_TRUE(called);
    EXPECT_EQ(dyn->current_gear(), 3) << "gear should follow the custom policy";
}

TEST(DrivetrainV2, LaunchesFromRest) {
    const auto vp = powertrain_vp();
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, vdsim::TireParams{}, vdsim::SolverParams{});
    dyn->reset(rolling(0.0));
    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::CmdL4 cmd; cmd.throttle = 1.0; cmd.gear = 1;
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < 3000; ++i) {           // 3 s
        vdsim::ContactArray c; ground->query(dyn->state(), vp, c);
        dyn->step(u, c, 1e-3);
    }
    EXPECT_GT(dyn->state().vx(), 2.0) << "idle-clutch launch should get the car moving";
    EXPECT_GE(dyn->engine_rpm(), 800.0);
}

// Regression: the gear-reflected engine inertia must reach the wheel-spin ODE on
// the open-diff path. The per-wheel divisor used the gearbox inertia but the
// open-diff carrier coupling used a SEPARATE legacy reflection, so with a powertrain
// enabled and no legacy engine_rotational_inertia the carrier inertia was zero and
// the coupling was skipped. The carrier only redistributes spin when the two wheels
// DIFFER, so the effect shows up under split-mu: the low-grip wheel free-spins, and a
// larger engine inertia (now correctly in the carrier) must hold its spin-up back.
namespace {
// Launch on split-mu (RR low grip), clutch locked, and return the runaway RR wheel
// speed after the inertial transient. The default powertrain exercises the open-diff
// carrier path; a custom drivetrain via set_drivetrain_module would hit the same code.
double rr_spin_split_mu(vdsim::IVehicleDynamics& dyn, const vdsim::VehicleParams& vp) {
    dyn.initialize(vp, vdsim::TireParams{}, vdsim::SolverParams{});
    dyn.reset(rolling(8.0));
    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::CmdL4 cmd; cmd.throttle = 1.0; cmd.gear = 1;
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < 150; ++i) {                       // 150 ms — clutch locked, inertial transient
        vdsim::ContactArray c; ground->query(dyn.state(), vp, c);
        c[vdsim::WHEEL_RR].mu_long = 0.08;                // icy right rear -> it spins up
        c[vdsim::WHEEL_RR].mu_lat  = 0.08;
        dyn.step(u, c, 1e-3);
    }
    return dyn.state().wheel_spin[vdsim::WHEEL_RR];
}
}  // namespace

TEST(DrivetrainV2, OpenDiffReflectsGearInertiaIntoSpin) {
    auto vp = powertrain_vp();                  // RWD
    vp.differential = vdsim::VehicleParams::Differential::Open;
    vp.engine_rotational_inertia = 0.0;         // legacy reflection off -> isolate the gearbox path

    auto low = vp;  low.powertrain.engine.inertia = 0.05;
    auto high = vp; high.powertrain.engine.inertia = 3.0;

    auto d_low  = vdsim::create_seven_dof();
    auto d_high = vdsim::create_seven_dof();
    const double s_low  = rr_spin_split_mu(*d_low,  low);
    const double s_high = rr_spin_split_mu(*d_high, high);

    EXPECT_GT(s_low, 0.0);
    EXPECT_LT(s_high, s_low * 0.95)
        << "a larger engine inertia (now in the open-diff carrier) must slow the "
           "spinning wheel's runaway (s_low=" << s_low << " s_high=" << s_high << ")";
}
// free_3d (L5) gets the identical eff_wheel_I / I_axle fix (source consistency,
// legacy bit-identical — covered by the stunt suite). Its launch + contact-grounded
// dynamics barely exercise the open-diff carrier inertia, so there is no robust
// behavioral signal to assert at L5; that fix is verified by inspection.

// Regression: reflected engine inertia must resist SYMMETRIC straight-line accel on an
// open diff. The old carrier blend was a no-op when the wheels turned equally, so the
// engine inertia was invisible to longitudinal launch (the common case). Now a larger
// engine inertia must measurably slow a grippy, straight, full-throttle launch.
TEST(OpenDiffInertia, EngineInertiaSlowsSymmetricLaunch) {
    auto run = [](double Ie) {
        vdsim::VehicleParams vp;                       // default: RWD, Open diff
        vp.engine_rotational_inertia = Ie;
        auto dyn = vdsim::create_seven_dof();
        dyn->initialize(vp, vdsim::TireParams{}, vdsim::SolverParams{});
        dyn->reset(rolling(2.0));
        auto ground = vdsim::create_flat_ground(0.0, 1.0);
        vdsim::CmdL4 cmd; cmd.throttle = 1.0;
        const vdsim::ControlInput u = cmd;
        for (int i = 0; i < 600; ++i) {
            vdsim::ContactArray c; ground->query(dyn->state(), vp, c);
            dyn->step(u, c, 1e-3);
        }
        return dyn->state().vx();
    };
    const double v_lowI  = run(0.0);
    const double v_highI = run(6.0);
    EXPECT_GT(v_lowI, 2.0) << "full throttle should accelerate the car";
    EXPECT_LT(v_highI, v_lowI * 0.97)
        << "engine inertia must slow symmetric open-diff accel "
           "(v_lowI=" << v_lowI << " v_highI=" << v_highI << ")";
}
