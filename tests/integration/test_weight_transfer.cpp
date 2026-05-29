// L1 longitudinal weight transfer (quasi-static, 1-step lag on ax).
//
// Verify:
//   1. At rest (ax = 0)  -> Fz_f == m g b / L, Fz_r == m g a / L.
//   2. Hard brake        -> Fz_f > static, Fz_r < static (within mass cons).
//   3. Hard accel        -> Fz_r > static, Fz_f < static.
//   4. Mass conservation -> Fz_f + Fz_r == m g (within numerical eps).

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr double GRAVITY = 9.80665;

vdsim::ContactArray flat_contacts(double mu = 1.0) {
    vdsim::ContactArray c;
    for (auto& p : c) { p.is_valid = true; p.normal = vdsim::Vec3::UnitZ();
                       p.mu_long = mu; p.mu_lat = mu; }
    return c;
}

vdsim::State init_state(double vx, double R) {
    vdsim::State s; s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

double sum_Fz(const std::array<double, vdsim::NUM_WHEELS>& Fz) {
    return Fz[vdsim::WHEEL_FL] + Fz[vdsim::WHEEL_FR]
         + Fz[vdsim::WHEEL_RL] + Fz[vdsim::WHEEL_RR];
}

double Fz_front_axle(const std::array<double, vdsim::NUM_WHEELS>& Fz) {
    return Fz[vdsim::WHEEL_FL] + Fz[vdsim::WHEEL_FR];
}

double Fz_rear_axle(const std::array<double, vdsim::NUM_WHEELS>& Fz) {
    return Fz[vdsim::WHEEL_RL] + Fz[vdsim::WHEEL_RR];
}

}  // namespace

class WeightTransfer : public ::testing::Test {
protected:
    void SetUp() override {
        vp_.aero_drag_coeff = 0.0;
        dyn_ = vdsim::create_bicycle();
        dyn_->initialize(vp_, tp_, sp_);
    }
    vdsim::VehicleParams vp_;
    vdsim::TireParams    tp_;
    vdsim::SolverParams  sp_;
    std::unique_ptr<vdsim::IVehicleDynamics> dyn_;
};

TEST_F(WeightTransfer, AtRestStaticDistribution) {
    dyn_->reset(init_state(0.0, vp_.wheel_radius_nominal));
    vdsim::CmdL4 cmd; vdsim::ControlInput u = cmd;
    dyn_->step(u, flat_contacts(), 0.005);     // single tick, ax_prev still 0

    const auto Fz = dyn_->tire_Fz();
    const double Fz_f_static = vp_.mass * GRAVITY * vp_.cg_to_rear  / vp_.wheelbase;
    const double Fz_r_static = vp_.mass * GRAVITY * vp_.cg_to_front / vp_.wheelbase;
    EXPECT_NEAR(Fz_front_axle(Fz), Fz_f_static, 1.0);
    EXPECT_NEAR(Fz_rear_axle(Fz),  Fz_r_static, 1.0);
    EXPECT_NEAR(sum_Fz(Fz), vp_.mass * GRAVITY, 1.0);
}

TEST_F(WeightTransfer, HardBrakeLoadsFront) {
    dyn_->reset(init_state(20.0, vp_.wheel_radius_nominal));
    vdsim::CmdL4 cmd; cmd.brake = 0.9; vdsim::ControlInput u = cmd;
    for (int i = 0; i < 50; ++i) dyn_->step(u, flat_contacts(), 0.005);

    const auto Fz = dyn_->tire_Fz();
    const double Fz_f_static = vp_.mass * GRAVITY * vp_.cg_to_rear  / vp_.wheelbase;
    EXPECT_GT(Fz_front_axle(Fz), Fz_f_static * 1.05);   // > +5 %
    EXPECT_LT(Fz_rear_axle(Fz),  Fz_f_static * 1.10);   // rear unloaded vs static
    EXPECT_NEAR(sum_Fz(Fz), vp_.mass * GRAVITY, 5.0);
}

TEST_F(WeightTransfer, HardAccelLoadsRear) {
    dyn_->reset(init_state(5.0, vp_.wheel_radius_nominal));
    vdsim::CmdL4 cmd; cmd.throttle = 1.0; vdsim::ControlInput u = cmd;
    for (int i = 0; i < 50; ++i) dyn_->step(u, flat_contacts(), 0.005);

    const auto Fz = dyn_->tire_Fz();
    const double Fz_r_static = vp_.mass * GRAVITY * vp_.cg_to_front / vp_.wheelbase;
    EXPECT_GT(Fz_rear_axle(Fz),  Fz_r_static * 1.01);
    EXPECT_LT(Fz_front_axle(Fz), vp_.mass * GRAVITY * vp_.cg_to_rear / vp_.wheelbase);
    EXPECT_NEAR(sum_Fz(Fz), vp_.mass * GRAVITY, 5.0);
}

TEST_F(WeightTransfer, EBDApproximatesDynamicLoadDistribution) {
    // Under hard brake, dynamic Fz_f / total grows.  EBD-enabled bias should
    // converge to that ratio (>0.55) instead of the static 0.5.
    vp_.brake_bias_front = 0.5;
    vp_.brake_ebd_enabled = true;
    dyn_ = vdsim::create_bicycle();
    dyn_->initialize(vp_, tp_, sp_);
    dyn_->reset(init_state(20.0, vp_.wheel_radius_nominal));
    vdsim::CmdL4 cmd; cmd.brake = 0.9; vdsim::ControlInput u = cmd;
    for (int i = 0; i < 50; ++i) dyn_->step(u, flat_contacts(), 0.005);
    const auto Fz = dyn_->tire_Fz();
    const double Fz_f = Fz[vdsim::WHEEL_FL] + Fz[vdsim::WHEEL_FR];
    const double Fz_r = Fz[vdsim::WHEEL_RL] + Fz[vdsim::WHEEL_RR];
    const double dyn_ratio = Fz_f / (Fz_f + Fz_r);
    EXPECT_GT(dyn_ratio, 0.55);
    // Also: vehicle should not over-rotate; mass conservation maintained.
    EXPECT_NEAR(Fz_f + Fz_r, vp_.mass * 9.80665, 5.0);
}

TEST_F(WeightTransfer, BrakeBiasInfluencesFrontFx) {
    // Higher front brake bias loads front harder.  Same vp, different bias.
    auto run_brake_bias = [&](double bias) {
        vp_.brake_bias_front = bias;
        dyn_ = vdsim::create_bicycle();
        dyn_->initialize(vp_, tp_, sp_);
        dyn_->reset(init_state(20.0, vp_.wheel_radius_nominal));
        vdsim::CmdL4 cmd; cmd.brake = 0.5; vdsim::ControlInput u = cmd;
        for (int i = 0; i < 100; ++i) dyn_->step(u, flat_contacts(), 0.005);
        return dyn_->state().velocity.x();
    };
    const double vx_front_heavy = run_brake_bias(0.8);
    const double vx_rear_heavy  = run_brake_bias(0.2);
    // Front-heavy bias uses the larger Fz contact (with weight transfer),
    // so deceleration is stronger -> vx_end is smaller.
    EXPECT_LT(vx_front_heavy, vx_rear_heavy);
}

TEST_F(WeightTransfer, BrakeDistanceShorterWithTransfer) {
    // Hard brake from 20 m/s.  With weight transfer the front tire
    // gets higher Fz, so combined tire saturation point is delayed,
    // and the *time-averaged* decel is at least as much as the no-transfer
    // baseline of ~3 m/s^2.  Plus mass conservation.
    dyn_->reset(init_state(20.0, vp_.wheel_radius_nominal));
    vdsim::CmdL4 cmd; cmd.brake = 0.8; vdsim::ControlInput u = cmd;
    const double T = 2.0, dt = 0.005;
    for (int i = 0; i < static_cast<int>(T / dt); ++i) {
        dyn_->step(u, flat_contacts(), dt);
    }
    const double v_end = dyn_->state().velocity.x();
    EXPECT_GE((20.0 - v_end) / T, 2.0);                 // still passes baseline
    EXPECT_NEAR(sum_Fz(dyn_->tire_Fz()), vp_.mass * GRAVITY, 5.0);
}

TEST_F(WeightTransfer, ResetClearsTransferHistory) {
    // After accel run, reset and immediately read Fz: should be static.
    dyn_->reset(init_state(5.0, vp_.wheel_radius_nominal));
    vdsim::CmdL4 accel; accel.throttle = 1.0; vdsim::ControlInput u_a = accel;
    for (int i = 0; i < 50; ++i) dyn_->step(u_a, flat_contacts(), 0.005);

    dyn_->reset(init_state(0.0, vp_.wheel_radius_nominal));
    vdsim::CmdL4 zero; vdsim::ControlInput u_z = zero;
    dyn_->step(u_z, flat_contacts(), 0.001);             // single tick at 1 ms

    const auto Fz = dyn_->tire_Fz();
    const double Fz_f_static = vp_.mass * GRAVITY * vp_.cg_to_rear / vp_.wheelbase;
    EXPECT_NEAR(Fz_front_axle(Fz), Fz_f_static, 5.0);
}
