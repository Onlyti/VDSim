// L1 Bicycle: steady-state cornering analytical comparison.
//
// With default VehicleParams + TireParams, the Pacejka linear-region
// cornering stiffness happens to be neutral steer (a*Cf == b*Cr), so the
// steady-state yaw rate equals the Ackerman value vx*delta/L.
//
// We initialise the bicycle at vx=10 m/s straight with zero slip, apply
// a small steering input (delta=0.05 rad, ay ~ 1.85 m/s^2, well inside
// the tire linear region), and integrate for ~5 s. The yaw rate at the
// end of the run should agree with the analytical value within ~10%.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>

namespace {

vdsim::ContactArray make_flat_contacts(double mu = 1.0) {
    vdsim::ContactArray c;
    for (auto& p : c) {
        p.is_valid = true;
        p.normal   = vdsim::Vec3::UnitZ();
        p.mu_long  = mu;
        p.mu_lat   = mu;
    }
    return c;
}

vdsim::State make_initial_state(double vx, double wheel_radius) {
    vdsim::State s;
    s.velocity.x() = vx;
    const double w = (wheel_radius > 0.0) ? vx / wheel_radius : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

double analytical_yaw_rate(const vdsim::VehicleParams& vp,
                           const vdsim::TireParams& tp,
                           double vx,
                           double delta) {
    // Linear-region per-axle cornering stiffness from Pacejka (small alpha):
    //   Cy = B_lat * C_lat * D_lat * Fz_axle * mu
    const double m = vp.mass;
    const double a = vp.cg_to_front;
    const double b = vp.cg_to_rear;
    const double L = vp.wheelbase;
    const double Fz_f = m * 9.80665 * b / L;
    const double Fz_r = m * 9.80665 * a / L;
    const double Cf = tp.B_lat * tp.C_lat * tp.D_lat * Fz_f * tp.mu_nominal;
    const double Cr = tp.B_lat * tp.C_lat * tp.D_lat * Fz_r * tp.mu_nominal;

    // Linear bicycle steady state:
    //   [(Cf+Cr)/vx          (a*Cf - b*Cr)/vx - m*vx ] [vy]   [Cf*delta ]
    //   [(a*Cf - b*Cr)/vx    (a^2*Cf + b^2*Cr)/vx    ] [r ] = [a*Cf*delta]
    const double A11 = (Cf + Cr) / vx;
    const double A12 = (a * Cf - b * Cr) / vx - m * vx;
    const double A21 = (a * Cf - b * Cr) / vx;
    const double A22 = (a * a * Cf + b * b * Cr) / vx;
    const double B1  = Cf * delta;
    const double B2  = a * Cf * delta;

    const double det = A11 * A22 - A12 * A21;
    const double r   = (A11 * B2 - A21 * B1) / det;
    return r;
}

}  // namespace

class BicycleSteadyState : public ::testing::Test {
protected:
    void SetUp() override {
        vp_ = vdsim::VehicleParams{};
        vp_.aero_drag_coeff = 0.0;     // remove drag so vx ~= constant
        tp_ = vdsim::TireParams{};
        sp_ = vdsim::SolverParams{};

        dyn_ = vdsim::create_bicycle();
        dyn_->initialize(vp_, tp_, sp_);
        dyn_->reset(make_initial_state(10.0, vp_.wheel_radius_nominal));
    }

    void simulate(double delta, double duration, double dt) {
        vdsim::CmdL4 cmd;
        cmd.steer_angle_wheel = delta;
        vdsim::ControlInput u = cmd;
        const auto contacts = make_flat_contacts();
        const int N = static_cast<int>(std::round(duration / dt));
        for (int i = 0; i < N; ++i) {
            dyn_->step(u, contacts, dt);
        }
    }

    vdsim::VehicleParams vp_;
    vdsim::TireParams    tp_;
    vdsim::SolverParams  sp_;
    std::unique_ptr<vdsim::IVehicleDynamics> dyn_;
};

TEST_F(BicycleSteadyState, LeftTurnYawRateMatchesAnalytical) {
    const double delta = 0.05;        // ~3 deg
    const double vx0   = 10.0;
    simulate(delta, 5.0, 0.005);      // 5 s, 200 Hz outer

    const auto& s = dyn_->state();
    const double r_sim = s.yaw_rate();
    const double r_ana = analytical_yaw_rate(vp_, tp_, vx0, delta);

    // Sign: left steer (delta>0) -> CCW yaw -> r>0
    EXPECT_GT(r_sim, 0.0);
    EXPECT_GT(r_ana, 0.0);
    EXPECT_NEAR(r_sim, r_ana, std::abs(r_ana) * 0.10);
}

TEST_F(BicycleSteadyState, RightTurnYawRateMatchesAnalytical) {
    const double delta = -0.05;
    const double vx0   = 10.0;
    simulate(delta, 5.0, 0.005);

    const auto& s = dyn_->state();
    const double r_sim = s.yaw_rate();
    const double r_ana = analytical_yaw_rate(vp_, tp_, vx0, delta);

    EXPECT_LT(r_sim, 0.0);
    EXPECT_LT(r_ana, 0.0);
    EXPECT_NEAR(r_sim, r_ana, std::abs(r_ana) * 0.10);
}

TEST_F(BicycleSteadyState, ZeroSteerStraightLine) {
    simulate(0.0, 3.0, 0.005);
    const auto& s = dyn_->state();
    EXPECT_NEAR(s.yaw_rate(),  0.0, 1e-6);
    EXPECT_NEAR(s.velocity.y(), 0.0, 1e-4);
    EXPECT_NEAR(s.position.y(), 0.0, 1e-3);
    EXPECT_NEAR(vdsim::yaw_from_quat(s.orientation), 0.0, 1e-6);
}

TEST_F(BicycleSteadyState, LowMuReducesYawRate) {
    // With mu=1.0 we already verified r matches analytical. With mu=0.3 the
    // tire can saturate, so the yaw rate should drop.
    const double delta = 0.05;
    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = delta;
    vdsim::ControlInput u = cmd;

    // High mu reference
    dyn_->reset(make_initial_state(10.0, vp_.wheel_radius_nominal));
    auto contacts_hi = make_flat_contacts(1.0);
    for (int i = 0; i < 1000; ++i) dyn_->step(u, contacts_hi, 0.005);
    const double r_hi = dyn_->state().yaw_rate();

    // Low mu re-run
    dyn_->reset(make_initial_state(10.0, vp_.wheel_radius_nominal));
    auto contacts_lo = make_flat_contacts(0.3);
    for (int i = 0; i < 1000; ++i) dyn_->step(u, contacts_lo, 0.005);
    const double r_lo = dyn_->state().yaw_rate();

    EXPECT_GT(r_hi, 0.0);
    EXPECT_GT(r_lo, 0.0);
    EXPECT_LE(r_lo, r_hi);   // low mu cannot exceed high mu
}

TEST(BicycleStubs, AllLevelsConstructible) {
    EXPECT_NO_THROW({ auto p = vdsim::create_bicycle();      (void)p; });
    EXPECT_NO_THROW({ auto p = vdsim::create_seven_dof();    (void)p; });
    EXPECT_NO_THROW({ auto p = vdsim::create_fourteen_dof(); (void)p; });
}

TEST(FlatGround, QueryFillsAllContacts) {
    auto provider = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::VehicleParams vp;
    vdsim::State s;
    vdsim::ContactArray out;
    provider->query(s, vp, out);
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
        EXPECT_TRUE(out[i].is_valid);
        EXPECT_DOUBLE_EQ(out[i].mu_long, 1.0);
        EXPECT_DOUBLE_EQ(out[i].normal.z(), 1.0);
    }
}
