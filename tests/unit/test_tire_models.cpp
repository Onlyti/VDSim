#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {
constexpr double kTol = 1e-9;
}

// =============================================================================
// Pacejka MF96
// =============================================================================
class PacejkaMF96Fixture : public ::testing::Test {
protected:
    void SetUp() override {
        model_ = vdsim::create_pacejka_mf96();
        model_->initialize(tp_);
    }

    vdsim::ITireModel::Input make_input(double Fz, double kappa, double alpha,
                                         double mu_long = 1.0, double mu_lat = 1.0) {
        vdsim::ITireModel::Input in;
        in.Fz       = Fz;
        in.kappa    = kappa;
        in.alpha    = alpha;
        in.mu_long  = mu_long;
        in.mu_lat   = mu_lat;
        in.Vx_wheel = 10.0;
        return in;
    }

    vdsim::TireParams tp_;
    std::unique_ptr<vdsim::ITireModel> model_;
};

TEST_F(PacejkaMF96Fixture, ZeroSlipZeroForce) {
    const auto out = model_->compute(make_input(4000, 0.0, 0.0));
    EXPECT_NEAR(out.Fx, 0.0, kTol);
    EXPECT_NEAR(out.Fy, 0.0, kTol);
    EXPECT_NEAR(out.Mz, 0.0, kTol);
}

TEST_F(PacejkaMF96Fixture, ZeroFzZeroForce) {
    const auto out = model_->compute(make_input(0.0, 0.05, 0.05));
    EXPECT_NEAR(out.Fx, 0.0, kTol);
    EXPECT_NEAR(out.Fy, 0.0, kTol);
}

TEST_F(PacejkaMF96Fixture, LinearRegionLateralSlope) {
    const double Fz    = 4000.0;
    const double alpha = 1e-4;
    const auto out = model_->compute(make_input(Fz, 0.0, alpha));
    const double expected_slope = -tp_.B_lat * tp_.C_lat * tp_.D_lat * Fz * tp_.mu_nominal;
    const double computed_slope = out.Fy / alpha;
    EXPECT_NEAR(computed_slope, expected_slope, std::abs(expected_slope) * 0.01);
}

TEST_F(PacejkaMF96Fixture, LinearRegionLongitudinalSlope) {
    const double Fz    = 4000.0;
    const double kappa = 1e-4;
    const auto out = model_->compute(make_input(Fz, kappa, 0.0));
    const double expected_slope = tp_.B_long * tp_.C_long * tp_.D_long * Fz * tp_.mu_nominal;
    const double computed_slope = out.Fx / kappa;
    EXPECT_NEAR(computed_slope, expected_slope, std::abs(expected_slope) * 0.01);
}

TEST_F(PacejkaMF96Fixture, SignConventions) {
    // alpha > 0  -> F_y < 0  (restoring)
    EXPECT_LT(model_->compute(make_input(4000, 0.0, 0.05)).Fy, 0.0);
    // alpha < 0  -> F_y > 0
    EXPECT_GT(model_->compute(make_input(4000, 0.0, -0.05)).Fy, 0.0);
    // kappa > 0  -> F_x > 0  (drive)
    EXPECT_GT(model_->compute(make_input(4000, 0.05, 0.0)).Fx, 0.0);
    // kappa < 0  -> F_x < 0  (brake)
    EXPECT_LT(model_->compute(make_input(4000, -0.05, 0.0)).Fx, 0.0);
}

TEST_F(PacejkaMF96Fixture, PeakBoundedByFzMu) {
    const double Fz = 4000.0, mu = 1.0;
    double max_abs_Fy = 0.0;
    for (double a = -0.5; a <= 0.5; a += 0.005) {
        const auto out = model_->compute(make_input(Fz, 0.0, a, mu, mu));
        max_abs_Fy = std::max(max_abs_Fy, std::abs(out.Fy));
    }
    EXPECT_LE(max_abs_Fy, Fz * mu * tp_.D_lat * 1.001);
}

TEST_F(PacejkaMF96Fixture, MuScalesLinearly) {
    // In linear region, F should scale linearly with mu.
    const auto base   = model_->compute(make_input(4000, 0.0, 1e-4, 1.0, 1.0));
    const auto halved = model_->compute(make_input(4000, 0.0, 1e-4, 0.5, 0.5));
    EXPECT_NEAR(halved.Fy / base.Fy, 0.5, 0.005);
}

TEST_F(PacejkaMF96Fixture, FzScalesLinearlyInLinearRegion) {
    const auto small = model_->compute(make_input(2000, 0.0, 1e-4));
    const auto large = model_->compute(make_input(4000, 0.0, 1e-4));
    EXPECT_NEAR(large.Fy / small.Fy, 2.0, 0.005);
}

TEST_F(PacejkaMF96Fixture, MonotonicallyIncreasingNearZero) {
    // |F_y| is monotonically increasing in |alpha| for alpha in [0, peak].
    double prev_abs = 0.0;
    for (double a = 0.0; a < 0.05; a += 0.005) {
        const auto out = model_->compute(make_input(4000, 0.0, a));
        const double abs_Fy = std::abs(out.Fy);
        EXPECT_GE(abs_Fy, prev_abs - 1e-9);
        prev_abs = abs_Fy;
    }
}

// =============================================================================
// Linear tire
// =============================================================================
class LinearTireFixture : public ::testing::Test {
protected:
    void SetUp() override {
        model_ = vdsim::create_linear_tire();
        model_->initialize(tp_);
    }

    vdsim::TireParams tp_;
    std::unique_ptr<vdsim::ITireModel> model_;
};

TEST_F(LinearTireFixture, LateralExactSlope) {
    vdsim::ITireModel::Input in;
    in.Fz       = 4000.0;
    in.alpha    = 0.01;
    in.mu_lat   = 1.0;
    in.mu_long  = 1.0;
    const auto out = model_->compute(in);
    EXPECT_NEAR(out.Fy, -tp_.cornering_stiffness * 0.01, kTol);
}

TEST_F(LinearTireFixture, ZeroSlipZeroForce) {
    vdsim::ITireModel::Input in;
    in.Fz = 4000.0;
    const auto out = model_->compute(in);
    EXPECT_NEAR(out.Fx, 0.0, kTol);
    EXPECT_NEAR(out.Fy, 0.0, kTol);
}

TEST_F(LinearTireFixture, MuScales) {
    vdsim::ITireModel::Input in;
    in.alpha  = 0.01;
    in.mu_lat = 0.5;
    in.mu_long = 0.5;
    in.Fz     = 4000.0;
    const auto out = model_->compute(in);
    EXPECT_NEAR(out.Fy, -tp_.cornering_stiffness * 0.01 * 0.5, kTol);
}
