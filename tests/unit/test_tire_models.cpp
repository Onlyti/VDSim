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

// =============================================================================
// Pacejka combined slip (friction ellipse) and aligning moment
// =============================================================================
class PacejkaCombinedFixture : public ::testing::Test {
protected:
    void SetUp() override {
        model_ = vdsim::create_pacejka_mf96();
        model_->initialize(tp_);
    }
    vdsim::ITireModel::Input make_input(double Fz, double kappa, double alpha,
                                         double mu = 1.0) {
        vdsim::ITireModel::Input in;
        in.Fz = Fz; in.kappa = kappa; in.alpha = alpha;
        in.mu_long = mu; in.mu_lat = mu; in.Vx_wheel = 10.0;
        return in;
    }
    vdsim::TireParams tp_;
    std::unique_ptr<vdsim::ITireModel> model_;
};

TEST_F(PacejkaCombinedFixture, FrictionEllipseBound) {
    const double Fz = 4000.0;
    const double Fx_max = tp_.D_long * Fz;
    const double Fy_max = tp_.D_lat  * Fz;
    double max_violation = 0.0;
    for (double k = -0.30; k <= 0.30; k += 0.02) {
        for (double a = -0.30; a <= 0.30; a += 0.02) {
            const auto out = model_->compute(make_input(Fz, k, a));
            const double rx = out.Fx / Fx_max;
            const double ry = out.Fy / Fy_max;
            const double ratio = std::sqrt(rx * rx + ry * ry);
            max_violation = std::max(max_violation, ratio - 1.0);
        }
    }
    EXPECT_LE(max_violation, 1e-9);
}

TEST_F(PacejkaCombinedFixture, PureSlipUnchangedByCombinedFlag) {
    const auto a_off = model_->compute(make_input(4000, 0.10, 0.0));
    const auto b_off = model_->compute(make_input(4000, 0.0, 0.10));
    vdsim::TireParams tp_off = tp_;
    tp_off.combined_slip_enabled = false;
    auto m2 = vdsim::create_pacejka_mf96();
    m2->initialize(tp_off);
    const auto a2 = m2->compute(make_input(4000, 0.10, 0.0));
    const auto b2 = m2->compute(make_input(4000, 0.0, 0.10));
    EXPECT_NEAR(a_off.Fx, a2.Fx, 1e-9);
    EXPECT_NEAR(a_off.Fy, a2.Fy, 1e-9);
    EXPECT_NEAR(b_off.Fx, b2.Fx, 1e-9);
    EXPECT_NEAR(b_off.Fy, b2.Fy, 1e-9);
}

TEST_F(PacejkaCombinedFixture, CombinedReducesPureForceMagnitudes) {
    const auto pure_x = model_->compute(make_input(4000, 0.15, 0.0));
    const auto pure_y = model_->compute(make_input(4000, 0.0, 0.15));
    const auto comb   = model_->compute(make_input(4000, 0.15, 0.15));
    EXPECT_LT(std::abs(comb.Fx), std::abs(pure_x.Fx));
    EXPECT_LT(std::abs(comb.Fy), std::abs(pure_y.Fy));
}

TEST_F(PacejkaCombinedFixture, MzZeroWhenAlphaZero) {
    const auto out = model_->compute(make_input(4000, 0.05, 0.0));
    EXPECT_NEAR(out.Mz, 0.0, 1e-9);
}

TEST_F(PacejkaCombinedFixture, MzSignOppositeFy) {
    // alpha > 0 -> Fy < 0 -> Mz > 0 (self-aligning)
    const auto pos = model_->compute(make_input(4000, 0.0, 0.05));
    EXPECT_LT(pos.Fy, 0.0);
    EXPECT_GT(pos.Mz, 0.0);
    const auto neg = model_->compute(make_input(4000, 0.0, -0.05));
    EXPECT_GT(neg.Fy, 0.0);
    EXPECT_LT(neg.Mz, 0.0);
}

TEST_F(PacejkaCombinedFixture, MzLinearRegionMatchesPneumaticTrail) {
    const double alpha = 1e-4;
    const auto out = model_->compute(make_input(4000, 0.0, alpha));
    const double expected_Mz = -tp_.pneumatic_trail * out.Fy;
    EXPECT_NEAR(out.Mz, expected_Mz, std::abs(expected_Mz) * 1e-4);
}

TEST_F(PacejkaCombinedFixture, CamberAddsLateralForce) {
    vdsim::TireParams tp2 = tp_;
    tp2.camber_stiffness = 1.5;
    auto m2 = vdsim::create_pacejka_mf96();
    m2->initialize(tp2);

    auto in = make_input(4000, 0.0, 0.0);   // pure camber
    in.gamma = 0.05;
    const auto out = m2->compute(in);
    EXPECT_LT(out.Fy, 0.0);                  // camber > 0 -> -y direction Fy
    EXPECT_NEAR(out.Fy, -1.5 * 0.05 * 4000.0, 1.0);
}

TEST_F(PacejkaCombinedFixture, CamberZeroByDefault) {
    // Default camber_stiffness = 0 -> no effect.
    auto in = make_input(4000, 0.0, 0.05);
    in.gamma = 0.10;
    const auto out_g = model_->compute(in);
    in.gamma = 0.0;
    const auto out_0 = model_->compute(in);
    EXPECT_NEAR(out_g.Fy, out_0.Fy, 1e-9);
}

TEST_F(PacejkaCombinedFixture, MzDecreasesAtLargeAlpha) {
    // Trail falls off with |alpha|, so Mz peaks and then decreases relative
    // to |Fy| at high slip.  Compare |Mz/Fy| at small vs large alpha.
    const auto small = model_->compute(make_input(4000, 0.0, 0.02));
    const auto large = model_->compute(make_input(4000, 0.0, 0.30));
    const double t_small = std::abs(small.Mz / small.Fy);
    const double t_large = std::abs(large.Mz / large.Fy);
    EXPECT_GT(t_small, t_large);
}
