// T1 — Magic Formula (MF2002 / .tir) evaluator sanity.
//
// The .tir parser + MF2002 evaluator already exist (magic_formula.hpp). This is
// the first T1 coverage: parse a *synthetic, public* .tir written at runtime
// (no .tir is committed — repo .gitignore blocks *.tir and real coefficients are
// confidential) and check the pure/combined-slip forces are physically sane.

#include "vdsim/interfaces.hpp"
#include "vdsim/magic_formula.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace {

// Synthetic, representative passenger-car coefficients (NOT measured data).
const char* kSyntheticTir = R"TIR(
$ synthetic public coefficients for testing only
[MODEL]
[DIMENSION]
UNLOADED_RADIUS = 0.31
[VERTICAL]
FNOMIN = 4000
[LONGITUDINAL_COEFFICIENTS]
PCX1 = 1.65
PDX1 = 1.20
PDX2 = -0.05
PEX1 = 0.50
PKX1 = 20.0
[LATERAL_COEFFICIENTS]
PCY1 = 1.30
PDY1 = 1.10
PDY2 = -0.05
PEY1 = -0.50
PKY1 = -15.0
[ALIGNING_COEFFICIENTS]
QBZ1 = 6.0
QDZ1 = 0.10
)TIR";

fs::path write_synthetic_tir() {
    const auto p = fs::temp_directory_path() /
        ("vdsim_synth_" + std::to_string(static_cast<long>(::getpid())) + ".tir");
    std::ofstream(p) << kSyntheticTir;
    return p;
}

vdsim::ITireModel::Input slip_input(double kappa, double alpha, double Fz) {
    vdsim::ITireModel::Input in;
    in.Fz = Fz;
    in.kappa = kappa;
    in.alpha = alpha;
    in.mu_long = 1.0;
    in.mu_lat = 1.0;
    in.Vx_wheel = 15.0;
    return in;
}

}  // namespace

TEST(Mf2002Catalog, ParsesSyntheticTir) {
    const auto p = write_synthetic_tir();
    const vdsim::MFCoeffs c = vdsim::parse_tir(p.string());
    EXPECT_TRUE(c.has("FNOMIN"));
    EXPECT_DOUBLE_EQ(c.g("FNOMIN", 0.0), 4000.0);
    EXPECT_DOUBLE_EQ(c.g("PKX1", 0.0), 20.0);
    EXPECT_DOUBLE_EQ(c.g("missing", -1.0), -1.0);   // default for absent key
    fs::remove(p);
}

TEST(Mf2002Catalog, SampleTirRunsAndIsSane) {
    const auto p = write_synthetic_tir();
    auto tire = vdsim::create_magic_formula_tire_from_tir(p.string());
    fs::remove(p);
    ASSERT_NE(tire, nullptr);

    const double Fz = 4000.0, muFz = 1.0 * Fz;

    // Zero slip -> ~zero force.
    {
        const auto o = tire->compute(slip_input(0.0, 0.0, Fz));
        EXPECT_LT(std::abs(o.Fx), 50.0);
        EXPECT_LT(std::abs(o.Fy), 50.0);
    }
    // Pure longitudinal: drive slip -> forward force, bounded by ~mu*Fz.
    {
        const auto o = tire->compute(slip_input(0.10, 0.0, Fz));
        EXPECT_GT(o.Fx, 500.0);
        EXPECT_LT(o.Fx, 1.3 * muFz);
        EXPECT_LT(std::abs(o.Fy), 200.0);          // little lateral at alpha=0
    }
    // Pure lateral: slip angle -> lateral force develops, bounded.
    {
        const auto o = tire->compute(slip_input(0.0, 0.10, Fz));
        EXPECT_GT(std::abs(o.Fy), 500.0);
        EXPECT_LT(std::abs(o.Fy), 1.3 * muFz);
    }
    // Saturation: large longitudinal slip stays within the friction budget.
    {
        const auto o = tire->compute(slip_input(0.8, 0.0, Fz));
        EXPECT_LT(std::abs(o.Fx), 1.5 * muFz);
    }
    // Below the load floor -> no force.
    {
        const auto o = tire->compute(slip_input(0.1, 0.1, 0.5));
        EXPECT_DOUBLE_EQ(o.Fx, 0.0);
        EXPECT_DOUBLE_EQ(o.Fy, 0.0);
    }
}
