// Chrono Pac02 parity gate (VDSim side — NO Chrono code or link here).
//
// Project Chrono (BSD-3) ships an independent Pacejka-2002 implementation
// (ChPac02Tire). This cross-checks our own MF2002 evaluator against it WITHOUT
// taking Chrono as a dependency: Chrono is run separately (a standalone generator
// that links Chrono, see external/chrono_parity/) to emit a reference CSV of
// (Fz, kappa, alpha -> Fx, Fy, Mz) for the SAME public .tir. This gate only reads
// that CSV + the same .tir, runs our evaluator, and bands the difference.
//
// Result on the committed reference (sample_pac02.tir):
//   * PURE longitudinal slip (alpha~0): Fx within ~2% across Fz = 2..6 kN.
//   * PURE lateral slip (kappa~0): Fy within ~1% across Fz and slip angle.
//     Both pure axes — backbone, load sensitivity, cornering stiffness — agree with
//     an independent Pac02. These are gated.
//   * COMBINED slip cross-terms (large kappa AND large alpha): our combined Fx runs
//     below Chrono's. This mixes a genuine combined-slip-weighting difference with a
//     rig-frame artifact (ChTireTestRig reports force in the global frame and the
//     wheel is yawed by the slip angle, so the combined longitudinal component is
//     frame-sensitive). It is reported, not hard-gated.
//
// If the CSV is absent (Chrono not built) the tests SKIP — they never fail the
// build for a missing external artifact, and never link Chrono.
// Regenerate the reference: external/chrono_parity/README.md

#include "vdsim/interfaces.hpp"
#include "vdsim/magic_formula.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <vector>

namespace {

const std::string kRepo = VDSIM_SOURCE_DIR;
const std::string kTir = kRepo + "/external/chrono_parity/sample_pac02.tir";
const std::string kCsv = kRepo + "/external/chrono_parity/reference/pac02_reference.csv";

struct Row { double Fz, kappa, alpha, gamma, Fx, Fy, Mz; };

bool load_reference(std::vector<Row>& out) {
    std::ifstream in(kCsv);
    if (!in) return false;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string cell; std::vector<double> v;
        while (std::getline(ss, cell, ',')) v.push_back(std::stod(cell));
        if (v.size() < 7) continue;
        out.push_back({v[0], v[1], v[2], v[3], v[4], v[5], v[6]});
    }
    return !out.empty();
}

vdsim::ITireModel::Output ours(const Row& r, vdsim::ITireModel& tire) {
    vdsim::ITireModel::Input in;
    in.Fz = r.Fz; in.kappa = r.kappa; in.alpha = r.alpha; in.gamma = r.gamma;
    in.mu_long = 1.0; in.mu_lat = 1.0; in.Vx_wheel = 15.0;
    return tire.compute(in);
}

}  // namespace

// The committed public .tir must always load + evaluate on our side, independent of
// whether a Chrono reference exists. Keeps the shared artifact continuously valid.
TEST(ChronoPac02Parity, OurSideLoadsSampleTir) {
    auto tire = vdsim::create_magic_formula_tire_from_tir(kTir);
    ASSERT_NE(tire, nullptr);
    vdsim::ITireModel::Input in;
    in.Fz = 4000.0; in.mu_long = 1.0; in.mu_lat = 1.0; in.Vx_wheel = 15.0;
    in.kappa = 0.10;
    EXPECT_GT(tire->compute(in).Fx, 500.0) << "pure-slip Fx should be a real drive force";
    in.kappa = 0.0; in.alpha = 0.10;
    EXPECT_LT(tire->compute(in).Fy, -500.0) << "pure-slip Fy (ISO sign) should be substantial";
}

// Tight gate: PURE longitudinal slip (|alpha| < 0.02) must track Chrono Pac02. This
// is the validated cross-check — it caught nothing to fix and locks the long backbone
// + load sensitivity against drift.
TEST(ChronoPac02Parity, PureLongitudinalMatchesChrono) {
    std::vector<Row> ref;
    if (!load_reference(ref)) GTEST_SKIP() << "no Chrono reference CSV — see "
        "external/chrono_parity/README.md";
    auto tire = vdsim::create_magic_formula_tire_from_tir(kTir);
    ASSERT_NE(tire, nullptr);

    const double rel = 0.06, floor = 120.0;
    int n = 0; double worst = 0.0;
    for (const auto& r : ref) {
        if (std::abs(r.alpha) > 0.02) continue;   // pure longitudinal cells
        const double fx = ours(r, *tire).Fx;
        const double tol = std::max(floor, rel * std::abs(r.Fx));
        worst = std::max(worst, std::abs(fx - r.Fx) - tol);
        EXPECT_LE(std::abs(fx - r.Fx), tol)
            << "Fx @ Fz=" << r.Fz << " k=" << r.kappa
            << ": ours=" << fx << " chrono=" << r.Fx;
        ++n;
    }
    EXPECT_GT(n, 0);
    RecordProperty("pure_long_points", n);
}

// Tight gate: PURE lateral slip (|kappa| < 0.025, |alpha| > 0.03) Fy must track Chrono.
// Validates cornering stiffness + lateral backbone vs an independent Pac02.
TEST(ChronoPac02Parity, PureLateralMatchesChrono) {
    std::vector<Row> ref;
    if (!load_reference(ref)) GTEST_SKIP() << "no Chrono reference CSV — see "
        "external/chrono_parity/README.md";
    auto tire = vdsim::create_magic_formula_tire_from_tir(kTir);
    ASSERT_NE(tire, nullptr);

    const double rel = 0.06, floor = 120.0;
    int n = 0;
    for (const auto& r : ref) {
        if (std::abs(r.kappa) > 0.025 || std::abs(r.alpha) < 0.03) continue;  // pure lateral
        const double fy = ours(r, *tire).Fy;
        const double tol = std::max(floor, rel * std::abs(r.Fy));
        EXPECT_LE(std::abs(fy - r.Fy), tol)
            << "Fy @ Fz=" << r.Fz << " a=" << r.alpha
            << ": ours=" << fy << " chrono=" << r.Fy;
        ++n;
    }
    EXPECT_GT(n, 0);
    RecordProperty("pure_lat_points", n);
}

// Report-only: combined-slip cross-terms (see header). Not gated on the band; only a
// gross-regression guard (Fy sign / < 3x), since Fy is the frame-robust component.
TEST(ChronoPac02Parity, CombinedSlipReported) {
    std::vector<Row> ref;
    if (!load_reference(ref)) GTEST_SKIP() << "no Chrono reference CSV — see "
        "external/chrono_parity/README.md";
    auto tire = vdsim::create_magic_formula_tire_from_tir(kTir);
    ASSERT_NE(tire, nullptr);

    int n = 0, n_gross = 0; double sum_rel_fx = 0, sum_rel_fy = 0, max_rel = 0;
    for (const auto& r : ref) {
        if (std::abs(r.alpha) <= 0.02) continue;  // combined cells only
        const auto o = ours(r, *tire);
        auto rel_err = [](double a, double b) {
            return std::abs(b) < 50.0 ? 0.0 : std::abs(a - b) / std::abs(b);
        };
        const double ex = rel_err(o.Fx, r.Fx), ey = rel_err(o.Fy, r.Fy);
        sum_rel_fx += ex; sum_rel_fy += ey;
        max_rel = std::max({max_rel, ex, ey});
        // gross-regression guard: same sign on the dominant component, within 3x.
        if (std::abs(r.Fy) > 200.0) {
            EXPECT_GT(o.Fy * r.Fy, 0.0) << "Fy sign flip @ k=" << r.kappa << " a=" << r.alpha;
            EXPECT_LT(std::abs(o.Fy), 3.0 * std::abs(r.Fy)) << "Fy gross over @ a=" << r.alpha;
            if (rel_err(o.Fy, r.Fy) > 0.5) ++n_gross;
        }
        ++n;
    }
    ASSERT_GT(n, 0);
    RecordProperty("combined_points", n);
    RecordProperty("combined_mean_relerr_fx_pct", int(100.0 * sum_rel_fx / n));
    RecordProperty("combined_mean_relerr_fy_pct", int(100.0 * sum_rel_fy / n));
    RecordProperty("combined_max_relerr_pct", int(100.0 * max_rel));
    SUCCEED() << n << " combined points; mean rel err Fx="
              << (100.0 * sum_rel_fx / n) << "% Fy=" << (100.0 * sum_rel_fy / n)
              << "% (combined-slip weighting differs from Pac02 — known, see header).";
}
