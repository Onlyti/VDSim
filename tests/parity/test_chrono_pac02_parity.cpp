// Chrono Pac02 parity gate (VDSim side — NO Chrono code or link here).
//
// Project Chrono (BSD-3) ships an independent Pacejka-2002 implementation
// (ChPac02Tire). This test cross-checks our own MF2002 evaluator against it WITHOUT
// taking Chrono as a dependency: Chrono is run separately (conda env, see
// external/chrono_parity/gen_pac02_reference.py) to emit a reference CSV of
// (Fz, kappa, alpha, gamma -> Fx, Fy, Mz) for the *same* public .tir. This gate only
// reads that CSV + the same .tir, runs our evaluator, and bands the difference.
//
// If the CSV is absent (Chrono env not built) the test SKIPS — it never fails the
// build for a missing external artifact, and it never links Chrono.
//
// Regenerate the reference:  see external/chrono_parity/README.md

#include "vdsim/interfaces.hpp"
#include "vdsim/magic_formula.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

const std::string kRepo = VDSIM_SOURCE_DIR;
const std::string kTir = kRepo + "/external/chrono_parity/sample_pac02.tir";
const std::string kCsv = kRepo + "/external/chrono_parity/reference/pac02_reference.csv";

struct Row {
    double Fz, kappa, alpha, gamma, Fx, Fy, Mz;
};

bool load_reference(std::vector<Row>& out) {
    std::ifstream in(kCsv);
    if (!in) return false;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> v;
        while (std::getline(ss, cell, ',')) v.push_back(std::stod(cell));
        if (v.size() < 7) continue;
        out.push_back({v[0], v[1], v[2], v[3], v[4], v[5], v[6]});
    }
    return !out.empty();
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

// Pure + combined slip force parity against the Chrono Pac02 reference. Fx/Fy are
// the load-bearing comparison; Mz (trail/aligning model differs more across MF
// implementations) is reported but not gated here.
TEST(ChronoPac02Parity, ForcesMatchReference) {
    std::vector<Row> ref;
    if (!load_reference(ref)) {
        GTEST_SKIP() << "no Chrono reference CSV at " << kCsv
                     << " — run external/chrono_parity/gen_pac02_reference.py "
                        "in the chrono conda env to generate it.";
    }

    auto tire = vdsim::create_magic_formula_tire_from_tir(kTir);
    ASSERT_NE(tire, nullptr);

    // Band: relative tolerance with an absolute floor so near-zero points (e.g.
    // Fy at zero slip) are not judged by relative error. Tightened after the first
    // real reference is inspected; loosen only with a documented reason.
    const double rel_tol = 0.06;     // 6 %
    const double abs_floor = 60.0;   // [N]

    double worst_fx = 0.0, worst_fy = 0.0;
    int n = 0, n_bad = 0;
    for (const auto& r : ref) {
        vdsim::ITireModel::Input in;
        in.Fz = r.Fz; in.kappa = r.kappa; in.alpha = r.alpha; in.gamma = r.gamma;
        in.mu_long = 1.0; in.mu_lat = 1.0; in.Vx_wheel = 15.0;
        const auto o = tire->compute(in);

        const double efx = std::abs(o.Fx - r.Fx);
        const double efy = std::abs(o.Fy - r.Fy);
        const double tfx = std::max(abs_floor, rel_tol * std::abs(r.Fx));
        const double tfy = std::max(abs_floor, rel_tol * std::abs(r.Fy));
        worst_fx = std::max(worst_fx, efx - tfx);
        worst_fy = std::max(worst_fy, efy - tfy);
        ++n;
        if (efx > tfx || efy > tfy) {
            ++n_bad;
            EXPECT_LE(efx, tfx) << "Fx @ Fz=" << r.Fz << " k=" << r.kappa
                                << " a=" << r.alpha << ": ours=" << o.Fx
                                << " chrono=" << r.Fx;
            EXPECT_LE(efy, tfy) << "Fy @ Fz=" << r.Fz << " k=" << r.kappa
                                << " a=" << r.alpha << ": ours=" << o.Fy
                                << " chrono=" << r.Fy;
        }
    }
    RecordProperty("points", n);
    RecordProperty("out_of_band", n_bad);
    SUCCEED() << n << " points compared, " << n_bad << " out of band ("
              << rel_tol * 100 << "% / " << abs_floor << " N).";
}
