// Chrono KC (suspension kinematics) parity gate (VDSim side — NO Chrono code or link).
//
// Project Chrono (BSD-3) ships Chrono::Vehicle with hardpoint-based suspension templates
// (DoubleWishbone, MacPherson, ...) and a ChSuspensionTestRig that imposes wheel travel /
// steer and reads back camber / toe / track — i.e. a kinematics-and-compliance (KC) rig.
// We cross-check VDSim's hardpoint-emergent L5 geometry against it WITHOUT taking Chrono as
// a dependency: Chrono is run separately (external/chrono_kc/, links Chrono) to emit a
// reference CSV for the SAME hardpoints; this gate only reads that CSV + the same kin YAML,
// runs our hard-joint DAE travel maps (the geometry the L5 free-3D model uses), and bands the
// difference. It is a genuine EXTERNAL cross-check — an independent constrained-MBD engine —
// far stronger than the internal native-kinematics == DAE consistency check.
//
// If the CSV is absent (Chrono not built) the parity tests SKIP — they never fail the build
// for a missing external artifact, and never link Chrono. Regenerate: external/chrono_kc/.

#include "vdsim/multibody.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#ifndef VDSIM_SOURCE_DIR
#define VDSIM_SOURCE_DIR "."
#endif

namespace {

const std::string kRepo = VDSIM_SOURCE_DIR;
const std::string kKin  = kRepo + "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml";
const std::string kCsv  = kRepo + "/external/chrono_kc/reference/kc_dw_front_reference.csv";

struct Row {  // travel_m, steer_m, camber_deg, toe_deg, track_mm, caster_deg
    double travel, steer, camber, toe, track, caster;
};

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
        if (v.size() < 6) continue;
        out.push_back({v[0], v[1], v[2], v[3], v[4], v[5]});
    }
    return !out.empty();
}

std::unique_ptr<vdsim::mb::IHardJointDaeModel> make_dae() {
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(kKin);
    topo.kind = vdsim::mb::TopologyKind::DoubleWishbone;
    return vdsim::mb::create_hard_joint_dae_model(topo);
}

constexpr double kRadToDeg = 180.0 / M_PI;

}  // namespace

// The shared kin YAML must always load + sweep on our side, independent of whether a Chrono
// reference exists. Keeps the shared artifact continuously valid (and a regression lock on a
// nonzero camber gain).
TEST(ChronoKcParity, OurSideSweepsKin) {
    auto dae = make_dae();
    ASSERT_NE(dae, nullptr);
    const double c_lo = dae->travel_maps(-0.03, 0.0, vdsim::Vec3::Zero()).camber_rad;
    const double c_hi = dae->travel_maps(+0.03, 0.0, vdsim::Vec3::Zero()).camber_rad;
    EXPECT_TRUE(std::isfinite(c_lo) && std::isfinite(c_hi));
    EXPECT_GT(std::abs(c_hi - c_lo), 1e-3) << "double-wishbone must have a real camber gain";
}

// External cross-check: bump-travel camber/toe (steer = 0) must track Chrono::Vehicle's
// suspension test rig for the same hardpoints. Two independent constrained-MBD engines on
// identical geometry should agree to a few percent (residual = joint-idealisation / hardpoint
// mapping differences, documented in external/chrono_kc/README.md).
TEST(ChronoKcParity, TravelSweepMatchesChrono) {
    std::vector<Row> ref;
    if (!load_reference(ref)) GTEST_SKIP() << "no Chrono KC reference CSV — see "
        "external/chrono_kc/README.md";
    auto dae = make_dae();
    ASSERT_NE(dae, nullptr);

    const double rel = 0.08, floor_deg = 0.10;   // joint-idealisation residual band
    int n = 0;
    for (const auto& r : ref) {
        if (std::abs(r.steer) > 1e-6) continue;   // travel sweep only (steer convention-free)
        const auto tm = dae->travel_maps(r.travel, 0.0, vdsim::Vec3::Zero());
        const double cam = tm.camber_rad * kRadToDeg;
        const double toe = tm.toe_rad * kRadToDeg;
        const double ctol = std::max(floor_deg, rel * std::abs(r.camber));
        const double ttol = std::max(floor_deg, rel * std::abs(r.toe));
        EXPECT_LE(std::abs(cam - r.camber), ctol)
            << "camber @ travel=" << r.travel << ": ours=" << cam << " chrono=" << r.camber;
        EXPECT_LE(std::abs(toe - r.toe), ttol)
            << "toe @ travel=" << r.travel << ": ours=" << toe << " chrono=" << r.toe;
        ++n;
    }
    EXPECT_GT(n, 0);
    RecordProperty("kc_travel_points", n);
}
