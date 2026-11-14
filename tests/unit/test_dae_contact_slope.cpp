// Work order 49a gate P9 (independent geometry), fixed before the implementation commit.
//
// P9: at zero travel and zero rack, the code's contact-patch lateral slope dy_cp/dz_v
//     (detail::contact_patch_lateral_slope on the front double-wishbone corner model) agrees
//     with the independent solver tools/kinematics/dw_3d_solver.py (no vdsim import) within
//     +-2 %, front left and front right, antidive (tests/fixtures/dw_front_antidive_sample)
//     and sports (dw_front_sports). Right = y mirror of the left corner, as the L5 core
//     mounts it (slope negated).
//
// Reference: python3 tests/scripts/gen_dae_contact_slope_ref.py  (R = 0.32 m, the generic
// sedan wheel_radius_nominal; h = 1e-3 m; exact lowest rim point from the solver's wheel
// centre and spin axis). Values below copied from its output before any code measurement.
// Seed: the corner state after IHardJointDaeModel::initialize at zero travel.
#include "free_3d_jacking_internal.hpp"
#include "vdsim/multibody.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <string>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;
constexpr double kR = 0.32;        // generic_sedan wheel_radius_nominal [m]
constexpr double kTol = 0.02;      // P9 gate, relative

struct Ref {
    const char* name;
    std::string path;
    double dy_cp;                  // independent dy_cp/dz_v, left corner [-]
};

void p9(const Ref& ref) {
    const auto topo = mb::SuspensionTopology::from_yaml(ref.path);
    const auto dae = mb::create_hard_joint_dae_model(topo);
    ASSERT_TRUE(dae);
    for (const double side : {+1.0, -1.0}) {      // +1 front left, -1 front right (mirror)
        mb::HardJointCornerState st;
        mb::PrescribedCornerMotion mot;            // zero travel, zero rack
        dae->initialize(st, mot);
        const double code = side * detail::contact_patch_lateral_slope(
                                       *dae, 0.0, side * mot.steer_rack_dy, st.knuckle_aa, kR);
        const double expected = side * ref.dy_cp;
        const double rel = code / expected - 1.0;
        std::printf("P9 %s %s code=%.9e ref=%.9e rel=%+.4f %% (gate +-2 %%)\n", ref.name,
                    side > 0 ? "FL" : "FR", code, expected, 100.0 * rel);
        EXPECT_LE(std::abs(rel), kTol) << ref.name << (side > 0 ? " FL" : " FR");
    }
}

}  // namespace

TEST(DaeContactSlope, P9_AntidiveFrontMatchesIndependentSolver) {
    p9({"antidive", kSrc + "/tests/fixtures/dw_front_antidive_sample.yaml",
        5.256018640382e-02});
}

TEST(DaeContactSlope, P9_SportsFrontMatchesIndependentSolver) {
    p9({"sports", kSrc + "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml",
        6.498077527345e-02});
}
