// Work order 44: one level-id -> model factory (create_dynamics_for_level).
//
// E1/E2 on the C++ side: every accepted id builds its model, any other id
// throws std::invalid_argument naming the accepted set, and the direct-control
// session goes through the same factory (it used to fall back to L2).
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <utility>

#include "vdsim/interfaces.hpp"
#include "vdsim/sim_session.hpp"

namespace {

using Level = vdsim::IVehicleDynamics::Level;

/// Message fragment every refusal must carry so the caller sees what is allowed.
constexpr const char* kAccepted = "expected one of K, L0, L1, L2, L3, L4, L5";

}  // namespace

/// Each accepted id builds the model whose level() it names.
TEST(LevelFactory, AcceptedIdsBuildTheirModel) {
    const std::pair<const char*, Level> cases[] = {
        {"K", Level::Lk_Kinematic},  {"L0", Level::Lk_Kinematic},
        {"L1", Level::L1_Bicycle},   {"L2", Level::L2_SevenDOF},
        {"L3", Level::L3_FourteenDOF}, {"L4", Level::L4_Kinematic},
        {"L5", Level::L5_Stunt}};
    for (const auto& c : cases) {
        auto dyn = vdsim::create_dynamics_for_level(c.first);
        ASSERT_TRUE(dyn) << c.first;
        EXPECT_EQ(dyn->level(), c.second) << c.first;
    }
}

/// Unknown, wrong-case and empty ids are refused, never sent to L2.
TEST(LevelFactory, UnknownIdsThrowWithTheAcceptedSet) {
    for (const char* id : {"L9", "l3", ""}) {
        try {
            (void)vdsim::create_dynamics_for_level(id);
            FAIL() << "no exception for level \"" << id << "\"";
        } catch (const std::invalid_argument& e) {
            const std::string msg = e.what();
            EXPECT_NE(msg.find(kAccepted), std::string::npos) << msg;
            EXPECT_NE(msg.find(std::string("\"") + id + "\""), std::string::npos) << msg;
        }
    }
}

/// make_direct_control_session builds L5 by name and refuses an unknown id.
TEST(LevelFactory, DirectControlSessionUsesTheFactory) {
    const vdsim::VehicleParams vp;
    const vdsim::TireSetup ts{vdsim::TireParams{}};
    const vdsim::SolverParams sp;
    vdsim::DirectControlSessionOptions opts;
    opts.level = "L5";
    auto sess = vdsim::make_direct_control_session(vp, ts, sp, opts);
    ASSERT_TRUE(sess);
    EXPECT_EQ(sess->dynamics().level(), Level::L5_Stunt);

    opts.level = "L9";
    EXPECT_THROW((void)vdsim::make_direct_control_session(vp, ts, sp, opts),
                 std::invalid_argument);
}
