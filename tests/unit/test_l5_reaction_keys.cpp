// L5 torque-reaction keys and knuckle helpers (work order 43): key round trip and
// rejection, defaults, strut-force / target queries, the below-L5 and
// equivalent-path warnings, and J6 (knuckle pitch rate, reaction split).
#include "vdsim/contact.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

#include "free_3d_reaction_internal.hpp"

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace vdsim;
using TR = VehicleParams::TorqueReaction;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;

std::string base_yaml() {
    std::ifstream f(kSrc + "/configs/vehicles/generic_sedan.yaml");
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string write_tmp(const std::string& name, const std::string& text) {
    const std::string p = ::testing::TempDir() + name;
    std::ofstream(p) << text;
    return p;
}

std::string read_all(const std::string& p) {
    std::ifstream f(p);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/// Count occurrences of `needle` in everything the default logger writes while `fn` runs.
template <typename F>
int count_log(const std::string& needle, F&& fn) {
    std::ostringstream os;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(os);
    auto logger = spdlog::default_logger();
    logger->sinks().push_back(sink);
    fn();
    logger->flush();
    logger->sinks().pop_back();
    const std::string s = os.str();
    int n = 0;
    for (std::size_t p = s.find(needle); p != std::string::npos; p = s.find(needle, p + 1)) ++n;
    return n;
}

std::unique_ptr<SimSession> l5_session(const VehicleParams& vp, bool strut, bool dae) {
    SolverParams sp;
    sp.stunt_physics = true;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 16;
    sp.l5_spatial_suspension = strut;
    SimConfig cfg;
    cfg.nominal_dt = 1e-3;
    auto s = std::make_unique<SimSession>(create_stunt_dof(), create_flat_ground(0.0, 1.0),
                                          vp, TireSetup(TireParams{}), sp, cfg);
    if (dae) {
        free_3d_attach_multibody(s->dynamics(), true, mb::SuspensionTopology::from_yaml(
            kSrc + "/tests/fixtures/dw_front_antidive_sample.yaml"), true);
        free_3d_attach_multibody(s->dynamics(), false, mb::SuspensionTopology::from_yaml(
            kSrc + "/configs/parts/susp_kinematics/kin/ta_rear_sedan.yaml"), true);
    }
    return s;
}

std::unique_ptr<mb::IHardJointDaeModel> dae(const std::string& rel) {
    return mb::create_hard_joint_dae_model(mb::SuspensionTopology::from_yaml(kSrc + rel));
}

}  // namespace

TEST(L5ReactionKeys, YamlRoundTripAndUnsetIsUntouched) {
    const auto p = write_tmp("rx_keys.yaml", base_yaml() +
        "\ndrive_reaction_front: unsprung\nbrake_reaction_rear: sprung\n");
    const auto vp = VehicleParams::from_yaml(p);
    ASSERT_TRUE(vp.drive_reaction_front.has_value());
    EXPECT_TRUE(*vp.drive_reaction_front == TR::Unsprung);
    EXPECT_FALSE(vp.drive_reaction_rear.has_value());
    EXPECT_FALSE(vp.brake_reaction_front.has_value());
    ASSERT_TRUE(vp.brake_reaction_rear.has_value());
    EXPECT_TRUE(*vp.brake_reaction_rear == TR::Sprung);

    const auto out = ::testing::TempDir() + "rx_keys_out.yaml";
    vp.to_yaml(out);
    const auto back = VehicleParams::from_yaml(out);
    EXPECT_TRUE(back.drive_reaction_front == vp.drive_reaction_front);
    EXPECT_TRUE(back.drive_reaction_rear == vp.drive_reaction_rear);
    EXPECT_TRUE(back.brake_reaction_front == vp.brake_reaction_front);
    EXPECT_TRUE(back.brake_reaction_rear == vp.brake_reaction_rear);

    // A vehicle without the keys writes none of them (to_yaml output unchanged).
    const auto plain = ::testing::TempDir() + "rx_plain_out.yaml";
    VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml").to_yaml(plain);
    EXPECT_EQ(read_all(plain).find("_reaction_"), std::string::npos);
}

TEST(L5ReactionKeys, UnknownValueThrows) {
    const auto p = write_tmp("rx_bad.yaml", base_yaml() + "\nbrake_reaction_front: inboard\n");
    EXPECT_THROW(VehicleParams::from_yaml(p), std::invalid_argument);
    EXPECT_THROW(parse_torque_reaction("Sprung", "drive_reaction_rear"), std::invalid_argument);
    EXPECT_TRUE(parse_torque_reaction("unsprung", "k") == TR::Unsprung);
}

TEST(L5ReactionKeys, DefaultsDriveSprungBrakeUnsprung) {
    VehicleParams vp;
    for (const bool front : {true, false}) {
        EXPECT_TRUE(resolved_torque_reaction(vp, true, front) == TR::Sprung);
        EXPECT_TRUE(resolved_torque_reaction(vp, false, front) == TR::Unsprung);
    }
    vp.drive_reaction_rear = TR::Unsprung;
    vp.brake_reaction_front = TR::Sprung;
    EXPECT_TRUE(resolved_torque_reaction(vp, true, false) == TR::Unsprung);
    EXPECT_TRUE(resolved_torque_reaction(vp, false, true) == TR::Sprung);
}

TEST(L5ReactionKeys, StrutForceIsNaNOffTheStrutPath) {
    VehicleParams vp;
    const auto pen = l5_session(vp, false, false);
    for (const double f : free_3d_strut_force(pen->dynamics())) EXPECT_TRUE(std::isnan(f));
    auto l2 = create_seven_dof();
    for (const double f : free_3d_strut_force(*l2)) EXPECT_TRUE(std::isnan(f));
    const auto strut = l5_session(vp, true, false);
    State s0;
    s0.position.z() = vp.cg_height;
    strut->reset_settled(s0);
    for (const double f : free_3d_strut_force(strut->dynamics())) {
        EXPECT_TRUE(std::isfinite(f));
        EXPECT_GT(f, 0.0);   // static preload
    }
}

TEST(L5ReactionKeys, EffectiveTargetsPerPath) {
    VehicleParams vp;
    const auto pen = l5_session(vp, false, false);
    const auto a = free_3d_torque_reaction_targets(pen->dynamics());
    EXPECT_EQ(a[0], "sprung");      EXPECT_EQ(a[1], "sprung");
    EXPECT_EQ(a[2], "equivalent");  EXPECT_EQ(a[3], "equivalent");
    const auto dae_s = l5_session(vp, true, true);
    const auto b = free_3d_torque_reaction_targets(dae_s->dynamics());
    EXPECT_EQ(b[2], "unsprung");    EXPECT_EQ(b[3], "unsprung");
    auto l2 = create_seven_dof();
    for (const auto& t : free_3d_torque_reaction_targets(*l2)) EXPECT_TRUE(t.empty());
}

TEST(L5ReactionKeys, BelowL5WarnsOnce) {
    VehicleParams vp;
    vp.brake_reaction_front = TR::Sprung;
    const int n = count_log("ignored below L5", [&] {
        for (int k = 0; k < 2; ++k) {
            SimSession s(create_seven_dof(), create_flat_ground(0.0, 1.0), vp,
                         TireSetup(TireParams{}), SolverParams{}, SimConfig{});
        }
    });
    EXPECT_EQ(n, 1);
}

TEST(L5ReactionKeys, ExplicitUnsprungWithoutLinkageWarnsOnce) {
    VehicleParams vp;
    const int none = count_log("has no corner linkage", [&] {
        auto s = l5_session(vp, false, false);   // default brake unsprung: no warning
        s->tick(1e-3);
    });
    EXPECT_EQ(none, 0);
    vp.drive_reaction_rear = TR::Unsprung;
    const int n = count_log("has no corner linkage", [&] {
        for (int k = 0; k < 2; ++k) {
            auto s = l5_session(vp, true, false);
            s->tick(1e-3);
            s->tick(1e-3);
        }
    });
    EXPECT_EQ(n, 1);
}

// ---- J6 ----
// (a) knuckle pitch rate at the static travel vs the instant-centre hand calc, +/-2 %.
TEST(L5ReactionKnuckle, J6a_PitchRateMatchesHandCalc) {
    const auto f = dae("/tests/fixtures/dw_front_antidive_sample.yaml");
    const auto r = dae("/configs/parts/susp_kinematics/kin/ta_rear_sedan.yaml");
    const auto tf = detail::knuckle_pitch_rate(*f, 0.0, 0.0, Vec3::Zero());
    const auto tr = detail::knuckle_pitch_rate(*r, 0.0, 0.0, Vec3::Zero());
    ASSERT_TRUE(tf && tr);
    std::printf("[J6a] front %.5f rad/m (hand -0.4975), rear %.5f rad/m (hand +1.5625)\n",
                *tf, *tr);
    EXPECT_NEAR(*tf, -0.4975, 0.02 * 0.4975);
    EXPECT_NEAR(*tr, 1.5625, 0.02 * 1.5625);
}

// (b) +/-0.5 mm vs +/-1 mm central difference, <= 1 %.
TEST(L5ReactionKnuckle, J6b_StepSizeInsensitive) {
    for (const char* rel : {"/tests/fixtures/dw_front_antidive_sample.yaml",
                            "/configs/parts/susp_kinematics/kin/ta_rear_sedan.yaml"}) {
        const auto d = dae(rel);
        const auto a = detail::knuckle_pitch_rate(*d, 0.0, 0.0, Vec3::Zero(), 0.5e-3);
        const auto b = detail::knuckle_pitch_rate(*d, 0.0, 0.0, Vec3::Zero(), 1.0e-3);
        ASSERT_TRUE(a && b) << rel;
        std::printf("[J6b] %s dz 0.5 mm %.6f, 1 mm %.6f rad/m\n", rel, *a, *b);
        EXPECT_LE(std::abs(*a - *b), 0.01 * std::abs(*b)) << rel;
    }
}

// (c) the split carries exactly M_pitch * y about any point, no net force, and the
// particle force does the knuckle's virtual work along the path.
TEST(L5ReactionKnuckle, J6c_SplitConservesMoment) {
    const double M = -1234.5, th = -0.4975;
    const Vec3 t(-0.0896, 0.031, 1.0);
    const Vec3 r_wheel(1.21, 0.78, -0.24), r_mount(1.2, 0.75, -0.23);
    const auto s = detail::split_knuckle_reaction(M, th, t, r_wheel, r_mount);
    const Vec3 net_f = s.force_on_wheel + s.force_on_body;
    const Vec3 moment = r_wheel.cross(s.force_on_wheel) + r_mount.cross(s.force_on_body)
                      + s.couple_on_body;
    EXPECT_LE(net_f.norm(), 1e-12 * std::abs(M));
    EXPECT_NEAR(moment.x(), 0.0, 1e-9 * std::abs(M));
    EXPECT_NEAR(moment.y(), M, 1e-9 * std::abs(M));
    EXPECT_NEAR(moment.z(), 0.0, 1e-9 * std::abs(M));
    EXPECT_NEAR(s.force_on_wheel.dot(t), M * th, 1e-9 * std::abs(M * th));
}
