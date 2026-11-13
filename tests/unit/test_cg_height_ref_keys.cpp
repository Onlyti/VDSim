// cg_height_ref key (work order 47 G7, C++ side): YAML round trip, ValueError
// (std::invalid_argument) for unknown values and for h_s <= R + 0.01 m on the L5
// strut path, one warning below the strut path, and no effect where it is ignored.
#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;

std::string sedan_yaml() {
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

SolverParams l5_solver(bool strut) {
    SolverParams sp;
    sp.stunt_physics = true;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 16;
    sp.l5_spatial_suspension = strut;
    return sp;
}

std::unique_ptr<SimSession> session(const std::string& level, const VehicleParams& vp,
                                    bool strut = false) {
    SimConfig cfg;
    cfg.nominal_dt = 1e-3;
    return std::make_unique<SimSession>(create_dynamics_for_level(level),
                                        create_flat_ground(0.0, 1.0), vp,
                                        TireSetup(TireParams{}),
                                        level == "L5" ? l5_solver(strut) : SolverParams{}, cfg);
}

/// 300 ticks of a fixed manoeuvre; returns the final state bytes for a bit comparison.
std::string run_bytes(const std::string& level, const VehicleParams& vp, bool strut) {
    auto s = session(level, vp, strut);
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity.x() = 10.0;
    s->reset_settled(s0);
    for (int k = 0; k < 300; ++k) {
        CmdL4 u;
        u.throttle = 0.2;
        u.steer_angle_wheel = k < 100 ? 0.0 : 0.04;
        s->set_input(u);
        s->tick(1e-3);
    }
    const auto o = s->output();
    std::string b;
    auto put = [&](double x) { b.append(reinterpret_cast<const char*>(&x), sizeof x); };
    for (int i = 0; i < 3; ++i) put(o.state.position[i]);
    for (int i = 0; i < 4; ++i) put(o.state.orientation.coeffs()[i]);
    for (int i = 0; i < 3; ++i) put(o.state.velocity[i]);
    for (int i = 0; i < 3; ++i) put(o.state.angular_velocity[i]);
    for (const double x : o.state.wheel_spin) put(x);
    for (const double x : o.Fz) put(x);
    put(o.ax);
    put(o.ay);
    return b;
}

}  // namespace

TEST(CgHeightRefKeys, YamlRoundTripAndUnsetIsUntouched) {
    for (const char* v : {"vehicle", "sprung"}) {
        const auto p = write_tmp(std::string("cgref_") + v + ".yaml",
                                 sedan_yaml() + "\ncg_height_ref: " + v + "\n");
        const auto vp = VehicleParams::from_yaml(p);
        EXPECT_EQ(vp.cg_height_ref, v);
        const auto out = ::testing::TempDir() + "cgref_out.yaml";
        vp.to_yaml(out);
        EXPECT_EQ(VehicleParams::from_yaml(out).cg_height_ref, v);
    }
    const auto plain = ::testing::TempDir() + "cgref_plain_out.yaml";
    const auto vp = VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml");
    EXPECT_TRUE(vp.cg_height_ref.empty());
    vp.to_yaml(plain);
    EXPECT_EQ(read_all(plain).find("cg_height_ref"), std::string::npos);
}

TEST(CgHeightRefKeys, UnknownValueThrows) {
    const auto p = write_tmp("cgref_bad.yaml", sedan_yaml() + "\ncg_height_ref: body\n");
    EXPECT_THROW(VehicleParams::from_yaml(p), std::invalid_argument);
    VehicleParams vp;
    vp.cg_height_ref = "Sprung";
    EXPECT_THROW(validate_cg_height_ref(vp), std::invalid_argument);
    for (const char* lvl : {"L1", "L2", "L3", "L4"})
        EXPECT_THROW(session(lvl, vp), std::invalid_argument) << lvl;
    EXPECT_THROW(session("L5", vp, false), std::invalid_argument);
    EXPECT_THROW(session("L5", vp, true), std::invalid_argument);
}

TEST(CgHeightRefKeys, BodyBelowWheelThrowsOnStrutOnly) {
    VehicleParams vp;   // m 1500, sum m_u 160, R 0.32
    // h_s = (1500 h - 160 * 0.32) / 1340 <= 0.33  <=>  h <= 0.32107
    vp.cg_height = 0.321;
    EXPECT_THROW(l5_strut_body_height(vp), std::invalid_argument);
    EXPECT_THROW(session("L5", vp, true), std::invalid_argument);
    EXPECT_NO_THROW(session("L5", vp, false));    // penalty path ignores the meaning
    EXPECT_NO_THROW(session("L2", vp));
    vp.cg_height_ref = "sprung";                  // 0.8.1 meaning: body at cg_height
    EXPECT_NO_THROW(session("L5", vp, true));
    vp.cg_height_ref.clear();
    vp.cg_height = 0.55;
    EXPECT_NEAR(l5_strut_body_height(vp), (1500.0 * 0.55 - 160.0 * 0.32) / 1340.0, 1e-15);
    vp.cg_height_ref = "sprung";
    EXPECT_EQ(l5_strut_body_height(vp), 0.55);
}

TEST(CgHeightRefKeys, BelowStrutPathWarnsOnce) {
    VehicleParams vp;
    const int none = count_log("cg_height_ref=", [&] {
        session("L2", vp);
        session("L5", vp, false);
    });
    EXPECT_EQ(none, 0);
    vp.cg_height_ref = "sprung";
    const int strut = count_log("cg_height_ref=", [&] { session("L5", vp, true); });
    EXPECT_EQ(strut, 0);
    const int n = count_log("cg_height_ref=", [&] {
        for (const char* lvl : {"L1", "L2", "L3", "L4"}) session(lvl, vp);
        session("L5", vp, false);
        vp.cg_height_ref = "vehicle";
        session("L2", vp);
    });
    EXPECT_EQ(n, 1);
}

TEST(CgHeightRefKeys, IgnoredBelowStrutPathIsBitIdentical) {
    for (const char* lvl : {"L1", "L2", "L3", "L4", "L5"}) {
        VehicleParams vp;
        const auto ref = run_bytes(lvl, vp, false);
        for (const char* v : {"vehicle", "sprung"}) {
            vp.cg_height_ref = v;
            EXPECT_EQ(run_bytes(lvl, vp, false), ref) << lvl << " " << v;
        }
    }
}

TEST(CgHeightRefKeys, VehicleIsTheDefaultOnStrutPath) {
    VehicleParams vp;
    const auto def = run_bytes("L5", vp, true);
    vp.cg_height_ref = "vehicle";
    EXPECT_EQ(run_bytes("L5", vp, true), def);
    vp.cg_height_ref = "sprung";
    EXPECT_NE(run_bytes("L5", vp, true), def);
}
