// wheel_gyro key (work order 50, C++ side). Fixed before the implementation commit.
//   - YAML / to_yaml round trip of legacy | full | off; unset writes no key.
//   - Unknown values: std::invalid_argument from from_yaml, wheel_gyro_mode and every
//     level's session (L1-L4 and both L5 paths).
//   - L1-L4 read the key and ignore it: one warning per process, none on L5.
//   - Spin-axis / gyroscopic-moment helper (V2): full == legacy bit for bit at zero
//     camber, legacy == the 0.8.1 loop bit for bit, off == zero, camber -2 deg
//     full - legacy printed as an information value (not a gate).
#include "free_3d_wheel_gyro_internal.hpp"
#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;
using Arr = std::array<double, NUM_WHEELS>;
constexpr double kPi = 3.14159265358979323846;

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

std::unique_ptr<SimSession> session(const std::string& level, const VehicleParams& vp,
                                    bool strut = false) {
    SolverParams sp;
    if (level == "L5") {
        sp.stunt_physics = true;
        sp.max_substep_dt = 2e-4;
        sp.max_substeps = 16;
        sp.l5_spatial_suspension = strut;
    }
    SimConfig cfg;
    cfg.nominal_dt = 1e-3;
    return std::make_unique<SimSession>(create_dynamics_for_level(level),
                                        create_flat_ground(0.0, 1.0), vp,
                                        TireSetup(TireParams{}), sp, cfg);
}

/// The 0.8.1 loop (645780c free_3d_dynamics.cpp), copied verbatim as the legacy reference.
Vec3 gyro_081(const Vec3& omega, const Arr& I, const Arr& spin, const Arr& delta_in) {
    Vec3 tau = Vec3::Zero();
    Vec3 H_spin = Vec3::Zero();
    for (int i = 0; i < NUM_WHEELS; ++i) {
        const double delta = delta_in[i];
        Vec3 s_hat(-delta, 1.0, 0.0);
        s_hat.normalize();
        H_spin += (I[i] * spin[i]) * s_hat;
    }
    tau -= omega.cross(H_spin);
    return tau;
}

/// Same accumulation order as the dynamics: tau_body += wheel_gyro_moment(...).
Vec3 gyro_new(WheelGyro m, const Vec3& omega, const Arr& I, const Arr& spin,
              const Arr& delta, const Arr& gamma) {
    Vec3 tau = Vec3::Zero();
    tau += detail::wheel_gyro_moment(m, omega, I, spin, delta, gamma);
    return tau;
}

bool same_bits(const Vec3& a, const Vec3& b) {
    return std::memcmp(a.data(), b.data(), 3 * sizeof(double)) == 0;
}

/// Deterministic input sweep (no RNG dependency): 64 states with steer, spin, rates.
template <typename F>
void for_each_case(F&& f) {
    for (int k = 0; k < 64; ++k) {
        const double t = 0.37 * k;
        const Vec3 omega(0.3 * std::sin(t), 0.2 * std::cos(1.3 * t), 1.1 * std::sin(0.7 * t));
        const Arr I {{2.048, 2.048, 1.7 + 0.01 * k, 2.3}};
        const Arr spin {{20.0 + k, 21.5 + 0.5 * k, -3.0 + k, 40.0 - 0.25 * k}};
        const Arr delta {{0.4 * std::sin(t), 0.38 * std::sin(t + 0.1), 0.01 * std::cos(t), 0.0}};
        f(omega, I, spin, delta);
    }
}

}  // namespace

// ---- key ------------------------------------------------------------------------------

TEST(WheelGyroKeys, YamlRoundTripAndUnsetIsUntouched) {
    for (const char* v : {"legacy", "full", "off"}) {
        const auto p = write_tmp(std::string("gyro_") + v + ".yaml",
                                 sedan_yaml() + "\nwheel_gyro: " + v + "\n");
        const auto vp = VehicleParams::from_yaml(p);
        EXPECT_EQ(vp.wheel_gyro, v);
        const auto out = ::testing::TempDir() + "gyro_out.yaml";
        vp.to_yaml(out);
        EXPECT_EQ(VehicleParams::from_yaml(out).wheel_gyro, v);
    }
    const auto vp = VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml");
    EXPECT_TRUE(vp.wheel_gyro.empty());
    EXPECT_EQ(wheel_gyro_mode(vp), WheelGyro::Legacy);
    const auto plain = ::testing::TempDir() + "gyro_plain_out.yaml";
    vp.to_yaml(plain);
    EXPECT_EQ(read_all(plain).find("wheel_gyro"), std::string::npos);
}

TEST(WheelGyroKeys, ModeMapping) {
    VehicleParams vp;
    EXPECT_EQ(wheel_gyro_mode(vp), WheelGyro::Legacy);
    vp.wheel_gyro = "legacy";
    EXPECT_EQ(wheel_gyro_mode(vp), WheelGyro::Legacy);
    vp.wheel_gyro = "full";
    EXPECT_EQ(wheel_gyro_mode(vp), WheelGyro::Full);
    vp.wheel_gyro = "off";
    EXPECT_EQ(wheel_gyro_mode(vp), WheelGyro::Off);
}

// Runs before UnknownValueThrows in file order; nothing else in this binary sets the key.
TEST(WheelGyroKeys, IgnoredBelowL5WarnsOncePerProcess) {
    VehicleParams vp;
    vp.wheel_gyro = "full";
    const int n_l5 = count_log("wheel_gyro=", [&] {
        session("L5", vp, false);
        session("L5", vp, true);
    });
    EXPECT_EQ(n_l5, 0);
    const int n = count_log("wheel_gyro=", [&] {
        for (int rep = 0; rep < 2; ++rep)
            for (const char* lvl : {"L1", "L2", "L3", "L4"}) session(lvl, vp);
    });
    EXPECT_EQ(n, 1);
}

TEST(WheelGyroKeys, UnknownValueThrows) {
    for (const char* bad : {"Full", "on", "none"}) {
        const auto p = write_tmp("gyro_bad.yaml", sedan_yaml() + "\nwheel_gyro: " + bad + "\n");
        EXPECT_THROW(VehicleParams::from_yaml(p), std::invalid_argument) << bad;
        VehicleParams vp;
        vp.wheel_gyro = bad;
        EXPECT_THROW(wheel_gyro_mode(vp), std::invalid_argument) << bad;
        for (const char* lvl : {"L1", "L2", "L3", "L4"})
            EXPECT_THROW(session(lvl, vp), std::invalid_argument) << lvl << ' ' << bad;
        EXPECT_THROW(session("L5", vp, false), std::invalid_argument) << bad;
        EXPECT_THROW(session("L5", vp, true), std::invalid_argument) << bad;
    }
}

// ---- helper (V2) ------------------------------------------------------------------------

TEST(WheelGyroHelper, LegacyMatches081LoopBitForBit) {
    const Arr zero {};
    int n = 0;
    for_each_case([&](const Vec3& w, const Arr& I, const Arr& spin, const Arr& d) {
        EXPECT_TRUE(same_bits(gyro_new(WheelGyro::Legacy, w, I, spin, d, zero),
                              gyro_081(w, I, spin, d)));
        // legacy ignores camber entirely
        const Arr g {{-0.035, 0.035, -0.02, 0.02}};
        EXPECT_TRUE(same_bits(gyro_new(WheelGyro::Legacy, w, I, spin, d, g),
                              gyro_081(w, I, spin, d)));
        ++n;
    });
    EXPECT_EQ(n, 64);
}

TEST(WheelGyroHelper, V2_FullEqualsLegacyAtZeroCamber) {
    const Arr zero {};
    for_each_case([&](const Vec3& w, const Arr& I, const Arr& spin, const Arr& d) {
        EXPECT_TRUE(same_bits(gyro_new(WheelGyro::Full, w, I, spin, d, zero),
                              gyro_new(WheelGyro::Legacy, w, I, spin, d, zero)));
    });
}

TEST(WheelGyroHelper, OffIsZero) {
    const Arr g {{-0.035, 0.035, -0.02, 0.02}};
    for_each_case([&](const Vec3& w, const Arr& I, const Arr& spin, const Arr& d) {
        const Vec3 m = detail::wheel_gyro_moment(WheelGyro::Off, w, I, spin, d, g);
        EXPECT_EQ(m.x(), 0.0);
        EXPECT_EQ(m.y(), 0.0);
        EXPECT_EQ(m.z(), 0.0);
    });
}

// Camber convention (vehicle frame, as gamma_dae_): + = wheel top toward body +y.
// The spin axis (oriented body +y) is then (0, cos g, -sin g) at zero steer.
TEST(WheelGyroHelper, FullSpinAxisCamberGeometry) {
    for (const double g_deg : {-2.0, 2.0, -5.0}) {
        const double g = g_deg * kPi / 180.0;
        const Vec3 s = detail::wheel_spin_axis(WheelGyro::Full, 0.0, g);
        EXPECT_NEAR(s.x(), 0.0, 1e-15);
        EXPECT_NEAR(s.y(), std::cos(g), 1e-15);
        EXPECT_NEAR(s.z(), -std::sin(g), 1e-15);
        EXPECT_NEAR(s.norm(), 1.0, 1e-15);
    }
}

// V2 information value: camber -2 deg (top-in on both sides: left -2 deg, right +2 deg in
// the vehicle frame) at the V3 skidpad operating point. Printed, not gated. The structural
// check only asserts that full reacts to camber at all (the negative check removes it).
TEST(WheelGyroHelper, V2_CamberMinus2DegInformation) {
    const double R_w = 0.32, V = 7.0, R_turn = 8.5, L = 2.7;
    const double r = V / R_turn;
    const double I = 0.5 * 40.0 * R_w * R_w;
    const Arr Iw {{I, I, I, I}};
    const Arr spin {{V / R_w, V / R_w, V / R_w, V / R_w}};
    const Arr delta {{L / R_turn, L / R_turn, 0.0, 0.0}};
    const double g = 2.0 * kPi / 180.0;
    const Arr gamma {{-g, g, -g, g}};
    const Arr zero {};
    for (const double p_rate : {0.0, 0.2}) {   // steady yaw only / yaw + 0.2 rad/s roll rate
        const Vec3 w(p_rate, 0.0, r);
        const Vec3 leg = gyro_new(WheelGyro::Legacy, w, Iw, spin, delta, zero);
        const Vec3 full = gyro_new(WheelGyro::Full, w, Iw, spin, delta, gamma);
        const Vec3 d = full - leg;
        std::printf("V2-info camber=-2deg p=%.2f legacy=(%.9f, %.9f, %.9f) N m "
                    "full-legacy=(%.9e, %.9e, %.9e) N m roll_rel=%.6e\n",
                    p_rate, leg.x(), leg.y(), leg.z(), d.x(), d.y(), d.z(),
                    d.x() / leg.x());
        EXPECT_FALSE(same_bits(full, leg)) << "full must react to non-zero camber";
    }
}
