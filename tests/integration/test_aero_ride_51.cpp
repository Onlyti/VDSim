// Aero ride-height / pitch slopes and the aero user module (work order 51, 0.9.0).
//
// Gates and expectations are fixed in tests/fixtures/aero_ride_51/DEFINITION.md,
// handcalc.py and b2_expected.json (committed before the implementation).
#include "vdsim/aero.hpp"
#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"
#include "aero_settle_test_access.hpp"

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;
const std::string kFix = kSrc + "/tests/fixtures/aero_ride_51";

struct Slope {
    double h_ref, dh, dp;
};

VehicleParams fs_car() {
    return VehicleParams::from_yaml(kSrc + "/configs/vehicles/fs_aero_example.yaml");
}

TireParams fs_tire() {
    return TireParams::from_yaml(kSrc + "/configs/parts/tire/generic_pacejka.yaml");
}

/// Copy of vp with the slope keys of the named elements set.
VehicleParams with_slopes(VehicleParams vp, const std::map<std::string, Slope>& sl) {
    for (auto& e : vp.aero_elements) {
        const auto it = sl.find(e.name);
        if (it == sl.end()) continue;
        e.h_ref = it->second.h_ref;
        e.dClA_dh = it->second.dh;
        e.dClA_dpitch = it->second.dp;
    }
    return vp;
}

std::unique_ptr<IVehicleDynamics> make_dyn(const std::string& level) {
    if (level == "L1") return create_bicycle();
    if (level == "L2") return create_seven_dof();
    if (level == "L3") return create_fourteen_dof();
    if (level == "L4") return create_fourteen_dof_kinematic();
    return create_stunt_dof();
}

SolverParams solver(const std::string& level, bool strut = false) {
    SolverParams sp;
    if (level == "L5") {
        sp.stunt_physics = true;
        sp.max_substep_dt = 2e-4;
        sp.max_substeps = 16;
        sp.l5_spatial_suspension = strut;
    }
    return sp;
}

std::unique_ptr<SimSession> session(const std::string& level, const VehicleParams& vp,
                                    bool strut = false) {
    SimConfig cfg;
    cfg.nominal_dt = 1e-3;
    return std::make_unique<SimSession>(make_dyn(level), create_flat_ground(0.0, 1.0), vp,
                                        TireSetup(fs_tire()), solver(level, strut), cfg);
}

/// Count occurrences of `needle` in everything the default logger writes while fn runs.
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

/// |a - b| <= tol * |b|, or <= 1e-12 when b == 0 (DEFINITION 2.3).
bool rel_close(double a, double b, double tol) {
    if (b == 0.0) return std::abs(a) <= 1e-12;
    return std::abs(a - b) <= tol * std::abs(b);
}

}  // namespace

// ---------------------------------------------------------------- B2

TEST(AeroRide51, B2_GridMatchesHandcalc) {
    const YAML::Node j = YAML::LoadFile(kFix + "/b2_expected.json");
    std::map<std::string, Slope> sl;
    for (const auto& kv : j["slopes"]) {
        const auto v = kv.second;
        sl[kv.first.as<std::string>()] = {v["h_ref"].as<double>(), v["dClA_dh"].as<double>(),
                                          v["dClA_dpitch"].as<double>()};
    }
    const VehicleParams vp = with_slopes(fs_car(), sl);
    validate_aero_params(vp);
    ASSERT_DOUBLE_EQ(vp.air_density, j["rho"].as<double>());
    const double tol = j["rel_tol"].as<double>();
    int n = 0;
    for (const auto& c : j["cases"]) {
        AeroState s;
        s.vx = j["vx"].as<double>();
        s.dz = c["dz"].as<double>();
        s.pitch = c["theta"].as<double>();
        const AeroLoads L = compute_aero_loads(vp, s);
        ASSERT_EQ(L.elements.size(), vp.aero_elements.size());
        for (std::size_t i = 0; i < vp.aero_elements.size(); ++i) {
            const std::string& name = vp.aero_elements[i].name;
            const double fz_exp = c["fz"][name].as<double>();
            const double h_exp = c["h"][name].as<double>();
            EXPECT_TRUE(rel_close(L.elements[i].fz, fz_exp, tol))
                << name << " dz=" << s.dz << " th=" << s.pitch << " fz=" << L.elements[i].fz
                << " exp=" << fz_exp;
            EXPECT_TRUE(rel_close(L.elements[i].h, h_exp, tol))
                << name << " dz=" << s.dz << " th=" << s.pitch << " h=" << L.elements[i].h
                << " exp=" << h_exp;
            n += 2;
        }
    }
    EXPECT_EQ(n, 9 * 4 * 2);
}

// ---------------------------------------------------------------- B6

TEST(AeroRide51, B6_PartialKeysRejected) {
    for (int mask = 1; mask < 7; ++mask) {   // one or two of the three keys
        VehicleParams vp = fs_car();
        auto& e = vp.aero_elements[0];
        if (mask & 1) e.h_ref = 0.1;
        if (mask & 2) e.dClA_dh = -1.0;
        if (mask & 4) e.dClA_dpitch = 2.0;
        EXPECT_THROW(validate_aero_params(vp), std::invalid_argument) << "mask " << mask;
    }
    const std::string yaml_path = kSrc + "/configs/vehicles/fs_aero_example.yaml";
    YAML::Node root = YAML::LoadFile(yaml_path);
    root["aero"]["elements"][0]["h_ref"] = 0.1;
    root["aero"]["elements"][0]["dClA_dh"] = -1.0;
    const std::string tmp = ::testing::TempDir() + "aero51_partial.yaml";
    {
        std::ofstream f(tmp);
        f << YAML::Dump(root);
    }
    EXPECT_THROW(VehicleParams::from_yaml(tmp), std::invalid_argument);
}

TEST(AeroRide51, B6_NonFiniteRejected) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (int k = 0; k < 3; ++k) {
        for (double bad : {nan, inf}) {
            VehicleParams vp = with_slopes(fs_car(), {{"undertray", {0.04, -2.0, 0.0}}});
            auto& e = vp.aero_elements[2];
            if (k == 0) e.h_ref = bad;
            if (k == 1) e.dClA_dh = bad;
            if (k == 2) e.dClA_dpitch = bad;
            EXPECT_THROW(validate_aero_params(vp), std::invalid_argument) << k;
        }
    }
}

TEST(AeroRide51, B6_L1L2SlopeRejectedZeroAccepted) {
    const VehicleParams active = with_slopes(fs_car(), {{"front_wing", {0.1, 0.0, 30.0}}});
    const VehicleParams zero = with_slopes(fs_car(), {{"front_wing", {0.1, 0.0, 0.0}}});
    for (const std::string lvl : {"L1", "L2"}) {
        EXPECT_THROW(session(lvl, active), std::invalid_argument) << lvl;
        EXPECT_NO_THROW(session(lvl, zero)) << lvl;
    }
    for (const std::string lvl : {"L3", "L4", "L5"}) EXPECT_NO_THROW(session(lvl, active)) << lvl;
}

TEST(AeroRide51, B6_ToYamlWritesKeysOnlyWhereSet) {
    const VehicleParams vp = with_slopes(fs_car(), {{"undertray", {0.04, -2.0, 0.5}}});
    const std::string tmp = ::testing::TempDir() + "aero51_roundtrip.yaml";
    vp.to_yaml(tmp);
    const auto slurp = [](const std::string& path) {
        std::ifstream f(path);
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
    };
    const std::string y = slurp(tmp);
    std::size_t n = 0;
    for (std::size_t p = y.find("h_ref"); p != std::string::npos; p = y.find("h_ref", p + 1)) ++n;
    EXPECT_EQ(n, 1u);
    const VehicleParams back = VehicleParams::from_yaml(tmp);
    ASSERT_EQ(back.aero_elements.size(), vp.aero_elements.size());
    for (std::size_t i = 0; i < vp.aero_elements.size(); ++i) {
        const auto& a = vp.aero_elements[i];
        const auto& b = back.aero_elements[i];
        EXPECT_EQ(a.h_ref.has_value(), b.h_ref.has_value());
        EXPECT_EQ(a.dClA_dh.has_value(), b.dClA_dh.has_value());
        EXPECT_EQ(a.dClA_dpitch.has_value(), b.dClA_dpitch.has_value());
        if (a.has_ride_slopes()) {
            EXPECT_EQ(*a.h_ref, *b.h_ref);
            EXPECT_EQ(*a.dClA_dh, *b.dClA_dh);
            EXPECT_EQ(*a.dClA_dpitch, *b.dClA_dpitch);
        }
    }
    const std::string tmp_plain = ::testing::TempDir() + "aero51_plain.yaml";
    fs_car().to_yaml(tmp_plain);
    EXPECT_EQ(slurp(tmp_plain).find("h_ref"), std::string::npos);
}

TEST(AeroRide51, B6_NegativeHeightWarnsOncePerSession) {
    // h_ref below the ground puts the element at h < 0 from the first tick.
    const VehicleParams vp = with_slopes(fs_car(), {{"front_wing", {-0.05, 0.0, 1.0}}});
    for (const std::string lvl : {"L3", "L5"}) {
        const int n = count_log("element 'front_wing' is below the ground", [&] {
            for (int r = 0; r < 2; ++r) {
                auto s = session(lvl, vp);
                State s0;
                s0.position.z() = vp.cg_height;
                s0.velocity.x() = 10.0;
                const double w = 10.0 / vp.wheel_radius_nominal;
                s0.wheel_spin = {{w, w, w, w}};
                s->reset_settled(s0);
                CmdL6 cmd;
                cmd.v_target = 10.0;
                for (int k = 0; k < 200; ++k) {
                    s->set_input(cmd);
                    s->tick(1e-3);
                }
            }
        });
        EXPECT_EQ(n, 2) << lvl;   // one per SimSession
    }
}

// ---------------------------------------------------------------- B1 / B1a helpers

namespace {

constexpr double kV60 = 60.0 / 3.6;

/// Every SimOutput field the dynamics write, as doubles (bit comparison).
std::vector<double> dump(const SimOutput& o) {
    std::vector<double> v;
    const auto st = [&v](const State& s) {
        for (int k = 0; k < 3; ++k) v.push_back(s.position[k]);
        for (int k = 0; k < 4; ++k) v.push_back(s.orientation.coeffs()[k]);
        for (int k = 0; k < 3; ++k) v.push_back(s.velocity[k]);
        for (int k = 0; k < 3; ++k) v.push_back(s.angular_velocity[k]);
        for (int i = 0; i < NUM_WHEELS; ++i) {
            v.push_back(s.wheel_spin[i]);
            v.push_back(s.susp_compression[i]);
            v.push_back(s.susp_velocity[i]);
            for (int k = 0; k < 3; ++k) v.push_back(s.unsprung_pos[i][k]);
            for (int k = 0; k < 3; ++k) v.push_back(s.unsprung_vel[i][k]);
        }
        v.push_back(s.rack_travel);
        v.push_back(s.rack_velocity);
    };
    st(o.state);
    st(o.measured);
    for (double x : {o.sim_time, o.ax, o.ay, o.az, o.roll, o.pitch, o.heave_z, o.rack_torque,
                     o.steer_applied, o.throttle_applied, o.brake_applied})
        v.push_back(x);
    for (int i = 0; i < NUM_WHEELS; ++i) {
        v.push_back(o.Fz[i]);
        for (int k = 0; k < 3; ++k) v.push_back(o.tire_forces[i][k]);
        for (int k = 0; k < 3; ++k) v.push_back(o.tire_forces_wheel[i][k]);
        v.push_back(o.slip_ratio[i]);
        v.push_back(o.slip_angle[i]);
        v.push_back(o.wheel_mu[i]);
        v.push_back(o.wheel_camber[i]);
        v.push_back(o.wheel_toe[i]);
    }
    return v;
}

bool bits_equal(const std::vector<double>& a, const std::vector<double>& b) {
    return a.size() == b.size() &&
           std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

State spawn60(const VehicleParams& vp) {
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity.x() = kV60;
    const double w = kV60 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    return s0;
}

/// Outputs right after reset_settled(60 km/h) and of `ticks` ticks CmdL6 60 km/h.
std::vector<std::vector<double>> run60(SimSession& s, const VehicleParams& vp, int ticks) {
    std::vector<std::vector<double>> out;
    s.reset_settled(spawn60(vp));
    out.push_back(dump(s.output()));
    CmdL6 cmd;
    cmd.v_target = kV60;
    for (int k = 0; k < ticks; ++k) {
        s.set_input(cmd);
        s.tick(1e-3);
        out.push_back(dump(s.output()));
    }
    return out;
}

}  // namespace

// ---------------------------------------------------------------- B1 (slope keys at 0)

TEST(AeroRide51, B1_ZeroSlopeKeysBitIdentical) {
    const VehicleParams plain = fs_car();
    VehicleParams keyed = plain;
    for (auto& e : keyed.aero_elements) {
        e.h_ref = e.z;
        e.dClA_dh = 0.0;
        e.dClA_dpitch = 0.0;
    }
    struct Case { std::string lvl; bool strut; };
    for (const Case& c : {Case{"L1", false}, Case{"L2", false}, Case{"L3", false},
                          Case{"L4", false}, Case{"L5", false}, Case{"L5", true}}) {
        auto a = session(c.lvl, plain, c.strut);
        auto b = session(c.lvl, keyed, c.strut);
        const auto ra = run60(*a, plain, 2000);
        const auto rb = run60(*b, keyed, 2000);
        ASSERT_EQ(ra.size(), rb.size());
        bool same = true;
        for (std::size_t k = 0; k < ra.size(); ++k) same = same && bits_equal(ra[k], rb[k]);
        EXPECT_TRUE(same) << c.lvl << (c.strut ? " strut" : "");
        EXPECT_FALSE(b->aero_reference().ran) << c.lvl << " settle run count must be 0";
    }
}

// ---------------------------------------------------------------- B1a

TEST(AeroRide51, B1a_SettleRunLeavesNothingButTheReference) {
    const VehicleParams vp = fs_car();
    for (const bool strut : {false, true}) {
        auto off = session("L5", vp, strut);
        auto on = session("L5", vp, strut);
        detail::AeroSettleTestAccess::force(*on, true);
        const auto r_off = run60(*off, vp, 2000);
        const auto r_on = run60(*on, vp, 2000);
        ASSERT_TRUE(on->aero_reference().ran);
        EXPECT_FALSE(off->aero_reference().ran);
        ASSERT_EQ(r_off.size(), r_on.size());
        std::size_t first_diff = r_off.size();
        for (std::size_t k = 0; k < r_off.size() && first_diff == r_off.size(); ++k)
            if (!bits_equal(r_off[k], r_on[k])) first_diff = k;
        EXPECT_EQ(first_diff, r_off.size()) << (strut ? "strut" : "penalty")
                                            << " first differing output " << first_diff;
        EXPECT_EQ(off->sim_time(), on->sim_time());
        const AeroReference ref = on->aero_reference();
        std::printf("B1a %s: settled=%d t_settle=%.3f ref_z=%.6f ref_pitch=%.3e\n",
                    strut ? "strut" : "penalty", int(ref.settled), ref.t_settle, ref.ref_z,
                    ref.ref_pitch);
        EXPECT_TRUE(ref.settled) << ref.unmet;
    }
}

TEST(AeroRide51, B1a_SettleLimitWarnsOnceWithChannels) {
    const VehicleParams vp = fs_car();
    auto s = session("L5", vp);
    detail::AeroSettleTestAccess::force(*s, true);
    detail::AeroSettleTestAccess::limit(*s, 0.01);
    std::ostringstream os;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(os);
    spdlog::default_logger()->sinks().push_back(sink);
    s->reset_settled(spawn60(vp));
    spdlog::default_logger()->flush();
    spdlog::default_logger()->sinks().pop_back();
    const std::string text = os.str();
    int n = 0;
    const std::string needle = "settle run reached its";
    for (std::size_t p = text.find(needle); p != std::string::npos; p = text.find(needle, p + 1))
        ++n;
    EXPECT_EQ(n, 1);
    const AeroReference ref = s->aero_reference();
    EXPECT_TRUE(ref.ran);
    EXPECT_FALSE(ref.settled);
    EXPECT_FALSE(ref.unmet.empty());
    EXPECT_NE(text.find(ref.unmet), std::string::npos) << text;
}

// Information item: wall-clock cost of the L5 settle run per reset (median of 3).
TEST(AeroRide51, Info_SettleRunWallClock) {
    const VehicleParams vp = with_slopes(fs_car(), {{"undertray", {0.04, -2.0, 0.0}}});
    for (const bool strut : {false, true}) {
        std::vector<double> ms;
        bool settled = true;
        for (int r = 0; r < 3; ++r) {
            auto s = session("L5", vp, strut);
            const auto t0 = std::chrono::steady_clock::now();
            s->reset_settled(spawn60(vp));
            const auto t1 = std::chrono::steady_clock::now();
            ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            ASSERT_TRUE(s->aero_reference().ran);
            settled = settled && s->aero_reference().settled;
        }
        std::sort(ms.begin(), ms.end());
        std::printf("INFO settle_wall_ms %s median=%.1f settled=%d\n", strut ? "strut" : "penalty",
                    ms[1], int(settled));
    }
}
