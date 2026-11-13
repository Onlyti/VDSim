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

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
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
    const VehicleParams vp = with_slopes(fs_car(), {{"front_wing", {-0.001, 0.0, 1.0}}});
    for (const std::string lvl : {"L3"}) {
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
