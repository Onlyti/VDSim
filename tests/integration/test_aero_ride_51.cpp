// Aero ride-height / pitch slopes and the aero user module (work order 51, 0.9.0).
//
// Gates and expectations are fixed in tests/fixtures/aero_ride_51/DEFINITION.md,
// handcalc.py and b2_expected.json (committed before the implementation).
#include "vdsim/aero.hpp"
#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/snapshot.hpp"
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
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>
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

// ---------------------------------------------------------------- B3' / B3b helpers

namespace {

constexpr double kGuardN = 10.0;   // |expected| below this: not judgeable (PO D2)
constexpr double kTolB3 = 0.02;

/// Front / rear rigid-axle aero load of vp's elements at q (drag moment included,
/// handcalc.rigid_split), with the ride change (dz, theta).
std::pair<double, double> rigid_split(const VehicleParams& vp, double q, double dz = 0.0,
                                      double theta = 0.0) {
    const double L = vp.wheelbase;
    const double x_o = -vp.cg_to_front;
    double ff = 0.0, tot = 0.0, md = 0.0;
    for (const auto& e : vp.aero_elements) {
        const double dh = dz - (e.x - x_o) * theta;
        double cla = e.ClA;
        if (e.has_ride_slopes()) cla += *e.dClA_dh * dh + *e.dClA_dpitch * theta;
        const double f = q * cla;
        ff += f * (e.x + L) / L;
        tot += f;
        md += q * e.CdA * e.z;
    }
    ff -= md / L;
    return {ff, tot - ff};
}

double mean(const std::vector<double>& v) {
    double s = 0.0;
    for (double x : v) s += x;
    return v.empty() ? 0.0 : s / static_cast<double>(v.size());
}

/// Least-squares slope of y(t).
double ls_slope(const std::vector<double>& t, const std::vector<double>& y) {
    const double mt = mean(t), my = mean(y);
    double num = 0.0, den = 0.0;
    for (std::size_t i = 0; i < t.size(); ++i) {
        num += (t[i] - mt) * (y[i] - my);
        den += (t[i] - mt) * (t[i] - mt);
    }
    return den > 0.0 ? num / den : 0.0;
}

/// Window record of one run.
struct Win {
    std::vector<double> t, q, dz, theta, ax, fzf, fzr, sumfz, vh;
};

/// Body-origin heave and pitch relative to a static reference (L5).
void ride_l5(const SimOutput& o, const AeroReference& ref, double& dz, double& th) {
    dz = o.state.position.z() - ref.ref_z;
    th = o.pitch - ref.ref_pitch;   // L5 pitch_angle_qs = euler pitch of the body
}

std::string out_dir() {
    const char* d = std::getenv("VDSIM_AERO51_OUT");
    return d ? std::string(d) + "/" : ::testing::TempDir();
}

/// Run L5 at constant speed `v` (CmdL6) for `t_end` s; record [t0, t_end).
Win run_cruise_l5(SimSession& s, const VehicleParams& vp, double v, double t0, double t_end,
                  const AeroReference& ref) {
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity.x() = v;
    const double w = v / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    s.reset_settled(s0);
    CmdL6 cmd;
    cmd.v_target = v;
    Win r;
    const int n = static_cast<int>(std::lround(t_end / 1e-3));
    for (int k = 0; k < n; ++k) {
        s.set_input(cmd);
        s.tick(1e-3);
        const SimOutput o = s.output();
        if (o.sim_time < t0 - 1e-9) continue;
        double dz = 0.0, th = 0.0;
        ride_l5(o, ref, dz, th);
        const double vx = o.state.velocity.x();
        r.t.push_back(o.sim_time);
        r.q.push_back(0.5 * vp.air_density * vx * vx);
        r.dz.push_back(dz);
        r.theta.push_back(th);
        r.ax.push_back(o.ax);
        r.fzf.push_back(o.Fz[WHEEL_FL] + o.Fz[WHEEL_FR]);
        r.fzr.push_back(o.Fz[WHEEL_RL] + o.Fz[WHEEL_RR]);
        r.sumfz.push_back(o.Fz[0] + o.Fz[1] + o.Fz[2] + o.Fz[3]);
        // A3 validity: CG velocity in world axes, horizontal component along the heading.
        const Mat3 Rm = o.state.orientation.toRotationMatrix();
        const Vec3 vw = Rm * o.state.velocity;
        const double yaw = o.state.yaw();
        r.vh.push_back(vw.x() * std::cos(yaw) + vw.y() * std::sin(yaw));
    }
    return r;
}

/// Car of DEFINITION 2.4 (undertray keys, slope dClA_dh).
VehicleParams b3l5_car(double dclh) {
    return with_slopes(fs_car(), {{"undertray", {0.040, dclh, 0.0}}});
}

}  // namespace

// ---------------------------------------------------------------- B3'-L5

TEST(AeroRide51, B3p_L5_UndertraySelfSagFixedPoint) {
    for (const bool strut : {false, true}) {
        // Undertray slope per path (handcalc.B3L5_DCLH): penalty re-fixed to -4.0 before
        // the re-measurement (PO mail 91b88183, -2.0 was below the 10 N guard); strut -2.0.
        const double dclh = strut ? -2.0 : -4.0;
        const VehicleParams on_vp = b3l5_car(dclh);
        const VehicleParams off_vp = b3l5_car(0.0);
        auto on = session("L5", on_vp, strut);
        auto off = session("L5", off_vp, strut);
        // Spawn states of the two runs must be bit identical (static reference shared).
        State s0;
        s0.position.z() = on_vp.cg_height;
        s0.velocity.x() = 35.0;
        const double w = 35.0 / on_vp.wheel_radius_nominal;
        s0.wheel_spin = {{w, w, w, w}};
        on->reset_settled(s0);
        off->reset_settled(s0);
        ASSERT_TRUE(bits_equal(dump(on->output()), dump(off->output())));
        const AeroReference ref = on->aero_reference();
        ASSERT_TRUE(ref.ran);
        EXPECT_FALSE(off->aero_reference().ran);
        const Win a = run_cruise_l5(*on, on_vp, 35.0, 10.0, 12.0, ref);
        const Win b = run_cruise_l5(*off, off_vp, 35.0, 10.0, 12.0, ref);
        const double ag_on = ls_slope(a.t, a.vh), ag_off = ls_slope(b.t, b.vh);
        EXPECT_LE(std::abs(ag_on), 0.02) << "validity (stop and report)";
        EXPECT_LE(std::abs(ag_off), 0.02) << "validity (stop and report)";
        // Expectation (handcalc.b3l5).
        const double L = on_vp.wheelbase, a_cg = on_vp.cg_to_front, b_cg = on_vp.cg_to_rear;
        const double q_on = mean(a.q), q_off = mean(b.q);
        const double dz_off = mean(b.dz), th_off = mean(b.theta);
        const double dzf_off = dz_off - a_cg * th_off;
        const double dzr_off = dz_off + b_cg * th_off;
        const auto split = rigid_split(off_vp, q_off);
        const double kf = split.first / (-dzf_off), kr = split.second / (-dzr_off);
        double xu = 0.0, cla_sum = 0.0;
        for (const auto& e : off_vp.aero_elements) {
            if (e.name == "undertray") xu = e.x;
            cla_sum += e.ClA;
        }
        const double su = (xu + L) / L;
        const double D = q_on * dclh;
        const double dh0 = su * dzf_off + (1.0 - su) * dzr_off;
        const double dh = dh0 / (1.0 + D * (su * su / kf + (1.0 - su) * (1.0 - su) / kr));
        const double exp_dF = cla_sum * (q_on - q_off) + D * dh;
        const double meas = mean(a.sumfz) - mean(b.sumfz);
        const double dh_u_on = mean(a.dz) - (xu + a_cg) * mean(a.theta);
        const double info = cla_sum * (q_on - q_off) + D * dh_u_on;
        const std::string tag = strut ? "strut" : "penalty";
        std::printf("B3p_L5 %s: dF_meas=%.6f dF_exp=%.6f rel=%+.5f K_f=%.1f K_r=%.1f "
                    "dh_fixed=%.6e dh_meas=%.6e info_dF_meas_dh=%.6f a_g=%.2e/%.2e\n",
                    tag.c_str(), meas, exp_dF, meas / exp_dF - 1.0, kf, kr, dh, dh_u_on, info,
                    ag_on, ag_off);
        {
            std::ofstream f(out_dir() + "b3l5_" + tag + ".json");
            f.precision(17);
            f << "{\"q_on\": " << q_on << ", \"q_off\": " << q_off << ", \"dzf_off\": " << dzf_off
              << ", \"dzr_off\": " << dzr_off << ", \"sumfz_on\": " << mean(a.sumfz)
              << ", \"sumfz_off\": " << mean(b.sumfz) << ", \"dh_u_on\": " << dh_u_on
              << ", \"dClA_dh\": " << dclh << "}\n";
        }
        EXPECT_GE(std::abs(exp_dF), kGuardN) << tag << " not judgeable (stop and report)";
        if (std::abs(exp_dF) >= kGuardN) EXPECT_LE(std::abs(meas / exp_dF - 1.0), kTolB3) << tag;
    }
}

// ---------------------------------------------------------------- B3'-L3 (L3, L4)

TEST(AeroRide51, B3p_L3_InjectedRideFirstTick) {
    for (const std::string lvl : {"L3", "L4"}) {
        const VehicleParams on_vp = b3l5_car(-2.0);
        const VehicleParams off_vp = b3l5_car(0.0);
        double sum[2] = {0.0, 0.0};
        int k = 0;
        for (const VehicleParams* vp : {&on_vp, &off_vp}) {
            auto s = session(lvl, *vp);
            State s0;
            s0.position.z() = vp->cg_height;
            s0.velocity.x() = 35.0;
            const double w = 35.0 / vp->wheel_radius_nominal;
            s0.wheel_spin = {{w, w, w, w}};
            s->reset_settled(s0);
            // Inject z_s and theta through the model's own aux restore (L3 layout:
            // z_s, z_s_dot, phi, phi_dot, th, th_dot, ...).
            std::vector<double> aux;
            s->dynamics().save_aux(aux);
            aux[0] = -0.010;
            aux[4] = +0.010;
            std::size_t p = 0;
            s->dynamics().restore_aux(aux, p);
            s->set_input(CmdL4{});
            s->tick(1e-3);
            const auto Fz = s->output().Fz;
            sum[k++] = Fz[0] + Fz[1] + Fz[2] + Fz[3];
        }
        const double exp_dF = 713.5625 * (-2.0) * (-0.0107);   // handcalc.b3l3
        const double meas = sum[0] - sum[1];
        std::printf("B3p_L3 %s: dF_meas=%.9f dF_exp=%.9f rel=%+.3e\n", lvl.c_str(), meas, exp_dF,
                    meas / exp_dF - 1.0);
        ASSERT_GE(std::abs(exp_dF), kGuardN);
        EXPECT_LE(std::abs(meas / exp_dF - 1.0), kTolB3) << lvl;
    }
}

// ---------------------------------------------------------------- B3b

TEST(AeroRide51, B3b_FrontWingPitchSlopeBraking) {
    struct Case { std::string lvl; bool strut; };
    for (const Case& c : {Case{"L3", false}, Case{"L5", false}, Case{"L5", true}}) {
        const VehicleParams on_vp = with_slopes(fs_car(), {{"front_wing", {0.100, 0.0, 30.0}}});
        const VehicleParams off_vp = fs_car();
        Win w[2];
        AeroReference ref_on;
        for (int r = 0; r < 2; ++r) {
            const VehicleParams& vp = r == 0 ? on_vp : off_vp;
            auto s = session(c.lvl, vp, c.strut);
            State s0;
            s0.position.z() = vp.cg_height;
            s0.velocity.x() = 32.0;
            const double wsp = 32.0 / vp.wheel_radius_nominal;
            s0.wheel_spin = {{wsp, wsp, wsp, wsp}};
            s->reset_settled(s0);
            if (r == 0) ref_on = s->aero_reference();
            CmdL5 cmd;
            cmd.ax_target = -4.0;
            for (int k = 0; k < 3000; ++k) {
                s->set_input(cmd);
                s->tick(1e-3);
                const SimOutput o = s->output();
                if (o.sim_time < 1.0 - 1e-9) continue;
                double th = o.pitch;   // L3: pitch_angle_qs = ride state theta
                if (c.lvl == "L5") {
                    double dz = 0.0;
                    ride_l5(o, ref_on, dz, th);
                }
                const double vx = o.state.velocity.x();
                w[r].t.push_back(o.sim_time);
                w[r].q.push_back(0.5 * vp.air_density * vx * vx);
                w[r].theta.push_back(th);
                w[r].ax.push_back(o.ax);
                w[r].fzf.push_back(o.Fz[WHEEL_FL] + o.Fz[WHEEL_FR]);
                w[r].fzr.push_back(o.Fz[WHEEL_RL] + o.Fz[WHEEL_RR]);
            }
        }
        if (c.lvl == "L5") ASSERT_TRUE(ref_on.ran);
        // Expectation (handcalc.b3b).
        std::vector<double> fon_f, fon_r, foff_f, foff_r;
        for (std::size_t i = 0; i < w[0].q.size(); ++i) {
            const auto f = rigid_split(on_vp, w[0].q[i], 0.0, w[0].theta[i]);
            fon_f.push_back(f.first);
            fon_r.push_back(f.second);
        }
        for (std::size_t i = 0; i < w[1].q.size(); ++i) {
            const auto f = rigid_split(off_vp, w[1].q[i]);
            foff_f.push_back(f.first);
            foff_r.push_back(f.second);
        }
        const double dax = mean(w[0].ax) - mean(w[1].ax);
        const double inert = on_vp.mass * dax * on_vp.cg_height / on_vp.wheelbase;
        const double ef = mean(fon_f) - mean(foff_f) - inert;
        const double er = mean(fon_r) - mean(foff_r) + inert;
        const double mf = mean(w[0].fzf) - mean(w[1].fzf);
        const double mr = mean(w[0].fzr) - mean(w[1].fzr);
        const std::string tag = c.lvl + (c.strut ? "_strut" : (c.lvl == "L5" ? "_penalty" : ""));
        std::printf("B3b %s: front meas=%.6f exp=%.6f rel=%+.5f | rear meas=%.6f exp=%.6f "
                    "rel=%+.5f | theta_mean=%.4e dax=%.4e\n",
                    tag.c_str(), mf, ef, mf / ef - 1.0, mr, er, mr / er - 1.0, mean(w[0].theta),
                    dax);
        {
            std::ofstream f(out_dir() + "b3b_" + tag + ".json");
            f.precision(17);
            const auto arr = [&f](const std::vector<double>& v) {
                f << "[";
                for (std::size_t i = 0; i < v.size(); ++i) f << (i ? ", " : "") << v[i];
                f << "]";
            };
            f << "{\"on\": {\"q\": ";
            arr(w[0].q);
            f << ", \"theta\": ";
            arr(w[0].theta);
            f << ", \"ax\": ";
            arr(w[0].ax);
            f << ", \"fz_front\": ";
            arr(w[0].fzf);
            f << ", \"fz_rear\": ";
            arr(w[0].fzr);
            f << "}, \"off\": {\"q\": ";
            arr(w[1].q);
            f << ", \"ax\": ";
            arr(w[1].ax);
            f << ", \"fz_front\": ";
            arr(w[1].fzf);
            f << ", \"fz_rear\": ";
            arr(w[1].fzr);
            f << "}}\n";
        }
        // DEFINITION: the guard applies to each axle; a not-judgeable axle fails the gate.
        // L5 only (PO mail 91b88183): the braking-pitch run is an information item there
        // (judged by B3b-L5i); a not-judgeable L5 case prints INFO and passes. L3 unchanged.
        if (c.lvl == "L5" && (std::abs(ef) < kGuardN || std::abs(er) < kGuardN)) {
            std::printf("B3b %s INFO not judgeable (|exp| < %.0f N): front exp=%.6f rel=%+.5f, "
                        "rear exp=%.6f rel=%+.5f\n",
                        tag.c_str(), kGuardN, ef, mf / ef - 1.0, er, mr / er - 1.0);
            continue;
        }
        EXPECT_GE(std::abs(ef), kGuardN) << tag << " front not judgeable (stop and report)";
        EXPECT_GE(std::abs(er), kGuardN) << tag << " rear not judgeable (stop and report)";
        if (std::abs(ef) >= kGuardN) EXPECT_LE(std::abs(mf / ef - 1.0), kTolB3) << tag << " front";
        if (std::abs(er) >= kGuardN) EXPECT_LE(std::abs(mr / er - 1.0), kTolB3) << tag << " rear";
    }
}

// ---------------------------------------------------------------- B3b-L5i

// Injected pitch on L5 (PO mail 91b88183, DEFINITION 2.5a): from the settled reference
// state only the body pitch is changed by theta_inj (strut: the unsprung points rotate
// with the body), restored at vx 25 m/s into the slope car and the no-key car; the
// per-axle aero load of the first aero evaluation (detail::AeroObs, read-only) differs
// by q * dClA_dpitch * theta * (s_e, 1 - s_e) (handcalc.b3l5i). +/-2 %, 10 N per axle.
TEST(AeroRide51, B3b_L5i_InjectedPitchAxleLoads) {
    std::ostringstream js;
    js.precision(17);
    for (const bool strut : {false, true}) {
        const VehicleParams on_vp = with_slopes(fs_car(), {{"front_wing", {0.100, 0.0, 30.0}}});
        const VehicleParams off_vp = fs_car();
        const double vx = 25.0, L = on_vp.wheelbase;
        double xe = 0.0;
        for (const auto& e : on_vp.aero_elements)
            if (e.name == "front_wing") xe = e.x;
        const double s_e = (xe + L) / L;
        const double q = 0.5 * on_vp.air_density * vx * vx;
        const std::string tag = strut ? "strut" : "penalty";
        js << (strut ? ", " : "") << "\"" << tag << "\": {\"cases\": [";
        for (const double th_inj : {0.01, -0.01}) {
            auto on = session("L5", on_vp, strut);
            auto off = session("L5", off_vp, strut);
            State s0;
            s0.position.z() = on_vp.cg_height;
            s0.velocity.x() = vx;
            const double w = vx / on_vp.wheel_radius_nominal;
            s0.wheel_spin = {{w, w, w, w}};
            on->reset_settled(s0);
            off->reset_settled(s0);
            const AeroReference ref = on->aero_reference();
            ASSERT_TRUE(ref.ran);
            // Snapshot of the settled session; replace the true state only.
            SessionSnapshot sn = on->snapshot();
            std::size_t p = 0;
            snap::check_header(sn.d, p);
            const std::size_t p0 = p;
            State st = snap::get_state(sn.d, p);
            const std::size_t p1 = p;
            const Euler e0 = euler_from_quat(st.orientation);
            const Quat q_new = quat_from_euler(Euler{ref.ref_roll, ref.ref_pitch + th_inj, e0.yaw});
            const Vec3 o_old = st.position;
            Vec3 o_new = st.position;
            o_new.z() = ref.ref_z;
            const Mat3 dR = q_new.toRotationMatrix() * st.orientation.toRotationMatrix().transpose();
            for (int i = 0; strut && i < NUM_WHEELS; ++i) {
                st.unsprung_pos[i] = o_new + dR * (st.unsprung_pos[i] - o_old);
                st.unsprung_vel[i] = q_new.toRotationMatrix() * Vec3(vx, 0.0, 0.0);
            }
            st.position = o_new;
            st.orientation = q_new;
            st.velocity = Vec3(vx, 0.0, 0.0);
            st.angular_velocity = Vec3::Zero();
            st.wheel_spin = {{w, w, w, w}};
            std::vector<double> ns;
            snap::put_state(ns, st);
            ASSERT_EQ(ns.size(), p1 - p0);
            std::copy(ns.begin(), ns.end(), sn.d.begin() + static_cast<long>(p0));
            double ff[2] = {0.0, 0.0}, fr[2] = {0.0, 0.0};
            detail::AeroObs obs_on;
            int k = 0;
            for (SimSession* s : {on.get(), off.get()}) {
                s->restore(sn);
                detail::free_3d_aero_obs_arm(s->dynamics());
                s->set_input(CmdL6{});
                s->tick(1e-3);
                const detail::AeroObs o = detail::free_3d_aero_obs(s->dynamics());
                ASSERT_GT(o.count, 0) << tag;
                if (k == 0) obs_on = o;
                ff[k] = o.fz_front;
                fr[k] = o.fz_rear;
                ++k;
            }
            // Validity: the slopes saw exactly the injected state.
            EXPECT_LE(std::abs(obs_on.pitch - th_inj), 1e-9) << tag << " injected pitch";
            EXPECT_LE(std::abs(obs_on.dz), 1e-9) << tag << " injected heave";
            EXPECT_LE(std::abs(obs_on.vx - vx), 1e-9) << tag << " injected speed";
            const double ef = q * 30.0 * th_inj * s_e, er = q * 30.0 * th_inj * (1.0 - s_e);
            const double mf = ff[0] - ff[1], mr = fr[0] - fr[1];
            std::printf("B3b_L5i %s theta=%+.3f: front meas=%.9f exp=%.9f rel=%+.3e | rear meas=%.9f "
                        "exp=%.9f rel=%+.3e | seen pitch=%.12e dz=%.3e\n",
                        tag.c_str(), th_inj, mf, ef, mf / ef - 1.0, mr, er, mr / er - 1.0,
                        obs_on.pitch, obs_on.dz);
            js << (th_inj > 0.0 ? "" : ", ") << "{\"theta\": " << th_inj << ", \"front_meas\": " << mf
               << ", \"rear_meas\": " << mr << "}";
            ASSERT_GE(std::abs(ef), kGuardN) << tag << " front not judgeable (stop and report)";
            ASSERT_GE(std::abs(er), kGuardN) << tag << " rear not judgeable (stop and report)";
            EXPECT_LE(std::abs(mf / ef - 1.0), kTolB3) << tag << " front";
            EXPECT_LE(std::abs(mr / er - 1.0), kTolB3) << tag << " rear";
        }
        js << "]}";
    }
    std::ofstream f(out_dir() + "b3l5i.json");
    f << "{" << js.str() << "}\n";
}

// ---------------------------------------------------------------- aero module (C++)

namespace {

/// Counts the module calls; returns no force.
struct CountingAero final : IAeroModel {
    int n_reset = 0, n_begin = 0, n_compute = 0;
    std::vector<AeroForce> compute(const SubsystemContext&, const AeroInput&) override {
        ++n_compute;
        return {};
    }
    void begin_step(const SubsystemContext&, double) override { ++n_begin; }
    void reset() override { ++n_reset; }
};

/// Returns a fixed lateral force (Fy only).
struct LateralAero final : IAeroModel {
    double fy = 0.0;
    std::vector<AeroForce> compute(const SubsystemContext&, const AeroInput&) override {
        return {AeroForce{"side", Vec3(0.0, fy, 0.0), Vec3(-0.5, 0.0, 0.3)}};
    }
};

/// fs_aero_example with the built-in aero switched off explicitly (DEFINITION 2.6).
VehicleParams no_aero_car() {
    VehicleParams vp = fs_car();
    vp.aero_elements.clear();
    vp.aero_drag_coeff = 0.0;
    vp.frontal_area = 0.0;
    vp.aero_lift_front = 0.0;
    vp.aero_lift_rear = 0.0;
    vp.aero_drag_height.reset();
    return vp;
}

}  // namespace

TEST(AeroRide51, B1a_CountingModuleCalls) {
    const VehicleParams vp = fs_car();
    for (const bool strut : {false, true}) {
        auto s = session("L5", vp, strut);
        auto m = std::make_shared<CountingAero>();
        ASSERT_TRUE(s->dynamics().set_aero_module(m));
        s->reset_settled(spawn60(vp));
        ASSERT_TRUE(s->aero_reference().ran);
        EXPECT_EQ(m->n_reset, 1);
        EXPECT_EQ(m->n_begin, 0);
        EXPECT_EQ(m->n_compute, 0);   // nothing during the settle run
        CmdL6 cmd;
        cmd.v_target = kV60;
        for (int k = 0; k < 100; ++k) {
            s->set_input(cmd);
            s->tick(1e-3);
        }
        EXPECT_EQ(m->n_begin, 100);
        EXPECT_GT(m->n_compute, 0);
        EXPECT_EQ(m->n_reset, 1);
    }
}

TEST(AeroRide51, B8_ModuleReplacesBuiltInAero) {
    const VehicleParams with_aero = fs_car();
    const VehicleParams none = no_aero_car();
    int warn = 0;
    struct Case { std::string lvl; bool strut; };
    const std::vector<Case> cases = {{"L2", false}, {"L3", false}, {"L5", false}, {"L5", true}};
    warn = count_log("aero module replaces the built-in aero", [&] {
        for (const Case& c : cases) {
            auto a = session(c.lvl, with_aero, c.strut);
            auto b = session(c.lvl, none, c.strut);
            ASSERT_TRUE(a->dynamics().set_aero_module(std::make_shared<CountingAero>()));
            const auto ra = run60(*a, with_aero, 2000);
            const auto rb = run60(*b, none, 2000);
            ASSERT_EQ(ra.size(), rb.size());
            double worst = 0.0;
            for (std::size_t k = 0; k < ra.size(); ++k)
                for (std::size_t i = 0; i < ra[k].size(); ++i) {
                    const double d = std::abs(ra[k][i] - rb[k][i]) /
                                     std::max(1.0, std::abs(rb[k][i]));
                    if (!(d <= worst)) worst = d;
                }
            std::printf("B8 %s%s: max rel diff %.3e\n", c.lvl.c_str(), c.strut ? " strut" : "",
                        worst);
            EXPECT_LE(worst, 1e-12) << c.lvl << (c.strut ? " strut" : "");
        }
    });
    EXPECT_EQ(warn, 1);
    auto l1 = session("L1", with_aero);
    EXPECT_FALSE(l1->dynamics().set_aero_module(std::make_shared<CountingAero>()));
}

TEST(AeroRide51, B6_ModuleFyIgnoredBelowL5) {
    const VehicleParams vp = no_aero_car();
    int warn = 0;
    warn = count_log("returned a lateral force Fy", [&] {
        for (const std::string lvl : {"L2", "L3"}) {
            auto a = session(lvl, vp);
            auto b = session(lvl, vp);
            auto fa = std::make_shared<LateralAero>();
            fa->fy = 500.0;
            auto fb = std::make_shared<LateralAero>();
            ASSERT_TRUE(a->dynamics().set_aero_module(fa));
            ASSERT_TRUE(b->dynamics().set_aero_module(fb));
            const auto ra = run60(*a, vp, 500);
            const auto rb = run60(*b, vp, 500);
            bool same = true;
            for (std::size_t k = 0; k < ra.size(); ++k) same = same && bits_equal(ra[k], rb[k]);
            EXPECT_TRUE(same) << lvl;
        }
        // L5 takes Fy: the lateral force moves the car sideways.
        auto c = session("L5", vp);
        auto fc = std::make_shared<LateralAero>();
        fc->fy = 500.0;
        ASSERT_TRUE(c->dynamics().set_aero_module(fc));
        run60(*c, vp, 500);
        EXPECT_GT(std::abs(c->output().state.position.y()), 1e-3);
    });
    EXPECT_EQ(warn, 1);
}
