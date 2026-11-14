// Rotor keys (work order 52, C++ side). Fixed before the implementation commit.
//
//   - rotor_inertia / rotor_ratio are aliases of engine_rotational_inertia /
//     final_drive_ratio (same storage, PO verdict D2 option 1).
//       * A conflict is judged only between names written in the YAML (never against the
//         built-in defaults 0.25 / 5.0): two written names with different values throw
//         std::invalid_argument; equal values are accepted.
//       * to_yaml keeps the names the input used; with neither alias written it writes
//         no rotor_* key (the existing keys stay as before).
//       * rotor_ratio <= 0 and rotor_inertia < 0 throw. The range check applies to the
//         rotor_* names only: final_drive_ratio and engine_rotational_inertia keep their
//         present behaviour (no range check), so existing YAML is unchanged (T1; PO
//         condition, mail c8103d86).
//       * powertrain v2 (a `powertrain:` map) together with rotor_inertia or rotor_ratio
//         throws; rotor_axis alone is accepted there.
//   - rotor_axis: none | +x | -x | +y | -y (body axes, x forward, y left). Anything else
//     throws from from_yaml, rotor_axis_vector and every level's session.
//   - L1-L4 read rotor_axis and ignore it: one warning per process for an axis other
//     than none, none on L5 and none for an explicit "none".
//   - V5b single source: rotor_inertia = I, rotor_ratio = G with the gyro absent (no axis
//     on L5, or any axis on L1-L4) runs bit for bit like engine_rotational_inertia = I,
//     final_drive_ratio = G, for FWD, RWD and AWD, 2000 ticks, L1-L5 (both L5 paths).
#include "vdsim/control.hpp"
#include "vdsim/contact.hpp"
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
#include <vector>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;

/// generic_sedan.yaml with the lines of the given top-level keys removed.
std::string sedan_without(const std::vector<std::string>& keys) {
    std::ifstream f(kSrc + "/configs/vehicles/generic_sedan.yaml");
    std::string line, out;
    while (std::getline(f, line)) {
        bool drop = false;
        for (const auto& k : keys)
            if (line.rfind(k + ":", 0) == 0) drop = true;
        if (!drop) out += line + "\n";
    }
    return out;
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

VehicleParams from_text(const std::string& name, const std::string& text) {
    return VehicleParams::from_yaml(write_tmp(name, text));
}

/// The two drivetrain keys and the drive type are removed, so each case writes its own.
std::string base_text() {
    return sedan_without({"engine_rotational_inertia", "final_drive_ratio", "drive_type"});
}

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

/// State and outputs over 2000 ticks of a throttle / steer / brake manoeuvre.
std::vector<double> trace(const std::string& level, const VehicleParams& vp, bool strut) {
    auto s = session(level, vp, strut);
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity.x() = 10.0;
    const double w = 10.0 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    if (level == "L5") s->reset_settled(s0); else s->reset(s0);
    std::vector<double> v;
    for (int k = 0; k < 2000; ++k) {
        CmdL4 u;
        u.throttle = k < 900 ? 0.3 : 0.0;
        u.brake = k >= 1400 ? 0.3 : 0.0;
        u.steer_angle_wheel = k < 300 ? 0.0 : (k < 1100 ? 0.06 : -0.04);
        s->set_input(u);
        s->tick(1e-3);
        const auto o = s->output();
        const State& st = o.state;
        for (int i = 0; i < 3; ++i) v.push_back(st.position[i]);
        for (int i = 0; i < 4; ++i) v.push_back(st.orientation.coeffs()[i]);
        for (int i = 0; i < 3; ++i) v.push_back(st.velocity[i]);
        for (int i = 0; i < 3; ++i) v.push_back(st.angular_velocity[i]);
        for (double x : st.wheel_spin) v.push_back(x);
        for (double x : o.Fz) v.push_back(x);
        v.push_back(o.ax);
        v.push_back(o.ay);
    }
    return v;
}

bool same_bits(const std::vector<double>& a, const std::vector<double>& b) {
    return a.size() == b.size() &&
           std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

}  // namespace

// ---- alias storage --------------------------------------------------------------------

TEST(RotorKeys, AliasesShareStorage) {
    const auto vp = from_text("rk_alias.yaml", base_text() +
                              "drive_type: RWD\nrotor_inertia: 0.3\nrotor_ratio: 4.2\n");
    EXPECT_EQ(vp.engine_rotational_inertia, 0.3);
    EXPECT_EQ(vp.final_drive_ratio, 4.2);
    // Only the alias written, value differs from the built-in default: accepted.
    const auto vi = from_text("rk_alias_i.yaml", base_text() + "rotor_inertia: 0.4\n");
    EXPECT_EQ(vi.engine_rotational_inertia, 0.4);
    EXPECT_EQ(vi.final_drive_ratio, 5.0);
}

TEST(RotorKeys, ConflictOnlyBetweenWrittenNames) {
    EXPECT_NO_THROW(from_text("rk_same.yaml", base_text() +
                              "rotor_inertia: 0.3\nengine_rotational_inertia: 0.3\n"
                              "rotor_ratio: 4.2\nfinal_drive_ratio: 4.2\n"));
    EXPECT_THROW(from_text("rk_diff_i.yaml", base_text() +
                           "rotor_inertia: 0.3\nengine_rotational_inertia: 0.31\n"),
                 std::invalid_argument);
    EXPECT_THROW(from_text("rk_diff_g.yaml", base_text() +
                           "rotor_ratio: 4.2\nfinal_drive_ratio: 4.0\n"),
                 std::invalid_argument);
}

TEST(RotorKeys, ToYamlKeepsInputNames) {
    const auto vp = from_text("rk_rt.yaml", base_text() +
                              "rotor_inertia: 0.3\nrotor_ratio: 4.2\nrotor_axis: +y\n");
    const auto out = ::testing::TempDir() + "rk_rt_out.yaml";
    vp.to_yaml(out);
    const auto txt = read_all(out);
    EXPECT_NE(txt.find("rotor_inertia"), std::string::npos);
    EXPECT_NE(txt.find("rotor_ratio"), std::string::npos);
    EXPECT_NE(txt.find("rotor_axis"), std::string::npos);
    const auto back = VehicleParams::from_yaml(out);
    EXPECT_EQ(back.engine_rotational_inertia, vp.engine_rotational_inertia);
    EXPECT_EQ(back.final_drive_ratio, vp.final_drive_ratio);
    EXPECT_EQ(back.rotor_axis, "+y");

    const auto plain = VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml");
    EXPECT_TRUE(plain.rotor_axis.empty());
    const auto plain_out = ::testing::TempDir() + "rk_plain_out.yaml";
    plain.to_yaml(plain_out);
    const auto ptxt = read_all(plain_out);
    EXPECT_EQ(ptxt.find("rotor_"), std::string::npos);
    EXPECT_NE(ptxt.find("engine_rotational_inertia"), std::string::npos);
    EXPECT_NE(ptxt.find("final_drive_ratio"), std::string::npos);
}

TEST(RotorKeys, AxisValuesAndVectors) {
    const struct { const char* name; double x, y; } cases[] = {
        {"none", 0, 0}, {"+x", 1, 0}, {"-x", -1, 0}, {"+y", 0, 1}, {"-y", 0, -1}};
    for (const auto& c : cases) {
        const auto vp = from_text("rk_axis.yaml", base_text() +
                                  std::string("rotor_axis: \"") + c.name + "\"\n");
        EXPECT_EQ(vp.rotor_axis, c.name);
        const Vec3 e = rotor_axis_vector(vp);
        EXPECT_EQ(e.x(), c.x) << c.name;
        EXPECT_EQ(e.y(), c.y) << c.name;
        EXPECT_EQ(e.z(), 0.0) << c.name;
    }
    VehicleParams unset;
    EXPECT_EQ(rotor_axis_vector(unset), Vec3::Zero());
}

TEST(RotorKeys, InvalidValuesThrow) {
    for (const char* bad : {"+z", "y", "x+", "lateral", "None"}) {
        EXPECT_THROW(from_text("rk_bad.yaml", base_text() + "rotor_axis: \"" + bad + "\"\n"),
                     std::invalid_argument) << bad;
        VehicleParams vp;
        vp.rotor_axis = bad;
        EXPECT_THROW(rotor_axis_vector(vp), std::invalid_argument) << bad;
        for (const char* lvl : {"L1", "L2", "L3", "L4"})
            EXPECT_THROW(session(lvl, vp), std::invalid_argument) << lvl << ' ' << bad;
        EXPECT_THROW(session("L5", vp, false), std::invalid_argument) << bad;
        EXPECT_THROW(session("L5", vp, true), std::invalid_argument) << bad;
    }
    EXPECT_THROW(from_text("rk_g0.yaml", base_text() + "rotor_ratio: 0.0\n"),
                 std::invalid_argument);
    EXPECT_THROW(from_text("rk_gneg.yaml", base_text() + "rotor_ratio: -5.0\n"),
                 std::invalid_argument);
    EXPECT_THROW(from_text("rk_ineg.yaml", base_text() + "rotor_inertia: -0.1\n"),
                 std::invalid_argument);
    EXPECT_NO_THROW(from_text("rk_i0.yaml", base_text() + "rotor_inertia: 0.0\n"));
}

// PO condition (mail c8103d86): the existing names get no new range check.
TEST(RotorKeys, ExistingNamesKeepNoRangeCheck) {
    for (const char* t : {"final_drive_ratio: 0.0\n", "final_drive_ratio: -5.0\n",
                          "engine_rotational_inertia: -0.1\n"})
        EXPECT_NO_THROW(from_text("rk_legacy_range.yaml", base_text() + t)) << t;
    VehicleParams vp;
    vp.final_drive_ratio = 0.0;
    vp.engine_rotational_inertia = -0.1;
    for (const char* lvl : {"L1", "L2", "L3", "L4"})
        EXPECT_NO_THROW(session(lvl, vp)) << lvl;
    EXPECT_NO_THROW(session("L5", vp, false));
    EXPECT_NO_THROW(session("L5", vp, true));
}

TEST(RotorKeys, PowertrainV2RejectsInertiaAndRatio) {
    const std::string pt = "powertrain:\n  engine:\n    inertia: 0.2\n";
    EXPECT_NO_THROW(from_text("rk_pt.yaml", base_text() + pt));
    EXPECT_THROW(from_text("rk_pt_i.yaml", base_text() + pt + "rotor_inertia: 0.25\n"),
                 std::invalid_argument);
    EXPECT_THROW(from_text("rk_pt_g.yaml", base_text() + pt + "rotor_ratio: 5.0\n"),
                 std::invalid_argument);
    EXPECT_NO_THROW(from_text("rk_pt_a.yaml", base_text() + pt + "rotor_axis: \"+x\"\n"));
}

// Nothing else in this binary sets rotor_axis, so the process-wide warning is still unused.
TEST(RotorKeys, IgnoredBelowL5WarnsOncePerProcess) {
    VehicleParams none;
    none.rotor_axis = "none";
    const int n_none = count_log("rotor_axis=", [&] {
        for (const char* lvl : {"L1", "L2", "L3", "L4"}) session(lvl, none);
    });
    EXPECT_EQ(n_none, 0);
    VehicleParams vp;
    vp.rotor_axis = "+y";
    const int n_l5 = count_log("rotor_axis=", [&] {
        session("L5", vp, false);
        session("L5", vp, true);
    });
    EXPECT_EQ(n_l5, 0);
    const int n = count_log("rotor_axis=", [&] {
        for (int rep = 0; rep < 2; ++rep)
            for (const char* lvl : {"L1", "L2", "L3", "L4"}) session(lvl, vp);
    });
    EXPECT_EQ(n, 1);
}

// ---- V5b single source ------------------------------------------------------------------

TEST(RotorKeys, V5b_AliasRunsBitIdenticalToLegacyKeys) {
    struct Lp { const char* level; bool strut; };
    const Lp paths[] = {{"L1", false}, {"L2", false}, {"L3", false}, {"L4", false},
                        {"L5", false}, {"L5", true}};
    for (const char* drive : {"FWD", "RWD", "AWD"}) {
        const std::string d = std::string("drive_type: ") + drive + "\n";
        const auto legacy = from_text("v5b_legacy.yaml", base_text() + d +
                                      "engine_rotational_inertia: 0.3\nfinal_drive_ratio: 4.2\n");
        const auto alias = from_text("v5b_alias.yaml", base_text() + d +
                                     "rotor_inertia: 0.3\nrotor_ratio: 4.2\n");
        const auto alias_y = from_text("v5b_alias_y.yaml", base_text() + d +
                                       "rotor_inertia: 0.3\nrotor_ratio: 4.2\nrotor_axis: \"+y\"\n");
        for (const auto& p : paths) {
            const auto ref = trace(p.level, legacy, p.strut);
            EXPECT_TRUE(same_bits(ref, trace(p.level, alias, p.strut)))
                << drive << ' ' << p.level << (p.strut ? " strut" : "");
            // L1-L4 ignore the axis, so the gyro is absent there as well.
            if (std::string(p.level) != "L5")
                EXPECT_TRUE(same_bits(ref, trace(p.level, alias_y, p.strut)))
                    << drive << ' ' << p.level << " with rotor_axis +y";
        }
    }
}
