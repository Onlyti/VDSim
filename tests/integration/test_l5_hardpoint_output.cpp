// Work order 48c (C++): L5 per-wheel camber/toe and hardpoint-coordinate output.
//
// Gates fixed before the implementation commit (PO 48c phase-1 and origin verdicts):
//   K2  every L5 tick, SimOutput camber/toe == free_3d_wheel_camber/toe, bit for bit.
//   C1  toe sign follows the product toe of theory 14 §14.16 (the locked 48b T5 table:
//       +22.5 mm rack -> FL and FR toe both positive, -22.5 mm -> both negative).
//   K3  |p_out - (p_yaml + dx_axle e_x)| <= 1e-9 m, right side mirrored in y, at
//       travel 0 / rack 0, both right after the attach and after reset().
//       dx_front = 0 - w_front.x, dx_rear = -wheelbase - w_rear.x (w = YAML wheel.center).
//   K4  rigid-link length change <= 1e-6 m over bump +-30 mm x rack +-22.5 mm, on the
//       corner model directly (prescribed sweep) and on every tick of a session run.
//       Sweep check (the sweep must really move the points, else K4 says nothing):
//       wheel.center z - static z == travel within 1e-6 m on the prescribed sweep, and
//       the output tie_rod.rack y - static y == the applied rack within 1e-12 m on the
//       session run.
//   K5  symmetric input (equal bump, rack 0): right point = left point with y negated,
//       other components equal, 1e-9 m.
//   K6  rack_per_rad 0.08 given == key absent (L4 and L5 state bit identical); 0.04
//       gives exactly half the L5 rack displacement of 0.08 for the same wheel command.
//   D2  L4 and any non-hardpoint path: camber/toe 0, valid false, no points.
// K7 (negative check) is a manual procedure: delete the per-tick hardpoint refresh in
// Free3DDynamics::update_corner_dae -> K4 fails; restore -> passes.
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"
#include "vdsim/suspension.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;
const std::string kKin = kSrc + "/configs/parts/susp_kinematics/kin/";
constexpr double kDt = 1e-3;
constexpr double kRackM = 0.0225;
constexpr double kBumpM = 0.030;
constexpr double kTolStatic = 1e-9;   // K3, K5
constexpr double kTolLink = 1e-6;     // K4
constexpr double kPi = 3.14159265358979323846;

/** One front/rear hardpoint pair under test. */
struct Pair {
    const char* front;
    const char* rear;
};
// Sedan pair (MacPherson + trailing arm) and sports pair (double wishbone + 5-link):
// together they cover every topology the L5 corner model exports.
const Pair kPairs[] = {{"mp_front_sedan", "ta_rear_sedan"},
                       {"dw_front_sports", "5link_rear_sports"}};

/** Exported point names per topology, in output order (the output contract). */
const std::map<std::string, std::vector<std::string>> kNames = {
    {"macpherson", {"lca.chassis_front", "lca.chassis_rear", "lca.knuckle", "strut.top",
                    "strut.bottom", "tie_rod.rack", "tie_rod.knuckle", "wheel.center"}},
    {"double_wishbone", {"lca.chassis_front", "lca.chassis_rear", "lca.knuckle",
                         "uca.chassis_front", "uca.chassis_rear", "uca.knuckle",
                         "tie_rod.rack", "tie_rod.knuckle", "spring_damper.chassis",
                         "spring_damper.lca", "wheel.center"}},
    {"trailing_arm", {"arm_pivot.chassis_inboard", "arm_pivot.chassis_outboard",
                      "wheel.center"}},
    {"five_link", {"links.upper_fore.chassis", "links.upper_fore.knuckle",
                   "links.upper_aft.chassis", "links.upper_aft.knuckle",
                   "links.lower_fore.chassis", "links.lower_fore.knuckle",
                   "links.lower_aft.chassis", "links.lower_aft.knuckle",
                   "links.toe_link.chassis", "links.toe_link.knuckle", "wheel.center"}},
};

/** Rigid pairs per topology (links + the knuckle body, wheel centre on the knuckle). */
const std::map<std::string, std::vector<std::pair<std::string, std::string>>> kRigid = {
    {"macpherson",
     {{"lca.chassis_front", "lca.knuckle"}, {"lca.chassis_rear", "lca.knuckle"},
      {"tie_rod.rack", "tie_rod.knuckle"},
      {"lca.knuckle", "strut.bottom"}, {"lca.knuckle", "tie_rod.knuckle"},
      {"strut.bottom", "tie_rod.knuckle"}, {"wheel.center", "lca.knuckle"},
      {"wheel.center", "strut.bottom"}, {"wheel.center", "tie_rod.knuckle"}}},
    {"double_wishbone",
     {{"lca.chassis_front", "lca.knuckle"}, {"lca.chassis_rear", "lca.knuckle"},
      {"uca.chassis_front", "uca.knuckle"}, {"uca.chassis_rear", "uca.knuckle"},
      {"tie_rod.rack", "tie_rod.knuckle"},
      {"lca.knuckle", "uca.knuckle"}, {"lca.knuckle", "tie_rod.knuckle"},
      {"uca.knuckle", "tie_rod.knuckle"}, {"wheel.center", "lca.knuckle"},
      {"wheel.center", "uca.knuckle"}, {"wheel.center", "tie_rod.knuckle"},
      {"spring_damper.lca", "lca.chassis_front"}, {"spring_damper.lca", "lca.chassis_rear"},
      {"spring_damper.lca", "lca.knuckle"}}},
    {"trailing_arm",
     {{"arm_pivot.chassis_inboard", "wheel.center"},
      {"arm_pivot.chassis_outboard", "wheel.center"}}},
    {"five_link",
     {{"links.upper_fore.chassis", "links.upper_fore.knuckle"},
      {"links.upper_aft.chassis", "links.upper_aft.knuckle"},
      {"links.lower_fore.chassis", "links.lower_fore.knuckle"},
      {"links.lower_aft.chassis", "links.lower_aft.knuckle"},
      {"links.toe_link.chassis", "links.toe_link.knuckle"},
      {"wheel.center", "links.upper_fore.knuckle"}, {"wheel.center", "links.upper_aft.knuckle"},
      {"wheel.center", "links.lower_fore.knuckle"}, {"wheel.center", "links.lower_aft.knuckle"},
      {"wheel.center", "links.toe_link.knuckle"},
      {"links.upper_fore.knuckle", "links.lower_aft.knuckle"},
      {"links.upper_aft.knuckle", "links.toe_link.knuckle"}}},
};

/** Part YAML read independently of the core (yaml-cpp, dotted key path). */
struct KinYaml {
    std::string type;
    std::map<std::string, Vec3> pts;   // every name of kNames[type]
};

Vec3 node_vec3(const YAML::Node& n) {
    return Vec3(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

KinYaml load_kin(const std::string& stem) {
    const YAML::Node root = YAML::LoadFile(kKin + stem + ".yaml");
    KinYaml k;
    k.type = root["type"].as<std::string>();
    for (const auto& name : kNames.at(k.type)) {
        YAML::Node n = YAML::Clone(root);
        std::size_t start = 0;
        while (true) {
            const std::size_t dot = name.find('.', start);
            const std::string part = name.substr(start, dot == std::string::npos
                                                            ? std::string::npos
                                                            : dot - start);
            n = n[part];
            if (dot == std::string::npos) break;
            start = dot + 1;
        }
        k.pts[name] = node_vec3(n);
    }
    return k;
}

VehicleParams sedan() {
    return VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml");
}

TireParams tire() {
    return TireParams::from_yaml(kSrc + "/configs/parts/tire/generic_pacejka.yaml");
}

SolverParams l5_dae() {
    SolverParams sp;
    sp.stunt_physics = true;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 16;
    sp.l5_spatial_suspension = true;
    return sp;
}

/** Test-only physical rack source whose value the test changes between ticks. */
class SettableRack final : public ISteeringSystem {
public:
    explicit SettableRack(std::shared_ptr<double> rack) : rack_(std::move(rack)) {}
    SteeringOutput apply(const SubsystemContext&) override {
        SteeringOutput out;
        out.roadwheel_angle = 0.0;
        out.rack_travel = *rack_;
        return out;
    }
private:
    std::shared_ptr<double> rack_;
};

std::unique_ptr<SimSession> make_session(const VehicleParams& vp, const SolverParams& sp,
                                         const char* level = "L5") {
    SimConfig cfg;
    cfg.nominal_dt = kDt;
    return std::make_unique<SimSession>(create_dynamics_for_level(level),
                                        create_flat_ground(0.0, 1.0), vp,
                                        TireSetup(tire()), sp, cfg);
}

void attach_pair(SimSession& s, const Pair& p) {
    ASSERT_TRUE(free_3d_attach_multibody(s.dynamics(), true,
                mb::SuspensionTopology::from_yaml(kKin + p.front + ".yaml"), true));
    ASSERT_TRUE(free_3d_attach_multibody(s.dynamics(), false,
                mb::SuspensionTopology::from_yaml(kKin + p.rear + ".yaml"), true));
}

/** Expected output point: YAML value shifted by the axle offset, y mirrored on the right. */
Vec3 expected_point(const Vec3& p_yaml, double dx, bool right) {
    return Vec3(p_yaml.x() + dx, right ? -p_yaml.y() : p_yaml.y(), p_yaml.z());
}

double axle_dx(const KinYaml& k, bool front, const VehicleParams& vp) {
    const double wx = k.pts.at("wheel.center").x();
    return front ? (0.0 - wx) : (-vp.wheelbase - wx);
}

/** K3 on one set of outputs. */
void check_static(const std::array<WheelHardpoints, NUM_WHEELS>& hp,
                  const std::array<bool, NUM_WHEELS>& valid,
                  const KinYaml& kf, const KinYaml& kr, const VehicleParams& vp,
                  const char* when) {
    for (int i = 0; i < NUM_WHEELS; ++i) {
        SCOPED_TRACE(std::string(when) + " wheel " + std::to_string(i));
        const bool front = (i == WHEEL_FL || i == WHEEL_FR);
        const bool right = (i == WHEEL_FR || i == WHEEL_RR);
        const KinYaml& k = front ? kf : kr;
        const auto& names = kNames.at(k.type);
        ASSERT_TRUE(valid[i]);
        ASSERT_EQ(hp[i].count, static_cast<int>(names.size()));
        const double dx = axle_dx(k, front, vp);
        for (std::size_t j = 0; j < names.size(); ++j) {
            const Vec3 e = expected_point(k.pts.at(names[j]), dx, right);
            EXPECT_LE((hp[i].p[j] - e).norm(), kTolStatic) << names[j];
        }
    }
}

int index_of(const std::vector<std::string>& names, const std::string& n) {
    for (std::size_t j = 0; j < names.size(); ++j)
        if (names[j] == n) return static_cast<int>(j);
    return -1;
}

/** Max rigid-pair length change [m] of one corner against the YAML design lengths. */
double max_link_error(const std::string& type, const std::vector<std::string>& names,
                      const std::array<Vec3, kMaxWheelHardpoints>& p,
                      const std::map<std::string, Vec3>& design) {
    double worst = 0.0;
    for (const auto& [a, b] : kRigid.at(type)) {
        const int ia = index_of(names, a), ib = index_of(names, b);
        if (ia < 0 || ib < 0) throw std::runtime_error("missing point " + a + "/" + b);
        const double L0 = (design.at(a) - design.at(b)).norm();
        worst = std::max(worst, std::abs((p[ia] - p[ib]).norm() - L0));
    }
    return worst;
}

}  // namespace

// --- Layout: names, axle offsets, wheel-centre offsets -----------------------------

TEST(L5HardpointOutput, LayoutNamesAndOffsets) {
    const VehicleParams vp = sedan();
    for (const auto& pr : kPairs) {
        SCOPED_TRACE(pr.front);
        auto s = make_session(vp, l5_dae());
        attach_pair(*s, pr);
        const KinYaml kf = load_kin(pr.front), kr = load_kin(pr.rear);
        const HardpointLayout lay = free_3d_hardpoint_layout(s->dynamics());
        EXPECT_EQ(lay.names[0], kNames.at(kf.type));
        EXPECT_EQ(lay.names[1], kNames.at(kr.type));
        EXPECT_EQ(lay.axle_offset_x[0], axle_dx(kf, true, vp));
        EXPECT_EQ(lay.axle_offset_x[1], axle_dx(kr, false, vp));
        EXPECT_EQ(lay.wheel_center_dy[0], kf.pts.at("wheel.center").y() - 0.5 * vp.track_front);
        EXPECT_EQ(lay.wheel_center_dy[1], kr.pts.at("wheel.center").y() - 0.5 * vp.track_rear);
    }
}

// --- K3: static coordinates at attach and after reset ------------------------------

TEST(L5HardpointOutput, K3StaticCoordinatesAtAttachAndAfterReset) {
    const VehicleParams vp = sedan();
    for (const auto& pr : kPairs) {
        SCOPED_TRACE(pr.front);
        auto s = make_session(vp, l5_dae());
        attach_pair(*s, pr);
        const KinYaml kf = load_kin(pr.front), kr = load_kin(pr.rear);
        check_static(free_3d_wheel_hardpoints(s->dynamics()),
                     free_3d_wheel_kin_valid(s->dynamics()), kf, kr, vp, "attach/dyn");
        const SimOutput o0 = s->output();
        check_static(o0.hardpoints, o0.wheel_kin_valid, kf, kr, vp, "attach/output");
        // Move the corners, then reset: the output must return to the static design.
        State s0;
        s0.position.z() = vp.cg_height;
        s->reset_settled(s0);
        for (int k = 0; k < 200; ++k) {
            CmdL4 u;
            u.throttle = 0.3;
            u.steer_angle_wheel = 0.04;
            s->set_input(u);
            s->tick(kDt);
        }
        s->reset(s0);
        const SimOutput o1 = s->output();
        check_static(o1.hardpoints, o1.wheel_kin_valid, kf, kr, vp, "reset/output");
    }
}

// --- K4: rigid links over a prescribed bump x rack sweep (corner model) ------------

TEST(L5HardpointOutput, K4LinkLengthsPrescribedSweep) {
    const char* stems[] = {"mp_front_sedan", "ta_rear_sedan", "dw_front_sports",
                           "5link_rear_sports"};
    for (const char* stem : stems) {
        SCOPED_TRACE(stem);
        const KinYaml k = load_kin(stem);
        const auto model = mb::create_hard_joint_dae_model(
            mb::SuspensionTopology::from_yaml(kKin + std::string(stem) + ".yaml"));
        const std::vector<std::string> names = model->hardpoint_names();
        ASSERT_EQ(names, kNames.at(k.type));
        const bool steered = index_of(names, "tie_rod.rack") >= 0;
        const int iw = index_of(names, "wheel.center");
        double worst = 0.0;
        for (int a = -4; a <= 4; ++a) {
            const double travel = kBumpM * a / 4.0;
            for (int r = -1; r <= 1; ++r) {
                const double rack = steered ? kRackM * r : 0.0;
                for (int side = 0; side < 2; ++side) {
                    const bool right = (side == 1);
                    mb::HardJointCornerState st;
                    mb::PrescribedCornerMotion mot;
                    mot.travel_z = travel;
                    mot.steer_rack_dy = right ? -rack : rack;
                    model->initialize(st, mot);
                    std::array<Vec3, mb::kMaxCornerHardpoints> raw {};
                    const int n = model->hardpoints_at(st, mot.steer_rack_dy, raw);
                    ASSERT_EQ(n, static_cast<int>(names.size()));
                    std::array<Vec3, kMaxWheelHardpoints> out {};
                    mb::corner_points_to_vehicle(raw, n, right, 0.0, out);
                    std::map<std::string, Vec3> design;
                    for (const auto& [nm, v] : k.pts) design[nm] = expected_point(v, 0.0, right);
                    // Sweep check: the wheel centre really sits at the commanded travel.
                    EXPECT_NEAR(out[iw].z() - design.at("wheel.center").z(), travel, kTolLink)
                        << "travel " << travel << " rack " << rack;
                    worst = std::max(worst, max_link_error(k.type, names, out, design));
                }
            }
        }
        std::printf("[K4 sweep] %s max link-length change %.3e m\n", stem, worst);
        EXPECT_LE(worst, kTolLink);
    }
}

// --- K4 on the session output + rack sweep check ------------------------------------

TEST(L5HardpointOutput, K4LinkLengthsEveryTickAndRackFollows) {
    for (const auto& pr : kPairs) {
        SCOPED_TRACE(pr.front);
        VehicleParams vp = sedan();
        vp.steer_input = "rack_travel";
        auto s = make_session(vp, l5_dae());
        attach_pair(*s, pr);
        const KinYaml kf = load_kin(pr.front), kr = load_kin(pr.rear);
        State s0;
        s0.position.z() = vp.cg_height;
        s0.velocity.x() = 5.0;
        const double w = 5.0 / vp.wheel_radius_nominal;
        s0.wheel_spin = {{w, w, w, w}};
        s->reset_settled(s0);
        // Same order as the 48b rack tests: replace the steering slot after the reset.
        auto rack = std::make_shared<double>(0.0);
        ASSERT_TRUE(s->dynamics().set_steering_module(std::make_shared<SettableRack>(rack)));
        const auto& names_f = kNames.at(kf.type);
        const int irack = index_of(names_f, "tie_rod.rack");
        ASSERT_GE(irack, 0);
        double worst = 0.0, worst_rack = 0.0, max_rack_seen = 0.0;
        const int n = 2000;
        for (int k = 0; k < n; ++k) {
            *rack = kRackM * std::sin(2.0 * kPi * 0.5 * k * kDt);
            CmdL4 u;
            u.throttle = k < 1000 ? 0.25 : 0.0;
            u.brake = k >= 1400 ? 0.3 : 0.0;
            s->set_input(u);
            s->tick(kDt);
            const SimOutput o = s->output();
            for (int i = 0; i < NUM_WHEELS; ++i) {
                const bool front = (i == WHEEL_FL || i == WHEEL_FR);
                const bool right = (i == WHEEL_FR || i == WHEEL_RR);
                const KinYaml& kk = front ? kf : kr;
                ASSERT_TRUE(o.wheel_kin_valid[i]);
                std::map<std::string, Vec3> design;
                const double dx = axle_dx(kk, front, vp);
                for (const auto& [nm, v] : kk.pts) design[nm] = expected_point(v, dx, right);
                worst = std::max(worst, max_link_error(kk.type, kNames.at(kk.type),
                                                       o.hardpoints[i].p, design));
                if (front) {
                    // Physical +y rack moves both inner tie-rod points by +rack in y.
                    const double dy = o.hardpoints[i].p[irack].y() - design.at("tie_rod.rack").y();
                    worst_rack = std::max(worst_rack, std::abs(dy - *rack));
                }
            }
            max_rack_seen = std::max(max_rack_seen, std::abs(*rack));
        }
        std::printf("[K4 session] %s max link change %.3e m, rack residual %.3e m\n",
                    pr.front, worst, worst_rack);
        EXPECT_GE(max_rack_seen, 0.99 * kRackM);
        EXPECT_LE(worst_rack, 1e-12);
        EXPECT_LE(worst, kTolLink);
    }
}

// --- K5: left/right mirror under a symmetric input ----------------------------------

TEST(L5HardpointOutput, K5MirrorPrescribed) {
    const char* stems[] = {"mp_front_sedan", "ta_rear_sedan", "dw_front_sports",
                           "5link_rear_sports"};
    for (const char* stem : stems) {
        SCOPED_TRACE(stem);
        const auto model = mb::create_hard_joint_dae_model(
            mb::SuspensionTopology::from_yaml(kKin + std::string(stem) + ".yaml"));
        for (int a = -4; a <= 4; ++a) {
            mb::PrescribedCornerMotion mot;
            mot.travel_z = kBumpM * a / 4.0;
            mot.steer_rack_dy = 0.0;
            std::array<std::array<Vec3, kMaxWheelHardpoints>, 2> out {};
            int n = 0;
            for (int side = 0; side < 2; ++side) {
                mb::HardJointCornerState st;
                model->initialize(st, mot);
                std::array<Vec3, mb::kMaxCornerHardpoints> raw {};
                n = model->hardpoints_at(st, 0.0, raw);
                mb::corner_points_to_vehicle(raw, n, side == 1, -1.35, out[side]);
            }
            for (int j = 0; j < n; ++j) {
                EXPECT_LE(std::abs(out[0][j].x() - out[1][j].x()), kTolStatic);
                EXPECT_LE(std::abs(out[0][j].y() + out[1][j].y()), kTolStatic);
                EXPECT_LE(std::abs(out[0][j].z() - out[1][j].z()), kTolStatic);
            }
        }
    }
}

TEST(L5HardpointOutput, K5MirrorSessionSymmetricBump) {
    const VehicleParams vp = sedan();
    for (const auto& pr : kPairs) {
        SCOPED_TRACE(pr.front);
        auto s = make_session(vp, l5_dae());
        attach_pair(*s, pr);
        State s0;
        s0.position.z() = vp.cg_height;
        s0.susp_compression = {{0.02, 0.02, -0.01, -0.01}};
        s->reset(s0);
        s->set_input(CmdL4{});
        s->tick(kDt);
        const SimOutput o = s->output();
        for (int axle = 0; axle < 2; ++axle) {
            const int l = axle == 0 ? WHEEL_FL : WHEEL_RL;
            const int r = axle == 0 ? WHEEL_FR : WHEEL_RR;
            ASSERT_EQ(o.hardpoints[l].count, o.hardpoints[r].count);
            ASSERT_GT(o.hardpoints[l].count, 0);
            for (int j = 0; j < o.hardpoints[l].count; ++j) {
                const Vec3& pl = o.hardpoints[l].p[j];
                const Vec3& pr_ = o.hardpoints[r].p[j];
                EXPECT_LE(std::abs(pl.x() - pr_.x()), kTolStatic);
                EXPECT_LE(std::abs(pl.y() + pr_.y()), kTolStatic);
                EXPECT_LE(std::abs(pl.z() - pr_.z()), kTolStatic);
            }
        }
    }
}

// --- K2: SimOutput camber/toe == free functions every tick ---------------------------

TEST(L5HardpointOutput, K2CamberToeBitIdenticalEveryTick) {
    const VehicleParams vp = sedan();
    for (const auto& pr : kPairs) {
        SCOPED_TRACE(pr.front);
        auto s = make_session(vp, l5_dae());
        attach_pair(*s, pr);
        State s0;
        s0.position.z() = vp.cg_height;
        s0.velocity.x() = 10.0;
        const double w = 10.0 / vp.wheel_radius_nominal;
        s0.wheel_spin = {{w, w, w, w}};
        s->reset_settled(s0);
        int mismatches = 0;
        double max_toe = 0.0;
        for (int k = 0; k < 1500; ++k) {
            CmdL4 u;
            u.throttle = k < 700 ? 0.25 : 0.0;
            u.brake = k >= 1000 ? 0.3 : 0.0;
            u.steer_angle_wheel = k < 200 ? 0.0 : (k < 600 ? 0.05 : -0.03);
            s->set_input(u);
            s->tick(kDt);
            const SimOutput o = s->output();
            const auto c = free_3d_wheel_camber(s->dynamics());
            const auto t = free_3d_wheel_toe(s->dynamics());
            if (std::memcmp(o.wheel_camber.data(), c.data(), sizeof(double) * NUM_WHEELS) != 0 ||
                std::memcmp(o.wheel_toe.data(), t.data(), sizeof(double) * NUM_WHEELS) != 0)
                ++mismatches;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                EXPECT_TRUE(o.wheel_kin_valid[i]);
                max_toe = std::max(max_toe, std::abs(o.wheel_toe[i]));
            }
        }
        EXPECT_EQ(mismatches, 0);
        EXPECT_GT(max_toe, 0.0);   // the run really produced toe
    }
}

// --- C1: toe sign = product toe (§14.16, locked 48b T5 rows) -----------------------

TEST(L5HardpointOutput, C1ToeSignFollowsProductToe) {
    for (const double rack_m : {+kRackM, -kRackM}) {
        SCOPED_TRACE(rack_m);
        VehicleParams vp = sedan();
        vp.steer_input = "rack_travel";
        auto s = make_session(vp, l5_dae());
        ASSERT_TRUE(free_3d_attach_multibody(s->dynamics(), true,
                    mb::SuspensionTopology::from_yaml(kKin + "dw_front_sports.yaml"), true));
        State s0;
        s0.position.z() = vp.cg_height;
        s->reset_settled(s0);
        auto rack = std::make_shared<double>(rack_m);
        ASSERT_TRUE(s->dynamics().set_steering_module(std::make_shared<SettableRack>(rack)));
        for (int k = 0; k < 500; ++k) {
            s->set_input(CmdL4{});
            s->tick(kDt);
        }
        const SimOutput o = s->output();
        // T5 row: +22.5 mm -> FL +4.546 deg, FR +4.554 deg; -22.5 mm -> both negative.
        const double sgn = rack_m > 0.0 ? 1.0 : -1.0;
        EXPECT_GT(sgn * o.wheel_toe[WHEEL_FL], 1.0 * kPi / 180.0);
        EXPECT_GT(sgn * o.wheel_toe[WHEEL_FR], 1.0 * kPi / 180.0);
        EXPECT_FALSE(o.wheel_kin_valid[WHEEL_RL]);
        EXPECT_FALSE(o.wheel_kin_valid[WHEEL_RR]);
    }
}

// --- D2 / validity: L4 and non-hardpoint paths export nothing -----------------------

TEST(L5HardpointOutput, D2InvalidOutsideL5HardpointPath) {
    const VehicleParams vp = sedan();
    auto expect_none = [](const SimOutput& o, int wheel) {
        EXPECT_FALSE(o.wheel_kin_valid[wheel]);
        EXPECT_EQ(o.wheel_camber[wheel], 0.0);
        EXPECT_EQ(o.wheel_toe[wheel], 0.0);
        EXPECT_EQ(o.hardpoints[wheel].count, 0);
    };
    {   // L4 with native hardpoints attached: 0.8.2 exports nothing there.
        auto s = make_session(vp, SolverParams{}, "L4");
        ASSERT_TRUE(attach_front_kinematics(s->dynamics(),
                    create_native_kinematics_from_yaml(kKin + "dw_front_sports.yaml")));
        State s0;
        s0.position.z() = vp.cg_height;
        s->reset(s0);
        for (int k = 0; k < 50; ++k) { s->set_input(CmdL4{}); s->tick(kDt); }
        const SimOutput o = s->output();
        for (int i = 0; i < NUM_WHEELS; ++i) expect_none(o, i);
        EXPECT_TRUE(free_3d_hardpoint_layout(s->dynamics()).names[0].empty());
    }
    {   // L5 penalty path (strut path off) with the corner model attached.
        SolverParams sp = l5_dae();
        sp.l5_spatial_suspension = false;
        auto s = make_session(vp, sp);
        attach_pair(*s, kPairs[0]);
        State s0;
        s0.position.z() = vp.cg_height;
        s->reset(s0);
        for (int k = 0; k < 50; ++k) { s->set_input(CmdL4{}); s->tick(kDt); }
        const SimOutput o = s->output();
        for (int i = 0; i < NUM_WHEELS; ++i) expect_none(o, i);
    }
    {   // L5 strut path, front axle only: the rear exports nothing.
        auto s = make_session(vp, l5_dae());
        ASSERT_TRUE(free_3d_attach_multibody(s->dynamics(), true,
                    mb::SuspensionTopology::from_yaml(kKin + "mp_front_sedan.yaml"), true));
        State s0;
        s0.position.z() = vp.cg_height;
        s->reset_settled(s0);
        for (int k = 0; k < 50; ++k) { s->set_input(CmdL4{}); s->tick(kDt); }
        const SimOutput o = s->output();
        EXPECT_TRUE(o.wheel_kin_valid[WHEEL_FL]);
        EXPECT_TRUE(o.wheel_kin_valid[WHEEL_FR]);
        expect_none(o, WHEEL_RL);
        expect_none(o, WHEEL_RR);
    }
}

// --- K6: rack_per_rad ---------------------------------------------------------------

namespace {

/** Per-tick state bytes of a run (position, velocity, attitude, spins, travel, Fz). */
std::vector<double> run_dump(const VehicleParams& vp, const char* level, bool l5) {
    SolverParams sp = l5 ? l5_dae() : SolverParams{};
    auto s = make_session(vp, sp, level);
    if (l5) {
        attach_pair(*s, kPairs[1]);
    } else {
        attach_front_kinematics(s->dynamics(),
                                create_native_kinematics_from_yaml(kKin + "dw_front_sports.yaml"));
        attach_rear_kinematics(s->dynamics(),
                               create_native_kinematics_from_yaml(kKin + "5link_rear_sports.yaml"));
    }
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity.x() = 10.0;
    const double w = 10.0 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    if (l5) s->reset_settled(s0); else s->reset(s0);
    std::vector<double> v;
    for (int k = 0; k < 1000; ++k) {
        CmdL4 u;
        u.throttle = 0.2;
        u.steer_angle_wheel = k < 200 ? 0.0 : 0.05;
        s->set_input(u);
        s->tick(kDt);
        const SimOutput o = s->output();
        const State& st = o.state;
        for (int c = 0; c < 3; ++c) v.push_back(st.position[c]);
        for (int c = 0; c < 3; ++c) v.push_back(st.velocity[c]);
        v.push_back(st.orientation.w()); v.push_back(st.orientation.x());
        v.push_back(st.orientation.y()); v.push_back(st.orientation.z());
        for (int i = 0; i < NUM_WHEELS; ++i) {
            v.push_back(st.wheel_spin[i]);
            v.push_back(st.susp_compression[i]);
            v.push_back(o.Fz[i]);
            v.push_back(o.slip_angle[i]);
        }
    }
    return v;
}

}  // namespace

TEST(L5HardpointOutput, K6RackPerRadExplicitDefaultIsBitIdentical) {
    VehicleParams absent = sedan();
    VehicleParams given = sedan();
    given.rack_per_rad = 0.08;
    for (const auto& [level, l5] : {std::pair<const char*, bool>{"L4", false},
                                    std::pair<const char*, bool>{"L5", true}}) {
        SCOPED_TRACE(level);
        const auto a = run_dump(absent, level, l5);
        const auto b = run_dump(given, level, l5);
        ASSERT_EQ(a.size(), b.size());
        EXPECT_EQ(std::memcmp(a.data(), b.data(), sizeof(double) * a.size()), 0);
    }
}

TEST(L5HardpointOutput, K6RackPerRadHalfGivesHalfRack) {
    std::array<double, NUM_WHEELS> rack_dy[2] {};
    const double ratios[2] = {0.08, 0.04};
    for (int r = 0; r < 2; ++r) {
        VehicleParams vp = sedan();
        vp.rack_per_rad = ratios[r];
        auto s = make_session(vp, l5_dae());
        attach_pair(*s, kPairs[1]);
        State s0;
        s0.position.z() = vp.cg_height;
        s->reset(s0);
        CmdL4 u;
        u.steer_angle_wheel = 0.05;
        s->set_input(u);
        s->tick(kDt);
        rack_dy[r] = free_3d_rack_dy(s->dynamics());
    }
    EXPECT_NE(rack_dy[0][WHEEL_FL], 0.0);
    EXPECT_EQ(rack_dy[1][WHEEL_FL], 0.5 * rack_dy[0][WHEEL_FL]);
    EXPECT_EQ(rack_dy[1][WHEEL_FR], 0.5 * rack_dy[0][WHEEL_FR]);
    EXPECT_EQ(rack_dy[0][WHEEL_RL], 0.0);
    EXPECT_EQ(rack_dy[0][WHEEL_RR], 0.0);
}
