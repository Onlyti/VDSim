#include "vdsim/multibody.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

namespace vdsim::mb {

namespace {

Vec3 vec3_from_node(const YAML::Node& n) {
    if (!n || !n.IsSequence() || n.size() < 3)
        throw std::runtime_error("mb: expected [x,y,z] sequence");
    return Vec3(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

bool is_coord_leaf(const YAML::Node& n) {
    if (!n.IsSequence() || n.size() != 3) return false;
    for (std::size_t i = 0; i < 3; ++i)
        if (!n[i].IsScalar()) return false;
    return true;
}

std::string owning_body_for_path(const std::string& path) {
    if (path == "wheel.center") return "knuckle";
    if (path.rfind("lca.", 0) == 0)
        return (path.find("knuckle") != std::string::npos) ? "knuckle" : "lca";
    if (path.rfind("uca.", 0) == 0)
        return (path.find("knuckle") != std::string::npos) ? "knuckle" : "uca";
    if (path.rfind("strut.", 0) == 0)
        return (path == "strut.top") ? "chassis" : "knuckle";
    if (path.rfind("tie_rod.", 0) == 0)
        return (path.find("rack") != std::string::npos) ? "chassis" : "knuckle";
    if (path.rfind("spring_damper.", 0) == 0)
        return (path.find("chassis") != std::string::npos) ? "chassis" : "lca";
    if (path.rfind("arm_pivot.", 0) == 0) return "chassis";
    if (path.rfind("links.", 0) == 0)
        return (path.find("knuckle") != std::string::npos) ? "knuckle" : "chassis";
    return "chassis";
}

void walk_kin_hardpoints(const YAML::Node& node, const std::string& prefix,
                         std::map<std::string, Hardpoint>& out) {
    static const std::set<std::string> kSkip = {"type", "side", "label", "id",
                                                "spin_axis", "static_radius"};
    if (!node.IsMap()) return;
    for (auto it = node.begin(); it != node.end(); ++it) {
        const std::string key = it->first.as<std::string>();
        if (kSkip.count(key)) continue;
        const std::string path = prefix.empty() ? key : prefix + "." + key;
        const YAML::Node& child = it->second;
        if (is_coord_leaf(child)) {
            Hardpoint hp;
            hp.name = path;
            hp.position = vec3_from_node(child);
            hp.body_id = owning_body_for_path(path);
            out[path] = hp;
        } else if (child.IsMap()) {
            walk_kin_hardpoints(child, path, out);
        }
    }
}

JointType joint_type_from_string(const std::string& s) {
    if (s == "ball" || s == "Ball" || s == "spherical") return JointType::Ball;
    if (s == "revolute" || s == "Revolute") return JointType::Revolute;
    if (s == "cylindrical" || s == "Cylindrical") return JointType::Cylindrical;
    if (s == "prismatic" || s == "Prismatic") return JointType::Prismatic;
    if (s == "universal" || s == "Universal") return JointType::Universal;
    if (s == "rigid" || s == "Rigid") return JointType::Rigid;
    throw std::runtime_error("mb: unknown joint type '" + s + "'");
}

void parse_explicit_hardpoints(const YAML::Node& root,
                               std::map<std::string, Hardpoint>& out) {
    const YAML::Node list = root["hardpoints"];
    if (!list || !list.IsSequence()) return;
    for (const auto& n : list) {
        Hardpoint hp;
        hp.name = n["name"].as<std::string>();
        hp.position = vec3_from_node(n["position"]);
        hp.body_id = n["body_id"] ? n["body_id"].as<std::string>()
                                  : owning_body_for_path(hp.name);
        out[hp.name] = hp;
    }
}

void parse_explicit_bodies(const YAML::Node& root, std::vector<RigidBody>& bodies) {
    const YAML::Node list = root["bodies"];
    if (!list || !list.IsSequence()) return;
    for (const auto& n : list) {
        RigidBody b;
        b.id = n["id"].as<std::string>();
        b.mass = n["mass"] ? n["mass"].as<double>() : 0.0;
        if (n["cg_local"]) b.cg_local = vec3_from_node(n["cg_local"]);
        bodies.push_back(b);
    }
}

void parse_explicit_joints(const YAML::Node& root, std::vector<Joint>& joints) {
    const YAML::Node list = root["joints"];
    if (!list || !list.IsSequence()) return;
    for (const auto& n : list) {
        Joint j;
        j.id = n["id"].as<std::string>();
        j.type = joint_type_from_string(n["type"].as<std::string>());
        j.body_a_id = n["body_a"].as<std::string>();
        j.body_b_id = n["body_b"].as<std::string>();
        if (n["position_in_a"]) j.position_in_a = vec3_from_node(n["position_in_a"]);
        if (n["position_in_b"]) j.position_in_b = vec3_from_node(n["position_in_b"]);
        if (n["axis_in_a"]) j.axis_in_a = vec3_from_node(n["axis_in_a"]).normalized();
        joints.push_back(j);
    }
}

void parse_explicit_bushings(const YAML::Node& root, std::vector<Bushing>& bushings) {
    const YAML::Node list = root["bushings"];
    if (!list || !list.IsSequence()) return;
    for (const auto& n : list) {
        Bushing b;
        b.id = n["id"].as<std::string>();
        b.body_a_id = n["body_a"].as<std::string>();
        b.body_b_id = n["body_b"].as<std::string>();
        if (n["k_translation"]) b.k_translation = vec3_from_node(n["k_translation"]);
        if (n["k_rotation"]) b.k_rotation = vec3_from_node(n["k_rotation"]);
        bushings.push_back(b);
    }
}

void add_body(std::vector<RigidBody>& bodies, const std::string& id, double mass) {
    if (std::any_of(bodies.begin(), bodies.end(),
                    [&](const RigidBody& b) { return b.id == id; }))
        return;
    RigidBody b;
    b.id = id;
    b.mass = mass;
    bodies.push_back(b);
}

Vec3 hp_pos(const std::map<std::string, Hardpoint>& hps, const std::string& key) {
    const auto it = hps.find(key);
    return (it != hps.end()) ? it->second.position : Vec3::Zero();
}

void add_ball(std::vector<Joint>& joints,
              const std::map<std::string, Hardpoint>& hps,
              const std::string& id,
              const std::string& body_a, const std::string& body_b,
              const std::string& hp_key) {
    Joint j;
    j.id = id;
    j.type = JointType::Ball;
    j.body_a_id = body_a;
    j.body_b_id = body_b;
    j.position_in_a = hp_pos(hps, hp_key);
    joints.push_back(j);
}

void build_macpherson(SuspensionTopology& topo) {
    add_body(topo.bodies, "chassis", 0.0);
    add_body(topo.bodies, "lca", 8.5);
    add_body(topo.bodies, "strut", 12.0);
    add_body(topo.bodies, "knuckle", 6.5);
    add_body(topo.bodies, "tie_rod", 1.8);
    const auto& h = topo.hardpoints;
    add_ball(topo.joints, h, "lca_inner_front", "chassis", "lca", "lca.chassis_front");
    add_ball(topo.joints, h, "lca_inner_rear",  "chassis", "lca", "lca.chassis_rear");
    add_ball(topo.joints, h, "lca_outer",       "lca",     "knuckle", "lca.knuckle");
    add_ball(topo.joints, h, "tie_inner",       "chassis", "tie_rod", "tie_rod.rack");
    add_ball(topo.joints, h, "tie_outer",       "tie_rod", "knuckle", "tie_rod.knuckle");
    {
        Joint j;
        j.id = "strut_upper";
        j.type = JointType::Cylindrical;
        j.body_a_id = "chassis";
        j.body_b_id = "strut";
        j.position_in_a = hp_pos(h, "strut.top");
        j.position_in_b = hp_pos(h, "strut.bottom");
        if (h.count("strut.top") && h.count("strut.bottom"))
            j.axis_in_a = (hp_pos(h, "strut.bottom") - hp_pos(h, "strut.top")).normalized();
        topo.joints.push_back(j);
    }
    add_ball(topo.joints, h, "strut_knuckle", "strut", "knuckle", "strut.bottom");
}

void build_double_wishbone(SuspensionTopology& topo) {
    add_body(topo.bodies, "chassis", 0.0);
    add_body(topo.bodies, "lca", 7.0);
    add_body(topo.bodies, "uca", 4.5);
    add_body(topo.bodies, "knuckle", 6.5);
    add_body(topo.bodies, "tie_rod", 1.8);
    const auto& h = topo.hardpoints;
    add_ball(topo.joints, h, "lca_inner_front", "chassis", "lca", "lca.chassis_front");
    add_ball(topo.joints, h, "lca_inner_rear",  "chassis", "lca", "lca.chassis_rear");
    add_ball(topo.joints, h, "lca_outer",       "lca",     "knuckle", "lca.knuckle");
    add_ball(topo.joints, h, "uca_inner_front", "chassis", "uca", "uca.chassis_front");
    add_ball(topo.joints, h, "uca_inner_rear",  "chassis", "uca", "uca.chassis_rear");
    add_ball(topo.joints, h, "uca_outer",       "uca",     "knuckle", "uca.knuckle");
    add_ball(topo.joints, h, "tie_inner",       "chassis", "tie_rod", "tie_rod.rack");
    add_ball(topo.joints, h, "tie_outer",       "tie_rod", "knuckle", "tie_rod.knuckle");
}

void build_trailing_arm(SuspensionTopology& topo) {
    add_body(topo.bodies, "chassis", 0.0);
    add_body(topo.bodies, "arm", 9.0);
    add_body(topo.bodies, "knuckle", 6.0);
    const auto& h = topo.hardpoints;
    add_ball(topo.joints, h, "arm_inboard",  "chassis", "arm", "arm_pivot.chassis_inboard");
    add_ball(topo.joints, h, "arm_outboard", "chassis", "arm", "arm_pivot.chassis_outboard");
    add_ball(topo.joints, h, "wheel_center", "arm", "knuckle", "wheel.center");
}

void build_multi_link5(SuspensionTopology& topo) {
    add_body(topo.bodies, "chassis", 0.0);
    add_body(topo.bodies, "knuckle", 6.5);
    for (const char* link : {"upper_fore", "upper_aft", "lower_fore", "lower_aft", "toe_link"})
        add_body(topo.bodies, link, 3.0);
    const auto& h = topo.hardpoints;
    for (const char* link : {"upper_fore", "upper_aft", "lower_fore", "lower_aft", "toe_link"}) {
        const std::string base = std::string("links.") + link;
        add_ball(topo.joints, h, std::string(link) + "_chassis", "chassis", link,
                 base + ".chassis");
        add_ball(topo.joints, h, std::string(link) + "_knuckle", link, "knuckle",
                 base + ".knuckle");
    }
}

void infer_graph(SuspensionTopology& topo) {
    if (!topo.bodies.empty() && !topo.joints.empty()) return;
    topo.bodies.clear();
    topo.joints.clear();
    switch (topo.kind) {
        case TopologyKind::MacPherson:       build_macpherson(topo); break;
        case TopologyKind::DoubleWishbone:   build_double_wishbone(topo); break;
        case TopologyKind::TrailingArm:      build_trailing_arm(topo); break;
        case TopologyKind::MultiLink5:     build_multi_link5(topo); break;
        case TopologyKind::BeamAxle:
        case TopologyKind::TwistBeam:
        case TopologyKind::DeDion:
            add_body(topo.bodies, "chassis", 0.0);
            add_body(topo.bodies, "axle", 25.0);
            break;
    }
}

}  // namespace

void populate_topology_from_yaml(SuspensionTopology& topo, const YAML::Node& root) {
    topo.hardpoints.clear();
    topo.bodies.clear();
    topo.joints.clear();
    topo.bushings.clear();

    parse_explicit_hardpoints(root, topo.hardpoints);
    if (topo.hardpoints.empty())
        walk_kin_hardpoints(root, "", topo.hardpoints);

    parse_explicit_bodies(root, topo.bodies);
    parse_explicit_joints(root, topo.joints);
    parse_explicit_bushings(root, topo.bushings);
    infer_graph(topo);
    ensure_default_bushings(topo);
}

int topology_hardpoint_count(const SuspensionTopology& topo) {
    return static_cast<int>(topo.hardpoints.size());
}

}  // namespace vdsim::mb
