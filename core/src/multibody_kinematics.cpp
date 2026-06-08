#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/suspension.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <unordered_map>

namespace vdsim::mb {

void populate_topology_from_yaml(SuspensionTopology& topo, const YAML::Node& root);

namespace {

TopologyKind kind_from_type(const std::string& type) {
    if (type == "macpherson" || type == "MacPherson") return TopologyKind::MacPherson;
    if (type == "double_wishbone" || type == "dw") return TopologyKind::DoubleWishbone;
    if (type == "5link" || type == "five_link" || type == "multi_link_5")
        return TopologyKind::MultiLink5;
    if (type == "trailing_arm" || type == "ta") return TopologyKind::TrailingArm;
    if (type == "beam_axle") return TopologyKind::BeamAxle;
    if (type == "twist_beam") return TopologyKind::TwistBeam;
    throw std::runtime_error("mb: unknown suspension type '" + type + "'");
}

class KinematicSolver final : public IMultibodySolver {
public:
    WheelPose forward_kinematics(SuspensionTopology& topo,
                                 double travel_z,
                                 double steer_rad) const override {
        const auto o = kin(topo).compute(travel_z, steer_rad);
        WheelPose wp;
        wp.toe_rad    = o.toe;
        wp.camber_rad = o.camber;
        wp.caster_rad = o.caster;
        topo.compliance_toe_deg = 0.0;
        topo.compliance_camber_deg = 0.0;
        topo.toe_deg     = o.toe * 180.0 / M_PI;
        topo.camber_deg  = o.camber * 180.0 / M_PI;
        topo.caster_deg  = o.caster * 180.0 / M_PI;
        return wp;
    }

    void quasi_static_compliance(SuspensionTopology& topo,
                                 const WheelLoad& load) const override {
        solve_quasi_static_compliance(topo, load);
    }

    void step_dynamics(SuspensionTopology& topo,
                       CornerDynamicsState& state,
                       const WheelLoad& load,
                       double dt) const override {
        step_corner_dynamics(state, topo, load, dt);
    }

private:
    ISuspensionKinematics& kin(const SuspensionTopology& topo) const {
        if (topo.kin_yaml_path.empty())
            throw std::runtime_error("mb: SuspensionTopology missing kin_yaml_path");
        auto& slot = cache_[topo.kin_yaml_path];
        if (!slot) slot = create_native_kinematics_from_yaml(topo.kin_yaml_path);
        return *slot;
    }

    mutable std::unordered_map<std::string, std::unique_ptr<ISuspensionKinematics>> cache_;
};

}  // namespace

SuspensionTopology SuspensionTopology::from_yaml(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    const std::string type = root["type"] ? root["type"].as<std::string>() : "";
    if (type.empty())
        throw std::runtime_error("mb: kin yaml missing 'type': " + path);
    SuspensionTopology topo;
    topo.kind = kind_from_type(type);
    if (root["kin_ref"])
        topo.kin_yaml_path = root["kin_ref"].as<std::string>();
    else
        topo.kin_yaml_path = path;
    if (root["label"])
        topo.name = root["label"].as<std::string>();
    else if (root["id"])
        topo.name = root["id"].as<std::string>();
    else
        topo.name = type;
    populate_topology_from_yaml(topo, root);
    return topo;
}

void SuspensionTopology::to_yaml(const std::string& path) const {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "label" << YAML::Value << name;
    out << YAML::Key << "type" << YAML::Value
        << (kind == TopologyKind::MacPherson       ? "macpherson"
            : kind == TopologyKind::DoubleWishbone ? "double_wishbone"
            : kind == TopologyKind::MultiLink5     ? "five_link"
            : kind == TopologyKind::TrailingArm    ? "trailing_arm"
            : kind == TopologyKind::BeamAxle       ? "beam_axle"
            : kind == TopologyKind::TwistBeam      ? "twist_beam"
                                                   : "unknown");
    if (!kin_yaml_path.empty())
        out << YAML::Key << "kin_ref" << YAML::Value << kin_yaml_path;
    if (!hardpoints.empty()) {
        out << YAML::Key << "hardpoints" << YAML::Value << YAML::BeginSeq;
        for (const auto& kv : hardpoints) {
            out << YAML::BeginMap;
            out << YAML::Key << "name" << YAML::Value << kv.second.name;
            out << YAML::Key << "body_id" << YAML::Value << kv.second.body_id;
            out << YAML::Key << "position" << YAML::Value << YAML::Flow
                << YAML::BeginSeq << kv.second.position.x()
                << kv.second.position.y() << kv.second.position.z() << YAML::EndSeq;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
    }
    if (!bodies.empty()) {
        out << YAML::Key << "bodies" << YAML::Value << YAML::BeginSeq;
        for (const auto& b : bodies) {
            out << YAML::BeginMap;
            out << YAML::Key << "id" << YAML::Value << b.id;
            out << YAML::Key << "mass" << YAML::Value << b.mass;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
    }
    if (!joints.empty()) {
        out << YAML::Key << "joints" << YAML::Value << YAML::BeginSeq;
        for (const auto& j : joints) {
            const char* jt =
                j.type == JointType::Ball        ? "ball"
                : j.type == JointType::Revolute  ? "revolute"
                : j.type == JointType::Cylindrical ? "cylindrical"
                : j.type == JointType::Prismatic ? "prismatic"
                : j.type == JointType::Universal ? "universal"
                : "rigid";
            out << YAML::BeginMap;
            out << YAML::Key << "id" << YAML::Value << j.id;
            out << YAML::Key << "type" << YAML::Value << jt;
            out << YAML::Key << "body_a" << YAML::Value << j.body_a_id;
            out << YAML::Key << "body_b" << YAML::Value << j.body_b_id;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
    }
    out << YAML::EndMap;
    std::ofstream f(path);
    if (!f) throw std::runtime_error("mb: cannot write " + path);
    f << out.c_str();
}

std::unique_ptr<IMultibodySolver> create_kinematic_solver() {
    return std::make_unique<KinematicSolver>();
}

bool attach_topology_front(IVehicleDynamics& dyn, const SuspensionTopology& topo) {
    if (!attach_front_kinematics(dyn, create_native_kinematics_from_yaml(topo.kin_yaml_path)))
        return false;
    const bool dyn_en = (dyn.level() == IVehicleDynamics::Level::L4_Kinematic);
    return fourteen_dof_attach_multibody(dyn, true, topo, dyn_en);
}

bool attach_topology_rear(IVehicleDynamics& dyn, const SuspensionTopology& topo) {
    if (!attach_rear_kinematics(dyn, create_native_kinematics_from_yaml(topo.kin_yaml_path)))
        return false;
    const bool dyn_en = (dyn.level() == IVehicleDynamics::Level::L4_Kinematic);
    return fourteen_dof_attach_multibody(dyn, false, topo, dyn_en);
}

}  // namespace vdsim::mb
