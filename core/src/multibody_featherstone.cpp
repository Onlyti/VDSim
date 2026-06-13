#include "vdsim/multibody.hpp"

#include "vdsim/multibody_math.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <cmath>
#include <stdexcept>

namespace vdsim::mb {

namespace {

Vec3 yaml_vec3(const YAML::Node& n) {
    return Vec3(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

struct LinkKinematics {
    Mat3 R {Mat3::Identity()};
    Vec3 joint_world {Vec3::Zero()};
    Vec3 com_world {Vec3::Zero()};
    Vec3 omega {Vec3::Zero()};
    Vec3 com_vel {Vec3::Zero()};
    Vec3 omega_dot {Vec3::Zero()};
    Vec3 com_acc {Vec3::Zero()};
    Vec3 axis_world {Vec3::UnitY()};
};

void forward_kinematics(const LinkTreeModel& model, const Eigen::VectorXd& q,
                        const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd,
                        std::vector<LinkKinematics>& kin) {
    const int n = model.num_dof();
    kin.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const auto& link = model.links[static_cast<std::size_t>(i)];
        LinkKinematics& k = kin[static_cast<std::size_t>(i)];
        const Vec3 axis_p = link.axis_in_parent.normalized();
        if (link.parent < 0) {
            k.R = rodrigues(axis_p, q(i));
            k.joint_world = link.r_joint_in_parent;
            k.axis_world = axis_p;
            k.omega = axis_p * qd(i);
            k.omega_dot = axis_p * qdd(i);
        } else {
            const auto& kp = kin[static_cast<std::size_t>(link.parent)];
            const Vec3 axis_in_parent = kp.R * axis_p;
            k.R = kp.R * rodrigues(axis_p, q(i));
            k.joint_world = kp.joint_world + kp.R * link.r_joint_in_parent;
            k.axis_world = axis_in_parent;
            k.omega = kp.omega + axis_in_parent * qd(i);
            k.omega_dot = kp.omega_dot + axis_in_parent * qdd(i);
        }
        const Vec3 r_com = k.R * link.com_in_child;
        k.com_world = k.joint_world + r_com;
        if (link.parent < 0) {
            k.com_vel = k.omega.cross(r_com);
            k.com_acc = k.omega_dot.cross(r_com) + k.omega.cross(k.omega.cross(r_com));
        } else {
            const auto& parent = kin[static_cast<std::size_t>(link.parent)];
            const Vec3 r_jp = k.joint_world - parent.com_world;
            const Vec3 v_joint = parent.com_vel + parent.omega.cross(r_jp);
            k.com_vel = v_joint + k.omega.cross(r_com);
            const Vec3 v_joint_dot = parent.com_acc + parent.omega_dot.cross(r_jp)
                                   + parent.omega.cross(parent.omega.cross(r_jp));
            k.com_acc = v_joint_dot + k.omega_dot.cross(r_com)
                      + k.omega.cross(k.omega.cross(r_com));
        }
    }
}

Eigen::VectorXd inverse_dynamics(const LinkTreeModel& model,
                                 const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& qd,
                                 const Eigen::VectorXd& qdd,
                                 const std::vector<LinkTreeExternalLoad>& ext,
                                 const Vec3& gravity_world) {
    const int n = model.num_dof();
    std::vector<LinkKinematics> kin;
    forward_kinematics(model, q, qd, qdd, kin);
    std::vector<Vec3> f_world(static_cast<std::size_t>(n), Vec3::Zero());
    std::vector<Vec3> n_world(static_cast<std::size_t>(n), Vec3::Zero());
    for (int i = 0; i < n; ++i) {
        const auto& link = model.links[static_cast<std::size_t>(i)];
        const auto& k = kin[static_cast<std::size_t>(i)];
        Vec3 f = link.mass * (k.com_acc - gravity_world);
        const Mat3 I_world = k.R * link.inertia_com * k.R.transpose();
        Vec3 n = I_world * k.omega_dot + k.omega.cross(I_world * k.omega);
        if (static_cast<std::size_t>(i) < ext.size()) {
            const Vec3 r = ext[static_cast<std::size_t>(i)].point_world - k.com_world;
            f += ext[static_cast<std::size_t>(i)].force_world;
            n += r.cross(ext[static_cast<std::size_t>(i)].force_world);
        }
        const Vec3 r_jc = k.com_world - k.joint_world;
        f_world[static_cast<std::size_t>(i)] = f;
        n_world[static_cast<std::size_t>(i)] = n + r_jc.cross(f);
    }
    for (int i = n - 1; i >= 0; --i) {
        const auto& link = model.links[static_cast<std::size_t>(i)];
        if (link.parent >= 0) {
            const auto& k = kin[static_cast<std::size_t>(i)];
            const auto& kp = kin[static_cast<std::size_t>(link.parent)];
            const Vec3 r = k.joint_world - kp.joint_world;
            f_world[static_cast<std::size_t>(link.parent)] += f_world[static_cast<std::size_t>(i)];
            n_world[static_cast<std::size_t>(link.parent)] +=
                n_world[static_cast<std::size_t>(i)] + r.cross(f_world[static_cast<std::size_t>(i)]);
        }
    }
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i)
        tau(i) = n_world[static_cast<std::size_t>(i)].dot(kin[static_cast<std::size_t>(i)].axis_world);
    return tau;
}

struct AxisPivot {
    Vec3 axis;
    Vec3 pivot;
};

AxisPivot revolute_axis_pivot(TopologyKind kind, const YAML::Node& root) {
    switch (kind) {
    case TopologyKind::TrailingArm: {
        const Vec3 pivot = yaml_vec3(root["arm_pivot"]["chassis_inboard"]);
        const Vec3 pivot_out = yaml_vec3(root["arm_pivot"]["chassis_outboard"]);
        return {(pivot_out - pivot).normalized(), pivot};
    }
    case TopologyKind::MacPherson:
    case TopologyKind::DoubleWishbone: {
        const Vec3 cf = yaml_vec3(root["lca"]["chassis_front"]);
        const Vec3 cr = yaml_vec3(root["lca"]["chassis_rear"]);
        return {(cr - cf).normalized(), cf};
    }
    case TopologyKind::MultiLink5: {
        const Vec3 pivot = yaml_vec3(root["links"]["lower_aft"]["chassis"]);
        return {Vec3::UnitY(), pivot};
    }
    default:
        throw std::runtime_error("mb: Featherstone tree unsupported topology kind");
    }
}

RevoluteLink lumped_revolute_link(const SuspensionTopology& topo, const Vec3& axis,
                                  const Vec3& pivot) {
    const Vec3 axis_u = axis.normalized();
    const LumpedCorner lump = lump_corner_about_axis(topo.bodies, axis_u, pivot);
    RevoluteLink link;
    link.parent = -1;
    link.axis_in_parent = axis_u;
    link.r_joint_in_parent = pivot;
    link.mass = lump.mass;
    link.com_in_child = lump.com - pivot;
    link.inertia_com = Mat3::Identity() * lump.i_com;
    return link;
}

}  // namespace

LinkTreeModel build_revolute_tree(const SuspensionTopology& topo) {
    if (topo.kin_yaml_path.empty())
        throw std::runtime_error("mb: link tree requires kin_yaml_path");
    YAML::Node root = YAML::LoadFile(topo.kin_yaml_path);
    const AxisPivot ap = revolute_axis_pivot(topo.kind, root);
    LinkTreeModel model;
    model.links.push_back(lumped_revolute_link(topo, ap.axis, ap.pivot));
    return model;
}

Eigen::VectorXd rnea_revolute_tree(const LinkTreeModel& model,
                                   const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& qd,
                                   const Eigen::VectorXd& qdd,
                                   const std::vector<LinkTreeExternalLoad>& ext,
                                   const Vec3& gravity_world) {
    return inverse_dynamics(model, q, qd, qdd, ext, gravity_world);
}

Eigen::VectorXd forward_dynamics_revolute_tree(
    const LinkTreeModel& model, const Eigen::VectorXd& q, const Eigen::VectorXd& qd,
    const Eigen::VectorXd& tau, const std::vector<LinkTreeExternalLoad>& ext,
    const Vec3& gravity_world) {
    const int n = model.num_dof();
    Eigen::VectorXd qdd0 = Eigen::VectorXd::Zero(n);
    const Eigen::VectorXd bias =
        inverse_dynamics(model, q, qd, qdd0, ext, gravity_world);
    Eigen::MatrixXd M(n, n);
    for (int j = 0; j < n; ++j) {
        Eigen::VectorXd unit = Eigen::VectorXd::Zero(n);
        unit(j) = 1.0;
        M.col(j) = inverse_dynamics(model, q, qd, unit, ext, gravity_world) - bias;
    }
    return M.ldlt().solve(tau - bias);
}

void step_revolute_link_tree(const LinkTreeModel& model, LinkTreeState& st,
                             const Eigen::VectorXd& tau,
                             const std::vector<LinkTreeExternalLoad>& ext,
                             double dt, const Vec3& gravity_world) {
    if (dt <= 0.0) return;
    const Eigen::VectorXd qdd =
        forward_dynamics_revolute_tree(model, st.q, st.qd, tau, ext, gravity_world);
    st.qd += dt * qdd;
    st.q += dt * st.qd;
}

}  // namespace vdsim::mb
