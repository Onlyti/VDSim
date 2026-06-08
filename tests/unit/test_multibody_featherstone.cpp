#include "vdsim/multibody.hpp"

#include <gtest/gtest.h>

#include <cmath>

#ifndef VDSIM_SOURCE_DIR
#define VDSIM_SOURCE_DIR "."
#endif

namespace {

vdsim::mb::LinkTreeModel single_pendulum_model() {
    vdsim::mb::RevoluteLink link;
    link.parent = -1;
    link.axis_in_parent = vdsim::Vec3::UnitY();
    link.r_joint_in_parent = vdsim::Vec3::Zero();
    link.mass = 2.0;
    link.com_in_child = vdsim::Vec3(0.5, 0.0, 0.0);
    link.inertia_com = vdsim::Mat3::Identity() * 0.01;
    vdsim::mb::LinkTreeModel model;
    model.links.push_back(link);
    return model;
}

}  // namespace

TEST(LinkTreeFeatherstone, SinglePendulumGravity) {
    const auto model = single_pendulum_model();
    Eigen::VectorXd q(1), qd(1), tau(1);
    q(0) = 0.0;
    qd(0) = 0.0;
    tau(0) = 0.0;
    const Eigen::VectorXd qdd = vdsim::mb::forward_dynamics_revolute_tree(
        model, q, qd, tau, {}, vdsim::Vec3(0, 0, -9.81));
    const double I = 0.01 + 2.0 * 0.5 * 0.5;
    const double expected = 2.0 * 9.81 * 0.5 / I;
    EXPECT_NEAR(qdd(0), expected, 0.5);
}

TEST(LinkTreeFeatherstone, TwoLinkChainNonSingular) {
    vdsim::mb::LinkTreeModel model;
    vdsim::mb::RevoluteLink root;
    root.parent = -1;
    root.axis_in_parent = vdsim::Vec3::UnitY();
    root.r_joint_in_parent = vdsim::Vec3::Zero();
    root.mass = 1.5;
    root.com_in_child = vdsim::Vec3(0.3, 0, 0);
    root.inertia_com = vdsim::Mat3::Identity() * 0.05;
    vdsim::mb::RevoluteLink child;
    child.parent = 0;
    child.axis_in_parent = vdsim::Vec3::UnitY();
    child.r_joint_in_parent = vdsim::Vec3(0.6, 0, 0);
    child.mass = 1.0;
    child.com_in_child = vdsim::Vec3(0.25, 0, 0);
    child.inertia_com = vdsim::Mat3::Identity() * 0.02;
    model.links = {root, child};
    Eigen::VectorXd q(2), qd(2), tau(2);
    q << 0.1, -0.2;
    qd << 0.0, 0.0;
    tau << 0.0, 0.0;
    const Eigen::VectorXd qdd = vdsim::mb::forward_dynamics_revolute_tree(
        model, q, qd, tau, {}, vdsim::Vec3(0, 0, -9.81));
    EXPECT_TRUE(std::isfinite(qdd(0)));
    EXPECT_TRUE(std::isfinite(qdd(1)));
    EXPECT_GT(std::abs(qdd(0)) + std::abs(qdd(1)), 0.1);
}

TEST(LinkTreeFeatherstone, TrailingArmBuildsFromTopology) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/ta_rear_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    const auto model = vdsim::mb::build_revolute_tree(topo);
    EXPECT_EQ(model.num_dof(), 1);
    EXPECT_GT(model.links[0].mass, 1.0);
}

class LinkTreeTopologyBuild : public ::testing::TestWithParam<const char*> {};

TEST_P(LinkTreeTopologyBuild, BuildsOneDofFromKinYaml) {
    const std::string path = std::string(VDSIM_SOURCE_DIR) + GetParam();
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    const auto model = vdsim::mb::build_revolute_tree(topo);
    EXPECT_EQ(model.num_dof(), 1);
    EXPECT_GT(model.links[0].mass, 1.0);
    EXPECT_GT(model.links[0].axis_in_parent.norm(), 0.9);
}

INSTANTIATE_TEST_SUITE_P(
    AllCornerTopologies, LinkTreeTopologyBuild,
    ::testing::Values("/configs/parts/susp_kinematics/kin/ta_rear_sedan.yaml",
                      "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml",
                      "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml",
                      "/configs/parts/susp_kinematics/kin/5link_rear_sports.yaml"));

namespace {

void expect_force_response(const std::string& kin_path, const vdsim::Vec3& force,
                           const vdsim::Vec3& wheel) {
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(kin_path);
    const auto model = vdsim::mb::build_revolute_tree(topo);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(1);
    vdsim::mb::LinkTreeExternalLoad ext;
    ext.force_world = force;
    ext.point_world = wheel;
    const Eigen::VectorXd qdd = vdsim::mb::forward_dynamics_revolute_tree(
        model, q, qd, tau, {ext}, vdsim::Vec3::Zero());
    const vdsim::Vec3 axis = model.links[0].axis_in_parent.normalized();
    const vdsim::Vec3 pivot = model.links[0].r_joint_in_parent;
    const double tau_ext = axis.dot((wheel - pivot).cross(force));
    Eigen::VectorXd unit(1);
    unit(0) = 1.0;
    const Eigen::VectorXd bias = vdsim::mb::rnea_revolute_tree(
        model, q, qd, Eigen::VectorXd::Zero(1), {ext}, vdsim::Vec3::Zero());
    const Eigen::VectorXd col = vdsim::mb::rnea_revolute_tree(
        model, q, qd, unit, {ext}, vdsim::Vec3::Zero()) - bias;
    const double M = col(0);
    EXPECT_NEAR(bias(0), tau_ext, std::abs(tau_ext) * 0.02 + 1e-3);
    EXPECT_NEAR(qdd(0), -bias(0) / M, std::abs(bias(0) / M) * 0.02 + 1e-3);
    EXPECT_GT(std::abs(qdd(0)), 0.1);
}

}  // namespace

TEST(LinkTreeFeatherstone, MacPhersonLateralForceAtWheel) {
    const std::string base = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    expect_force_response(base, vdsim::Vec3(0.0, 4000.0, 0.0),
                          vdsim::Vec3(0.0, 0.775, 0.305));
}

TEST(LinkTreeFeatherstone, DoubleWishboneLateralForceAtWheel) {
    const std::string base = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml";
    expect_force_response(base, vdsim::Vec3(0.0, 3500.0, 0.0),
                          vdsim::Vec3(0.0, 0.79, 0.33));
}

TEST(LinkTreeFeatherstone, FiveLinkLongitudinalForceAtWheel) {
    const std::string base = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/5link_rear_sports.yaml";
    expect_force_response(base, vdsim::Vec3(3000.0, 0.0, 0.0),
                          vdsim::Vec3(-1.35, 0.78, 0.305));
}

TEST(LinkTreeFeatherstone, TrailingArmLateralForceMatchesScalar) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/ta_rear_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    const auto model = vdsim::mb::build_revolute_tree(topo);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(1);
    vdsim::mb::LinkTreeExternalLoad ext;
    ext.force_world.y() = 2500.0;
    ext.point_world = vdsim::Vec3(-1.35, 0.78, 0.305);
    const Eigen::VectorXd qdd = vdsim::mb::forward_dynamics_revolute_tree(
        model, q, qd, tau, {ext}, vdsim::Vec3::Zero());
    const vdsim::Vec3 axis = model.links[0].axis_in_parent.normalized();
    const vdsim::Vec3 pivot = model.links[0].r_joint_in_parent;
    const vdsim::Vec3 r = ext.point_world - pivot;
    const double tau_ext = axis.dot(r.cross(ext.force_world));
    Eigen::VectorXd unit(1);
    unit(0) = 1.0;
    const Eigen::VectorXd bias = vdsim::mb::rnea_revolute_tree(
        model, q, qd, Eigen::VectorXd::Zero(1), {ext}, vdsim::Vec3::Zero());
    const Eigen::VectorXd col = vdsim::mb::rnea_revolute_tree(
        model, q, qd, unit, {ext}, vdsim::Vec3::Zero()) - bias;
    const double M = col(0);
    EXPECT_NEAR(bias(0), tau_ext, std::abs(tau_ext) * 0.02 + 1e-3);
    EXPECT_NEAR(qdd(0), -bias(0) / M, std::abs(bias(0) / M) * 0.02 + 1e-3);
}
