#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/suspension.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

#ifndef VDSIM_SOURCE_DIR
#define VDSIM_SOURCE_DIR "."
#endif

TEST(MultibodyKinematics, FromYamlMacPherson) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    ASSERT_TRUE(std::filesystem::exists(path));
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    EXPECT_EQ(topo.kind, vdsim::mb::TopologyKind::MacPherson);
    EXPECT_EQ(topo.kin_yaml_path, path);
}

TEST(MultibodyKinematics, ForwardKinematicsFinite) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    auto solver = vdsim::mb::create_kinematic_solver();
    const auto pose = solver->forward_kinematics(topo, 0.0, 0.0);
    EXPECT_TRUE(std::isfinite(pose.camber_rad));
    EXPECT_TRUE(std::isfinite(pose.toe_rad));
    EXPECT_TRUE(std::isfinite(topo.camber_deg));
}

TEST(MultibodyTopology, MacPhersonGraph) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    EXPECT_GE(vdsim::mb::topology_hardpoint_count(topo), 8);
    EXPECT_GE(topo.bodies.size(), 4u);
    EXPECT_GE(topo.joints.size(), 4u);
    EXPECT_TRUE(topo.hardpoints.count("lca.knuckle"));
    EXPECT_TRUE(topo.hardpoints.count("wheel.center"));
    EXPECT_DOUBLE_EQ(topo.hardpoints.at("wheel.center").position.y(), 0.775);
}

// Re-taxonomy step 2: an optional top-level `knuckle:` block defines knuckle
// attachment points and the steering knuckle-arm point relative to the wheel
// centre; the parser resolves each to an absolute body-frame hardpoint
// (wheel.center + offset). Legacy kin files omit it, so the topology is unchanged
// (the MacPhersonGraph test above still passes bit-identically).
TEST(MultibodyTopology, KnuckleBlockWheelCenterRelative) {
    const auto out = std::filesystem::temp_directory_path() / "vdsim_knuckle_kin.yaml";
    {
        std::ofstream f(out);
        f << "type: macpherson\n"
             "side: left\n"
             "wheel:\n"
             "  center: [0.0, 0.8, 0.3]\n"
             "  spin_axis: [0.0, 1.0, 0.0]\n"
             "lca:\n"
             "  chassis_front: [0.1, 0.32, 0.18]\n"
             "  chassis_rear: [-0.2, 0.32, 0.18]\n"
             "  knuckle: [-0.02, 0.74, 0.2]\n"
             "strut:\n"
             "  top: [-0.04, 0.6, 0.65]\n"
             "  bottom: [-0.04, 0.72, 0.31]\n"
             "tie_rod:\n"
             "  rack: [-0.3, 0.4, 0.22]\n"
             "  knuckle: [-0.3, 0.73, 0.25]\n"
             "knuckle:\n"
             "  ref: wheel_center\n"
             "  points:\n"
             "    lca: [-0.02, -0.06, -0.10]\n"
             "    tie_rod: [-0.30, -0.07, -0.05]\n"
             "  arm: [-0.12, -0.05, 0.02]\n";
    }
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(out.string());
    ASSERT_TRUE(topo.hardpoints.count("knuckle.lca"));
    ASSERT_TRUE(topo.hardpoints.count("knuckle.arm"));
    // Resolved = wheel.center + offset.
    const auto& kl = topo.hardpoints.at("knuckle.lca").position;
    EXPECT_DOUBLE_EQ(kl.x(), 0.0 + (-0.02));
    EXPECT_DOUBLE_EQ(kl.y(), 0.8 + (-0.06));
    EXPECT_DOUBLE_EQ(kl.z(), 0.3 + (-0.10));
    const auto& ka = topo.hardpoints.at("knuckle.arm").position;
    EXPECT_DOUBLE_EQ(ka.x(), 0.0 + (-0.12));
    EXPECT_DOUBLE_EQ(ka.y(), 0.8 + (-0.05));
    EXPECT_DOUBLE_EQ(ka.z(), 0.3 + 0.02);
    EXPECT_EQ(topo.hardpoints.at("knuckle.arm").body_id, "knuckle");
    // The legacy absolute knuckle fields are still parsed (dynamics unchanged).
    EXPECT_TRUE(topo.hardpoints.count("lca.knuckle"));
    std::filesystem::remove(out);
}

TEST(MultibodyTopology, FiveLinkKindAlias) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/5link_rear_sports.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    EXPECT_EQ(topo.kind, vdsim::mb::TopologyKind::MultiLink5);
    EXPECT_GE(topo.hardpoints.size(), 10u);
    EXPECT_GE(topo.bodies.size(), 6u);
}

TEST(MultibodyTopology, ToYamlRoundtrip) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml";
    auto topo1 = vdsim::mb::SuspensionTopology::from_yaml(path);
    const auto out = std::filesystem::temp_directory_path() / "vdsim_topo_rt.yaml";
    topo1.to_yaml(out.string());
    auto topo2 = vdsim::mb::SuspensionTopology::from_yaml(out.string());
    EXPECT_EQ(topo2.kind, topo1.kind);
    EXPECT_EQ(topo2.hardpoints.size(), topo1.hardpoints.size());
    EXPECT_EQ(topo2.bodies.size(), topo1.bodies.size());
    EXPECT_EQ(topo2.joints.size(), topo1.joints.size());
    std::filesystem::remove(out);
}

TEST(MultibodyCompliance, ZeroLoadNoDeflection) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    auto solver = vdsim::mb::create_kinematic_solver();
    solver->forward_kinematics(topo, 0.0, 0.0);
    const double toe0 = topo.toe_deg;
    vdsim::mb::WheelLoad load;
    solver->quasi_static_compliance(topo, load);
    EXPECT_NEAR(topo.compliance_toe_deg, 0.0, 1e-12);
    EXPECT_NEAR(topo.compliance_camber_deg, 0.0, 1e-12);
    EXPECT_DOUBLE_EQ(topo.toe_deg, toe0);
    EXPECT_FALSE(topo.bushings.empty());
}

TEST(MultibodyCompliance, LateralLoadComplianceSteer) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    auto solver = vdsim::mb::create_kinematic_solver();
    solver->forward_kinematics(topo, 0.0, 0.0);
    const double toe0 = topo.toe_deg;
    vdsim::mb::WheelLoad load;
    load.force_world.y() = 2500.0;
    load.force_world.z() = 4500.0;
    solver->quasi_static_compliance(topo, load);
    EXPECT_GT(std::abs(topo.compliance_toe_deg), 0.005);
    EXPECT_GT(std::abs(topo.toe_deg - toe0), 0.005);
    EXPECT_GT(std::abs(topo.compliance_camber_deg), 0.001);
}

TEST(MultibodyCompliance, StifferBushingLessSteer) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    auto soft = vdsim::mb::SuspensionTopology::from_yaml(path);
    auto stiff = vdsim::mb::SuspensionTopology::from_yaml(path);
    for (auto& b : soft.bushings)
        b.k_rotation = vdsim::Vec3(5.0e3, 5.0e3, 5.0e3);
    for (auto& b : stiff.bushings)
        b.k_rotation = vdsim::Vec3(5.0e5, 5.0e5, 5.0e5);
    auto solver = vdsim::mb::create_kinematic_solver();
    solver->forward_kinematics(soft, 0.0, 0.0);
    solver->forward_kinematics(stiff, 0.0, 0.0);
    vdsim::mb::WheelLoad load;
    load.force_world.y() = 3000.0;
    load.force_world.z() = 5000.0;
    solver->quasi_static_compliance(soft, load);
    solver->quasi_static_compliance(stiff, load);
    EXPECT_GT(std::abs(soft.compliance_toe_deg), std::abs(stiff.compliance_toe_deg));
}

TEST(HardJointDae, TrailingArmTracksPrescribedTravel) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/ta_rear_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    auto model = vdsim::mb::create_hard_joint_dae_model(topo);
    vdsim::mb::HardJointCornerState st;
    vdsim::mb::PrescribedCornerMotion mot;
    mot.travel_z = 0.02;
    model->initialize(st, mot);
    vdsim::mb::WheelLoad zl;
    for (int i = 0; i < 100; ++i)
        vdsim::mb::step_hard_joint_dae(st, *model, topo, mot, zl, 0.005);
    EXPECT_NEAR(topo.compliance_toe_deg, 0.0, 1e-12);
    EXPECT_TRUE(std::isfinite(st.q));
    EXPECT_TRUE(std::isfinite(st.qd));
}

TEST(HardJointDae, MacPhersonMatchesNativeKinematics) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    auto model = vdsim::mb::create_hard_joint_dae_model(topo);
    auto kin = vdsim::create_native_kinematics_from_yaml(path);
    vdsim::mb::HardJointCornerState st;
    vdsim::mb::PrescribedCornerMotion mot;
    mot.travel_z = -0.015;
    mot.steer_rack_dy = 0.004;
    model->initialize(st, mot);
    vdsim::mb::WheelLoad zl;
    const auto wp = model->step(st, mot, zl, 0.0);
    const auto o = kin->compute(mot.travel_z, mot.steer_rack_dy);
    EXPECT_NEAR(wp.toe_rad, o.toe, 2e-3);
    EXPECT_NEAR(wp.camber_rad, o.camber, 2e-3);
}

TEST(HardJointDae, DoubleWishboneMatchesNativeKinematics) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    auto model = vdsim::mb::create_hard_joint_dae_model(topo);
    auto kin = vdsim::create_native_kinematics_from_yaml(path);
    vdsim::mb::HardJointCornerState st;
    vdsim::mb::PrescribedCornerMotion mot;
    mot.travel_z = -0.02;
    mot.steer_rack_dy = 0.003;
    model->initialize(st, mot);
    vdsim::mb::WheelLoad zl;
    const auto wp = model->step(st, mot, zl, 0.0);
    const auto o = kin->compute(mot.travel_z, mot.steer_rack_dy);
    EXPECT_NEAR(wp.toe_rad, o.toe, 3e-3);
    EXPECT_NEAR(wp.camber_rad, o.camber, 3e-3);
}

TEST(HardJointDae, FiveLinkMatchesNativeKinematics) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/5link_rear_sports.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    auto model = vdsim::mb::create_hard_joint_dae_model(topo);
    auto kin = vdsim::create_native_kinematics_from_yaml(path);
    vdsim::mb::HardJointCornerState st;
    vdsim::mb::PrescribedCornerMotion mot;
    mot.travel_z = 0.018;
    model->initialize(st, mot);
    vdsim::mb::WheelLoad zl;
    const auto wp = model->step(st, mot, zl, 0.0);
    const auto o = kin->compute(mot.travel_z, 0.0);
    EXPECT_NEAR(wp.toe_rad, o.toe, 3e-3);
    EXPECT_NEAR(wp.camber_rad, o.camber, 3e-3);
}

TEST(HardJointDae, DoubleWishboneLateralLoadNoBushingCompliance) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    auto model = vdsim::mb::create_hard_joint_dae_model(topo);
    vdsim::mb::HardJointCornerState st0, st1;
    vdsim::mb::PrescribedCornerMotion mot;
    model->initialize(st0, mot);
    model->initialize(st1, mot);
    vdsim::mb::WheelLoad load;
    load.force_world.y() = 3500.0;
    load.force_world.z() = 4800.0;
    vdsim::mb::WheelLoad zl;
    const auto wp0 = model->step(st0, mot, zl, 0.0);
    for (int i = 0; i < 400; ++i)
        vdsim::mb::step_hard_joint_dae(st1, *model, topo, mot, load, 0.005);
    const auto wp1 = model->step(st1, mot, zl, 0.0);
    EXPECT_NEAR(wp1.toe_rad, wp0.toe_rad, 8e-3);
    EXPECT_NEAR(topo.compliance_toe_deg, 0.0, 1e-12);
}

TEST(HardJointDae, LateralLoadNoBushingCompliance) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    auto model = vdsim::mb::create_hard_joint_dae_model(topo);
    vdsim::mb::HardJointCornerState st0, st1;
    vdsim::mb::PrescribedCornerMotion mot;
    model->initialize(st0, mot);
    model->initialize(st1, mot);
    vdsim::mb::WheelLoad load;
    load.force_world.y() = 4000.0;
    load.force_world.z() = 5000.0;
    vdsim::mb::WheelLoad zl;
    const auto wp0 = model->step(st0, mot, zl, 0.0);
    for (int i = 0; i < 400; ++i)
        vdsim::mb::step_hard_joint_dae(st1, *model, topo, mot, load, 0.005);
    const auto wp1 = model->step(st1, mot, zl, 0.0);
    EXPECT_NEAR(wp1.toe_rad, wp0.toe_rad, 5e-3);
    EXPECT_NEAR(topo.compliance_toe_deg, 0.0, 1e-12);
}

TEST(MultibodyKcSweep, MacPhersonTravelCurvesFinite) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    const auto r = vdsim::mb::run_kc_sweep(topo);
    ASSERT_EQ(r.travel.size(), 41u);
    ASSERT_EQ(r.steer.size(), 17u);
    ASSERT_EQ(r.compliance_fy.size(), 9u);
    EXPECT_TRUE(std::isfinite(r.travel.front().camber_deg));
    EXPECT_TRUE(std::isfinite(r.travel.back().toe_deg));
    EXPECT_NE(r.travel.front().abscissa, r.travel.back().abscissa);
}

TEST(MultibodyKcXcheck, IdenticalYamlPasses) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml";
    const auto rep = vdsim::mb::run_kc_xcheck(path, path, 0.05);
    EXPECT_TRUE(rep.all_ok);
    EXPECT_EQ(rep.deltas.size(), 5u);
}

TEST(MultibodyKcXcheck, AdamsImportedSampleMatchesReference) {
    const std::string ref = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml";
    const std::string cand = std::string(VDSIM_SOURCE_DIR)
        + "/tests/fixtures/adams_dw_imported.yaml";
    const auto rep = vdsim::mb::run_kc_xcheck(ref, cand, 0.02);
    EXPECT_TRUE(rep.all_ok);
    EXPECT_GT(std::abs(rep.deltas[1].reference), 0.01);
}

TEST(MultibodyKcXcheck, PerturbedHardpointFails) {
    const std::string ref = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml";
    const std::string perturbed = std::string(VDSIM_SOURCE_DIR)
        + "/tests/fixtures/adams_dw_perturbed.yaml";
    const auto rep = vdsim::mb::run_kc_xcheck(ref, perturbed, 0.05);
    EXPECT_FALSE(rep.all_ok);
}

TEST(MultibodyKcSweep, ComplianceToeVsFy) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    const auto r = vdsim::mb::run_kc_sweep(topo);
    const double toe_lo = r.compliance_fy.front().compliance_toe_deg;
    const double toe_hi = r.compliance_fy.back().compliance_toe_deg;
    EXPECT_GT(std::abs(toe_hi - toe_lo), 0.01);
}

TEST(MultibodyKinematics, AttachTopologyFront) {
    const std::string path = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(path);
    auto dyn = vdsim::create_fourteen_dof_kinematic();
    vdsim::VehicleParams vp;
    vdsim::TireParams tp;
    vdsim::SolverParams sp;
    dyn->initialize(vp, tp, sp);
    EXPECT_TRUE(vdsim::mb::attach_topology_front(*dyn, topo));
}
