// Compile-only smoke test for vdsim/multibody.hpp (M0 stub).

#include "vdsim/multibody.hpp"

#include <gtest/gtest.h>

TEST(MultibodyHeader, BasicTypesInstantiate) {
    vdsim::mb::RigidBody body;
    body.id = "test_body";
    body.mass = 10.0;
    EXPECT_EQ(body.id, "test_body");
    EXPECT_DOUBLE_EQ(body.mass, 10.0);

    vdsim::mb::Joint joint;
    joint.type = vdsim::mb::JointType::Ball;
    joint.position_in_a = vdsim::Vec3(0.1, 0.2, 0.3);
    EXPECT_EQ(joint.type, vdsim::mb::JointType::Ball);

    vdsim::mb::Bushing bushing;
    bushing.k_translation = vdsim::Vec3(1e6, 1e6, 1e6);
    EXPECT_DOUBLE_EQ(bushing.k_translation.x(), 1e6);
}

TEST(MultibodyHeader, SuspensionTopologyDiagOutputsDefaultZero) {
    vdsim::mb::SuspensionTopology topo;
    topo.kind = vdsim::mb::TopologyKind::MacPherson;
    topo.name = "test";
    EXPECT_DOUBLE_EQ(topo.toe_deg, 0.0);
    EXPECT_DOUBLE_EQ(topo.camber_deg, 0.0);
    EXPECT_DOUBLE_EQ(topo.scrub_radius_mm, 0.0);
}

TEST(MultibodyHeader, AllTopologyKindsEnumerable) {
    // sanity: each enum value is distinct.
    using K = vdsim::mb::TopologyKind;
    EXPECT_NE((int)K::MacPherson, (int)K::DoubleWishbone);
    EXPECT_NE((int)K::DoubleWishbone, (int)K::MultiLink5);
    EXPECT_NE((int)K::TrailingArm, (int)K::BeamAxle);
    EXPECT_NE((int)K::BeamAxle, (int)K::DeDion);
    EXPECT_NE((int)K::TwistBeam, (int)K::MacPherson);
}
