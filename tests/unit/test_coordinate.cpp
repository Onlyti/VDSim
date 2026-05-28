#include "vdsim/coordinate.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {
constexpr double kTol = 1e-9;
}

// =============================================================================
// Euler <-> Quat
// =============================================================================
TEST(Coordinate, QuatFromEulerIdentity) {
    const vdsim::Euler e {0.0, 0.0, 0.0};
    const vdsim::Quat q = vdsim::quat_from_euler(e);
    EXPECT_NEAR(q.w(), 1.0, kTol);
    EXPECT_NEAR(q.x(), 0.0, kTol);
    EXPECT_NEAR(q.y(), 0.0, kTol);
    EXPECT_NEAR(q.z(), 0.0, kTol);
}

TEST(Coordinate, PureYawQuat) {
    const double yaw = M_PI / 4.0;   // 45 deg
    const vdsim::Quat q = vdsim::quat_from_euler({0.0, 0.0, yaw});

    EXPECT_NEAR(vdsim::yaw_from_quat(q), yaw, kTol);
    EXPECT_NEAR(q.w(), std::cos(yaw / 2.0), kTol);
    EXPECT_NEAR(q.z(), std::sin(yaw / 2.0), kTol);
    EXPECT_NEAR(q.x(), 0.0, kTol);
    EXPECT_NEAR(q.y(), 0.0, kTol);
}

TEST(Coordinate, EulerRoundtripSmallAngles) {
    const vdsim::Euler e0 {0.05, 0.03, 0.7};   // typical car attitude
    const vdsim::Quat  q  = vdsim::quat_from_euler(e0);
    const vdsim::Euler e1 = vdsim::euler_from_quat(q);
    EXPECT_NEAR(e0.roll,  e1.roll,  1e-12);
    EXPECT_NEAR(e0.pitch, e1.pitch, 1e-12);
    EXPECT_NEAR(e0.yaw,   e1.yaw,   1e-12);
}

TEST(Coordinate, EulerRoundtripNegativeYaw) {
    const vdsim::Euler e0 {0.0, 0.0, -M_PI / 3.0};
    const vdsim::Quat  q  = vdsim::quat_from_euler(e0);
    const vdsim::Euler e1 = vdsim::euler_from_quat(q);
    EXPECT_NEAR(e1.yaw, e0.yaw, 1e-12);
}

TEST(Coordinate, YawCcwPositive) {
    // X forward rotated +90 deg yaw should map (1,0,0) -> (0,1,0) (left).
    const vdsim::Quat q = vdsim::quat_from_euler({0.0, 0.0, M_PI / 2.0});
    const vdsim::Vec3 x_body {1.0, 0.0, 0.0};
    const vdsim::Vec3 x_world = q * x_body;
    EXPECT_NEAR(x_world.x(), 0.0, kTol);
    EXPECT_NEAR(x_world.y(), 1.0, kTol);
    EXPECT_NEAR(x_world.z(), 0.0, kTol);
}

// =============================================================================
// UE <-> VDSim conversion
// =============================================================================
TEST(CoordinateUE, PositionForward) {
    // VDSim (1m fwd, 2m left, 3m up) -> UE (100cm fwd, -200cm right, 300cm up).
    const vdsim::Vec3 vd {1.0, 2.0, 3.0};
    const vdsim::Vec3 ue = vdsim::ue::to_ue_position(vd);
    EXPECT_NEAR(ue.x(),  100.0, kTol);
    EXPECT_NEAR(ue.y(), -200.0, kTol);
    EXPECT_NEAR(ue.z(),  300.0, kTol);
}

TEST(CoordinateUE, PositionRoundtrip) {
    const vdsim::Vec3 vd {1.234, -5.678, 9.0};
    const vdsim::Vec3 back = vdsim::ue::from_ue_position(vdsim::ue::to_ue_position(vd));
    EXPECT_NEAR((back - vd).norm(), 0.0, kTol);
}

TEST(CoordinateUE, VelocityRoundtrip) {
    const vdsim::Vec3 v {10.0, -3.0, 0.0};
    const vdsim::Vec3 back = vdsim::ue::from_ue_velocity(vdsim::ue::to_ue_velocity(v));
    EXPECT_NEAR((back - v).norm(), 0.0, kTol);
}

TEST(CoordinateUE, RotationRoundtrip) {
    // arbitrary non-trivial attitude
    const vdsim::Quat vd_q = vdsim::quat_from_euler({0.1, 0.2, 0.3});
    const vdsim::Quat ue_q = vdsim::ue::to_ue_rotation(vd_q);
    const vdsim::Quat back = vdsim::ue::from_ue_rotation(ue_q);
    // Quaternions may differ by sign and still be equal rotations.
    const double dot = std::abs(back.dot(vd_q));
    EXPECT_NEAR(dot, 1.0, 1e-12);
}

TEST(CoordinateUE, YawSignFlip) {
    // VDSim yaw = +pi/2 (CCW left, RH) should become UE yaw = -pi/2 (CW right, LH).
    const vdsim::Quat vd_q = vdsim::quat_from_euler({0.0, 0.0, M_PI / 2.0});
    const vdsim::Quat ue_q = vdsim::ue::to_ue_rotation(vd_q);
    // In UE quaternion under same Eigen rep, yaw (Z component sign) should flip:
    EXPECT_NEAR(vd_q.z(), -ue_q.z(), kTol);
    EXPECT_NEAR(vd_q.w(),  ue_q.w(), kTol);
}

TEST(CoordinateUE, IdentityIsIdentity) {
    const vdsim::Quat I = vdsim::Quat::Identity();
    const vdsim::Quat ue_I = vdsim::ue::to_ue_rotation(I);
    EXPECT_NEAR(ue_I.w(), 1.0, kTol);
    EXPECT_NEAR(ue_I.x(), 0.0, kTol);
    EXPECT_NEAR(ue_I.y(), 0.0, kTol);
    EXPECT_NEAR(ue_I.z(), 0.0, kTol);
}
