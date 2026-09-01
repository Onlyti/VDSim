#include "vdsim/contact_frame.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

/** Return a road normal for positive/negative bank about body x. */
vdsim::Vec3 bank_normal(double bank_rad) {
    return vdsim::Vec3(0.0, -std::sin(bank_rad), std::cos(bank_rad));
}

}  // namespace

TEST(ContactFrame, FlatBasisPreservesCanonicalWheelFrame) {
    const double steer = 0.17;
    const vdsim::Vec3 forward(std::cos(steer), std::sin(steer), 0.0);
    const auto frame = vdsim::make_contact_frame(vdsim::Vec3::UnitZ(), forward);
    ASSERT_TRUE(frame.valid);
    EXPECT_DOUBLE_EQ(frame.x_c.x(), forward.x());
    EXPECT_DOUBLE_EQ(frame.x_c.y(), forward.y());
    EXPECT_DOUBLE_EQ(frame.y_c.x(), -forward.y());
    EXPECT_DOUBLE_EQ(frame.y_c.y(), forward.x());
    EXPECT_EQ(frame.z_c, vdsim::Vec3::UnitZ());
}

TEST(ContactFrame, FiniteBankUsesExactProjectedRightHandedBasis) {
    const double bank = 10.0 * kDegToRad;
    const auto plus = vdsim::make_contact_frame(
        bank_normal(bank), vdsim::Vec3::UnitX());
    const auto minus = vdsim::make_contact_frame(
        bank_normal(-bank), vdsim::Vec3::UnitX());
    ASSERT_TRUE(plus.valid);
    ASSERT_TRUE(minus.valid);

    EXPECT_NEAR(plus.x_c.norm(), 1.0, 1e-15);
    EXPECT_NEAR(plus.y_c.norm(), 1.0, 1e-15);
    EXPECT_NEAR(plus.z_c.norm(), 1.0, 1e-15);
    EXPECT_NEAR(plus.x_c.dot(plus.z_c), 0.0, 1e-15);
    EXPECT_NEAR((plus.z_c.cross(plus.x_c) - plus.y_c).norm(), 0.0, 1e-15);

    const vdsim::Vec3 velocity(15.0, 1.0, 0.0);
    const auto vp = plus.resolve_velocity(velocity);
    const auto vm = minus.resolve_velocity(velocity);
    EXPECT_NEAR(vp.x(), 15.0, 1e-15);
    EXPECT_NEAR(vp.y(), std::cos(bank), 1e-15);
    EXPECT_NEAR(vm.y(), std::cos(bank), 1e-15);
    EXPECT_NEAR(vp.z(), -vm.z(), 1e-15);

    const auto fp = plus.tangential_force(100.0, -200.0);
    const auto fm = minus.tangential_force(100.0, -200.0);
    EXPECT_NEAR(fp.x(), fm.x(), 1e-13);
    EXPECT_NEAR(fp.y(), fm.y(), 1e-13);
    EXPECT_NEAR(fp.z(), -fm.z(), 1e-13);
    EXPECT_NEAR(fp.dot(plus.z_c), 0.0, 1e-13);
    EXPECT_NEAR(fm.dot(minus.z_c), 0.0, 1e-13);
}

TEST(ContactFrame, FzIsTheInjectedNormalDirectionComponent) {
    const auto frame = vdsim::make_contact_frame(
        bank_normal(10.0 * kDegToRad), vdsim::Vec3::UnitX());
    ASSERT_TRUE(frame.valid);
    const double Fz = 3200.0;
    const auto normal_force = frame.normal_force(Fz);
    EXPECT_NEAR(normal_force.norm(), Fz, 1e-12);
    EXPECT_NEAR(normal_force.dot(frame.z_c), Fz, 1e-12);
    EXPECT_NEAR(normal_force.dot(frame.x_c), 0.0, 1e-12);
    EXPECT_NEAR(normal_force.dot(frame.y_c), 0.0, 1e-12);
}

TEST(ContactFrame, DegenerateInputsFailClosedWithoutPlanarFallback) {
    EXPECT_FALSE(vdsim::make_contact_frame(
        vdsim::Vec3::Zero(), vdsim::Vec3::UnitX()).valid);
    EXPECT_FALSE(vdsim::make_contact_frame(
        vdsim::Vec3::UnitX(), vdsim::Vec3::UnitX()).valid);
    vdsim::Vec3 nonfinite = vdsim::Vec3::UnitZ();
    nonfinite.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(vdsim::make_contact_frame(
        nonfinite, vdsim::Vec3::UnitX()).valid);
}
