#include "raycast_contact_provider.hpp"
#include "vdsim/interfaces.hpp"

#include <gtest/gtest.h>

namespace {

vdsim_carla::RaycastFn flat_ground_raycast(int surface_id, double z_ground) {
    return [=](const vdsim::Vec3& /*start*/, double /*max_depth*/,
               double& out_z, vdsim::Vec3& out_normal, int& out_sid) {
        out_z = z_ground;
        out_normal = vdsim::Vec3::UnitZ();
        out_sid = surface_id;
        return true;
    };
}

vdsim_carla::RaycastFn always_miss_raycast() {
    return [](const vdsim::Vec3&, double, double&, vdsim::Vec3&, int&) {
        return false;
    };
}

}  // namespace

TEST(CarlaPlugin, FlatGroundLookupYieldsKnownMu) {
    vdsim_carla::RaycastContactProvider::SurfaceMaterial mats[] = {
        {7, 1.05, 1.10},
        {8, 0.20, 0.20},
    };
    auto prov = vdsim_carla::create_raycast_provider(
        flat_ground_raycast(7, 0.0), mats, 2, 1.0);

    vdsim::State s;
    vdsim::VehicleParams vp;
    vdsim::ContactArray out;
    prov->query(s, vp, out);
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
        EXPECT_TRUE(out[i].is_valid);
        EXPECT_DOUBLE_EQ(out[i].mu_long, 1.05);
        EXPECT_DOUBLE_EQ(out[i].mu_lat,  1.10);
    }
}

TEST(CarlaPlugin, UnknownSurfaceFallsBackToDefault) {
    vdsim_carla::RaycastContactProvider::SurfaceMaterial mats[] = {
        {7, 0.50, 0.50}
    };
    auto prov = vdsim_carla::create_raycast_provider(
        flat_ground_raycast(99, 0.0), mats, 1, 0.65);

    vdsim::State s; vdsim::VehicleParams vp;
    vdsim::ContactArray out;
    prov->query(s, vp, out);
    EXPECT_DOUBLE_EQ(out[0].mu_long, 0.65);
}

TEST(CarlaPlugin, MissedRaycastInvalidatesContact) {
    auto prov = vdsim_carla::create_raycast_provider(
        always_miss_raycast(), nullptr, 0, 1.0);
    vdsim::State s; vdsim::VehicleParams vp;
    vdsim::ContactArray out;
    prov->query(s, vp, out);
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
        EXPECT_FALSE(out[i].is_valid);
    }
}

TEST(CarlaPlugin, NullRaycastSafe) {
    auto prov = vdsim_carla::create_raycast_provider(
        nullptr, nullptr, 0, 1.0);
    vdsim::State s; vdsim::VehicleParams vp;
    vdsim::ContactArray out;
    prov->query(s, vp, out);                       // must not crash
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) EXPECT_FALSE(out[i].is_valid);
}
