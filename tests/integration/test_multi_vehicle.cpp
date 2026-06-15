// Multi-vehicle world runtime (#157): the realtime world spawns N independent
// SimSessions, routes commands per vehicle, and steps them in parallel. These
// tests lock the two load-bearing invariants:
//   1. load_scene() parses an N-vehicle resolved world (ids / poses / paths).
//   2. independent per-vehicle state + command routing (a command to one
//      vehicle does not perturb another); the server's command demux relies on
//      this isolation.

#include "vdsim/interfaces.hpp"
#include "vdsim/sim_session.hpp"

#include "scene_loader.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>

#ifndef VDSIM_SOURCE_DIR
#define VDSIM_SOURCE_DIR "."
#endif

namespace {

std::unique_ptr<vdsim::SimSession> make_world_vehicle(double vx0) {
    vdsim::VehicleParams vp;
    vdsim::TireParams tp;
    vdsim::SolverParams sp;
    auto s = std::make_unique<vdsim::SimSession>(
        vdsim::create_seven_dof(), vdsim::create_flat_ground(0.0, 1.0), vp, tp, sp);
    vdsim::State s0;
    s0.velocity = {vx0, 0.0, 0.0};
    const double w0 = vx0 / std::max(0.05, vp.wheel_radius_nominal);
    s0.wheel_spin = {{w0, w0, w0, w0}};
    s->reset(s0);
    return s;
}

}  // namespace

TEST(MultiVehicle, LoadSceneParsesFleet) {
    const std::string veh = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/body/sedan.yaml";
    const std::string tire = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/tire/default_pacejka.yaml";
    const auto path = std::filesystem::temp_directory_path() / "vdsim_world_2veh.yaml";
    {
        std::ofstream f(path);
        f << "mu: 1.0\nrate: 200\nvehicles:\n"
             "- id: 0\n  vehicle: " << veh << "\n  tire: " << tire << "\n"
             "  level: L2\n  x0: -15\n  y0: -1.5\n  vx0: 12\n  control: internal\n"
             "- id: 7\n  vehicle: " << veh << "\n  tire: " << tire << "\n"
             "  level: L2\n  x0: -15\n  y0: 1.5\n  vx0: 9\n";
    }
    const auto w = vdsim::cosim::load_scene(path.string());
    ASSERT_EQ(w.vehicles.size(), 2u);
    EXPECT_EQ(w.vehicles[0].id, 0u);
    EXPECT_EQ(w.vehicles[1].id, 7u);                 // explicit ids preserved
    EXPECT_DOUBLE_EQ(w.vehicles[0].y0, -1.5);
    EXPECT_DOUBLE_EQ(w.vehicles[1].y0, 1.5);
    EXPECT_DOUBLE_EQ(w.vehicles[1].vx0, 9.0);
    EXPECT_EQ(w.vehicles[0].control, "internal");    // per-agent control parsed
    EXPECT_EQ(w.vehicles[1].control, "external");    // default when omitted
    std::filesystem::remove(path);
}

// Two SimSessions stepped side by side (as the realtime world does): a throttle
// command to vehicle 0 must accelerate only vehicle 0; vehicle 1 (coasting) must
// be unaffected. This is the per-vehicle isolation the command demux depends on.
TEST(MultiVehicle, IndependentStateAndCommandRouting) {
    auto v0 = make_world_vehicle(12.0);
    auto v1 = make_world_vehicle(12.0);
    vdsim::CmdL4 throttle; throttle.throttle = 1.0;
    vdsim::CmdL4 coast;     // no input
    v0->set_input(throttle);
    v1->set_input(coast);
    for (int i = 0; i < 400; ++i) { v0->tick(0.005); v1->tick(0.005); }  // 2 s
    const double vx0 = v0->state().vx();
    const double vx1 = v1->state().vx();
    EXPECT_GT(vx0, 12.5) << "throttled vehicle 0 should accelerate";
    EXPECT_LT(vx1, vx0 - 1.0) << "coasting vehicle 1 must not track vehicle 0";
    // Lateral isolation: steer only vehicle 1, vehicle 0 stays straight.
    vdsim::CmdL4 steer; steer.steer_angle_wheel = 0.05;
    v1->set_input(steer);
    v0->set_input(coast);
    for (int i = 0; i < 200; ++i) { v0->tick(0.005); v1->tick(0.005); }
    EXPECT_GT(std::abs(v1->state().yaw_rate()), 1e-3) << "vehicle 1 should yaw";
    EXPECT_LT(std::abs(v0->state().yaw_rate()), 1e-3) << "vehicle 0 must stay straight";
}
