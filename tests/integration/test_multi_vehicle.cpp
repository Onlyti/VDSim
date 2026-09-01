// Multi-vehicle world runtime (#157): the realtime world spawns N independent
// SimSessions, routes commands per vehicle, and steps them in parallel. These
// tests lock the two load-bearing invariants:
//   1. load_scene() parses an N-vehicle resolved world (ids / poses / paths).
//   2. independent per-vehicle state + command routing (a command to one
//      vehicle does not perturb another); the server's command demux relies on
//      this isolation.
// The SceneSensors cases cover the agents.vehicle.sensors[] declaration
// (CONFIG_GUIDE §2.3): noise -> SensorParams, mount pose/rate -> SceneSensor,
// the scenario-level vs per-vehicle precedence, and the malformed-input errors.

#include "vdsim/interfaces.hpp"
#include "vdsim/sim_session.hpp"

#include "scene_loader.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

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

std::string body_yaml() { return std::string(VDSIM_SOURCE_DIR) + "/configs/parts/body/sedan.yaml"; }
std::string tire_yaml() {
    return std::string(VDSIM_SOURCE_DIR) + "/configs/parts/tire/default_pacejka.yaml";
}

// Write a one-vehicle world whose only interesting part is the `sensors:` block,
// then load it. `sensors_block` is indented YAML pasted under the vehicle entry.
vdsim::cosim::WorldScenario load_world_with_sensors(const std::string& name,
                                                    const std::string& sensors_block) {
    const auto path = std::filesystem::temp_directory_path() / ("vdsim_world_" + name + ".yaml");
    {
        std::ofstream f(path);
        f << "mu: 1.0\nrate: 200\nvehicles:\n"
             "- id: 0\n  vehicle: " << body_yaml() << "\n  tire: " << tire_yaml() << "\n"
             "  level: L2\n  vx0: 10\n" << sensors_block;
    }
    struct Cleanup {
        std::filesystem::path p;
        ~Cleanup() { std::error_code ec; std::filesystem::remove(p, ec); }
    } cleanup{path};
    return vdsim::cosim::load_scene(path.string());
}

// The message load_scene() throws for a malformed `sensors:` block ("" = it did not throw).
std::string sensors_parse_error(const std::string& name, const std::string& sensors_block) {
    try {
        load_world_with_sensors(name, sensors_block);
    } catch (const std::exception& e) {
        return e.what();
    }
    return "";
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

// Scenario-level comms spec parses into TX (source/template/to) and RX
// (direction:in -> listen.port) channels. The realtime server routes by these.
TEST(MultiVehicle, CommsSpecParsed) {
    const std::string veh = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/body/sedan.yaml";
    const std::string tire = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/tire/default_pacejka.yaml";
    const auto path = std::filesystem::temp_directory_path() / "vdsim_world_comms.yaml";
    {
        std::ofstream f(path);
        f << "mu: 1.0\nrate: 200\n"
             "comms:\n  name: hil\n  channels:\n"
             "  - source: 0.state\n    template: vds1\n"
             "    to: [ {ip: 127.0.0.1, port: 7100}, {ip: 127.0.0.1, port: 7101} ]\n"
             "  - direction: in\n    template: vds1_cmd\n    listen: { port: 7001 }\n"
             "vehicles:\n- id: 0\n  vehicle: " << veh << "\n  tire: " << tire << "\n"
             "  level: L2\n  vx0: 10\n";
    }
    const auto w = vdsim::cosim::load_scene(path.string());
    ASSERT_EQ(w.comms.channels.size(), 2u);
    EXPECT_EQ(w.comms.name, "hil");
    // channel 0 = TX fan-out
    EXPECT_FALSE(w.comms.channels[0].rx);
    EXPECT_EQ(w.comms.channels[0].source, "0.state");
    EXPECT_EQ(w.comms.channels[0].templ, "vds1");
    ASSERT_EQ(w.comms.channels[0].to.size(), 2u);
    EXPECT_EQ(w.comms.channels[0].to[1].port, 7101);
    // channel 1 = RX fan-in
    EXPECT_TRUE(w.comms.channels[1].rx);
    EXPECT_EQ(w.comms.channels[1].listen_port, 7001);
    std::filesystem::remove(path);
}

// Per-vehicle sensors[] list is parsed into VehicleSpawn.sensors (SensorParams).
// Vehicle 0 has inline sensors; vehicle 1 has none (identity / truth pass-through).
TEST(MultiVehicle, SensorsListParsed) {
    const std::string veh = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/body/sedan.yaml";
    const std::string tire = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/tire/default_pacejka.yaml";
    const auto path = std::filesystem::temp_directory_path() / "vdsim_world_sensors.yaml";
    {
        std::ofstream f(path);
        f << "mu: 1.0\nrate: 200\nvehicles:\n"
             "- id: 0\n  vehicle: " << veh << "\n  tire: " << tire << "\n"
             "  level: L2\n  vx0: 10\n"
             "  sensors:\n"
             "  - { type: gnss,  noise_std: 0.3 }\n"
             "  - { type: imu,   noise_std: 0.05 }\n"
             "  - { type: steer, noise_std: 0.002 }\n"
             "- id: 1\n  vehicle: " << veh << "\n  tire: " << tire << "\n"
             "  level: L2\n  vx0: 10\n";
    }
    const auto w = vdsim::cosim::load_scene(path.string());
    ASSERT_EQ(w.vehicles.size(), 2u);
    // vehicle 0: sensors parsed -> optional has value, enabled=true
    ASSERT_TRUE(w.vehicles[0].sensors.has_value());
    EXPECT_TRUE(w.vehicles[0].sensors->enabled);
    EXPECT_GT(w.vehicles[0].sensors->gnss_pos.noise_std, 0.0);
    EXPECT_GT(w.vehicles[0].sensors->imu_accel.noise_std, 0.0);
    EXPECT_GT(w.vehicles[0].sensors->steer.noise_std, 0.0);
    // vehicle 1: no sensors key -> optional empty
    EXPECT_FALSE(w.vehicles[1].sensors.has_value());
    std::filesystem::remove(path);
}

// Scene sensor declarations (CONFIG_GUIDE §2.3): id / type / mount pose / rate
// round-trip into VehicleSpawn::scene_sensors. The gnss entry carries BOTH a
// mount and a noise_std, and must feed both halves: the mount list and the
// SensorParams noise field. camera is declaration-only (no noise model in core).
TEST(SceneSensors, MountPoseIdAndRateParsed) {
    const auto w = load_world_with_sensors("mount",
        "  sensors:\n"
        "  - { id: gnss_roof, type: gnss, mount: { pos: [1.4, 0.1, 1.0], rpy: [0, 0, 0.2] },\n"
        "      rate: 10, noise_std: 0.3 }\n"
        "  - { id: cam, type: camera, mount: { pos: [1.6, 0, 1.2], rpy: [0, -0.05, 0] },\n"
        "      rate: 30, params: { fov_deg: 90 } }\n"
        "  - { type: steer }\n");
    ASSERT_EQ(w.vehicles.size(), 1u);
    const auto& sensors = w.vehicles[0].scene_sensors;
    ASSERT_EQ(sensors.size(), 3u);

    EXPECT_EQ(sensors[0].id, "gnss_roof");
    EXPECT_EQ(sensors[0].type, "gnss");
    EXPECT_DOUBLE_EQ(sensors[0].mount_pos[0], 1.4);
    EXPECT_DOUBLE_EQ(sensors[0].mount_pos[1], 0.1);
    EXPECT_DOUBLE_EQ(sensors[0].mount_pos[2], 1.0);
    EXPECT_DOUBLE_EQ(sensors[0].mount_rpy[2], 0.2);
    EXPECT_DOUBLE_EQ(sensors[0].rate, 10.0);
    // same entry also drove the noise model
    ASSERT_TRUE(w.vehicles[0].sensors.has_value());
    EXPECT_DOUBLE_EQ(w.vehicles[0].sensors->gnss_pos.noise_std, 0.3);
    EXPECT_DOUBLE_EQ(w.vehicles[0].sensors->gnss_vel.noise_std, 0.3);

    EXPECT_EQ(sensors[1].type, "camera");
    EXPECT_DOUBLE_EQ(sensors[1].mount_rpy[1], -0.05);
    EXPECT_DOUBLE_EQ(sensors[1].rate, 30.0);
    ASSERT_EQ(sensors[1].params.count("fov_deg"), 1u);
    EXPECT_DOUBLE_EQ(sensors[1].params.at("fov_deg"), 90.0);

    // id defaults to type; mount/rate default to zero (= unspecified)
    EXPECT_EQ(sensors[2].id, "steer");
    EXPECT_DOUBLE_EQ(sensors[2].mount_pos[2], 0.0);
    EXPECT_DOUBLE_EQ(sensors[2].rate, 0.0);
}

// bias / bias_rw and the wheel_speed + gnss_vel groups: fields the inline list
// has always supported but nothing exercised.
TEST(SceneSensors, BiasAndRandomWalkPerGroupParsed) {
    const auto w = load_world_with_sensors("bias",
        "  sensors:\n"
        "  - { type: wheel_speed, noise_std: 0.2, bias: 0.03, bias_rw: 0.004 }\n"
        "  - { type: gnss_vel,    noise_std: 0.5, bias: 0.06, bias_rw: 0.007 }\n");
    ASSERT_TRUE(w.vehicles[0].sensors.has_value());
    const auto& sp = *w.vehicles[0].sensors;
    EXPECT_DOUBLE_EQ(sp.wheel_speed.noise_std, 0.2);
    EXPECT_DOUBLE_EQ(sp.wheel_speed.bias, 0.03);
    EXPECT_DOUBLE_EQ(sp.wheel_speed.bias_rw, 0.004);
    EXPECT_DOUBLE_EQ(sp.gnss_vel.noise_std, 0.5);
    EXPECT_DOUBLE_EQ(sp.gnss_vel.bias, 0.06);
    EXPECT_DOUBLE_EQ(sp.gnss_vel.bias_rw, 0.007);
    // gnss_vel must not have splashed onto gnss_pos
    EXPECT_DOUBLE_EQ(sp.gnss_pos.noise_std, 0.0);
}

// The suite form reaches SensorParams::enabled and ::seed, which the bare list
// form cannot express, while still declaring mounts.
TEST(SceneSensors, SuiteFormCarriesEnabledAndSeed) {
    const auto w = load_world_with_sensors("suite",
        "  sensors:\n"
        "    enabled: false\n"
        "    seed: 42\n"
        "    list:\n"
        "    - { id: gnss, type: gnss, mount: { pos: [1.0, 0, 1.1] }, rate: 5, noise_std: 0.9 }\n");
    ASSERT_TRUE(w.vehicles[0].sensors.has_value());
    EXPECT_FALSE(w.vehicles[0].sensors->enabled);
    EXPECT_EQ(w.vehicles[0].sensors->seed, 42u);
    EXPECT_DOUBLE_EQ(w.vehicles[0].sensors->gnss_pos.noise_std, 0.9);
    ASSERT_EQ(w.vehicles[0].scene_sensors.size(), 1u);
    EXPECT_DOUBLE_EQ(w.vehicles[0].scene_sensors[0].mount_pos[2], 1.1);
    EXPECT_DOUBLE_EQ(w.vehicles[0].scene_sensors[0].mount_rpy[0], 0.0);  // rpy omitted -> zero
}

// The scalar form names a sensors yaml. It carries noise only, so no mounts.
TEST(SceneSensors, FilePathFormLoadsSensorsYaml) {
    const std::string file = std::string(VDSIM_SOURCE_DIR) + "/configs/sensors/noisy.yaml";
    const auto w = load_world_with_sensors("file", "  sensors: " + file + "\n");
    ASSERT_TRUE(w.vehicles[0].sensors.has_value());
    const auto& sp = *w.vehicles[0].sensors;
    EXPECT_TRUE(sp.enabled);
    EXPECT_EQ(sp.seed, 7u);                       // seed reaches through the file form
    EXPECT_DOUBLE_EQ(sp.imu_accel.bias, 0.05);
    EXPECT_DOUBLE_EQ(sp.wheel_speed.noise_std, 0.2);
    EXPECT_DOUBLE_EQ(sp.gnss_vel.bias_rw, 0.01);
    EXPECT_TRUE(w.vehicles[0].scene_sensors.empty());
}

// Precedence (world_scenario.hpp VehicleSpawn::sensors): the scenario-level
// `sensors:` file is the default, a per-vehicle spec overrides it outright.
// effective_sensor_params() is the single place the realtime server applies it.
TEST(SceneSensors, PerVehicleSpecOverridesScenarioLevelFile) {
    const std::string file = std::string(VDSIM_SOURCE_DIR) + "/configs/sensors/noisy.yaml";
    const auto path = std::filesystem::temp_directory_path() / "vdsim_world_sensor_prec.yaml";
    {
        std::ofstream f(path);
        f << "mu: 1.0\nrate: 200\nsensors: " << file << "\nvehicles:\n"
             "- id: 0\n  vehicle: " << body_yaml() << "\n  tire: " << tire_yaml() << "\n"
             "  level: L2\n  vx0: 10\n"
             "  sensors:\n"
             "  - { id: gnss, type: gnss, mount: { pos: [1.4, 0, 1.0] }, rate: 10, noise_std: 0.01 }\n"
             "- id: 1\n  vehicle: " << body_yaml() << "\n  tire: " << tire_yaml() << "\n"
             "  level: L2\n  vx0: 10\n";
    }
    const auto w = vdsim::cosim::load_scene(path.string());
    ASSERT_EQ(w.vehicles.size(), 2u);
    EXPECT_EQ(w.road.sensors, file);              // scenario-level default parsed

    const auto scenario_default = vdsim::SensorParams::from_yaml(w.road.sensors);
    EXPECT_DOUBLE_EQ(scenario_default.gnss_pos.noise_std, 1.5);

    // vehicle 0 declared its own -> the override wins, the file's 1.5 does not
    const auto eff0 = vdsim::cosim::effective_sensor_params(scenario_default, w.vehicles[0]);
    EXPECT_DOUBLE_EQ(eff0.gnss_pos.noise_std, 0.01);
    EXPECT_DOUBLE_EQ(eff0.imu_accel.bias, 0.0);   // not inherited from the file
    // vehicle 1 declared nothing -> it keeps the scenario-level file
    EXPECT_FALSE(w.vehicles[1].sensors.has_value());
    const auto eff1 = vdsim::cosim::effective_sensor_params(scenario_default, w.vehicles[1]);
    EXPECT_DOUBLE_EQ(eff1.gnss_pos.noise_std, 1.5);
    EXPECT_DOUBLE_EQ(eff1.imu_accel.bias, 0.05);
    std::filesystem::remove(path);
}

// Every malformed shape names the vehicle and the offending key/value instead of
// silently defaulting to "this vehicle has no sensors".
TEST(SceneSensors, MalformedDeclarationsThrow) {
    struct Case {
        const char* name;
        const char* block;
        const char* expect;   // substring the message must carry
    };
    const Case cases[] = {
        {"badtype", "  sensors:\n  - { type: radar, noise_std: 0.1 }\n",
         "unknown type 'radar'"},
        {"notype", "  sensors:\n  - { id: x, noise_std: 0.1 }\n",
         "missing required key 'type'"},
        {"badkey", "  sensors:\n  - { type: gnss, noise_stdd: 0.1 }\n",
         "unknown key 'noise_stdd'"},
        {"map", "  sensors:\n    gnss: { noise_std: 0.3 }\n",
         "unknown key 'gnss'"},
        {"nolist", "  sensors:\n    enabled: true\n    seed: 3\n",
         "needs 'list:'"},
        {"null", "  sensors:\n",
         "must be a sequence of entries"},
        {"badmount", "  sensors:\n  - { type: gnss, mount: [1.4, 0, 1.0] }\n",
         "mount must be a map"},
        {"shortpos", "  sensors:\n  - { type: gnss, mount: { pos: [1.4, 0] } }\n",
         "mount.pos must be a sequence of 3 numbers"},
        {"badmountkey", "  sensors:\n  - { type: gnss, mount: { position: [1, 0, 1] } }\n",
         "unknown mount key 'position'"},
        {"badrate", "  sensors:\n  - { type: gnss, rate: fast }\n",
         "rate must be a number"},
        {"cameranoise", "  sensors:\n  - { type: camera, noise_std: 0.2 }\n",
         "has no measurement model"},
        {"dupid", "  sensors:\n  - { id: g, type: gnss }\n  - { id: g, type: imu }\n",
         "duplicate sensor id 'g'"},
        {"scalarentry", "  sensors:\n  - gnss\n",
         "entry must be a map"},
        {"missingfile", "  sensors: /nonexistent/sensors_xyz.yaml\n",
         "sensors file not loadable"},
    };
    for (const auto& c : cases) {
        const std::string msg = sensors_parse_error(c.name, c.block);
        ASSERT_FALSE(msg.empty()) << c.name << ": malformed sensors must throw";
        EXPECT_NE(msg.find(c.expect), std::string::npos)
            << c.name << ": message was \"" << msg << '"';
        EXPECT_NE(msg.find("world scenario: vehicle 0 sensors"), std::string::npos)
            << c.name << ": message must name the vehicle, was \"" << msg << '"';
    }
}

// The shipped catalog scene declares sensors with mount poses, so the schema
// cannot rot: this loads configs/scenes/two_vehicle_race.yaml through the
// materializer and asserts the declarations survive to WorldScenario.
TEST(SceneSensors, ShippedSceneDeclaresMountPoses) {
    const std::string scene = std::string(VDSIM_SOURCE_DIR)
        + "/configs/scenes/two_vehicle_race.yaml";
    const auto w = vdsim::cosim::load_scene(scene);
    ASSERT_EQ(w.vehicles.size(), 2u);
    const auto& sensors = w.vehicles[0].scene_sensors;
    ASSERT_EQ(sensors.size(), 4u) << "two_vehicle_race vehicle 0 ships 4 sensor mounts";
    EXPECT_EQ(sensors[0].id, "gnss_roof");
    EXPECT_EQ(sensors[0].type, "gnss");
    EXPECT_DOUBLE_EQ(sensors[0].mount_pos[2], 1.42);
    EXPECT_DOUBLE_EQ(sensors[0].rate, 10.0);
    EXPECT_EQ(sensors[3].id, "cam_front");
    EXPECT_EQ(sensors[3].type, "camera");
    EXPECT_DOUBLE_EQ(sensors[3].mount_rpy[1], -0.05);
    EXPECT_DOUBLE_EQ(sensors[3].params.at("fov_deg"), 90.0);
    // The scene deliberately declares no noise, so vehicle 0 stays truth-identical
    // for the comms/GUI tests that read it.
    ASSERT_TRUE(w.vehicles[0].sensors.has_value());
    EXPECT_DOUBLE_EQ(w.vehicles[0].sensors->gnss_pos.noise_std, 0.0);
    EXPECT_DOUBLE_EQ(w.vehicles[0].sensors->imu_accel.noise_std, 0.0);
    // vehicle 1 declares none
    EXPECT_TRUE(w.vehicles[1].scene_sensors.empty());
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
