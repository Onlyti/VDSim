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
        {"badmount", "  sensors:\n  - { type: gnss, mount: 1.4 }\n",
         "mount must be a map"},
        {"shortmountseq", "  sensors:\n  - { type: gnss, mount: [1.4, 0] }\n",
         "mount must be a sequence of 3 numbers"},
        {"yawandrpy", "  sensors:\n  - { type: gnss, mount: { rpy: [0, 0, 0.2] }, yaw: 15 }\n",
         "yaw and mount.rpy both set the heading"},
        {"badtypekind", "  sensors:\n  - { type: [gnss] }\n",
         "type must be one of the type names"},
        {"emptyscalar", "  sensors: \"\"\n",
         "empty value"},
        {"shortpos", "  sensors:\n  - { type: gnss, mount: { pos: [1.4, 0] } }\n",
         "mount.pos must be a sequence of 3 numbers"},
        {"badmountkey", "  sensors:\n  - { type: gnss, mount: { position: [1, 0, 1] } }\n",
         "unknown mount key 'position'"},
        {"badrate", "  sensors:\n  - { type: gnss, rate: fast }\n",
         "rate must be a number"},
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
    // The scene deliberately declares no noise. Mount-only, so it must NOT claim a
    // per-vehicle SensorParams: the scene has no scenario-level `sensors:` file, so
    // both vehicles run on the same truth-identical default.
    EXPECT_FALSE(w.vehicles[0].sensors.has_value());
    EXPECT_FALSE(w.vehicles[1].sensors.has_value());
    const vdsim::SensorParams scenario_default;   // no scene-level sensors file here
    EXPECT_FALSE(vdsim::cosim::effective_sensor_params(scenario_default, w.vehicles[0]).enabled);
    // vehicle 1 declares none
    EXPECT_TRUE(w.vehicles[1].scene_sensors.empty());
}

// A `sensors:` block that declares only mounts says nothing about measurement
// noise, so it must not opt the vehicle out of the scenario-level `sensors:` file.
// Before this was fixed, vehicle 0 below ran on an all-zero SensorParams (clean)
// while vehicle 1 ran the file's 1.5 m GNSS noise — from a MOUNT declaration.
TEST(SceneSensors, MountOnlyBlockKeepsScenarioLevelFile) {
    const std::string file = std::string(VDSIM_SOURCE_DIR) + "/configs/sensors/noisy.yaml";
    const auto path = std::filesystem::temp_directory_path() / "vdsim_world_mountonly.yaml";
    {
        std::ofstream f(path);
        f << "mu: 1.0\nrate: 200\nsensors: " << file << "\nvehicles:\n"
             "- id: 0\n  vehicle: " << body_yaml() << "\n  tire: " << tire_yaml() << "\n"
             "  level: L2\n  vx0: 10\n"
             "  sensors:\n"
             "  - { id: gnss_roof, type: gnss, mount: { pos: [0.2, 0, 1.42] }, rate: 10 }\n"
             "- id: 1\n  vehicle: " << body_yaml() << "\n  tire: " << tire_yaml() << "\n"
             "  level: L2\n  vx0: 10\n";
    }
    const auto w = vdsim::cosim::load_scene(path.string());
    ASSERT_EQ(w.vehicles.size(), 2u);
    const auto scenario_default = vdsim::SensorParams::from_yaml(w.road.sensors);
    // the mount survived ...
    ASSERT_EQ(w.vehicles[0].scene_sensors.size(), 1u);
    EXPECT_EQ(w.vehicles[0].scene_sensors[0].id, "gnss_roof");
    EXPECT_DOUBLE_EQ(w.vehicles[0].scene_sensors[0].mount_pos[2], 1.42);
    // ... and did NOT become a noise override
    EXPECT_FALSE(w.vehicles[0].sensors.has_value());
    const auto eff0 = vdsim::cosim::effective_sensor_params(scenario_default, w.vehicles[0]);
    const auto eff1 = vdsim::cosim::effective_sensor_params(scenario_default, w.vehicles[1]);
    EXPECT_DOUBLE_EQ(eff0.gnss_pos.noise_std, 1.5);
    EXPECT_DOUBLE_EQ(eff0.imu_accel.bias, 0.05);
    EXPECT_EQ(eff0.seed, eff1.seed);              // both neighbours run the same model
    EXPECT_DOUBLE_EQ(eff0.gnss_pos.noise_std, eff1.gnss_pos.noise_std);
    std::filesystem::remove(path);
}

// The suite form's enabled/seed ARE a statement about the noise model, so a suite
// overrides the scenario-level file even when its list carries mounts only.
TEST(SceneSensors, SuiteEnabledSeedOverridesScenarioLevelFile) {
    const std::string file = std::string(VDSIM_SOURCE_DIR) + "/configs/sensors/noisy.yaml";
    const auto path = std::filesystem::temp_directory_path() / "vdsim_world_suiteonly.yaml";
    {
        std::ofstream f(path);
        f << "mu: 1.0\nrate: 200\nsensors: " << file << "\nvehicles:\n"
             "- id: 0\n  vehicle: " << body_yaml() << "\n  tire: " << tire_yaml() << "\n"
             "  level: L2\n  vx0: 10\n"
             "  sensors:\n    enabled: false\n    list:\n"
             "    - { id: gnss_roof, type: gnss, mount: { pos: [0.2, 0, 1.42] } }\n";
    }
    const auto w = vdsim::cosim::load_scene(path.string());
    ASSERT_TRUE(w.vehicles[0].sensors.has_value());
    const auto scenario_default = vdsim::SensorParams::from_yaml(w.road.sensors);
    const auto eff = vdsim::cosim::effective_sensor_params(scenario_default, w.vehicles[0]);
    EXPECT_FALSE(eff.enabled);
    EXPECT_DOUBLE_EQ(eff.gnss_pos.noise_std, 0.0);
    std::filesystem::remove(path);
}

// Overlapping type groups: a mount-only entry writes no noise at all, so it cannot
// reset a group a wider entry already set. `gnss` then `gnss_pos` keeps gnss_pos at
// the gnss value; `imu` then `imu_gyro` keeps imu_gyro at the imu value.
TEST(SceneSensors, MountOnlyEntryDoesNotResetOverlappingGroup) {
    const auto gnss = load_world_with_sensors("overlap_gnss",
        "  sensors:\n"
        "  - { type: gnss, noise_std: 0.5 }\n"
        "  - { type: gnss_pos }\n");
    ASSERT_TRUE(gnss.vehicles[0].sensors.has_value());
    EXPECT_DOUBLE_EQ(gnss.vehicles[0].sensors->gnss_pos.noise_std, 0.5);
    EXPECT_DOUBLE_EQ(gnss.vehicles[0].sensors->gnss_vel.noise_std, 0.5);

    const auto imu = load_world_with_sensors("overlap_imu",
        "  sensors:\n"
        "  - { type: imu, noise_std: 0.05 }\n"
        "  - { type: imu_gyro, mount: { pos: [0, 0, 0.4] } }\n");
    ASSERT_TRUE(imu.vehicles[0].sensors.has_value());
    EXPECT_DOUBLE_EQ(imu.vehicles[0].sensors->imu_accel.noise_std, 0.05);
    EXPECT_DOUBLE_EQ(imu.vehicles[0].sensors->imu_gyro.noise_std, 0.05);

    // An entry that DOES carry a noise key still overwrites the group outright.
    const auto over = load_world_with_sensors("overlap_write",
        "  sensors:\n"
        "  - { type: gnss, noise_std: 0.5 }\n"
        "  - { type: gnss_pos, noise_std: 0.1 }\n");
    EXPECT_DOUBLE_EQ(over.vehicles[0].sensors->gnss_pos.noise_std, 0.1);
    EXPECT_DOUBLE_EQ(over.vehicles[0].sensors->gnss_vel.noise_std, 0.5);
}

// Uniqueness is enforced over explicitly written ids only. Two entries of the same
// type without an `id` are legal (last noise write wins, as it always did), and a
// synthesised default id never collides with an unrelated explicit id.
TEST(SceneSensors, DefaultIdsMayRepeat) {
    const auto w = load_world_with_sensors("dupdefault",
        "  sensors:\n"
        "  - { type: gnss, noise_std: 0.3 }\n"
        "  - { type: gnss, noise_std: 0.9 }\n");
    ASSERT_EQ(w.vehicles[0].scene_sensors.size(), 2u);
    EXPECT_EQ(w.vehicles[0].scene_sensors[0].id, "gnss");
    EXPECT_EQ(w.vehicles[0].scene_sensors[1].id, "gnss");
    EXPECT_DOUBLE_EQ(w.vehicles[0].sensors->gnss_pos.noise_std, 0.9);

    const auto cross = load_world_with_sensors("crossid",
        "  sensors:\n"
        "  - { id: imu, type: gnss }\n"
        "  - { type: imu }\n");
    ASSERT_EQ(cross.vehicles[0].scene_sensors.size(), 2u);
    EXPECT_EQ(cross.vehicles[0].scene_sensors[0].id, "imu");
    EXPECT_EQ(cross.vehicles[0].scene_sensors[1].id, "imu");
}

// The builder (builder/README.md, builder/index.html addSensor()) emits
// `mount: [x,y,z]` + `yaw: <deg>` and a `noise_std` on every type including
// camera/lidar. That shape must load: it is what this repo already publishes.
TEST(SceneSensors, BuilderMountSequenceAndYawFormAccepted) {
    const auto w = load_world_with_sensors("builder",
        "  sensors:\n"
        "  - { id: gnss, type: gnss,   mount: [0, 0, 1.0],   yaw: 0,  rate: 20, noise_std: 0.3 }\n"
        "  - { id: cam,  type: camera, mount: [1.6, 0, 1.2], yaw: 90, rate: 30, noise_std: 0.3 }\n"
        "  - { id: lid,  type: lidar,  mount: [0, 0, 1.8],   yaw: 0,  rate: 10, noise_std: 0.3 }\n");
    const auto& s = w.vehicles[0].scene_sensors;
    ASSERT_EQ(s.size(), 3u);
    EXPECT_DOUBLE_EQ(s[0].mount_pos[2], 1.0);
    EXPECT_DOUBLE_EQ(s[0].mount_rpy[2], 0.0);
    EXPECT_DOUBLE_EQ(s[1].mount_pos[0], 1.6);
    // yaw is degrees (the builder's label); mount_rpy is radians
    EXPECT_NEAR(s[1].mount_rpy[2], 1.5707963267948966, 1e-12);
    EXPECT_DOUBLE_EQ(s[2].rate, 10.0);
    // gnss noise applies; the camera/lidar noise_std is accepted and dropped,
    // because the core has no measurement model for those types.
    ASSERT_TRUE(w.vehicles[0].sensors.has_value());
    EXPECT_DOUBLE_EQ(w.vehicles[0].sensors->gnss_pos.noise_std, 0.3);

    // camera/lidar alone: noise ignored AND not treated as a noise override
    const auto only_cam = load_world_with_sensors("builder_cam",
        "  sensors:\n  - { id: cam, type: camera, mount: [1.6, 0, 1.2], noise_std: 0.3 }\n");
    EXPECT_FALSE(only_cam.vehicles[0].sensors.has_value());
    ASSERT_EQ(only_cam.vehicles[0].scene_sensors.size(), 1u);
}

// Every number the block carries is checked for finiteness, and rate / noise_std
// for sign. A NaN noise_std would otherwise turn every measured channel into NaN.
TEST(SceneSensors, NonFiniteAndOutOfRangeNumbersRejected) {
    struct Case { const char* name; const char* block; const char* expect; };
    const Case cases[] = {
        {"negrate", "  sensors:\n  - { type: gnss, rate: -10 }\n", "rate must be >= 0"},
        {"nanrate", "  sensors:\n  - { type: gnss, rate: .nan }\n", "rate must be finite"},
        {"nanpos", "  sensors:\n  - { type: gnss, mount: { pos: [.nan, 0, 0] } }\n",
         "mount.pos[0] must be finite"},
        {"infpos", "  sensors:\n  - { type: gnss, mount: { pos: [.inf, 0, 0] } }\n",
         "mount.pos[0] must be finite"},
        {"infrpy", "  sensors:\n  - { type: gnss, mount: { rpy: [0, 0, -.inf] } }\n",
         "mount.rpy[2] must be finite"},
        {"infseq", "  sensors:\n  - { type: gnss, mount: [0, .inf, 0] }\n",
         "mount[1] must be finite"},
        {"nanyaw", "  sensors:\n  - { type: gnss, mount: [0, 0, 0], yaw: .nan }\n",
         "yaw must be finite"},
        {"negstd", "  sensors:\n  - { type: gnss, noise_std: -5 }\n", "noise_std must be >= 0"},
        {"nanstd", "  sensors:\n  - { type: gnss, noise_std: .nan }\n", "noise_std must be finite"},
        {"nanbias", "  sensors:\n  - { type: imu, bias: .nan }\n", "bias must be finite"},
        {"infrw", "  sensors:\n  - { type: imu, bias_rw: .inf }\n", "bias_rw must be finite"},
        {"nanparam", "  sensors:\n  - { type: camera, params: { fov_deg: .nan } }\n",
         "params.fov_deg must be finite"},
        // validated even where the value is then dropped (camera has no noise model)
        {"camnegstd", "  sensors:\n  - { type: camera, noise_std: -1 }\n",
         "noise_std must be >= 0"},
    };
    for (const auto& c : cases) {
        const std::string msg = sensors_parse_error(c.name, c.block);
        ASSERT_FALSE(msg.empty()) << c.name << ": must throw";
        EXPECT_NE(msg.find(c.expect), std::string::npos)
            << c.name << ": message was \"" << msg << '"';
    }
    // rate 0 stays legal: it is the documented "unspecified" value.
    const auto ok = load_world_with_sensors("zerorate", "  sensors:\n  - { type: gnss, rate: 0 }\n");
    EXPECT_DOUBLE_EQ(ok.vehicles[0].scene_sensors[0].rate, 0.0);
}

// Optional keys distinguish omission (documented default) from an explicit YAML
// null, which is a present value of the wrong type and therefore a hard error.
TEST(SceneSensors, ExplicitNullOptionalKeysAreHardErrors) {
    struct Case { const char* name; const char* block; const char* expect; };
    const Case cases[] = {
        {"nullid",     "  sensors:\n  - { id: ~, type: gnss }\n", "id must be a string"},
        {"nullmount",  "  sensors:\n  - { type: gnss, mount: ~ }\n", "mount must be a map"},
        {"nullpos",    "  sensors:\n  - { type: gnss, mount: { pos: ~ } }\n", "mount.pos must be a sequence"},
        {"nullrpy",    "  sensors:\n  - { type: gnss, mount: { rpy: ~ } }\n", "mount.rpy must be a sequence"},
        {"nullyaw",    "  sensors:\n  - { type: gnss, yaw: ~ }\n", "yaw must be a number"},
        {"nullrate",   "  sensors:\n  - { type: gnss, rate: ~ }\n", "rate must be a number"},
        {"nullstd",    "  sensors:\n  - { type: gnss, noise_std: ~ }\n", "noise_std must be a number"},
        {"nullbias",   "  sensors:\n  - { type: imu, bias: ~ }\n", "bias must be a number"},
        {"nullrw",     "  sensors:\n  - { type: imu, bias_rw: ~ }\n", "bias_rw must be a number"},
        {"nullparams", "  sensors:\n  - { type: camera, params: ~ }\n", "params must be a map"},
        {"nullparam",  "  sensors:\n  - { type: camera, params: { fov_deg: ~ } }\n", "params.fov_deg must be a number"},
        {"nullenabled", "  sensors:\n    enabled: ~\n    list: []\n", "enabled must be a bool"},
        {"nullseed",    "  sensors:\n    seed: ~\n    list: []\n", "seed a non-negative integer"},
    };
    for (const auto& c : cases) {
        const std::string msg = sensors_parse_error(c.name, c.block);
        ASSERT_FALSE(msg.empty()) << c.name << ": explicit null must throw";
        EXPECT_NE(msg.find(c.expect), std::string::npos)
            << c.name << ": message was \"" << msg << '"';
    }

    // Omitting those same optional keys retains the documented zero/default values.
    const auto omitted = load_world_with_sensors("optional_omitted",
        "  sensors:\n  - { type: gnss }\n");
    ASSERT_EQ(omitted.vehicles[0].scene_sensors.size(), 1u);
    EXPECT_EQ(omitted.vehicles[0].scene_sensors[0].id, "gnss");
    EXPECT_DOUBLE_EQ(omitted.vehicles[0].scene_sensors[0].rate, 0.0);
    EXPECT_FALSE(omitted.vehicles[0].sensors.has_value());
}

// The scalar/file form must not swallow yaml-cpp's reason: a malformed sensors
// file is undebuggable without the line and column it reports.
TEST(SceneSensors, FilePathFormNestsTheUnderlyingReason) {
    const auto bad = std::filesystem::temp_directory_path() / "vdsim_broken_sensors.yaml";
    { std::ofstream f(bad); f << "sensors:\n  gnss_pos:\n    noise_std: [1.0\n"; }
    std::string underlying;
    try { vdsim::SensorParams::from_yaml(bad.string()); }
    catch (const std::exception& e) { underlying = e.what(); }
    ASSERT_FALSE(underlying.empty()) << "the fixture must be a malformed sensors file";

    const std::string msg = sensors_parse_error("brokenfile", "  sensors: " + bad.string() + "\n");
    EXPECT_NE(msg.find("sensors file not loadable"), std::string::npos) << msg;
    EXPECT_NE(msg.find(underlying), std::string::npos)
        << "the yaml-cpp reason must survive, was \"" << msg << '"';
    std::filesystem::remove(bad);
}

// If the CWD-relative file exists but is malformed, resolution stops there. The
// error must not claim that the scene-directory candidate was also attempted.
TEST(SceneSensors, CwdMalformedFileDoesNotClaimSceneDirAttempt) {
    const std::string filename = "vdsim_cwd_broken_sensors_p3.yaml";
    const auto cwd_file = std::filesystem::current_path() / filename;
    {
        std::ofstream f(cwd_file);
        f << "gnss_pos: { noise_std: [broken }\n";
    }
    const std::string msg = sensors_parse_error("cwd_broken", "  sensors: " + filename + "\n");
    std::error_code ec;
    std::filesystem::remove(cwd_file, ec);

    ASSERT_FALSE(msg.empty()) << "the malformed CWD file must fail";
    EXPECT_NE(msg.find(filename), std::string::npos) << msg;
    EXPECT_EQ(msg.find("also tried"), std::string::npos)
        << "the scene-directory candidate was not attempted: " << msg;
}

// A relative `sensors:` path is used as written first (CWD-relative, which is how
// the CONFIG_GUIDE examples read from the repo root), then retried against the
// directory of the scene file so a scene-local suite resolves from anywhere.
TEST(SceneSensors, RelativeSensorsPathResolvesAgainstSceneDir) {
    const auto dir = std::filesystem::temp_directory_path() / "vdsim_relsensors";
    std::filesystem::create_directories(dir);
    const auto sensors = dir / "local_sensors.yaml";
    {
        std::ofstream f(sensors);
        f << "enabled: true\nseed: 5\ngnss_pos: { noise_std: 0.77 }\n";
    }
    const auto path = dir / "world.yaml";
    {
        std::ofstream f(path);
        f << "mu: 1.0\nrate: 200\nvehicles:\n"
             "- id: 0\n  vehicle: " << body_yaml() << "\n  tire: " << tire_yaml() << "\n"
             "  level: L2\n  vx0: 10\n  sensors: local_sensors.yaml\n";
    }
    const auto w = vdsim::cosim::load_scene(path.string());
    ASSERT_TRUE(w.vehicles[0].sensors.has_value());
    EXPECT_DOUBLE_EQ(w.vehicles[0].sensors->gnss_pos.noise_std, 0.77);
    EXPECT_EQ(w.vehicles[0].sensors->seed, 5u);
    // An unresolvable relative path names both candidates it tried.
    {
        std::ofstream f(path);
        f << "mu: 1.0\nrate: 200\nvehicles:\n"
             "- id: 0\n  vehicle: " << body_yaml() << "\n  tire: " << tire_yaml() << "\n"
             "  level: L2\n  vx0: 10\n  sensors: no_such_sensors.yaml\n";
    }
    std::string msg;
    try { vdsim::cosim::load_scene(path.string()); }
    catch (const std::exception& e) { msg = e.what(); }
    EXPECT_NE(msg.find("no_such_sensors.yaml"), std::string::npos) << msg;
    EXPECT_NE(msg.find("also tried"), std::string::npos) << msg;
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// `id` is the literal scalar text, so YAML's bare true/12 become "true"/"12".
TEST(SceneSensors, IdIsTheLiteralScalarText) {
    const auto w = load_world_with_sensors("idkind",
        "  sensors:\n  - { id: true, type: gnss }\n  - { id: 12, type: imu }\n");
    ASSERT_EQ(w.vehicles[0].scene_sensors.size(), 2u);
    EXPECT_EQ(w.vehicles[0].scene_sensors[0].id, "true");
    EXPECT_EQ(w.vehicles[0].scene_sensors[1].id, "12");
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
