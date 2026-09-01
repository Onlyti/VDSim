#pragma once

#include "comms_templates.hpp"
#include "vdsim/sensors.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace vdsim::cosim {

// Simulation timing + solver configuration (scenario-level, applies to all vehicles).
// dt = core timestep [s]; rate = real-time target [Hz] (informational for sync/display).
// t_end = batch max time [s] (0 = unbounded, for interactive/cosim).
// time_scale: 1.0=realtime, <1=slower, >1=faster.
struct SimConfig {
    double dt           {0.005};     // core timestep [s]
    double rate         {200.0};     // real-time target rate [Hz]; typically ~1/dt
    double t_end        {0.0};       // batch max simulation time [s] (0 = unlimited)
    double time_scale   {1.0};       // playback rate: 1.0=realtime, <1=slower, >1=faster
    double max_substep_dt  {1e-4};   // RK4 substep size [s]
    int    max_substeps    {24};     // max RK4 substeps per core step
    bool   stunt_physics   {false};  // enable stunt-specific physics (grade, L5)
};

struct RoadConfig {
    double mu          {1.0};
    double mu_right    {-1.0};
    double mu_boundary {0.0};
    double grade       {0.0};
    double bank        {0.0};
    double rough_amp   {0.0};
    double rough_wl    {4.0};
    int    iso_class   {-1};
    std::string terrain;
    std::string sensors;
    double sensor_delay {0.0};
};

// stunt.ground: "ramp" | "loop" (L5). YAML keys match configs/scenes/*_demo.yaml.
struct StuntConfig {
    std::string ground;
    double ramp_x_start {20.0};
    double ramp_x_top   {24.0};
    double ramp_height  {0.6};
    double ramp_lip     {0.4};
    double loop_center_x {50.0};
    double loop_center_z {15.0};
    double loop_radius   {10.0};
    bool   rail_guide    {false};
};

// One sensor exactly as the scene declares it (agents.vehicle.sensors[], see
// docs/CONFIG_GUIDE.md §2.3):
//   - { id: gnss, type: gnss, mount: { pos: [1.4,0,1.0], rpy: [0,0,0] }, rate: 10 }
//   - { id: gnss, type: gnss, mount: [1.4,0,1.0], yaw: 0, rate: 10 }   (builder form)
// This is the mounting half of the declaration. The measurement-noise half of the
// same list entry (noise_std/bias/bias_rw) goes to VehicleSpawn::sensors instead,
// because vdsim::SensorParams models noise per signal group, not per mounted device.
//
// Nothing reads the mount pose or rate yet: they are parsed and stored for the
// future render / sensor-frame coupling and do not affect the simulation.
struct SceneSensor {
    std::string id;      // label ("gnss_roof"); defaults to `type` when omitted. Ids the
                         // scene writes explicitly must be unique within a vehicle; a
                         // defaulted one is a display label and may repeat.
    std::string type;    // gnss | imu | wheel_speed | steer | camera | lidar, plus the
                         // gnss_pos / gnss_vel / imu_accel / imu_gyro sub-signals
    std::array<double, 3> mount_pos {{0.0, 0.0, 0.0}};  // body frame [m]: x fwd, y left, z up
    std::array<double, 3> mount_rpy {{0.0, 0.0, 0.0}};  // body frame [rad]: roll, pitch, yaw
    double rate {0.0};   // declared sample rate [Hz]; 0 = unspecified (core runs at sim rate)
    // Type-specific knobs (camera fov_deg, lidar channels, ...) from an explicit
    // `params:` sub-map. A map, not fields: the set differs per sensor type and is
    // still growing. The well-known fields above stay typed.
    std::map<std::string, double> params;
};

struct VehicleSpawn {
    uint32_t    id {0};
    std::string vehicle_yaml;
    std::string tire_yaml;
    std::string tire_rear_yaml;
    std::string tire_fr_yaml;
    std::string tire_rl_yaml;
    std::string tire_rr_yaml;
    std::string level {"L2"};
    std::string front_susp;
    std::string rear_susp;
    double x0 {0.0}, y0 {0.0}, z0 {0.0}, yaw0 {0.0}, vx0 {0.0};
    // "external" (default): driven over UDP comms, failsafe on timeout.
    // "internal": built-in controller (v1 = speed-hold cruise at vx0).
    std::string control {"external"};
    // Per-vehicle sensor noise spec (from agents.vehicle.sensors[] in the scene).
    // When set, overrides the scenario-level RoadConfig.sensors file path. Set only
    // when the declaration actually specifies noise (a noise key on an entry that has
    // a measurement model, an explicit enabled/seed, or the sensors-file form): a
    // mount-only `sensors:` block leaves this empty so the vehicle keeps the
    // scenario-level file rather than silently running clean beside its neighbours.
    std::optional<vdsim::SensorParams> sensors;
    // Mount/rate declarations from that same agents.vehicle.sensors[] list, one
    // entry per declared device. Empty when the vehicle uses the sensors-file form
    // (a sensors yaml carries noise only, no mount pose). Stored, not yet consumed.
    std::vector<SceneSensor> scene_sensors;
    // Per-vehicle reference path for internal path-follow controller.
    // path_yaml: trajectory file ({points:[[x,y,vx],...]} — pure path data, no controller params)
    // path_lookahead: pure-pursuit lookahead distance [m] (controller param, not part of trajectory)
    std::string path_yaml;
    double path_lookahead {8.0};
};

// Scenario-level comms routing (realtime). A scene owns one comms spec; each
// channel is either TX (a source fans out to one or more ip:port destinations)
// or RX (direction:in, a listen port feeds control into the sim).
struct CommsDest {
    std::string ip {"127.0.0.1"};
    int         port {0};
};

struct CommsChannel {
    bool        rx {false};         // true = direction:in (listen for cmd)
    std::string source;             // tx: "<id>.state" | "ego.state" | "<id>.sensor.<x>"
    std::string templ {"vds1"};     // template/규약: vds1 | vds1_cmd | json | nmea_gga
    int         listen_port {0};    // rx: udp port to bind
    std::vector<CommsDest> to;      // tx: fan-out destinations
    // Geodetic datum the sim's ENU metres are referenced to, for templates that
    // emit lat/lon (nmea_gga). YAML: origin: {lat: 37.5, lon: 127.0, alt: 38.0}.
    // Optional; defaults to (0,0,0) so existing scenes keep working unchanged.
    GeodeticOrigin origin;
};

struct CommsConfig {
    std::string name;
    std::vector<CommsChannel> channels;
    bool empty() const { return channels.empty(); }
};

struct WorldScenario {
    SimConfig sim;                  // timing + solver config (dt, rate, t_end, time_scale, ...)
    double cmd_timeout {0.1};       // UDP command timeout before failsafe [s]
    RoadConfig road;
    StuntConfig stunt;
    std::vector<VehicleSpawn> vehicles;
    CommsConfig comms;
};

WorldScenario load_world_scenario(const std::string& path);

// The sensor params actually in force for one spawn. Precedence, as documented on
// VehicleSpawn::sensors: a per-vehicle spec wins outright over the scenario-level
// default (the RoadConfig::sensors file, already loaded by the caller).
vdsim::SensorParams effective_sensor_params(const vdsim::SensorParams& scenario_default,
                                            const VehicleSpawn& v);

}  // namespace vdsim::cosim
