#pragma once

#include "vdsim/sensors.hpp"

#include <cstdint>
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
    // When set, overrides the scenario-level RoadConfig.sensors file path.
    std::optional<vdsim::SensorParams> sensors;
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

}  // namespace vdsim::cosim
