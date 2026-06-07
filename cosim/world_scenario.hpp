#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace vdsim::cosim {

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

struct VehicleSpawn {
    uint32_t    id {0};
    std::string vehicle_yaml;
    std::string tire_yaml;
    std::string level {"L2"};
    std::string front_susp;
    std::string rear_susp;
    double x0 {0.0}, y0 {0.0}, yaw0 {0.0}, vx0 {0.0};
};

struct WorldScenario {
    double rate        {200.0};
    double cmd_timeout {0.1};
    double time_scale  {1.0};
    RoadConfig road;
    std::vector<VehicleSpawn> vehicles;
};

WorldScenario load_world_scenario(const std::string& path);

}  // namespace vdsim::cosim
