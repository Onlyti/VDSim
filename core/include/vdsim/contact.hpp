#pragma once

#include <array>

#include "vdsim/types.hpp"

namespace vdsim {

// Single wheel contact result (one raycast).
struct ContactPoint {
    Vec3   position    {Vec3::Zero()};   // world [m]
    Vec3   normal      {Vec3::UnitZ()};  // unit, points away from surface
    int    surface_id  {0};              // material lookup id
    bool   is_valid    {false};          // false: wheel off ground
    double mu_long     {1.0};            // Pacejka mu_x scaling [-]
    double mu_lat      {1.0};            // Pacejka mu_y scaling [-]
    double penetration {0.0};            // [m] from suspension nominal (+: compressed)
    double road_dz     {0.0};            // [m] road height deviation for ride
                                         //     (roughness; 0 on smooth surfaces)
};

using ContactArray = std::array<ContactPoint, NUM_WHEELS>;

}  // namespace vdsim
