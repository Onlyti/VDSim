#pragma once

#include "world_scenario.hpp"

#include <string>

namespace vdsim::cosim {

// Load a catalog scene (fleet[] + blueprint) or a materialized world (vehicles[]).
WorldScenario load_scene(const std::string& path);

}  // namespace vdsim::cosim
