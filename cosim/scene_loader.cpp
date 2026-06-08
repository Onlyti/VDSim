#include "scene_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <random>
#include <sstream>
#include <stdexcept>

#ifndef VDSIM_SOURCE_DIR
#define VDSIM_SOURCE_DIR "."
#endif
#ifndef VDSIM_PYTHON
#define VDSIM_PYTHON "python3"
#endif

namespace vdsim::cosim {

static bool is_catalog_scene(const std::string& path) {
    const YAML::Node root = YAML::LoadFile(path);
    const auto veh = root["vehicles"];
    if (veh && veh.IsSequence() && veh.size() > 0)
        return false;
    const auto fleet = root["fleet"];
    return fleet && fleet.IsSequence() && fleet.size() > 0;
}

static std::string temp_world_path() {
    const auto dir = std::filesystem::temp_directory_path();
    std::random_device rd;
    const auto p = dir / ("vdsim_world_" + std::to_string(rd()) + ".yaml");
    return p.string();
}

static std::string shell_quote(const std::string& s) {
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    out += '\'';
    return out;
}

static std::string materialize_catalog_scene(const std::string& scene_path) {
    const std::string out = temp_world_path();
    const std::string script = std::string(VDSIM_SOURCE_DIR) + "/tools/materialize_scene.py";
    std::ostringstream cmd;
    cmd << VDSIM_PYTHON << ' ' << shell_quote(script) << ' '
        << shell_quote(scene_path) << ' ' << shell_quote(out);
    const int rc = std::system(cmd.str().c_str());
    if (rc != 0 || !std::filesystem::is_regular_file(out))
        throw std::runtime_error("catalog scene materialize failed: " + scene_path);
    return out;
}

WorldScenario load_scene(const std::string& path) {
    if (is_catalog_scene(path))
        return load_world_scenario(materialize_catalog_scene(path));
    return load_world_scenario(path);
}

}  // namespace vdsim::cosim
