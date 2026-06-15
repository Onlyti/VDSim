#include "world_scenario.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace vdsim::cosim {

static double node_d(const YAML::Node& n, const char* k, double d) {
    return n[k] ? n[k].as<double>() : d;
}

// Parse a comms document node ({name, channels: [...]}) into CommsConfig.
static CommsConfig parse_comms_node(const YAML::Node& c) {
    CommsConfig cc;
    if (c["name"]) cc.name = c["name"].as<std::string>();
    const auto chans = c["channels"];
    if (!chans || !chans.IsSequence()) return cc;
    for (const auto& ch : chans) {
        CommsChannel cm;
        const bool is_in = ch["direction"] &&
            ch["direction"].as<std::string>() == "in";
        cm.rx = is_in || ch["listen"];
        if (ch["source"])   cm.source = ch["source"].as<std::string>();
        if (ch["template"]) cm.templ  = ch["template"].as<std::string>();
        if (ch["listen"] && ch["listen"]["port"])
            cm.listen_port = ch["listen"]["port"].as<int>();
        if (ch["to"] && ch["to"].IsSequence()) {
            for (const auto& d : ch["to"]) {
                CommsDest dst;
                if (d["ip"])   dst.ip   = d["ip"].as<std::string>();
                if (d["port"]) dst.port = d["port"].as<int>();
                cm.to.push_back(dst);
            }
        }
        cc.channels.push_back(cm);
    }
    return cc;
}

WorldScenario load_world_scenario(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    WorldScenario w;
    w.rate        = node_d(root, "rate", 200.0);
    w.cmd_timeout = node_d(root, "cmd_timeout", 0.1);
    w.time_scale  = node_d(root, "time_scale", 1.0);
    if (root["mu"]) w.road.mu = root["mu"].as<double>();
    if (root["mu_right"]) w.road.mu_right = root["mu_right"].as<double>();
    if (root["mu_boundary"]) w.road.mu_boundary = root["mu_boundary"].as<double>();
    if (root["grade"]) w.road.grade = root["grade"].as<double>();
    if (root["bank"]) w.road.bank = root["bank"].as<double>();
    if (root["rough_amp"]) w.road.rough_amp = root["rough_amp"].as<double>();
    if (root["rough_wl"]) w.road.rough_wl = root["rough_wl"].as<double>();
    if (root["iso_class"]) w.road.iso_class = root["iso_class"].as<int>();
    if (root["terrain"]) w.road.terrain = root["terrain"].as<std::string>();
    if (root["sensors"]) w.road.sensors = root["sensors"].as<std::string>();
    if (root["sensor_delay"]) w.road.sensor_delay = root["sensor_delay"].as<double>();
    if (root["stunt"]) {
        const auto st = root["stunt"];
        if (st["ground"]) w.stunt.ground = st["ground"].as<std::string>();
        w.stunt.ramp_x_start  = node_d(st, "x_start", w.stunt.ramp_x_start);
        w.stunt.ramp_x_top    = node_d(st, "x_top", w.stunt.ramp_x_top);
        w.stunt.ramp_height   = node_d(st, "height_m", w.stunt.ramp_height);
        w.stunt.ramp_lip      = node_d(st, "lip_m", w.stunt.ramp_lip);
        w.stunt.loop_center_x = node_d(st, "center_x", w.stunt.loop_center_x);
        w.stunt.loop_center_z = node_d(st, "center_z", w.stunt.loop_center_z);
        w.stunt.loop_radius   = node_d(st, "radius_m", w.stunt.loop_radius);
        if (st["rail_guide"]) w.stunt.rail_guide = st["rail_guide"].as<bool>();
    }
    if (root["comms"]) {
        const auto cnode = root["comms"];
        if (cnode.IsMap()) {
            w.comms = parse_comms_node(cnode);          // inlined by materialize
        } else if (cnode.IsScalar()) {                  // direct world.yaml: a file path
            try {
                w.comms = parse_comms_node(YAML::LoadFile(cnode.as<std::string>()));
            } catch (const std::exception&) {
                throw std::runtime_error(
                    "world scenario: comms file not loadable: " + cnode.as<std::string>());
            }
        }
    }

    const auto veh = root["vehicles"];
    if (!veh || !veh.IsSequence() || veh.size() == 0)
        throw std::runtime_error("world scenario: missing vehicles[]");
    for (const auto& v : veh) {
        VehicleSpawn s;
        s.id = v["id"] ? v["id"].as<uint32_t>() : static_cast<uint32_t>(w.vehicles.size());
        if (!v["vehicle"] || !v["tire"])
            throw std::runtime_error("world scenario: vehicle entry needs vehicle+tire paths");
        s.vehicle_yaml = v["vehicle"].as<std::string>();
        s.tire_yaml    = v["tire"].as<std::string>();
        if (v["level"]) s.level = v["level"].as<std::string>();
        if (v["control"]) s.control = v["control"].as<std::string>();
        if (v["front_susp"]) s.front_susp = v["front_susp"].as<std::string>();
        if (v["rear_susp"]) s.rear_susp = v["rear_susp"].as<std::string>();
        s.x0   = node_d(v, "x0", 0.0);
        s.y0   = node_d(v, "y0", 0.0);
        s.z0   = node_d(v, "z0", 0.0);
        s.yaw0 = node_d(v, "yaw0", 0.0);
        s.vx0  = node_d(v, "vx0", 0.0);
        w.vehicles.push_back(s);
    }
    return w;
}

}  // namespace vdsim::cosim
