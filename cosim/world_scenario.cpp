#include "world_scenario.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace vdsim::cosim {

static double node_d(const YAML::Node& n, const char* k, double d) {
    return n[k] ? n[k].as<double>() : d;
}

// ---------------------------------------------------------------------------
// agents.vehicle.sensors[] — see docs/CONFIG_GUIDE.md §2.3.
//
// One declaration carries two independent things, and this parser splits them:
//   * measurement noise  -> vdsim::SensorParams (one per vehicle, per signal group)
//   * mount pose + rate  -> SceneSensor         (one per declared device)
// Accepted shapes for the `sensors:` node on a vehicle entry:
//   sensors: [ {...}, {...} ]                       sequence of entries
//   sensors: { enabled: false, seed: 7, list: [..] } suite form (adds enabled/seed)
//   sensors: configs/sensors/noisy.yaml              a SensorParams file path
// Every other shape, key or type is a mistake we refuse to guess at: it throws.
// ---------------------------------------------------------------------------

// The two halves of one vehicle's sensor declaration.
struct SensorSuite {
    vdsim::SensorParams      params;
    std::vector<SceneSensor> mounts;
};

// Errors name the vehicle and the offending entry, in the same "world scenario: ..."
// shape the rest of this file throws. `where` is "" or "[2] (id=cam)".
[[noreturn]] static void sensor_throw(uint32_t vid, const std::string& where,
                                      const std::string& what) {
    throw std::runtime_error("world scenario: vehicle " + std::to_string(vid)
                             + " sensors" + where + ": " + what);
}

// Best-effort rendering of a node for an error message.
static std::string node_text(const YAML::Node& n) {
    if (n.IsScalar())   return n.Scalar();
    if (n.IsSequence()) return "<sequence of " + std::to_string(n.size()) + ">";
    if (n.IsMap())      return "<map>";
    return "<null>";
}

static double sensor_num(const YAML::Node& n, uint32_t vid, const std::string& where,
                         const std::string& key) {
    try {
        return n.as<double>();
    } catch (const YAML::Exception&) {
        sensor_throw(vid, where, key + " must be a number, got '" + node_text(n) + "'");
    }
}

// mount.pos / mount.rpy: exactly three numbers.
static std::array<double, 3> parse_vec3(const YAML::Node& n, uint32_t vid,
                                        const std::string& where, const std::string& key) {
    if (!n.IsSequence() || n.size() != 3)
        sensor_throw(vid, where, key + " must be a sequence of 3 numbers, got '"
                                 + node_text(n) + "'");
    std::array<double, 3> v {{0.0, 0.0, 0.0}};
    for (std::size_t i = 0; i < 3; ++i)
        v[i] = sensor_num(n[i], vid, where, key + "[" + std::to_string(i) + "]");
    return v;
}

// mount: { pos: [x,y,z], rpy: [r,p,y] } — both optional, default zero.
static void parse_mount(const YAML::Node& m, uint32_t vid, const std::string& where,
                        SceneSensor& out) {
    if (!m.IsMap())
        sensor_throw(vid, where, "mount must be a map { pos: [x,y,z], rpy: [r,p,y] }, got '"
                                 + node_text(m) + "'");
    for (const auto& kv : m) {
        const std::string k = kv.first.as<std::string>();
        if (k != "pos" && k != "rpy")
            sensor_throw(vid, where, "unknown mount key '" + k + "' (accepted: pos, rpy)");
    }
    if (m["pos"]) out.mount_pos = parse_vec3(m["pos"], vid, where, "mount.pos");
    if (m["rpy"]) out.mount_rpy = parse_vec3(m["rpy"], vid, where, "mount.rpy");
}

// params: { fov_deg: 90, ... } — an explicit bag of type-specific numeric knobs.
static void parse_sensor_params_map(const YAML::Node& p, uint32_t vid,
                                    const std::string& where, SceneSensor& out) {
    if (!p.IsMap())
        sensor_throw(vid, where, "params must be a map of numbers, got '" + node_text(p) + "'");
    for (const auto& kv : p) {
        const std::string k = kv.first.as<std::string>();
        out.params[k] = sensor_num(kv.second, vid, where, "params." + k);
    }
}

// Route a sensor type onto the SensorParams noise fields it drives. Returns false
// for an unknown type. camera/lidar are declaration-only (mount + rate): the core
// has no measurement model for them, so they map to no noise field at all.
static bool noise_targets_for(const std::string& type, vdsim::SensorParams& sp,
                              std::vector<vdsim::SensorNoise*>& out) {
    if      (type == "gnss")        out = {&sp.gnss_pos, &sp.gnss_vel};
    else if (type == "gnss_pos")    out = {&sp.gnss_pos};
    else if (type == "gnss_vel")    out = {&sp.gnss_vel};
    else if (type == "imu")         out = {&sp.imu_accel, &sp.imu_gyro};
    else if (type == "imu_accel")   out = {&sp.imu_accel};
    else if (type == "imu_gyro")    out = {&sp.imu_gyro};
    else if (type == "wheel_speed") out = {&sp.wheel_speed};
    else if (type == "steer")       out = {&sp.steer};
    else if (type == "camera" || type == "lidar") out.clear();
    else return false;
    return true;
}

static const char* const kSensorTypes =
    "gnss, gnss_pos, gnss_vel, imu, imu_accel, imu_gyro, wheel_speed, steer, camera, lidar";
static const char* const kSensorKeys =
    "id, type, mount, rate, noise_std, bias, bias_rw, params";

static bool is_known_sensor_key(const std::string& k) {
    return k == "id" || k == "type" || k == "mount" || k == "rate"
        || k == "noise_std" || k == "bias" || k == "bias_rw" || k == "params";
}

// Parse the sequence of entries into both halves of `suite`.
static void parse_sensors_seq(const YAML::Node& list, uint32_t vid, SensorSuite& suite) {
    std::set<std::string> seen_ids;
    std::size_t index = 0;
    for (const auto& item : list) {
        const std::string at = "[" + std::to_string(index++) + "]";
        if (!item.IsMap())
            sensor_throw(vid, at, "entry must be a map { id, type, mount, rate, ... }, got '"
                                  + node_text(item) + "'");

        SceneSensor sensor;
        if (item["id"]) {
            if (!item["id"].IsScalar())
                sensor_throw(vid, at, "id must be a string, got '" + node_text(item["id"]) + "'");
            sensor.id = item["id"].Scalar();
        }
        // Everything past this point can name the sensor in its error message.
        const std::string where = sensor.id.empty() ? at : at + " (id=" + sensor.id + ")";

        for (const auto& kv : item) {
            const std::string k = kv.first.as<std::string>();
            if (!is_known_sensor_key(k))
                sensor_throw(vid, where, "unknown key '" + k + "' (accepted: "
                                         + std::string(kSensorKeys) + ")");
        }
        if (!item["type"] || !item["type"].IsScalar())
            sensor_throw(vid, where, "missing required key 'type' (accepted: "
                                     + std::string(kSensorTypes) + ")");
        sensor.type = item["type"].Scalar();

        std::vector<vdsim::SensorNoise*> targets;
        if (!noise_targets_for(sensor.type, suite.params, targets))
            sensor_throw(vid, where, "unknown type '" + sensor.type + "' (accepted: "
                                     + std::string(kSensorTypes) + ")");

        const bool has_noise = item["noise_std"] || item["bias"] || item["bias_rw"];
        if (has_noise && targets.empty())
            sensor_throw(vid, where, "type '" + sensor.type + "' has no measurement model; "
                                     "noise_std/bias/bias_rw do not apply to it");
        if (has_noise) {
            vdsim::SensorNoise n;
            if (item["noise_std"]) n.noise_std = sensor_num(item["noise_std"], vid, where, "noise_std");
            if (item["bias"])      n.bias      = sensor_num(item["bias"],      vid, where, "bias");
            if (item["bias_rw"])   n.bias_rw   = sensor_num(item["bias_rw"],   vid, where, "bias_rw");
            for (auto* t : targets) *t = n;
        }

        if (sensor.id.empty()) sensor.id = sensor.type;
        if (!seen_ids.insert(sensor.id).second)
            sensor_throw(vid, where, "duplicate sensor id '" + sensor.id
                                     + "'; give each entry a unique id");
        if (item["mount"]) parse_mount(item["mount"], vid, where, sensor);
        if (item["rate"])  sensor.rate = sensor_num(item["rate"], vid, where, "rate");
        if (item["params"]) parse_sensor_params_map(item["params"], vid, where, sensor);
        suite.mounts.push_back(std::move(sensor));
    }
}

// Dispatch on the shape of a vehicle's `sensors:` node. Anything unrecognised throws
// rather than leaving the vehicle silently sensorless.
static SensorSuite parse_sensors_node(const YAML::Node& sn, uint32_t vid) {
    SensorSuite suite;
    if (sn.IsSequence()) {
        suite.params.enabled = true;
        parse_sensors_seq(sn, vid, suite);
    } else if (sn.IsMap()) {
        for (const auto& kv : sn) {
            const std::string k = kv.first.as<std::string>();
            if (k != "enabled" && k != "seed" && k != "list")
                sensor_throw(vid, "", "unknown key '" + k
                                      + "' (the suite form accepts: enabled, seed, list)");
        }
        if (!sn["list"] || !sn["list"].IsSequence())
            sensor_throw(vid, "", "suite form needs 'list:' holding a sequence of sensor entries");
        suite.params.enabled = true;
        parse_sensors_seq(sn["list"], vid, suite);
        try {
            if (sn["enabled"]) suite.params.enabled = sn["enabled"].as<bool>();
            if (sn["seed"])    suite.params.seed    = sn["seed"].as<unsigned>();
        } catch (const YAML::Exception&) {
            sensor_throw(vid, "", "enabled must be a bool and seed a non-negative integer");
        }
    } else if (sn.IsScalar()) {
        try {
            suite.params = vdsim::SensorParams::from_yaml(sn.Scalar());
        } catch (const std::exception&) {
            sensor_throw(vid, "", "sensors file not loadable: " + sn.Scalar());
        }
    } else {
        sensor_throw(vid, "", "must be a sequence of entries, a suite map "
                              "{enabled, seed, list}, or a sensors yaml path");
    }
    return suite;
}

vdsim::SensorParams effective_sensor_params(const vdsim::SensorParams& scenario_default,
                                            const VehicleSpawn& v) {
    return v.sensors ? *v.sensors : scenario_default;
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
        // origin: {lat, lon, alt} — datum for lat/lon templates (nmea_gga).
        if (ch["origin"]) {
            const auto& og = ch["origin"];
            cm.origin.lat_deg = node_d(og, "lat", 0.0);
            cm.origin.lon_deg = node_d(og, "lon", 0.0);
            cm.origin.alt_m   = node_d(og, "alt", 0.0);
        }
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
    w.cmd_timeout = node_d(root, "cmd_timeout", 0.1);
    // Parse sim: {dt, rate, t_end, time_scale, max_substep_dt, max_substeps, stunt_physics}
    // Fallback to top-level rate/time_scale for backward compat.
    if (root["sim"]) {
        const auto& sim_node = root["sim"];
        w.sim.dt            = node_d(sim_node, "dt", 0.005);
        w.sim.rate          = node_d(sim_node, "rate", 200.0);
        w.sim.t_end         = node_d(sim_node, "t_end", 0.0);
        w.sim.time_scale    = node_d(sim_node, "time_scale", 1.0);
        w.sim.max_substep_dt = node_d(sim_node, "max_substep_dt", 1e-4);
        if (sim_node["max_substeps"]) w.sim.max_substeps = sim_node["max_substeps"].as<int>();
        if (sim_node["stunt_physics"]) w.sim.stunt_physics = sim_node["stunt_physics"].as<bool>();
    } else {
        // fallback: top-level rate / time_scale (deprecated)
        w.sim.rate = node_d(root, "rate", 200.0);
        w.sim.time_scale = node_d(root, "time_scale", 1.0);
    }
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
        if (v["tire_rear"]) s.tire_rear_yaml = v["tire_rear"].as<std::string>();
        if (v["tire_fr"])   s.tire_fr_yaml   = v["tire_fr"].as<std::string>();
        if (v["tire_rl"])   s.tire_rl_yaml   = v["tire_rl"].as<std::string>();
        if (v["tire_rr"])   s.tire_rr_yaml   = v["tire_rr"].as<std::string>();
        if (v["level"]) s.level = v["level"].as<std::string>();
        if (v["control"]) s.control = v["control"].as<std::string>();
        if (v["front_susp"]) s.front_susp = v["front_susp"].as<std::string>();
        if (v["rear_susp"]) s.rear_susp = v["rear_susp"].as<std::string>();
        if (v["path"])           s.path_yaml      = v["path"].as<std::string>();
        if (v["path_lookahead"]) s.path_lookahead = v["path_lookahead"].as<double>();
        // per-vehicle sensors: inline list / suite map / a sensors.yaml file path.
        // The list form fills both the noise model and the mount declarations.
        // IsDefined(), not truthiness: a bare `sensors:` with nothing under it is a
        // null node, and must be reported rather than treated as "no sensors".
        if (v["sensors"].IsDefined()) {
            auto suite = parse_sensors_node(v["sensors"], s.id);
            s.sensors       = std::move(suite.params);
            s.scene_sensors = std::move(suite.mounts);
        }
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
