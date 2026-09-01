#include "world_scenario.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
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
// An entry itself has two accepted mount spellings, both of which the repo
// publishes: the canonical `mount: {pos: [x,y,z], rpy: [r,p,y]}` (CONFIG_GUIDE
// §2.3) and the builder's `mount: [x,y,z]` + `yaw: <deg>` (builder/README.md,
// builder/index.html addSensor()). Every other shape, key or type is a mistake
// we refuse to guess at: it throws. docs/CONFIG_GUIDE.md §2.3.1 lists them.
// ---------------------------------------------------------------------------

// The two halves of one vehicle's sensor declaration, plus whether the
// declaration said anything at all about measurement noise.
struct SensorSuite {
    vdsim::SensorParams      params;
    std::vector<SceneSensor> mounts;
    // True only when the block actually specifies noise: a noise key on an entry
    // that has a measurement model, or an explicit `enabled:`/`seed:`, or the
    // sensors-file form. A mount-only declaration leaves this false so the
    // scenario-level `sensors:` file stays in force for that vehicle instead of
    // being silently replaced by an all-zero SensorParams.
    bool overrides_noise {false};
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

// Range constraint a numeric field must satisfy on top of being finite. Every
// number this parser reads is checked: a NaN noise_std turns every measured
// channel into NaN at runtime, and a negative rate/noise_std is meaningless.
enum class NumRange { Any, NonNegative };

static double sensor_num(const YAML::Node& n, uint32_t vid, const std::string& where,
                         const std::string& key, NumRange range = NumRange::Any) {
    double v = 0.0;
    try {
        v = n.as<double>();
    } catch (const YAML::Exception&) {
        sensor_throw(vid, where, key + " must be a number, got '" + node_text(n) + "'");
    }
    if (!std::isfinite(v))
        sensor_throw(vid, where, key + " must be finite, got '" + node_text(n) + "'");
    if (range == NumRange::NonNegative && v < 0.0)
        sensor_throw(vid, where, key + " must be >= 0, got '" + node_text(n) + "'");
    return v;
}

// mount.pos / mount.rpy: exactly three finite numbers.
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

// mount: { pos: [x,y,z], rpy: [r,p,y] }  — canonical form, both keys optional.
// mount: [x, y, z]                       — the builder's form (builder/README.md):
//                                          position only, heading comes from `yaw:`.
// Sets `rpy_given` when the declaration fixed the orientation here, so the caller
// can reject a `yaw:` that would fight with it.
static void parse_mount(const YAML::Node& m, uint32_t vid, const std::string& where,
                        SceneSensor& out, bool& rpy_given) {
    if (m.IsSequence()) {
        out.mount_pos = parse_vec3(m, vid, where, "mount");
        return;
    }
    if (!m.IsMap())
        sensor_throw(vid, where, "mount must be a map { pos: [x,y,z], rpy: [r,p,y] } "
                                 "or a sequence [x,y,z], got '" + node_text(m) + "'");
    for (const auto& kv : m) {
        const std::string k = kv.first.as<std::string>();
        if (k != "pos" && k != "rpy")
            sensor_throw(vid, where, "unknown mount key '" + k + "' (accepted: pos, rpy)");
    }
    if (m["pos"].IsDefined()) out.mount_pos = parse_vec3(m["pos"], vid, where, "mount.pos");
    if (m["rpy"].IsDefined()) {
        out.mount_rpy = parse_vec3(m["rpy"], vid, where, "mount.rpy");
        rpy_given = true;
    }
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
// has no measurement model for them, so they map to no noise field at all, and a
// noise key on them is accepted and ignored (the builder emits noise_std for every
// type it can author, see builder/index.html addSensor()).
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
    "id, type, mount, yaw, rate, noise_std, bias, bias_rw, params";

static bool is_known_sensor_key(const std::string& k) {
    return k == "id" || k == "type" || k == "mount" || k == "yaw" || k == "rate"
        || k == "noise_std" || k == "bias" || k == "bias_rw" || k == "params";
}

// Parse the sequence of entries into both halves of `suite`.
static void parse_sensors_seq(const YAML::Node& list, uint32_t vid, SensorSuite& suite) {
    // Uniqueness is enforced only over ids the scene actually wrote. A default id
    // (the type name, filled in below) is a display label, not a user promise, so
    // two `{type: gnss}` entries — or a `{type: imu}` next to an unrelated
    // `{id: imu, ...}` — must not be rejected as a "duplicate".
    std::set<std::string> explicit_ids;
    std::size_t index = 0;
    for (const auto& item : list) {
        const std::string at = "[" + std::to_string(index++) + "]";
        if (!item.IsMap())
            sensor_throw(vid, at, "entry must be a map { id, type, mount, rate, ... }, got '"
                                  + node_text(item) + "'");

        SceneSensor sensor;
        if (item["id"].IsDefined()) {
            if (!item["id"].IsScalar())
                sensor_throw(vid, at, "id must be a string, got '" + node_text(item["id"]) + "'");
            // Taken as the literal scalar text: `id: true` is the id "true",
            // `id: 12` is "12". Quote it to be unambiguous.
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
        if (!item["type"].IsDefined())
            sensor_throw(vid, where, "missing required key 'type' (accepted: "
                                     + std::string(kSensorTypes) + ")");
        if (!item["type"].IsScalar())
            sensor_throw(vid, where, "type must be one of the type names, got '"
                                     + node_text(item["type"]) + "' (accepted: "
                                     + std::string(kSensorTypes) + ")");
        sensor.type = item["type"].Scalar();

        std::vector<vdsim::SensorNoise*> targets;
        if (!noise_targets_for(sensor.type, suite.params, targets))
            sensor_throw(vid, where, "unknown type '" + sensor.type + "' (accepted: "
                                     + std::string(kSensorTypes) + ")");

        if (item["noise_std"].IsDefined() || item["bias"].IsDefined()
            || item["bias_rw"].IsDefined()) {
            vdsim::SensorNoise n;
            if (item["noise_std"].IsDefined())
                n.noise_std = sensor_num(item["noise_std"], vid, where, "noise_std",
                                         NumRange::NonNegative);
            if (item["bias"].IsDefined())
                n.bias = sensor_num(item["bias"], vid, where, "bias");
            if (item["bias_rw"].IsDefined())
                n.bias_rw = sensor_num(item["bias_rw"], vid, where, "bias_rw");
            // targets is empty for camera/lidar: the numbers are still validated,
            // then dropped, because the core has no measurement model to feed.
            for (auto* t : targets) *t = n;
            if (!targets.empty()) suite.overrides_noise = true;
        }
        // No noise key: the entry declares a mount only, so it writes nothing into
        // SensorParams. It must not zero a group an earlier entry (or the
        // scenario-level file) already set — `{type: gnss, noise_std: 0.5}` followed
        // by `{type: gnss_pos}` keeps gnss_pos at 0.5.

        if (!sensor.id.empty() && !explicit_ids.insert(sensor.id).second)
            sensor_throw(vid, where, "duplicate sensor id '" + sensor.id
                                     + "'; give each entry a unique id");
        if (sensor.id.empty()) sensor.id = sensor.type;
        bool rpy_given = false;
        if (item["mount"].IsDefined()) parse_mount(item["mount"], vid, where, sensor, rpy_given);
        if (item["yaw"].IsDefined()) {
            if (rpy_given)
                sensor_throw(vid, where, "yaw and mount.rpy both set the heading; give only one");
            // The builder authors yaw in degrees (builder/index.html: "yaw [deg]");
            // SceneSensor::mount_rpy is radians.
            constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
            sensor.mount_rpy[2] = sensor_num(item["yaw"], vid, where, "yaw") * kDegToRad;
        }
        if (item["rate"].IsDefined())
            sensor.rate = sensor_num(item["rate"], vid, where, "rate", NumRange::NonNegative);
        if (item["params"].IsDefined()) parse_sensor_params_map(item["params"], vid, where, sensor);
        suite.mounts.push_back(std::move(sensor));
    }
}

// `sensors: <path>` — a SensorParams yaml. The path is used as written (absolute,
// or relative to the process CWD); if that names no file, it is retried relative to
// the directory of the scene file, which is what a scene-local path means to a user.
static vdsim::SensorParams load_sensors_file(const std::string& raw, uint32_t vid,
                                             const std::string& scene_dir) {
    if (raw.empty())
        sensor_throw(vid, "", "empty value; expected a sensors yaml path, a sequence of "
                              "entries, or a suite map {enabled, seed, list}");
    std::error_code ec;
    std::string chosen = raw;
    std::string alt;
    const std::filesystem::path p(raw);
    if (p.is_relative() && !scene_dir.empty()) {
        if (!std::filesystem::is_regular_file(p, ec)) {
            alt = (std::filesystem::path(scene_dir) / p).lexically_normal().string();
            if (std::filesystem::is_regular_file(alt, ec)) chosen = alt;
        }
    }
    try {
        return vdsim::SensorParams::from_yaml(chosen);
    } catch (const std::exception& e) {
        // Nest the underlying reason: yaml-cpp reports the line and column, and
        // dropping it makes a malformed sensors file strictly harder to debug.
        std::string msg = "sensors file not loadable: '" + chosen + "'";
        if (!alt.empty() && alt != chosen) msg += " (also tried '" + alt + "')";
        sensor_throw(vid, "", msg + ": " + e.what());
    }
}

/// @brief Parse one vehicle's sensor declaration in list, suite, or file form.
/// @param sn YAML node stored at the vehicle's `sensors:` key.
/// @param vid Vehicle id included in diagnostic messages.
/// @param scene_dir Directory used to resolve a relative sensor-parameter file.
/// @return Parsed measurement-noise override and mounted sensor declarations.
/// @throws std::runtime_error if the shape, key, type, number, or referenced file
///         violates the scene sensor contract.
static SensorSuite parse_sensors_node(const YAML::Node& sn, uint32_t vid,
                                      const std::string& scene_dir) {
    SensorSuite suite;
    if (sn.IsSequence()) {
        parse_sensors_seq(sn, vid, suite);
        if (suite.overrides_noise) suite.params.enabled = true;
    } else if (sn.IsMap()) {
        for (const auto& kv : sn) {
            const std::string k = kv.first.as<std::string>();
            if (k != "enabled" && k != "seed" && k != "list")
                sensor_throw(vid, "", "unknown key '" + k
                                      + "' (the suite form accepts: enabled, seed, list)");
        }
        if (!sn["list"].IsDefined() || !sn["list"].IsSequence())
            sensor_throw(vid, "", "suite form needs 'list:' holding a sequence of sensor entries");
        parse_sensors_seq(sn["list"], vid, suite);
        if (suite.overrides_noise) suite.params.enabled = true;
        // Writing enabled/seed is itself a statement about the noise model, so the
        // suite then overrides the scenario-level file even with a mount-only list.
        if (sn["enabled"].IsDefined() || sn["seed"].IsDefined())
            suite.overrides_noise = true;
        try {
            if (sn["enabled"].IsDefined()) suite.params.enabled = sn["enabled"].as<bool>();
            if (sn["seed"].IsDefined())    suite.params.seed    = sn["seed"].as<unsigned>();
        } catch (const YAML::Exception&) {
            sensor_throw(vid, "", "enabled must be a bool and seed a non-negative integer");
        }
    } else if (sn.IsScalar()) {
        suite.params = load_sensors_file(sn.Scalar(), vid, scene_dir);
        suite.overrides_noise = true;
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
    // Fallback base for a relative `sensors:` path on a vehicle entry (see
    // load_sensors_file): the directory holding this scene/world file.
    const std::string scene_dir = std::filesystem::path(path).parent_path().string();
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
        // IsDefined() is spelled out for the reader, not for yaml-cpp: Node's
        // operator bool *is* IsDefined() (yaml-cpp node/node.h), so the two are the
        // same test. A bare `sensors:` is a defined null node either way, and it is
        // parse_sensors_node's final else branch that reports it.
        if (v["sensors"].IsDefined()) {
            auto suite = parse_sensors_node(v["sensors"], s.id, scene_dir);
            // Only a block that actually specifies noise replaces the scenario-level
            // `sensors:` file. A mount-only declaration must not opt the vehicle out
            // of the noise model its neighbours run with.
            if (suite.overrides_noise) s.sensors = std::move(suite.params);
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
