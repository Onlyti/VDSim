// YAML I/O for VehicleParams / TireParams.
//
// Schema rule: top-level YAML keys map 1:1 to struct member names, with
// per-wheel arrays as 4-sequences ordered FL, FR, RL, RR.
//   - Missing keys keep the struct default (forward compatible).
//   - Wrong array length / bad enum string => std::runtime_error.
//   - Unknown extra keys are silently ignored.

#include "vdsim/params.hpp"
#include "vdsim/sensors.hpp"

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

namespace vdsim {

namespace {

template <typename T>
void pull(const YAML::Node& node, const char* key, T& dst) {
    const auto sub = node[key];
    if (!sub || sub.IsNull()) return;
    try {
        dst = sub.as<T>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(std::string("YAML field '") + key + "': " + e.what());
    }
}

void pull_array4(const YAML::Node& node, const char* key,
                 std::array<double, NUM_WHEELS>& dst) {
    const auto sub = node[key];
    if (!sub || sub.IsNull()) return;
    if (!sub.IsSequence() || sub.size() != NUM_WHEELS) {
        throw std::runtime_error(std::string("YAML field '") + key +
                                 "' must be a length-4 sequence [FL,FR,RL,RR]");
    }
    for (int i = 0; i < NUM_WHEELS; ++i) dst[i] = sub[i].as<double>();
}

void pull_vec3(const YAML::Node& node, const char* key, Vec3& dst) {
    const auto sub = node[key];
    if (!sub || sub.IsNull()) return;
    if (!sub.IsSequence() || sub.size() != 3) {
        throw std::runtime_error(std::string("YAML field '") + key +
                                 "' must be a length-3 sequence [x,y,z]");
    }
    dst.x() = sub[0].as<double>();
    dst.y() = sub[1].as<double>();
    dst.z() = sub[2].as<double>();
}

VehicleParams::Drive parse_drive(const std::string& s) {
    if (s == "FWD") return VehicleParams::Drive::FWD;
    if (s == "RWD") return VehicleParams::Drive::RWD;
    if (s == "AWD") return VehicleParams::Drive::AWD;
    throw std::runtime_error("YAML field 'drive_type': unknown value '" + s +
                             "' (expected FWD/RWD/AWD)");
}

VehicleParams::Differential parse_diff(const std::string& s) {
    if (s == "Open")   return VehicleParams::Differential::Open;
    if (s == "Locked") return VehicleParams::Differential::Locked;
    if (s == "LSD")    return VehicleParams::Differential::LSD;
    throw std::runtime_error("YAML field 'differential': unknown value '" + s +
                             "' (expected Open/Locked/LSD)");
}
const char* diff_to_string(VehicleParams::Differential d) {
    switch (d) {
        case VehicleParams::Differential::Open:   return "Open";
        case VehicleParams::Differential::Locked: return "Locked";
        case VehicleParams::Differential::LSD:    return "LSD";
    }
    return "Open";
}

const char* drive_to_string(VehicleParams::Drive d) {
    switch (d) {
        case VehicleParams::Drive::FWD: return "FWD";
        case VehicleParams::Drive::RWD: return "RWD";
        case VehicleParams::Drive::AWD: return "AWD";
    }
    return "RWD";
}

YAML::Node load_root(const std::string& path) {
    try {
        return YAML::LoadFile(path);
    } catch (const YAML::BadFile&) {
        throw std::runtime_error("Cannot open YAML file: " + path);
    } catch (const YAML::ParserException& e) {
        throw std::runtime_error("YAML parse error in '" + path + "': " + e.what());
    }
}

void write_seq_double(YAML::Emitter& out, const char* key,
                      const std::array<double, NUM_WHEELS>& a) {
    out << YAML::Key << key << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto v : a) out << v;
    out << YAML::EndSeq;
}

}  // namespace

// =============================================================================
// VehicleParams
// =============================================================================

VehicleParams VehicleParams::from_yaml(const std::string& path) {
    const auto root = load_root(path);
    VehicleParams p;

    pull(root, "mass",                p.mass);
    pull(root, "mass_sprung",         p.mass_sprung);
    pull_vec3(root, "inertia_diag",   p.inertia_diag);

    pull(root, "wheelbase",            p.wheelbase);
    pull(root, "cg_to_front",          p.cg_to_front);
    pull(root, "cg_to_rear",           p.cg_to_rear);
    pull(root, "track_front",          p.track_front);
    pull(root, "track_rear",           p.track_rear);
    pull(root, "cg_height",            p.cg_height);
    pull(root, "wheel_radius_nominal", p.wheel_radius_nominal);

    pull_array4(root, "spring_stiffness",   p.spring_stiffness);
    pull_array4(root, "damper_coefficient", p.damper_coefficient);
    pull_array4(root, "unsprung_mass",      p.unsprung_mass);
    pull_array4(root, "wheel_inertia",      p.wheel_inertia);
    pull(root, "arb_stiffness_front",  p.arb_stiffness_front);
    pull(root, "arb_stiffness_rear",   p.arb_stiffness_rear);
    pull(root, "roll_center_height_front", p.roll_center_height_front);
    pull(root, "roll_center_height_rear",  p.roll_center_height_rear);
    pull(root, "anti_dive_front",      p.anti_dive_front);
    pull(root, "anti_squat_rear",      p.anti_squat_rear);
    pull(root, "camber_per_roll",      p.camber_per_roll);

    if (const auto n = root["drive_type"]; n && !n.IsNull()) {
        p.drive_type = parse_drive(n.as<std::string>());
    }
    if (const auto n = root["differential"]; n && !n.IsNull()) {
        p.differential = parse_diff(n.as<std::string>());
    }
    pull(root, "lsd_preload", p.lsd_preload);
    pull(root, "lsd_ramp",    p.lsd_ramp);
    pull(root, "max_motor_torque", p.max_motor_torque);
    pull(root, "final_drive_ratio", p.final_drive_ratio);
    pull(root, "max_brake_torque", p.max_brake_torque);
    pull(root, "brake_bias_front",  p.brake_bias_front);
    pull(root, "brake_ebd_enabled", p.brake_ebd_enabled);

    pull(root, "steering_ratio",        p.steering_ratio);
    pull(root, "max_steer_angle_wheel", p.max_steer_angle_wheel);
    pull(root, "ackerman_percent",      p.ackerman_percent);

    pull(root, "aero_drag_coeff", p.aero_drag_coeff);
    pull(root, "frontal_area",    p.frontal_area);
    pull(root, "aero_lift_front", p.aero_lift_front);
    pull(root, "aero_lift_rear",  p.aero_lift_rear);

    // Self-consistency: cg_to_front + cg_to_rear should equal wheelbase.
    const double sum = p.cg_to_front + p.cg_to_rear;
    if (std::abs(sum - p.wheelbase) > 1e-6 * std::max(1.0, p.wheelbase)) {
        spdlog::warn("[vdsim params '{}'] cg_to_front + cg_to_rear ({}) != wheelbase ({})",
                     path, sum, p.wheelbase);
    }
    return p;
}

void VehicleParams::to_yaml(const std::string& path) const {
    YAML::Emitter out;
    out.SetIndent(2);
    out << YAML::BeginMap;

    out << YAML::Key << "mass"         << YAML::Value << mass;
    out << YAML::Key << "mass_sprung"  << YAML::Value << mass_sprung;
    out << YAML::Key << "inertia_diag" << YAML::Value
        << YAML::Flow << YAML::BeginSeq
        << inertia_diag.x() << inertia_diag.y() << inertia_diag.z()
        << YAML::EndSeq;

    out << YAML::Key << "wheelbase"            << YAML::Value << wheelbase;
    out << YAML::Key << "cg_to_front"          << YAML::Value << cg_to_front;
    out << YAML::Key << "cg_to_rear"           << YAML::Value << cg_to_rear;
    out << YAML::Key << "track_front"          << YAML::Value << track_front;
    out << YAML::Key << "track_rear"           << YAML::Value << track_rear;
    out << YAML::Key << "cg_height"            << YAML::Value << cg_height;
    out << YAML::Key << "wheel_radius_nominal" << YAML::Value << wheel_radius_nominal;

    write_seq_double(out, "spring_stiffness",   spring_stiffness);
    write_seq_double(out, "damper_coefficient", damper_coefficient);
    write_seq_double(out, "unsprung_mass",      unsprung_mass);
    write_seq_double(out, "wheel_inertia",      wheel_inertia);

    out << YAML::Key << "arb_stiffness_front"  << YAML::Value << arb_stiffness_front;
    out << YAML::Key << "arb_stiffness_rear"   << YAML::Value << arb_stiffness_rear;
    out << YAML::Key << "roll_center_height_front" << YAML::Value << roll_center_height_front;
    out << YAML::Key << "roll_center_height_rear"  << YAML::Value << roll_center_height_rear;
    out << YAML::Key << "anti_dive_front"      << YAML::Value << anti_dive_front;
    out << YAML::Key << "anti_squat_rear"      << YAML::Value << anti_squat_rear;
    out << YAML::Key << "camber_per_roll"      << YAML::Value << camber_per_roll;

    out << YAML::Key << "drive_type"        << YAML::Value << drive_to_string(drive_type);
    out << YAML::Key << "differential"      << YAML::Value << diff_to_string(differential);
    out << YAML::Key << "lsd_preload"       << YAML::Value << lsd_preload;
    out << YAML::Key << "lsd_ramp"          << YAML::Value << lsd_ramp;
    out << YAML::Key << "max_motor_torque"  << YAML::Value << max_motor_torque;
    out << YAML::Key << "final_drive_ratio" << YAML::Value << final_drive_ratio;
    out << YAML::Key << "max_brake_torque"  << YAML::Value << max_brake_torque;
    out << YAML::Key << "brake_bias_front"  << YAML::Value << brake_bias_front;
    out << YAML::Key << "brake_ebd_enabled" << YAML::Value << brake_ebd_enabled;

    out << YAML::Key << "steering_ratio"        << YAML::Value << steering_ratio;
    out << YAML::Key << "max_steer_angle_wheel" << YAML::Value << max_steer_angle_wheel;
    out << YAML::Key << "ackerman_percent"      << YAML::Value << ackerman_percent;

    out << YAML::Key << "aero_drag_coeff" << YAML::Value << aero_drag_coeff;
    out << YAML::Key << "frontal_area"    << YAML::Value << frontal_area;
    out << YAML::Key << "aero_lift_front" << YAML::Value << aero_lift_front;
    out << YAML::Key << "aero_lift_rear"  << YAML::Value << aero_lift_rear;

    out << YAML::EndMap;

    std::ofstream ofs(path);
    if (!ofs) throw std::runtime_error("Cannot open YAML file for write: " + path);
    ofs << out.c_str() << "\n";
}

// =============================================================================
// TireParams
// =============================================================================

TireParams TireParams::from_yaml(const std::string& path) {
    const auto root = load_root(path);
    TireParams p;
    pull(root, "B_long", p.B_long);
    pull(root, "C_long", p.C_long);
    pull(root, "D_long", p.D_long);
    pull(root, "E_long", p.E_long);
    pull(root, "B_lat",  p.B_lat);
    pull(root, "C_lat",  p.C_lat);
    pull(root, "D_lat",  p.D_lat);
    pull(root, "E_lat",  p.E_lat);
    pull(root, "mu_nominal",          p.mu_nominal);
    pull(root, "Fz_nominal",          p.Fz_nominal);
    pull(root, "cornering_stiffness", p.cornering_stiffness);
    pull(root, "rolling_resistance",  p.rolling_resistance);
    pull(root, "combined_slip_enabled", p.combined_slip_enabled);
    pull(root, "pneumatic_trail",       p.pneumatic_trail);
    pull(root, "trail_falloff_alpha",   p.trail_falloff_alpha);
    pull(root, "camber_stiffness",      p.camber_stiffness);
    pull(root, "load_sensitivity",      p.load_sensitivity);
    pull(root, "relaxation_length_lat", p.relaxation_length_lat);
    pull(root, "relaxation_length_long", p.relaxation_length_long);
    pull(root, "tire_vertical_stiffness", p.tire_vertical_stiffness);
    return p;
}

void TireParams::to_yaml(const std::string& path) const {
    YAML::Emitter out;
    out.SetIndent(2);
    out << YAML::BeginMap;
    out << YAML::Key << "B_long" << YAML::Value << B_long;
    out << YAML::Key << "C_long" << YAML::Value << C_long;
    out << YAML::Key << "D_long" << YAML::Value << D_long;
    out << YAML::Key << "E_long" << YAML::Value << E_long;
    out << YAML::Key << "B_lat"  << YAML::Value << B_lat;
    out << YAML::Key << "C_lat"  << YAML::Value << C_lat;
    out << YAML::Key << "D_lat"  << YAML::Value << D_lat;
    out << YAML::Key << "E_lat"  << YAML::Value << E_lat;
    out << YAML::Key << "mu_nominal"            << YAML::Value << mu_nominal;
    out << YAML::Key << "Fz_nominal"            << YAML::Value << Fz_nominal;
    out << YAML::Key << "cornering_stiffness"   << YAML::Value << cornering_stiffness;
    out << YAML::Key << "rolling_resistance"    << YAML::Value << rolling_resistance;
    out << YAML::Key << "combined_slip_enabled" << YAML::Value << combined_slip_enabled;
    out << YAML::Key << "pneumatic_trail"       << YAML::Value << pneumatic_trail;
    out << YAML::Key << "trail_falloff_alpha"   << YAML::Value << trail_falloff_alpha;
    out << YAML::Key << "camber_stiffness"      << YAML::Value << camber_stiffness;
    out << YAML::Key << "load_sensitivity"      << YAML::Value << load_sensitivity;
    out << YAML::Key << "relaxation_length_lat" << YAML::Value << relaxation_length_lat;
    out << YAML::Key << "relaxation_length_long" << YAML::Value << relaxation_length_long;
    out << YAML::Key << "tire_vertical_stiffness" << YAML::Value << tire_vertical_stiffness;
    out << YAML::EndMap;
    std::ofstream ofs(path);
    if (!ofs) throw std::runtime_error("Cannot open YAML file for write: " + path);
    ofs << out.c_str() << "\n";
}

TireParams TireParams::from_tir(const std::string& /*path*/) {
    throw std::runtime_error("TireParams::from_tir is not implemented "
                             "(AVL .tir importer is Phase 2)");
}

// =============================================================================
// SolverParams
// =============================================================================

namespace {

SolverParams::Integrator parse_integrator(const std::string& s) {
    if (s == "Euler") return SolverParams::Integrator::Euler;
    if (s == "RK4")   return SolverParams::Integrator::RK4;
    throw std::runtime_error("YAML field 'integrator': unknown value '" + s +
                             "' (expected Euler/RK4)");
}

const char* integrator_to_string(SolverParams::Integrator i) {
    switch (i) {
        case SolverParams::Integrator::Euler: return "Euler";
        case SolverParams::Integrator::RK4:   return "RK4";
    }
    return "RK4";
}

}  // namespace

SolverParams SolverParams::from_yaml(const std::string& path) {
    const auto root = load_root(path);
    SolverParams p;
    if (const auto n = root["integrator"]; n && !n.IsNull()) {
        p.integrator = parse_integrator(n.as<std::string>());
    }
    pull(root, "max_substep_dt", p.max_substep_dt);
    pull(root, "max_substeps",   p.max_substeps);
    if (p.max_substep_dt <= 0.0) {
        throw std::runtime_error("YAML field 'max_substep_dt' must be > 0");
    }
    if (p.max_substeps <= 0) {
        throw std::runtime_error("YAML field 'max_substeps' must be > 0");
    }
    return p;
}

void SolverParams::to_yaml(const std::string& path) const {
    YAML::Emitter out;
    out.SetIndent(2);
    out << YAML::BeginMap;
    out << YAML::Key << "integrator"     << YAML::Value << integrator_to_string(integrator);
    out << YAML::Key << "max_substep_dt" << YAML::Value << max_substep_dt;
    out << YAML::Key << "max_substeps"   << YAML::Value << max_substeps;
    out << YAML::EndMap;
    std::ofstream ofs(path);
    if (!ofs) throw std::runtime_error("Cannot open YAML file for write: " + path);
    ofs << out.c_str() << "\n";
}

// ---- SensorParams ----
namespace {
void pull_noise(const YAML::Node& root, const char* key, SensorNoise& n) {
    const auto g = root[key];
    if (!g || !g.IsMap()) return;
    pull(g, "noise_std", n.noise_std);
    pull(g, "bias",      n.bias);
    pull(g, "bias_rw",   n.bias_rw);
}
void emit_noise(YAML::Emitter& out, const char* key, const SensorNoise& n) {
    out << YAML::Key << key << YAML::Value << YAML::BeginMap
        << YAML::Key << "noise_std" << YAML::Value << n.noise_std
        << YAML::Key << "bias"      << YAML::Value << n.bias
        << YAML::Key << "bias_rw"   << YAML::Value << n.bias_rw
        << YAML::EndMap;
}
}  // namespace

SensorParams SensorParams::from_yaml(const std::string& path) {
    const auto root = load_root(path);
    SensorParams p;
    pull(root, "enabled", p.enabled);
    pull(root, "seed",    p.seed);
    pull_noise(root, "imu_accel",   p.imu_accel);
    pull_noise(root, "imu_gyro",    p.imu_gyro);
    pull_noise(root, "wheel_speed", p.wheel_speed);
    pull_noise(root, "steer",       p.steer);
    pull_noise(root, "gnss_pos",    p.gnss_pos);
    pull_noise(root, "gnss_vel",    p.gnss_vel);
    return p;
}

void SensorParams::to_yaml(const std::string& path) const {
    YAML::Emitter out;
    out.SetIndent(2);
    out << YAML::BeginMap;
    out << YAML::Key << "enabled" << YAML::Value << enabled;
    out << YAML::Key << "seed"    << YAML::Value << seed;
    emit_noise(out, "imu_accel",   imu_accel);
    emit_noise(out, "imu_gyro",    imu_gyro);
    emit_noise(out, "wheel_speed", wheel_speed);
    emit_noise(out, "steer",       steer);
    emit_noise(out, "gnss_pos",    gnss_pos);
    emit_noise(out, "gnss_vel",    gnss_vel);
    out << YAML::EndMap;
    std::ofstream ofs(path);
    if (!ofs) throw std::runtime_error("Cannot open YAML file for write: " + path);
    ofs << out.c_str() << "\n";
}

}  // namespace vdsim
