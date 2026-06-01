// Scenario YAML DSL.
//
// Schema:
//   name:          string
//   initial_vx:    [m/s]
//   duration:      [s]
//   dt:            [s] outer tick
//   mu:            surface mu multiplier (default 1.0)
//   interpolation: zoh | linear   (default zoh)
//   controls:
//     - { t: 0.0,  throttle: 0.0, brake: 0.0, steer: 0.0, gear: 1 }
//     - ...
//
// `controls` must be sorted ascending by `t`.  Each control field is optional
// and defaults to 0 (gear defaults to 1).  Outside the table the boundary
// values (first / last) hold.

#include "vdsim/scenario.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
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
    try { dst = sub.as<T>(); }
    catch (const YAML::Exception& e) {
        throw std::runtime_error(std::string("scenario field '") + key +
                                 "': " + e.what());
    }
}

Scenario::Interp parse_interp(const std::string& s) {
    if (s == "zoh")    return Scenario::Interp::ZOH;
    if (s == "linear") return Scenario::Interp::Linear;
    throw std::runtime_error("scenario 'interpolation': unknown value '" + s +
                             "' (expected zoh / linear)");
}

const char* interp_to_string(Scenario::Interp i) {
    return (i == Scenario::Interp::Linear) ? "linear" : "zoh";
}

YAML::Node load_root(const std::string& path) {
    try { return YAML::LoadFile(path); }
    catch (const YAML::BadFile&) {
        throw std::runtime_error("Cannot open scenario YAML: " + path);
    } catch (const YAML::ParserException& e) {
        throw std::runtime_error("Scenario parse error in '" + path + "': " + e.what());
    }
}

}  // namespace

Scenario Scenario::from_yaml(const std::string& path) {
    const auto root = load_root(path);
    Scenario s;
    pull(root, "name",         s.name);
    pull(root, "initial_vx",   s.initial_vx);
    pull(root, "duration",     s.duration);
    pull(root, "dt",           s.dt);
    pull(root, "mu",           s.mu);

    if (const auto n = root["interpolation"]; n && !n.IsNull()) {
        s.interpolation = parse_interp(n.as<std::string>());
    }
    if (s.duration <= 0.0) throw std::runtime_error("scenario 'duration' must be > 0");
    if (s.dt       <= 0.0) throw std::runtime_error("scenario 'dt' must be > 0");
    if (s.mu       <  0.0) throw std::runtime_error("scenario 'mu' must be >= 0");

    if (const auto seq = root["mu_profile"]; seq && seq.IsSequence()) {
        s.mu_profile.reserve(seq.size());
        for (const auto& row : seq) {
            MuSample ms;
            pull(row, "t",  ms.t);
            pull(row, "mu", ms.mu);
            if (ms.mu < 0.0) ms.mu = 0.0;
            s.mu_profile.push_back(ms);
        }
    }
    if (const auto seq = root["controls"]; seq && seq.IsSequence()) {
        s.controls.reserve(seq.size());
        for (const auto& row : seq) {
            ControlSample cs;
            pull(row, "t",        cs.t);
            pull(row, "throttle", cs.throttle);
            pull(row, "brake",    cs.brake);
            pull(row, "steer",    cs.steer);
            pull(row, "gear",     cs.gear);
            cs.throttle = std::clamp(cs.throttle, 0.0, 1.0);
            cs.brake    = std::clamp(cs.brake,    0.0, 1.0);
            s.controls.push_back(cs);
        }
    }
    if (s.controls.empty()) {
        s.controls.push_back({});      // single zero command
    }
    if (!std::is_sorted(s.controls.begin(), s.controls.end(),
                        [](const ControlSample& a, const ControlSample& b){
                            return a.t < b.t;
                        })) {
        throw std::runtime_error("scenario 'controls' must be sorted by t");
    }
    return s;
}

void Scenario::to_yaml(const std::string& path) const {
    YAML::Emitter out;
    out.SetIndent(2);
    out << YAML::BeginMap;
    out << YAML::Key << "name"          << YAML::Value << name;
    out << YAML::Key << "initial_vx"    << YAML::Value << initial_vx;
    out << YAML::Key << "duration"      << YAML::Value << duration;
    out << YAML::Key << "dt"            << YAML::Value << dt;
    out << YAML::Key << "mu"            << YAML::Value << mu;
    out << YAML::Key << "interpolation" << YAML::Value << interp_to_string(interpolation);
    out << YAML::Key << "controls" << YAML::Value << YAML::BeginSeq;
    for (const auto& c : controls) {
        out << YAML::Flow << YAML::BeginMap
            << YAML::Key << "t"        << YAML::Value << c.t
            << YAML::Key << "throttle" << YAML::Value << c.throttle
            << YAML::Key << "brake"    << YAML::Value << c.brake
            << YAML::Key << "steer"    << YAML::Value << c.steer
            << YAML::Key << "gear"     << YAML::Value << c.gear
            << YAML::EndMap;
    }
    out << YAML::EndSeq << YAML::EndMap;
    std::ofstream ofs(path);
    if (!ofs) throw std::runtime_error("cannot open scenario YAML for write: " + path);
    ofs << out.c_str() << "\n";
}

double Scenario::sample_mu(double t) const {
    if (mu_profile.empty()) return mu;
    if (t <= mu_profile.front().t) return mu_profile.front().mu;
    if (t >= mu_profile.back().t)  return mu_profile.back().mu;
    auto it = std::upper_bound(mu_profile.begin(), mu_profile.end(), t,
                               [](double v, const MuSample& m){ return v < m.t; });
    const auto& hi = *it;
    const auto& lo = *(it - 1);
    // Always linear interp on mu (continuous surface).
    const double span = hi.t - lo.t;
    const double a = (span > 1e-12) ? (t - lo.t) / span : 0.0;
    return lo.mu + a * (hi.mu - lo.mu);
}

ControlSample Scenario::sample(double t) const {
    if (controls.empty()) return {};
    if (t <= controls.front().t) return controls.front();
    if (t >= controls.back().t)  return controls.back();

    // Find first sample with .t > t (upper_bound).
    auto it = std::upper_bound(controls.begin(), controls.end(), t,
                               [](double v, const ControlSample& s){ return v < s.t; });
    const auto& hi = *it;
    const auto& lo = *(it - 1);
    if (interpolation == Interp::ZOH) {
        ControlSample s = lo;
        s.t = t;
        return s;
    }
    // Linear interp on throttle/brake/steer; ZOH for gear.
    const double span = hi.t - lo.t;
    const double a = (span > 1e-12) ? (t - lo.t) / span : 0.0;
    ControlSample s;
    s.t        = t;
    s.throttle = lo.throttle + a * (hi.throttle - lo.throttle);
    s.brake    = lo.brake    + a * (hi.brake    - lo.brake);
    s.steer    = lo.steer    + a * (hi.steer    - lo.steer);
    s.gear     = lo.gear;
    return s;
}

}  // namespace vdsim
