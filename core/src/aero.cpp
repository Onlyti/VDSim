// Aerodynamic loads shared by every dynamics level (see vdsim/aero.hpp).
#include "vdsim/aero.hpp"

#include <cmath>
#include <limits>
#include <algorithm>
#include <atomic>
#include <set>
#include <stdexcept>
#include <string>

#include <spdlog/spdlog.h>

namespace vdsim {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// One force contribution in the common element form: downforce fz and drag fx
/// [N] (fx opposes forward motion), lateral fy [N] (body +y), point x, y [m] and
/// the drag heights used for the axle split and the body wrench [m].
struct Piece {
    double fz, fx, fy, x, y, z_split, z_wrench;
};

/**
 * @brief Axle / wheel / wrench distribution shared by the built-in elements and
 *        the aero module (one arithmetic, so a re-implementing module matches).
 * @param vp vehicle geometry
 * @param pieces force contributions
 * @param out loads to fill (elements left untouched)
 */
void distribute(const VehicleParams& vp, const std::vector<Piece>& pieces, AeroLoads& out) {
    const double L  = vp.wheelbase;
    const double a  = vp.cg_to_front;
    const double h  = vp.cg_height;
    const double tf = vp.track_front;
    const double tr = vp.track_rear;
    out.fz_wheel_split.fill(0.0);
    out.fz_wheel_roll.fill(0.0);
    double M_drag = 0.0;
    for (const auto& p : pieces) {
        const double fz = p.fz;
        const double fx = p.fx;
        out.fz_total += fz;
        out.fx_total += fx;
        M_drag += fx * p.z_split;
        const double share_f = (L > 0.0) ? (p.x + L) / L : 0.5;
        const double Ff = fz * share_f;
        const double Fr = fz - Ff;
        out.fz_front += Ff;
        out.fz_rear  += Fr;
        out.fz_wheel_split[WHEEL_FL] += 0.5 * Ff;
        out.fz_wheel_split[WHEEL_FR] += 0.5 * Ff;
        out.fz_wheel_split[WHEEL_RL] += 0.5 * Fr;
        out.fz_wheel_split[WHEEL_RR] += 0.5 * Fr;
        const double kf = (tf > 1e-3) ? p.y / tf : 0.0;
        const double kr = (tr > 1e-3) ? p.y / tr : 0.0;
        out.fz_wheel_roll[WHEEL_FL] += Ff * (0.5 + kf);
        out.fz_wheel_roll[WHEEL_FR] += Ff * (0.5 - kf);
        out.fz_wheel_roll[WHEEL_RL] += Fr * (0.5 + kr);
        out.fz_wheel_roll[WHEEL_RR] += Fr * (0.5 - kr);
        // Body wrench about the CG; arm from the CG to the element point.
        const Vec3 r(p.x + a, p.y, p.z_wrench - h);
        const Vec3 F(-fx, p.fy, -fz);
        out.force_body  += F;
        out.torque_body += r.cross(F);
    }
    out.drag_pitch_moment = M_drag;
    const double dM = (L > 0.0) ? M_drag / L : 0.0;   // nose-up: front unloads
    out.fz_front -= dM;
    out.fz_rear  += dM;
    out.fz_wheel_split[WHEEL_FL] -= 0.5 * dM;
    out.fz_wheel_split[WHEEL_FR] -= 0.5 * dM;
    out.fz_wheel_split[WHEEL_RL] += 0.5 * dM;
    out.fz_wheel_split[WHEEL_RR] += 0.5 * dM;
    out.fz_wheel_roll[WHEEL_FL] -= 0.5 * dM;
    out.fz_wheel_roll[WHEEL_FR] -= 0.5 * dM;
    out.fz_wheel_roll[WHEEL_RL] += 0.5 * dM;
    out.fz_wheel_roll[WHEEL_RR] += 0.5 * dM;
    out.balance_front = (out.fz_total != 0.0) ? out.fz_front / out.fz_total : kNaN;
    out.ld = (out.fx_total != 0.0) ? out.fz_total / out.fx_total : kNaN;
}

bool valid_name(const std::string& n) {
    if (n.empty()) return false;
    for (const char c : n) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
        if (!ok) return false;
    }
    return true;
}

}  // namespace

bool aero_uses_element_path(const VehicleParams& vp) {
    return !vp.aero_elements.empty() || vp.aero_drag_height.has_value();
}

bool aero_has_active_slopes(const VehicleParams& vp) {
    for (const auto& e : vp.aero_elements)
        if (e.slopes_active()) return true;
    return false;
}

void validate_aero_params(const VehicleParams& vp) {
    if (!(std::isfinite(vp.air_density) && vp.air_density > 0.0))
        throw std::invalid_argument("air_density must be finite and > 0");
    if (vp.aero_drag_height) {
        const double h = *vp.aero_drag_height;
        if (!std::isfinite(h) || h < 0.0)
            throw std::invalid_argument("aero_drag_height must be finite and >= 0");
    }
    if (vp.aero_elements.empty()) return;
    if (vp.aero_lift_front != 0.0 || vp.aero_lift_rear != 0.0)
        throw std::invalid_argument(
            "aero.elements cannot be combined with aero_lift_front/aero_lift_rear");
    if (vp.aero_drag_height)
        throw std::invalid_argument(
            "aero.elements cannot be combined with aero_drag_height "
            "(each element carries its own z)");
    std::set<std::string> seen;
    for (const auto& e : vp.aero_elements) {
        if (!valid_name(e.name))
            throw std::invalid_argument("aero element name '" + e.name +
                                        "' must match [a-z0-9_]+");
        if (!seen.insert(e.name).second)
            throw std::invalid_argument("duplicate aero element name '" + e.name + "'");
        if (!(std::isfinite(e.ClA) && std::isfinite(e.CdA) && std::isfinite(e.x) &&
              std::isfinite(e.y) && std::isfinite(e.z)))
            throw std::invalid_argument("aero element '" + e.name + "' has a non-finite value");
        if (e.CdA < 0.0)
            throw std::invalid_argument("aero element '" + e.name + "': CdA must be >= 0");
        if (e.z < 0.0)
            throw std::invalid_argument("aero element '" + e.name + "': z must be >= 0");
        const int n_slope = int(e.h_ref.has_value()) + int(e.dClA_dh.has_value()) +
                            int(e.dClA_dpitch.has_value());
        if (n_slope != 0 && n_slope != 3)
            throw std::invalid_argument("aero element '" + e.name +
                                        "': h_ref, dClA_dh and dClA_dpitch go together "
                                        "(all three or none)");
        if (n_slope == 3 && !(std::isfinite(*e.h_ref) && std::isfinite(*e.dClA_dh) &&
                              std::isfinite(*e.dClA_dpitch)))
            throw std::invalid_argument("aero element '" + e.name +
                                        "' has a non-finite h_ref / dClA_dh / dClA_dpitch");
    }
}

AeroLoads compute_aero_loads(const VehicleParams& vp, const AeroState& s) {
    const double rho = vp.air_density;
    const double vx  = s.vx;
    const double q_down = 0.5 * rho * vx * vx;          // downforce: vx^2
    const double q_drag = 0.5 * rho * vx * std::abs(vx);  // drag: sign(vx) vx^2
    const double L  = vp.wheelbase;
    const double a  = vp.cg_to_front;
    const double h  = vp.cg_height;
    const double x_o = -a;   // body-origin x in the element frame (CG and sprung origin)

    AeroLoads out;
    std::vector<Piece> pieces;
    if (!vp.aero_elements.empty()) {
        pieces.reserve(vp.aero_elements.size());
        out.elements.reserve(vp.aero_elements.size());
        for (const auto& e : vp.aero_elements) {
            // h - h_ref = dz - (x - x_o)*theta. The slope term is skipped (not
            // multiplied by zero) when both slopes are 0, so ClA stays bit exact.
            const double dh = s.dz - (e.x - x_o) * s.pitch;
            double ClA = e.ClA;
            if (e.slopes_active())
                ClA = e.ClA + *e.dClA_dh * dh + *e.dClA_dpitch * s.pitch;
            const double fz = ClA * q_down;
            const double fx = e.CdA * q_drag;
            pieces.push_back({fz, fx, 0.0, e.x, e.y, e.z, e.z});
            out.elements.push_back({fz, fx, e.h_ref.value_or(e.z) + dh});
        }
    } else {
        const double A = vp.frontal_area;
        const double zs = vp.aero_drag_height.value_or(0.0);  // L1-L4 legacy: ground
        const double zw = vp.aero_drag_height.value_or(h);    // L5 legacy: CG
        pieces.push_back({vp.aero_lift_front * A * q_down, 0.0 * q_drag, 0.0,
                          0.0, 0.0, 0.0, 0.0});
        pieces.push_back({vp.aero_lift_rear * A * q_down, 0.0 * q_drag, 0.0,
                          -L, 0.0, 0.0, 0.0});
        pieces.push_back({0.0 * q_down, vp.aero_drag_coeff * A * q_drag, 0.0,
                          -a, 0.0, zs, zw});
    }
    distribute(vp, pieces, out);
    return out;
}

void warn_negative_element_height(const VehicleParams& vp, const AeroLoads& loads,
                                  bool& warned) {
    if (warned) return;
    const std::size_t n = std::min(vp.aero_elements.size(), loads.elements.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (!vp.aero_elements[i].has_ride_slopes()) continue;
        if (loads.elements[i].h < 0.0) {
            warned = true;
            spdlog::warn("[vdsim aero] element '{}' is below the ground (h = {} m); its "
                         "ride-height slope is extrapolated", vp.aero_elements[i].name,
                         loads.elements[i].h);
            return;
        }
    }
}

AeroInput make_aero_input(const VehicleParams& vp, const Vec3& v_body, const AeroState& s) {
    AeroInput in;
    in.v_air_body = v_body;
    in.slip_angle = std::atan2(v_body.y(), v_body.x());
    in.dz_heave = s.dz;
    in.pitch = s.pitch;
    in.roll = s.roll;
    in.dh_front = s.dz - vp.cg_to_front * s.pitch;
    in.dh_rear = s.dz + vp.cg_to_rear * s.pitch;
    in.rho = vp.air_density;
    return in;
}

AeroLoads aero_loads_from_forces(const VehicleParams& vp,
                                 const std::vector<AeroForce>& forces, bool lateral) {
    std::vector<Piece> pieces;
    pieces.reserve(forces.size());
    bool fy_dropped = false;
    for (const auto& f : forces) {
        double fy = f.force_body.y();
        if (!lateral && fy != 0.0) {
            fy = 0.0;
            fy_dropped = true;
        }
        // Body force (-fx, fy, -fz): drag fx and downforce fz in the element form.
        pieces.push_back({-f.force_body.z(), -f.force_body.x(), fy, f.point.x(), f.point.y(),
                          f.point.z(), f.point.z()});
    }
    if (fy_dropped) {
        static std::atomic<bool> warned{false};   // once per process
        if (!warned.exchange(true))
            spdlog::warn("[vdsim aero] aero module returned a lateral force Fy; it is ignored "
                         "below L5 (L2-L4 have no lateral aero path)");
    }
    AeroLoads out;
    distribute(vp, pieces, out);
    return out;
}

void warn_aero_module_replaces(const VehicleParams& vp) {
    std::string what;
    const auto add = [&what](const std::string& s) {
        if (!what.empty()) what += ", ";
        what += s;
    };
    if (vp.aero_elements.empty()) {
        if (vp.aero_drag_coeff != 0.0 && vp.frontal_area != 0.0)
            add("Cd=" + std::to_string(vp.aero_drag_coeff) +
                " frontal_area=" + std::to_string(vp.frontal_area));
        if (vp.aero_lift_front != 0.0) add("aero_lift_front=" + std::to_string(vp.aero_lift_front));
        if (vp.aero_lift_rear != 0.0) add("aero_lift_rear=" + std::to_string(vp.aero_lift_rear));
        if (vp.aero_drag_height) add("aero_drag_height");
    } else {
        std::string names;
        for (const auto& e : vp.aero_elements) names += (names.empty() ? "" : " ") + e.name;
        add("aero.elements [" + names + "]");
    }
    if (what.empty()) return;
    static std::atomic<bool> warned{false};   // once per process
    if (!warned.exchange(true))
        spdlog::warn("[vdsim aero] aero module replaces the built-in aero of the vehicle ({}); "
                     "air_density is passed to the module as rho", what);
}

}  // namespace vdsim
