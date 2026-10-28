// Aerodynamic loads shared by every dynamics level (see vdsim/aero.hpp).
#include "vdsim/aero.hpp"

#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

namespace vdsim {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// One force contribution in the common element form.
struct Piece {
    double ClA, CdA, x, y, z_split, z_wrench;
};

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
    const double tf = vp.track_front;
    const double tr = vp.track_rear;

    std::vector<Piece> pieces;
    if (!vp.aero_elements.empty()) {
        pieces.reserve(vp.aero_elements.size());
        for (const auto& e : vp.aero_elements)
            pieces.push_back({e.ClA, e.CdA, e.x, e.y, e.z, e.z});
    } else {
        const double A = vp.frontal_area;
        const double zs = vp.aero_drag_height.value_or(0.0);  // L1-L4 legacy: ground
        const double zw = vp.aero_drag_height.value_or(h);    // L5 legacy: CG
        pieces.push_back({vp.aero_lift_front * A, 0.0, 0.0, 0.0, 0.0, 0.0});
        pieces.push_back({vp.aero_lift_rear  * A, 0.0, -L,  0.0, 0.0, 0.0});
        pieces.push_back({0.0, vp.aero_drag_coeff * A, -a, 0.0, zs, zw});
    }

    AeroLoads out;
    out.fz_wheel_split.fill(0.0);
    out.fz_wheel_roll.fill(0.0);
    double M_drag = 0.0;
    for (const auto& p : pieces) {
        const double fz = p.ClA * q_down;
        const double fx = p.CdA * q_drag;
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
        const Vec3 F(-fx, 0.0, -fz);
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

    if (!vp.aero_elements.empty()) {
        out.elements.reserve(pieces.size());
        for (const auto& p : pieces) out.elements.push_back({p.ClA * q_down, p.CdA * q_drag});
    }
    out.balance_front = (out.fz_total != 0.0) ? out.fz_front / out.fz_total : kNaN;
    out.ld = (out.fx_total != 0.0) ? out.fz_total / out.fx_total : kNaN;
    return out;
}

}  // namespace vdsim
