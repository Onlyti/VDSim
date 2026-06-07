#pragma once

#include <algorithm>
#include <cmath>

#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"

namespace vdsim {

inline double lugre_sigma1(const LuGreTireParams& p) {
    if (p.sigma1 > 0.0) return p.sigma1;
    const double m = std::max(1.0, p.m_eff);
    return 2.0 * std::sqrt(std::max(1.0, p.sigma0) * m);
}

inline double lugre_breakaway(const ITireModel::Output& mf, bool longitudinal) {
    const double g = longitudinal ? std::abs(mf.Fx) : std::abs(mf.Fy);
    return std::max(g, 1.0);
}

inline double lugre_z_dot(double z, double v_r, double g, double sigma0) {
    g = std::max(g, 1.0);
    return v_r - (sigma0 * std::abs(v_r) / g) * z;
}

inline double lugre_force(double z, double v_r, double g, const LuGreTireParams& p) {
    const double sigma0 = std::max(1.0, p.sigma0);
    const double z_dot  = lugre_z_dot(z, v_r, g, sigma0);
    return sigma0 * z + lugre_sigma1(p) * z_dot + p.sigma2 * v_r;
}

inline double lugre_z_max(double g, double sigma0) {
    return std::max(g, 1.0) / std::max(1.0, sigma0);
}

inline double lugre_advance_z(double z, double v_r, double g, double sigma0, double dt) {
    g = std::max(g, 1.0);
    const double abs_v = std::abs(v_r);
    const double z_new = (z + dt * v_r) / (1.0 + dt * sigma0 * abs_v / g);
    const double z_lim = lugre_z_max(g, sigma0);
    return std::clamp(z_new, -z_lim, z_lim);
}

struct LuGreWheelOutput {
    double Fx {0.0};
    double Fy {0.0};
    double Mz {0.0};
    double fx_kin {0.0};
};

inline LuGreWheelOutput lugre_wheel_forces(const ITireModel& tire,
                                           const TireParams& tp,
                                           double z_long,
                                           double z_lat,
                                           double v_slip_long,
                                           double v_slip_lat,
                                           const ITireModel::Input& in) {
    LuGreWheelOutput out{};
    const auto& p = tp.lugre;
    const double sigma0 = std::max(1.0, p.sigma0);

    const auto mf = tire.compute(in);
    const double g_long = lugre_breakaway(mf, true);
    const double g_lat  = lugre_breakaway(mf, false);

    out.Fx = lugre_force(z_long, v_slip_long, g_long, p);
    out.Fy = lugre_force(z_lat,  v_slip_lat,  g_lat,  p);

    if (tp.combined_slip_enabled) {
        const double Fx_max = std::max(std::abs(mf.Fx), 1.0);
        const double Fy_max = std::max(std::abs(mf.Fy), 1.0);
        const double rx = out.Fx / Fx_max;
        const double ry = out.Fy / Fy_max;
        const double ratio_sq = rx * rx + ry * ry;
        if (ratio_sq > 1.0) {
            const double scale = 1.0 / std::sqrt(ratio_sq);
            out.Fx *= scale;
            out.Fy *= scale;
        }
    }

    const double tp0  = tp.pneumatic_trail;
    const double a_fo = (tp.trail_falloff_alpha > 1e-6) ? tp.trail_falloff_alpha : 1e-6;
    const double trail = tp0 / std::sqrt(1.0 + (in.alpha / a_fo) * (in.alpha / a_fo));
    out.Mz = -trail * out.Fy;
    out.fx_kin = out.Fx;
    return out;
}

}  // namespace vdsim
