// Inverted tire interface (Phase 2), shared by every ITireModel backend.
//
// evaluate() / advance_bristle() / advance_relaxation() turn the "kinematics-in ->
// wrench-out" contract into one implementation: slip / effective rolling radius /
// camber contact migration come from the shared tire_contact module, the transient
// (belt relaxation / relaxation-length lag / LuGre bristle) is supplied frozen by the
// caller, and the constitutive force law is dispatched through the virtual compute().
// A new tire is therefore just a compute() override; the slip/transient plumbing is
// not reimplemented per backend. See docs/design/TIRE_INTERFACE_INVERSION.md.

#include "vdsim/interfaces.hpp"

#include "vdsim/belt_tire.hpp"
#include "vdsim/lugre_tire.hpp"
#include "vdsim/tire_contact.hpp"

#include <algorithm>
#include <cmath>

namespace vdsim {

ITireModel::Wrench ITireModel::evaluate(const ContactInput& ci, const Transient& tr) const {
    Wrench w;
    const auto ck = tire_contact_kinematics(
        ci.Vx, ci.Vy, ci.omega, ci.Fz, ci.gamma, params_, ci.R0);
    w.Re = ck.Re;
    w.kappa = ck.kappa;            // geometric slip (reported / diagnostics)
    w.alpha = ck.alpha;
    w.contact_dy = ck.contact_dy;
    w.Mx = ci.Fz * ck.contact_dy;  // camber contact-migration overturning moment

    const bool lugre_on   = params_.lugre.enabled;
    const bool belt_on    = params_.belt.enabled && !lugre_on;
    const bool belt_lugre = params_.belt.enabled && lugre_on;

    Input in;
    in.Fz    = ci.Fz;
    in.kappa = belt_on ? tr.belt_kappa : ck.kappa;
    in.alpha = belt_on ? tr.belt_alpha
        : ((lugre_on || params_.relaxation_length_lat <= 1e-6) ? ck.alpha : tr.alpha_dyn);
    in.mu_long = ci.mu_long;
    in.mu_lat  = ci.mu_lat;
    in.Vx_wheel = ci.Vx;
    in.gamma    = ci.gamma;

    if (lugre_on) {
        const double v_slip_long = belt_lugre ? tr.belt_vlong : ck.vsx;
        const double v_slip_lat  = belt_lugre ? tr.belt_vlat  : ck.vsy;
        const auto l = lugre_wheel_forces(*this, params_, tr.lugre_z_long, tr.lugre_z_lat,
                                          v_slip_long, v_slip_lat, in);
        w.Fx = l.Fx;
        w.Fy = l.Fy;
        w.Mz = l.Mz;
        w.mu_peak = compute(in).mu_peak;
    } else {
        const auto out = compute(in);
        w.Fx = out.Fx;
        w.Fy = out.Fy;
        w.Mz = out.Mz;
        w.Mx += out.Mx;   // carcass Mx from the force law (0 for the simple laws), plus camber Mx
        w.mu_peak = out.mu_peak;
    }
    return w;
}

ITireModel::Transient ITireModel::advance_bristle(const ContactInput& ci,
                                                  const Transient& tr, double dt) const {
    if (!params_.lugre.enabled) return tr;
    Transient out = tr;
    const auto ck = tire_contact_kinematics(
        ci.Vx, ci.Vy, ci.omega, ci.Fz, ci.gamma, params_, ci.R0);
    const double sigma0 = std::max(1.0, params_.lugre.sigma0);
    Input in;
    in.Fz = ci.Fz; in.kappa = ck.kappa; in.alpha = ck.alpha;
    in.mu_long = ci.mu_long; in.mu_lat = ci.mu_lat; in.Vx_wheel = ci.Vx;
    const auto mf = compute(in);
    out.lugre_z_long = lugre_advance_z(
        tr.lugre_z_long, ck.vsx,
        lugre_breakaway(mf, true, in.Fz, in.mu_long, in.mu_lat), sigma0, dt);
    out.lugre_z_lat = lugre_advance_z(
        tr.lugre_z_lat, ck.vsy,
        lugre_breakaway(mf, false, in.Fz, in.mu_long, in.mu_lat, in.alpha), sigma0, dt);
    return out;
}

ITireModel::Transient ITireModel::advance_relaxation(const ContactInput& ci,
                                                     const Transient& tr, double dt) const {
    Transient out = tr;
    const auto ck = tire_contact_kinematics(
        ci.Vx, ci.Vy, ci.omega, ci.Fz, ci.gamma, params_, ci.R0);
    if (params_.lugre.enabled) {
        // Belt on the LuGre path relaxes the slip *velocities* feeding the bristle.
        if (params_.belt.enabled) {
            out.belt_vlong = belt_relax(tr.belt_vlong, ck.vsx, ci.Vx,
                                        params_.belt.sigma_long, dt);
            out.belt_vlat  = belt_relax(tr.belt_vlat, ck.vsy, ci.Vx,
                                        params_.belt.sigma_lat, dt);
        }
    } else {
        // Relaxation-length transient slip angle (exponential lag toward geometric).
        if (params_.relaxation_length_lat > 1e-6) {
            const double v_safe = std::max(std::abs(ci.Vx), kTireSpeedEps);
            const double decay  = std::exp(-v_safe * dt / params_.relaxation_length_lat);
            out.alpha_dyn = ck.alpha + (tr.alpha_dyn - ck.alpha) * decay;
        }
        // Belt on the MF path relaxes kappa / alpha directly.
        if (params_.belt.enabled) {
            out.belt_kappa = belt_relax(tr.belt_kappa, ck.kappa, ci.Vx,
                                        params_.belt.sigma_long, dt);
            out.belt_alpha = belt_relax(tr.belt_alpha, ck.alpha, ci.Vx,
                                        params_.belt.sigma_lat, dt);
        }
    }
    return out;
}

}  // namespace vdsim
