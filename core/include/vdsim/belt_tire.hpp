#pragma once

// Belt / carcass transient — first-order slip relaxation (Phase T2 primitive).
//
// A steady Magic Formula maps the *instantaneous* slip (kappa, alpha) to force.
// A real tire's carcass/belt deflects before the contact patch reaches the steady
// deflection distribution, so the *effective* slip the tread sees lags the
// *geometric* slip by a time constant tau = sigma / |Vx|, where sigma is the
// relaxation length [m]. Feeding the relaxed slip into the constitutive law
// (MF96 / MF2002 / LuGre) reproduces step-steer / brake-release transient shape
// that an instantaneous map cannot.
//
//   sigma/|Vx| * d(s_eff)/dt = s_geom - s_eff
//
// Integrated exactly over a step dt at frozen Vx (unconditionally stable, and
// correct at standstill where |Vx|->0 freezes the state rather than dividing):
//
//   s_eff <- s_geom + (s_eff - s_geom) * exp(-|Vx| * dt / sigma)
//
// This is the carcass-relaxation belt model (Pacejka 3rd ed. Ch.7/9, first order).
// Higher-order belt eigenmodes (rigid-ring / FTire-class) are out of scope; see
// docs/design/TIRE_ROADMAP.md.

#include <cmath>

namespace vdsim {

/**
 * @brief Relax one slip component (kappa or alpha) toward its geometric value over dt.
 * @param s_eff  Relaxed slip before the step.
 * @param s_geom Geometric slip held over the step.
 * @param Vx     Longitudinal contact speed [m/s].
 * @param sigma  Relaxation length [m]; <= 0 disables the lag (returns s_geom).
 * @param dt     Step [s].
 * @param v_low  Low-speed floor [m/s]: the decay uses max(|Vx|, v_low) so the state keeps
 *               relaxing at standstill (tire_low_speed: relaxed). 0 = no floor (the state
 *               freezes at |Vx| = 0, the original behaviour).
 * @return Relaxed slip after the step.
 */
inline double belt_relax(double s_eff, double s_geom, double Vx,
                         double sigma, double dt, double v_low = 0.0) {
    if (!(sigma > 1e-6) || !(dt > 0.0)) return s_geom;
    const double v = v_low > 0.0 ? std::fmax(std::fabs(Vx), v_low) : std::fabs(Vx);
    const double decay = std::exp(-v * dt / sigma);
    return s_geom + (s_eff - s_geom) * decay;
}

/**
 * @brief Weight of the relaxed slip in the force-law input near standstill
 *        (tire_low_speed: relaxed, work order 57 section 7.4).
 *
 * The LuGre bristle already carries the standstill compliance; stacking the belt
 * first-order lag on top of it near standstill closes a feedback loop (force chatter).
 * Below v_low the force law reads the same slip as the key off, between v_low and
 * 2 v_low it blends linearly, above 2 v_low it reads the relaxed state.
 * @param Vx    Longitudinal contact speed [m/s] (the speed the relaxation uses).
 * @param v_low Low-speed floor [m/s]; <= 0 (belt switched on from the tyre file) = 1.
 * @return w in [0, 1]; 1 = relaxed state only, 0 = key-off input only.
 */
inline double belt_force_weight(double Vx, double v_low) {
    if (!(v_low > 0.0)) return 1.0;
    const double w = (std::fabs(Vx) - v_low) / v_low;
    return w <= 0.0 ? 0.0 : (w >= 1.0 ? 1.0 : w);
}

/**
 * @brief Force-law slip input s_off + w (s_relax - s_off), exact at the ends.
 * @param s_off   Slip the force law reads with the key off.
 * @param s_relax Relaxed slip state.
 * @param w       Weight from belt_force_weight().
 * @return s_relax for w >= 1, s_off for w <= 0, the linear blend otherwise.
 */
inline double belt_force_input(double s_off, double s_relax, double w) {
    if (w >= 1.0) return s_relax;
    if (w <= 0.0) return s_off;
    return s_off + w * (s_relax - s_off);
}

}  // namespace vdsim
