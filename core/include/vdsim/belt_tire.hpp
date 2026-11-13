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

}  // namespace vdsim
