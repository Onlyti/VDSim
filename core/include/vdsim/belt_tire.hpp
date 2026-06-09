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

// Relax one slip component (kappa or alpha) toward its geometric value over dt.
// sigma <= 0 disables the lag (returns the geometric slip = instant response).
inline double belt_relax(double s_eff, double s_geom, double Vx,
                         double sigma, double dt) {
    if (!(sigma > 1e-6) || !(dt > 0.0)) return s_geom;
    const double decay = std::exp(-std::fabs(Vx) * dt / sigma);
    return s_geom + (s_eff - s_geom) * decay;
}

}  // namespace vdsim
