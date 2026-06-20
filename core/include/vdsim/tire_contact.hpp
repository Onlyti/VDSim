#pragma once

#include <cmath>

#include "vdsim/params.hpp"

namespace vdsim {

// Minimum speed [m/s] for the slip-ratio / slip-angle denominator. Shared by all
// dynamics models so the slip definition is identical everywhere.
constexpr double kTireSpeedEps = 0.15;

// Geometric contact state at one wheel: the slip kinematics + the load-dependent
// effective rolling radius + the camber-induced contact-point migration. This is
// the single place where "how velocity, spin and load become slip" lives, so a
// change to the slip / Re / camber-offset definition touches one function rather
// than every dynamics model.
struct ContactKinematics {
    double kappa      {0.0};   // [-]   slip ratio   (Re*omega - Vx)/max(|Vx|,eps)
    double alpha      {0.0};   // [rad] slip angle   atan2(Vy, Vx)
    double Re         {0.0};   // [m]   effective rolling radius used
    double vsx        {0.0};   // [m/s] longitudinal slip velocity  Re*omega - Vx
    double vsy        {0.0};   // [m/s] lateral slip velocity        Vy
    double contact_dy {0.0};   // [m]   lateral contact-point offset from camber
    double contact_dz {0.0};   // [m]   vertical contact drop from camber (2nd order)
};

// Vx, Vy: contact-patch velocity in the wheel (tyre) frame [m/s].
// omega:  wheel spin [rad/s].  Fz: vertical load [N].  gamma: camber [rad].
// R0:     unloaded wheel radius [m].
//
// Camber migration: a cambered toroidal tread contacts on the leaning side, so the
// contact point shifts by crown_radius*sin(gamma) laterally (drops by
// crown_radius*(1-cos gamma) vertically). crown_radius=0 -> no migration (legacy).
inline ContactKinematics tire_contact_kinematics(
        double Vx, double Vy, double omega, double Fz,
        double gamma, const TireParams& tp, double R0) {
    ContactKinematics ck;
    ck.Re  = effective_rolling_radius(tp, R0, Fz);
    const double denom = std::max(std::abs(Vx), tp.vlow_speed_eps > 0.0
                                              ? tp.vlow_speed_eps
                                              : kTireSpeedEps);
    ck.vsx = ck.Re * omega - Vx;
    ck.vsy = Vy;
    ck.kappa = ck.vsx / denom;
    ck.alpha = std::atan2(Vy, Vx);
    if (tp.crown_radius > 0.0) {
        ck.contact_dy = tp.crown_radius * std::sin(gamma);
        ck.contact_dz = tp.crown_radius * (1.0 - std::cos(gamma));
    }
    return ck;
}

}  // namespace vdsim
