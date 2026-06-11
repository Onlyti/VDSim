#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "vdsim/params.hpp"
#include "vdsim/types.hpp"

namespace vdsim {

inline double wheel_inertia_base(const VehicleParams& vp, int wheel) {
    if (vp.wheel_inertia[wheel] > 0.0) return vp.wheel_inertia[wheel];
    const double R = vp.wheel_radius_nominal;
    const double m_w = vp.unsprung_mass[wheel] > 0.0 ? vp.unsprung_mass[wheel] : 25.0;
    return std::max(0.01, 0.5 * m_w * R * R);
}

inline double reflected_engine_inertia(const VehicleParams& vp) {
    if (vp.engine_rotational_inertia <= 0.0) return 0.0;
    const double g = vp.final_drive_ratio;
    return vp.engine_rotational_inertia * g * g;
}

inline void axle_reflected_shares(const VehicleParams& vp,
                                  double& front_axle,
                                  double& rear_axle) {
    const double I = reflected_engine_inertia(vp);
    front_axle = 0.0;
    rear_axle  = 0.0;
    switch (vp.drive_type) {
        case VehicleParams::Drive::FWD:
            front_axle = I;
            break;
        case VehicleParams::Drive::RWD:
            rear_axle = I;
            break;
        case VehicleParams::Drive::AWD:
            front_axle = 0.5 * I;
            rear_axle  = 0.5 * I;
            break;
    }
}

inline double wheel_engine_inertia_share(const VehicleParams& vp, int wheel) {
    double I_f = 0.0, I_r = 0.0;
    axle_reflected_shares(vp, I_f, I_r);
    return (wheel == WHEEL_FL || wheel == WHEEL_FR) ? 0.5 * I_f : 0.5 * I_r;
}

// NOTE: the older couple_open_axle_spin() post-hoc carrier blend was removed — it was a
// no-op under symmetric wheel speeds (so straight-line accel felt no engine inertia).
// Open-differential wheel-spin accelerations for one axle.
//
// The reflected engine+carrier inertia I_e is geared to the differential carrier,
// whose speed is the mean of the two wheels, omega_c = (omega_L + omega_R)/2. With
// the open diff's equal-torque split this yields the coupled mass matrix
//
//   [ I_L + I_e/4   I_e/4       ] [domega_L]   [T_L]
//   [ I_e/4         I_R + I_e/4 ] [domega_R] = [T_R]
//
// so SYMMETRIC acceleration feels the engine inertia (domega = T/(I + I_e/2)) while
// DIFFERENTIAL motion does not (the spinning wheel is free) — the defining open-diff
// behaviour. T_L/T_R are the net wheel torques (drive + brake - road reaction).
inline void open_axle_spin_accel(double& d_omega_L, double& d_omega_R,
                                 double T_L, double T_R,
                                 double I_wheel_L, double I_wheel_R,
                                 double I_axle_refl) {
    if (I_axle_refl <= 0.0) {
        d_omega_L = T_L / I_wheel_L;
        d_omega_R = T_R / I_wheel_R;
        return;
    }
    const double q   = 0.25 * I_axle_refl;
    const double a   = I_wheel_L + q;
    const double b   = I_wheel_R + q;
    const double det = a * b - q * q;            // = I_L*I_R + q*(I_L+I_R) > 0
    d_omega_L = (b * T_L - q * T_R) / det;
    d_omega_R = (a * T_R - q * T_L) / det;
}

}  // namespace vdsim
