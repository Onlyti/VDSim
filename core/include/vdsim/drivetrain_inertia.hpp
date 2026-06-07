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

inline void couple_open_axle_spin(double& d_omega_L,
                                  double& d_omega_R,
                                  double I_wheel_L,
                                  double I_wheel_R,
                                  double I_axle_refl) {
    if (I_axle_refl <= 0.0) return;
    const double d_carrier = 0.5 * (d_omega_L + d_omega_R);
    const double I_half    = 0.5 * I_axle_refl;
    d_omega_L = (I_wheel_L * d_omega_L + I_half * d_carrier) / (I_wheel_L + I_half);
    d_omega_R = (I_wheel_R * d_omega_R + I_half * d_carrier) / (I_wheel_R + I_half);
}

}  // namespace vdsim
