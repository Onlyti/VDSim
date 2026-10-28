#pragma once

#include <array>
#include <vector>

#include "vdsim/params.hpp"
#include "vdsim/types.hpp"

namespace vdsim {

/**
 * @brief Air-relative state the aero loads depend on.
 *
 * A struct (not a bare vx argument) so later fields (ride height, pitch) can be
 * added without breaking the compute_aero_loads() signature.
 */
struct AeroState {
    double vx {0.0};   ///< body-x speed relative to still air [m/s], negative = reversing
};

/** @brief Force of one element. */
struct AeroElementLoad {
    double fz {0.0};   ///< downforce [N], + presses the car down
    double fx {0.0};   ///< drag [N], + opposes forward motion (body force = -fx)
};

/**
 * @brief Aero loads of a vehicle at one state.
 *
 * Downforce scales with vx^2 (reversing does not turn it into lift); drag
 * follows sign(vx) * vx^2. Axle and wheel splits are rigid-axle reactions that
 * include the drag pitch moment sum(fx_i * z_i) / L. With the four-key path
 * and aero_drag_height unset the split treats drag as acting at the ground
 * (L1-L4 legacy) while the body wrench puts it at the CG (L5 legacy).
 */
struct AeroLoads {
    std::vector<AeroElementLoad> elements;  ///< one per VehicleParams::aero_elements (empty on the four-key path)
    double fz_total {0.0};                  ///< total downforce [N]
    double fx_total {0.0};                  ///< total drag [N]
    double drag_pitch_moment {0.0};         ///< sum fx_i * z_i about the ground [N m]
    double fz_front {0.0};                  ///< front axle reaction increase [N]
    double fz_rear  {0.0};                  ///< rear axle reaction increase [N]
    std::array<double, NUM_WHEELS> fz_wheel_split {};  ///< halves per axle, y ignored (L1/L2)
    std::array<double, NUM_WHEELS> fz_wheel_roll  {};  ///< y distributed by the axle's own track (L3/L4)
    Vec3 force_body  {Vec3::Zero()};        ///< body-frame resultant force [N]
    Vec3 torque_body {Vec3::Zero()};        ///< body-frame moment about the CG [N m]
    double balance_front {0.0};             ///< fz_front / fz_total [-], NaN if fz_total == 0
    double ld {0.0};                        ///< fz_total / fx_total [-], NaN if fx_total == 0
};

/**
 * @brief True if the vehicle uses the element / drag-height aero code path.
 *
 * False means the four legacy keys only, for which L1-L4 keep their original
 * arithmetic bit for bit.
 */
bool aero_uses_element_path(const VehicleParams& vp);

/**
 * @brief Validate the aero inputs of a vehicle.
 * @throws std::invalid_argument on: elements together with aero_lift_* != 0 or
 *         aero_drag_height, empty/invalid/duplicate names, CdA < 0, z < 0,
 *         non-finite values, air_density <= 0.
 */
void validate_aero_params(const VehicleParams& vp);

/**
 * @brief Compute element, axle, wheel and body-wrench aero loads.
 * @param vp vehicle (validated with validate_aero_params())
 * @param s  air-relative state
 * @return loads; see AeroLoads for sign conventions
 */
AeroLoads compute_aero_loads(const VehicleParams& vp, const AeroState& s);

}  // namespace vdsim
