// Internal (not installed): spawn settle with an explicit body-origin height.
//
// The public settle_spawn_on_ground() places the body origin as the whole-vehicle
// CG at cg_height (L1-L4, L5 penalty path). The L5 strut / DAE body is the sprung
// mass alone, whose CG sits at the derived h_s (work order 47); SimSession passes
// that height here so the wheels touch down at the strut path's own mounts.
#pragma once

#include "vdsim/interfaces.hpp"

namespace vdsim::detail {

/**
 * @brief settle_spawn_on_ground() with the body origin at body_height.
 * @param ground      Contact provider queried for wheel penetration.
 * @param vp          Vehicle parameters (a, b, tracks, wheel radius).
 * @param s           Spawn pose; z is set to body_height when below 1e-6, then
 *                    dropped / raised until the wheels just touch; vz is zeroed.
 * @param body_height Body origin height above ground at design attitude [m].
 *                    Equal to vp.cg_height reproduces settle_spawn_on_ground() exactly.
 */
void settle_spawn_on_ground_at(IContactProvider& ground, const VehicleParams& vp,
                               State& s, double body_height);

}  // namespace vdsim::detail
