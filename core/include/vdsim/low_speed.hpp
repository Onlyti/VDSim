#pragma once

// Shared low-speed kinematic-dynamic blend + brake-hold creep constants.
//
// The L1 bicycle and L2 seven_dof (and L3 via its inner L2) blend from the
// validated dynamic model down to a slip-free kinematic bicycle near standstill
// and hold the car on a grade with a viscous brake-hold damper. These constants
// set the blend onset / relaxation rate / hold stiffness and MUST stay identical
// across the ladder, so they live here rather than being copied per source.

namespace vdsim {

inline constexpr double kStickBlend = 3.0;    // [m/s] dynamic <-> low-speed (kinematic) blend
inline constexpr double kStickC     = 6.0e4;  // [N s/m] brake-hold creep damping (per wheel)
inline constexpr double kKinTau     = 0.05;   // [s] low-speed kinematic relaxation time

}  // namespace vdsim
