#pragma once

// Vehicle key `tire_low_speed: off | relaxed` (work order 57, PROP-2784 option 2).
//
// relaxed = carcass relaxation length + low-speed damping, one bundle:
//   - the belt first-order slip relaxation (belt_tire.hpp) is switched on for every
//     wheel, tau = sigma / max(|Vx|, v_low),
//   - sigma_y / sigma_x: the tyre's own relaxation_length_lat / _long when > 0,
//     otherwise the defaults below,
//   - v_low keeps the lag state relaxing at standstill instead of freezing it.
//   - near standstill the force law does not read the lag state (section 7.4,
//     PROP-2793 option 2): |Vx| <= v_low reads the key-off slip, v_low..2 v_low blends
//     linearly, >= 2 v_low reads the relaxed state (belt_force_weight). Below 2 v_low =
//     0.3 m/s relaxed therefore equals off; the standstill compliance is the LuGre
//     bristle's. The state update itself is unchanged (it keeps tracking the geometric
//     slip at standstill, so a restart does not read a stale value).
//
// Coefficients fixed before the measurement (gate commit). Changing them after the
// measurement is a gate change (stop and report).
//   kRelaxedSigmaLat  = 0.6 m  : relaxation_length_lat in configs/parts/tire/
//                                default_pacejka.yaml, kinematic_fallback.yaml,
//                                lugre_on.yaml (the shipped passenger-car tyres).
//   kRelaxedSigmaLong = 0.15 m : relaxation_length_long in the same three files.
//   kRelaxedVLow      = 0.15 m/s: the tyre low-speed floor already in the core
//                                (TireParams::vlow_speed_eps default, T3 VLOW,
//                                configs/parts/tire/generic_pacejka.yaml), so the slip
//                                denominator floor and the relaxation floor are the
//                                same speed. Rule check: the longest lag at the floor,
//                                sigma_y / v_low = 4 s, is 4000 ticks of 1 ms (20000
//                                substeps of 0.2 ms), and the shortest judged lag in
//                                K1, sigma_x / 10 m/s = 15 ms, is 15 ticks.

namespace vdsim {

inline constexpr double kRelaxedSigmaLat  = 0.6;   ///< [m] default sigma_y for relaxed
inline constexpr double kRelaxedSigmaLong = 0.15;  ///< [m] default sigma_x for relaxed
inline constexpr double kRelaxedVLow      = 0.15;  ///< [m/s] low-speed relaxation floor

}  // namespace vdsim
