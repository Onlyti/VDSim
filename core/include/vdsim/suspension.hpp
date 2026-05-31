// =============================================================================
// ISuspensionKinematics — wheel-travel + steering input → wheel pose deltas.
//
// Phase 2 (Ld4): geometry-driven suspension kinematics, replacing Ld3's
// phenomenological camber_per_roll / roll_center_height_front scalars.
//
// The interface is stateless and host-managed: implementations may hold an
// internal lookup table (e.g. precomputed by tools/kinematics/dw_3d_solver.py)
// but compute() must be re-entrant and side-effect-free.
//
// All angles in radians, distances in meters.  ISO 8855 RH body frame
// (camber > 0 = top of wheel toward +y / vehicle inboard for LEFT wheel,
//  toe > 0 = wheel spin axis tilts toward vehicle front, i.e. toe-in for left).
// =============================================================================
#pragma once

#include <memory>
#include <string>

namespace vdsim {

class ISuspensionKinematics {
public:
    struct Output {
        double camber       {0.0};   // [rad]
        double toe          {0.0};   // [rad]
        double track_change {0.0};   // [m]
        double caster       {0.0};   // [rad]
    };

    virtual ~ISuspensionKinematics() = default;

    // wheel_travel > 0 = bump (wheel up relative to chassis).
    // steer_input  : implementation-defined (typ. steering rack lateral disp
    //                in meters for DW, or steer angle for trailing arm, etc.)
    virtual Output compute(double wheel_travel,
                           double steer_input) const noexcept = 0;
};

// Load a precomputed (travel × steer) lookup table from CSV.
// Expected columns: wheel_travel, steer_rack_dy, camber, toe, track_change, caster, valid
std::unique_ptr<ISuspensionKinematics>
create_lookup_kinematics(const std::string& csv_path);

// Native (in-process) DW kinematics solver — reads a hardpoint YAML and
// solves the linkage at every compute() call.  No offline precompute.
//   - Newton on LCA θ for true wheel z (vs target = static + travel)
//   - UCA θ from sphere–sphere intersection (|UK − LK| = const)
//   - Trilateration for TK (3 spheres: LK, UK, TR_inner)
//   - Camber / toe / track_change / caster from knuckle frame
// Throws if the YAML is missing fields or has type ≠ "double_wishbone".
std::unique_ptr<ISuspensionKinematics>
create_dw_native_kinematics(const std::string& yaml_path);

// Attach a kinematics model to a FourteenDOFDynamics instance.  Returns false
// if `dyn` is not Ld3.  Ld1/Ld2 ignore this (no per-wheel suspension state).
class IVehicleDynamics;
bool attach_front_kinematics(IVehicleDynamics& dyn,
                              std::unique_ptr<ISuspensionKinematics> k);
bool attach_rear_kinematics (IVehicleDynamics& dyn,
                              std::unique_ptr<ISuspensionKinematics> k);

}  // namespace vdsim
