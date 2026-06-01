#pragma once

// L4-L5 multibody suspension dynamics — header-only skeleton (M0 stub).
//
// Out of scope for the PoC W1-W12; intended as the entry surface for the
// post-graduation phase ("Adams-class hardpoint-driven multibody on top of
// the same C++17 / pybind11 stack").
//
// Design intent:
//   * Lightweight rigid body + joint + bushing description.
//   * Hardpoint-driven YAML schema for standard suspension topologies.
//   * Forward kinematics (suspension travel → wheel pose) first, then
//     compliance, then full DAE.
//   * Compatible with the existing IVehicleDynamics interface — an
//     L4_MultibodyKinematic / L5_MultibodyCompliant impl would plug in
//     beside L3 14-DOF.
//
// References (intended):
//   * Featherstone, "Rigid Body Dynamics Algorithms" (2008).
//   * Genta, "Motor Vehicle Dynamics: Modeling and Simulation" (2014).
//   * Reimpell / Stoll / Betzler, "The Automotive Chassis" (2001).

#include <array>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "vdsim/types.hpp"

namespace vdsim::mb {

// -----------------------------------------------------------------------------
// Rigid body — Newton-Euler in body frame.
// -----------------------------------------------------------------------------
struct RigidBody {
    std::string id;
    double      mass            {0.0};      // [kg]
    Mat3        inertia_body    {Mat3::Identity()};   // [kg m^2]  about its CG
    Vec3        cg_local        {Vec3::Zero()};       // CG offset in body frame [m]

    // Inertial state (filled by solver).
    Vec3        position_world  {Vec3::Zero()};       // [m]
    Quat        orientation     {Quat::Identity()};   // body -> world
    Vec3        velocity_world  {Vec3::Zero()};       // [m/s] of body-frame origin
    Vec3        omega_body      {Vec3::Zero()};       // [rad/s] body-frame angular vel
};

// -----------------------------------------------------------------------------
// Joints — kinematic constraints between two bodies.
// -----------------------------------------------------------------------------
enum class JointType {
    Ball,        // 3-translation constrained; 3 rotation free
    Revolute,    // 1 rotation free about axis
    Cylindrical, // 1 rot + 1 trans about/along axis
    Prismatic,   // 1 trans along axis
    Universal,   // 2 rot free (cross axes), 1 rot constrained
    Spherical,   // synonym of Ball
    Rigid,       // 6-DOF locked (used for suspension top-mount when modeled rigid)
};

struct Joint {
    std::string id;
    JointType   type;
    std::string body_a_id;
    std::string body_b_id;
    Vec3        position_in_a {Vec3::Zero()};         // joint location in body A frame
    Vec3        position_in_b {Vec3::Zero()};         // joint location in body B frame
    Vec3        axis_in_a     {Vec3::UnitX()};        // for revolute / cylindrical / prismatic
};

// -----------------------------------------------------------------------------
// Bushing — 6-DOF compliant element (translation + rotation stiffness/damping).
//
//   F_local = - K_diag · q_local  - C_diag · q_dot_local
//
// PoC: linear diagonal stiffness; nonlinear lookups can be added per axis.
// -----------------------------------------------------------------------------
struct Bushing {
    std::string id;
    std::string body_a_id;
    std::string body_b_id;
    Vec3        position_in_a {Vec3::Zero()};
    Vec3        position_in_b {Vec3::Zero()};
    Vec3        axis_in_a     {Vec3::UnitX()};        // local x of bushing frame
    // Diagonal stiffness / damping in bushing-local axes.
    Vec3        k_translation {Vec3::Zero()};         // [N/m]   (x, y, z)
    Vec3        k_rotation    {Vec3::Zero()};         // [N m/rad]  (about x, y, z)
    Vec3        c_translation {Vec3::Zero()};         // [N s/m]
    Vec3        c_rotation    {Vec3::Zero()};         // [N m s/rad]
};

// -----------------------------------------------------------------------------
// Hardpoint — labeled coordinate in body frame (suspension geometry inputs).
// -----------------------------------------------------------------------------
struct Hardpoint {
    std::string name;       // e.g. "lca_inner_front"
    Vec3        position;   // [m] in body frame (parent body's local frame)
    std::string body_id;    // owning body
};

// -----------------------------------------------------------------------------
// Suspension topology — one corner.
// -----------------------------------------------------------------------------
enum class TopologyKind {
    MacPherson,
    DoubleWishbone,
    MultiLink5,
    TrailingArm,
    BeamAxle,
    DeDion,
    TwistBeam,
};

struct SuspensionTopology {
    TopologyKind                 kind;
    std::string                  name;            // human-readable
    std::vector<RigidBody>       bodies;          // links + knuckle + (sprung body ref)
    std::vector<Joint>           joints;          // ball / revolute / etc.
    std::vector<Bushing>         bushings;        // optional compliant elements
    std::map<std::string, Hardpoint> hardpoints;  // labeled coordinates

    // Diagnostic outputs filled by forward kinematics evaluation.
    double toe_deg            {0.0};
    double camber_deg         {0.0};
    double caster_deg         {0.0};
    double kingpin_incl_deg   {0.0};
    double scrub_radius_mm    {0.0};
    double mech_trail_mm      {0.0};

    static SuspensionTopology from_yaml(const std::string& path);
    void                       to_yaml(const std::string& path) const;
};

// -----------------------------------------------------------------------------
// Solvers (forward declarations only — empty impl in M0 stub).
//
//   IMultibodySolver
//     * Forward kinematics: given suspension travel z and steering input,
//       compute wheel pose (toe, camber, caster) and bushing deflections.
//     * Compliance / static balance: given external loads at wheel center
//       (Fx, Fy, Fz, Mz), compute equilibrium.
//     * Dynamic: full DAE (Featherstone or augmented Lagrangian); reserved
//       for M4.
// -----------------------------------------------------------------------------
struct WheelLoad {
    Vec3 force_world  {Vec3::Zero()};       // [N]  Fx, Fy, Fz at wheel center, world frame
    Vec3 moment_world {Vec3::Zero()};       // [N m] aligning + roll moments
};

struct WheelPose {
    Vec3   position_world {Vec3::Zero()};   // wheel center [m]
    Quat   orientation    {Quat::Identity()};
    double toe_rad        {0.0};
    double camber_rad     {0.0};
    double caster_rad     {0.0};
};

class IMultibodySolver {
public:
    virtual ~IMultibodySolver() = default;

    // M1 forward kinematics: travel + steer -> wheel pose
    virtual WheelPose forward_kinematics(const SuspensionTopology& topo,
                                          double travel_z,
                                          double steer_rad) const = 0;

    // M3 quasi-static compliance: wheel load -> bushing equilibrium
    virtual void quasi_static_compliance(SuspensionTopology& topo,
                                          const WheelLoad& load) const = 0;

    // M4 full dynamics (placeholder)
    virtual void step_dynamics(SuspensionTopology& topo, double dt) const = 0;
};

// Factory (M1 implementation slot).
std::unique_ptr<IMultibodySolver> create_kinematic_solver();

}  // namespace vdsim::mb
