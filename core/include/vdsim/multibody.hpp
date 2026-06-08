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

#include <Eigen/Core>

#include "vdsim/types.hpp"

namespace vdsim {
class IVehicleDynamics;
}

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
    std::string                  kin_yaml_path;   // native ISuspensionKinematics config
    std::vector<RigidBody>       bodies;          // links + knuckle + (sprung body ref)
    std::vector<Joint>           joints;          // ball / revolute / etc.
    std::vector<Bushing>         bushings;        // optional compliant elements
    std::map<std::string, Hardpoint> hardpoints;  // labeled coordinates

    // Diagnostic outputs filled by forward kinematics + compliance (M3).
    double toe_deg            {0.0};
    double camber_deg         {0.0};
    double caster_deg         {0.0};
    double compliance_toe_deg    {0.0};
    double compliance_camber_deg {0.0};
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
//     * Dynamic: hard-joint corner DAE (revolute + Baumgarte travel constraint);
//       bushing compliance optional / off by default.
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

struct PrescribedCornerMotion {
    double travel_z       {0.0};
    double travel_z_dot   {0.0};
    double travel_z_ddot  {0.0};
    double steer_rack_dy  {0.0};
    double steer_rack_dy_dot {0.0};
};

struct HardJointCornerState {
    double q  {0.0};
    double qd {0.0};
    Vec3   knuckle_aa {Vec3::Zero()};
};

struct CornerDaeParams {
    double m_toe    {0.15};
    double m_camber {0.15};
    double c_toe    {0.0};
    double c_camber {0.0};
    double k_toe    {1.0};
    double k_camber {1.0};
};

struct CornerDynamicsState {
    double q_toe_rad    {0.0};
    double q_toe_dot      {0.0};
    double q_camber_rad   {0.0};
    double q_camber_dot   {0.0};
};

class IHardJointDaeModel {
public:
    virtual ~IHardJointDaeModel() = default;
    virtual void initialize(HardJointCornerState& st,
                            const PrescribedCornerMotion& mot) const = 0;
    virtual WheelPose step(HardJointCornerState& st,
                           const PrescribedCornerMotion& mot,
                           const WheelLoad& load,
                           double dt) const = 0;
};

class IMultibodySolver {
public:
    virtual ~IMultibodySolver() = default;

    // M1 forward kinematics: travel + steer -> wheel pose
    virtual WheelPose forward_kinematics(SuspensionTopology& topo,
                                          double travel_z,
                                          double steer_rad) const = 0;

    // M3 quasi-static compliance: wheel load -> bushing equilibrium
    virtual void quasi_static_compliance(SuspensionTopology& topo,
                                          const WheelLoad& load) const = 0;

    // M4 bushing dynamics: F -> q'', integrated over dt
    virtual void step_dynamics(SuspensionTopology& topo,
                               CornerDynamicsState& state,
                               const WheelLoad& load,
                               double dt) const = 0;
};

// Factory (M1 implementation slot).
std::unique_ptr<IMultibodySolver> create_kinematic_solver();

bool attach_topology_front(IVehicleDynamics& dyn, const SuspensionTopology& topo);
bool attach_topology_rear(IVehicleDynamics& dyn, const SuspensionTopology& topo);

int topology_hardpoint_count(const SuspensionTopology& topo);

void ensure_default_bushings(SuspensionTopology& topo);
void solve_quasi_static_compliance(SuspensionTopology& topo, const WheelLoad& load);
void compliance_targets_rad(const SuspensionTopology& topo, const WheelLoad& load,
                            double& toe_rad, double& camber_rad);
CornerDaeParams corner_dae_params(SuspensionTopology& topo);
void step_corner_dynamics(CornerDynamicsState& state,
                          SuspensionTopology& topo,
                          const WheelLoad& load,
                          double dt);

std::unique_ptr<IHardJointDaeModel> create_hard_joint_dae_model(
    const SuspensionTopology& topo);
void step_hard_joint_dae(HardJointCornerState& state,
                         IHardJointDaeModel& model,
                         SuspensionTopology& topo,
                         const PrescribedCornerMotion& mot,
                         const WheelLoad& load,
                         double dt);

struct KcSweepParams {
    double travel_min_m    {-0.10};
    double travel_max_m    { 0.10};
    int    travel_n        {41};
    double steer_rack_min_m {-0.04};
    double steer_rack_max_m { 0.04};
    int    steer_n         {17};
    double fy_min_n        {-4000.0};
    double fy_max_n        { 4000.0};
    int    fy_n            {9};
    double fz_nominal_n    {4500.0};
};

struct KcSweepSample {
    double abscissa            {0.0};
    double toe_deg             {0.0};
    double camber_deg          {0.0};
    double caster_deg          {0.0};
    double track_mm            {0.0};
    double compliance_toe_deg    {0.0};
    double compliance_camber_deg {0.0};
};

struct KcSweepResult {
    std::vector<KcSweepSample> travel;
    std::vector<KcSweepSample> steer;
    std::vector<KcSweepSample> compliance_fy;
};

KcSweepResult run_kc_sweep(const SuspensionTopology& topo,
                           const KcSweepParams& params = {});

struct KcMetrics {
    double toe_gain_travel_deg_per_mm     {0.0};
    double camber_gain_travel_deg_per_mm  {0.0};
    double track_gain_travel_mm_per_mm    {0.0};
    double toe_gain_steer_deg_per_mm      {0.0};
    double caster_gain_steer_deg_per_mm   {0.0};
};

struct KcMetricDelta {
    std::string name;
    double reference {0.0};
    double candidate {0.0};
    double rel_error {0.0};
    bool   ok        {false};
};

struct KcXcheckReport {
    bool all_ok {false};
    std::vector<KcMetricDelta> deltas;
};

KcMetrics compute_kc_metrics(const KcSweepResult& sweep);
KcXcheckReport compare_kc_metrics(const KcMetrics& reference,
                                  const KcMetrics& candidate,
                                  double rtol = 0.05,
                                  double atol = 1e-4);
KcXcheckReport run_kc_xcheck(const std::string& reference_yaml,
                             const std::string& candidate_yaml,
                             double rtol = 0.05,
                             const KcSweepParams& params = {});

struct RevoluteLink {
    int    parent {-1};
    Vec3   axis_in_parent {Vec3::UnitY()};
    Vec3   r_joint_in_parent {Vec3::Zero()};
    double mass {1.0};
    Vec3   com_in_child {Vec3::Zero()};
    Mat3   inertia_com {Mat3::Identity()};
};

struct LinkTreeModel {
    std::vector<RevoluteLink> links;
    int num_dof() const { return static_cast<int>(links.size()); }
};

struct LinkTreeState {
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
};

struct LinkTreeExternalLoad {
    Vec3 force_world {Vec3::Zero()};
    Vec3 point_world {Vec3::Zero()};
};

LinkTreeModel build_revolute_tree(const SuspensionTopology& topo);
Eigen::VectorXd rnea_revolute_tree(const LinkTreeModel& model,
                                   const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& qd,
                                   const Eigen::VectorXd& qdd,
                                   const std::vector<LinkTreeExternalLoad>& ext,
                                   const Vec3& gravity_world = Vec3(0, 0, -9.81));
Eigen::VectorXd forward_dynamics_revolute_tree(
    const LinkTreeModel& model, const Eigen::VectorXd& q, const Eigen::VectorXd& qd,
    const Eigen::VectorXd& tau, const std::vector<LinkTreeExternalLoad>& ext,
    const Vec3& gravity_world = Vec3(0, 0, -9.81));
void step_revolute_link_tree(const LinkTreeModel& model, LinkTreeState& st,
                             const Eigen::VectorXd& tau,
                             const std::vector<LinkTreeExternalLoad>& ext,
                             double dt,
                             const Vec3& gravity_world = Vec3(0, 0, -9.81));

}  // namespace vdsim::mb
