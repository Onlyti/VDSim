#pragma once

#include <array>
#include <memory>
#include <tuple>
#include <vector>

#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"
#include "vdsim/subsystems.hpp"
#include "vdsim/types.hpp"

namespace vdsim {

namespace mb {
struct SuspensionTopology;
}

// =============================================================================
// IVehicleDynamics — top-level vehicle dynamics interface
// =============================================================================
class IVehicleDynamics {
public:
    // Ladder IDs (YAML/GUI): K, L1–L5. Theory names (Ld0–Ld5):
    //   K/Ld0 Kinematic bicycle | L1/Ld1 Single-track bicycle (5-DOF)
    //   L2/Ld2 Planar seven-DOF per-wheel | L3/Ld3 Ride fourteen-DOF
    //   L4/Ld4 Hardpoint kinematic multibody | L5/Ld5 Free 3D stunt
    enum class Level { L1_Bicycle, L2_SevenDOF, L3_FourteenDOF, L4_Kinematic,
                       L5_Stunt, Lk_Kinematic };

    virtual ~IVehicleDynamics() = default;

    virtual Level level() const noexcept = 0;

    // Setup (throw on invalid input)
    virtual void initialize(const VehicleParams&,
                            const TireParams&,
                            const SolverParams&) = 0;

    // Set internal state (no error: caller's responsibility)
    virtual void reset(const State& initial) noexcept = 0;

    // Advance one tick. NaN / out-of-range inputs are logged and clamped.
    virtual void step(const ControlInput& u,
                      const ContactArray& contacts,
                      double dt) noexcept = 0;

    virtual const State& state() const noexcept = 0;

    // Per-wheel diagnostics (for validation/debug)
    virtual std::array<Vec3,   NUM_WHEELS> tire_forces_body()  const = 0;  // [N], body frame
    // Per-wheel tire force in the WHEEL (tire) frame: x = wheel heading
    // (longitudinal), y = wheel lateral. The body-frame force is this rotated by
    // the wheel's steer angle. Default returns the body-frame force (levels with no
    // per-wheel steer separation); L2/L3 override with the un-rotated wheel force.
    virtual std::array<Vec3,   NUM_WHEELS> tire_forces_wheel() const { return tire_forces_body(); }
    virtual std::array<double, NUM_WHEELS> tire_Fz()           const = 0;  // [N]
    virtual std::array<double, NUM_WHEELS> wheel_slip_ratio()  const = 0;  // [-]
    virtual std::array<double, NUM_WHEELS> wheel_slip_angle()  const = 0;  // [rad]
    // Friction coefficient each wheel used this step (from the contact provider).
    virtual std::array<double, NUM_WHEELS> wheel_mu() const {
        return {{0.0, 0.0, 0.0, 0.0}};
    }
    // Realized load-dependent peak friction coefficient (force/Fz) per wheel.
    virtual std::array<double, NUM_WHEELS> wheel_mu_peak() const { return wheel_mu(); }
    // Per-wheel overturning moment [N m] about the wheel-forward axis: tire carcass
    // Mx + camber contact-point migration (Fz * crown_radius * sin gamma). Feeds the
    // roll DOF on models that have one (L3/L5). Default 0 (no camber migration).
    virtual std::array<double, NUM_WHEELS> wheel_overturning_moment() const {
        return {{0.0, 0.0, 0.0, 0.0}};
    }

    // Quasi-static roll / pitch estimates (zero for L1; non-zero for L2 onward).
    virtual double roll_angle_qs()  const { return 0.0; }   // [rad]
    virtual double pitch_angle_qs() const { return 0.0; }   // [rad]
    virtual double ax_body_est()    const { return 0.0; }   // [m/s^2]
    virtual double ay_body_est()    const { return 0.0; }

    // Steering-rack feedback torque (sum of front-wheel Mz times steering ratio).
    // Useful for driver model torque feedback. Returns 0 for L1 (axle-averaged Mz).
    virtual double steering_rack_torque() const { return 0.0; }   // [N m]

    // External per-wheel camber input [rad] for the next step.  Used by L3
    // (roll-induced camber) and by callers that have their own suspension
    // kinematics.  Default no-op; overriding levels may use it.
    virtual void set_camber_per_wheel(
        const std::array<double, NUM_WHEELS>& /*gamma*/) noexcept {}
    // External per-wheel toe input [rad].  Additive to the Ackerman-corrected
    // steer angle on front; affects all wheels (bump-steer / kinematic toe).
    virtual void set_toe_per_wheel(
        const std::array<double, NUM_WHEELS>& /*toe*/) noexcept {}
    // External per-wheel vertical load [N] for the NEXT step's tire grip, in
    // place of the model's own quasi-static Fz.  Used by L3 to feed its dynamic
    // (ride/road-coupled) tire load into the grip calc.  One-shot: consumed by
    // the next step().  Default no-op.
    virtual void set_external_fz(
        const std::array<double, NUM_WHEELS>& /*fz*/) noexcept {}

    // L4 multibody bushing compliance state [rad]; axle 0=front, 1=rear.
    virtual double compliance_toe_rad(int /*axle*/) const noexcept { return 0.0; }

    // Engine + gearbox (Drivetrain v2). 0 / 0 unless a powertrain is enabled.
    virtual double engine_rpm()   const { return 0.0; }   // [rpm]
    virtual int    current_gear() const { return 0; }     // 1..N, 0=N, <0=R
    // Install a programmatic shift policy (e.g. a Python callable). Returns false
    // if this model has no gearbox.
    virtual bool set_shift_policy(ShiftPolicy /*fn*/) { return false; }

    // User-defined subsystem modules. Replace a built-in module with a custom one
    // (a C++ subclass or a Python subclass via pybind). Each returns false if the
    // model does not host that module (e.g. suspension/ARB live only on L3).
    // The module's begin_step() runs once per step and apply()/force() per RK4 stage,
    // so step-coherent state belongs in begin_step (brake/steering/drivetrain);
    // suspension/ARB force laws are evaluated per stage and must be memoryless.
    virtual bool set_brake_module(std::shared_ptr<IBrakeSystem> /*m*/)        { return false; }
    virtual bool set_steering_module(std::shared_ptr<ISteeringSystem> /*m*/)  { return false; }
    virtual bool set_drivetrain_module(std::shared_ptr<IDrivetrain> /*m*/)    { return false; }
    virtual bool set_suspension_module(std::shared_ptr<ISuspension> /*m*/)    { return false; }
    virtual bool set_antirollbar_module(int /*axle*/,
                                        std::shared_ptr<IAntiRollBar> /*m*/)  { return false; }
};

std::unique_ptr<IVehicleDynamics> create_bicycle();
std::unique_ptr<IVehicleDynamics> create_seven_dof();
std::unique_ptr<IVehicleDynamics> create_fourteen_dof();
std::unique_ptr<IVehicleDynamics> create_fourteen_dof_kinematic();
bool fourteen_dof_attach_multibody(IVehicleDynamics& dyn,
                                   bool front_axle,
                                   const mb::SuspensionTopology& topo,
                                   bool enable_dynamics);
bool fourteen_dof_mb_dynamics_enabled(const IVehicleDynamics& dyn, int axle);
std::unique_ptr<IVehicleDynamics> create_stunt_dof();
class IContactProvider;   // defined below; needed by free_3d_attach_contact_provider

// L5 (free_3d) spatial-strut corner DAE: per corner, feed the strut travel to the
// L4 hard-joint corner DAE and apply the resulting toe/camber to the tire. Requires
// SolverParams::l5_spatial_suspension (the DAE consumes the strut travel). front_axle
// selects the FL/FR (true) or RL/RR (false) topology.
bool free_3d_attach_multibody(IVehicleDynamics& dyn,
                              bool front_axle,
                              const mb::SuspensionTopology& topo,
                              bool enable_dynamics);
bool free_3d_mb_dynamics_enabled(const IVehicleDynamics& dyn, int axle);
// Per-wheel suspension toe/camber [rad] last applied to the tire (strut path + DAE).
// Zero for any other model. For inspection / suspension-vs-L4 validation.
std::array<double, NUM_WHEELS> free_3d_wheel_camber(const IVehicleDynamics& dyn);
std::array<double, NUM_WHEELS> free_3d_wheel_toe(const IVehicleDynamics& dyn);
// Attach a contact provider for per-substep contact re-query (free-3D strut path). The
// dynamics then refreshes the contact at each internal substep pose, shrinking the
// once-per-step frozen-contact discretization error on curved surfaces. Non-owning: the
// provider must outlive the dynamics. Pass nullptr to revert to the frozen ContactArray.
bool free_3d_attach_contact_provider(IVehicleDynamics& dyn, IContactProvider* provider);
// Kinematic bicycle (no tire forces / no slip): yaw_rate = v*tan(delta)/L.
// For path-planning / kinematic-MPC use and as the simplest ladder rung.
std::unique_ptr<IVehicleDynamics> create_kinematic();

// =============================================================================
// ITireModel — tire force model (Pacejka MF96, linear, ...)
// =============================================================================
class ITireModel {
public:
    struct Input {
        double Fz       {0.0};    // [N]
        double kappa    {0.0};    // slip ratio
        double alpha    {0.0};    // slip angle [rad]
        double mu_long  {1.0};    // surface mu scaling [-]
        double mu_lat   {1.0};    // surface mu scaling [-]
        double Vx_wheel {0.0};    // [m/s]
        double gamma    {0.0};    // camber angle [rad]
    };
    struct Output {
        double Fx {0.0};          // [N] body frame, wheel-axis longitudinal
        double Fy {0.0};          // [N] lateral
        double Mz {0.0};          // [N m] aligning moment (about contact normal)
        double Mx {0.0};          // [N m] overturning moment (about wheel-forward axis)
        double mu_peak {0.0};     // realized resultant peak coefficient (force/Fz)
    };

    // ----- Inverted ("kinematics-in -> wrench-out") interface (Phase 2) -----
    // The dynamics supplies the raw contact kinematics + load; the tire owns the slip
    // definition, effective rolling radius, camber migration and (eventually) the
    // transient state (relaxation / belt / LuGre). The transient is OWNED and stored
    // per-wheel by the caller so the RK4 integrator can checkpoint/restore it: evaluate()
    // is the frozen per-stage force evaluation, advance() integrates the transient once
    // per substep. Migration in progress — see docs/design/TIRE_INTERFACE_INVERSION.md.
    struct ContactInput {
        double Fz      {0.0};     // [N] vertical load
        double Vx      {0.0};     // [m/s] wheel-frame longitudinal contact velocity
        double Vy      {0.0};     // [m/s] wheel-frame lateral contact velocity
        double omega   {0.0};     // [rad/s] wheel spin
        double gamma   {0.0};     // [rad] camber
        double mu_long {1.0};
        double mu_lat  {1.0};
        double R0      {0.32};    // [m] unloaded radius
    };
    struct Transient {
        double belt_kappa   {0.0};   // MF path: relaxed slip ratio
        double belt_alpha   {0.0};   // MF path: relaxed slip angle [rad]
        double belt_vlong   {0.0};   // LuGre path: relaxed long. slip velocity [m/s]
        double belt_vlat    {0.0};   // LuGre path: relaxed lat. slip velocity [m/s]
        double lugre_z_long {0.0};
        double lugre_z_lat  {0.0};
        double alpha_dyn    {0.0};   // relaxation-length transient slip angle [rad]
    };
    struct Wrench {
        double Fx {0.0}, Fy {0.0};
        double Mx {0.0}, My {0.0}, Mz {0.0};
        double Re {0.0};
        double kappa {0.0}, alpha {0.0};
        double contact_dy {0.0};
        double mu_peak {0.0};
    };

    virtual ~ITireModel() = default;

    // Store the tire parameters (Re / camber / belt / relaxation / LuGre live here so the
    // inverted interface below works for every backend) and forward to the backend hook.
    void initialize(const TireParams& tp) { params_ = tp; on_initialize(tp); }

    virtual Output compute(const Input&) const noexcept = 0;

    // Inverted interface (Phase 2) — defined once in tire_model.cpp for ALL backends; it
    // computes slip / Re / camber-migration from the contact kinematics, applies the
    // transient, and dispatches the constitutive force law via the virtual compute().
    // evaluate() is the frozen per-RK4-stage force evaluation. The transient advances at
    // two cadences, so it has two integrators:
    //   advance_bristle()    — the LuGre bristle z (fast contact state): once per RK4 stage.
    //   advance_relaxation() — carcass/belt + relaxation-length lag (slow): once per substep
    //                          against the final stage's geometric slip.
    // Folding both into one dt is not byte-equivalent (a slow lag relaxed N times against
    // per-stage targets differs from once against the final target), hence the split.
    // See docs/design/TIRE_INTERFACE_INVERSION.md.
    Wrench    evaluate(const ContactInput&, const Transient&) const;
    Transient advance_bristle(const ContactInput&, const Transient&, double dt) const;
    Transient advance_relaxation(const ContactInput&, const Transient&, double dt) const;

protected:
    // Backend-specific setup hook (e.g. precompute stiffnesses). params_ is already stored.
    virtual void on_initialize(const TireParams&) {}
    TireParams params_ {};
};

std::unique_ptr<ITireModel> create_pacejka_mf96();
std::unique_ptr<ITireModel> create_linear_tire();

// Dispatch a tire force model from TireParams.backend:
//   "mf96" (default) -> create_pacejka_mf96; "linear" -> create_linear_tire;
//   "magic_formula" / "mf2002" -> MF2002 from tp.tir_path (throws if path empty).
std::unique_ptr<ITireModel> create_tire_from_params(const TireParams& tp);

// Dynamics factories that inject a custom tire model (e.g. full Magic Formula
// from a .tir).  Ownership transfers to the dynamics; for Ld3 the tire is
// forwarded to the inner Ld2.  Declared here, after ITireModel.
std::unique_ptr<IVehicleDynamics> create_bicycle(std::unique_ptr<ITireModel> tire);
std::unique_ptr<IVehicleDynamics> create_seven_dof(std::unique_ptr<ITireModel> tire);
std::unique_ptr<IVehicleDynamics> create_fourteen_dof(std::unique_ptr<ITireModel> tire);

// =============================================================================
// IContactProvider — 4-wheel contact information
// =============================================================================
class IContactProvider {
public:
    virtual ~IContactProvider() = default;
    virtual void query(const State& vehicle,
                       const VehicleParams& vparams,
                       ContactArray& out) = 0;
};

std::unique_ptr<IContactProvider> create_flat_ground(double z = 0.0,
                                                     double mu = 1.0);

// Split-mu plane: per-wheel friction by world-y (y >= boundary -> mu_left).
std::unique_ptr<IContactProvider> create_split_mu_ground(
    double z, double mu_left, double mu_right, double boundary_y = 0.0);

// Piecewise-x friction patches on a flat road (straight road s≈x).
std::unique_ptr<IContactProvider> create_friction_patch_ground(
    double z, double base_mu,
    const std::vector<std::tuple<double, double, double>>& patches);

// Inclined plane: grade [rad] (uphill toward +x), bank [rad] (up toward +y).
std::unique_ptr<IContactProvider> create_inclined_ground(
    double z0, double grade, double bank, double mu = 1.0);

// Half-cosine ramp + lip + cliff (T23 profile). x_start..x_top rise, lip, then drop.
std::unique_ptr<IContactProvider> create_ramp_ground(
    double x_start, double x_top, double height, double lip_length,
    double mu = 1.0);

// Vertical loop in x-z plane: center (xc, zc), radius R [m]. y ignored.
std::unique_ptr<IContactProvider> create_loop_ground(
    double xc, double zc, double radius, double mu = 1.0);

// Banked circular turn (velodrome / oval turn) in the x-y plane: reference
// circle radius R about (xc, yc), road cross-section banked inward by `bank`
// (outer edge higher). Height z = z0 + (rho - R)*tan(bank); the surface normal
// tilts toward the centre (centripetal) and up, so banking supplies part of the
// cornering force. rho is the radial distance from (xc, yc).
std::unique_ptr<IContactProvider> create_curved_ground(
    double xc, double yc, double radius, double bank, double z0 = 0.0,
    double mu = 1.0);

// Rough flat plane: two-tone road profile -> road_dz (L3 ride excitation).
std::unique_ptr<IContactProvider> create_rough_ground(
    double z, double mu, double amp, double wavelength = 4.0);

// Heightmap terrain: grid h[iy*nx+ix] at (x0+ix*dx, y0+iy*dy). Bilinear height +
// gradient normal per wheel (drives slope-gravity on arbitrary terrain).
std::unique_ptr<IContactProvider> create_heightmap_ground(
    std::vector<double> h, int nx, int ny,
    double x0, double y0, double dx, double dy, double mu = 1.0);

// ISO 8608 road roughness: a random profile synthesized from the standard
// spatial PSD Gd(n) = Gd(n0)*(n/n0)^-2 for road class 0=A (very good) .. 7=H
// (very poor). Independent left/right tracks (seeded) excite roll; the wheelbase
// gap between front/rear samples excites pitch. Feeds road_dz for the L3 ride.
std::unique_ptr<IContactProvider> create_iso8608_ground(
    double z, double mu, int road_class, unsigned seed = 1u);

// General PSD road: Gd(n)=Gd0 (n/n0)^-w single slope, or continuous dual-slope
// (exponent w below n_break, w_high above) for surface-specific spectra (e.g.
// Belgian pavé: small w_high -> strong short-wavelength content). n0=0.1.
std::unique_ptr<IContactProvider> create_psd_ground(
    double z, double mu, double gd_n0, double waviness = 2.0,
    double n_break = 0.0, double waviness_high = 2.0,
    double n_min = 0.011, double n_max = 4.0, unsigned seed = 1u);

// PSD road from a measured (n, Gd) table (n ascending, log-log interpolated) —
// plug a proving-ground RLDA spectrum directly.
std::unique_ptr<IContactProvider> create_psd_ground_table(
    double z, double mu, std::vector<double> n, std::vector<double> gd,
    double n_min = 0.011, double n_max = 10.0, unsigned seed = 1u);

// =============================================================================
// IRoughnessProvider — terrain roughness (Phase 2; reserved)
// =============================================================================
class IRoughnessProvider {
public:
    virtual ~IRoughnessProvider() = default;
    virtual double sample_height(const Vec2& world_xy) const = 0;
};

std::unique_ptr<IRoughnessProvider> create_flat();
std::unique_ptr<IRoughnessProvider> create_iso8608_psd(int grade);   // A=1..E=5

}  // namespace vdsim
