#pragma once

#include <array>
#include <memory>
#include <vector>

#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"
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
    virtual std::array<double, NUM_WHEELS> tire_Fz()           const = 0;  // [N]
    virtual std::array<double, NUM_WHEELS> wheel_slip_ratio()  const = 0;  // [-]
    virtual std::array<double, NUM_WHEELS> wheel_slip_angle()  const = 0;  // [rad]

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
        double Mz {0.0};          // [N m] aligning moment
    };

    virtual ~ITireModel() = default;
    virtual void   initialize(const TireParams&)         = 0;
    virtual Output compute   (const Input&) const noexcept = 0;
};

std::unique_ptr<ITireModel> create_pacejka_mf96();
std::unique_ptr<ITireModel> create_linear_tire();

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
