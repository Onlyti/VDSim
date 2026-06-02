#pragma once

#include <array>
#include <memory>

#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"
#include "vdsim/types.hpp"

namespace vdsim {

// =============================================================================
// IVehicleDynamics — top-level vehicle dynamics interface
// =============================================================================
class IVehicleDynamics {
public:
    enum class Level { L1_Bicycle, L2_SevenDOF, L3_FourteenDOF };

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
};

std::unique_ptr<IVehicleDynamics> create_bicycle();
std::unique_ptr<IVehicleDynamics> create_seven_dof();
std::unique_ptr<IVehicleDynamics> create_fourteen_dof();

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
