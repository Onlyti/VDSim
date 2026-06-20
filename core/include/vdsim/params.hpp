#pragma once

#include <array>
#include <cmath>
#include <string>

#include "vdsim/powertrain.hpp"
#include "vdsim/types.hpp"

namespace vdsim {

struct VehicleParams {
    // ---- Mass ----
    double mass         {1500.0};                                      // total [kg]
    double mass_sprung  {1350.0};                                      // [kg]
    Vec3   inertia_diag {Vec3(500.0, 2000.0, 2500.0)};                 // Ixx, Iyy, Izz of sprung [kg m^2]

    // ---- Geometry ----
    double wheelbase            {2.7};                                 // L [m]
    double cg_to_front          {1.2};                                 // a [m]
    double cg_to_rear           {1.5};                                 // b [m]
    double track_front          {1.55};                                // Tw_f [m]
    double track_rear           {1.55};                                // Tw_r [m]
    double cg_height            {0.55};                                // h_cg [m]
    double wheel_radius_nominal {0.32};                                // [m]

    // ---- L3 suspension (per wheel) ----
    std::array<double, NUM_WHEELS> spring_stiffness   {{30000, 30000, 30000, 30000}};  // [N/m]
    std::array<double, NUM_WHEELS> damper_coefficient {{3000,  3000,  3000,  3000}};   // [N s/m]
    std::array<double, NUM_WHEELS> unsprung_mass      {{40,    40,    40,    40}};     // [kg]
    // Wheel rotational inertia about spin axis [kg m^2]. 0 = auto-derive from the
    // solid-disk approximation 0.5 * unsprung_mass * wheel_radius^2; a positive
    // value overrides it (rim+tire is not a uniform disk, so measured I differs).
    std::array<double, NUM_WHEELS> wheel_inertia      {{0, 0, 0, 0}};
    // Anti-roll bar roll stiffness per axle [N m/rad]. The spring contribution to
    // roll stiffness is derived from spring_stiffness x track (see dynamics), so
    // this is the ARB-only addition. 0 = no anti-roll bar.
    double arb_stiffness_front  {0.0};                                 // [N m/rad]
    double arb_stiffness_rear   {0.0};                                 // [N m/rad]
    // Roll-center height per axle [m] above ground. Sets the geometric (jacking)
    // vs elastic split of lateral load transfer. 0 = roll axis at ground (all
    // sprung-mass transfer is elastic, through the springs/ARB).
    double roll_center_height_front {0.0};                             // [m]
    double roll_center_height_rear  {0.0};                             // [m]
    double anti_dive_front      {0.0};                                 // [0, 1] fraction
    double anti_squat_rear      {0.0};                                 // [0, 1] fraction
    double camber_per_roll      {0.0};                                 // [rad/rad] roll-to-camber gain
    // Stunt / jump: progressive bump stop as multiple of static corner load (educative).
    double spring_bump_ratio    {3.0};                                 // [-]
    double spring_droop_ratio   {0.0};                                 // [0,1] min spring force fraction

    // ---- Drivetrain ----
    enum class Drive { FWD, RWD, AWD };
    enum class Differential { Open, Locked, LSD };
    Drive  drive_type            {Drive::RWD};
    bool   plant_path            {false};
    Differential differential    {Differential::Open};
    double lsd_preload           {0.10};    // [-] 0..0.5, baseline bias magnitude
    double lsd_ramp              {0.20};    // [-] per (rad/s) bias growth
    double max_motor_torque      {300.0};                              // [Nm] peak motor (pre-gear)
    double final_drive_ratio     {5.0};                                // [-] motor->wheel torque mult
    // Crankshaft + flywheel inertia at engine [kg m^2], reflected to wheels via
    // final_drive_ratio^2. 0 disables (legacy wheel-only spin-up).
    double engine_rotational_inertia {0.25};
    double max_brake_torque      {2000.0};                             // [Nm]
    double brake_bias_front      {0.5};                                // front share [0, 1]
    bool   brake_ebd_enabled     {false};                              // dynamic Fz-based bias

    // Engine + gearbox (Drivetrain v2). Opt-in: a YAML `powertrain:` block sets
    // `powertrain.enabled`; otherwise the legacy flat torque above is used and the
    // ISO baseline is unchanged. See powertrain.hpp.
    PowertrainParams powertrain;

    // ---- Actuator transport deadtime (subsystem modules) ----
    double brake_deadtime_s      {0.0};                                // [s] pedal->brake lag
    double drive_deadtime_s      {0.0};                                // [s] throttle->drive lag
    double steer_deadtime_s      {0.0};                                // [s] handwheel->steer lag

    // ---- Steering ----
    double steering_ratio        {15.0};                               // wheel/driver
    double max_steer_angle_wheel {0.5};                                // [rad]
    double ackerman_percent      {0.0};                                // 0 = parallel, 100 = perfect Ackerman
    // Dynamic-steering Rack EOM params (used only when ISteeringSystem outputs mode=Dynamic).
    bool   steering_dynamic {false}; // true → DynamicSteering (Rack EOM, accepts torque/rate)
    double rack_mass     {30.0};    // [kg] effective rack + reflected column/motor inertia
    double rack_damping  {800.0};   // [N/(m/s)] rack viscous damping
    double pinion_radius {0.0075};  // [m] pinion pitch radius (rack travel per pinion rad)
    double caster_trail  {0.03};    // [m] mechanical + pneumatic trail (Fy → rack force)

    // ---- Aero ----
    double aero_drag_coeff       {0.30};                               // Cd
    double frontal_area          {2.2};                                // [m^2]
    double aero_lift_front       {0.0};                                // Cl_front (positive = downforce)
    double aero_lift_rear        {0.0};                                // Cl_rear  (positive = downforce)

    static VehicleParams from_yaml(const std::string& path);
    void to_yaml(const std::string& path) const;
};

// Axle roll stiffness [N m/rad] derived from the corner springs plus the ARB.
// Two springs at half-track produce K = (k_left + k_right) * (track/2)^2; the
// anti-roll bar adds its own roll-only rate. axle: 0 = front, 1 = rear.
inline double axle_roll_stiffness(const VehicleParams& vp, int axle) {
    const int l   = axle ? WHEEL_RL : WHEEL_FL;
    const int r   = axle ? WHEEL_RR : WHEEL_FR;
    const double tw_half = 0.5 * (axle ? vp.track_rear : vp.track_front);
    const double k_spring = (vp.spring_stiffness[l] + vp.spring_stiffness[r])
                          * tw_half * tw_half;
    const double arb = axle ? vp.arb_stiffness_rear : vp.arb_stiffness_front;
    return k_spring + (arb > 0.0 ? arb : 0.0);
}

struct LuGreTireParams {
    bool   enabled {true};
    double sigma0  {9.0e4};    // bristle stiffness [N/m]
    double sigma1  {0.0};      // micro-damping [N·s/m]; 0 -> critical from m_eff
    double sigma2  {75.0};     // viscous [N·s/m]
    double m_eff   {40.0};     // contact mass for critical sigma1 [kg]
};

// Belt / carcass first-order slip relaxation (Phase T2). Opt-in; filters the
// geometric slip (kappa, alpha) that feeds the MF tire model with tau = sigma/|Vx|
// so step-steer / brake-release transients have carcass lag. See belt_tire.hpp.
struct BeltTireParams {
    bool   enabled   {false};
    double sigma_lat  {0.5};   // lateral relaxation length sigma_y [m]
    double sigma_long {0.5};   // longitudinal relaxation length sigma_x [m]
};

struct TireParams {
    // Pacejka MF96 simple form:
    //   F = D * sin(C * atan(B*s - E*(B*s - atan(B*s))))
    // Longitudinal (slip ratio kappa)
    double B_long {10.0};
    double C_long {1.65};
    double D_long {1.0};    // peak normalized to Fz * mu
    double E_long {0.97};
    // Lateral (slip angle alpha)
    double B_lat  {8.0};
    double C_lat  {1.30};
    double D_lat  {1.0};
    double E_lat  {-1.0};

    // Friction
    double mu_nominal {1.0};
    double Fz_nominal {4000.0};                 // [N] reference for MF

    // Linear region (for validation / fallback)
    double cornering_stiffness   {80000.0};     // C_alpha [N/rad]
    double rolling_resistance    {0.0};         // 0 = off (analytical baseline). Typical 0.010-0.015.

    // Combined slip (friction ellipse) and aligning moment
    bool   combined_slip_enabled {true};        // false -> Fx, Fy decoupled (legacy)
    double pneumatic_trail       {0.05};        // [m] t_p_0, Mz = -t_p * Fy
    double trail_falloff_alpha   {0.20};        // [rad] t_p decay scale

    // Camber thrust: extra Fy_camber = +camber_stiffness * gamma * Fz * mu
    // (ISO 8855, y=left: positive inclination -> +y thrust).
    double camber_stiffness      {0.0};         // [1/rad]  default off

    // Tread crown radius (transverse curvature). A cambered tire contacts on the
    // leaning side -> contact point migrates dy = crown_radius * sin(gamma),
    // producing an overturning moment Fz*dy. 0 = off (contact stays at centerline).
    double crown_radius          {0.0};         // [m]  default off

    // Load sensitivity:  μ_eff(Fz) = μ_nominal · (1 - load_sensitivity · (Fz/Fz_nominal - 1))
    // Floor μ_eff at 0.3 · μ_nominal to keep numerics sane at very high Fz.
    // Typical 0.10 – 0.25.  0.0 = legacy (no load sensitivity).
    double load_sensitivity      {0.0};

    // Transient response (relaxation length).  σ / |v_long| acts as 1st-order
    // lag on slip; tire force responds to a transient slip α_dyn / κ_dyn that
    // satisfies σ/|v| · ṡ_dyn = s_geom − s_dyn.  Stored on the dynamics state,
    // NOT inside the stateless compute() — these are advisory params for the
    // host integrator (Ld2 / Ld3 / Ld1).  0.0 = legacy (instant response).
    double relaxation_length_lat   {0.0};       // σ_y [m]
    double relaxation_length_long  {0.0};       // σ_x [m]

    // Vertical tire stiffness (for L3 ride dynamics). Typical 150-300 kN/m.
    double tire_vertical_stiffness {220000.0};  // [N/m]

    // Effective rolling radius (Pacejka): the slip ratio uses Re, not the unloaded
    // radius, so a free-rolling loaded tire reports kappa = 0 (no phantom drive slip).
    //   Re = R0 - (Fz0/Cz)·(DREFF·atan(BREFF·rho_n) + FREFF·rho_n),  rho_n = Fz/Fz0
    // BREFF=DREFF=FREFF=0 → Re falls back to the unloaded radius (legacy behaviour).
    double reff_breff {0.0};   // [-] BREFF
    double reff_dreff {0.0};   // [-] DREFF
    double reff_freff {0.0};   // [-] FREFF

    // Force backend selector (T1):
    //   "mf96"          — parametric Pacejka MF96 from the B/C/D/E fields above (default)
    //   "magic_formula" — full MF2002 evaluated from a `.tir` at `tir_path`
    //   "linear"        — linear cornering-stiffness fallback
    // `tir_path` is a runtime path; `.tir` files hold (often confidential) measured
    // coefficients and are never committed (see `.gitignore`).
    std::string backend  {"mf96"};
    std::string tir_path {};

    LuGreTireParams lugre;
    BeltTireParams  belt;

    // True when the tire model itself produces combined-slip forces (so the host
    // must NOT re-clip them with its circular friction ellipse): LuGre (combined)
    // or the MF2002 evaluator (Gxa/Gyk weighting). MF96 returns false -> the host
    // ellipse couples its decoupled Fx/Fy.
    bool model_provides_combined_slip() const {
        return combined_slip_enabled &&
               (lugre.enabled || backend == "magic_formula" || backend == "mf2002");
    }

    static TireParams from_yaml(const std::string& path);
    static TireParams from_tir(const std::string& path);   // AVL .tir (Phase 2)
    void              to_yaml (const std::string& path) const;
};

// Per-wheel tire setup (FL/FR/RL/RR). Unified and per-axle constructors duplicate
// params across corners; optional per-corner YAML/catalog slots override individually.
struct TireSetup {
    std::array<TireParams, NUM_WHEELS> wheel {};

    TireSetup() = default;
    explicit TireSetup(const TireParams& unified) { wheel.fill(unified); }
    TireSetup(const TireParams& front_axle, const TireParams& rear_axle) {
        wheel[WHEEL_FL] = wheel[WHEEL_FR] = front_axle;
        wheel[WHEEL_RL] = wheel[WHEEL_RR] = rear_axle;
    }
    TireSetup(const TireParams& fl, const TireParams& fr,
              const TireParams& rl, const TireParams& rr) {
        wheel = {{fl, fr, rl, rr}};
    }
    explicit TireSetup(std::array<TireParams, NUM_WHEELS> w) : wheel(w) {}

    const TireParams& for_wheel(int w) const { return wheel[w]; }
    const TireParams& for_axle(int axle) const {
        return axle ? wheel[WHEEL_RL] : wheel[WHEEL_FL];
    }

    static TireSetup from_yaml_paths(const std::string& front_path,
                                     const std::string& rear_path = {});
    static TireSetup from_corner_yaml_paths(const std::string& fl_path,
                                            const std::string& fr_path = {},
                                            const std::string& rl_path = {},
                                            const std::string& rr_path = {});
};

struct SolverParams {
    enum class Integrator { Euler, RK4 };
    Integrator integrator   {Integrator::RK4};
    double     max_substep_dt {1e-3};           // [s]
    int        max_substeps   {10};
    bool       stunt_physics  {false};          // world-z, airborne, full gravity (L3+/L5)
    double     loop_radius    {0.0};            // [m] 0=off; loop track active
    double     loop_center_x  {0.0};
    double     loop_center_z  {0.0};
    bool       loop_rail_guide {false};         // true=kinematic rail; false=tire-driven arc
    // L5 (free_3d) spatial suspension (opt-in). false -> legacy penalty-at-hub contact
    // (body held up by a capped tire-stiffness penalty, no unsprung). true -> per-corner
    // unsprung mass + body-frame strut spring/damper on the 6-DOF body; the body becomes
    // the sprung mass and the tire-spring acts on the unsprung. See free_3d_dynamics.cpp.
    bool       l5_spatial_suspension {false};

    static SolverParams from_yaml(const std::string& path);
    void                to_yaml (const std::string& path) const;
};

// Effective rolling radius Re(Fz) — the radius at which a free-rolling loaded tire
// satisfies vx = omega·Re (so slip = (omega·Re - vx)/vx is zero at free roll).
// Pacejka form using BREFF/DREFF/FREFF; falls back to R0 when those are zero.
//   rho_n = Fz / Fz0   (load ratio)
//   Re    = R0 - (Fz0/Cz)·(DREFF·atan(BREFF·rho_n) + FREFF·rho_n)
inline double effective_rolling_radius(const TireParams& tp, double R0, double Fz) {
    if (tp.reff_breff == 0.0 && tp.reff_dreff == 0.0 && tp.reff_freff == 0.0)
        return R0;                                  // legacy: unloaded radius
    const double Cz  = tp.tire_vertical_stiffness > 1.0 ? tp.tire_vertical_stiffness : 220000.0;
    const double Fz0 = tp.Fz_nominal > 1.0 ? tp.Fz_nominal : 4000.0;
    const double rho_n = (Fz > 0.0 ? Fz : 0.0) / Fz0;
    const double Re = R0 - (Fz0 / Cz) *
                      (tp.reff_dreff * std::atan(tp.reff_breff * rho_n) + tp.reff_freff * rho_n);
    return Re > 0.05 ? Re : R0;                      // guard against degenerate params
}

// Static per-wheel vertical load [N] on level ground (no aero, no load transfer).
// Front axle carries m·g·b/L, rear m·g·a/L; split evenly L/R. Order FL,FR,RL,RR.
inline std::array<double, NUM_WHEELS> static_wheel_loads(const VehicleParams& vp) {
    constexpr double g = 9.80665;
    const double L = vp.wheelbase > 1e-6 ? vp.wheelbase : 2.7;
    const double Fz_f = 0.5 * vp.mass * g * vp.cg_to_rear  / L;
    const double Fz_r = 0.5 * vp.mass * g * vp.cg_to_front / L;
    return {{Fz_f, Fz_f, Fz_r, Fz_r}};
}

// Free-rolling wheel spin [rad/s] consistent with the effective rolling radius, so a
// tire initialised at this spin reports slip=0 (matches MF-Tyre/CarMaker static init).
// Per-wheel because front/rear static load -> different Re. Falls back to vx/R0 when
// the tire has no Re coefficients (reff_*=0).
inline std::array<double, NUM_WHEELS> free_roll_wheel_spin(
        const VehicleParams& vp, const TireSetup& ts, double vx) {
    const auto Fz = static_wheel_loads(vp);
    const double R0 = vp.wheel_radius_nominal > 1e-6 ? vp.wheel_radius_nominal : 0.32;
    std::array<double, NUM_WHEELS> w{};
    for (int i = 0; i < NUM_WHEELS; ++i) {
        const double Re = effective_rolling_radius(ts.for_wheel(i), R0, Fz[i]);
        w[i] = Re > 1e-6 ? vx / Re : 0.0;
    }
    return w;
}

inline std::array<double, NUM_WHEELS> free_roll_wheel_spin(
        const VehicleParams& vp, const TireParams& tp, double vx) {
    return free_roll_wheel_spin(vp, TireSetup(tp), vx);
}

}  // namespace vdsim
