#pragma once

#include <array>
#include <string>

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
    double roll_stiffness_front {30000.0};                             // [N m/rad]
    double roll_stiffness_rear  {25000.0};                             // [N m/rad]
    double anti_dive_front      {0.0};                                 // [0, 1] fraction
    double anti_squat_rear      {0.0};                                 // [0, 1] fraction
    double camber_per_roll      {0.0};                                 // [rad/rad] roll-to-camber gain

    // ---- Drivetrain ----
    enum class Drive { FWD, RWD, AWD };
    enum class Differential { Open, Locked, LSD };
    Drive  drive_type            {Drive::RWD};
    Differential differential    {Differential::Open};
    double lsd_preload           {0.10};    // [-] 0..0.5, baseline bias magnitude
    double lsd_ramp              {0.20};    // [-] per (rad/s) bias growth
    double max_motor_torque      {300.0};                              // [Nm]
    double max_brake_torque      {2000.0};                             // [Nm]
    double brake_bias_front      {0.5};                                // front share [0, 1]
    bool   brake_ebd_enabled     {false};                              // dynamic Fz-based bias

    // ---- Steering ----
    double steering_ratio        {15.0};                               // wheel/driver
    double max_steer_angle_wheel {0.5};                                // [rad]
    double ackerman_percent      {0.0};                                // 0 = parallel, 100 = perfect Ackerman

    // ---- Aero ----
    double aero_drag_coeff       {0.30};                               // Cd
    double frontal_area          {2.2};                                // [m^2]
    double aero_lift_front       {0.0};                                // Cl_front (positive = downforce)
    double aero_lift_rear        {0.0};                                // Cl_rear  (positive = downforce)

    static VehicleParams from_yaml(const std::string& path);
    void to_yaml(const std::string& path) const;
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

    // Camber thrust: extra Fy_camber = -camber_stiffness * gamma * Fz * mu
    double camber_stiffness      {0.0};         // [1/rad]  default off

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

    static TireParams from_yaml(const std::string& path);
    static TireParams from_tir(const std::string& path);   // AVL .tir (Phase 2)
    void              to_yaml (const std::string& path) const;
};

struct SolverParams {
    enum class Integrator { Euler, RK4 };
    Integrator integrator   {Integrator::RK4};
    double     max_substep_dt {1e-3};           // [s]
    int        max_substeps   {10};

    static SolverParams from_yaml(const std::string& path);
    void                to_yaml (const std::string& path) const;
};

}  // namespace vdsim
