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
    double roll_stiffness_front {30000.0};                             // [N m/rad]
    double roll_stiffness_rear  {25000.0};                             // [N m/rad]

    // ---- Drivetrain ----
    enum class Drive { FWD, RWD, AWD };
    Drive  drive_type            {Drive::RWD};
    double max_motor_torque      {300.0};                              // [Nm]
    double max_brake_torque      {2000.0};                             // [Nm]

    // ---- Steering ----
    double steering_ratio        {15.0};                               // wheel/driver
    double max_steer_angle_wheel {0.5};                                // [rad]

    // ---- Aero ----
    double aero_drag_coeff       {0.30};                               // Cd
    double frontal_area          {2.2};                                // [m^2]

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
    double rolling_resistance    {0.015};

    static TireParams from_yaml(const std::string& path);
    static TireParams from_tir(const std::string& path);   // AVL .tir (Phase 2)
};

struct SolverParams {
    enum class Integrator { Euler, RK4 };
    Integrator integrator   {Integrator::RK4};
    double     max_substep_dt {1e-3};           // [s]
    int        max_substeps   {10};
};

}  // namespace vdsim
