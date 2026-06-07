import vdsim



VEHICLES = ["sedan", "sports", "fsk_formula", "race_car"]
LEVELS = ["K", "L1", "L2", "L3"]   # K = kinematic bicycle (no tire/slip)
_ALL = "K,L1,L2,L3"

# Enum value maps (name <-> bound enum) for dropdown fields.
ENUM_MAPS = {
    "drive_type":   {"FWD": vdsim.Drive.FWD, "RWD": vdsim.Drive.RWD,
                     "AWD": vdsim.Drive.AWD},
    "differential": {"Open": vdsim.Differential.Open,
                     "Locked": vdsim.Differential.Locked,
                     "LSD": vdsim.Differential.LSD},
    "integrator":   {"Euler": vdsim.Integrator.Euler, "RK4": vdsim.Integrator.RK4},
}

# Editable parameter schema: (attr, label, group, kind, applicable_levels)
#   kind: "num" scalar | "arr" 4-wheel | "bool" checkbox | "enum" dropdown
VEHICLE_FIELDS = [
    ("mass", "Mass [kg]", "Mass & inertia", "num", _ALL),
    ("mass_sprung", "Sprung mass [kg]", "Mass & inertia", "num", "L3"),
    ("ixx", "Roll inertia Ixx [kg·m²]", "Mass & inertia", "num", "L3"),
    ("iyy", "Pitch inertia Iyy [kg·m²]", "Mass & inertia", "num", "L3"),
    ("izz", "Yaw inertia Izz [kg·m²]", "Mass & inertia", "num", "L1,L2,L3"),
    ("wheelbase", "Wheelbase [m]", "Geometry", "num", _ALL),
    ("cg_to_front", "CG→front [m]", "Geometry", "num", _ALL),
    ("cg_to_rear", "CG→rear [m]", "Geometry", "num", _ALL),
    ("track_front", "Track front [m]", "Geometry", "num", "L2,L3"),
    ("track_rear", "Track rear [m]", "Geometry", "num", "L2,L3"),
    ("cg_height", "CG height [m]", "Geometry", "num", "L1,L2,L3"),
    ("wheel_radius_nominal", "Wheel radius [m]", "Geometry", "num", _ALL),
    ("spring_stiffness", "Spring stiffness [N/m]", "Suspension", "arr", "L2,L3"),
    ("damper_coefficient", "Damper [N·s/m]", "Suspension", "arr", "L3"),
    ("unsprung_mass", "Unsprung mass [kg]", "Suspension", "arr", "L2,L3"),
    ("wheel_inertia", "Wheel inertia [kg·m²] (0=auto)", "Suspension", "arr", "L1,L2,L3"),
    ("arb_stiffness_front", "Anti-roll bar front", "Suspension", "num", "L2,L3"),
    ("arb_stiffness_rear", "Anti-roll bar rear", "Suspension", "num", "L2,L3"),
    ("roll_center_height_front", "Roll center height front [m]", "Suspension", "num", "L2,L3"),
    ("roll_center_height_rear", "Roll center height rear [m]", "Suspension", "num", "L2,L3"),
    ("anti_dive_front", "Anti-dive front [-] (typ 0–1)", "Suspension", "num", "L3"),
    ("anti_squat_rear", "Anti-squat rear [-] (typ 0–1)", "Suspension", "num", "L3"),
    ("camber_per_roll", "Camber/roll gain [rad/rad]", "Suspension", "num", "L3"),
    ("drive_type", "Drive", "Drivetrain", "enum", "L1,L2,L3"),
    ("differential", "Differential", "Drivetrain", "enum", "L2,L3"),
    ("lsd_preload", "LSD preload [-]", "Drivetrain", "num", "L2,L3"),
    ("lsd_ramp", "LSD ramp [-]", "Drivetrain", "num", "L2,L3"),
    ("max_motor_torque", "Max motor torque [N·m]", "Drivetrain", "num", _ALL),
    ("final_drive_ratio", "Final drive ratio [-]", "Drivetrain", "num", _ALL),
    ("drive_deadtime_s", "Throttle deadtime [s]", "Drivetrain", "num", "L2,L3"),
    ("max_brake_torque", "Max brake torque [N·m]", "Drivetrain", "num", _ALL),
    ("brake_bias_front", "Brake bias — front share [0–1] (rear = 1−front)", "Drivetrain", "num", "L1,L2,L3"),
    ("brake_ebd_enabled", "Brake EBD (Fz-based bias)", "Drivetrain", "bool", "L2,L3"),
    ("brake_deadtime_s", "Brake deadtime [s]", "Drivetrain", "num", "L2,L3"),
    ("steering_ratio", "Steering ratio [-]", "Steering", "num", "L1,L2,L3"),
    ("steer_deadtime_s", "Steer deadtime [s]", "Steering", "num", "L2,L3"),
    ("max_steer_angle_wheel", "Max steer [rad]", "Steering", "num", _ALL),
    ("ackerman_percent", "Ackermann [%]", "Steering", "num", "L2,L3"),
    ("aero_drag_coeff", "Drag coeff [-]", "Aero", "num", "L1,L2,L3"),
    ("frontal_area", "Frontal area [m²]", "Aero", "num", "L1,L2,L3"),
    ("aero_lift_front", "Lift coeff front [-]", "Aero", "num", "L1,L2,L3"),
    ("aero_lift_rear", "Lift coeff rear [-]", "Aero", "num", "L1,L2,L3"),
]
TIRE_FIELDS = [
    ("lugre.enabled", "LuGre dynamic tire (off → kinematic blend)", "LuGre", "bool", "L1,L2,L3"),
    ("lugre.sigma0", "LuGre σ₀ [N/m]", "LuGre", "num", "L1,L2,L3"),
    ("lugre.sigma1", "LuGre σ₁ [N·s/m] (0 → critical)", "LuGre", "num", "L1,L2,L3"),
    ("lugre.sigma2", "LuGre σ₂ [N·s/m]", "LuGre", "num", "L1,L2,L3"),
    ("lugre.m_eff", "LuGre m_eff [kg]", "LuGre", "num", "L1,L2,L3"),
    ("B_long", "B long", "Longitudinal", "num", "L1,L2,L3"),
    ("C_long", "C long", "Longitudinal", "num", "L1,L2,L3"),
    ("D_long", "D long", "Longitudinal", "num", "L1,L2,L3"),
    ("E_long", "E long", "Longitudinal", "num", "L1,L2,L3"),
    ("B_lat", "B lat", "Lateral", "num", "L1,L2,L3"),
    ("C_lat", "C lat", "Lateral", "num", "L1,L2,L3"),
    ("D_lat", "D lat", "Lateral", "num", "L1,L2,L3"),
    ("E_lat", "E lat", "Lateral", "num", "L1,L2,L3"),
    ("mu_nominal", "μ nominal", "General", "num", "L1,L2,L3"),
    ("Fz_nominal", "Fz nominal [N]", "General", "num", "L1,L2,L3"),
    ("cornering_stiffness", "Cornering stiffness [N/rad]", "General", "num", "L1,L2,L3"),
    ("rolling_resistance", "Rolling resistance", "General", "num", "L1,L2,L3"),
    ("load_sensitivity", "Load sensitivity", "General", "num", "L1,L2,L3"),
    ("combined_slip_enabled", "Combined slip (friction ellipse)", "General", "bool", "L1,L2,L3"),
    ("pneumatic_trail", "Pneumatic trail [m]", "Aligning", "num", "L1,L2,L3"),
    ("trail_falloff_alpha", "Trail falloff α [rad]", "Aligning", "num", "L1,L2,L3"),
    ("camber_stiffness", "Camber stiffness [1/rad]", "Camber", "num", "L1,L2,L3"),
    ("relaxation_length_lat", "Relaxation len lat [m]", "Transient", "num", "L1,L2,L3"),
    ("relaxation_length_long", "Relaxation len long [m]", "Transient", "num", "L1,L2,L3"),
    ("tire_vertical_stiffness", "Vertical stiffness [N/m]", "Vertical", "num", "L3"),
]
# Actuator + feedback schema. Dotted paths walk the nested ActuatorParams; the
# two "@" names are handled specially (sensor delay + solver substeps).
ACTUATOR_FIELDS = [
    ("steer.ch.dead_time_s", "Steer dead time [s]", "Steering", "num"),
    ("steer.ch.tau_s", "Steer lag τ [s]", "Steering", "num"),
    ("steer.ch.rate_limit", "Steer rate limit [rad/s] (0=off)", "Steering", "num"),
    ("steer.friction.enabled", "Servo+LuGre mode (off → first-order lag)", "Steering", "bool"),
    ("steer.servo_kp", "Servo kp", "Steering", "num"),
    ("steer.servo_kd", "Servo kd", "Steering", "num"),
    ("throttle.dead_time_s", "Throttle dead time [s]", "Throttle", "num"),
    ("throttle.tau_s", "Throttle lag τ [s]", "Throttle", "num"),
    ("throttle.rate_limit", "Throttle rate limit [1/s] (0=off)", "Throttle", "num"),
    ("throttle.dead_zone", "Throttle dead-zone [-] (pedal tip-in)", "Throttle", "num"),
    ("brake.ch.dead_time_s", "Brake dead time [s]", "Brake", "num"),
    ("brake.ch.tau_s", "Brake lag τ [s]", "Brake", "num"),
    ("brake.ch.dead_zone", "Brake dead-zone [-] (pad clearance)", "Brake", "num"),
    ("brake.thermal_enabled", "Brake thermal fade", "Brake", "bool"),
    ("@sensor_delay_s", "Sensor feedback delay [s]", "Feedback", "num"),
]
# Sensor noise/bias schema (dotted paths into SensorParams). "enabled" is a bool.
SENSOR_FIELDS = [
    ("enabled", "Sensors enabled (off → truth)", "General", "bool"),
    ("imu_accel.noise_std", "IMU accel noise [m/s²]", "IMU", "num"),
    ("imu_accel.bias", "IMU accel bias [m/s²]", "IMU", "num"),
    ("imu_gyro.noise_std", "IMU gyro noise [rad/s]", "IMU", "num"),
    ("imu_gyro.bias", "IMU gyro bias [rad/s]", "IMU", "num"),
    ("imu_gyro.bias_rw", "IMU gyro bias random-walk", "IMU", "num"),
    ("wheel_speed.noise_std", "Wheel-speed noise [rad/s]", "Wheel", "num"),
    ("steer.noise_std", "Steer noise [rad]", "Steer", "num"),
    ("steer.bias", "Steer bias [rad]", "Steer", "num"),
    ("gnss_pos.noise_std", "GNSS position noise [m]", "GNSS", "num"),
    ("gnss_vel.noise_std", "GNSS velocity noise [m/s]", "GNSS", "num"),
]
