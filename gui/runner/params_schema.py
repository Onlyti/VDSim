import vdsim



VEHICLES = ["sedan", "sports", "fsk_formula", "race_car"]
from catalog.levels import LEVELS  # K, L1–L5 — names in catalog/levels.py
_ALL = "K,L1,L2,L3,L4,L5"
_L1UP = "L1,L2,L3,L4,L5"
_L2UP = "L2,L3,L4,L5"
_L3STUNT = "L3,L4,L5"
_L3UP = "L3,L4"

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
    ("mass_sprung", "Sprung mass [kg]", "Mass & inertia", "num", _L3UP),
    ("ixx", "Roll inertia Ixx [kg·m²]", "Mass & inertia", "num", _L3STUNT),
    ("iyy", "Pitch inertia Iyy [kg·m²]", "Mass & inertia", "num", _L3STUNT),
    ("izz", "Yaw inertia Izz [kg·m²]", "Mass & inertia", "num", _L1UP),
    ("wheelbase", "Wheelbase [m]", "Geometry", "num", _ALL),
    ("cg_to_front", "CG→front [m]", "Geometry", "num", _ALL),
    ("cg_to_rear", "CG→rear [m]", "Geometry", "num", _ALL),
    ("track_front", "Track front [m]", "Geometry", "num", _L2UP),
    ("track_rear", "Track rear [m]", "Geometry", "num", _L2UP),
    ("cg_height", "CG height [m]", "Geometry", "num", _L1UP),
    ("wheel_radius_nominal", "Wheel radius [m]", "Geometry", "num", _ALL),
    ("spring_stiffness", "Spring stiffness [N/m]", "Suspension", "arr", "L2,L3,L4"),
    ("damper_coefficient", "Damper [N·s/m]", "Suspension", "arr", _L3UP),
    ("unsprung_mass", "Unsprung mass [kg]", "Suspension", "arr", _L2UP),
    ("wheel_inertia", "Wheel inertia [kg·m²] (0=auto)", "Suspension", "arr", _L1UP),
    ("arb_stiffness_front", "Anti-roll bar front", "Suspension", "num", "L2,L3,L4"),
    ("arb_stiffness_rear", "Anti-roll bar rear", "Suspension", "num", "L2,L3,L4"),
    ("roll_center_height_front", "Roll center height front [m]", "Suspension", "num", "L2,L3,L4"),
    ("roll_center_height_rear", "Roll center height rear [m]", "Suspension", "num", "L2,L3,L4"),
    ("anti_dive_front", "Anti-dive front [-] (typ 0–1)", "Suspension", "num", _L3UP),
    ("anti_squat_rear", "Anti-squat rear [-] (typ 0–1)", "Suspension", "num", _L3UP),
    ("camber_per_roll", "Camber/roll gain [rad/rad]", "Suspension", "num", _L3UP),
    ("drive_type", "Drive", "Drivetrain", "enum", _L1UP),
    ("differential", "Differential", "Drivetrain", "enum", _L2UP),
    ("lsd_preload", "LSD preload [-]", "Drivetrain", "num", _L2UP),
    ("lsd_ramp", "LSD ramp [-]", "Drivetrain", "num", _L2UP),
    ("max_motor_torque", "Max motor torque [N·m]", "Drivetrain", "num", _ALL),
    ("final_drive_ratio", "Final drive ratio [-]", "Drivetrain", "num", _ALL),
    ("drive_deadtime_s", "Throttle deadtime [s]", "Drivetrain", "num", _L2UP),
    ("max_brake_torque", "Max brake torque [N·m]", "Drivetrain", "num", _ALL),
    ("brake_bias_front", "Brake bias — front share [0–1] (rear = 1−front)", "Drivetrain", "num", _L1UP),
    ("brake_ebd_enabled", "Brake EBD (Fz-based bias)", "Drivetrain", "bool", _L2UP),
    ("brake_deadtime_s", "Brake deadtime [s]", "Drivetrain", "num", _L2UP),
    ("steering_ratio", "Steering ratio [-]", "Steering", "num", _L1UP),
    ("steer_deadtime_s", "Steer deadtime [s]", "Steering", "num", _L2UP),
    ("max_steer_angle_wheel", "Max steer [rad]", "Steering", "num", _ALL),
    ("ackerman_percent", "Ackermann [%]", "Steering", "num", _L2UP),
    ("aero_drag_coeff", "Drag coeff [-]", "Aero", "num", _L1UP),
    ("frontal_area", "Frontal area [m²]", "Aero", "num", _L1UP),
    ("aero_lift_front", "Lift coeff front [-]", "Aero", "num", _L1UP),
    ("aero_lift_rear", "Lift coeff rear [-]", "Aero", "num", _L1UP),
]
TIRE_FIELDS = [
    ("lugre.enabled", "LuGre dynamic tire (off → kinematic blend)", "LuGre", "bool", _L1UP),
    ("lugre.sigma0", "LuGre σ₀ [N/m]", "LuGre", "num", _L1UP),
    ("lugre.sigma1", "LuGre σ₁ [N·s/m] (0 → critical)", "LuGre", "num", _L1UP),
    ("lugre.sigma2", "LuGre σ₂ [N·s/m]", "LuGre", "num", _L1UP),
    ("lugre.m_eff", "LuGre m_eff [kg]", "LuGre", "num", _L1UP),
    ("B_long", "B long", "Longitudinal", "num", _L1UP),
    ("C_long", "C long", "Longitudinal", "num", _L1UP),
    ("D_long", "D long", "Longitudinal", "num", _L1UP),
    ("E_long", "E long", "Longitudinal", "num", _L1UP),
    ("B_lat", "B lat", "Lateral", "num", _L1UP),
    ("C_lat", "C lat", "Lateral", "num", _L1UP),
    ("D_lat", "D lat", "Lateral", "num", _L1UP),
    ("E_lat", "E lat", "Lateral", "num", _L1UP),
    ("mu_nominal", "μ nominal", "General", "num", _L1UP),
    ("Fz_nominal", "Fz nominal [N]", "General", "num", _L1UP),
    ("cornering_stiffness", "Cornering stiffness [N/rad]", "General", "num", _L1UP),
    ("rolling_resistance", "Rolling resistance", "General", "num", _L1UP),
    ("load_sensitivity", "Load sensitivity", "General", "num", _L1UP),
    ("combined_slip_enabled", "Combined slip (friction ellipse)", "General", "bool", _L1UP),
    ("pneumatic_trail", "Pneumatic trail [m]", "Aligning", "num", _L1UP),
    ("trail_falloff_alpha", "Trail falloff α [rad]", "Aligning", "num", _L1UP),
    ("camber_stiffness", "Camber stiffness [1/rad]", "Camber", "num", _L1UP),
    ("relaxation_length_lat", "Relaxation len lat [m]", "Transient", "num", _L1UP),
    ("relaxation_length_long", "Relaxation len long [m]", "Transient", "num", _L1UP),
    ("tire_vertical_stiffness", "Vertical stiffness [N/m]", "Vertical", "num", _L3STUNT),
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
