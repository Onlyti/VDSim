# Component samples (WS2 / WS4)

Reusable parameter snippets for the GUI workshops. Vehicle assembly (v0.2) references
parts by stem name:

| Part | Fleet / scenario field | Config path |
|------|------------------------|-------------|
| Chassis | `vehicle` | `configs/vehicles/<stem>.yaml` |
| Tire | `tire` | `configs/tires/<stem>.yaml` |
| Front kinematics | `front_susp` | `configs/suspensions/<stem>.yaml` |
| Rear kinematics | `rear_susp` | `configs/suspensions/<stem>.yaml` |

L3 runtime: `front_susp` / `rear_susp` attach via `create_native_kinematics_from_yaml`
in `vdsim_realtime`.

Subdirectories:

- `suspension/` — spring/damper presets (soft / med / stiff) for workshop UI.

Engine torque–RPM maps and brake curves are v0.3 (`V0.2_DRIVETRAIN.md`).
