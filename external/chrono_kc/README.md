# Chrono KC (suspension kinematics) parity gate

Independent cross-check of VDSim's hardpoint-emergent L5 suspension geometry against
[Project Chrono](https://projectchrono.org/)'s Chrono::Vehicle (BSD-3) — a separate
constrained-multibody engine. Confirms our camber / toe / track vs wheel travel match a
third-party MBD for the **same hardpoints**, **without taking Chrono as a dependency**.

This is the EXTERNAL companion to the internal check
`MultibodyKcXval.DaeTravelMatchesNativeKinematics` (which only proves our two own solvers —
native kinematics and the hard-joint DAE — agree). Chrono is a different engine and team, so
agreement here is real external validation. (Chrono is still a simulator, not a physical
KC-rig; physical / commercial KC-rig data remains a separate gap.)

## Isolation (why nothing mixes)

```
external/chrono_kc/                       <- Chrono touched ONLY here, built separately
  gen_kc_reference.cpp                     <- links Chrono; ChSuspensionTestRig sweep -> CSV
  CMakeLists.txt                           <- standalone; find_package(Chrono), NOT VDSim's tree
  reference/kc_dw_front_reference.csv      <- the only artifact that crosses back into the repo
tests/parity/test_chrono_kc_parity.cpp     <- VDSim side: reads the CSV, NO Chrono link
```

- VDSim's CMake / `libvdsim_core` never `find_package(Chrono)` and never link it.
- The gate (`ctest -R ChronoKcParity`) reads the CSV; if the CSV is absent it **SKIPs**
  (never fails the build for a missing external artifact).
- The shared input is the kin YAML
  `configs/parts/susp_kinematics/kin/dw_front_sports.yaml` — the generator builds the Chrono
  suspension from the SAME hardpoints (read it directly to avoid transcription drift), and the
  VDSim gate runs the hard-joint DAE on that same YAML.

## Hardpoint mapping (VDSim YAML -> Chrono ChDoubleWishbone::PointId)

| VDSim YAML key            | Chrono PointId        |
|---------------------------|-----------------------|
| `wheel.center`            | `SPINDLE`             |
| `uca.chassis_front`       | `UCA_F`               |
| `uca.chassis_rear`        | `UCA_B`               |
| `uca.knuckle`             | `UCA_U`               |
| `lca.chassis_front`       | `LCA_F`               |
| `lca.chassis_rear`        | `LCA_B`               |
| `lca.knuckle`             | `LCA_U`               |
| `tie_rod.rack`            | `TIEROD_C`            |
| `tie_rod.knuckle`         | `TIEROD_U`            |
| `spring_damper.chassis`   | `SPRING_C` / `SHOCK_C`|
| `spring_damper.lca`       | `SPRING_A` / `SHOCK_A`|
| (UCA_U + LCA_U) midpoint  | `UPRIGHT` (approx)    |

Frames: VDSim and Chrono::Vehicle are both ISO-ish (x fwd, y left, z up). Verify the rig's
output is reduced to the ISO convention (toe +, camber +, track outward) and that the
abscissa is the real vertical wheel-centre travel `z_v` (matches VDSim's `travel`).

## CSV format

```
travel_m,steer_m,camber_deg,toe_deg,track_mm,caster_deg
-0.060,0,<camber>,<toe>,<track>,<caster>
 ...                                          (travel sweep, steer = 0)
 0,<rack>,...                                  (optional steer sweep, travel = 0)
```

The gate currently bands the **travel sweep** (steer = 0) camber + toe (convention-free).
Steer-sweep rows are reserved for a future bump-steer cross-check once the steer-input
convention (rack travel m vs roadwheel rad) is reconciled.

## Build & regenerate

```
cmake -B build -DChrono_DIR=<chrono-build>/cmake
cmake --build build -j
./build/gen_kc_reference "$(pwd)/../.."     # writes reference/kc_dw_front_reference.csv
ctest -R ChronoKcParity                      # now active (was SKIP without the CSV)
```

## Status

SKELETON: `gen_kc_reference.cpp` has the structure + hardpoint mapping; the Chrono API
specifics (PointId spelling, `ChSuspensionTestRig` construction/readout, spring/damper
getters) are marked TODO and must be filled against the installed Chrono version. Tolerances
in the gate (rel 8 %, floor 0.1 deg) are placeholders for the joint-idealisation / mapping
residual — tighten once the first reference is captured. No reference CSV is committed yet, so
the parity gate SKIPs.
