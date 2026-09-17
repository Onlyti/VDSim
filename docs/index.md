# VDSim

**Open-core vehicle dynamics simulator** — bridging the autonomy stack
(throttle / pedal / path) and chassis design (hardpoint / multibody) in a
single C++17 + Python ABI.

[GitHub](https://github.com/Onlyti/VDSim){ .md-button .md-button--primary }
[PoC summary v3](tasks/68_poc_summary_v3/README.md){ .md-button }
[Theory](theory/README.md){ .md-button }

---

## Status

- **Release line**: v0.5 terrain + Ld5 stunt driving; parts catalog + scene runtime
  (`--scene=`), GUI catalog API.
- **Physics**: LuGre tire default on (`lugre.enabled`); opt-in **MF2002 `.tir` backend**
  + **belt transient** (L1–L5); opt-in **drivetrain v2** (`powertrain:` — 2D engine map +
  gearbox + shift policy); kinematic fallback via `tire.kinematic_fallback` or `--no-lugre`.
- **Trace & visualization (in integration)**: opt-in `.vdtrace` recording and headless BEV rendering;
  single-run playback, multi-run comparison, and separate **waypoint**, **target**, and
  **prediction** overlays.
- **Render presets (in integration)**: declarative panel and BEV-layer selection,
  including the built-in `overview` layout.
- **C2 road contact (in integration)**: L3/L4 consume the full 3-D surface normal,
  including grade/bank gravity and normal-load response. The road-bundle provider
  remains pending.
- **Tests**: the current canonical result, configuration, toolchain, exclusions, commit,
  and measurement date live only in [Validation](VALIDATION.md).
- **Roadmap**: [Product roadmap](ROADMAP.md) — shipped vs planned by subsystem.
- **User guide**: [Catalog & physics options](CATALOG_AND_PHYSICS.md).
- **10 CLI demo binaries**, **Python bindings**, **CARLA-ready raycast ABI**,
  **Web GUI** (3D viewer + catalog/scene authoring).

## Two ladders

VDSim 의 차별화 — single ABI 안에서 두 사다리 m × n 매트릭스.

### Dynamics ladder (fidelity)

| Tier | DOF | Status |
|---|---|---|
| Ld1-Bicycle | 5 | done |
| Ld2-SevenDOF | 7 | done |
| Ld3-FourteenDOF | 14 (sprung 3 + unsprung 4) | done |
| Ld4-MultibodyKinematic | hardpoint FK + hard-joint DAE | **shipped M1–M7** ([spec](design/LD4_MULTIBODY.md)) |
| Ld5-Free3D | quaternion 6-DOF + hub contact | **shipped** (v0.4 stunt); terrain → v0.5 |

### Control ladder (abstraction)

| Tier | Input | Status |
|---|---|---|
| Lc1-PerWheel | motor / brake torque per wheel | via lowering |
| Lc2-AxleTorque | axle drive / brake | via lowering |
| Lc3-FxTotal | longitudinal force | via lowering |
| **Lc4-Pedal** | **throttle / brake / steer** (CARLA-호환) | **primary** |
| Lc5-AxTarget | ax_target | done (PI + FF) |
| Lc6-VTarget | v_target | done (cascade PI) |
| Lc7-PathCurvature | v_target + κ (Pure Pursuit) | done |
| Lc8-Waypoint | path + lookahead | done (figure-8 demo) |

## Quick start

```bash
git clone https://github.com/Onlyti/VDSim
cd VDSim
cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Debug \
      -DVDSIM_BUILD_PYTHON=ON -DVDSIM_BUILD_CARLA_PLUGIN=ON
cmake --build build -j

# Run a scene (catalog fleet + maneuvers materialized at launch)
build/bin/vdsim_realtime --scene=configs/scenes/l3_sedan_kinematics.yaml

# Launch the 3D viewer (real-time web viewer + live sim)
python3 gui/server.py --port 8100 &
# → browse http://localhost:8100
```

## What to read

- **First time**: [Theory overview](theory/README.md) → chapters 01-04 sequentially.
- **Cherry-pick**: jump to [Ld3-FourteenDOF](theory/06_ld3_fourteen_dof.md)
  or [Pure Pursuit](theory/09_pure_pursuit_path.md).
- **Sales / PT**: [PoC summary v3](tasks/68_poc_summary_v3/README.md)
  and [Competitive matrix](tasks/69_competitive_matrix/README.md).
- **Implementation tour**: [Software architecture](theory/12_software_architecture.md).
- **Roadmap to multibody**: [Multibody outlook](theory/13_multibody_outlook.md).
