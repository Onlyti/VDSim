# VDSim ↔ CARLA Python bridge

Drives a CARLA actor with VDSim's dynamics. CARLA renders + sensors + traffic;
VDSim is the sole authority for the ego vehicle's motion.

## Architecture

```
┌──────────────────────────────────────────────────┐
│  CARLA UE4 server  (CarlaUE4.sh)                  │
│  - world / map / sensors / NPC traffic            │
│  - cast_ray(start, end)  for ground contacts      │
│  - rendering + spectator camera                   │
└──────────────────────────────────────────────────┘
              ▲ contacts via raycast                
              │                                     
              ▼ set_transform / set_target_velocity 
┌──────────────────────────────────────────────────┐
│  VDSimCarlaBridge (Python)                        │
│  - per tick: query 4-wheel contacts               │
│  - run VDSim dynamics.step()                      │
│  - ISO 8855 RH → CARLA / UE4 frame conversion     │
└──────────────────────────────────────────────────┘
              ▲ control input (Lc4-Pedal)           
              │                                     
              ▼ telemetry (vx, ay, Fz, ...)         
┌──────────────────────────────────────────────────┐
│  User control logic                               │
│  - Pure Pursuit + Lc6 PI (built-in demo)          │
│  - or external MPC / driver model / keyboard      │
└──────────────────────────────────────────────────┘
```

## Quick start

```bash
# 1. Build VDSim with Python bindings
cd ~/git/VDSim
cmake -G Ninja -B build -DVDSIM_BUILD_PYTHON=ON
cmake --build build -j

# 2. Install CARLA Python client (matching server version)
pip install carla==0.9.15

# 3. Start CARLA server (separately)
~/path-to-carla/CarlaUE4.sh -RenderOffScreen -nosound

# 4. Run the demo (figure-8 with Pure Pursuit on sports config)
python3 carla_integration/python/run_demo.py \
    --vehicle configs/vehicles/sports.yaml \
    --tire    configs/tires/default_pacejka.yaml \
    --level   L2 \
    --duration 30 \
    --v_target 12 \
    --driver
```

## Frame conventions

VDSim is **ISO 8855 RH** (+x forward, +y leftward, +z up).
CARLA / UE4 is **left-handed** (+x forward, +y rightward, +z up).

Mapping applied inside the bridge:

| Quantity | VDSim → CARLA |
|---|---|
| `x` | `+x` |
| `y` | `-y` |
| `z` | `+z` |
| `yaw` (rad) | `-yaw` (deg) |
| `roll` | `+roll` |
| `pitch` | `-pitch` (sign convention is empirical for PoC) |

All velocities go through `carla_velocity_from_vdsim_body` which combines the
yaw-rotation back to world frame plus the y-flip.

## Why disable CARLA's own physics

`self.actor.set_simulate_physics(False)` makes VDSim the sole authority. CARLA
still updates the world (sensors, NPCs) but no longer applies wheel forces or
gravity to the ego — we set the transform directly each tick.

This is the standard "external physics" pattern used by VI-grade, IPG, and
proprietary motion platforms when integrating with CARLA.

## Mapping tiers

| `--level` | VDSim impl | When to use |
|---|---|---|
| `L1` | Ld1-Bicycle (5 DOF) | controller dev, MPC inner loop |
| `L2` | Ld2-SevenDOF (7 DOF) | ADAS validation, per-tire Fz |
| `L3` | Ld3-FourteenDOF (14 DOF) | ride + roll/pitch visualisation |

## Limits (PoC v1)

| Item | Limit |
|---|---|
| Surface mu lookup | single `--default_mu` (no per-material) |
| Contact normal | assumed `Vec3::UnitZ()` (flat-ground simplification) |
| Ground z from CARLA | not yet used — VDSim integrates from spawn |
| Roll/pitch sign convention | UE4 ↔ ISO 8855 empirical mapping; verify on your map |
| World-rel coordinates | spawn-anchored (VDSim integrates relative to spawn pose) |
| Sensors integration | not yet — cameras / LiDAR work but VDSim does not consume |

## Next steps

- `RaycastContactProvider` (C++ side, `carla_integration/plugin/`) is the
  matching ABI for the UE5 plugin build (Phase 2).
- Per-material μ lookup table reading CARLA's `PhysicalMaterial`.
- Drive recording → CSV → 3D viewer replay.
- Pose / velocity drift comparison vs CARLA's stock TM (traffic manager).
