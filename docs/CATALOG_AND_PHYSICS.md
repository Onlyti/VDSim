# Parts catalog, scenes, and physics options (v0.3+)

User-facing summary of what shipped after the v0.3 catalog cutover and the
v0.3+ physics extensions (drivetrain inertia, LuGre tire). Design specs:
[`design/PARTS_CATALOG.md`](design/PARTS_CATALOG.md),
[`design/V0.2_DRIVETRAIN.md`](design/V0.2_DRIVETRAIN.md),
[`design/V0.2_TIRE_LUGRE.md`](design/V0.2_TIRE_LUGRE.md).
Roadmap: [`ROADMAP.md`](ROADMAP.md).

## Layout

```text
configs/catalog/manifest.yaml     # builtin part + blueprint index
configs/parts/**                  # chassis, tire, drivetrain, susp_*, …
configs/blueprints/**             # vehicle = parts + level
configs/scenes/**                 # fleet[] + gui (saved runs)
configs/maneuvers/**              # time-varying driver / mu (was scenarios/)
```

Legacy flat paths (`configs/vehicles/`, `tires/`, `scenarios/`) are **removed**.
Resolve presets in Python via `vdsim_lab.Vehicle.preset()` / `Tire.preset()`
(catalog-backed).

## Dynamics ladder (K, L1–L5)

Runtime / YAML / GUI use **K, L1–L5**; theory chapters use **Ld0–Ld5** (Ld0 = K).
Canonical names: `python/catalog/levels.py`.

| ID | Ld | Name | Model |
|----|-----|------|--------|
| **K** | Ld0 | Kinematic | Kinematic bicycle — no tire slip |
| **L1** | Ld1 | Bicycle | Single-track bicycle (5-DOF planar) |
| **L2** | Ld2 | Seven-DOF | Planar seven-DOF — 3 chassis + 4 wheel ω, per-wheel Pacejka, Ackermann |
| **L3** | Ld3 | Fourteen-DOF | Ride fourteen-DOF — sprung + unsprung vertical |
| **L4** | Ld4 | Hardpoint | Hardpoint kinematic multibody (L3 + attach kin) |
| **L5** | Ld5 | Free-3D | Free 3D stunt — 6-DOF body, hub contact, airborne |

L2 is **not** “3-DOF Ackermann”: the chassis is planar 3-DOF (x, y, yaw); the
ladder name **Seven-DOF** counts four wheel-spin states. Ackermann steering is one
feature of L2, not the level name.

## Computational cost per step

**Benchmark** (ioniq5_awd, dt=5e-4, MF2002 tire, flat ground):

| Level | Model | ms/step | Realtime factor |
|-------|-------|--------:|-----------------|
| **K** | Kinematic bicycle | 0.0005 | ~1000× |
| **L1** | Bicycle | 0.0162 | ~31× |
| **L2** | Seven-DOF | 0.0313 | ~16× |
| **L3** | Fourteen-DOF | 0.0317 | ~16× |
| **L4** | Hardpoint multibody | 0.0321 | ~15× |
| **L5** | Free-3D stunt | 0.0354 | ~14× |

**Notes:**
- **K** has no tire slip / dynamics; ~30× faster than L1.
- **L2–L4** are dominated by MF2002 tire evaluation; L3/L4 nearly identical (L4 hardpoint kinematics not yet wired to suspension geometry in this test).
- **L5** adds 3D orientation + strut DAE (~10% slower than L4).
- Measurement includes `SimSession.tick()` overhead (ground contact, actuator lag, sensor filtering).

**Tool:** `python3 tools/bench_dynamics_levels.py` (Python L0–L4) + `build/bin/vdsim_bench_levels` (C++ L5).

## Running a scene

```bash
build/bin/vdsim_realtime --scene=configs/scenes/two_vehicle_race.yaml \
    --cmd-port=7001 --state-port=7002 --rate=200
```

At launch the runtime materializes each fleet entry (blueprint + part overrides)
into per-vehicle `vehicle.yaml` / `tire.yaml` (+ L3 suspension paths). Flat
`vehicle.yaml tire.yaml` argv is no longer supported.

Python batch / lab:

```python
from vdsim_lab import Vehicle, Tire
vp = Vehicle.preset("sedan").vp
tp = Tire.preset("default_pacejka").tp
```

## GUI catalog API (M4)

| Endpoint | Role |
|----------|------|
| `GET /api/catalog` | manifest index; `?type=` `?q=` |
| `GET /api/catalog/parts`, `/parts/:id` | list / load part envelope + body |
| `GET /api/catalog/blueprints/:id` | blueprint doc |
| `GET /api/scene/list`, `/scene/:name` | saved scenes |
| `POST /api/scene`, `/scene/save` | import / save scene v3 |
| `GET/POST /api/simconfig` | scene **version 3** (v2 rejected) |
| `GET/POST /api/runconfig` | materialized world (`vehicles[]` + `gui`) |

Legacy shims: `/api/parts/registry`, `/api/suspension/list`, `/api/scenario/list`.

## External catalog packs (M5)

```bash
python3 tools/import_part_pack.py /path/to/pack          # dry-run + id collision check
python3 tools/import_part_pack.py /path/to/pack --install
```

Install target: `configs/catalog/packages/<package_id>/`. Id collision with
builtin catalog is an error (no override).

**GUI parts library** (Vehicle Edit → Parts library): registers chassis/tire YAML
into `configs/catalog/packages/gui_custom/` and merges via manifest `packages:`.
Builtin parts are read-only — clone to customize. Import: YAML file or `.tir` (tire).

## Drivetrain engine inertia

**Param:** `VehicleParams.engine_rotational_inertia` [kg·m²] at the crank
(catalog: `configs/parts/drivetrain/*.yaml`). Default `0.25`.

Reflected at the wheels: `I_refl = I_engine × final_drive_ratio²`, split per
driven axle and coupled on open diffs (`core/include/vdsim/drivetrain_inertia.hpp`).

| Differential | Wheel spin model |
|--------------|------------------|
| Open | wheel-only `dω` + carrier coupling between left/right |
| Locked / LSD | `dω = T_net / (I_wheel + I_engine_share)` |

Set `engine_rotational_inertia: 0` for legacy wheel-only spin-up.

ISO / accel benchmarks may shift vs pre-inertia baselines — re-run
`apps/validation/run_validation.py` when re-baselining.

## LuGre dynamic tire (opt-in)

**Param:** `TireParams.lugre` (catalog: `lugre:` block under tire parts).

```yaml
lugre:
  enabled: true     # catalog default (tire.default_pacejka); set false for kinematic fallback
  sigma0: 300000.0   # bristle stiffness [N/m]
  sigma1: 0.0        # 0 → critical damping from m_eff
  sigma2: 120.0      # viscous [N·s/m]
  m_eff: 40.0        # contact mass for critical sigma1 [kg]
```

**Kinematic fallback:** `tire.kinematic_fallback` (`lugre.enabled: false`) — MF96 +
kinematic blend + brake-hold (legacy ISO baseline). CLI: `--no-lugre`.

When `enabled: true` (L1 + L2 + L3 via inner L2):

- MF96 supplies the breakaway envelope `g()` at current slip; floors are
  **asymmetric**: longitudinal `g_x = max(|Fx_MF|, 1 N)` (no `mu*Fz` — avoids
  phantom drive/brake at rest); lateral `g_y = max(|Fy_MF|, w(alpha)*mu_lat*Fz, 1 N)`
  with `w = exp(-(alpha/0.03)^2)` for presliding cornering stiffness.
- Per-wheel bristle states `z_long`, `z_lat`; semi-implicit update; RK4 advances
  `z` four times per substep (`h/4` after each stage).
- Combined slip: peak-axis friction ellipse (`D*mu*Fz` per axis), same as MF96
  task 15; outer circular `mu*Fz` clip is skipped when LuGre + combined slip.
- Lateral force: `Fy = -F_raw` so ISO 8855 sign matches Pacejka.
- Wheel spin couples to the same clipped LuGre `Fx` (`fx_kin`).
- Kinematic blend, brake-hold damper, and lambda force fades are **skipped**.
- `relaxation_length_*` is ignored (LuGre handles transients).

When `enabled: false` (`tire.kinematic_fallback` or `--no-lugre`): low-speed path in
[`design/LOW_SPEED_HANDLING.md`](design/LOW_SPEED_HANDLING.md).

Theory: [Chapter 19 — LuGre / brush-dynamic tire](theory/19_lugre_dynamic_tire.md).

**Try it**

| Path | How |
|------|-----|
| Catalog part | Default `tire.default_pacejka` (LuGre on); fallback `tire.kinematic_fallback` |
| CLI | `--no-lugre` (kinematic fallback) / `--lugre` (force on) |
| GUI | Tire panel → **LuGre** group → *LuGre dynamic tire* toggle (saved in scene `fleet_overrides` on Play) |
| Python | `Tire.preset().lugre(True)` or `Experiment().tire(...)` |
| Demo | `python3 examples/lugre_demo.py` · scene `configs/scenes/lugre_grade_demo.yaml` |

## L4 kinematic multibody

Blueprint `level: L4` (or scene fleet entry) uses `create_fourteen_dof_kinematic()`:
native suspension kin YAML per corner + **hard-joint corner DAE** in the step loop.

| Item | Location |
|------|----------|
| Kin parts | `configs/parts/susp_kinematics/kin/*.yaml` |
| Example scene | `configs/scenes/l4_sedan_kinematics.yaml` |
| Design spec | [`design/LD4_MULTIBODY.md`](design/LD4_MULTIBODY.md) |
| K&C charts (GUI) | Suspension modal → `/api/suspension/kc` |
| Adams x-check | `tools/kinematics/adams_xcheck.py` |

## Verification

```bash
cmake --build build -j
cd build && ctest --output-on-failure    # 254 tests (2026-06-08)
```

Relevant ctests: `catalog_*`, `scene_materialize`, `import_part_pack`,
`EngineInertiaSlowsLowMuWheelSpinup`, `LuGreTire/*`, `TireYaml.LuGreRoundtrip`.
