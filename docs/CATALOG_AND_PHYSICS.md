# Parts catalog, scenes, and physics options (v0.3+)

User-facing summary of what shipped after the v0.3 catalog cutover and the
v0.3+ physics extensions (drivetrain inertia, LuGre tire). Design specs:
[`design/PARTS_CATALOG.md`](design/PARTS_CATALOG.md),
[`design/V0.2_DRIVETRAIN.md`](design/V0.2_DRIVETRAIN.md),
[`design/V0.2_TIRE_LUGRE.md`](design/V0.2_TIRE_LUGRE.md).

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
  enabled: false    # default — keeps kinematic blend path
  sigma0: 200000.0   # bristle stiffness [N/m]
  sigma1: 0.0        # 0 → critical damping from m_eff
  sigma2: 50.0       # viscous [N·s/m]
  m_eff: 40.0        # contact mass for critical sigma1 [kg]
```

When `enabled: true` (L1 + L2 + L3 via inner L2):

- MF96 supplies the steady-state friction envelope `g()`.
- Per-wheel bristle states `z` (long/lat); semi-implicit update each substep.
- Kinematic blend, brake-hold damper, and lambda force fades are **skipped**.
- `relaxation_length_*` is ignored (LuGre handles transients).

When `enabled: false` (default): unchanged low-speed path documented in
[`design/LOW_SPEED_HANDLING.md`](design/LOW_SPEED_HANDLING.md).

## Verification

```bash
cmake --build build -j
cd build && ctest --output-on-failure    # 201 tests (2026-06-06)
```

Relevant ctests: `catalog_*`, `scene_materialize`, `import_part_pack`,
`EngineInertiaSlowsLowMuWheelSpinup`, `LuGreTire/*`, `TireYaml.LuGreRoundtrip`.
