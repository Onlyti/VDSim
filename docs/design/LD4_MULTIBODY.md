# Ld4 — hardpoint multibody (M1–M7)

Status: **M1–M7 implemented** on branch `feat/v0.4-slope-jump-m5` (2026-06-08).
**254/254** ctest green.

## 1. Position in the dynamics ladder

| Tier | Model | Runtime default |
|------|--------|-----------------|
| Ld3 | 14-DOF sprung + unsprung, lookup / native kin attach | `level=L3` |
| **Ld4** | Ld3 + hardpoint topology + **hard-joint corner DAE** | `level=L4` |
| Ld5 | Free 6-DOF body (quaternion), hub contact | `level=L5` stunt |

Ld4 **does not** replace Ld3 tire or sprung-mass integration. It replaces per-corner
wheel pose source: from tabulated / native FK alone → **hard-joint pose + optional
bushing compliance (off in L4 step)**.

## 2. Architecture

```text
kin YAML (per corner)
    │
    ├─► SuspensionTopology::from_yaml  (M2 graph: bodies, joints, hardpoints)
    │
    ├─► IMultibodySolver::forward_kinematics     (M1 FK)
    ├─► solve_quasi_static_compliance            (M3 bushings, design-time)
    ├─► run_kc_sweep / run_kc_xcheck             (M5–M6 offline)
    ├─► build_revolute_tree + RNEA               (M7 offline, not in L4 step)
    │
    └─► IHardJointDaeModel::step                 (M4 runtime, L4 step)
            │
            ▼
    FourteenDOFDynamics (L4_Kinematic)
        tire contact → axle Fy → step_hard_joint_dae
```

### Runtime vs offline

| Path | Used in `vdsim_realtime` L4 step | Purpose |
|------|----------------------------------|---------|
| Hard-joint DAE + Baumgarte travel | **Yes** | Corner θ dynamics under prescribed travel |
| Bushing `step_corner_dynamics` | No (retained API) | Quasi-static / future compliant Ld4 |
| Featherstone `step_revolute_link_tree` | **No** | Open-chain 1-DOF validation, v0.6+ |
| K&C sweep / Adams x-check | No (GUI / CLI / pybind) | Design verification |

## 3. Milestones (M1–M7)

### M1 — Kinematics attach

- `Level::L4_Kinematic`, `create_fourteen_dof_kinematic()`
- `core/src/multibody_kinematics.cpp` — `create_kinematic_solver()`, `attach_topology_*`
- Steer rack: `0.08 m/rad` → `ISuspensionKinematics::compute(travel, rack_dy)`
- Scene: `configs/scenes/l4_sedan_kinematics.yaml`

### M2 — Topology graph

- `core/src/multibody_topology.cpp` — kin YAML → `hardpoints`, inferred `bodies` / `joints`
- Kinds: `macpherson`, `double_wishbone`, `trailing_arm`, `five_link`
- `SuspensionTopology::to_yaml` round-trip

### M3 — Quasi-static compliance

- `core/src/multibody_compliance.cpp` — default bushings per topology
- `quasi_static_compliance` → `compliance_toe_deg` / `compliance_camber_deg`
- Linear bushing K, Fz scaling

### M4 — Hard-joint corner DAE (production path)

- `core/src/multibody_hard_dae.cpp` — `IHardJointDaeModel`
- **Common pattern:** one driving revolute + algebraic loop closure + Baumgarte
  `z_wheel = z_static + travel`

| Topology | Driving DOF | Loop closure |
|----------|-------------|--------------|
| Trailing arm | Arm pivot θ | — |
| MacPherson | LCA θ | Strut cylinder + tie-rod LM |
| Double wishbone | LCA θ | UCA sphere + tie-rod trilateration |
| 5-link | Lower-aft θ | 5-link LM (+ full pose at dt=0) |

- L4 step: `compliance_toe_rad = 0` (bushing dynamics disabled)
- Tests: `HardJointDae.*`, `FourteenDOF.L4MultibodyDynamicsInStep`

### M5 — K&C sweep charts

- `core/src/multibody_kc_sweep.cpp` — travel ±100 mm, steer rack ±40 mm, Fy compliance
- GUI: `GET /api/suspension/kc?name=` → 7 plots
- `vdsim.run_kc_sweep(kin_yaml)` pybind

### M6 — Adams import cross-check

- `core/src/multibody_kc_xcheck.cpp` — 5 gain metrics, default rtol **5%**
- `tools/kinematics/adams_xcheck.py` — CSV → YAML → metric compare
- `vdsim.run_kc_xcheck(ref, cand)` pybind

### M7 — Lumped revolute dynamics (offline)

- `core/src/multibody_featherstone.cpp` — RNEA + composite-body `M`
- `build_revolute_tree(topo)` — **1-DOF lump** per corner (not full loop tree):

| Topology | Pivot | Axis |
|----------|-------|------|
| TA | `arm_pivot.chassis_inboard` | inboard → outboard |
| MP / DW | `lca.chassis_front` | LCA front → rear |
| 5-link | `links.lower_aft.chassis` | +Y |

- **Not wired** into L4 step. Full multi-link Featherstone → v0.6+.

## 4. Config layout

```text
configs/parts/susp_kinematics/
  kin/mp_front_sedan.yaml      # native ISuspensionKinematics + topology source
  kin/dw_front_sports.yaml
  kin/ta_rear_sedan.yaml
  kin/5link_rear_sports.yaml
```

Wheel order everywhere: **FL=0, FR=1, RL=2, RR=3**. Frame: ISO 8855 RH (+x fwd, +y left, +z up).

### Optional `knuckle:` block (wheel-centre-relative)

Hardpoints are body-frame absolute, but the knuckle (hub carrier) is naturally
designed relative to the **wheel centre**. An optional top-level `knuckle:` block lets
the kin file express the knuckle attachment points and the steering knuckle-arm point as
offsets from `wheel.center`; the parser resolves each to an absolute hardpoint
`knuckle.<name>` / `knuckle.arm` (body `knuckle`):

```yaml
knuckle:
  ref: wheel_center          # offsets are relative to wheel.center
  points:
    lca:     [-0.02, -0.06, -0.10]   # LCA->knuckle joint, rel. to wheel centre
    tie_rod: [-0.30, -0.07, -0.05]
  arm:       [-0.12, -0.05,  0.02]   # steering knuckle-arm point
```

Additive: the legacy kin files omit it, so the topology — and the hard-DAE dynamics,
which read the absolute `lca.knuckle` / `tie_rod.knuckle` fields — are unchanged. The
resolved `knuckle.*` points are added to the topology for design / K&C / future steering
geometry; they are not yet consumed by the L4 DAE. Test
`MultibodyTopology.KnuckleBlockWheelCenterRelative`.

## 5. Python / GUI entry points

```python
import vdsim
topo = vdsim.mb.SuspensionTopology.from_yaml("configs/parts/susp_kinematics/kin/mp_front_sedan.yaml")
sweep = vdsim.run_kc_sweep("configs/parts/susp_kinematics/kin/mp_front_sedan.yaml")
report = vdsim.run_kc_xcheck("ref.yaml", "cand.yaml", rtol=0.05)
```

CLI:

```bash
python3 tools/kinematics/adams_xcheck.py --csv export.csv --reference ref.yaml
python3 gui/server.py   # Suspension modal → K&C charts (L3/L4)
```

Cosim / realtime:

```bash
build/bin/vdsim_realtime --scene=configs/scenes/l4_sedan_kinematics.yaml
# vehicle level L4 in scene / catalog
```

## 6. Tests (representative)

| Suite | File |
|-------|------|
| Topology / FK / compliance / hard DAE | `tests/unit/test_multibody_kinematics.cpp` |
| Featherstone | `tests/unit/test_multibody_featherstone.cpp` |
| L4 in 14-DOF step | `tests/integration/test_fourteen_dof.cpp` |
| K&C GUI API | `tests/scripts/test_suspension_kc.py` |
| Adams x-check | `tests/scripts/test_adams_xcheck.py` |

## 7. Known limits

- Hard-joint lateral load at fixed travel: Baumgarte z-constraint → **q̈≈0** (by design).
- M7 open-chain q̈ under lateral Fy **≠** hard DAE (unconstrained lump).
- Inertia lump + rotation math now live in **`multibody_math.{hpp,cpp}`**
  (`rodrigues`, `axis_angle_to_R`, `corner_inertia_about_axis`,
  `lump_corner_about_axis`); both the hard DAE and Featherstone trees call it, so the
  inertia model has a single definition (v0.6 M1 — was the "keep in sync" hazard).
- Beam axle / twist beam: kinematic FK fallback only.

## 8. Next (post-M7)

1. v0.5 terrain + L5 hub contact — [`V0.5_TERRAIN_L5.md`](V0.5_TERRAIN_L5.md) — **done**
2. Shared `multibody_math` helpers (rodrigues, inertia lump) — **done (v0.6 M1)**
3. Full loop Featherstone or augmented Lagrangian (v0.6+)
4. ISO re-baseline on flat (`run_validation.py`) — **done**
