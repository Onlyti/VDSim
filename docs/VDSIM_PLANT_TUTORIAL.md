# VDSim Plant — tutorial (closed-loop control plant)

A step-by-step guide to using **VDSim as the vehicle-dynamics plant** for a closed-loop
controller (e.g. an MPC). You call `step()`; VDSim integrates one control period and returns
ground-truth state + per-wheel tyre forces. Mid-fidelity (7-DOF + Pacejka MF2002 `.tir` with
load-dependent coefficients + load transfer), deterministic, sub-real-time — a drop-in upgrade
from a single-track bicycle plant.

> Status: **BETA** (first release). The API below is stable for use; expect refinements from
> early feedback. Branch `VDSim-Thesis`.

---

## 0. What you get

- Control input each step: `u = [delta_roadwheel_rad, Fx_total_N]` — the road-wheel steer angle
  and a longitudinal **force intent** (no pedal/throttle calibration; +Fx drives, −Fx brakes).
- Observation each step: true vehicle state + per-wheel tyre forces in the **contact frame**,
  with the actual friction each wheel used. Everything ISO 8855, wheels `FL=0,FR=1,RL=2,RR=3`.
- Real tyre saturation: a too-aggressive command **loses grip** (the tyre slips), it is not
  silently clamped — this is the point of using VDSim over a toy plant.
- Spatially varying friction (low-μ patches) for "unseen low-μ" scenarios.

---

## 1. Build (one time)

```bash
cd /path/to/VDSim
cmake -B build -DCMAKE_BUILD_TYPE=Release -DVDSIM_BUILD_PYTHON=ON   # first time
cmake --build build -j                                              # core + python module
```

`VDSIM_BUILD_PYTHON` defaults **OFF** — you must pass `-DVDSIM_BUILD_PYTHON=ON` to get the
`vdsim` pybind module under `build/python/`.

### Building for an external interpreter (conda / venv) — important

The pybind module is **ABI-locked to one Python**. A shipped `vdsim.cpython-38-*.so` will **not
import** under a different interpreter (e.g. a conda env on 3.11 — the typical acados / cvxpy
MPC setup). If your controller runs in its own env, rebuild the module **with that
interpreter**:

```bash
conda activate vla                       # your MPC env (e.g. python 3.11)
pip install pybind11                      # into that env
cmake -B build_vla -DCMAKE_BUILD_TYPE=Release -DVDSIM_BUILD_PYTHON=ON \
      -DPython3_EXECUTABLE="$(which python)" \
      -Dpybind11_DIR="$(python -m pybind11 --cmakedir)"
cmake --build build_vla -j
```

Then point `sys.path` at `build_vla/python` (not `build/python`). Symptom of a mismatch:
`ImportError: ... undefined symbol` or `module compiled against a different Python`.

---

## 2. Import the plant

`VDSimPlant` lives in `python/vdsim_plant.py`; the compiled `vdsim` module in `build/python/`.
Point Python at both:

```python
import sys
sys.path += ["/path/to/VDSim/build/python", "/path/to/VDSim/python"]
from vdsim_plant import VDSimPlant
```

---

## 3. Quickstart (copy-paste, runs in seconds)

```python
import sys
sys.path += ["/path/to/VDSim/build/python", "/path/to/VDSim/python"]
from vdsim_plant import VDSimPlant

plant = VDSimPlant(
    config="ioniq5_awd",     # bundled preset (configs/vehicles/ioniq5_awd.yaml)
    base_mu=0.9,             # uniform dry friction
    control_dt=0.05,         # one step() = 0.05 s of sim, held ZOH
    substep_dt=5e-4,         # internal fixed integration step (stiff wheel spin)
)

obs = plant.reset(state0=[0.0, 0.0, 0.0, 16.7, 0.0, 0.0])   # [X,Y,psi,vx,vy,r]; vx=16.7 m/s

for k in range(100):                     # 5 s
    delta = 0.03                         # rad, road-wheel
    Fx    = -1500.0                      # N, light brake (force intent)
    obs   = plant.step([delta, Fx])
    print(f"t={k*plant.control_dt:4.2f}  yaw_rate={obs['r']:+.3f}  ay={obs['ay']:+.2f}"
          f"  FL_kappa={obs['wheel'][0]['kappa']:+.3f}")
```

A ready-to-run version is `examples/vla_plant_demo.py`:

```bash
cd /path/to/VDSim && python3 examples/vla_plant_demo.py     # writes /tmp/vla_plant_demo.csv
```

---

## 4. The observation dict

`reset()` and `step()` both return one `obs` dict (true state — no sensor noise):

| key | meaning | units / frame |
|-----|---------|---------------|
| `X`, `Y` | position (world) | m, ISO 8855 ENU |
| `psi` | yaw | rad |
| `vx`, `vy` | body-frame velocity | m/s (+vx fwd, +vy left) |
| `r` | yaw rate | rad/s (+ = left turn) |
| `ax`, `ay` | body specific force (accelerometer) | m/s² |
| `beta` | side-slip `atan2(vy,vx)` | rad |
| `wheel[i]` | per wheel `i` (FL0,FR1,RL2,RR3) | dict below |
| `wheel[i].Fx`, `.Fy` | tyre force, **contact/wheel frame** | N (+Fx drive, +Fy left) |
| `wheel[i].Fz` | vertical load | N |
| `wheel[i].alpha` | slip angle | rad |
| `wheel[i].kappa` | slip ratio | – |
| `wheel[i].mu` | **road-surface** friction at this wheel (contact μ) | – |
| `wheel[i].mu_peak` | **realized** tyre peak coefficient at this Fz (load-dependent) | – |

Friction-circle check (on you, the analyst). Define the **friction saturation ratio**

```
sat = ‖[Fx,Fy]‖ / (mu_peak·Fz)          # use mu_peak, NOT the road mu
```

Use `mu_peak` in the denominator, **not** `mu` (the road μ): because the MF peak coefficient
*rises above* the nominal μ at low load (`PDY2<0`), a lightly loaded wheel can carry
`‖F‖ > mu·Fz`, so `‖F‖/(mu·Fz)` can read >1 even though the tyre is inside its own
load-dependent ellipse. `mu_peak` removes that ambiguity, and `sat ≤ 1`.

**`sat` is a saturation ratio, not a monotone "grip utilization".** The tyre force-slip curve
peaks then falls (sliding tail), so `sat=1` only *at* the peak slip; both the rising side
(reserve) and the sliding side (drift, past peak) read `sat<1`. A bare `sat=0.8` is therefore
ambiguous. Disambiguate with the slip channel:

| `sat` | slip angle | state |
|---|---|---|
| ≈1 | ≈ peak slip | at the grip limit |
| <1 | small | genuine reserve (rising side) |
| <1 | large | past peak — drift / departure (sliding tail) |

This is also why the plant exposes only raw ground truth (force, slip, Fz, mu_peak) and leaves
the metric to you: a single self-contained "grip-usage" scalar is ill-defined past the peak.

---

## 5. Control input `u = [delta, Fx_total]`

- `delta` — road-wheel steer angle [rad] (what an MPC outputs; no hand-wheel ratio).
- `Fx_total` — total longitudinal **force intent** [N], +drive / −brake. The plant converts it
  to per-wheel drive/brake torque (50/50 axle split, `drive_split_front`) and applies it to the
  wheel-spin dynamics, so the tyre's combined-slip limit is real:
  - moderate command → realised `ΣFx ≈ Fx_total`;
  - over-command → wheels slip/lock, realised `ΣFx < Fx_total` = physical grip loss.

---

## 6. Low-μ patches (spatially varying friction)

Pass `friction_map` as a list of `(x_start, x_end, mu)` along the road (world-x; straight road
⇒ s=x). Outside the patches, `base_mu` applies. Friction is per-wheel by contact x, so the
front axle enters a patch before the rear (real time lag).

```python
plant = VDSimPlant(
    config="ioniq5_awd",
    base_mu=0.9,
    friction_map=[(40.0, 60.0, 0.5)],   # μ=0.5 ice patch from x=40 m to x=60 m
    control_dt=0.05, substep_dt=5e-4,
)
```

---

## 7. Closed-loop with your controller (drop-in)

The plant replaces a Python bicycle plant with no controller change. Pattern:

```python
plant = VDSimPlant(config="ioniq5_awd", base_mu=0.9,
                   friction_map=[(40.0, 60.0, 0.5)],
                   control_dt=0.1, substep_dt=1e-3)   # control_dt = your MPC sample
obs = plant.reset(state0=[0, 0, 0, 16.7, 0, 0])
for k in range(N):
    # build the MPC state from obs (ISO 8855, same convention)
    x = [obs["X"], obs["Y"], obs["psi"], obs["vx"], obs["vy"], obs["r"]]
    delta, Fx = mpc.solve(x)             # your acados MPC — UNCHANGED
    obs = plant.step([delta, Fx])
```

To replace `~/vla_design/sim/closed_loop_sim.py`'s plant: keep the MPC; swap the plant object
for `VDSimPlant` and feed `step([delta, Fx])` instead of the bicycle `plant_deriv/rk4`.

Note: `control_dt` (the MPC sample, held ZOH) must be **≥** `substep_dt`, and `substep_dt`
must divide `control_dt` evenly. The MPC at dt=0.1 with `control_dt=0.1, substep_dt=1e-3` runs
100 internal substeps per control step.

---

## 8. Parameter file — `ioniq5_awd.yaml`

The bundled vehicle preset (`configs/vehicles/ioniq5_awd.yaml`), public Ioniq5-class:

```yaml
mass: 2359.0
mass_sprung: 2200.0
inertia_diag: [1200.0, 4200.0, 3400.0]   # [Ixx, Iyy, Izz]; Izz=3400 is the yaw inertia
wheelbase: 2.97
cg_to_front: 1.17                         # lf
cg_to_rear: 1.80                          # lr
track_front: 1.635
track_rear: 1.635
cg_height: 0.58
wheel_radius_nominal: 0.338
drive_type: AWD
drive_split_front: 0.5                    # 50/50 front/rear Fx split (drive AND brake)
plant_path: true                          # bypass the vehicle low-speed kinematic blend
differential: Open
steering_ratio: 14.0
aero_drag_coeff: 0.28
frontal_area: 2.45
tire_yaml: parts/tire/ioniq5_pac2002.yaml  # the tyre sidecar (below)
```

Tyre `configs/parts/tire/ioniq5_pac2002.yaml` (MF2002 `.tir` backend, LuGre OFF, combined-slip
ON). Unlike the old MF96 (constant B/C/E), the MF2002 coefficients are **load-dependent**, so
cornering stiffness and peak μ change with Fz — the load-transfer / limit behaviour an MPC
robustness study needs:

```yaml
backend: mf2002
tir_path: ioniq5_pac2002.tir   # Pacejka-2002 coefficient file (next to this yaml)
mu_nominal: 0.9
Fz_nominal: 5764.0
combined_slip_enabled: true
lugre: { enabled: false }
```

The `.tir` (`configs/parts/tire/ioniq5_pac2002.tir`, public-synthetic — no measured data) holds
the Pacejka-2002 coefficients. The load-shaping ones:

| coeff | role |
|---|---|
| `PKY1`, `PKY2` | cornering-stiffness magnitude + its **saturation with load** (Ca concave in Fz) |
| `PDY1`, `PDY2` | lateral peak factor + **peak-μ fall with load** (`μ_peak ↓` as Fz↑) |
| `PCY1`, `PEY1` | lateral shape / curvature |
| `PKX1`, `PDX1`, `PDX2`, `PCX1` | longitudinal stiffness, peak, load sensitivity, shape |

### Making your own vehicle

Copy `ioniq5_awd.yaml`, edit mass/geometry. The MF2002 cornering stiffness per wheel is
`Kya(Fz) = PKY1 · FNOMIN · sin( 2·atan( Fz / (PKY2·FNOMIN) ) ) · LKY` — load-dependent, so a
**single** `.tir` reproduces different front/rear axle stiffness automatically from each axle's
static Fz (no per-axle file needed). To target known axle stiffnesses, pick `PKY1` (overall
magnitude) and `PKY2` (where it saturates) so that `2·Kya(Fz_front)=Ca_f` and
`2·Kya(Fz_rear)=Ca_r`.

Worked example (Ioniq5, this preset): `PKY1=-24.6, PKY2=2.56` give
`Kya(7011 N)=109.3k` → axle `Ca_f≈218k` and `Kya(4558 N)=79.5k` → axle `Ca_r≈159k`
(targets 220k / 160k, ~1 %). Because `Kya` is concave, `Kya(2·Fz0)/Kya(Fz0)=1.435` (< 2 =
saturating, not linear), and `PDY2=-0.10` drops peak μ from 0.90 to 0.81 at 2·Fz0.

---

## 9. API reference

```python
VDSimPlant(config="ioniq5_awd",   # vehicle preset name or path
           friction_map=None,     # [(x0, x1, mu), ...] or None (uniform base_mu)
           base_mu=0.9,           # (0, 1.2]
           control_dt=0.05,       # s, one step() period (ZOH); >= substep_dt
           substep_dt=5e-4)       # s, internal fixed integration step; divides control_dt

plant.reset(state0=[X, Y, psi, vx, vy, r]) -> obs   # seeds wheel spin = vx / R_eff
plant.step([delta_rad, Fx_total_N])         -> obs   # advance one control period
```

Inputs are validated; bad ones raise a clear `ValueError`/`TypeError` (see Troubleshooting).

---

## 10. What's validated (so you can trust it)

Gated by `ctest -R VlaPlant` (394/394 suite green):
- **Dry handling = linear bicycle within ~4 %** (steady yaw-rate gain vs the single-track model
  with the same `Caf/Car`) — your bicycle results carry over. Checked for both the analytic-B
  tyre and the real MF2002 `.tir`.
- **Load-dependent tyre (MF2002)**: cornering stiffness concave in Fz
  (`Kya(2Fz0)/Kya(Fz0)=1.435 < 2`) and peak μ falls with load (`0.90→0.81` at 2·Fz0) — the
  load-transfer limit behaviour MF96 could not represent.
- **Friction circle held**: `‖[Fx,Fy]‖ ≤ mu·Fz` every step; over-command ⇒ real slip + the
  vehicle cannot deliver the commanded Fx (grip loss), and a brake never spins a wheel
  backwards.
- **Force ↔ accel consistency**: `ΣFy ≈ m·ay`.
- **Throttle bypass**: `Fx` is applied as torque to the wheel spin, not a pedal map.
- **Speed**: a 5 s trajectory runs in ~25 ms (≫ real time → cheap sweeps).

This is an independent-of-the-controller mid-fidelity plant; the MF2002 load-dependent tyre ≠
your bicycle internal (linear-Ca) model is an *intended* plant–model mismatch — a stronger
robustness story than a matched plant, and exactly the Fz-driven nonlinearity a σ_Fz /
chance-constraint study wants to stress.

---

## 11. Tips & gotchas

- **Determinism**: no RNG; identical inputs ⇒ identical outputs. Keep `substep_dt` fixed for
  reproducible figures.
- **Two-tier dt**: `control_dt` = your control rate; `substep_dt` ≤ 1 ms (wheel spin is stiff).
  Coarsening `substep_dt` can make hard-brake locking inaccurate.
- **Frames**: `wheel.Fx/Fy` are in the **contact (wheel) frame** — correct for the friction
  circle; do not re-rotate by steer.
- **Low-speed**: the plant uses a tyre VLOW floor (no kinematic-blend fade), so the saturating
  tyre stays active even near a stop.
- **Beta**: report rough edges — API and defaults may be tuned from your usage.

---

## 12. Troubleshooting

| symptom | cause / fix |
|---|---|
| `ValueError: friction_map[i]: x0>=x1` | patch start must be < end |
| `ValueError: ... mu=... outside (0, 1.2]` | friction must be in (0, 1.2] |
| `ValueError: substep_dt must divide control_dt evenly` | pick e.g. control_dt=0.05, substep_dt=5e-4 (×100) |
| `ValueError: step(u): u must be [delta_rad, Fx_total_N]` | pass a 2-element `[delta, Fx]` |
| `FileNotFoundError: ... config` | use a bundled name (`ioniq5_awd`) or an absolute YAML path |
| `import vdsim` fails | build first (`cmake --build build -j`) and add `build/python` to `sys.path` |
| car won't slow under braking | `Fx` is a force intent; on low μ the realised brake force is capped at grip — expected |
