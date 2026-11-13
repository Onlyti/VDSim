# Work order 51 — acceptance gates fixed before implementation

Aero ride-height / pitch slopes and the aero user module (vdsim 0.9.0).
Base: origin/main faf59f2 (canonical 649). This file, `handcalc.py` and
`b2_expected.json` are committed before any implementation or measurement.
Numbers below do not change after measurement. A gate that misses stops the
work and is reported; measurement-condition fixes and harness corrections are
done first and reported afterwards (PROP-2775 item 3), gate numbers never.

- `handcalc.py` sha256: `7245be5206a5123ee78749eb73b1e672445fd1ef6e58768cd6f49c0387dbfdfd`
  (re-fix before the re-measurement, PO mail 91b88183: per-path B3'-L5 slope,
  `b3l5_select`, `b3l5i`; the first fixed file was `3c9c201653155c7b…eabd5`)
- `b2_expected.json` sha256: `a2db9722c3a72b94fd403a996ee3bba9e2640777d1f82193ed4d8f497de2aa4c` (output of `handcalc.py b2`)

## 1. Definitions

| Symbol | Meaning |
|---|---|
| element frame | origin on the ground below the front axle, x forward, y left, z up [m] (params.hpp `AeroElement`) |
| x_o | body-origin x = −cg_to_front (CG and sprung-body origin share x on every path) |
| dz | heave of the body origin relative to the static reference [m], + up |
| θ | pitch relative to the static reference [rad], ISO 8855, + nose down |
| h_i − h_ref,i | dz − (x_i − x_o)·θ |
| ClA_eff,i | ClA_i + dClA_dh,i·(h_i − h_ref,i) + dClA_dpitch,i·θ (no clamp) |
| q | ½·air_density·vx² |
| static reference L3/L4 | the ride states right after reset (z_s = θ = φ = 0) |
| static reference L5 | body-origin z, pitch, roll after the in-reset settle run (PO D1, 1st option) |

YAML keys per element: `h_ref` [m], `dClA_dh` [m²/m], `dClA_dpitch` [m²/rad]; all
three together or none.

### 1.1 L5 settle run (PO D1 conditions 1–7)
- Runs inside `reset()` / `reset_settled()` only when some element has
  dClA_dh ≠ 0 or dClA_dpitch ≠ 0, or an aero module is attached. Otherwise zero
  settle runs (B1).
- Procedure: snapshot the reset state; run with zero velocity and zero command
  until the 48b settle condition holds (all channels for 0.5 s continuously,
  limit 8 s); record body-origin z, pitch, roll as ref_z, ref_pitch, ref_roll;
  restore the snapshot.
- During the settle run the aero contribution is exactly 0 and the aero module
  is not called (air-relative speed is 0). Not consumed or recorded by the
  settle run: once-per-process warning flags, trace rows, sim time and tick
  counters, module `reset` / `begin_step` / `compute` calls. Re-entering
  `reset` from inside the settle run is refused (internal guard).
- Limit reached: one warning naming the unmet channels; the last state is the
  reference; `settled = false`.
- Trace manifest `aero` block gets `ref_z`, `ref_pitch`, `ref_roll`,
  `settled`, `t_settle` only when a settle run happened (no change otherwise).
- Known limitation (doc only): spawning on a slope mixes the ground slope into θ
  (about 12 mm per degree at x = 0.7 m).

## 2. Gates

| ID | Content | Gate |
|---|---|---|
| B1 | no slope keys / slope keys with all slopes 0 / no module → base main | bit identical |
| B1a | settle run on vs off | bit identical |
| B2 | ClA_eff and h_i on the fixed grid | relative error ≤ 1e-9 |
| B3'-L5 | undertray self-sag fixed point, L5 penalty and L5 strut | ±2 % |
| B3'-L3 | injected (dz, θ), first tick, L3 and L4 | ±2 % |
| B3b | front-wing pitch slope in steady braking, L3 per axle (L5 penalty / strut: information when not judgeable, 2.5) | ±2 % |
| B3b-L5i | injected pitch, first aero evaluation, L5 penalty and L5 strut, per axle (2.5a) | ±2 % |
| B4 | Python module re-implementing the built-in elements, L3 and L5 | state max diff ≤ 1e-9 relative |
| B5 | module exception / non-finite output | same handling and message form as the 23 chapter modules |
| B6 | rejection rules (table 2.6) | as tabled |
| B7 | negative builds, local canonical, doc audit | as listed |
| B8 | module together with built-in aero | doc + 1 test |

Validity guard (PO D2): any B3'/B3b expectation with |expected| < 10 N is "not
judgeable" and stops the work (no wider tolerance).

### 2.1 B1
- Tools: `~/q48c_keep/q48c/k1_bytes.cpp` (24 serialized cases L1–L5),
  `~/q34/rl_digest.py` (pedal, accel), `~/q34/digest.py` (closed-loop arrays),
  head tree vs a clean tree of the base main.
- Added case (head tree only, the base cannot parse the keys):
  `fs_aero_example.yaml` with `h_ref` set and both slopes 0 on every element
  vs the same file without the keys, L1–L5, 2000 ticks CmdL6 60 km/h, all
  outputs bit identical; on L5 the settle run count is 0.
- Re-measured against the main this branch merges before its PR.

### 2.2 B1a (settle run leaves nothing but the reference)
- Internal test hook forces the settle run with all slopes 0 and no module.
- L5 penalty and L5 strut, `fs_aero_example.yaml`, `reset_settled` at 60 km/h,
  then 2000 ticks CmdL6 60 km/h: the state right after reset and every
  `SimOutput` of the 2000 ticks equal the run without the settle run, bit for
  bit; sim time and tick counter equal.
- Same run with a counting module (C++): `reset` called once per reset,
  `begin_step` once per tick, `compute` 0 times during the settle run.
- Limit check: the hook's limit set to 0.01 s → exactly one warning, text
  contains the unmet channel names; `settled = false` in the manifest.

### 2.3 B2
- `compute_aero_loads` with `AeroState{vx = 20, dz, θ}` on
  `fs_aero_example.yaml` with the slopes of `handcalc.B2_SLOPES`; grid
  dz ∈ {−0.020, 0, 0.015} m × θ ∈ {−0.010, 0, 0.020} rad.
- Element downforce and h_i equal `b2_expected.json` within 1e-9 relative
  (absolute 1e-12 for zeros). C++ and Python counted separately.

### 2.4 B3'-L5 (PO D2: method changed before measurement, gate unchanged)
42's "force the static ride height (added mass)" cancels against the settled
reference, so the forcing is the car's own aero sag at speed.
- Car: `fs_aero_example.yaml`; on run adds to `undertray`
  `h_ref 0.040, dClA_dh s_p, dClA_dpitch 0.0` with s_p = −4.0 on L5 penalty and
  −2.0 on L5 strut (`handcalc.B3L5_DCLH`); off run has the same three keys
  with `dClA_dh 0.0` (no settle run, static reference taken from the on run:
  at zero speed aero is 0 so the settled state is the same; the test asserts
  the two spawn states are bit identical).
- Run: `reset_settled` at vx 35 m/s, CmdL6 v_target 35 m/s, 12 s, nominal dt
  1 ms, `stunt_physics`, max_substep_dt 2e-4, max_substeps 16, flat ground,
  `generic_pacejka.yaml`. Paths: L5 penalty and L5 strut.
- Window: last 2 s ([10, 12) s). Validity: |a_g| ≤ 0.02 m/s² (A3 definition)
  in both runs, else stop and report.
- Measured: ΔF_down = ⟨ΣFz⟩_on − ⟨ΣFz⟩_off (four tyre Fz).
- K_f, K_r: off run, rigid-split aero axle load / (−axle-point heave), window
  means; axle points Δz_f = dz − a·θ, Δz_r = dz + b·θ relative to the
  static reference.
- Expected: `handcalc.b3l5` fixed point
  Δh_u = Δh_u0 / (1 + D·(s²/K_f + (1−s)²/K_r)), D = q_on·dClA_dh,
  ΔF_exp = ΣClA·(q_on − q_off) + D·Δh_u. Gate |ΔF/ΔF_exp − 1| ≤ 0.02.
- Size check (info, `handcalc.py predict`): ΔF ≈ 25 N, Δh_u ≈ −17.7 mm.
- Re-fix of the penalty slope (PO mail 91b88183, after the first measurement,
  gate unchanged): with −2.0 the penalty expectation was 9.518 N (< 10 N, not
  judgeable; measured 9.516 N, −0.016 %, kept as information for the PR body).
  Rule: the smallest |slope| of {2, 3, 4, 5, 6} whose `handcalc.py b3l5_select`
  expectation from the measured slope-0 penalty run is ≥ 15 N. Output:
  −2.0 → 9.52 N, −3.0 → 14.31 N, −4.0 → 19.12 N → s_p = −4.0. If the
  re-measurement misses ±2 %, stop and report (no further re-fix).
- Info column: ΔF from the measured Δh_u (separates slope arithmetic from the
  stiffness model).

### 2.5 B3'-L3 and B3b
B3'-L3 (L3 and L4):
- Same car and undertray keys as 2.4. State at vx 35 m/s with z_s = −0.010 m
  and θ = +0.010 rad injected by snapshot restore; one tick with and without
  the slope. Measured ΣFz_on − ΣFz_off of that tick.
- Expected `handcalc.b3l3`: 713.5625·(−2.0)·(−0.0107) = 15.2702375 N. ±2 %.

B3b (L3, L5 penalty, L5 strut):
- Car `fs_aero_example.yaml`; on run adds to `front_wing`
  `h_ref 0.100, dClA_dh 0.0, dClA_dpitch 30.0`; off run without the keys.
- Run: `reset_settled` at vx 32 m/s, CmdL5 ax_target −4.0 from t = 0, window
  [1.0, 3.0) s, other settings as 2.4.
- Expected per axle (`handcalc.b3b`, per-tick rigid split with each run's own q
  and the on run's θ, drag moment included):
  ΔF_f = ⟨F_f,on⟩ − ⟨F_f,off⟩ − m·Δa_x·h/L, ΔF_r = ⟨F_r,on⟩ − ⟨F_r,off⟩ + m·Δa_x·h/L,
  Δa_x = ⟨ax_on⟩ − ⟨ax_off⟩ (SimOutput ax), h = cg_height (whole-vehicle CG, 47).
- Measured: ⟨Fz_FL + Fz_FR⟩_on − ⟨…⟩_off, same for rear. ±2 % per axle, guard
  applies to each axle. Size check: front ≈ +69 N, rear ≈ −18 N.
- On L3 θ comes from inertia only (aero does not drive the L3 ride states;
  known limitation, CHANGELOG line per PO D2).
- L5 penalty and L5 strut (PO mail 91b88183): the braking pitch on L5 is about
  1/10 of L3, so the expectation fell below 10 N per axle. The run stays in the
  canonical test; on L5 only, a not-judgeable case prints `INFO not judgeable`
  with the values and passes; a judgeable L5 case is judged as before. L3,
  B3'-L3 and B3'-L5 still fail when not judgeable. The gate on L5 is B3b-L5i.

### 2.5a B3b-L5i (injected pitch, L5)
- Car: on = `fs_aero_example.yaml` with `front_wing` `h_ref 0.100, dClA_dh 0.0,
  dClA_dpitch 30.0`; off = the same file without the keys. L5 penalty and L5
  strut, settings as 2.4.
- State: `reset_settled` at vx 25 m/s on the on car (settle run → reference);
  in its snapshot only the true state is replaced: body z = ref_z, orientation
  from (ref_roll, ref_pitch + θ_inj, yaw), body velocity (25, 0, 0) m/s, angular
  velocity 0, wheel spin 25/R (strut: unsprung points rotated with the body,
  their velocity = body velocity). θ_inj = +0.01 and −0.01 rad. The same
  snapshot is restored into both sessions, one tick of 1 ms.
- Measured: per-axle aero load (`AeroLoads::fz_front`, `fz_rear`) of the first
  aero evaluation after the restore, on − off. Read through a read-only internal
  observation (`detail::AeroObs`), which the model never reads.
- Validity: the observed static-relative pitch equals θ_inj, heave 0 and vx 25
  within 1e-9.
- Expected (`handcalc.b3l5i`): front = q·30·θ_inj·s_e, rear = q·30·θ_inj·(1−s_e),
  q = ½·ρ·25², s_e = (0.550 + L)/L: ±147.97 N / ∓38.76 N. ±2 % per axle, 10 N
  guard per axle (not judgeable → fail). Negative check: N3 (θ sign flipped)
  must fail B3b-L5i.

The C++ tests compute the same expectations and write the window means to a
JSON next to the test log; `handcalc.py b3l5|b3b` re-judges that JSON. A gate
passes only when both agree.

### 2.6 B4, B5, B6, B8
B4:
- Python `vdsim.AeroModule` subclass re-implementing the built-in elements
  including the slopes (undertray as 2.4 on run; front_wing as B3b on run).
- vs the built-in path on the same car, L3 and L5 penalty, `reset_settled` at
  60 km/h, CmdL6 60 km/h, 5 s. Gate: every State component
  |a − b| ≤ 1e-9·max(1, |b|) at every tick.

B5:
- Module raising, and module returning NaN / inf: same handling and message
  form as the existing user modules of docs/theory/23 (path checked in the
  implementation step; if no such path exists, stop and report).

B6 (C++ `std::invalid_argument`, Python `ValueError`):

| Input | Result |
|---|---|
| element with one or two of the three keys | rejected |
| non-finite h_ref / dClA_dh / dClA_dpitch | rejected |
| L1 or L2 with any slope ≠ 0 | rejected |
| L1 or L2 with the keys and both slopes 0 | accepted, bit identical (B1) |
| h_i < 0 during a run | one warning per SimSession, element named |
| module force with Fy ≠ 0 on L2–L4 | Fy ignored, one warning per process |
| `to_yaml` | writes the three keys only for elements that had them |

B8:
- Module attached → the module replaces the whole built-in aero (four keys,
  default drag, elements, slopes, aero_drag_height); air_density goes to the
  module as rho. One warning per process naming the replaced built-in values
  (Cd, frontal_area, aero_lift_*, element names).
- Test: car with elements + module returning no forces vs the same car with no
  aero at all: warning count 1, outputs within 1e-12 relative for 2000 ticks.
- "No aero at all" (PO clarification, before implementation): the comparison
  car sets `aero_drag_coeff`, `frontal_area`, `aero_lift_front` and
  `aero_lift_rear` explicitly to 0 and has no `aero.elements`. Omitting the
  keys is not allowed (the default drag coefficient would stay on the
  comparison car only and B8 would fail falsely).
- The module car runs the D1 settle run and the comparison car does not. Both
  compare from right after `reset()`; that difference is the range covered by
  B1a. If B8 exceeds 1e-12, report it together with the B1a result.

### 2.7 B7
Each negative build is made in a scratch copy, run once, then reverted:
- N1 slope term removed (ClA_eff = ClA) → B2 and B3'-L5 fail.
- N2 static reference replaced by world absolute z / pitch → B3'-L5 or B3b fails.
- N3 θ sign flipped → B3'-L5 or B3b fails.
- Restored tree passes all three. Then local canonical run (registered-name
  diff vs main, C++ and Python counted separately) and the doc audit script on
  head and main: FAIL count not increased.

## 3. Information items (no gate)
- Wall-clock cost of the L5 settle run per reset, median of 3 (PR body).
- B3'-L5 ΔF from measured Δh_u, B3b θ window mean.
