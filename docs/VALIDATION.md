# VDSim validation & credibility

> **v0.1.0 experimental / pre-release.** The claims below rest on open evidence
> (analytic + ISO standard + cross-model/cross-tool self-consistency). There is
> no published cross-validation against a commercial reference on real-vehicle
> data — see "Honest limitations". Not for production use.

What "validated dynamics" means here, and how to reproduce every claim. VDSim is
validated on four layers that need no proprietary data; the honest limits are in
the last section.

## At a glance — validated / not yet

| Validated (open, reproducible) | NOT yet validated |
|---|---|
| L1–L3 dynamics vs analytic (linear-bicycle yaw, drag coast, weight transfer) | Full-vehicle cross-validation vs CarMaker/CarSim/Adams on real-vehicle data (data confidential, not redistributable) |
| **Tire force vs CarMaker MF-Tyre/MF-Swift** (same MF6.2 `.tir`, steady-state pure slip): pure-long 0.00%, pure-lat 0.09% | Full-vehicle (suspension/transient) commercial cross-val |
| ISO 7401 step-steer / 4138 understeer / 3888-2 DLC — run + measured | Tire thermal, wear, full transient beyond first-order relaxation |
| L1↔L2↔L3 cross-model consistency where physics overlaps | Dependent axles (twist-beam / solid beam) — configs are stubs |
| FMI round-trip Δ=0 (machine precision); ISO 8608 PSD RMS per class | L3 unsprung lateral-transfer term (small) |
| Full suite: **345/345 ctest green** | — |
| Drivetrain engine inertia (open-diff carrier coupling) | — |
| **ISO step-steer signature gated in CI** (`ctest -R IsoBaseline`, sedan L2 LuGre) | DLC moose gate is a preset property, not a defect (see note) |

Note: ISO 3888-2 DLC@60 not meeting the 1.0 m gate is a default-preset
vehicle/controller property, not a sim defect (see "Notes on specific results").

## Validation layers

1. **Analytic** — closed-form results the model must reproduce (linear bicycle
   yaw rate, drag-only coast decay, static/Newtonian weight transfer).
2. **ISO standard maneuvers** — ISO 7401 step-steer, ISO 4138 understeer
   gradient, ISO 3888-2 double-lane-change, run by `apps/validation`.
3. **Cross-model consistency** — the ladder (L1↔L2↔L3) must agree where the
   physics overlaps (steady-state transfer, planar motion).
4. **Cross-tool** — the exported FMU must reproduce the native run, and the
   road roughness must match the ISO 8608 spatial PSD.

## Benchmark matrix

| # | Property | Reference | Result | Tol | Reproduce |
|---|---|---|---|---|---|
| 1 | L1 steady yaw rate | linear bicycle model | within 10% (L,R turn) | 10% | `ctest -R BicycleSteadyState` |
| 2 | Zero-steer tracking | r=vy=Y=0 | <1e-6 / <1e-3 | — | `ctest -R ZeroSteerStraightLine` |
| 3 | Drag-only coast | v(t)=v0/(1+v0 k t) | within 5% | 5% | `ctest -R DragCoastMatchesAnalytical` |
| 4 | Brake decel | tire-limited | a_avg ≥ 2 m/s² , monotonic | — | `ctest -R BrakeStep` |
| 5 | Lateral weight transfer | m·a_y·h/T direction+sign | outer wheels gain | — | `ctest -R WeightTransfer` |
| 6 | L3 ↔ L2 planar | identical grip at steady state | divergence ~3e-6 | 1e-3 | `ctest -R PlanarMotionMatchesL2` |
| 7 | L3 attitude on slope | follows surface plane | 6° bank→5.98° roll | — | see ch06 §6.4 |
| 8 | ISO 7401 step-steer | transient shape | ψ̇_ss 30.3°/s, U 1.16 (+16% OS), a_y_ss 8.34 m/s² (0.85 g) | gated ±band | `ctest -R IsoBaseline` / `run_validation.py` |
| 9 | ISO 4138 understeer | sign + magnitude of K | K = +24.8 mrad/g (understeer), linear ≤4.0 m/s² | — | `run_validation.py` |
| 10 | ISO 3888-2 DLC | excursion/speed-loss metric | 1.18 m / 0.9 km/h (sedan: does not meet 1.0 m) | — | same runner |
| 11 | FMI round-trip | native VDSim | max \|Δvx\| = 0 (machine precision) | 1e-9 | `python3 fmi_export/test_roundtrip.py` |
| 12 | ISO 8608 roughness | PSD Gd(n)=Gd(n0)(n/n0)⁻² | RMS doubles/class: A 3.5, B 7.0, C 14.1, D 28 mm | 15% | `ctest -R Iso8608` |
| 13 | **MF2002 vs Chrono Pac02** (BSD-3, independent) | same public `.tir` | pure-long Fx ~0.8% + pure-lat Fy ~0.7% (Fz 2–6 kN, mean); combined cross-terms differ (rig-frame, reported) | 6% | `ctest -R ChronoPac02Parity` · `tools/tire_validation.py` |
| 14 | Control ladder (Lc1–Lc8 + split) | each level reaches target band | cruise/ax/yaw-rate/curvature + EPS torque verified | gated | `examples/control_ladder_demo.py` (ctest `control_ladder`) |
| 15 | **MF vs CarMaker MF-Tyre/MF-Swift** (commercial, independent) | same MF6.2 `.tir`, same Re-based slip | pure-long Fx **0.00%**, pure-lat Fy **0.09%** (machine precision, steady-state pure slip) | — | `external/carmaker_parity/compare_vdsim_carmaker.py` (needs a CarMaker license) |
| 16 | **Effective rolling radius (Re) consistency** | reff-enabled tire, free-roll init via `free_roll_wheel_spin` | first-step \|κ\|<5e-4, no phantom Fx (L1/L2/L3) | gated | `ctest -R "EffectiveRollingRadius\|NoPhantom"` |
| 17 | **Camber contact migration -> overturning** | crown_radius-enabled tire, camber input | Mx = Fz·crown_radius·sin γ per wheel; feeds L3 roll DOF; crown=0 -> Mx=0 | gated | `ctest -R CamberMigration` |

Evidence bundle for #16/#17 + the inverted interface (Re(Fz) curve, phantom-force
removal table, camber Mx vs analytic, canary results): `docs/evidence/tire/EVIDENCE.md`
+ figures, regenerated by `python3 tools/tire_inversion_evidence.py`.

Slip kinematics, Re and camber contact-point migration now live in one shared module
(`tire_contact.hpp::tire_contact_kinematics`) that all four dynamics models call, instead of
being inlined per model. A cambered tire contacts on the leaning side
(`contact_dy = crown_radius·sin γ`), shifting the vertical-load line and producing an
overturning moment `Mx = Fz·contact_dy` that feeds the roll DOF (L3 14-DOF). Both Re and
camber migration are opt-in (reff_*=0 / crown_radius=0 -> legacy behaviour), so all baselines
are unchanged.

Slip now uses the load-dependent effective rolling radius Re(Fz) (Pacejka BREFF/DREFF/FREFF)
across L1/L2/L3/L5 — slip = (ω·Re − vx)/vx, so a free-rolling loaded tire reports κ=0,
matching MF-Tyre/CarMaker. Initial wheel spin is set per-wheel via `free_roll_wheel_spin`
(vx/Re at static load) so no phantom longitudinal force appears at t=0. Re is opt-in: tires
without BREFF/DREFF/FREFF (reff_*=0) fall back to the unloaded radius R0 unchanged.

Full automated suite: `cd build && ctest` — 354 checks, 100% green (measured 2026-06-16).
One-command evidence bundle (ctests + tire parity table + ISO figures + summary.md):
`python3 tools/validation_report.py`.

**LuGre baseline (re-baselined 2026-06-10, sedan L2, `default_pacejka` tire):**

```bash
python3 apps/validation/run_validation.py \
  --vehicle configs/parts/chassis/sedan.yaml \
  --tire configs/parts/tire/default_pacejka.yaml \
  --level L2 --out apps/validation/results/lugre_sedan_l2
```

| Maneuver | Key metrics |
|----------|-------------|
| ISO 7401 (6°, 80 km/h) | ψ̇_ss 30.3°/s, U 1.16 (+16% OS), a_y_ss 8.34 m/s² (0.85 g) |
| ISO 4138 | K +24.8 mrad/g (understeer), linear a_y to 4 m/s² |
| ISO 3888-2 DLC@60 | excursion 1.18 m (fail 1.0 m gate), speed loss −0.9 km/h |

This is the shipped default preset (LuGre brush on `default_pacejka`). The four
force-sensitive 7401 metrics are locked in CI by `ctest -R IsoBaseline`
(`tests/integration/test_iso_baseline.cpp`) so any future change that moves tire/
dynamics forces fails the build; rebaseline that test together with this table.
Legacy kinematic-tire numbers reproduce with `tire.kinematic_fallback`.

**Drivetrain inertia:** `engine_rotational_inertia` slows wheel spin-up on low-μ
wheels (fixes power-on-turn overspeed). Its effect on the steady ISO numbers is
folded into the re-baselined table above.

## Notes on specific results

- **#10 moose test "does not pass"** is a vehicle/controller property, not a sim
  defect: the reference passive sedan with the default pure-pursuit driver runs a
  1.18 m excursion at 60 km/h (criterion 1.0 m). The metric is computed per ISO
  3888-2; passing requires a stiffer setup or a tracking controller — the point
  is the maneuver is exercised and measured correctly.
- **#9 understeer sign** matters more than the absolute value: a front-heavy
  sedan must be understeer (K>0), which it is. The magnitude depends on the tire
  cornering stiffness in the config.

## Honest limitations (what is NOT validated)

- **No published cross-validation against a commercial reference (CarMaker /
  CarSim / Adams) on real-vehicle data.** Such comparisons were run internally,
  but the measured tire/vehicle data are confidential and cannot be redistributed,
  so they are not part of this open benchmark. The open claims above rest on
  analytic/standard/self-consistency evidence only.
- **Tire model** is fitted Pacejka MF (and a linear fallback); no thermal,
  transient-relaxation beyond the first-order lag, or combined wear effects.
- **Standstill / low speed**: a kinematic-dynamic blend below 3 m/s governs the
  lateral states with the slip-free kinematic bicycle (no spin/oscillation at
  parking speed), and a viscous brake-hold damper holds the car on a grade. The
  hold is not a true static hold — it leaves a small creep proportional to grade
  (~1-2 cm/s on 8%); a zero-creep handbrake/locked-wheel static model is a v0.2
  item. See `docs/design/LOW_SPEED_HANDLING.md`.
- **L1/L2 are planar** — no body-attitude state from road slope (only force +
  quasi-static estimate); see theory ch05.
- **Dependent axles** (twist-beam, solid beam) are not yet modeled (configs are
  stubs); only the four independent topologies are validated.
- **Drivetrain (default off for legacy):** `engine_rotational_inertia` (default
  `0.25` kg·m² at crank) is reflected to the wheels and open diffs are
  carrier-coupled. Set `0` to recover pre–v0.3 wheel-only spin-up. The matrix and
  baseline table above were re-baselined 2026-06-10 with this change active.
  See `docs/design/V0.2_DRIVETRAIN.md`.
- **Tire default:** LuGre brush (`lugre.enabled: true` on `tire.default_pacejka`).
  **ISO runner default** uses `tire.kinematic_fallback` to preserve the published
  matrix; pass `--tire configs/parts/tire/default_pacejka.yaml` to validate under LuGre.
  Kinematic blend path: `LOW_SPEED_HANDLING.md`.
- **L3 grip Fz** couples ride/road dynamically but omits the unsprung lateral-
  transfer term (small); see ch06 §6.4.
- **Stunt / vertical loop (v0.4) — default penalty path:** the dynamic Free3D (Ld5)
  plant holds the car on the track by the contact normal force (penalty from
  penetration); the centripetal condition is *emergent* and it loses the track if too
  slow. The default path (`l5_spatial_suspension = false`) is robust-tuned for stunts,
  not validated road contact: the loop surface uses stiffened/capped contact (higher Fz
  cap, capped penetration, track-tangent longitudinal), so it is a plausible-but-not-
  quantitatively-validated demo. See `docs/design/V0.4_SLOPE_JUMP_DYNAMICS.md`.
- **L5 spatial-strut path (opt-in, `l5_spatial_suspension = true`) — validated.** The
  6-DOF body carries per-corner sprung/unsprung strut dynamics with the inverted tire
  and (optionally) the L4 corner DAE for toe/camber. Phase-C evidence
  (`docs/evidence/l5/PHASE_C.md`, suite `L5StrutValidation.*`): flat cross-model vs L2
  (yaw rate 1.9%, lateral g 3.6%), ballistic jump (fit g within 0.5%, speed drift 0.5%),
  emergent loop critical speed, and per-corner camber matching the standalone L4 DAE to
  < 1.3e-4 rad. Quarter-car heave 1.33 vs 1.41 Hz analytic (`L5Strut.*`). See
  `docs/design/L5_6DOF_MULTIBODY.md`.

## Reproducing the whole report

```sh
cmake --build build -j && (cd build && ctest --output-on-failure)   # 345 checks
python3 apps/validation/run_validation.py    # ISO 7401/4138/3888 -> REPORT.md
python3 fmi_export/test_roundtrip.py          # FMU vs native
```
