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
| L1–L3 dynamics vs analytic (linear-bicycle yaw, drag coast, weight transfer) | Cross-validation vs CarMaker/CarSim/Adams on real-vehicle data (data confidential, not redistributable) |
| ISO 7401 step-steer / 4138 understeer / 3888-2 DLC — run + measured | Tire thermal, wear, full transient beyond first-order relaxation |
| L1↔L2↔L3 cross-model consistency where physics overlaps | Dependent axles (twist-beam / solid beam) — configs are stubs |
| FMI round-trip Δ=0 (machine precision); ISO 8608 PSD RMS per class | L3 unsprung lateral-transfer term (small) |
| Full suite: **187/187 ctest green** | — |

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
| 8 | ISO 7401 step-steer | transient shape | yaw SS 30.1°/s, peak 36.4 (20.8% OS), t_settle 2.4 s, a_y 0.81 g | — | `python3 apps/validation/run_validation.py` |
| 9 | ISO 4138 understeer | sign of K | K = +9.44 mrad/g (understeer), linear ≤5.66 m/s² | — | same runner |
| 10 | ISO 3888-2 DLC | excursion/speed-loss metric | 1.69 m / 1.34 km/h (sedan: does not meet 1.0 m) | — | same runner |
| 11 | FMI round-trip | native VDSim | max \|Δvx\| = 0 (machine precision) | 1e-9 | `python3 fmi_export/test_roundtrip.py` |
| 12 | ISO 8608 roughness | PSD Gd(n)=Gd(n0)(n/n0)⁻² | RMS doubles/class: A 3.5, B 7.0, C 14.1, D 28 mm | 15% | `ctest -R Iso8608` |

Full automated suite: `cd build && ctest` — 187 checks, 100% green (measured 2026-06-04).

## Notes on specific results

- **#10 moose test "does not pass"** is a vehicle/controller property, not a sim
  defect: the reference passive sedan with the default pure-pursuit driver runs a
  1.69 m excursion at 60 km/h (criterion 1.0 m). The metric is computed per ISO
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
- **L1/L2 are planar** — no body-attitude state from road slope (only force +
  quasi-static estimate); see theory ch05.
- **Dependent axles** (twist-beam, solid beam) are not yet modeled (configs are
  stubs); only the four independent topologies are validated.
- **L3 grip Fz** couples ride/road dynamically but omits the unsprung lateral-
  transfer term (small); see ch06 §6.4.

## Reproducing the whole report

```sh
cmake --build build -j && (cd build && ctest --output-on-failure)   # 185 checks
python3 apps/validation/run_validation.py    # ISO 7401/4138/3888 -> REPORT.md
python3 fmi_export/test_roundtrip.py          # FMU vs native
```
