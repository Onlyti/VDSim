# [260901]_[Validation]_[D0FlatContactBaseline]

## Purpose

Freeze the current L3 flat-road response before D1 contact-frame work.

- Contact contract: every wheel has `normal=(0,0,1)`, `road_dz=0`, and `is_valid=true`.
- Scope: regression evidence only; this gate changes no core behavior and does not implement D1.
- Failure rule: any value outside the channel-specific tolerance blocks D1 work until reviewed.

## Representative scenario

- Plant: `create_fourteen_dof()` with default vehicle and tire parameters.
- Initial speed: 15 m/s; wheel angular speed initialized to pure rolling.
- Duration: 2.0 s at 0.005 s per step.
- Commands:
  - 0.0--0.5 s: throttle 0.20, steer 0.
  - 0.5--1.5 s: throttle 0.15, steer 0.06 rad.
  - 1.5--2.0 s: brake 0.25, steer 0.02 rad.
- Checkpoints: steps 0, 1, 20, 100, 200, 300, and 400.
- Raw channels: pose, body velocity, angular velocity, wheel spin, suspension travel/rate, rack state, per-wheel body force, Fz, slip, friction, roll, pitch, acceleration, and rack torque.

## Frozen baseline

- Fixture: `tests/fixtures/d0_flat_contact_l3_baseline.csv`.
- Encoding: decimal IEEE-754 doubles written with 17 significant digits for exact binary round-trip.
- SHA-256: `bc81114b12f57fcad6d53ba997765e06853d505d7eaad32ff33211ff9d6cef53`.
- Rows: 7 checkpoints plus one header row.

## Equivalence tolerance

- State/output absolute tolerance: `1e-12` in each channel's SI unit.
- Force absolute tolerance: `1e-9 N`.
- Rationale:
  - The captured baseline and an immediate repeat are bit-identical, so the expected difference is zero.
  - The 17-digit CSV round-trip is also exact for the captured values.
  - `1e-12` preserves an identity-level state/output gate while allowing harmless compiler operation-order noise above machine epsilon.
  - `1e-9 N` is approximately `2e-13` relative to a representative kilonewton wheel load and permits only floating-point algebra noise, not a physical force change.

## Measured result (2026-09-01)

- Repeat-run bit-identical: `true`.
- Repeat-run maximum absolute difference: `0`.
- Fixture state/output maximum absolute difference: `0`.
- Fixture force maximum absolute difference: `0 N`.
- Full suite with Node v20.20.2: `467/467` passed in 114.34 s.

## Reproduction

Run the committed gate normally:

```bash
cmake -S . -B build
cmake --build build --target vdsim_integration_tests
ctest --test-dir build -R D0FlatContactBaseline --output-on-failure
```

Baseline regeneration is intentionally explicit and requires physics review before committing:

```bash
VDSIM_D0_CAPTURE=/tmp/d0_flat_contact_l3_baseline.csv \
  build/bin/vdsim_integration_tests \
  --gtest_filter=D0FlatContactBaseline.RepresentativeScenarioMatchesFrozenFixture
```
