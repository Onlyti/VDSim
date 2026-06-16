# L1 per-tire + L5 phase-A (6-DOF inverted tire) — verification evidence

Verifies the two behaviour-changing commits:
- `0329847` L1 bicycle on the inverted interface (per-tire individual contact).
- `38b5d8c` L5 free_3d on the inverted interface (contact-frame slip + LuGre bristle).

Python cannot drive L5 (contact `penetration` and the loop/ramp ground providers are not
bound), so verification is by the C++ gated tests. `ctest` selection below: **100/100 pass**;
full suite **364/364**.

```
ctest -R "BicycleSteadyState|NoPhantom|EffectiveRolling|LuGreTire|Stunt|L5|Terrain|Belt|\
IsoBaseline|ChronoPac02|TireInversion|CamberMigration|SevenDOF"  ->  100/100 passed
```

## L1 bicycle (per-tire individual contact, ×2 for the axle)

| Test | Asserted property | Threshold | Result |
|---|---|---|---|
| `BicycleSteadyState` | steady yaw rate vs linear-bicycle analytic | within band | pass |
| `NoPhantomForce/L1` (reff) | free-roll first step: slip + no phantom Fx | \|κ\|<5e-4, \|ΣFx\|<60 N | pass |
| `LuGreTire.HighSpeedLongitudinalNearPacejka` (L1) | LuGre Fx vs Pacejka over 1200-step throttle | within 35% | pass |

Byte-stability: load_sensitivity defaults to 0, so `2·F(0.5·Fz) == F(Fz)` and the MF path is
unchanged; the per-tire change only alters behaviour for `load_sensitivity≠0` (then more
correct — μ applied at the true per-tire load). LuGre slip velocity now uses Re consistently.

## L5 free_3d (phase A: inverted tire + contact-frame slip)

| Test | Asserted property | Threshold | Result |
|---|---|---|---|
| `Stunt.FreeLoopCompletesLap` | car rotates through a vertical loop | θ_peak>1.0 rad, \|pitch\|>0.28 | pass |
| `Stunt.LoopSlipAngleFrontRearBalanced` | front slip excited through loop | n>80 samples, peak α_f>0.02 | pass |
| `Stunt.JumpAirborneInterval` | leaves ground off a ramp | airborne=true, z_peak>cg_h+0.2 | pass |
| `Stunt.JumpLandingNoSink` | lands without sinking through road | z_min(late)>0.38 m | pass |
| `Stunt.L5CoastOnFlatNoSink` | rests at ride height on flat | \|z−z0\|<0.04 m | pass |
| `Stunt.GradeL2CoastSlowsOnUphill` | uphill coast decelerates | vx < 0.95·vx_flat | pass |
| `L5Driving.*` (8), `Terrain.*` (7), `BeltTransient.L5PathLagsWithBelt` | attitude / load / climb / bank / belt lag | per test | pass |

Key results of phase A:
- The loop-specific slip-denominator floor was removed; the contact-frame `v_long_k`
  (hub velocity onto the wheel-heading tangent, track-tangent fallback near a loop top)
  alone keeps `FreeLoopCompletesLap` / `LoopSlipAngleFrontRearBalanced` passing — i.e. the
  floor was **not load-bearing**, confirming the contact-frame slip is the right primitive.
- Non-loop paths (jump / flat / terrain / grade) are byte-stable (slip == standard for
  surface_id≠2), so those tests are unchanged.
- Latent bug fixed: the LuGre bristle `z` is now integrated per stage (was stuck at 0);
  `L5Driving.LuGreFlatDrivingSmoke` + `BeltTransient.L5PathLagsWithBelt` exercise it green.

## Not yet validated (phase B/C, #222–#224)

L5 still uses penalty-at-hub contact with no suspension; it remains labelled experimental in
VALIDATION.md. Promotion needs the spatial-strut build (B1) + corner DAE (B2) + the phase-C
evidence (ballistic-jump parabola/energy, loop critical speed, flat cross-model vs L2/L3,
suspension travel vs L4). See `docs/design/L5_6DOF_MULTIBODY.md`.
