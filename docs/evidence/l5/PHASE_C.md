# L5 spatial-strut — phase C validation evidence

Gating evidence for the L5 6-DOF spatial-strut model (phases B1/B2). All four
checks run the strut path (`SolverParams::l5_spatial_suspension = true`) against an
independent reference. Source: `tests/integration/test_l5_strut_validation.cpp`
(suite `L5StrutValidation.*`, default vehicle params, MF96 tire, aero off).

Reproduce:

```sh
cmake --build build -j
./build/bin/vdsim_integration_tests --gtest_filter='L5StrutValidation.*'
```

## Results (default params: m=1500, m_sprung=1350, k_s=30000 N/m/corner, k_t=220000 N/m)

| Check | Metric | L5 strut | Reference | Rel. err | Tol | Pass |
|---|---|---|---|---|---|---|
| C1 flat cross-model | yaw rate [rad/s], 0.03 rad steer @ 15 m/s | 0.15282 | 0.15574 (L2 seven_dof) | 1.9% | 20% | yes |
| C1 flat cross-model | lateral specific force ay [m/s^2] | 2.4006 | 2.3167 (L2) | 3.6% | 20%+0.3 | yes |
| C2 ballistic jump | fit g over airborne arc [m/s^2] | 9.7532 | 9.80665 (analytic) | 0.5% | 5% | yes |
| C2 ballistic jump | horizontal speed drift [-] | 0.0051 | 0 (no aero) | 0.5% | 3% | yes |
| C3 vertical loop | climbed arc @ 1.15 v_crit [rad] | 20.03 | — | — | >1.0 | yes |
| C3 vertical loop | climbed arc @ 0.70 v_crit [rad] | 2.68 | — | — | fast>slow+0.5 | yes |
| C4 corner camber | FL camber @ travel +5.9 mm [rad] | -0.003595 | -0.003470 (standalone L4 DAE) | 1.3e-4 abs | 2e-3 | yes |
| C4 corner camber | RL camber @ travel -5.9 mm [rad] | +0.002293 | +0.002315 (standalone L4 DAE) | 2.2e-5 abs | 2e-3 | yes |

## Interpretation

- **C1** — On flat ground at low excitation the 6-DOF spatial model reduces to the
  planar L2 handling: yaw rate within 1.9%, lateral specific force within 3.6%. The
  in-plane response uses the total mass (the unsprung is rigidly carried), and the
  rotational inertia uses the same `inertia_diag` as L2/L3, so the spatial model is a
  consistent extension rather than a different vehicle. (`ax/ay_body_est` report the
  body-frame specific force, gravity removed — the L2/L3 convention — not the body-
  velocity derivative, which is ~0 in a steady turn.)
- **C2** — Off a ramp the airborne CG follows projectile motion: a least-squares
  quadratic fit of z(t) over the mid-flight window recovers g = 9.75 m/s^2 (0.5% of
  9.80665) and the horizontal speed holds to 0.5% (no aero). Gravity and the free-
  flight condition are emergent, not imposed.
- **C3** — The centripetal condition is emergent: at 1.15 v_crit the car climbs ~20
  rad around the vertical loop (multiple revolutions with throttle), at 0.70 v_crit it
  stalls near the bottom (~2.7 rad). v_crit = sqrt(5 g R), R = 10 m.
- **C4** — Under a brake-dive jounce the front struts compress (+5.9 mm) and the rear
  extend (-5.9 mm); the per-wheel camber applied to the tire matches the standalone L4
  hard-joint corner DAE evaluated at the same travel to < 1.3e-4 rad. The strut path
  delegates toe/camber to the identical L4 DAE, here verified end-to-end through the
  6-DOF body.

All four pass, so the L5 spatial-strut path is promoted from "plausible-but-not-
validated" to a validated model on these axes. The legacy penalty path (default,
`l5_spatial_suspension = false`) is unchanged and remains the stunt demo plant.
