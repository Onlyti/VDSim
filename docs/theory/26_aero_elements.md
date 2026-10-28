# Ch.26 — Aero elements, air density and drag height

From 0.8.0 a vehicle file can describe its aerodynamics as a list of named
**elements** (front wing, rear wing, undertray, body …), each with its own
coefficients and point of application. The older four keys
(`frontal_area`, `aero_drag_coeff`, `aero_lift_front`, `aero_lift_rear`) keep
working; the two forms cannot be mixed.

## 26.1 Inputs

Axes follow ISO 8855 (x forward, y left, z up). The origin is **on the ground
below the front axle**, so rear elements have negative `x`. Lengths in m.

```yaml
air_density: 1.165          # [kg/m^3] optional, default 1.225, used by every aero path
aero_drag_height: 0.30      # [m] optional, four-key path only (see 26.3)
aero:
  elements:
    - name: front_wing      # [a-z0-9_], unique
      ClA: 0.70             # [m^2] + = downforce
      CdA: 0.12             # [m^2] >= 0
      x: 0.550              # [m] centre of pressure, + forward of the front axle
      z: 0.100              # [m] >= 0, height above ground
      y: 0.0                # [m] optional, roll moment (L3-L5)
```

Rejected with `ValueError`: `aero.elements` together with any of the four keys
or `aero_drag_height`; empty, invalid or duplicate names; `CdA < 0`; `z < 0`;
non-finite values; `air_density <= 0`. On L1/L2 an element `y` is ignored with
one warning per process. A worked example is
`configs/vehicles/fs_aero_example.yaml` (illustrative values, not a real car).

## 26.2 Forces

With $q = \tfrac12 \rho\, v_x |v_x|$ (sign of travel kept):

$$
F_{\text{down},i} = \text{ClA}_i\, |q|, \qquad
F_{\text{drag},i} = \text{CdA}_i\, q .
$$

Downforce scales with $v_x^2$ and never turns into lift when reversing; drag
opposes the direction of travel. `vdsim.compute_aero_loads(vp,
vdsim.AeroState(vx))` returns the per-element forces, totals, the axle split
and the body wrench, and is the function the acceptance checks use.

**Axle split (rigid axles).** With wheelbase $L$ the front share of element
$i$'s downforce is $(x_i + L)/L$. The drag pitch moment
$\sum F_{\text{drag},i} z_i$ moves load from the front to the rear axle by
$\sum F_{\text{drag},i} z_i / L$. The reported aero balance
(`aero_balance_front`) is the front axle increase over the total downforce,
drag moment included.

## 26.3 What each level applies

| Level | Downforce | Drag height | Element `y` (roll) |
|---|---|---|---|
| L1 bicycle | axle split, $(x_i+L)/L$ | drag moment $\sum F_d z / L$ on the axles | ignored (warning) |
| L2 7-DOF | axle split, halves per axle | as L1 | ignored (warning) |
| L3 14-DOF | axle split, per wheel; added to tyre loads only | as L1 | split by each axle's own track |
| L4 | as L3 | as L1 | as L3 |
| L5 free 3D | body wrench at $(x_i, y_i, z_i)$ | force at $z_i$ in the body wrench | in the wrench |

Four-key path: `aero_lift_front` / `aero_lift_rear` act at the front and rear
axle (ground height) on every level, including L5 since 0.8.0. Drag acts at
`aero_drag_height` if set; if unset it acts at the ground on L1–L4 and at the
CG on L5 (the 0.7.0 behaviour, bit-identical).

## 26.4 Logging

`enable_trace(extra_channels=("aero",))` adds `aero_<name>_fz`,
`aero_<name>_fx` [N], `aero_fz_total` [N], `aero_balance_front` [-] and
`aero_ld` [-] (NaN at zero speed). The channels are optional; schema 0.5 is
unchanged and a trace without them is bit-identical.

## 26.5 Limits

- **Check aero balance numbers on L1–L3.** L5 axle loads include the L5 limit
  below: L5 applies tyre longitudinal forces at the wheel centre and does not
  return the drive/brake torque reactions to the body, so longitudinal load
  transfer uses $(h_{cg} - R)$ instead of $h_{cg}$; with aero drag the front
  axle share is biased by $\sum F_d R / L$ ($R$ = wheel radius). The aero
  wrench itself is applied correctly (equal-drag and drag-only checks below).
- L3/L4 downforce is added to the tyre loads only; it does not press the
  suspension (no body heave or pitch from aero).
- The L3 output `Fz` is the internal quasi-static 7-DOF wheel load.

## 26.6 Verification

`ctest -R "aero"` runs the unit checks (`test_aero.cpp`: totals, centre of
pressure, axle share with the drag moment, density scaling, rejection rules,
reversing) and the steady-state checks (`test_aero_elements.cpp`: axle-load
increase at a held speed on L1–L5, L4 = L3, L5 total reaction, four-key path
vs elements, four-key lift on L5 vs L3). On L5 the axle split is checked
against a reference run with the same drag (`A3_L5a`), and the wheel-centre
term above is checked against a drag-only run (`A3_L5b`).
