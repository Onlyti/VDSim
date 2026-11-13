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

- **Check aero balance numbers on L1–L3.** Up to 0.8.0, L5 axle loads carried
  two core defects: tyre longitudinal forces acted at the wheel centre with no
  drive/brake torque reaction on the body, and with a LuGre tyre the wheel spin
  equation had no tyre reaction torque. With aero drag this biased the front
  axle share (0.8.0 drag-only check: $\sum F_d R / L$, $R$ = wheel radius).
  Both are fixed in 0.8.1 (see the changelog). The aero wrench itself was
  applied correctly (equal-drag and drag-only checks below).
- L3/L4 downforce is added to the tyre loads only; it does not press the
  suspension (no body heave or pitch from aero).
- The L3 output `Fz` is the internal quasi-static 7-DOF wheel load.
- Ride slopes and the aero module (26.7, 26.8): the aero trace channels are
  evaluated at the sample speed without ride states; a spawn on a slope mixes
  the ground slope into the L5 pitch reference.

## 26.6 Verification

`ctest -R "aero"` runs the unit checks (`test_aero.cpp`: totals, centre of
pressure, axle share with the drag moment, density scaling, rejection rules,
reversing) and the steady-state checks (`test_aero_elements.cpp`: axle-load
increase at a held speed on L1–L5, L4 = L3, L5 total reaction, four-key path
vs elements, four-key lift on L5 vs L3). On L5 the axle split is checked
against a reference run with the same drag (`A3_L5a`), and the wheel-centre
term above is checked against a drag-only run (`A3_L5b`).
Ride slopes and the module (work order 51): `ctest -R "AeroRide51|aero_ride_51"`
(gates and expectations in `tests/fixtures/aero_ride_51/`).

## 26.7 Ride-height and pitch slopes (0.9.0)

An element may carry three more keys, all together or none:

```yaml
    - name: undertray
      ClA: 0.80
      CdA: 0.05
      x: -0.750
      z: 0.060
      h_ref: 0.040          # [m] element height at the static reference
      dClA_dh: -2.0         # [m^2/m] ClA change per metre of element height
      dClA_dpitch: 0.0      # [m^2/rad] ClA change per radian of pitch (ISO, + nose down)
```

$$
h_i - h_{\text{ref},i} = \Delta z - (x_i - x_o)\,\theta, \qquad
\text{ClA}_{\text{eff},i} = \text{ClA}_i + \frac{\partial \text{ClA}_i}{\partial h}(h_i - h_{\text{ref},i})
  + \frac{\partial \text{ClA}_i}{\partial \theta}\,\theta
$$

with $x_o = -a$ (`cg_to_front`, the body origin in the element frame), $\Delta z$
the body-origin heave [m, + up] and $\theta$ the pitch [rad, + nose down], both
relative to the **static reference**:

| Level | Static reference | $\Delta z$, $\theta$ |
|---|---|---|
| L1, L2 | none (no ride states) | a non-zero slope is rejected |
| L3, L4 | the reset equilibrium | the ride states $z_s$, $\theta$ |
| L5 | settle run inside `reset()` | body z and pitch minus the settled values |

The L5 settle run starts from the reset pose with zero speed and command, steps
at 1 ms until the 0.8.2 settle condition (speed and angular rate ≤ 1e-3, wheel
travel rate ≤ 1e-3 m/s, toe rate ≤ 0.01 °/s, load rate ≤ 1 N/s, loads > 0) holds
for 0.5 s or 8 s pass, records the body-origin z, pitch and roll and restores the
model exactly. Aero is off during the run. It runs only when a slope is non-zero
or an aero module is attached. `AeroElementLoad.h` reports $h_i$; an element
with slope keys below the ground warns once per session (the slope is linear,
no clamp).

## 26.8 Aero module (0.9.0)

A user module replaces the whole built-in aero (see ch. 23 for the call rules):

```python
class MyAero(vdsim.AeroModule):
    def compute(self, ctx, inp):           # -> list[vdsim.AeroForce]
        q = 0.5 * inp.rho * inp.v_air_body[0] ** 2
        return [vdsim.AeroForce("wing", (0.0, 0.0, -1.2 * q), (0.55, 0.0, 0.10))]

dyn.set_aero_module(MyAero())              # L2-L5; on L5 before reset()
```

`force_body` is in body axes (drag −x, downforce −z) [N]; `point` is in the
element frame of 26.1 [m]. The forces go through the axle / wheel / wrench split
of 26.2 and 26.3, so a module that re-implements the elements reproduces the
built-in path. `inp` carries `v_air_body`, `slip_angle`, `dz_heave`, `pitch`,
`roll`, `dh_front` ($\Delta z - a\theta$), `dh_rear` ($\Delta z + b\theta$) and
`rho` (`air_density`). Fy acts on L5; L2–L4 ignore it with one warning.
