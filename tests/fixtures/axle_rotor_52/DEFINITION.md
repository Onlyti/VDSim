# Work order 52 — gate definition (axle tyres R4 + rotor gyroscope V5)

Fixed before the implementation commit. Base: origin/main faf59f2 (canonical 649).
Sources: work order `52_axle_tire_rotor_gyro_workorder.md` §3, PO verdict on the first
reply (mail b0191ad6: D1 option 1, D2 option 1 alias, D3 proposal, D4 (a)(b), V5b2 and
N3 added). Gate numbers (±2 %, 2 N·m not-determinable, 1e-12, bit identity) are the work
order values and are not changed here.

## 1. Test constants
- rotor_inertia I_r = 0.25 kg·m², rotor_ratio G = 5.0, drive RWD (generic sedan values).
- Analytic rotor moment estimate |r · I_r · G · ω_c| ≈ 22.5 N·m (R 8.5 m, 7 m/s) and
  ≈ 23.5 N·m (R 15 m, 9.5 m/s), about 11 × the 2 N·m not-determinable limit.

## 2. Gates and where they live

| ID | Definition | Gate | Test |
|---|---|---|---|
| T1 / V5a | New keys unset: byte probe of 24 L1–L5 cases (`~/q48c_keep/q48c/k1_bytes.cpp`), RL observation and reward sha256 (pedal, accel; `~/q34/rl_digest.py`), closed-loop deterministic arrays, against the baseline tree `~/baselines/vdsim-faf59f2` (re-taken against the merged main if 49a / 51 / 57 land first) | bit identical | measurement procedure (stage driver), plus `L5RotorGyro.V5a_*` |
| T2 | `TireSetup(tp, tp)` vs `tp` through make_sim_session (L1–L5, L5 both paths), make_vec_session (L2 × 2), make_sim_session_heightmap (L3), make_sim_session_psd (L3), 2000 ticks | bit identical | `tests/scripts/test_axle_tire_rotor.py` |
| T3 | Front mu 1.0 / rear mu 1.3, cornering_stiffness 100000, B_lat 9.0 via `TireSetup(front, rear)`; every 50th of 400 ticks, each wheel's live tyre model on the input built from that tick's output (Fz, slip ratio, slip angle, body vx) vs the same wheel of a reference session built through the existing single-tyre path with that axle's parameters | Fx, Fy, Mz rel ≤ 1e-12 (denominator max(\|ref\|, 1 N)); rear vs front-parameter reference must differ > 1e-3 on ≥ 1 sample, else not determinable = failure | same script |
| T4 | Python factories take TireParams or TireSetup (other types TypeError); Lab `Experiment.tire(front, rear=None)`, `Sim(..., tire_rear=None)`, config key `tire_rear`; `tire_rear` without `tire` ValueError. C++ YAML paths (realtime_server, world_scenario, catalog materialize) unchanged (D1 condition 1) | as listed | same script |
| Keys | rotor_inertia / rotor_ratio alias the existing storage; conflict only between written names; to_yaml keeps written names; rotor_ratio ≤ 0, rotor_inertia < 0, unknown rotor_axis invalid_argument / ValueError; powertrain v2 + rotor_inertia / rotor_ratio rejected, rotor_axis accepted; L1–L4 warn once per process for an axis other than none | as listed | `tests/unit/test_rotor_keys.cpp`, Python script |
| V5b | rotor_* = (I, G) with no gyro (no axis on L5, any axis on L1–L4) == engine_rotational_inertia / final_drive_ratio = (I, G), FWD / RWD / AWD, L1–L5 (both L5 paths), 2000 ticks | bit identical | `RotorKeys.V5b_*` |
| V5b2 | rotor_axis +y on, L5 both paths, FWD / RWD / AWD: reflected spin inertia per wheel used by the spin equation (`detail::free_3d_wheel_engine_inertia_obs`) == share · I_r · G² from the test constants (RWD rear, FWD front 0.5; AWD 0.25; others 0), and the wheel inertia (`free_3d_wheel_inertia_obs`) == wheel_inertia or 0.5 m_u R² | bit identical (written as equality of the reflected part, so no subtraction rounding) | `L5RotorGyro.V5b2_*` |
| V5c | V3b skidpad R 8.5 m · 7 m/s, PI hold, last 2 s; off = rotor_axis none, on = +y (roll, e_x) / +x (pitch, e_y); wheel_gyro legacy in both; expected Gr = mean_on((Rω) × (R H_r)), H_r = I_r · G · ω_c · ê, ω_c = (ω_RL + ω_RR)/2, test constants (not core outputs) | \|dD/Gr − 1\| ≤ 2 %, penalty and strut × two axes; \|Gr\| < 2 N·m = not determinable = failure | `L5RotorGyro.V5c_*` |
| V5d | V5c at R 15 m · 9.5 m/s, fixed before any run | same | `L5RotorGyro.V5d_*` |
| G | local canonical (serial preset, C++ and Python counted apart), doc audit FAIL increase 0, theory ch. 20 rotor section, CHANGELOG [0.9.0], configuration notes | as listed | stage driver |

### 2.1 Budget sign convention (V5c)
- D = (E + Lc − T − B) is the angular momentum rate the external moments supply beyond
  the modelled bodies, i.e. ω × H of the unmodelled spin (V3b uses the same D and
  compares it with −G_w = (ω × H_w)·e_x).
- The rotor moment on the chassis is −ω × H_r (work order wording); the budget residual
  carries its negative, so the expected change is +(ω × H_r)·e_k. A sign error gives a
  ratio near −1 and fails.

### 2.2 Strut couple term Lc (D4 (b))
- Lc = Σ_strut (r_arm − x_u) × R F_tire, r_arm = the chassis torque arm of the core.
- faf59f2: the mount x_b + R r_i at the strut-body height h_s (`l5_strut_body_height`).
- After work order 57 reaches main (61fc1cb, harness d6a389a): mount under
  cg_height_ref: sprung (where h_s = cg_height), wheel particle otherwise (Lc ≡ 0).
  The merge of 57 touches the same skidpad lines and updates Lc with them; V5c is
  re-measured after that merge (same definition as 49a P10).
- The V3b `L` term of work order 50 is left as it is (it uses cg_height for the mount;
  in an off/on difference the mount-height error cancels). Not changed by 52.

### 2.3 Pitch budget (D4 (a))
- Heading-frame e_y = (−sin ψ, cos ψ, 0); E, T, B, Lc as V3b, y component.
- Off-run pitch residual mean(D_y)_off is printed as information (no gate).

## 3. Negative checks (stage driver, temporary edits, never committed)

| ID | Edit | Must happen | Then |
|---|---|---|---|
| N1 | Python factories drop the rear axle (pass `TireSetup(front)` instead of the given setup) | T3 fails on L1–L5 | restore → T3 passes |
| N2 | rotor gyro term removed from the L5 derivative | V5c fails on both axes and both paths, the +x pitch gate included | restore → V5c passes |
| N3 | gyro uses G = 1 instead of the shared ratio (spin keeps I_r · G²) | V5c fails | restore → V5c passes |

If N2 does not fail the +x pitch gate, stop and report (PO verdict D4 (a)).

## 4. Information only
- powertrain v2 + rotor_axis: gyro uses the engine rpm state; one information run.
- FWD / AWD: V5b and V5b2 judge; V5c values may be printed as information.
- `make_init_state(vp, tp)` tyre use is reported in the second reply (D1 condition 2).
- catalog assembly rear mu display: one information test with the implementation.

## 5. Stop rules
- A gate missed, a not-determinable case, or a T1 bit difference: stop and report;
  no change to gate numbers, windows, normalisation or expected formulas.
- Measurement-condition or harness corrections follow PROP-2775 rule 3 (proceed, report after).
