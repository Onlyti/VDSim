# VDSim plant for the VLA thesis (T-IV) — implementation spec (GO, 2026-06-17)

Next-session entry point. Self-contained: implement this, then `closed_loop_sim.py`'s Python
dynamic-bicycle plant is drop-in replaced by VDSim. MPC (acados, Python) is UNCHANGED — only
the plant is swapped. Reference behaviour: `~/vla_design/sim/closed_loop_sim.py`
(`plant_tire/plant_deriv/rk4/usage_demanded`).

## Decisions (thesis-side, FINAL)
1. **Level = Ld2 7DOF** (`create_seven_dof`). CmdL3 native + load transfer + fast = enough for
   the P1 gate. Roll (Ld3 14DOF) = optional for final paper figures only, not now. **Ld5
   forbidden** (it maps Fx→throttle).
2. **AWD = CmdL3 `Fx_total` drop-in**, 50/50 front/rear. MPC uses `KF_DRIVE=0.5`. Expose
   `drive_split_front=0.5` in the yaml and make Ld2 split `Fx_total` by it. (No per-axle CmdL1.)
3. **Friction circle = combined slip is mandatory.** The headline acceptance case is
   **brake+turn simultaneously on the low-μ patch** (the pre-brake narrative), NOT lateral-only.
4. **friction patch provider = per-wheel contact-point μ.** `create_friction_patch_ground(
   base_mu, [(x0,x1,mu)])`, straight road so s=x. Per-wheel (front enters the patch before
   rear = real time lag), an upgrade over the Python CG-single-μ plant. μ(x,y) general would
   give split-μ for free, but x-interval is sufficient for P1.
5. **Metric lives in analysis (MPC side), NOT the plant.** The plant exposes RAW GT only:
   per-wheel actual Fx,Fy,Fz + the real μ that wheel used (`wheel_mu()`). The headline
   violation `‖F_belief(μ_nom)‖/(μ_real·Fz)` needs the MPC belief → computed thesis-side. A
   plant-side `actual/(μ·Fz)` (≤1, clamp-active = at-limit) is incidental, not headline. Do
   NOT put usage logic in the plant.
6. **dry match = qualitative; keep the model mismatch.** MF96 (plant) ≠ the MPC's saturating
   bicycle is an INTENDED plant-model mismatch (the chance constraint absorbing it is the
   stronger robustness story). Do not match them. BUT align the low-slip cornering stiffness so
   dry is qualitatively consistent: MF96 `Ca = B·C·D` must give front 2.2e5 / rear 1.6e5 N/rad
   with `C=1.3`, `D=μ·Fz` ⇒ `B = Ca/(C·D)`.
7. **Determinism / frame.** LuGre OFF + fixed `substep_dt` (figure reproducibility); confirm no
   nondeterministic element. ISO 8855, +δ→+yaw→+Y, wheel FL0/FR1/RL2/RR3.
8. **Low-speed = VLOW, not the kinematic blend.** The tyre already has a slip-denominator VLOW
   floor (`tire_model.cpp` `kTireSpeedEps`). The VEHICLE-side kinematic blend (seven_dof
   `lambda = clamp(speed/kStickBlend)`) fades the dynamic tyre at low speed and would MASK grip
   loss → bypass it on the plant path (rely on the tyre VLOW). Maneuvers are 16.7 m/s ≫
   kStickBlend so it rarely engages, but bypass it for grip-loss consistency. (Aligns with
   roadmap T3 "VLOW-class unified low speed".)

## ioniq5_awd.yaml (values FINAL)
`m=2359, Iz=3400, lf=1.17, lr=1.80, mu_nom=0.9, drive_split_front=0.5`. track / h_cg = public
Ioniq5-class (NO confidential/measured data). MF96 BCD `D=mu·Fz, C=1.3, B=Ca/(C·D)`.
**TRAP: `Ca_f=2.2e5 / Ca_r=1.6e5` are AXLE values** (the thesis bicycle is single-track), but
7DOF has 4 wheels with PER-WHEEL MF. So per front wheel use `Ca=Ca_f/2` and
`D=mu·Fz_front_wheel` (`Fz_front_wheel = m·g·lr/L / 2`); likewise rear. Calibrate B at the
static per-wheel Fz. (Get this wrong ⇒ each axle 2× too stiff.)

## Already in VDSim — REUSE, do not reimplement
- Lockstep step API: `vdsim` pybind `SimSession` — `sess.reset(make_init_state(...))`,
  `sess.set_input(cmd)`, step, state get. Pattern: `examples/estimator_in_loop.py`.
- Tyre saturation: Pacejka MF96 combined slip (`model_provides_combined_slip`) — saturates at μFz.
- Per-wheel GT getters: `tire_forces_wheel`(Fx,Fy in the CONTACT/wheel frame — USE THIS for the
  friction circle, not `tire_forces_body`) · `tire_Fz` · `wheel_slip_ratio`(κ) ·
  `wheel_slip_angle`(α). Vehicle: State X,Y,ψ,vx,vy,r + `ax/ay_body_est` + `beta()`.
- Load transfer: Ld2 7DOF (`WeightTransfer` tests). YAML params: `VehicleParams` / `Vehicle.preset`.
- Per-wheel μ contact: `create_split_mu_ground` (y-based; the new provider is the x-based analog).
- High-level Python: `python/vdsim_lab.py` (build session + step loop + per-wheel GT CSV) —
  build the wrapper ON this, don't duplicate.

## CORRECTION — Fx is NOT a clean drop-in (the main coordination item)
`vdsim.CmdL3 = {Fx_total[N], steer_angle_wheel[rad]}` LOOKS right, but **Ld2's `lower_to_l4`
THROTTLE-MAPS it**: `throttle = Fx_total/(1500·5)` → drivetrain → wheel torque → slip. So the
commanded Fx_total is NOT delivered as-is (arbitrary 7500 N = full-throttle calibration) — this
is exactly the pedal/throttle map the thesis forbids (P0.2). Steer (δ) IS clean.
- Good side: the torque path runs wheel-spin → slip-ratio → MF96 combined, so longitudinal
  grip-loss and κ ARE physical (the friction circle works).
- Required: a **direct-Fx force-input path in Ld2** that bypasses the throttle map and splits
  `Fx_total` by `drive_split_front`. Decide how the longitudinal channel saturates:
  (i) apply Fx as a force + clamp combined `‖[Fx,Fy]‖ ≤ μ·Fz` (grip-loss via clamp; κ becomes
  diagnostic only), or (ii) convert Fx→wheel torque and keep the spin/slip dynamics (κ stays
  physical — preferred, since the thesis wants κ as GT). This is deliverable #0 below and the
  acceptance #2 crux.

## Deliverables (the job)
0. **Direct-Fx force-input path in Ld2** (bypass the throttle map; split by `drive_split_front`;
   decide force+clamp vs Fx→torque per the CORRECTION above). Without this, acceptance #2 fails
   and Fx_commanded ≠ Fx_delivered.
1. **`create_friction_patch_ground(base_mu, [(x0,x1,mu)...])`** — per-wheel contact-point μ by
   wheel world-x (front-before-rear lag). C++ provider + pybind. (μ(x,y) general = bonus.)
2. **`wheel_mu()` getter** on the dynamics (the real μ each wheel used this step) + pybind.
   (forces/Fz already exposed.) No usage computation.
3. **`configs/.../ioniq5_awd.yaml`** with the §8 values + the BCD mapping.
4. **`VDSimPlant` Python wrapper** (`from vdsim_plant import VDSimPlant`): ctor(config,
   friction_patch, base_mu, dt) → `reset(state0)` → `step([delta, Fx]) -> obs dict`. Internally:
   Ld2 SimSession + CmdL3 + the getters, on top of `vdsim_lab`. obs (per §API).
5. **3 smoke tests** = acceptance §below. #2 MUST be brake+turn on the patch (combined).

## Target API (drop-in)
```python
plant = VDSimPlant(config="ioniq5_awd.yaml", friction_map=[(s0,s1,0.5)], base_mu=0.9, dt=0.05)
plant.reset(state0=[X,Y,psi,vx,vy,r])
for k in range(N):
    u = mpc.solve(...)              # u = [delta_roadwheel_rad, Fx_N]
    obs = plant.step(u)            # one fixed-dt step
    # obs: { X,Y,psi,vx,vy,r,ax,ay,beta,
    #        wheel: [{Fx,Fy,Fz,alpha,kappa,mu}]*4 }   (mu = real μ used; no usage field)
```

## Acceptance
1. **dry qualitative**: μ=0.9 uniform, moderate lane change (16.7 m/s) — trajectory / yawrate
   sign+magnitude qualitatively match the Python bicycle plant (NOT bit-exact; mismatch kept).
2. **patch combined grip-loss (headline)**: **brake+turn on the μ=0.5 patch** — when the
   controller over-demands, per-wheel `‖[Fx,Fy]‖` saturates at `μ·Fz` and the vehicle
   slips/departs. This is where a throttle-mapped Fx (wrong) would show up.
3. **GT consistency**: per-wheel `‖[Fx,Fy]‖ ≤ μ·Fz` (+ numeric eps) always; `ΣFx,ΣFy ↔ m·ax,ay`.
4. **determinism**: same input twice ⇒ bit/near-equal.
5. **speed**: one Ld2 7DOF traj (≈5 s, dt=0.05) ≪ 1 s wall-clock.

## Verify / coordinate before claiming done (open risks)
- **deliverable #0 (direct-Fx path)** is the crux — see CORRECTION. Ld2 currently throttle-maps
  Fx_total; acceptance #2 fails until a direct force/torque path delivers the commanded Fx and
  grip-loses on over-demand. (Lateral grip loss via MF96 is already certain.)
- **wheel_spin seed at reset = `vx / wheel_radius`** (7DOF integrates spin; unseeded ⇒ a huge
  first-step κ transient / spurious longitudinal force). `make_init_state` already does this —
  confirm the wrapper uses it, not a raw State.
- **plant obs = TRUE state** (P1 has no sensor noise): use SimSession's `true_state`, not the
  measured/noisy state.
- **friction circle frame**: use `tire_forces_wheel()` (contact frame), not `tire_forces_body()`.
- **vehicle kinematic-blend bypass** on the plant path (decision 8; rely on tyre VLOW).
- **dt = 0.05 ZOH**: the MPC sample = plant.step dt; VDSim subdivides internally (substep_dt).
- straight road ⇒ s≈x for the patch; fine for lane-change/DLC, revisit if the path curves a lot.

## Guardrails
wheel FL0/FR1/RL2/RR3, ISO 8855, YAML params, estimation noise = Q-process/R-meas (plant-
irrelevant). No confidential/measured tyre data (Ioniq5 = public approx). No push/force/tag
before approval; keep tests green. Fidelity discipline: 7DOF + MF + load transfer is enough —
no new dynamics, no full suspension (P2). Reuse Ld2 + MF96 + SimSession + vdsim_lab; only add
the patch provider, the μ getter, the yaml, the wrapper, the smokes.
