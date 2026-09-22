# Reinforcement learning with VDSim (`vdsim_rl`)

`vdsim_rl` is a Gymnasium-compatible environment layer over the C++ core. One
`VDSimVecEnv` owns *N* independent vehicle plants and a persistent worker pool,
so a policy can drive dozens of cars at once and collect experience from all of
them in a single call.

## 1. Background — what it is for

Learning a driving controller needs many rollouts, and the rollouts are only
worth having if the car in them behaves like a real car. VDSim supplies both:

- **Vehicle dynamics, not a kinematic toy.** Every env runs one of the VDSim
  plant models with tyre forces, load transfer and the actuator path.
- **Parallel rollouts.** A vector step is one GIL-released C++ call
  (`VecSession.advance`) that ticks every env, evaluates termination and writes
  the flat `float32` observation rows in place. Python never touches per-tick
  data.

| `level` | Plant | Use it when | Cost |
|---|---|---|---|
| `L1` | bicycle (single-track) | lateral behaviour only, fastest iteration | lowest |
| `L2` | 7-DOF (planar body + 4 wheel spins) | default for training | medium |
| `L3` | 14-DOF (adds roll, pitch, heave, suspension) | reward reads per-wheel load or body attitude | highest |

Higher levels are more faithful and slower; section 6 has measured throughput.

## 2. Concepts

### 2.1 Vehicle

`EnvConfig.vehicle` and `EnvConfig.tire` name preset files by stem:
`configs/vehicles/<vehicle>.yaml` and `configs/parts/tire/<tire>.yaml`. The
shipped RL configs (`configs/rl/default_env.yaml`, `configs/rl/fast_env.yaml`)
select the generic car:

```yaml
vehicle: generic_sedan
tire: generic_pacejka
```

`generic_sedan` / `generic_pacejka` are the values `core/src/params.cpp`
compiles in as `VehicleParams()` / `TireParams()`, written out as files so a
run records the car it used. They are a solver default set, not a measured
vehicle.

- The preset is read **once, in the parent process**, and shared by every env.
  No file is written.
- A key the C++ parser would ignore (a typo), a missing required key (mass,
  wheelbase, CG position, tracks, CG height, wheel radius; tyre `mu_nominal`,
  `Fz_nominal`), or a catalog-format part file (`schema:` / `body:`) raises
  `ValueError` — it never falls back to generic defaults silently.
- `vehicle=None` (the `EnvConfig()` dataclass default, kept for old code) uses
  the C++ built-in generic car and emits
  `UserWarning: vehicle=None: C++ built-in generic parameters`.
- A car whose parameters must stay out of this repository goes in a private
  root with the same layout: set `VDSIM_PRIVATE_CONFIGS=/abs/root` and keep
  `<root>/vehicles/<stem>.yaml` (+ `<root>/parts/tire/<stem>.yaml`), or pass
  `EnvConfig(vehicle="<stem>", vehicle_file="/abs/path.yaml")`. Search order:
  `vehicle_file` (absolute only) -> `$VDSIM_PRIVATE_CONFIGS` -> `configs/`.
- `reset()` returns `info = {"vehicle", "tire", "param_hash", "source"}` and
  the same dict is in `env.metadata["vdsim"]`. `param_hash` is a sha256 over
  the parameters the core actually received (plus the `.tir` file bytes), so a
  training log can prove which car it trained on; `source` is `public` for a
  preset read from `configs/` and `private` for anything else. Neither the
  info dict nor the metadata carries a path.

### 2.2 Observation

`obs_fields` lists the observation columns left to right; a per-wheel name
expands to four columns in FL, FR, RL, RR order (`slip_ratio` becomes
`slip_ratio.0 … slip_ratio.3`). `env.obs_columns` gives the expanded names.
Frame: ISO 8855 (x forward, y left, z up), angles in radians.

| Field | Unit | Meaning |
|---|---|---|
| `vx`, `vy` | m/s | body-frame velocity |
| `yaw_rate` | rad/s | yaw rate, + = turning left |
| `beta` | rad | side-slip angle |
| `ay` | m/s² | body lateral acceleration, + = left |
| `x`, `y`, `z` | m | position in the odom frame; `y` is the lateral offset from the road centre |
| `yaw` | rad | heading in the odom frame |
| `slip_ratio` | – | per wheel |
| `slip_angle` | rad | per wheel |
| `fz` | N | per-wheel vertical load |
| `wheel_mu` | – | per wheel, as reported in `SimOutput.wheel_mu` |
| `tire_fx`, `tire_fy` | N | per-wheel tyre forces in the wheel frame |
| `steer_applied`, `throttle_applied`, `brake_applied` | rad, –, – | commands after the actuator |

### 2.3 Action

`Box(-1, 1, shape=(2,))` per env:

- `a[0]` steer, scaled to `±max_steer` rad at the wheel (default 0.5 rad);
- `a[1]` pedal, `> 0` throttle, `< 0` brake.

Actions are clipped to `[-1, 1]`. One action is held for `action_repeat`
physics ticks (default 4 × 5 ms = 20 ms control interval).

### 2.4 Reward

`default_reward` keeps the lane and holds speed, per control step:

```
r = 1 - |y| / max_lateral - 0.05 |vx - speed_target| - 0.2 |beta| - 0.05 |Δsteer|
```

minus `crash_penalty` (default 10) on a terminating step. Pass
`reward_fn(obs, col, action, prev_action, cfg) -> (N,)` to replace it.

### 2.5 Episode end

| Reason | Condition (`<= 0` disables) | Gymnasium flag |
|---|---|---|
| `off_track` | `|y - lane_y| > max_lateral` | terminated |
| `rollover` | `|roll| > max_roll` | terminated |
| `spin_out` | `|beta| > max_beta` or `|yaw_rate| > max_yaw_rate` | terminated |
| `stall` | planar speed `< min_speed` | terminated |
| `time_limit` | episode sim time `>= time_limit_s` | truncated |

A non-finite state ends the episode as `nan_state` (checked first). The checks
run in the order rollover, off_track, spin_out, stall, time_limit; the first hit
is the reported reason. `info["termination_reason"]`
names the reason; `VDSimVecEnv` auto-resets finished envs and keeps the last
observation in `info["final_observation"]`.

### 2.6 Reset randomization

Every reset draws, per env: initial speed from `speed_range` [m/s], lateral
offset `±lateral_range` [m] and heading `±yaw_range` [rad].

### 2.7 Domain randomization

`(lo, hi)` multipliers drawn per env at every reset, `(1, 1)` = off:
`mass_scale_range` (mass, sprung mass and inertia), `mu_scale_range` (road
friction), `tire_stiffness_scale_range` (tyre B coefficients and cornering
stiffness), `tire_mu_scale_range` (tyre peak mu), and
`sensor_delay_range` [s] (`< 0` keeps the configured delay). The multipliers
scale the **loaded preset** — with `vehicle: generic_sedan`, `mass_scale 1.1`
means 1.1 × 1500 kg; with any other preset it is 1.1 × that preset's mass,
never 1.1 × the built-in default.

### 2.8 Parallelism and seeds

`num_envs` plants share one C++ worker pool; `threads: 0` uses every hardware
thread. `reset(seed=s)` gives env *i* its own Python stream (from
`SeedSequence(s).spawn`) and core noise seed `s + i`, so a run is reproducible
env by env.

## 3. Quick start

```bash
pip install vdsim gymnasium        # or a locally built wheel; add stable-baselines3 for 4.2
```

```python
from vdsim_rl import EnvConfig, VDSimEnv

env = VDSimEnv(EnvConfig(vehicle="generic_sedan", tire="generic_pacejka"), seed=0)
obs, info = env.reset(seed=0)
print(info["vehicle"], env.obs_columns)
for _ in range(200):
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    if terminated or truncated:
        print("ended:", info["termination_reason"])
        obs, info = env.reset()
```

## 4. Examples

### 4.1 Random policy on 16 envs

```python
from vdsim_rl import EnvConfig, VDSimVecEnv

cfg = EnvConfig.from_yaml("configs/rl/default_env.yaml")    # generic car, 1 ms
env = VDSimVecEnv(16, cfg, seed=0)
obs, info = env.reset(seed=0)                                # obs: (16, obs_dim)
for _ in range(500):
    obs, rew, term, trunc, info = env.step(env.action_space.sample())
    if "termination_reason" in info:                         # auto-reset done
        print([r for r in info["termination_reason"] if r])
```

### 4.2 Short PPO run (Stable-Baselines3, optional dependency)

```python
from stable_baselines3 import PPO
from vdsim_rl import EnvConfig, make_sb3_vec_env

cfg = EnvConfig.from_yaml("configs/rl/fast_env.yaml")
venv = make_sb3_vec_env(16, cfg, seed=0)
model = PPO("MlpPolicy", venv, n_steps=256, batch_size=1024, verbose=1, device="cpu")
model.learn(total_timesteps=100_000)
model.save("ppo_lanekeep")
```

`make_sb3_vec_env` implements SB3's own `VecEnv` protocol directly, so no
`DummyVecEnv` wrapper (and no per-env Python object) is involved.

### 4.3 Domain randomization: road friction 0.6–1.0

```python
from vdsim_rl import EnvConfig, VDSimVecEnv

cfg = EnvConfig.from_yaml("configs/rl/default_env.yaml")
cfg.mu_scale_range = (0.6, 1.0)
cfg.mass_scale_range = (0.9, 1.1)
env = VDSimVecEnv(64, cfg, seed=1)
env.reset(seed=1)
print([round(d.mu_scale, 2) for d in env.core.randomization[:8]])   # this reset's draws
```

`configs/rl/fast_env.yaml` already turns on mass (0.85–1.15), friction
(0.6–1.0) and tyre stiffness (0.85–1.15) randomization.

## 5. Demo videos

Both videos draw every frame from a recorded plant state — no interpolation,
no synthesised frames. The caption in each frame names vehicle, tyre, level,
N, policy, seed, config and commit. Reproduce with the two scripts in
`examples/rl/` (system `ffmpeg` needed for mp4; without it the scripts write
PNG frames and print the encode command).

**A — gym grid** ([`assets/rl/vec_grid.mp4`](assets/rl/vec_grid.mp4)):
16 envs, random policy, 20 s.

<video src="../assets/rl/vec_grid.mp4" controls muted width="100%"
       poster="../assets/rl/vec_grid_reset.png"></video>

What to look at: each tile keeps its own episode counter and return; when a car
crosses the grey band's edge the tile flags `reset <- off_track` and the car
restarts from a new random speed, offset and heading.

```bash
python examples/rl/record_vec_demo.py --config configs/rl/fast_env.yaml \
    --num-envs 16 --seconds 20 --npz /tmp/rl_grid.npz
python examples/rl/render_vec_grid.py /tmp/rl_grid.npz --out docs/assets/rl/vec_grid.mp4
```

**B — one road, 64 cars** ([`assets/rl/vec_overlay.mp4`](assets/rl/vec_overlay.mp4)):
the first episode of 64 envs overlaid, 15 s. Every env runs the same fixed
proportional lane-keeper (a scripted baseline, not a trained policy) plus its
*own* random steer disturbance and pedal, drawn from a per-env generator
(`SeedSequence(seed).spawn(64)`), on top of reset randomization and the
`fast_env.yaml` domain randomization.

<video src="../assets/rl/vec_overlay.mp4" controls muted width="100%"
       poster="../assets/rl/vec_overlay_end.png"></video>

What to look at: 64 independent rollouts — cars that leave the road (grey
marker = episode ended) do so on both sides, and the rest settle at different
distances because their inputs, initial speed and friction differ. For the
spread from randomization alone, record with `--shared-actions`.

```bash
python examples/rl/record_vec_demo.py --config configs/rl/fast_env.yaml \
    --num-envs 64 --seconds 15 --seed 3 --policy lanekeep --steer-scale 0.3 \
    --hold 1.0 --npz /tmp/rl_overlay.npz
python examples/rl/render_vec_grid.py /tmp/rl_overlay.npz --overlay \
    --out docs/assets/rl/vec_overlay.mp4
```

Both videos ship as recorded by those two commands on the generic car of
`fast_env.yaml`. The caption carries the commit of the tree they were recorded
from, so running the commands at that commit reproduces them.

`record_vec_demo.py --trace-dir` writes one `.vdtrace` per env instead, which
`vdsim-render` overlays with its full per-run panels; that view reads well for
a handful of runs, not for 64.

## 6. Limits

Throughput below is in **physics ticks per second summed over all envs**
(one tick = one `dt` step of one plant; a control step is `action_repeat` ticks).

- **Throughput (reference box, i7-13700K 8P+16E).** Core 378 k ticks/s at 22
  threads with the C++ generic car, 93.7 % of it retained through the Python
  adapter; end-to-end PPO reaches 61 k ticks/s because the CPU policy update
  dominates (`docs/ROADMAP.md` §10, `tests/rl/bench_throughput_layers.py`).
- **`fast_env.yaml`'s substep was chosen on a criterion fixed before the
  measurement:** ay and yaw-rate ≤ 2 %, front-left vertical load ≤ 5 % worst
  error against a finer-step reference of the same plant (L2, 5 ms tick,
  2026-09-22). The reference was first 0.1 ms, then re-integrated at
  1 µs; both are shown:

  | car | reference | substep | ay | yaw rate | Fz_FL | throughput (64 envs) | verdict |
  |---|---|---|---|---|---|---|---|
  | generic car | 0.1 ms | 1.0 ms | 0.55 % | 0.20 % | 1.22 % | 131 k steps/s | passes = `default_env.yaml` |
  | generic car | 0.1 ms | 2.5 ms | 1.45 % | 0.53 % | 4.07 % | 326 k steps/s | passes = `fast_env.yaml` |
  | generic car | 1 µs | 1.0 ms | 0.61 % | 0.22 % | 1.34 % | (as above) | passes |
  | generic car | 1 µs | 2.5 ms | 1.51 % | 0.55 % | 4.19 % | (as above) | passes |

  The error falls first-order with the substep (observed order 1.01–1.06
  from 1 ms down to 0.01 ms, against the 1 µs reference), so the 0.1 ms
  reference carried its own 0.06 % (ay) / 0.12 % (Fz_FL) error and the
  0.1 ms rows are slightly optimistic; the verdicts do not change.

  `fast_env.yaml` takes the 2.5 ms rows for 2.4× the throughput; 5 ms was
  rejected on the L3 table in `configs/rl/fast_env.yaml` (ay 6.82 %, Fz
  6.69 %). Both shipped files name the generic car (`generic_sedan` +
  `generic_pacejka`), which is why neither warns about unnamed parameters.
  Pitch and roll are not in the criterion and are coarse at 2.5 ms (brake pitch
  30.6 %, step-steer roll 8.4 %); do not build a reward on them with
  `fast_env.yaml`.
- **A different car changes both accuracy and throughput, so the substep does
  not transfer.** A preset that evaluates a measured `.tir` file through the
  MF2002 backend instead of the built-in MF96 has been measured ≈ 2.7× slower
  at the same substep, with a larger vertical-load error at 2.5 ms. Run
  `tests/rl/gate_g1_substep_accuracy.py --vehicle <stem> --tire <stem>` on any
  preset you swap in before training on it.
- **Presets whose specs are not public.** Point `VDSIM_PRIVATE_CONFIGS` at a
  directory outside the repository (`vehicles/<stem>.yaml`,
  `parts/tire/<stem>.yaml`), or pass `vehicle_file=` an absolute path. The
  private copy wins over a repository file of the same stem, and `info`
  records only the stem, the parameter hash and `source: private` — never the
  path. Accuracy and throughput tables for those presets belong with them, not
  in this guide.
- The road is straight and flat; there is no track or traffic yet.
