# R2/R3/R4 verification: gym adapter, termination codes, flat obs buffer.
import json, sys, time
sys.path.insert(0, "/home/ailab-12/git/VDSim/build/python")
sys.path.insert(0, "/home/ailab-12/git/VDSim/python")
import numpy as np
import vdsim
from vdsim_rl import (EnvConfig, VDSimVecEnv, VDSimEnv, make_sb3_vec_env,
                      expand_obs_fields, TERM_NAMES)

res = {}

# ---- R4a: YAML declaration -> layout, and the buffer is written in place ----
cfg = EnvConfig.from_yaml("/home/ailab-12/git/VDSim/configs/rl/default_env.yaml")
env = VDSimVecEnv(8, cfg, seed=0)
print("obs columns:", env.obs_columns)
obs, _ = env.reset(seed=0)
assert obs.dtype == np.float32 and obs.shape == (8, len(env.obs_columns))
res["obs_dim"] = int(env.core.vs.obs_dim)
res["columns"] = env.obs_columns

buf_id = env.core.obs.__array_interface__["data"][0]
env.step(np.zeros((8, 2), dtype=np.float32))
assert env.core.obs.__array_interface__["data"][0] == buf_id, "obs buffer reallocated"
res["zero_copy"] = True

# ---- R4b: obs columns match an independent SimSession readout ----
c2 = EnvConfig(**{**cfg.__dict__, "speed_range": (15.0, 15.0), "lateral_range": 0.0,
                  "yaw_range": 0.0, "time_limit_s": -1.0})
e1 = VDSimVecEnv(1, c2, seed=3)
o, _ = e1.reset(seed=3)
a = np.array([[0.3, 0.4]], dtype=np.float32)
for _ in range(50):
    o, r, te, tr, info = e1.step(a)
s = e1.core.vs.at(0)
out = s.output()
ref = {"vx": out.state.vx(), "vy": out.state.vy(), "yaw_rate": out.state.yaw_rate(),
       "beta": out.state.beta(), "ay": out.ay, "y": out.state.position[1],
       "yaw": out.state.yaw(), "slip_ratio.0": out.slip_ratio[0],
       "slip_angle.3": out.slip_angle[3], "steer_applied": out.steer_applied,
       "throttle_applied": out.throttle_applied, "brake_applied": out.brake_applied}
col = e1.core.col
worst = 0.0
for k, v in ref.items():
    d = abs(float(o[0, col[k]]) - float(v))
    worst = max(worst, d / max(1.0, abs(v)))
print(f"obs vs SimSession readout: worst relative diff = {worst:.3e} (float32 rounding)")
res["obs_worst_rel_diff"] = worst
assert worst < 1e-6

# ---- R3: every termination code reachable, with the right reason ----
def force(kind):
    c = EnvConfig(**{**cfg.__dict__})
    c.speed_range = (20.0, 20.0); c.lateral_range = 0.0; c.yaw_range = 0.0
    c.time_limit_s = -1.0
    act = np.array([[0.0, 0.5]], dtype=np.float32)
    if kind == "off_track":
        c.max_lateral = 0.5; act = np.array([[0.5, 0.3]], dtype=np.float32)
    elif kind == "spin_out":
        c.max_beta = 0.05; act = np.array([[1.0, 0.5]], dtype=np.float32)
    elif kind == "rollover":
        c.max_roll = 0.001; act = np.array([[1.0, 0.5]], dtype=np.float32)
    elif kind == "stall":
        c.min_speed = 25.0
    elif kind == "time_limit":
        c.time_limit_s = 0.2
    e = VDSimVecEnv(1, c, seed=1)
    e.reset(seed=1)
    for _ in range(400):
        o, r, te, tr, info = e.step(act)
        if te[0] or tr[0]:
            return info["termination_reason"][0], float(r[0])
    return "none", 0.0

res["term"] = {}
for kind in ("off_track", "spin_out", "rollover", "stall", "time_limit"):
    got, rw = force(kind)
    res["term"][kind] = got
    print(f"termination {kind:11s} -> {got:11s} (reward {rw:+.2f})")
    assert got == kind, f"{kind} produced {got}"

# ---- R2: gymnasium API shape + single-env wrapper ----
import gymnasium as gym
se = VDSimEnv(cfg, seed=7)
o, _ = se.reset(seed=7)
o, r, te, tr, info = se.step(np.array([0.1, 0.2], dtype=np.float32))
assert isinstance(r, float) and isinstance(te, bool)
res["single_env_ok"] = True
print("VDSimEnv single-env step ok; obs", o.shape, "reward", round(r, 4))

ve = VDSimVecEnv(16, cfg, seed=5)
o, _ = ve.reset(seed=5)
assert ve.observation_space.shape == (16, len(ve.obs_columns))
assert ve.action_space.shape == (16, 2)
n_done = 0
t0 = time.perf_counter()
for _ in range(500):
    o, r, te, tr, info = ve.step(ve.action_space.sample())
    n_done += int((te | tr).sum())
el = time.perf_counter() - t0
sps = 500 * 16 * cfg.action_repeat / el
res["vec_env_ticks_per_s"] = sps
res["auto_resets"] = n_done
print(f"VDSimVecEnv 16 envs x 500 control steps: {sps:,.0f} physics ticks/s, "
      f"{n_done} auto-resets")

# ---- R2: SB3 PPO smoke ----
from stable_baselines3 import PPO
venv = make_sb3_vec_env(8, cfg, seed=11)
t0 = time.perf_counter()
model = PPO("MlpPolicy", venv, n_steps=64, batch_size=64, n_epochs=2, verbose=0,
            device="cpu")
model.learn(total_timesteps=2048)
el = time.perf_counter() - t0
obs = venv.reset()
act, _ = model.predict(obs, deterministic=True)
res["sb3"] = {"seconds": el, "action_shape": list(np.asarray(act).shape)}
print(f"SB3 PPO learn(2048) on 8 envs: {el:.1f} s, predict -> {np.asarray(act).shape}")
venv.close()

json.dump(res, open("/tmp/rl_smoke.json", "w"), indent=1, default=str)
print("ALL CHECKS PASSED")
