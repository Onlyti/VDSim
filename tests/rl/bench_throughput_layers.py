"""N4: where the RL throughput actually goes (PROP-326 item 4).

Three layers measured in the SAME unit (physics ticks/s) so they are comparable:
  L0 core     -- VecSession.tick(), C++ only
  L1 adapter  -- VDSimVecEnv.step() with random actions (obs copy, reward, reset)
  L2 training -- PPO.learn() (adds policy forward + backward)
"""
import json, sys, time
import numpy as np
sys.path.insert(0, "/home/ailab-12/git/VDSim/build/python")
sys.path.insert(0, "/home/ailab-12/git/VDSim/python")
import vdsim
from vdsim_rl import EnvConfig, make_sb3_vec_env

CFG = EnvConfig.from_yaml("/home/ailab-12/git/VDSim/configs/rl/fast_env.yaml")
DT, LEVEL = CFG.dt, CFG.level
AR = CFG.action_repeat
out = {"substep_dt": CFG.max_substep_dt, "dt": DT, "level": LEVEL,
       "action_repeat": AR}

# ---------- L0: core only, thread sweep at the adopted training substep ----------
vp, tp = vdsim.VehicleParams(), vdsim.TireParams()
sp = vdsim.SolverParams()
sp.max_substep_dt = CFG.max_substep_dt
sp.max_substeps = CFG.max_substeps
ENVS, STEPS = 64, 4000
core = []
for th in (1, 2, 4, 8, 12, 16, 20, 22):
    v = vdsim.make_vec_session(ENVS, vp, tp, level=LEVEL, nominal_dt=DT,
                               solver=sp, threads=th)
    v.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 10.0)])
    c = vdsim.CmdL4(); c.throttle = 0.3; c.steer_angle_wheel = 0.02
    v.set_input_all(c)
    v.tick(DT, 50)
    t0 = time.perf_counter(); v.tick(DT, STEPS); el = time.perf_counter() - t0
    core.append({"threads": th, "ticks_per_s": ENVS * STEPS / el})
    print("L0 core threads=%2d  %12.0f ticks/s" % (th, ENVS * STEPS / el))
out["L0_core"] = core
core_peak = max(r["ticks_per_s"] for r in core)

# ---------- L1: python adapter, random actions ----------
adapter = []
for n in (16, 32, 64):
    env = make_sb3_vec_env(n, CFG, seed=1)
    env.reset()
    a = np.zeros((n, env.action_space.shape[0]), dtype=np.float32)
    for _ in range(20):
        env.step(a)
    N = 400
    t0 = time.perf_counter()
    for _ in range(N):
        env.step(a)
    el = time.perf_counter() - t0
    env.close()
    tps = n * N * AR / el
    adapter.append({"n_envs": n, "ticks_per_s": tps, "env_steps_per_s": n * N / el})
    print("L1 adapter n=%2d  %12.0f ticks/s" % (n, tps))
out["L1_adapter"] = adapter
adapter_peak = max(r["ticks_per_s"] for r in adapter)

# ---------- L2: full PPO ----------
from stable_baselines3 import PPO
train = []
for n in (16, 64):
    env = make_sb3_vec_env(n, CFG, seed=2)
    m = PPO("MlpPolicy", env, seed=2, n_steps=256, batch_size=512, verbose=0,
            device="cpu")
    t0 = time.perf_counter()
    m.learn(total_timesteps=50_000)
    el = time.perf_counter() - t0
    env.close()
    tps = 50_000 * AR / el
    train.append({"n_envs": n, "ticks_per_s": tps, "env_steps_per_s": 50_000 / el})
    print("L2 PPO     n=%2d  %12.0f ticks/s" % (n, tps))
out["L2_training"] = train
train_peak = max(r["ticks_per_s"] for r in train)

out["summary"] = {
    "core_peak_ticks_per_s": round(core_peak),
    "adapter_peak_ticks_per_s": round(adapter_peak),
    "training_peak_ticks_per_s": round(train_peak),
    "adapter_frac_of_core": round(adapter_peak / core_peak, 3),
    "training_frac_of_core": round(train_peak / core_peak, 3),
}
print("SUMMARY", json.dumps(out["summary"]))
with open("/home/ailab-12/git/VDSim/tests/rl/_artifacts/n4_throughput.json", "w") as fh:
    json.dump(out, fh, indent=2)
print("N4_DONE")
