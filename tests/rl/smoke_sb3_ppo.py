# R2: stable-baselines3 PPO smoke on make_sb3_vec_env (needs torch + SB3).
# Not a ctest target: CI does not install torch.  The torch-free checks are in
# test_rl_env.py (ctest rl_env).  Paths are repo-relative; VDSIM_BUILD_PY
# overrides the built module dir.
import os, sys, time
from pathlib import Path
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, os.environ.get("VDSIM_BUILD_PY", str(REPO / "build" / "python")))
sys.path.insert(0, str(REPO / "python"))
import numpy as np
from stable_baselines3 import PPO
from vdsim_rl import EnvConfig, make_sb3_vec_env

cfg = EnvConfig.from_yaml(str(REPO / "configs/rl/default_env.yaml"))
venv = make_sb3_vec_env(8, cfg, seed=11)
t0 = time.perf_counter()
model = PPO("MlpPolicy", venv, n_steps=64, batch_size=64, n_epochs=2, verbose=0,
            device="cpu")
model.learn(total_timesteps=2048)
el = time.perf_counter() - t0
obs = venv.reset()
act, _ = model.predict(obs, deterministic=True)
assert np.asarray(act).shape == (8, 2)
print(f"SB3 PPO learn(2048) on 8 envs: {el:.1f} s, predict -> {np.asarray(act).shape}")
venv.close()
print("SB3 SMOKE PASSED")
