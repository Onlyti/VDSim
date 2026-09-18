"""N2: PPO 100k-step real training run on VDSimVecEnv (PROP-326 item 2).

Success criterion fixed before the run:
  (A) mean episode return over the last 10% of episodes > first 10% by >20%
  (B) mean episode length rises (fewer early terminations)
Writes rl_train_100k.csv (rollout-level) and rl_train_100k.json (verdict).
"""
import json, os, sys, time
import numpy as np

sys.path.insert(0, "/home/ailab-12/git/VDSim/build/python")
sys.path.insert(0, "/home/ailab-12/git/VDSim/python")

from vdsim_rl import EnvConfig, make_sb3_vec_env
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback

OUT = "/home/ailab-12/git/VDSim/tests/rl/_artifacts"
os.makedirs(OUT, exist_ok=True)

CFG = EnvConfig.from_yaml("/home/ailab-12/git/VDSim/configs/rl/fast_env.yaml")
N_ENVS = 16
TOTAL = 100_000
SEED = 20260917


class Recorder(BaseCallback):
    """Collect per-episode return/length, plus rollout-level aggregates."""

    def __init__(self):
        super().__init__()
        self.episodes = []          # (timestep, ep_return, ep_len)
        self.rollouts = []          # (timestep, mean_ret, mean_len, n_ep)
        self._mark = 0

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            ep = info.get("episode")
            if ep is not None:
                self.episodes.append((self.num_timesteps, float(ep["r"]), int(ep["l"])))
        return True

    def _on_rollout_end(self) -> None:
        new = self.episodes[self._mark:]
        self._mark = len(self.episodes)
        if new:
            self.rollouts.append((self.num_timesteps,
                                  float(np.mean([e[1] for e in new])),
                                  float(np.mean([e[2] for e in new])),
                                  len(new)))


def main():
    env = make_sb3_vec_env(N_ENVS, CFG, seed=SEED)
    from stable_baselines3.common.vec_env import VecMonitor
    env = VecMonitor(env)

    model = PPO("MlpPolicy", env, seed=SEED, n_steps=256, batch_size=512,
                learning_rate=3e-4, verbose=1, device="cpu")
    rec = Recorder()
    t0 = time.time()
    model.learn(total_timesteps=TOTAL, callback=rec, progress_bar=False)
    wall = time.time() - t0

    eps = rec.episodes
    with open(os.path.join(OUT, "rl_train_100k.csv"), "w") as fh:
        fh.write("timestep,ep_return,ep_len\n")
        for t, r, l in eps:
            fh.write(f"{t},{r:.4f},{l}\n")

    k = max(1, len(eps) // 10)
    first_r = float(np.mean([e[1] for e in eps[:k]])) if eps else float("nan")
    last_r = float(np.mean([e[1] for e in eps[-k:]])) if eps else float("nan")
    first_l = float(np.mean([e[2] for e in eps[:k]])) if eps else float("nan")
    last_l = float(np.mean([e[2] for e in eps[-k:]])) if eps else float("nan")

    gain = (last_r - first_r) / abs(first_r) if first_r not in (0.0,) else float("nan")
    verdict = {
        "total_timesteps": TOTAL, "n_envs": N_ENVS, "n_episodes": len(eps),
        "wall_s": round(wall, 1), "steps_per_s": round(TOTAL / wall, 1),
        "first10pct_return": round(first_r, 3), "last10pct_return": round(last_r, 3),
        "return_gain_frac": round(gain, 4),
        "first10pct_len": round(first_l, 1), "last10pct_len": round(last_l, 1),
        "criterion_A_return_up_20pct": bool(gain > 0.20),
        "criterion_B_len_up": bool(last_l > first_l),
        "rollouts": rec.rollouts,
    }
    verdict["PASS"] = bool(verdict["criterion_A_return_up_20pct"]
                           and verdict["criterion_B_len_up"])
    with open(os.path.join(OUT, "rl_train_100k.json"), "w") as fh:
        json.dump(verdict, fh, indent=2)
    print("VERDICT", json.dumps({k: v for k, v in verdict.items() if k != "rollouts"}))
    print("N2_DONE")


if __name__ == "__main__":
    main()
