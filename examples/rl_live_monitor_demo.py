#!/usr/bin/env python3
"""Watch a short RL-style training loop on 16 envs in the live 3D monitor.

Each env runs its own candidate of a two-gain lane-keeping policy
(steer = -(k_y * y + k_psi * yaw), speed held at the task target). After every
iteration the best quarter of the candidates sets the next sampling mean (a
cross-entropy search). ``VDSimVecEnv(monitor=...)`` draws every env in its own
colour on one page; an env that ends its episode restarts at once and only its
trail is cleared. The run is not paced: the cars move as fast as the
simulation steps, and the HUD shows the sim/wall rate (``x N``).

    python examples/rl_live_monitor_demo.py                  # page: http://<host>:8790/
    python examples/rl_live_monitor_demo.py --layout grid    # one lane per env
    python examples/rl_live_monitor_demo.py --pace 1.0       # real time

The script waits up to ``--wait`` seconds for a page to connect before the
first step; open the printed address in that time.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

from vdsim_rl import VDSimVecEnv  # noqa: E402

K_V = 0.3            # speed gain on the pedal action [s/m]


def policy(obs, col, gains, v_ref):
    """Actions for all envs: lane keeping with per-env gains (k_y, k_psi).

    :param obs: observations, one row per env.
    :param col: column index by name (``VDSimVecEnv.obs_columns``).
    :param gains: (num_envs, 2) array of k_y [1/m], k_psi [-].
    :param v_ref: held speed [m/s].
    :returns: (num_envs, 2) actions in [-1, 1] (steer, pedal).
    """
    y, yaw, vx = obs[:, col["y"]], obs[:, col["yaw"]], obs[:, col["vx"]]
    a = np.empty((len(obs), 2), dtype=np.float32)
    a[:, 0] = np.clip(-(gains[:, 0] * y + gains[:, 1] * yaw), -1.0, 1.0)
    a[:, 1] = np.clip(K_V * (v_ref - vx), -1.0, 1.0)
    return a


def main():
    """Run the search with the monitor on and print one line per iteration."""
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--envs", type=int, default=16)
    ap.add_argument("--iters", type=int, default=6)
    ap.add_argument("--steps", type=int, default=300, help="steps per iteration")
    ap.add_argument("--port", type=int, default=8790)
    ap.add_argument("--layout", choices=("overlay", "grid"), default="overlay")
    ap.add_argument("--pace", type=float, default=None, help="sim/wall ratio to hold")
    ap.add_argument("--wait", type=float, default=10.0, help="wait for a page [s]")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    env = VDSimVecEnv(a.envs, seed=a.seed, monitor=a.port, monitor_layout=a.layout,
                      monitor_pace=a.pace, monitor_wait_s=a.wait)
    col = {c: i for i, c in enumerate(env.obs_columns)}
    v_ref = env.core.cfg.speed_target
    rng = np.random.default_rng(a.seed)
    mean, std = np.array([0.05, 0.5]), np.array([0.2, 1.0])
    n_elite = max(1, a.envs // 4)
    obs, _ = env.reset(seed=a.seed)
    try:
        for it in range(a.iters):
            gains = mean + std * rng.standard_normal((a.envs, 2))
            ret = np.zeros(a.envs)
            ends = np.zeros(a.envs, dtype=int)
            for _ in range(a.steps):
                obs, rew, term, trunc, _ = env.step(policy(obs, col, gains, v_ref))
                ret += rew
                ends += term
            elite = gains[np.argsort(ret)[-n_elite:]]
            mean, std = elite.mean(axis=0), elite.std(axis=0) + 0.01
            print(f"iter {it}: return mean {ret.mean():8.2f} best {ret.max():8.2f} | "
                  f"crashes {int(ends.sum()):3d} | k_y {mean[0]:.3f} k_psi {mean[1]:.3f}",
                  flush=True)
    finally:
        env.close()


if __name__ == "__main__":
    main()
