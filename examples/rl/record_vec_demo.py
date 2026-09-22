#!/usr/bin/env python3
"""Record a vdsim_rl vector-env rollout for the RL demo videos.

Every stored sample is the plant state the env produced at that control step;
nothing is interpolated or synthesised.  Two outputs, pick either or both:

* ``--npz``: per-step arrays for all envs across auto-resets (video A, the
  gym grid: ``render_vec_grid.py``).
* ``--trace-dir``: one ``.vdtrace`` per env covering its *first* episode only,
  for ``vdsim-render`` / ``vdsim_render.render_multi``.  That overlay has one
  legend row and one HUD row per run, so it reads well up to a handful of runs;
  for dozens use ``render_vec_grid.py --overlay`` on the ``--npz`` output.

Example::

    python examples/rl/record_vec_demo.py --config configs/rl/fast_env.yaml \\
        --num-envs 16 --seconds 20 --npz /tmp/rl_grid.npz
    python examples/rl/record_vec_demo.py --config configs/rl/fast_env.yaml \\
        --num-envs 64 --seconds 15 --seed 3 --policy lanekeep --steer-scale 0.3 --shared-actions \\
        --hold 1.0 --npz /tmp/rl_overlay.npz

The policy is piecewise-constant uniform random: a new action is drawn from
the action space every ``--hold`` seconds and held in between (a per-step
uniform draw averages out to driving straight and shows nothing).  The steer
component is multiplied by ``--steer-scale``.  With ``--shared-actions`` every
env receives the *same* action sequence, so any spread between the envs comes
from reset randomization and domain randomization alone.

``--policy lanekeep`` replaces the random steer by one fixed proportional
lane-keeping law for every env, ``steer = -(k_y*y + k_yaw*yaw)`` on the env's
own observation, plus the random action scaled by ``--steer-scale`` as a
steer disturbance, and keeps the random pedal.  It is a scripted baseline, not
a trained policy; it keeps cars on the road long enough for the randomization
spread to show.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

import vdsim  # noqa: E402
from vdsim_rl import TERM_NAMES, EnvConfig, VDSimVecEnv  # noqa: E402


def git_sha() -> str:
    """Short commit of the tree the demo ran from, or ``"unknown"``."""
    try:
        return subprocess.run(["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return "unknown"


def random_policy(num_envs: int, hold_steps: int, seed: int,
                  steer_scale: float = 1.0, shared: bool = False):
    """Piecewise-constant uniform random actions in [-1, 1]^2.

    :param steer_scale: multiplier on the steer component.
    :param shared: one draw broadcast to every env instead of one per env.
    :returns: callable ``k -> (num_envs, 2)`` float32 action for control step k.
    """
    rng = np.random.default_rng(seed)
    held = np.zeros((num_envs, 2), dtype=np.float32)

    def act(k: int) -> np.ndarray:
        if k % hold_steps == 0:
            held[:] = rng.uniform(-1.0, 1.0, size=(1 if shared else num_envs, 2))
            held[:, 0] *= steer_scale
        return held.copy()
    return act


def lanekeep_policy(num_envs: int, hold_steps: int, seed: int, col: dict,
                    max_steer: float, k_y: float, k_yaw: float, shared: bool,
                    steer_scale: float = 0.0):
    """Fixed P lane-keeper on (y, yaw) + random steer disturbance and pedal.

    :param col: observation column index map of the env.
    :returns: callable ``(k, obs) -> (num_envs, 2)`` action.
    """
    rand = random_policy(num_envs, hold_steps, seed, steer_scale, shared)

    def act(k: int, obs: np.ndarray) -> np.ndarray:
        a = rand(k)
        steer = -(k_y * obs[:, col["y"]] + k_yaw * obs[:, col["yaw"]])
        a[:, 0] = np.clip(steer / max_steer + a[:, 0], -1.0, 1.0)
        return a
    return act


def sample_env(sess) -> dict:
    """One env's plant state, same sources as ``VDSimPlant._record``."""
    o = sess.output()
    st = o.state
    return {
        "pose": (float(st.position[0]), float(st.position[1]), float(st.yaw())),
        "v_body": (float(st.vx()), float(st.vy())),
        "yaw_rate": float(st.yaw_rate()),
        "u_steer": float(o.steer_applied),
        "wheel_F": [(float(o.tire_forces_wheel[i][0]), float(o.tire_forces_wheel[i][1]),
                     float(o.Fz[i])) for i in range(4)],
        "wheel_mu": [float(o.wheel_mu[i]) for i in range(4)],
        "wheel_kappa": [float(o.slip_ratio[i]) for i in range(4)],
        "wheel_alpha": [float(o.slip_angle[i]) for i in range(4)],
    }


def record(cfg: EnvConfig, num_envs: int, seconds: float, seed: int, hold_s: float,
           steer_scale: float = 1.0, shared: bool = False, policy_name: str = "random",
           k_y: float = 0.02, k_yaw: float = 0.3):
    """Run the env; auto-reset is done here so the terminal state is kept.

    ``env.core.step`` / ``env.core.reset_done`` are the two halves of
    ``VDSimVecEnv.step``; calling them separately lets the recorder store the
    state *at* termination before the env is reset.

    :returns: (env, list of per-step dicts)
    """
    env = VDSimVecEnv(num_envs, cfg, seed=seed)
    env.reset(seed=seed)
    ctrl_dt = cfg.dt * cfg.action_repeat
    n_steps = int(round(seconds / ctrl_dt))
    hold = max(1, int(round(hold_s / ctrl_dt)))
    if policy_name == "lanekeep":
        lk = lanekeep_policy(num_envs, hold, seed + 1, env.core.col, cfg.max_steer,
                             k_y, k_yaw, shared, steer_scale)
        policy = lambda k: lk(k, env.core.obs)
    else:
        policy = random_policy(num_envs, hold, seed + 1, steer_scale, shared)
    episode = np.zeros(num_envs, dtype=np.int64)
    ep_t = np.zeros(num_envs)
    steps = [{"k": 0, "episode": episode.copy(), "ep_t": ep_t.copy(),
              "ep_return": np.zeros(num_envs), "code": np.zeros(num_envs, np.int32),
              "env": [sample_env(env.core.vs.at(i)) for i in range(num_envs)]}]
    for k in range(1, n_steps + 1):
        _, _, term, trunc, codes = env.core.step(policy(k - 1))
        ep_t += ctrl_dt
        done = term | trunc
        steps.append({"k": k, "episode": episode.copy(), "ep_t": ep_t.copy(),
                      "ep_return": env.core.ep_ret.copy(), "code": codes.copy(),
                      "env": [sample_env(env.core.vs.at(i)) for i in range(num_envs)]})
        if done.any():
            env.core.reset_done(done)
            episode[done] += 1
            ep_t[done] = 0.0
    return env, steps


def save_npz(path: Path, steps, meta: dict) -> None:
    """Stack the per-step dicts into ``(T, N, ...)`` arrays."""
    arr = lambda key: np.array([[e[key] for e in s["env"]] for s in steps])
    np.savez_compressed(
        path, k=np.array([s["k"] for s in steps]),
        episode=np.array([s["episode"] for s in steps]),
        ep_t=np.array([s["ep_t"] for s in steps]),
        ep_return=np.array([s["ep_return"] for s in steps]),
        code=np.array([s["code"] for s in steps]),
        pose=arr("pose"), v_body=arr("v_body"), yaw_rate=arr("yaw_rate"),
        u_steer=arr("u_steer"), meta=json.dumps(meta))


def save_traces(out_dir: Path, steps, cfg: EnvConfig, env, meta: dict) -> list:
    """One ``.vdtrace`` per env, first episode only (to and incl. termination)."""
    import vdsim_trace
    from vdsim_plant import CONTACT_SCOPE_BY_LEVEL, body_dimensions, \
        measure_mu_aniso, resolve_vehicle_config

    vp, tp = _params(cfg)
    shape, aniso = measure_mu_aniso(tp)
    body_lwh, wheel_width = body_dimensions(resolve_vehicle_config(cfg.vehicle))
    geometry = {
        "wheelbase_m": float(vp.wheelbase), "track_m": float(vp.track_front),
        "steer_ratio": float(vp.steering_ratio),
        "track_front_m": float(vp.track_front), "track_rear_m": float(vp.track_rear),
        "cg_to_front_m": float(vp.cg_to_front), "cg_to_rear_m": float(vp.cg_to_rear),
        "wheel_radius_m": float(vp.wheel_radius_nominal),
        "mass_kg": float(vp.mass), "cg_height_m": float(vp.cg_height),
        "wheel_width_m": wheel_width, "body_lwh_m": body_lwh,
    }
    # u_fx is a pedal command here, not a force: omitted, never zero-filled.
    channels = ("t", "pose", "v_body", "yaw_rate", "u_steer", "wheel_F",
                "wheel_mu", "wheel_kappa", "wheel_alpha")
    ctrl_dt = cfg.dt * cfg.action_repeat
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for i in range(env.num_envs):
        w = vdsim_trace.TraceWriter(
            out_dir / f"env{i:02d}.vdtrace", geometry=geometry,
            tire={"friction_shape": shape, "mu_aniso": aniso,
                  "mu_aniso_source": "measured"},
            repro={"vdsim_version": getattr(vdsim, "__version__", "unknown"),
                   "git_sha": meta["commit"], "param_hash": meta["param_hash"],
                   "seed": meta["seed"], "dt_s": ctrl_dt, "run_id": f"env{i:02d}"},
            producer={"name": "record_vec_demo.py", "version": meta["commit"]},
            channels=channels, extra={"tags": {"rl_env_index": i, **meta["tags"]}},
            role="plant", model_level=cfg.level,
            contact_scope=CONTACT_SCOPE_BY_LEVEL[cfg.level])
        for s in steps:
            if s["episode"][i] > 0:
                break
            w.append({"t": s["k"] * ctrl_dt, **s["env"][i]})
            if s["code"][i] != vdsim.TERM_NONE:
                break
        paths.append(str(w.finalize()))
    return paths


def _params(cfg: EnvConfig):
    from vdsim_rl import load_vehicle_preset
    vp, tp, _ = load_vehicle_preset(cfg.vehicle, cfg.tire)
    return vp, tp


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", default=str(REPO / "configs/rl/fast_env.yaml"))
    ap.add_argument("--num-envs", type=int, default=16)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--hold", type=float, default=0.5, help="policy hold [s]")
    ap.add_argument("--steer-scale", type=float, default=1.0)
    ap.add_argument("--shared-actions", action="store_true",
                    help="same random draws for every env")
    ap.add_argument("--policy", choices=("random", "lanekeep"), default="random")
    ap.add_argument("--k-y", type=float, default=0.02, help="lanekeep [rad/m]")
    ap.add_argument("--k-yaw", type=float, default=0.3, help="lanekeep [rad/rad]")
    ap.add_argument("--npz", type=Path)
    ap.add_argument("--trace-dir", type=Path)
    args = ap.parse_args(argv)
    if not (args.npz or args.trace_dir):
        ap.error("give --npz and/or --trace-dir")

    cfg = EnvConfig.from_yaml(args.config)
    env, steps = record(cfg, args.num_envs, args.seconds, args.seed, args.hold,
                        args.steer_scale, args.shared_actions, args.policy,
                        args.k_y, args.k_yaw)
    prov = env.core.provenance
    codes = np.array([s["code"] for s in steps[1:]])
    reasons = {TERM_NAMES[c]: int((codes == c).sum()) for c in np.unique(codes)
               if c != vdsim.TERM_NONE}
    meta = {"vehicle": prov["vehicle"], "tire": prov["tire"],
            "param_hash": prov["param_hash"], "level": cfg.level,
            "num_envs": args.num_envs, "seconds": args.seconds, "seed": args.seed,
            "policy": ((f"lanekeep P k_y={args.k_y:g} k_yaw={args.k_yaw:g} + random"
                        if args.policy == "lanekeep" else "random")
                       + (" shared" if args.shared_actions else "")
                       + f", hold {args.hold:g} s"
                       + (f", steer x{args.steer_scale:g}" if args.steer_scale != 1.0 else "")),
            "control_dt_s": cfg.dt * cfg.action_repeat, "config": Path(args.config).name,
            "commit": git_sha(), "terminations": reasons,
            "max_lateral_m": cfg.max_lateral, "lane_y_m": cfg.lane_y,
            "tags": {"config": Path(args.config).name, "policy": args.policy}}
    if args.npz:
        save_npz(args.npz, steps, meta)
        print(f"wrote {args.npz}")
    if args.trace_dir:
        paths = save_traces(args.trace_dir, steps, cfg, env, meta)
        print(f"wrote {len(paths)} traces to {args.trace_dir}")
    print(json.dumps(meta))
    return 0


if __name__ == "__main__":
    sys.exit(main())
