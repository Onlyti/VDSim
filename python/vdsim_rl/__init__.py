"""vdsim_rl - RL environment layer over the C++ VecSession.

One VecSession owns N independent plants and a persistent worker pool; a whole
vector step is a single GIL-released call (``advance``) that ticks every env,
evaluates the termination conditions and writes the flat float32 observation
rows in place.  Python never touches per-tick data.

    from vdsim_rl import EnvConfig, VDSimVecEnv
    env = VDSimVecEnv(64, EnvConfig())
    obs, info = env.reset(seed=0)
    obs, rew, term, trunc, info = env.step(env.action_space.sample())

Action (per env, Box(-1, 1, shape=(2,))):
    a[0] = steer, scaled to +-cfg.max_steer [rad] at the wheel
    a[1] = pedal, >0 -> throttle, <0 -> brake

Observation: cfg.obs_fields, flattened left to right; a bare per-wheel name
expands to four columns in FL, FR, RL, RR order.
"""
from __future__ import annotations

import dataclasses
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

import vdsim

__all__ = [
    "EnvConfig", "VDSimVecEnv", "VDSimEnv", "make_sb3_vec_env",
    "expand_obs_fields", "TERM_NAMES", "default_reward",
]

# Per-wheel fields in core/src/vec_session.cpp; a bare name expands to 4 columns.
WHEEL_FIELDS = frozenset({
    "wheel_spin", "slip_ratio", "slip_angle", "fz",
    "susp_compression", "susp_velocity", "wheel_mu", "tire_fx", "tire_fy",
})

TERM_NAMES = {
    vdsim.TERM_NONE: "none",
    vdsim.TERM_OFF_TRACK: "off_track",
    vdsim.TERM_ROLLOVER: "rollover",
    vdsim.TERM_SPIN_OUT: "spin_out",
    vdsim.TERM_NAN_STATE: "nan_state",
    vdsim.TERM_TIME_LIMIT: "time_limit",
    vdsim.TERM_STALL: "stall",
}

DEFAULT_OBS_FIELDS = [
    "vx", "vy", "yaw_rate", "beta", "ay", "y", "yaw",
    "slip_ratio", "slip_angle",
    "steer_applied", "throttle_applied", "brake_applied",
]


def expand_obs_fields(fields: Sequence[str]) -> List[str]:
    """Column names of the flat observation, matching the C++ layout."""
    out: List[str] = []
    for f in fields:
        base = f.rsplit(".", 1)[0] if (len(f) > 2 and f[-2] == "." and f[-1].isdigit()) else f
        if base in WHEEL_FIELDS and base == f:
            out.extend(f"{f}.{w}" for w in range(4))
        else:
            out.append(f)
    return out


@dataclasses.dataclass
class EnvConfig:
    """Everything the env needs; ``from_yaml`` loads the same keys from a file."""
    level: str = "L2"                 # plant fidelity (L1 bicycle .. L3 14-DOF)
    dt: float = 0.005                 # physics tick [s]
    action_repeat: int = 4            # control interval = dt * action_repeat
    max_steer: float = 0.5            # action scale, wheel steer [rad]
    mu: float = 1.0
    obs_fields: List[str] = dataclasses.field(
        default_factory=lambda: list(DEFAULT_OBS_FIELDS))

    # --- termination (R3); <=0 disables a check ---
    lane_y: float = 0.0
    max_lateral: float = 6.0          # [m] off-track
    max_roll: float = 0.7             # [rad] rollover
    max_beta: float = 0.7             # [rad] spin-out
    max_yaw_rate: float = 3.0         # [rad/s] spin-out
    time_limit_s: float = 20.0        # [s] truncation
    min_speed: float = 1.0            # [m/s] stall

    # --- reset randomization ---
    speed_range: Tuple[float, float] = (8.0, 25.0)   # [m/s]
    lateral_range: float = 1.0                       # +-[m]
    yaw_range: float = 0.1                           # +-[rad]

    # --- domain randomization (R5): (lo, hi) multipliers, (1, 1) = off ---
    mass_scale_range: Tuple[float, float] = (1.0, 1.0)
    mu_scale_range: Tuple[float, float] = (1.0, 1.0)
    tire_stiffness_scale_range: Tuple[float, float] = (1.0, 1.0)
    tire_mu_scale_range: Tuple[float, float] = (1.0, 1.0)
    sensor_delay_range: Tuple[float, float] = (-1.0, -1.0)   # [s], <0 = keep

    # --- solver / pool ---
    max_substep_dt: float = 0.001
    max_substeps: int = 64
    threads: int = 0                  # 0 = hardware_concurrency

    # --- reward shaping ---
    crash_penalty: float = 10.0
    speed_target: float = 15.0        # [m/s]

    @classmethod
    def from_yaml(cls, path: str) -> "EnvConfig":
        import yaml
        with open(path, "r") as fh:
            raw = yaml.safe_load(fh) or {}
        known = {f.name for f in dataclasses.fields(cls)}
        unknown = set(raw) - known
        if unknown:
            raise ValueError(f"unknown EnvConfig keys in {path}: {sorted(unknown)}")
        for key in ("speed_range", "mass_scale_range", "mu_scale_range",
                    "tire_stiffness_scale_range", "tire_mu_scale_range",
                    "sensor_delay_range"):
            if key in raw:
                raw[key] = tuple(raw[key])
        return cls(**raw)


def default_reward(obs: np.ndarray, col: Dict[str, int], action: np.ndarray,
                   prev_action: np.ndarray, cfg: EnvConfig) -> np.ndarray:
    """Lane-keeping + speed-holding, penalising jerky steer. Shape (N,)."""
    y = obs[:, col["y"]]
    vx = obs[:, col["vx"]]
    beta = obs[:, col["beta"]]
    r = 1.0 - np.abs(y) / max(cfg.max_lateral, 1e-6)
    r -= 0.05 * np.abs(vx - cfg.speed_target)
    r -= 0.2 * np.abs(beta)
    r -= 0.05 * np.abs(action[:, 0] - prev_action[:, 0])
    return r.astype(np.float32)


class _Core:
    """VecSession + buffers + episode bookkeeping, shared by both front ends."""

    def __init__(self, num_envs: int, cfg: EnvConfig,
                 reward_fn: Optional[Callable] = None, seed: Optional[int] = None):
        if num_envs < 1:
            raise ValueError("num_envs must be >= 1")
        self.cfg = cfg
        self.num_envs = num_envs
        self.reward_fn = reward_fn or default_reward

        solver = vdsim.SolverParams()
        solver.max_substep_dt = cfg.max_substep_dt
        solver.max_substeps = cfg.max_substeps

        self.vs = vdsim.make_vec_session(
            num_envs, vdsim.VehicleParams(), vdsim.TireParams(),
            level=cfg.level, nominal_dt=cfg.dt, mu=cfg.mu,
            solver=solver, threads=cfg.threads)

        self.columns = expand_obs_fields(cfg.obs_fields)
        self.vs.set_obs_fields(list(cfg.obs_fields))
        if self.vs.obs_dim != len(self.columns):
            raise RuntimeError(
                f"obs layout mismatch: core says {self.vs.obs_dim}, "
                f"python expanded {len(self.columns)}")
        self.col = {name: i for i, name in enumerate(self.columns)}

        term = vdsim.TermSpec()
        term.lane_y = cfg.lane_y
        term.max_lateral = cfg.max_lateral
        term.max_roll = cfg.max_roll
        term.max_beta = cfg.max_beta
        term.max_yaw_rate = cfg.max_yaw_rate
        term.time_limit_s = cfg.time_limit_s
        term.min_speed = cfg.min_speed
        term.check_nan = True
        self.vs.set_term_spec(term)

        # R5: one live spec per env; only the envs being reset get a new draw.
        self._dr = [vdsim.DomainRandomization() for _ in range(num_envs)]
        self._dr_active = any(
            lo != hi or lo != 1.0
            for lo, hi in (cfg.mass_scale_range, cfg.mu_scale_range,
                           cfg.tire_stiffness_scale_range, cfg.tire_mu_scale_range)
        ) or cfg.sensor_delay_range[1] >= 0.0

        self.obs = np.zeros((num_envs, self.vs.obs_dim), dtype=np.float32)
        self.term = np.zeros(num_envs, dtype=np.int32)
        self.prev_action = np.zeros((num_envs, 2), dtype=np.float32)
        self.ep_len = np.zeros(num_envs, dtype=np.int64)
        self.ep_ret = np.zeros(num_envs, dtype=np.float64)
        self._cmds = [vdsim.CmdL4() for _ in range(num_envs)]
        self.seed(seed)

    # --- R6: one independent stream per env, python side and core side ---
    def seed(self, seed: Optional[int]) -> None:
        base = 0 if seed is None else int(seed)
        seq = np.random.SeedSequence(base)
        self.rng = [np.random.default_rng(s) for s in seq.spawn(self.num_envs)]
        # Core-side noise streams; re-armed inside every SimSession.reset().
        self.vs.set_seeds([(base + i) & 0xFFFFFFFF for i in range(self.num_envs)])

    # --- R5: draw a fresh randomization for the envs about to be reset ---
    def _draw_randomization(self, idx: Sequence[int]) -> None:
        if not self._dr_active:
            return
        cfg = self.cfg
        for i in idx:
            g = self.rng[i]
            d = self._dr[i]
            d.mass_scale = float(g.uniform(*cfg.mass_scale_range))
            d.mu_scale = float(g.uniform(*cfg.mu_scale_range))
            d.tire_stiffness_scale = float(g.uniform(*cfg.tire_stiffness_scale_range))
            d.tire_mu_scale = float(g.uniform(*cfg.tire_mu_scale_range))
            lo, hi = cfg.sensor_delay_range
            d.sensor_delay_s = float(g.uniform(lo, hi)) if hi >= 0.0 else -1.0
        self.vs.set_randomizations(self._dr)

    @property
    def randomization(self) -> List["vdsim.DomainRandomization"]:
        return self._dr

    def _init_state(self, i: int):
        g = self.rng[i]
        cfg = self.cfg
        v = g.uniform(*cfg.speed_range)
        y = cfg.lane_y + g.uniform(-cfg.lateral_range, cfg.lateral_range)
        yaw = g.uniform(-cfg.yaw_range, cfg.yaw_range)
        return vdsim.make_init_state(0.0, y, yaw, v)

    def reset_all(self) -> np.ndarray:
        self._draw_randomization(range(self.num_envs))
        self.vs.reset_all([self._init_state(i) for i in range(self.num_envs)])
        self.vs.observe(self.obs)
        self.prev_action[:] = 0.0
        self.ep_len[:] = 0
        self.ep_ret[:] = 0.0
        self.term[:] = 0
        return self.obs

    def _apply(self, actions: np.ndarray) -> None:
        cfg = self.cfg
        a = np.clip(np.asarray(actions, dtype=np.float32), -1.0, 1.0)
        for i in range(self.num_envs):
            c = self._cmds[i]
            c.steer_angle_wheel = float(a[i, 0]) * cfg.max_steer
            p = float(a[i, 1])
            c.throttle = p if p > 0.0 else 0.0
            c.brake = -p if p < 0.0 else 0.0
        self.vs.set_inputs(self._cmds)

    def step(self, actions: np.ndarray):
        """-> obs, reward, terminated, truncated, codes (no auto-reset here)."""
        a = np.clip(np.asarray(actions, dtype=np.float32), -1.0, 1.0)
        self._apply(a)
        self.vs.advance(self.cfg.dt, self.cfg.action_repeat, self.obs, self.term)

        reward = self.reward_fn(self.obs, self.col, a, self.prev_action, self.cfg)
        codes = self.term
        truncated = codes == vdsim.TERM_TIME_LIMIT
        terminated = (codes != vdsim.TERM_NONE) & ~truncated
        reward = np.where(terminated, reward - self.cfg.crash_penalty, reward)
        reward = reward.astype(np.float32)

        self.prev_action[:] = a
        self.ep_len += 1
        self.ep_ret += reward
        return self.obs, reward, terminated, truncated, codes

    def reset_done(self, done: np.ndarray) -> None:
        idx = np.flatnonzero(done)
        if idx.size == 0:
            return
        self._draw_randomization([int(i) for i in idx])
        self.vs.reset_subset([int(i) for i in idx],
                             [self._init_state(int(i)) for i in idx])
        self.vs.observe(self.obs)
        self.prev_action[idx] = 0.0
        self.ep_len[idx] = 0
        self.ep_ret[idx] = 0.0

    def close(self) -> None:
        self.vs = None


def _spaces(obs_dim: int):
    import gymnasium as gym
    obs_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,),
                               dtype=np.float32)
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
    return obs_space, act_space


class VDSimVecEnv:
    """gymnasium VectorEnv over one VecSession (auto-resets on done)."""

    metadata = {"render_modes": []}

    def __init__(self, num_envs: int, cfg: Optional[EnvConfig] = None,
                 reward_fn: Optional[Callable] = None, seed: Optional[int] = None):
        import gymnasium as gym
        self.core = _Core(num_envs, cfg or EnvConfig(), reward_fn, seed)
        self.num_envs = num_envs
        self.single_observation_space, self.single_action_space = _spaces(
            self.core.vs.obs_dim)
        self.observation_space = gym.vector.utils.batch_space(
            self.single_observation_space, num_envs)
        self.action_space = gym.vector.utils.batch_space(
            self.single_action_space, num_envs)
        self.closed = False

    @property
    def obs_columns(self) -> List[str]:
        return self.core.columns

    def reset(self, *, seed: Optional[int] = None, options=None):
        if seed is not None:
            self.core.seed(seed)
        obs = self.core.reset_all().copy()
        return obs, {}

    def step(self, actions):
        obs, rew, terminated, truncated, codes = self.core.step(actions)
        done = terminated | truncated
        info: Dict[str, object] = {}
        if done.any():
            final_obs = np.empty(self.num_envs, dtype=object)
            final_obs[:] = None
            reasons = np.empty(self.num_envs, dtype=object)
            reasons[:] = None
            for i in np.flatnonzero(done):
                final_obs[i] = obs[i].copy()
                reasons[i] = TERM_NAMES.get(int(codes[i]), "unknown")
            info["final_observation"] = final_obs
            info["_final_observation"] = done.copy()
            info["termination_reason"] = reasons
            self.core.reset_done(done)
        return obs.copy(), rew.copy(), terminated.copy(), truncated.copy(), info

    def close(self):
        if not self.closed:
            self.core.close()
            self.closed = True

    def __len__(self):
        return self.num_envs


class VDSimEnv:
    """Single-env gymnasium API (one VecSession of size 1)."""

    metadata = {"render_modes": []}

    def __init__(self, cfg: Optional[EnvConfig] = None,
                 reward_fn: Optional[Callable] = None, seed: Optional[int] = None):
        self.core = _Core(1, cfg or EnvConfig(), reward_fn, seed)
        self.observation_space, self.action_space = _spaces(self.core.vs.obs_dim)

    @property
    def obs_columns(self) -> List[str]:
        return self.core.columns

    def reset(self, *, seed: Optional[int] = None, options=None):
        if seed is not None:
            self.core.seed(seed)
        return self.core.reset_all()[0].copy(), {}

    def step(self, action):
        a = np.asarray(action, dtype=np.float32).reshape(1, 2)
        obs, rew, terminated, truncated, codes = self.core.step(a)
        info = {"termination_reason": TERM_NAMES.get(int(codes[0]), "unknown")}
        return (obs[0].copy(), float(rew[0]), bool(terminated[0]),
                bool(truncated[0]), info)

    def close(self):
        self.core.close()


def make_sb3_vec_env(num_envs: int, cfg: Optional[EnvConfig] = None,
                     reward_fn: Optional[Callable] = None,
                     seed: Optional[int] = None):
    """A stable-baselines3 VecEnv (its own protocol, not gymnasium's)."""
    from stable_baselines3.common.vec_env.base_vec_env import VecEnv

    class _SB3VecEnv(VecEnv):
        def __init__(self):
            self.core = _Core(num_envs, cfg or EnvConfig(), reward_fn, seed)
            obs_space, act_space = _spaces(self.core.vs.obs_dim)
            super().__init__(num_envs, obs_space, act_space)
            self._actions = np.zeros((num_envs, 2), dtype=np.float32)

        def reset(self):
            return self.core.reset_all().copy()

        def step_async(self, actions):
            self._actions = actions

        def step_wait(self):
            obs, rew, terminated, truncated, codes = self.core.step(self._actions)
            done = terminated | truncated
            infos = [{} for _ in range(num_envs)]
            for i in np.flatnonzero(done):
                infos[i]["terminal_observation"] = obs[i].copy()
                infos[i]["TimeLimit.truncated"] = bool(truncated[i])
                infos[i]["termination_reason"] = TERM_NAMES.get(int(codes[i]), "unknown")
            if done.any():
                self.core.reset_done(done)
            return obs.copy(), rew.copy(), done.copy(), infos

        def close(self):
            self.core.close()

        def get_attr(self, attr_name, indices=None):
            return [getattr(self, attr_name)] * self._n(indices)

        def set_attr(self, attr_name, value, indices=None):
            setattr(self, attr_name, value)

        def env_method(self, method_name, *args, indices=None, **kwargs):
            raise NotImplementedError("VDSim VecEnv has no per-env python objects")

        def env_is_wrapped(self, wrapper_class, indices=None):
            return [False] * self._n(indices)

        def _n(self, indices):
            if indices is None:
                return num_envs
            if isinstance(indices, int):
                return 1
            return len(indices)

    return _SB3VecEnv()
