"""vdsim_rl - RL environment layer over the C++ VecSession.

One VecSession owns N independent plants and a persistent worker pool; a whole
vector step is a single GIL-released call (``advance``) that ticks every env,
evaluates the termination conditions and writes the flat float32 observation
rows in place.  Python never touches per-tick data.

    from vdsim_rl import EnvConfig, VDSimVecEnv
    env = VDSimVecEnv(64, EnvConfig(vehicle="generic_sedan", tire="generic_pacejka"))
    obs, info = env.reset(seed=0)
    obs, rew, term, trunc, info = env.step(env.action_space.sample())

Action (per env, Box(-1, 1, shape=(2,))):
    a[0] = steer, scaled to +-cfg.max_steer [rad] at the wheel
    a[1] = pedal, >0 -> throttle, <0 -> brake

Observation: cfg.obs_fields, flattened left to right; a bare per-wheel name
expands to four columns in FL, FR, RL, RR order.  ``az`` is the body vertical
acceleration [m/s^2] straight from the plant (gravity excluded, 0 at rest; not
an accelerometer's specific force), 0 on the planar levels.
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import warnings
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

from vdsim_guard import load_core

vdsim = load_core()

__all__ = [
    "EnvConfig", "VDSimVecEnv", "VDSimEnv", "make_sb3_vec_env",
    "expand_obs_fields", "TERM_NAMES", "default_reward", "load_vehicle_preset",
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


#: Keys a vehicle preset YAML may carry that VehicleParams does not bind, each
#: with the consumer that reads it.  Anything else unbound is an error.
VEHICLE_SIDECAR_KEYS = frozenset({
    "tire_yaml",        # vdsim_plant._load_tire_setup_for_vehicle, load_vehicle_preset
    "body_lwh_m",       # trace 0.3 geometry (vdsim_render3d)
    "wheel_width_m",    # trace 0.3 geometry (vdsim_render3d)
})

#: Keys core/src/params.cpp parses that the bindings expose under another shape,
#: so they are not attribute names of the bound struct.
VEHICLE_PARSED_UNBOUND = frozenset({
    "inertia_diag",     # parsed as Vector3, bound as ixx / iyy / izz
    "powertrain",       # nested block (parse_powertrain), not bound
})
TIRE_PARSED_UNBOUND = frozenset({
    "belt",             # nested block, not bound
})

#: Keys a preset must state.  VehicleParams/TireParams.from_yaml keep the C++
#: generic default for a missing key, so a preset that omits one of these would
#: silently train a different car.
VEHICLE_REQUIRED_KEYS = ("mass", "wheelbase", "cg_to_front", "cg_to_rear",
                         "track_front", "track_rear", "cg_height",
                         "wheel_radius_nominal")
TIRE_REQUIRED_KEYS = ("mu_nominal", "Fz_nominal")

BUILTIN_WARNING = "vehicle=None: C++ built-in generic parameters"


def _bound_fields(obj) -> List[str]:
    """Data members a pybind11 params struct exposes (methods excluded)."""
    return sorted(n for n in dir(obj)
                  if not n.startswith("_") and not callable(getattr(obj, n)))


def _check_keys(raw: dict, proto, extra, required, where: str) -> None:
    """Reject keys the C++ parser would ignore and required keys that are missing.

    The allowed set is read from the bound struct itself (the YAML schema rule
    in core/src/params.cpp maps top-level keys 1:1 to member names), so this is
    not a second copy of the parser's key list; ``extra`` holds only the
    exceptions (sidecar keys and keys bound under another shape).  Nested maps
    (e.g. ``lugre``) are checked against the nested struct the same way.
    """
    allowed = set(_bound_fields(proto)) | set(extra)
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(f"{where}: keys the parser does not know: {unknown}")
    missing = [k for k in required if k not in raw]
    if missing:
        raise ValueError(f"{where}: required keys missing: {missing}")
    for key, val in raw.items():
        if isinstance(val, dict) and key not in extra:
            sub_unknown = sorted(set(val) - set(_bound_fields(getattr(proto, key))))
            if sub_unknown:
                raise ValueError(f"{where}: keys under '{key}' the parser does "
                                 f"not know: {sub_unknown}")


def _digest_value(v):
    if isinstance(v, (bool, int, float, str)) or v is None:
        return v
    if isinstance(v, (list, tuple)):
        return [_digest_value(x) for x in v]
    if isinstance(v, np.ndarray):
        return v.tolist()
    if hasattr(type(v), "__members__"):     # pybind11 enum
        return str(v)
    fields = _bound_fields(v)
    if fields:
        return {f: _digest_value(getattr(v, f)) for f in fields}
    return repr(v)


def _params_hash(vp, tp, tir_file: Optional[Path]) -> str:
    """sha256 over the parameters as the core receives them (+ the .tir bytes)."""
    blob = {"vehicle": _digest_value(vp), "tire": _digest_value(tp),
            "tir_sha256": (hashlib.sha256(tir_file.read_bytes()).hexdigest()
                           if tir_file is not None else None)}
    text = json.dumps(blob, sort_keys=True, separators=(",", ":"), default=repr)
    return hashlib.sha256(text.encode()).hexdigest()


def _read_yaml(path: Path) -> dict:
    import yaml
    raw = yaml.safe_load(path.read_text()) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: top level must be a mapping")
    return raw


#: Environment variable naming a preset root outside the repository, for cars
#: whose parameters must not be published.  Same layout as ``configs/``:
#: ``<root>/vehicles/<stem>.yaml``, ``<root>/parts/tire/<stem>.yaml``.  The
#: search itself is ``vdsim_plant.find_preset`` (one rule for plant and RL).
PRIVATE_ROOT_ENV = "VDSIM_PRIVATE_CONFIGS"


def _preset_source(path: Path) -> str:
    """``"public"`` for a file under the repo's ``configs/``, else ``"private"``."""
    from vdsim_plant import _conf_root
    try:
        path.resolve().relative_to(_conf_root().resolve())
        return "public"
    except ValueError:
        return "private"


def _resolve_preset_path(rel: str, explicit: Optional[str], what: str) -> Path:
    """Locate one preset file, in a fixed order.

    ``<what>_file`` (an absolute path) wins over ``$VDSIM_PRIVATE_CONFIGS/<rel>``,
    which wins over the repository's ``configs/<rel>``.  ``rel`` is the path
    inside a preset root (``vehicles/x.yaml``, ``parts/tire/x.yaml``).  A
    relative ``<what>_file`` is refused: it would resolve against the working
    directory, so one declaration would load different cars depending on where
    the run was started.
    """
    from vdsim_plant import find_preset

    if explicit:
        p = Path(explicit).expanduser()
        if not p.is_absolute():
            raise ValueError(f"{what}_file must be an absolute path: {explicit!r}")
        if not p.is_file():
            raise FileNotFoundError(f"{what}_file not found: {p}")
        return p
    return find_preset(rel, what)


def load_vehicle_preset(vehicle: Optional[str], tire: Optional[str], *,
                        vehicle_file: Optional[str] = None,
                        tire_file: Optional[str] = None):
    """Build VehicleParams/TireParams from preset names; reads files, writes none.

    ``vehicle`` is a stem under ``configs/vehicles/``, ``tire`` a stem under
    ``configs/parts/tire/``; ``vehicle_file`` / ``tire_file`` give an absolute
    path instead (search order in ``_resolve_preset_path``).  ``tire=None`` with
    a vehicle takes that vehicle's ``tire_yaml``, resolved against the vehicle
    file's own root first, so a private car can carry a private tyre.
    ``vehicle=None`` keeps the C++ built-in generic car and warns.  Unknown keys
    or missing required keys raise ``ValueError``.

    Returns ``(VehicleParams, TireParams, provenance)``; provenance is
    ``{"vehicle", "tire", "param_hash", "source"}``.  It carries names only --
    no path, so a training log cannot leak a private directory layout.
    """
    from vdsim_plant import _resolve_tir_path

    used: List[Path] = []
    if vehicle is None:
        if vehicle_file:
            raise ValueError("vehicle_file needs vehicle= as the recorded name")
        warnings.warn(BUILTIN_WARNING, UserWarning, stacklevel=3)
        vp = vdsim.VehicleParams()
        vp_path = None
        tire_rel = None
    else:
        vp_path = _resolve_preset_path(f"vehicles/{Path(vehicle).stem}.yaml",
                                       vehicle_file, "vehicle")
        used.append(vp_path)
        raw = _read_yaml(vp_path)
        _check_keys(raw, vdsim.VehicleParams(),
                    VEHICLE_SIDECAR_KEYS | VEHICLE_PARSED_UNBOUND,
                    VEHICLE_REQUIRED_KEYS, str(vp_path))
        vp = vdsim.VehicleParams.from_yaml(str(vp_path))
        tire_rel = raw.get("tire_yaml")

    if tire is not None:
        tp_path = _resolve_preset_path(f"parts/tire/{Path(tire).stem}.yaml",
                                       tire_file, "tire")
    elif tire_rel:
        sibling = vp_path.parent.parent / tire_rel
        tp_path = (sibling if sibling.is_file()
                   else _resolve_preset_path(tire_rel, None, "tire"))
    elif vehicle is not None:
        raise ValueError(f"vehicle '{vehicle}' names no tire_yaml; pass tire=")
    else:
        if tire_file:
            raise ValueError("tire_file needs tire= as the recorded name")
        tp_path = None

    tir_file = None
    if tp_path is None:
        tp = vdsim.TireParams()
    else:
        used.append(tp_path)
        raw_tp = _read_yaml(tp_path)
        if "schema" in raw_tp and "body" in raw_tp:
            raise ValueError(f"{tp_path}: catalog part (schema/body); vdsim_rl reads "
                             f"flat TireParams files only")
        _check_keys(raw_tp, vdsim.TireParams(), TIRE_PARSED_UNBOUND,
                    TIRE_REQUIRED_KEYS, str(tp_path))
        tp = vdsim.TireParams.from_yaml(str(tp_path))
        if tp.tir_path:
            tp.tir_path = _resolve_tir_path(tp_path, tp.tir_path)
            tir_file = Path(tp.tir_path)
            if not tir_file.is_file():
                raise FileNotFoundError(f"{tp_path}: tir_path not found: {tir_file}")

    provenance = {
        "vehicle": None if vehicle is None else Path(vehicle).stem,
        "tire": None if tp_path is None else tp_path.stem,
        "param_hash": _params_hash(vp, tp, tir_file),
        "source": ("private" if any(_preset_source(q) == "private" for q in used)
                   else "public"),
    }
    return vp, tp, provenance


ACTION_MODES = ("pedal", "accel")

#: Levels whose dynamics carry a vertical ride (road_dz reaches the body).  The
#: others are planar: a rough, ISO 8608 or PSD road only moves their contact
#: heights, which they never read, so such a road is refused, not ignored.
VERTICAL_LEVELS = ("L3", "L4")
#: Levels that read no contact at all (kinematic model): grade/bank are refused.
CONTACT_FREE_LEVELS = ("K", "L0")
#: Keys of EnvConfig.road_psd -> make_vec_session keyword (psd_<key>).
ROAD_PSD_KEYS = ("gd_n0", "waviness", "n_break", "waviness_high",
                 "n_min", "n_max", "n", "gd")


def _check_road(cfg: "EnvConfig") -> None:
    """Refuse a road the core would drop without a word.

    The core builds one ground per session and takes the first match of
    PSD > ISO 8608 > rough > grade/bank (python/bindings.cpp
    build_ground_session), so two kinds at once would silently lose one.
    """
    if not -1 <= cfg.iso_class <= 7:
        raise ValueError(f"iso_class must be -1 (off) or 0 (A) .. 7 (H), got {cfg.iso_class}")
    if cfg.rough_amp < 0.0:
        raise ValueError(f"rough_amp must be >= 0 [m], got {cfg.rough_amp}")
    if not cfg.rough_wavelength > 0.0:
        raise ValueError(f"rough_wavelength must be > 0 [m], got {cfg.rough_wavelength}")
    if cfg.road_psd is not None:
        if not isinstance(cfg.road_psd, dict):
            raise ValueError("road_psd must be a mapping")
        unknown = sorted(set(cfg.road_psd) - set(ROAD_PSD_KEYS))
        if unknown:
            raise ValueError(f"road_psd: unknown keys {unknown}; allowed {list(ROAD_PSD_KEYS)}")
        table = "n" in cfg.road_psd or "gd" in cfg.road_psd
        if table == ("gd_n0" in cfg.road_psd):
            raise ValueError("road_psd needs either gd_n0 (analytic) or n + gd (table), not both")
        if table and len(cfg.road_psd.get("n", ())) != len(cfg.road_psd.get("gd", ())):
            raise ValueError("road_psd: n and gd must have the same length")
    vertical = [name for name, on in (("road_psd", cfg.road_psd is not None),
                                      ("iso_class", cfg.iso_class >= 0),
                                      ("rough_amp", cfg.rough_amp > 0.0)) if on]
    kinds = vertical + (["grade/bank"] if (cfg.grade != 0.0 or cfg.bank != 0.0) else [])
    if len(kinds) > 1:
        raise ValueError(f"one road kind per env, got {kinds}")
    if vertical and cfg.level not in VERTICAL_LEVELS:
        raise ValueError(f"{vertical[0]} excites the vertical ride, which level "
                         f"{cfg.level!r} does not model; use one of {VERTICAL_LEVELS}")
    if kinds == ["grade/bank"] and cfg.level in CONTACT_FREE_LEVELS:
        raise ValueError(f"grade/bank: level {cfg.level!r} reads no road contact")


def _actuator_record(spec, ap) -> Optional[dict]:
    """Provenance of the actuator: preset id and a hash of what the core gets."""
    if spec is None:
        return None
    blob = json.dumps(_digest_value(ap), sort_keys=True, separators=(",", ":"),
                      default=repr)
    preset = spec if isinstance(spec, str) else (
        spec.get("preset") if isinstance(spec, dict) else None)
    return {"preset": preset,
            "param_hash": hashlib.sha256(blob.encode()).hexdigest()}


@dataclasses.dataclass
class EnvConfig:
    """Everything the env needs; ``from_yaml`` loads the same keys from a file."""
    level: str = "L2"                 # plant fidelity (L1 bicycle .. L3 14-DOF)
    # car: preset stems (configs/vehicles, configs/parts/tire); None = C++ built-in
    vehicle: Optional[str] = None
    tire: Optional[str] = None
    # absolute paths that win over both preset roots, for a car kept out of
    # the repository; the stem above is still what gets recorded as its name
    vehicle_file: Optional[str] = None
    tire_file: Optional[str] = None
    dt: float = 0.005                 # physics tick [s]
    action_repeat: int = 4            # control interval = dt * action_repeat
    max_steer: float = 0.5            # action scale, wheel steer [rad]
    # action[1]: "pedal" = throttle (+) / brake (-) fraction, CmdL4.
    # "accel" = longitudinal acceleration target action[1] * max_accel [m/s^2],
    # CmdL5; each env's own plant-side PI cascade turns it into pedals, so the
    # realised a_x lags the target and saturates at the pedal limits.
    action_mode: str = "pedal"
    max_accel: float = 4.0            # action scale in accel mode [m/s^2]
    mu: float = 1.0
    # actuator between the action and the plant: None = identity, a catalog id
    # ("actuator.generic_sedan") or a spec dict (vdsim_plant.build_actuator_params)
    actuator: Optional[Union[str, dict]] = None

    # --- road (one kind per env; env i uses ground_seed + i) ---
    grade: float = 0.0                # [rad] uphill toward +x
    bank: float = 0.0                 # [rad] up toward +y
    rough_amp: float = 0.0            # [m] two-tone profile, 0 = off (L3/L4)
    rough_wavelength: float = 4.0     # [m]
    iso_class: int = -1               # ISO 8608 class 0=A .. 7=H, -1 = off (L3/L4)
    # general PSD road (L3/L4): {gd_n0, waviness, n_break, waviness_high,
    # n_min, n_max} or a measured table {n, gd, n_min, n_max}; None = off
    road_psd: Optional[dict] = None
    ground_seed: int = 1
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

    def __post_init__(self):
        """Reject an unknown action mode, a non-positive accel scale or a road
        the chosen level would ignore."""
        if self.action_mode not in ACTION_MODES:
            raise ValueError(f"action_mode must be one of {ACTION_MODES}, "
                             f"got {self.action_mode!r}")
        if not self.max_accel > 0.0:
            raise ValueError(f"max_accel must be > 0 [m/s^2], got {self.max_accel}")
        _check_road(self)
        if self.level not in VERTICAL_LEVELS and "az" in self.obs_fields:
            warnings.warn(f"obs field 'az': level {self.level!r} has no heave, the "
                          f"column is always 0; use one of {VERTICAL_LEVELS}",
                          UserWarning, stacklevel=3)

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
                 reward_fn: Optional[Callable] = None, seed: Optional[int] = None,
                 monitor=None, monitor_opts: Optional[dict] = None):
        if num_envs < 1:
            raise ValueError("num_envs must be >= 1")
        self.cfg = cfg
        self.num_envs = num_envs
        self.reward_fn = reward_fn or default_reward

        solver = vdsim.SolverParams()
        solver.max_substep_dt = cfg.max_substep_dt
        solver.max_substeps = cfg.max_substeps

        # Parsed once here in the parent; every env shares these values and no
        # file is written, so the Q10-c shared-cache race has no path in.
        vp, tp, self.provenance = load_vehicle_preset(
            cfg.vehicle, cfg.tire,
            vehicle_file=cfg.vehicle_file, tire_file=cfg.tire_file)
        from vdsim_plant import build_actuator_params
        ap = build_actuator_params(cfg.actuator)
        if cfg.actuator is not None:
            self.provenance["actuator"] = _actuator_record(cfg.actuator, ap)
        psd = {f"psd_{k}": v for k, v in (cfg.road_psd or {}).items()}
        self.vs = vdsim.make_vec_session(
            num_envs, vp, tp,
            level=cfg.level, nominal_dt=cfg.dt, mu=cfg.mu, actuator=ap,
            solver=solver, grade=cfg.grade, bank=cfg.bank,
            rough_amp=cfg.rough_amp, rough_wavelength=cfg.rough_wavelength,
            iso_class=cfg.iso_class, threads=cfg.threads,
            ground_seed=cfg.ground_seed, **psd)

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
        cmd_type = vdsim.CmdL5 if cfg.action_mode == "accel" else vdsim.CmdL4
        self._cmds = [cmd_type() for _ in range(num_envs)]
        self.seed(seed)

        # Live monitor is opt-in; `monitor is None` is the OFF path (nothing
        # imported, no socket, no thread, no extra read of the pool).
        self.monitor = None
        self._mon_t = 0.0       # simulated time per env since construction [s]
        if monitor is not None and monitor is not False:
            import vdsim_livemonitor
            self.monitor = vdsim_livemonitor.open_monitor(
                monitor, mode="rl", n_vehicles=num_envs,
                vehicle=vdsim_livemonitor.vehicle_geometry(vp),
                **(monitor_opts or {}))

    def _monitor_publish(self) -> None:
        """Send the drawn envs' poses; the pool is read only when a send is
        due (at most the monitor rate per wall second)."""
        att = self.monitor
        if att.due():
            from vdsim_livemonitor import state_from_output
            outs = self.vs.outputs()
            dt = self.cfg.dt * self.cfg.action_repeat
            att.publish([state_from_output(outs[i], float(self.ep_len[i]) * dt)
                         for i in range(att.n_shown)], self._mon_t)

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
        if self.monitor is not None:
            self.monitor.reset(range(self.num_envs))
            self._monitor_publish()
        return self.obs

    def _apply(self, actions: np.ndarray) -> None:
        cfg = self.cfg
        a = np.clip(np.asarray(actions, dtype=np.float32), -1.0, 1.0)
        if cfg.action_mode == "accel":
            for i in range(self.num_envs):
                c = self._cmds[i]
                c.steer_angle_wheel = float(a[i, 0]) * cfg.max_steer
                c.ax_target = float(a[i, 1]) * cfg.max_accel
            self.vs.set_inputs(self._cmds)
            return
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
        if self.monitor is not None:
            self._mon_t += self.cfg.dt * self.cfg.action_repeat
            self._monitor_publish()
            self.monitor.hold_pace(self._mon_t)
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
        if self.monitor is not None:
            self.monitor.reset(idx)

    def close(self) -> None:
        if self.monitor is not None:
            self.monitor.close()
            self.monitor = None
        self.vs = None


def _spaces(obs_dim: int):
    import gymnasium as gym
    obs_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,),
                               dtype=np.float32)
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
    return obs_space, act_space


class VDSimVecEnv:
    """gymnasium VectorEnv over one VecSession (auto-resets on done).

    ``monitor`` (off by default) draws the envs in the browser live monitor:
    ``True`` = port 8790, an ``int`` = that port, or a
    ``vdsim_livemonitor.LiveMonitor``. The first ``monitor_max_vehicles`` envs
    are drawn, each in its own colour; an env that resets has its trail
    cleared. ``monitor_layout="grid"`` shifts env i by
    ``i * monitor_grid_spacing`` [m] to the left on the page (display only).
    Poses are read from the pool only when a send is due (at most 30 per wall
    second). ``monitor_pace`` / ``monitor_wait_s`` as for
    ``vdsim_plant.VDSimPlant``; ``close()`` marks the run finished.
    """

    metadata = {"render_modes": []}

    def __init__(self, num_envs: int, cfg: Optional[EnvConfig] = None,
                 reward_fn: Optional[Callable] = None, seed: Optional[int] = None,
                 monitor=None, monitor_max_vehicles: int = 16,
                 monitor_layout: str = "overlay", monitor_grid_spacing: float = 8.0,
                 monitor_pace: Optional[float] = None, monitor_wait_s: float = 0.0):
        import gymnasium as gym
        self.core = _Core(num_envs, cfg or EnvConfig(), reward_fn, seed,
                          monitor=monitor,
                          monitor_opts={"max_vehicles": monitor_max_vehicles,
                                        "layout": monitor_layout,
                                        "spacing": monitor_grid_spacing,
                                        "pace": monitor_pace, "wait_s": monitor_wait_s})
        self.num_envs = num_envs
        self.single_observation_space, self.single_action_space = _spaces(
            self.core.vs.obs_dim)
        self.observation_space = gym.vector.utils.batch_space(
            self.single_observation_space, num_envs)
        self.action_space = gym.vector.utils.batch_space(
            self.single_action_space, num_envs)
        self.metadata = {**type(self).metadata, "vdsim": dict(self.core.provenance)}
        self.closed = False

    @property
    def obs_columns(self) -> List[str]:
        return self.core.columns

    def reset(self, *, seed: Optional[int] = None, options=None):
        if seed is not None:
            self.core.seed(seed)
        obs = self.core.reset_all().copy()
        return obs, dict(self.core.provenance)

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

    @property
    def monitor(self):
        """The ``vdsim_livemonitor.MonitorAttachment``, or None."""
        return self.core.monitor

    def close(self):
        if not self.closed:
            self.core.close()
            self.closed = True

    def __len__(self):
        return self.num_envs


class VDSimEnv:
    """Single-env gymnasium API (one VecSession of size 1).

    ``monitor``, ``monitor_pace``, ``monitor_wait_s``: see :class:`VDSimVecEnv`.
    """

    metadata = {"render_modes": []}

    def __init__(self, cfg: Optional[EnvConfig] = None,
                 reward_fn: Optional[Callable] = None, seed: Optional[int] = None,
                 monitor=None, monitor_pace: Optional[float] = None,
                 monitor_wait_s: float = 0.0):
        self.core = _Core(1, cfg or EnvConfig(), reward_fn, seed, monitor=monitor,
                          monitor_opts={"pace": monitor_pace, "wait_s": monitor_wait_s})
        self.observation_space, self.action_space = _spaces(self.core.vs.obs_dim)
        self.metadata = {**type(self).metadata, "vdsim": dict(self.core.provenance)}

    @property
    def obs_columns(self) -> List[str]:
        return self.core.columns

    def reset(self, *, seed: Optional[int] = None, options=None):
        if seed is not None:
            self.core.seed(seed)
        return self.core.reset_all()[0].copy(), dict(self.core.provenance)

    def step(self, action):
        a = np.asarray(action, dtype=np.float32).reshape(1, 2)
        obs, rew, terminated, truncated, codes = self.core.step(a)
        info = {"termination_reason": TERM_NAMES.get(int(codes[0]), "unknown")}
        return (obs[0].copy(), float(rew[0]), bool(terminated[0]),
                bool(truncated[0]), info)

    @property
    def monitor(self):
        """The ``vdsim_livemonitor.MonitorAttachment``, or None."""
        return self.core.monitor

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
            self.metadata = {**getattr(self, "metadata", {}),
                             "vdsim": dict(self.core.provenance)}
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
