# R2/R3/R4 verification: gym adapter, termination codes, flat obs buffer,
# and (Q23) the vehicle preset input.
# Paths are repo-relative; VDSIM_BUILD_PY overrides the built module dir.
import json, os, sys, time, warnings
from pathlib import Path
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, os.environ.get("VDSIM_BUILD_PY", str(REPO / "build" / "python")))
sys.path.insert(0, str(REPO / "python"))
import numpy as np
import yaml
import vdsim
import vdsim_rl
from vdsim_rl import (EnvConfig, VDSimVecEnv, VDSimEnv,
                      expand_obs_fields, TERM_NAMES, load_vehicle_preset)

res = {}
G = 9.81
IONIQ5 = yaml.safe_load((REPO / "configs/vehicles/ioniq5_awd.yaml").read_text())


def plant_weight_kg(cfg, n=2):
    """Total vertical tyre load / g of the plant the env actually built [kg].

    Coasting straight at 10 m/s for 1 s; reads the per-wheel ``fz`` columns,
    so it measures the core's mass, not the Python-side params object.
    """
    c = EnvConfig(**{**cfg.__dict__, "level": "L2",
                     "obs_fields": ["vx", "fz"], "speed_range": (10.0, 10.0),
                     "lateral_range": 0.0, "yaw_range": 0.0,
                     "time_limit_s": -1.0, "min_speed": -1.0})
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        e = VDSimVecEnv(n, c, seed=0,
                        reward_fn=lambda o, *_: np.zeros(len(o), np.float32))
    e.reset(seed=0)
    for _ in range(50):
        o, *_ = e.step(np.zeros((n, 2), dtype=np.float32))
    cols = [e.core.col[f"fz.{w}"] for w in range(4)]
    return o[:, cols].sum(axis=1) / G


# ---- Q23-1: vehicle="ioniq5_awd" -> the plant carries the YAML car ----
vp, tp, prov = load_vehicle_preset("ioniq5_awd", "ioniq5_pac2002")
for key in ("mass", "wheelbase", "cg_to_front", "cg_to_rear", "cg_height",
            "track_front", "wheel_radius_nominal"):
    assert abs(getattr(vp, key) - IONIQ5[key]) < 1e-9, key
w = plant_weight_kg(EnvConfig(vehicle="ioniq5_awd", tire="ioniq5_pac2002"))
print(f"Q23-1 ioniq5 plant weight {w.round(1)} kg vs YAML mass {IONIQ5['mass']}")
assert np.all(np.abs(w / IONIQ5["mass"] - 1.0) < 0.01), w
res["q23_ioniq5_weight_kg"] = w.tolist()

# ---- Q23-2: the shipped RL YAMLs select Ioniq5 ----
for name in ("default_env.yaml", "fast_env.yaml"):
    c = EnvConfig.from_yaml(str(REPO / "configs/rl" / name))
    assert (c.vehicle, c.tire) == ("ioniq5_awd", "ioniq5_pac2002"), name
    e = VDSimVecEnv(2, c, seed=0)
    _, info = e.reset(seed=0)
    assert info["vehicle"] == "ioniq5_awd" and info["tire"] == "ioniq5_pac2002", info
    assert len(info["param_hash"]) == 64 and e.metadata["vdsim"] == info
    w = plant_weight_kg(EnvConfig(**{**c.__dict__, "mass_scale_range": (1.0, 1.0)}))
    assert np.all(np.abs(w / IONIQ5["mass"] - 1.0) < 0.01), (name, w)
    print(f"Q23-2 {name}: info {info['vehicle']}/{info['tire']} "
          f"hash {info['param_hash'][:12]}, weight {w.round(1)} kg")
res["q23_yaml_info"] = info

# ---- Q23-3: vehicle=None still works but says so ----
with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter("always")
    e = VDSimVecEnv(1, EnvConfig(), seed=0)
msgs = [str(x.message) for x in caught if issubclass(x.category, UserWarning)]
assert vdsim_rl.BUILTIN_WARNING in msgs, msgs
assert e.reset(seed=0)[1]["vehicle"] is None
print(f"Q23-3 vehicle=None warns: {msgs}")

# ---- Q23-4: domain randomization scales the Ioniq5 baseline ----
w = plant_weight_kg(EnvConfig(vehicle="ioniq5_awd", tire="ioniq5_pac2002",
                              mass_scale_range=(1.1, 1.1)))
print(f"Q23-4 mass_scale 1.1 -> {w.round(1)} kg (expect {1.1 * IONIQ5['mass']:.1f})")
assert np.all(np.abs(w / (1.1 * IONIQ5["mass"]) - 1.0) < 0.01), w

# ---- Q23-5: loading writes nothing ----
def tree(root):
    return {str(q): (q.stat().st_size, q.stat().st_mtime_ns)
            for q in Path(root).rglob("*") if q.is_file()}
import builtins
from vdsim_plant import _conf_root
before = (tree(_conf_root()), tree(os.getcwd()) if Path.cwd() != REPO else {})
opened_for_write = []
_real_open = builtins.open
def _spy_open(file, mode="r", *a, **k):
    if any(m in mode for m in "wax+"):
        opened_for_write.append(str(file))
    return _real_open(file, mode, *a, **k)
builtins.open = _spy_open
try:
    e = VDSimVecEnv(8, EnvConfig.from_yaml(str(REPO / "configs/rl/default_env.yaml")), seed=1)
    e.reset(seed=1)
finally:
    builtins.open = _real_open
after = (tree(_conf_root()), tree(os.getcwd()) if Path.cwd() != REPO else {})
assert not opened_for_write, opened_for_write
assert before == after, "files changed while loading the preset"
print(f"Q23-5 files opened for write: {len(opened_for_write)}, "
      f"config tree unchanged ({len(before[0])} files)")

# ---- Q23-6: a preset key the parser ignores, or a missing required key, raises ----
import tempfile
from vdsim_rl import _check_keys, VEHICLE_SIDECAR_KEYS, VEHICLE_PARSED_UNBOUND, \
    VEHICLE_REQUIRED_KEYS
extra = VEHICLE_SIDECAR_KEYS | VEHICLE_PARSED_UNBOUND
for bad, want in (({**IONIQ5, "masss": 1.0}, "does not know"),
                  ({k: v for k, v in IONIQ5.items() if k != "cg_height"}, "missing")):
    try:
        _check_keys(bad, vdsim.VehicleParams(), extra, VEHICLE_REQUIRED_KEYS, "probe")
    except ValueError as exc:
        assert want in str(exc), exc
        print(f"Q23-6 rejected: {exc}")
    else:
        raise AssertionError(f"not rejected: {want}")
# flat tyre presets load (no false rejection); catalog parts (schema/body) are
# refused -- read flat, every coefficient would silently be the C++ default.
for tyre in sorted((REPO / "configs/parts/tire").glob("*.yaml")):
    catalog = "schema" in (yaml.safe_load(tyre.read_text()) or {})
    try:
        load_vehicle_preset("ioniq5_awd", tyre.stem)
        assert not catalog, f"catalog part {tyre.name} was accepted"
    except ValueError as exc:
        assert catalog and "catalog part" in str(exc), exc
    print(f"Q23-6 tyre {tyre.stem:20s} {'refused (catalog part)' if catalog else 'loads'}")
print("Q23 vehicle preset checks passed")

# ---- R4a: YAML declaration -> layout, and the buffer is written in place ----
cfg = EnvConfig.from_yaml(str(REPO / "configs/rl/default_env.yaml"))
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

# The SB3 PPO smoke (torch) lives in smoke_sb3_ppo.py, outside ctest: this
# file is the ctest target rl_env and needs gymnasium but not torch/SB3.

json.dump(res, open("/tmp/rl_smoke.json", "w"), indent=1, default=str)
print("ALL CHECKS PASSED")
