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
GENERIC = yaml.safe_load((REPO / "configs/vehicles/generic_sedan.yaml").read_text())

# A synthetic probe car, injected the way a preset with non-public specs is
# injected ($VDSIM_PRIVATE_CONFIGS -> vehicles/<stem>.yaml).  Every shipped car
# carries the C++ default values, so "the YAML reached the plant" checked
# against one of them would be vacuous; Q23-1 asserts the fixture actually
# differs.  These numbers are invented for this test and describe no vehicle.
import contextlib, shutil, tempfile
PROBE_NAME = "probe_car"
PROBE = {**GENERIC, "mass": 1750.0, "mass_sprung": 1580.0, "wheelbase": 2.6,
         "cg_to_front": 1.15, "cg_to_rear": 1.45, "cg_height": 0.58,
         "track_front": 1.58, "track_rear": 1.58, "wheel_radius_nominal": 0.33}
PROBE_ROOT = Path(tempfile.mkdtemp(prefix="vdsim_probe_"))
(PROBE_ROOT / "vehicles").mkdir()
(PROBE_ROOT / "vehicles" / f"{PROBE_NAME}.yaml").write_text(yaml.safe_dump(PROBE))


@contextlib.contextmanager
def probe_root():
    """Expose the probe car only where it is used.

    A private root left set would also resolve the shipped tyre privately and
    flip ``info["source"]`` for the public declarations checked in Q23-2.
    """
    os.environ[vdsim_rl.PRIVATE_ROOT_ENV] = str(PROBE_ROOT)
    try:
        yield
    finally:
        os.environ.pop(vdsim_rl.PRIVATE_ROOT_ENV, None)


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


# ---- Q23-1: a named preset -> the plant carries that YAML's car ----
with probe_root():
    vp, tp, prov = load_vehicle_preset(PROBE_NAME, "generic_pacejka")
    for key in ("mass", "wheelbase", "cg_to_front", "cg_to_rear", "cg_height",
                "track_front", "wheel_radius_nominal"):
        assert abs(PROBE[key] - GENERIC[key]) > 1e-9, f"{key} cannot discriminate"
        assert abs(getattr(vp, key) - PROBE[key]) < 1e-9, key
    assert prov["source"] == "private" and prov["vehicle"] == PROBE_NAME, prov
    w = plant_weight_kg(EnvConfig(vehicle=PROBE_NAME, tire="generic_pacejka"))
print(f"Q23-1 probe plant weight {w.round(1)} kg vs YAML mass {PROBE['mass']}")
assert np.all(np.abs(w / PROBE["mass"] - 1.0) < 0.01), w
res["q23_probe_weight_kg"] = w.tolist()

# ---- Q23-2 (Q23-f): both shipped RL declarations select the public generic
#      car, name it, and do not warn (PO decision 78) ----
for name in ("default_env.yaml", "fast_env.yaml"):
    c = EnvConfig.from_yaml(str(REPO / "configs/rl" / name))
    assert (c.vehicle, c.tire) == ("generic_sedan", "generic_pacejka"), \
        (name, c.vehicle, c.tire)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        e = VDSimVecEnv(2, c, seed=0)
    assert not [x for x in caught
                if vdsim_rl.BUILTIN_WARNING in str(x.message)], (name, caught)
    _, info = e.reset(seed=0)
    assert info["vehicle"] == "generic_sedan", info
    assert info["tire"] == "generic_pacejka" and info["source"] == "public", info
    assert len(info["param_hash"]) == 64 and e.metadata["vdsim"] == info
    w = plant_weight_kg(EnvConfig(**{**c.__dict__, "mass_scale_range": (1.0, 1.0)}))
    assert np.all(np.abs(w / GENERIC["mass"] - 1.0) < 0.01), (name, w)
    print(f"Q23-2 {name}: info {info['vehicle']}/{info['tire']} "
          f"({info['source']}) hash {info['param_hash'][:12]}, "
          f"weight {w.round(1)} kg")
res["q23_yaml_info"] = info
assert EnvConfig.from_yaml(str(REPO / "configs/rl/fast_env.yaml")).max_substep_dt \
    == 0.0025

# ---- Q23-3: vehicle=None still works but says so ----
with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter("always")
    e = VDSimVecEnv(1, EnvConfig(), seed=0)
msgs = [str(x.message) for x in caught if issubclass(x.category, UserWarning)]
assert vdsim_rl.BUILTIN_WARNING in msgs, msgs
assert e.reset(seed=0)[1]["vehicle"] is None
print(f"Q23-3 vehicle=None warns: {msgs}")

# ---- Q23-4: domain randomization scales the loaded preset, not the default ----
with probe_root():
    w = plant_weight_kg(EnvConfig(vehicle=PROBE_NAME, tire="generic_pacejka",
                                  mass_scale_range=(1.1, 1.1)))
print(f"Q23-4 mass_scale 1.1 -> {w.round(1)} kg (expect {1.1 * PROBE['mass']:.1f})")
assert np.all(np.abs(w / (1.1 * PROBE["mass"]) - 1.0) < 0.01), w
assert np.all(np.abs(w / (1.1 * GENERIC["mass"]) - 1.0) > 0.05), w

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
for bad, want in (({**PROBE, "masss": 1.0}, "does not know"),
                  ({k: v for k, v in PROBE.items() if k != "cg_height"}, "missing")):
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
        with probe_root():
            load_vehicle_preset(PROBE_NAME, tyre.stem)
        assert not catalog, f"catalog part {tyre.name} was accepted"
    except ValueError as exc:
        assert catalog and "catalog part" in str(exc), exc
    print(f"Q23-6 tyre {tyre.stem:20s} {'refused (catalog part)' if catalog else 'loads'}")
shutil.rmtree(PROBE_ROOT)
# ---- Q23-f1: generic_sedan/generic_pacejka == the C++ built-in defaults ----
#      Field-by-field, so a change to either the YAML or the C++ default that
#      is not mirrored in the other one fails here instead of silently
#      training a different car than fast_env.yaml's G1 table (a) describes.
def bound_fields(obj):
    return sorted(n for n in dir(obj)
                  if not n.startswith("_") and not callable(getattr(obj, n)))


vp_g, tp_g, prov_g = load_vehicle_preset("generic_sedan", "generic_pacejka")
drift = []
for proto, got in ((vdsim.VehicleParams(), vp_g), (vdsim.TireParams(), tp_g)):
    for f in bound_fields(proto):
        a = vdsim_rl._digest_value(getattr(proto, f))
        b = vdsim_rl._digest_value(getattr(got, f))
        if a != b:
            drift.append((type(proto).__name__, f, a, b))
assert not drift, f"generic preset drifted from the C++ defaults: {drift}"
with warnings.catch_warnings(record=True):
    warnings.simplefilter("always")
    _, _, prov_builtin = load_vehicle_preset(None, None)
assert prov_g["param_hash"] == prov_builtin["param_hash"], (prov_g, prov_builtin)
assert set(prov_g) == {"vehicle", "tire", "param_hash", "source"}, prov_g
assert prov_g["source"] == "public", prov_g
assert "/" not in prov_g["vehicle"] + prov_g["tire"], prov_g
res["q23f_generic_param_hash"] = prov_g["param_hash"]
print(f"Q23-f1 generic_sedan+generic_pacejka == C++ defaults on "
      f"{len(bound_fields(vdsim.VehicleParams())) + len(bound_fields(vdsim.TireParams()))} "
      f"fields, same param_hash {prov_g['param_hash'][:12]}")

# ---- Q23-f2: search order vehicle_file -> $VDSIM_PRIVATE_CONFIGS -> repo ----
import shutil
_priv = Path(tempfile.mkdtemp(prefix="vdsim_priv_"))
(_priv / "vehicles").mkdir()
(_priv / "parts" / "tire").mkdir(parents=True)
_shadow = {**GENERIC, "mass": 1750.0}
(_priv / "vehicles" / "generic_sedan.yaml").write_text(yaml.safe_dump(_shadow))
shutil.copy(REPO / "configs/parts/tire/generic_pacejka.yaml",
            _priv / "parts" / "tire" / "generic_pacejka.yaml")
_abs = Path(tempfile.mkdtemp(prefix="vdsim_abs_")) / "whatever.yaml"
_abs.write_text(yaml.safe_dump({**GENERIC, "mass": 1900.0}))
os.environ[vdsim_rl.PRIVATE_ROOT_ENV] = str(_priv)
try:
    vp_p, _, prov_p = load_vehicle_preset("generic_sedan", "generic_pacejka")
    assert vp_p.mass == 1750.0 and prov_p["source"] == "private", (vp_p.mass, prov_p)
    # the private vehicle's own tire_yaml resolves inside the private root
    _, _, prov_pt = load_vehicle_preset("generic_sedan", None)
    assert prov_pt["source"] == "private" and prov_pt["tire"] == "generic_pacejka", prov_pt
    # an absolute file wins over the private root, and only the stem is recorded
    vp_f, _, prov_f = load_vehicle_preset("generic_sedan", "generic_pacejka",
                                          vehicle_file=str(_abs))
    assert vp_f.mass == 1900.0 and prov_f["source"] == "private", (vp_f.mass, prov_f)
    assert prov_f["vehicle"] == "generic_sedan", prov_f
finally:
    os.environ.pop(vdsim_rl.PRIVATE_ROOT_ENV)
vp_r, _, prov_r = load_vehicle_preset("generic_sedan", "generic_pacejka")
assert vp_r.mass == GENERIC["mass"] and prov_r["source"] == "public", (vp_r.mass, prov_r)
try:
    load_vehicle_preset("generic_sedan", "generic_pacejka",
                        vehicle_file="configs/vehicles/generic_sedan.yaml")
except ValueError as exc:
    assert "absolute" in str(exc), exc
else:
    raise AssertionError("a relative vehicle_file was accepted")
print("Q23-f2 search order: abs file 1900 kg > private root 1750 kg > repo "
      f"{GENERIC['mass']:.0f} kg, relative vehicle_file refused")
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

# ---- Q27: action_mode "accel" -> per-env CmdL5 through the plant PI cascade ----
# Q27-1 the mode is declared, defaults to pedal, and a bad value is refused.
assert EnvConfig().action_mode == "pedal"
for bad in ({"action_mode": "torque"}, {"action_mode": "accel", "max_accel": 0.0}):
    try:
        EnvConfig(**bad)
    except ValueError as exc:
        print(f"Q27-1 refused {bad}: {exc}")
    else:
        raise AssertionError(f"EnvConfig accepted {bad}")


def accel_cfg(**over):
    """default_env.yaml in accel mode, straight spawn, no episode cut-offs."""
    base = EnvConfig.from_yaml(str(REPO / "configs/rl/default_env.yaml"))
    kw = {**base.__dict__, "action_mode": "accel", "speed_range": (15.0, 15.0),
          "lateral_range": 0.0, "yaw_range": 0.0, "time_limit_s": -1.0,
          "min_speed": -1.0, "obs_fields": ["vx", "ax", "throttle_applied",
                                            "brake_applied"]}
    kw.update(over)
    return EnvConfig(**kw)


def quiet_env(n, c, seed):
    return VDSimVecEnv(n, c, seed=seed,
                       reward_fn=lambda o, *_: np.zeros(len(o), np.float32))


# Q27-2 snapshot carries the cascade memory, per env, bit for bit.
# Four envs get different ax targets, so their integrators differ; the tape
# after the snapshot is replayed twice from the same snapshot.
ca = accel_cfg()
ve = quiet_env(4, ca, seed=11)
ve.reset(seed=11)
warm = np.array([[0.0, 0.5], [0.0, -0.5], [0.0, 0.25], [0.0, -0.25]], np.float32)
for _ in range(40):
    ve.step(warm)
snap = ve.core.vs.snapshots()
tape = [np.array([[0.0, 0.3 * np.sin(0.2 * k + j)] for j in range(4)], np.float32)
        for k in range(40)]


def rollout(snaps):
    ve.core.vs.restore(snaps)
    rows = []
    for a in tape:
        o, *_ = ve.step(a)
        rows.append(o.copy())
    return np.stack(rows)


x1 = rollout(snap)
x2 = rollout(snap)
assert np.array_equal(x1, x2), "accel-mode rollout not bit-identical after restore"
# Non-vacuity: the same snapshot with the cascade block put back to its reset
# values (LongVx integ/first, LongAx integ/prev_err/first, pp index, yaw PI --
# the last 7 entries, see SimSession::snapshot) must give a different rollout,
# i.e. the cascade memory is live at the snapshot and the round trip carries it.
cleared = []
for s in snap:
    c = vdsim.SessionSnapshot()
    c.data = list(s.data[:-7]) + [0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    c.rng = s.rng
    cleared.append(c)
x3 = rollout(cleared)
d_clear = float(np.abs(x3 - x1).max())
print(f"Q27-2 restore twice: identical={np.array_equal(x1, x2)}; "
      f"cascade block cleared: max|diff| = {d_clear:.3e}")
assert d_clear > 0.0, "clearing the cascade block changed nothing -- vacuous check"

# Q27-3 a_x step: the realised a_x follows the target (discriminating test).
# Steady state = mean over the last 1 s of each 4 s hold; latency = time from
# the step to 63 % / 90 % of the change. Default cascade gains, no tuning.
ce = accel_cfg()
se = quiet_env(1, ce, seed=2)
se.reset(seed=2)
dt_ctrl = ce.dt * ce.action_repeat
col = se.core.col
holds = [(0.0, 1.0), (1.0, 4.0), (-2.0, 4.0)]      # (target [m/s^2], seconds)
t, ax, thr, brk, tgt = [], [], [], [], []
k = 0
for target, secs in holds:
    for _ in range(int(round(secs / dt_ctrl))):
        o, *_ = se.step(np.array([[0.0, target / ce.max_accel]], np.float32))
        k += 1
        t.append(k * dt_ctrl); tgt.append(target)
        ax.append(float(o[0, col["ax"]]))
        thr.append(float(o[0, col["throttle_applied"]]))
        brk.append(float(o[0, col["brake_applied"]]))
t, ax, thr, brk, tgt = map(np.asarray, (t, ax, thr, brk, tgt))
start = 0.0
q27_step = {}
for i, (target, secs) in enumerate(holds):
    end = start + secs
    if i > 0:
        win = (t > end - 1.0) & (t <= end)
        ss = float(ax[win].mean())
        err = abs(ss - target) / abs(target)
        sat = bool((thr[win] >= 0.999).any() or (brk[win] >= 0.999).any())
        prev = holds[i - 1][0]
        seg = (t > start) & (t <= end)
        frac = (ax[seg] - prev) / (target - prev)
        ts = t[seg] - start
        t63 = float(ts[np.argmax(frac >= 0.63)]) if (frac >= 0.63).any() else float("nan")
        t90 = float(ts[np.argmax(frac >= 0.90)]) if (frac >= 0.90).any() else float("nan")
        q27_step[target] = dict(ss=ss, err=err, saturated=sat, t63=t63, t90=t90)
        print(f"Q27-3 target {target:+.1f} m/s^2: steady {ss:+.4f} (err {100 * err:.2f} %), "
              f"t63 {t63:.2f} s, t90 {t90:.2f} s, pedal saturated={sat}")
        assert err <= 0.05 or sat, f"target {target}: {100 * err:.2f} % off, not saturated"
    start = end
res["q27_step"] = q27_step
print("Q27 accel action checks passed")

# The SB3 PPO smoke (torch) lives in smoke_sb3_ppo.py, outside ctest: this
# file is the ctest target rl_env and needs gymnasium but not torch/SB3.

json.dump(res, open("/tmp/rl_smoke.json", "w"), indent=1, default=str)
print("ALL CHECKS PASSED")
