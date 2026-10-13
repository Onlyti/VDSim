# Work order 33: the RL connection layer -- actuator and road declared in
# EnvConfig reach the VecSession (R1, R2), the "az" observation and the
# reward hook (R3).  R1/R2 run on vdsim_rl._Core; R3-c drives VDSimVecEnv and
# VDSimEnv, so it needs gymnasium (pinned in CI for ctest rl_env), not torch.
# Paths are repo-relative; VDSIM_BUILD_PY overrides the built module dir.
import json, os, sys, tempfile, warnings
from pathlib import Path
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, os.environ.get("VDSIM_BUILD_PY", str(REPO / "build" / "python")))
sys.path.insert(0, str(REPO / "python"))
import hashlib
import numpy as np
import yaml
import vdsim_rl
from vdsim_rl import EnvConfig, _Core

res = {}
DT = 0.005


def zero_reward(obs, col, action, prev_action, cfg):
    """Reward stub: the checks below read observations only."""
    return np.zeros(obs.shape[0], dtype=np.float32)


def make(n=1, reward_fn=zero_reward, **over):
    """A _Core on the public car, one control tick per action, no termination.

    Every env starts at the same state (15 m/s, on the lane, heading 0) so two
    envs differ only through what this file declares.
    """
    kw = dict(vehicle="generic_sedan", tire="generic_pacejka", dt=DT,
              action_repeat=1, speed_range=(15.0, 15.0), lateral_range=0.0,
              yaw_range=0.0, threads=1, time_limit_s=0.0, min_speed=0.0,
              max_lateral=0.0, max_roll=0.0, max_beta=0.0, max_yaw_rate=0.0,
              obs_fields=["steer_applied", "vx", "pitch", "roll",
                          "pitch_rate", "roll_rate"])
    kw.update(over)
    return _Core(n, EnvConfig(**kw), reward_fn=reward_fn, seed=0)


def run(core, steps, steer=0.0, pedal=0.0, step_at=0):
    """Roll out a constant action (steer switched on at ``step_at``) -> (steps, n, dim)."""
    core.reset_all()
    rows = []
    for k in range(steps):
        a = np.array([[steer if k >= step_at else 0.0, pedal]] * core.num_envs,
                     dtype=np.float32)
        obs, *_ = core.step(a)
        rows.append(obs.copy())
    return np.array(rows)


def digest(cfg, n=4, steps=200):
    """sha256 over obs and reward of a fixed-seed rollout with random actions."""
    core = _Core(n, cfg, seed=0)
    rng = np.random.default_rng(123)
    h = hashlib.sha256(core.reset_all().tobytes())
    for _ in range(steps):
        obs, rew, term, trunc, _codes = core.step(
            rng.uniform(-1, 1, size=(n, 2)).astype(np.float32))
        h.update(obs.tobytes()); h.update(rew.tobytes())
        core.reset_done(term | trunc)
        h.update(core.obs.tobytes())
    return h.hexdigest()


# ---- R1-a: actuator=None and the flat road leave the default env unchanged ----
# The new keywords at their defaults must equal the call _Core made before
# work order 33 (no actuator / ground keyword at all).
DEFAULT_YAML = str(REPO / "configs/rl/default_env.yaml")
cfg = EnvConfig.from_yaml(DEFAULT_YAML)
cfg.threads = 2
now = digest(cfg)
NEW_KW = ("actuator", "grade", "bank", "rough_amp", "rough_wavelength",
          "iso_class", "ground_seed")
_orig = vdsim_rl.vdsim.make_vec_session


def _legacy_make_vec_session(*a, **kw):
    """make_vec_session called with the pre-33 argument set."""
    assert not any(k.startswith("psd_") for k in kw), "default config passed psd_*"
    for k in NEW_KW:
        kw.pop(k, None)
    return _orig(*a, **kw)


vdsim_rl.vdsim.make_vec_session = _legacy_make_vec_session
try:
    legacy = digest(cfg)
finally:
    vdsim_rl.vdsim.make_vec_session = _orig
print(f"R1-a default_env digest {now[:16]} legacy call {legacy[:16]}")
assert now == legacy, "default config differs from the pre-33 make_vec_session call"
assert "actuator" not in _Core(1, cfg, seed=0).provenance
res["r1a"] = dict(digest=now, legacy=legacy)

# ---- R1-b: dead time and rate limit act tick-exactly on steer_applied ----
D = 0.02
STEER = 0.2                          # action -> 0.2 * max_steer 0.5 = 0.1 rad
ref = run(make(), 16, steer=STEER, step_at=2)[:, 0, 0]
dly = run(make(actuator={"steer": {"dead_time_s": D}}), 16, steer=STEER,
          step_at=2)[:, 0, 0]
first_ref = int(np.flatnonzero(ref != 0.0)[0])
first_dly = int(np.flatnonzero(dly != 0.0)[0])
lag = first_dly - first_ref
print(f"R1-b dead_time {D} s: steer_applied moves at tick {first_dly}, "
      f"identity at {first_ref} -> lag {lag} ticks (expect {round(D / DT)})")
assert lag == round(D / DT), lag
np.testing.assert_array_equal(dly[first_dly:], ref[first_ref:first_ref + len(dly) - first_dly])

RATE = 0.5                           # [rad/s]
ramp = run(make(actuator={"steer": {"rate_limit": RATE}}), 16, steer=STEER,
           step_at=2)[:, 0, 0]
d = np.diff(ramp.astype(np.float64))
print(f"R1-b rate_limit {RATE} rad/s: max step {d.max():.6g} rad/tick "
      f"(bound {RATE * DT:.6g}), ramp steps {np.round(d[2:6], 6)}")
assert d.max() <= RATE * DT * (1 + 1e-5)
np.testing.assert_allclose(d[2:12], RATE * DT, rtol=1e-4)
res["r1b"] = dict(lag_ticks=lag, max_rate_step=float(d.max()))

# LuGre steer friction: only that the parameters land in every env's session.
LUGRE = {"sigma0": 2.0e4, "Tc": 3.0}
core = make(n=2, actuator={"steer": {"friction": LUGRE}})
for i, ap in enumerate(core.vs.actuators()):
    fr = ap.steer.friction
    assert fr.enabled, f"env {i}: friction not enabled"
    for k, v in LUGRE.items():
        assert getattr(fr, k) == v, (i, k, getattr(fr, k))
print("R1-b LuGre steer friction enabled in every env with the declared values")

# Provenance: the preset id or the hash of what the core received.
p_id = make(actuator="actuator.generic_sedan").provenance["actuator"]
p_dict = make(actuator={"steer": {"dead_time_s": D}}).provenance["actuator"]
assert p_id["preset"] == "actuator.generic_sedan" and len(p_id["param_hash"]) == 64
assert p_dict["preset"] is None and p_dict["param_hash"] != p_id["param_hash"]
with tempfile.TemporaryDirectory() as td:
    raw = yaml.safe_load(open(DEFAULT_YAML))
    raw["actuator"] = {"preset": "actuator.generic_sedan", "steer": {"dead_time_s": D}}
    path = Path(td) / "env.yaml"
    path.write_text(yaml.safe_dump(raw))
    c = _Core(1, EnvConfig.from_yaml(str(path)), seed=0)
    assert c.vs.actuators()[0].steer.ch.dead_time_s == D
    assert c.provenance["actuator"]["preset"] == "actuator.generic_sedan"
print("R1 provenance and from_yaml actuator key checks passed")

# ---- R2-a: an ISO 8608 road excites pitch/roll on L3, the flat road does not ----
COL = {n: i for i, n in enumerate(make().columns)}
WIN = slice(400, None)               # 2..6 s: past the start transient


def ride_std(**road):
    """std of pitch_rate, roll_rate [rad/s] over WIN, L3 straight at fixed pedal."""
    r = run(make(level="L3", **road), 1200, pedal=0.1)[WIN, 0]
    assert np.isfinite(r).all()
    return float(r[:, COL["pitch_rate"]].std()), float(r[:, COL["roll_rate"]].std())


flat = ride_std()
iso = ride_std(iso_class=2)
print(f"R2-a L3 15 m/s: flat std pitch_rate {flat[0]:.3g}, roll_rate {flat[1]:.3g}; "
      f"ISO 8608 class C {iso[0]:.3g}, {iso[1]:.3g} [rad/s]")
assert max(flat) <= 1e-6, flat
assert iso[0] >= 10 * max(flat[0], 1e-12) and iso[1] >= 10 * max(flat[1], 1e-12)
assert min(iso) >= 10 * 1e-6
res["r2a"] = dict(flat=flat, iso_c=iso)

# General PSD road, analytic: with the ISO class C coefficients it is the
# same ground as iso_class=2, bit for bit; a measured table runs on L3 too.
a_iso = run(make(level="L3", iso_class=2), 300, pedal=0.1)
a_psd = run(make(level="L3", road_psd={"gd_n0": 256e-6}), 300, pedal=0.1)
np.testing.assert_array_equal(a_iso, a_psd)
n_tab = [0.011, 0.1, 1.0, 4.0]
tab = run(make(level="L3", road_psd={"n": n_tab, "gd": [256e-6 * (x / 0.1) ** -2 for x in n_tab]}),
          300, pedal=0.1)
assert np.isfinite(tab).all() and tab[100:, 0, COL["pitch_rate"]].std() > 1e-4
print("R2 road_psd: analytic == iso_class bitwise, table road excites pitch")

# ---- R2-b: grade/bank signs (ISO 8855: pitch > 0 nose down, roll > 0 left up) ----
SLOPE = 0.05                         # [rad]
signs = {}
for key, field in (("grade", "pitch"), ("bank", "roll")):
    for s in (+SLOPE, -SLOPE):
        r = run(make(level="L3", speed_range=(0.0, 0.0), **{key: s}), 600,
                pedal=-1.0)[-1, 0]
        val = float(r[COL[field]])
        signs[f"{key}{s:+}"] = val
        # grade > 0 is uphill toward +x: nose up, pitch < 0.
        # bank > 0 raises the +y (left) side: roll > 0.
        expect = -s if key == "grade" else s
        print(f"R2-b {key} {s:+.2f} rad (brake held, vx {r[COL['vx']]:+.3f} m/s): "
              f"{field} {val:+.5f} rad, expect {expect:+.2f}")
        assert np.sign(val) == np.sign(expect) and abs(val - expect) <= 0.1 * SLOPE
res["r2b"] = signs

# ---- R2-c: ground_seed reproducible, seeds and env indices differ ----
A = run(make(n=3, level="L3", iso_class=2, ground_seed=7), 200, pedal=0.1)
B = run(make(n=3, level="L3", iso_class=2, ground_seed=7), 200, pedal=0.1)
C = run(make(n=3, level="L3", iso_class=2, ground_seed=8), 200, pedal=0.1)
np.testing.assert_array_equal(A, B)
assert not np.array_equal(A[:, 0], C[:, 0])
for i, j in ((0, 1), (0, 2), (1, 2)):
    assert not np.array_equal(A[:, i], A[:, j]), (i, j)
print("R2-c same ground_seed bitwise equal, other seed differs, env i != env j")

# ---- R2-d: a road the level cannot feel is refused, not ignored ----
REFUSED = [dict(level="L2", iso_class=2), dict(level="L1", rough_amp=0.01),
           dict(level="L2", road_psd={"gd_n0": 64e-6}), dict(level="K", grade=0.05),
           dict(level="L0", bank=0.05), dict(level="L3", iso_class=2, grade=0.05),
           dict(level="L3", iso_class=2, rough_amp=0.01), dict(level="L3", iso_class=8),
           dict(level="L3", road_psd={"gd_n0": 64e-6, "n": [0.1], "gd": [1e-6]}),
           dict(level="L3", road_psd={"gd_n": 1.0})]
for over in REFUSED:
    try:
        EnvConfig(**over)
    except ValueError as e:
        print(f"R2-d refused {over}: {e}")
    else:
        raise AssertionError(f"accepted {over}")
for over in (dict(level="L2", grade=0.05), dict(level="L1", bank=0.05),
             dict(level="L4", iso_class=0)):
    EnvConfig(**over)
print("R2-d planar level + vertical road -> ValueError")

# ---- S1: the binding refuses a half-given or mismatched PSD table ----
VP, TP, _prov = vdsim_rl.load_vehicle_preset("generic_sedan", "generic_pacejka")
for kw in (dict(psd_n=[0.1, 1.0], psd_gd=[]), dict(psd_n=[], psd_gd=[1e-6]),
           dict(psd_n=[0.1, 1.0], psd_gd=[1e-6])):
    try:
        vdsim_rl.vdsim.make_vec_session(1, VP, TP, level="L3", iso_class=2, **kw)
    except ValueError as e:
        print(f"S1 refused {kw}: {e}")
    else:
        raise AssertionError(f"make_vec_session accepted {kw}")

# ---- R3: the "az" observation and the reward hook ----
RIDE = ["az", "pitch", "roll"]
assert "az" in vdsim_rl.vdsim.obs_field_names()

# R3-a: obs az is SimOutput.az of the same tick -- 1-env VecSession against a
# standalone SimSession on the same ISO 8608 ground (env 0 = ground_seed 1).
core = make(level="L3", iso_class=2, obs_fields=RIDE)
SP = vdsim_rl.vdsim.SolverParams()
SP.max_substep_dt = core.cfg.max_substep_dt
SP.max_substeps = core.cfg.max_substeps
solo = vdsim_rl.vdsim.make_sim_session(VP, TP, level="L3", nominal_dt=DT,
                                       solver=SP, iso_class=2)
core.reset_all()
solo.reset(vdsim_rl.vdsim.make_init_state(0.0, 0.0, 0.0, 15.0))
cmd = vdsim_rl.vdsim.CmdL4()
cmd.throttle = float(np.float32(0.1))   # the env's action is float32
az_vec, az_solo, heave = [], [], []
for k in range(800):
    obs, *_ = core.step(np.array([[0.0, 0.1]], dtype=np.float32))
    solo.set_input(cmd)
    solo.tick(DT)
    az_vec.append(obs[0, core.col["az"]])
    az_solo.append(np.float32(solo.output().az))
    heave.append(core.vs.outputs()[0].heave_z)
az_vec, az_solo, heave = map(np.array, (az_vec, az_solo, heave))
np.testing.assert_array_equal(az_vec, az_solo)
assert np.isfinite(az_vec).all() and az_vec[200:].std() > 0.1, az_vec[200:].std()
print(f"R3-a obs az == standalone SimSession az over 800 ticks "
      f"(std {az_vec[200:].std():.3g} m/s^2 on ISO class C)")

# R3-b: az agrees with the second central difference of the sprung-CG heave
# SimOutput.heave_z (difference used here only, never on the observation
# path).  The work order named vz, but State.velocity z stays 0 on L3 -- the
# 14-DOF heave lives in its own state -- so vz cannot carry this check.
ddz = (heave[2:] - 2.0 * heave[1:-1] + heave[:-2]) / DT ** 2
rho = float(np.corrcoef(ddz[200:], az_vec[1:-1][200:])[0, 1])
print(f"R3-b corr(second central diff of heave_z, az) = {rho:.4f} (need >= 0.95)")
assert rho >= 0.95, rho

# R3-c: a user reward_fn reading col["az"] is called by both gymnasium wrappers
# and is what they return.
W_AZ = 0.5
calls = []


def ride_reward(obs, col, action, prev_action, cfg):
    """Ride comfort example: -w * az^2 per env."""
    calls.append(obs.shape[0])
    return (-W_AZ * obs[:, col["az"]].astype(np.float64) ** 2).astype(np.float32)


ride_kw = dict(vehicle="generic_sedan", tire="generic_pacejka", level="L3",
               iso_class=2, dt=DT, action_repeat=1, speed_range=(15.0, 15.0),
               lateral_range=0.0, yaw_range=0.0, threads=1, time_limit_s=0.0,
               min_speed=0.0, obs_fields=RIDE)
venv = vdsim_rl.VDSimVecEnv(2, EnvConfig(**ride_kw), reward_fn=ride_reward, seed=0)
venv.reset(seed=0)
for _ in range(50):
    obs, rew, term, trunc, info = venv.step(np.tile([0.0, 0.1], (2, 1)).astype(np.float32))
expect = -W_AZ * obs[:, venv.obs_columns.index("az")].astype(np.float64) ** 2
np.testing.assert_allclose(rew, expect, rtol=1e-6)
assert (rew < 0).all() and len(calls) == 50 and set(calls) == {2}
senv = vdsim_rl.VDSimEnv(EnvConfig(**ride_kw), reward_fn=ride_reward, seed=0)
senv.reset(seed=0)
for _ in range(50):
    o1, r1, *_ = senv.step(np.array([0.0, 0.1], dtype=np.float32))
assert r1 == float(np.float32(-W_AZ * float(o1[senv.obs_columns.index("az")]) ** 2))
assert r1 < 0 and len(calls) == 100 and calls[-1] == 1
print(f"R3-c ride reward via col['az']: VDSimVecEnv {rew}, VDSimEnv {r1:.4g}")

# Planar level: az is 0 and EnvConfig warns once.
with warnings.catch_warnings(record=True) as wl:
    warnings.simplefilter("always")
    pcfg = EnvConfig(level="L2", obs_fields=["vx", "az"])
assert sum("obs field 'az'" in str(w.message) for w in wl) == 1, [str(w.message) for w in wl]
pl = run(make(level="L2", obs_fields=["vx", "az"]), 50, steer=0.1, pedal=0.3)
assert (pl[:, 0, 1] == 0.0).all()
print("R3 planar L2: az column all 0, one UserWarning at EnvConfig")

# R3-d: requesting az leaves every other column unchanged, bit for bit.
base_f = ["steer_applied", "vx", "pitch", "roll", "pitch_rate", "roll_rate", "fz"]
plain = run(make(n=2, level="L3", iso_class=2, obs_fields=base_f), 300,
            steer=0.1, pedal=0.1, step_at=50)
with_az = run(make(n=2, level="L3", iso_class=2, obs_fields=base_f[:3] + ["az"] + base_f[3:]),
              300, steer=0.1, pedal=0.1, step_at=50)
np.testing.assert_array_equal(plain, np.delete(with_az, 3, axis=2))
print("R3-d obs columns without az identical with and without the az column")
res["r3"] = dict(az_std=float(az_vec[200:].std()), corr_dvz_az=rho,
                 vec_reward=rew.tolist(), single_reward=r1)

json.dump(res, open(os.path.join(tempfile.gettempdir(), "rl_connection.json"), "w"),
          indent=1, default=str)
print("ALL CHECKS PASSED")
