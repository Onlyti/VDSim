# R5/R6 verification: runtime domain randomization + per-env seed streams.
import json, sys, time
sys.path.insert(0, "/home/ailab-12/git/VDSim/build/python")
sys.path.insert(0, "/home/ailab-12/git/VDSim/python")
import numpy as np
import vdsim
from vdsim_rl import EnvConfig, VDSimVecEnv

res = {}
DT = 0.005

def peak_lateral(mu_scale=1.0, mass_scale=1.0, tire_stiffness_scale=1.0,
                 v0=25.0, steer=0.15, reuse=None):
    """Peak |ay| [m/s^2] in a hard steady turn -- grip-limited, so it tracks mu.

    (A full-brake test would be brake-torque limited on this vehicle, not
    grip limited, and would barely move with mu.)
    """
    vs = reuse or vdsim.make_vec_session(1, vdsim.VehicleParams(), vdsim.TireParams(),
                                         level="L2", nominal_dt=DT, mu=1.0, threads=1)
    d = vdsim.DomainRandomization()
    d.mu_scale = mu_scale
    d.mass_scale = mass_scale
    d.tire_stiffness_scale = tire_stiffness_scale
    vs.set_randomizations([d])
    vs.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, v0)])
    c = vdsim.CmdL4(); c.steer_angle_wheel = steer
    vs.set_input_all(c)
    peak = 0.0
    for _ in range(200):                     # 1 s
        vs.tick(DT, 1)
        peak = max(peak, abs(vs.outputs()[0].ay))
    return peak, vs

# ---- R5a: one session object, several mu draws, no re-allocation ----
base, vs = peak_lateral(1.0)
rows = []
for mu in (1.0, 0.7, 0.4):
    ay, _ = peak_lateral(mu_scale=mu, reuse=vs)
    rows.append({"mu_scale": mu, "peak_ay": ay})
    print(f"mu_scale={mu:.1f}  peak |ay| = {ay:5.2f} m/s^2   (same session object)")
res["mu"] = rows
assert rows[2]["peak_ay"] < rows[0]["peak_ay"] * 0.75, "mu_scale had no grip effect"

# ---- R5b: mass and tire stiffness change the sub-limit response ----
# (at the grip limit ay_max = mu*g regardless of mass, so mass has to be read
#  from the sub-limit yaw-rate gain, not from peak |ay|.)
def step_steer_yawrate(tire_stiffness_scale=1.0, mass_scale=1.0):
    vs = vdsim.make_vec_session(1, vdsim.VehicleParams(), vdsim.TireParams(),
                                level="L2", nominal_dt=DT, mu=1.0, threads=1)
    d = vdsim.DomainRandomization()
    d.tire_stiffness_scale = tire_stiffness_scale
    d.mass_scale = mass_scale
    vs.set_randomizations([d])
    vs.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 20.0)])
    c = vdsim.CmdL4(); c.steer_angle_wheel = 0.05
    vs.set_input_all(c)
    vs.tick(DT, 100)          # 0.5 s (transient; the steady state is grip-set)
    o = vs.outputs()[0]
    return vs.states()[0].yaw_rate(), abs(o.slip_angle[0])

(m_lo, _), (m_hi, _) = step_steer_yawrate(mass_scale=1.0), step_steer_yawrate(mass_scale=1.4)
print(f"mass_scale 1.0 -> yaw_rate {m_lo:.4f} rad/s | 1.4 -> {m_hi:.4f} rad/s "
      f"(steer 0.05 rad, 20 m/s, 0.5 s)")
res["mass_yawrate"] = {"x1.0": m_lo, "x1.4": m_hi}
assert abs(m_hi - m_lo) > 1e-3, "mass_scale had no effect"

(r_lo, a_lo), (r_hi, a_hi) = step_steer_yawrate(0.5), step_steer_yawrate(1.5)
print(f"tire_stiffness_scale 0.5 -> yaw_rate {r_lo:.4f} rad/s, |slip FL| "
      f"{np.degrees(a_lo):.2f} deg | 1.5 -> {r_hi:.4f} rad/s, "
      f"{np.degrees(a_hi):.2f} deg")
res["tire_stiffness"] = {"x0.5": {"yaw_rate": r_lo, "slip_deg": float(np.degrees(a_lo))},
                         "x1.5": {"yaw_rate": r_hi, "slip_deg": float(np.degrees(a_hi))}}
assert r_hi > r_lo * 1.2, "tire stiffness scale had no effect"

# ---- R5c: sensor delay promoted to a reset argument ----
sens = vdsim.SensorParams(); sens.enabled = True
vs2 = vdsim.make_vec_session(1, vdsim.VehicleParams(), vdsim.TireParams(),
                             level="L2", nominal_dt=DT, sensor_delay_s=0.0,
                             sensors=sens, threads=1)
lags = []
for delay in (0.0, 0.10):
    d = vdsim.DomainRandomization(); d.sensor_delay_s = delay
    vs2.set_randomizations([d])
    vs2.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 10.0)])
    c = vdsim.CmdL4(); c.throttle = 1.0
    vs2.set_input_all(c)
    vs2.tick(DT, 100)          # 0.5 s
    s = vs2.at(0)
    lags.append(s.state().vx() - s.measured_state().vx())
print(f"sensor_delay 0.00 s -> true-measured dvx {lags[0]:+.4f} | "
      f"0.10 s -> {lags[1]:+.4f} m/s")
res["sensor_delay_dvx"] = lags
assert abs(lags[1]) > abs(lags[0]) + 0.05, "sensor_delay_s had no effect"

# ---- R6: same seed -> bitwise identical rollout (randomization + noise on) ----
cfg = EnvConfig(
    level="L2", time_limit_s=1.0, speed_range=(10.0, 20.0), lateral_range=1.0,
    mass_scale_range=(0.8, 1.2), mu_scale_range=(0.6, 1.0),
    tire_stiffness_scale_range=(0.8, 1.2), threads=4)

def rollout(seed, steps=200):
    env = VDSimVecEnv(8, cfg, seed=seed)
    obs, _ = env.reset(seed=seed)
    rng = np.random.default_rng(1234)          # same action sequence every run
    traj, sens_out, reasons = [obs.copy()], [], []
    for _ in range(steps):
        a = rng.uniform(-1, 1, size=(8, 2)).astype(np.float32)
        obs, r, te, tr, info = env.step(a)
        traj.append(obs.copy())
        if (te | tr).any():
            reasons.append(sorted(str(x) for x in info["termination_reason"] if x))
    drs = [(d.mass_scale, d.mu_scale, d.tire_stiffness_scale)
           for d in env.core.randomization]
    return np.stack(traj), drs, reasons

a1, dr1, rs1 = rollout(42)
a2, dr2, rs2 = rollout(42)
a3, dr3, rs3 = rollout(43)
same = np.array_equal(a1, a2)
diff = not np.array_equal(a1, a3)
print(f"seed 42 twice: bitwise identical = {same} (max |diff| = {np.abs(a1-a2).max():.3e})")
print(f"seed 43 differs = {diff} (max |diff| vs seed 42 = {np.abs(a1-a3).max():.3e})")
print(f"per-env randomization draws (seed 42, first 3 envs): "
      f"{[tuple(round(x,4) for x in d) for d in dr1[:3]]}")
res["repro"] = {"same_seed_bitwise": bool(same), "diff_seed_differs": bool(diff),
                "draws_match": dr1 == dr2, "n_resets_seen": len(rs1)}
assert same and diff and dr1 == dr2

# ---- R6b: episode k is independent of what ran before it ----
env = VDSimVecEnv(4, cfg, seed=7)
env.reset(seed=7)
for _ in range(50):
    env.step(np.zeros((4, 2), dtype=np.float32))
o_a, _ = env.reset(seed=7)
env2 = VDSimVecEnv(4, cfg, seed=7)
o_b, _ = env2.reset(seed=7)
print(f"re-armed reset matches a fresh env bitwise = {np.array_equal(o_a, o_b)}")
res["reset_rearm_bitwise"] = bool(np.array_equal(o_a, o_b))
assert np.array_equal(o_a, o_b)

json.dump(res, open("/tmp/r56.json", "w"), indent=1, default=str)
print("ALL CHECKS PASSED")
