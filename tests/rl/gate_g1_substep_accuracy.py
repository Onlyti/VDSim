"""G1: does relaxing max_substep_dt (the RL preset) cost accuracy?

Reference = the same plant integrated with max_substep_dt = 0.1 ms (100x finer
than the shipped default), everything else identical.  Error metrics are taken
against that reference on two L3 manoeuvres, and throughput is measured on the
same box so the trade is visible in one table.
"""
import json, sys, time
sys.path.insert(0, "/home/ailab-12/git/VDSim/build/python")
import numpy as np
import vdsim

DT = 0.005            # outer tick [s] (the RL control tick)
LEVEL = "L3"
REF_SUBSTEP = 1e-4

def session(max_substep_dt, n=1, threads=1):
    sp = vdsim.SolverParams()
    sp.max_substep_dt = max_substep_dt
    sp.max_substeps = 2000           # never clamp: we are measuring the step size
    return vdsim.make_vec_session(n, vdsim.VehicleParams(), vdsim.TireParams(),
                                  level=LEVEL, nominal_dt=DT, mu=1.0,
                                  solver=sp, threads=threads)

def step_steer(max_substep_dt, v0=20.0, steer=0.05, t_end=3.0, t_step=0.5):
    v = session(max_substep_dt)
    v.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, v0)])
    n = int(round(t_end / DT))
    out = np.zeros((n, 4))
    for k in range(n):
        c = vdsim.CmdL4()
        c.steer_angle_wheel = steer if k * DT >= t_step else 0.0
        v.set_input_all(c)
        v.tick(DT, 1)
        o = v.outputs()[0]
        out[k] = (o.state.yaw_rate(), o.ay, o.roll, o.state.vy())
    return out

def brake(max_substep_dt, v0=25.0, t_end=5.0):
    v = session(max_substep_dt)
    v.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, v0)])
    n = int(round(t_end / DT))
    out = np.zeros((n, 4))
    for k in range(n):
        c = vdsim.CmdL4(); c.brake = 1.0
        v.set_input_all(c)
        v.tick(DT, 1)
        o = v.outputs()[0]
        out[k] = (o.state.vx(), o.state.position[0], o.pitch, o.Fz[0])
    return out

def throughput(max_substep_dt, envs=64, threads=22, steps=2000):
    v = session(max_substep_dt, envs, threads)
    v.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 20.0)])
    c = vdsim.CmdL4(); c.throttle = 0.3; c.steer_angle_wheel = 0.02
    v.set_input_all(c)
    v.tick(DT, 20)
    t0 = time.perf_counter(); v.tick(DT, steps); el = time.perf_counter() - t0
    return envs * steps / el

CANDIDATES = [5e-4, 1e-3, 2.5e-3, 5e-3]     # 1e-3 is the shipped default
SIGNALS = {"step_steer": ["yaw_rate [rad/s]", "ay [m/s^2]", "roll [rad]", "vy [m/s]"],
           "brake":      ["vx [m/s]", "x [m]", "pitch [rad]", "Fz_FL [N]"]}

ref = {"step_steer": step_steer(REF_SUBSTEP), "brake": brake(REF_SUBSTEP)}
rows = []
for h in CANDIDATES:
    got = {"step_steer": step_steer(h), "brake": brake(h)}
    rec = {"max_substep_dt": h, "substeps_per_tick": max(1, int(round(DT / h)))}
    for man, names in SIGNALS.items():
        r, g = ref[man], got[man]
        for j, nm in enumerate(names):
            err = g[:, j] - r[:, j]
            scale = max(np.abs(r[:, j]).max(), 1e-12)
            rec[f"{man}:{nm}"] = {
                "rmse": float(np.sqrt(np.mean(err ** 2))),
                "max_abs": float(np.abs(err).max()),
                "max_rel_pct": float(100.0 * np.abs(err).max() / scale),
                "ref_peak": float(np.abs(r[:, j]).max()),
            }
    rec["steps_per_s_64env_22thr"] = throughput(h)
    rows.append(rec)
    worst = max(v["max_rel_pct"] for k, v in rec.items() if isinstance(v, dict))
    print(f"max_substep_dt={h*1000:5.2f} ms ({rec['substeps_per_tick']:2d} sub/tick)  "
          f"worst max-rel error {worst:6.3f} %   "
          f"{rec['steps_per_s_64env_22thr']:11,.0f} steps/s")

print()
print(f"{'signal':28s} " + " ".join(f"{h*1000:>10.2f}ms" for h in CANDIDATES))
for man, names in SIGNALS.items():
    for nm in names:
        key = f"{man}:{nm}"
        line = f"{key:28s} " + " ".join(
            f"{r[key]['max_rel_pct']:11.4f}%" for r in rows)
        print(line)

ref_stop = ref["brake"][-1, 1]
print(f"\nreference stopping distance {ref_stop:.4f} m; candidates: " +
      ", ".join(f"{h*1000:.2f}ms -> {brake(h)[-1,1]:.4f} m" for h in CANDIDATES))

json.dump({"reference_max_substep_dt": REF_SUBSTEP, "outer_dt": DT,
           "level": LEVEL, "rows": rows},
          open("/tmp/g1.json", "w"), indent=1)
print("G1 DONE")
