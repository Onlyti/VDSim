"""G1: does relaxing max_substep_dt (the RL preset) cost accuracy?

Reference = the same plant integrated with max_substep_dt = 0.1 ms (100x finer
than the shipped default), everything else identical.  Error metrics are taken
against that reference on two manoeuvres, and throughput is measured on the
same box so the trade is visible in one table.

The car is part of the measurement: ``--vehicle``/``--tire`` take the same
preset stems as ``EnvConfig`` and go through ``vdsim_rl.load_vehicle_preset``,
so the printed ``param_hash`` is the one an env built from that YAML reports.
Without ``--vehicle`` the C++ built-in generic car is used (the 2026-09-17
table in configs/rl/fast_env.yaml was measured that way, at L3).

    python3 tests/rl/gate_g1_substep_accuracy.py --vehicle generic_sedan \\
        --tire generic_pacejka --level L2 --out /tmp/g1_generic_L2.json

A preset that lives outside the repository is measured the same way: point
$VDSIM_PRIVATE_CONFIGS at its directory and pass its stems.

Python paths: VDSIM_BUILD_PY (compiled module dir) and VDSIM_PY (repo python/)
default to the in-repo build tree.
"""
import argparse, json, os, sys, time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, os.environ.get("VDSIM_BUILD_PY", str(_REPO / "build" / "python")))
sys.path.insert(0, os.environ.get("VDSIM_PY", str(_REPO / "python")))
import numpy as np
import vdsim

DT = 0.005            # outer tick [s] (the RL control tick)
REF_SUBSTEP = 1e-4

ap = argparse.ArgumentParser()
ap.add_argument("--vehicle", default=None, help="configs/vehicles/<stem>.yaml")
ap.add_argument("--tire", default=None, help="configs/parts/tire/<stem>.yaml")
ap.add_argument("--level", default="L3")
ap.add_argument("--out", default="/tmp/g1.json")
ARGS = ap.parse_args()
LEVEL = ARGS.level

if ARGS.vehicle is None and ARGS.tire is None:
    VP, TP = vdsim.VehicleParams(), vdsim.TireParams()
    PROV = {"vehicle": None, "tire": None, "param_hash": None}
else:
    from vdsim_rl import load_vehicle_preset
    VP, TP, PROV = load_vehicle_preset(ARGS.vehicle, ARGS.tire)


def session(max_substep_dt, n=1, threads=1):
    sp = vdsim.SolverParams()
    sp.max_substep_dt = max_substep_dt
    sp.max_substeps = 2000           # never clamp: we are measuring the step size
    return vdsim.make_vec_session(n, VP, TP, level=LEVEL, nominal_dt=DT, mu=1.0,
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

print(f"level={LEVEL} vehicle={PROV['vehicle']} tire={PROV['tire']} "
      f"param_hash={PROV['param_hash']}")
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
           "level": LEVEL, "provenance": PROV, "rows": rows},
          open(ARGS.out, "w"), indent=1)
print("G1 DONE")
