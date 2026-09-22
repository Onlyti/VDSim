"""Why is the Ioniq5 preset slower than the built-in car?  (Q23-d, measure only)

Core-only throughput (VecSession.tick, no Python adapter) at the fast_env
solver settings, changing ONE factor at a time between the C++ built-in car and
the ioniq5_awd / ioniq5_pac2002 preset:

  tire backend   mf96 (built-in)  vs  mf2002 (.tir file)
  tire params    built-in values  vs  preset values, same backend
  vehicle params built-in values  vs  preset values
  plant_path     False (built-in) vs  True (preset)

Each case is timed REPEAT times; the median is reported.  Nothing is tuned.

    python3 tests/rl/bench_preset_cost.py --out /tmp/preset_cost.json
"""
import argparse, json, os, statistics, sys, time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, os.environ.get("VDSIM_BUILD_PY", str(_REPO / "build" / "python")))
sys.path.insert(0, os.environ.get("VDSIM_PY", str(_REPO / "python")))
import vdsim
from vdsim_rl import EnvConfig, load_vehicle_preset

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=str(_REPO / "configs" / "rl" / "fast_env.yaml"))
ap.add_argument("--envs", type=int, default=64)
ap.add_argument("--threads", type=int, default=22)
ap.add_argument("--steps", type=int, default=3000)
ap.add_argument("--repeat", type=int, default=3)
ap.add_argument("--out", default="/tmp/preset_cost.json")
A = ap.parse_args()

CFG = EnvConfig.from_yaml(A.config)
VP_I, TP_I, PROV = load_vehicle_preset(CFG.vehicle, CFG.tire)


def fresh(vp_src, tp_src):
    """Copy through YAML-free attribute cloning so cases never share objects."""
    vp, tp = vdsim.VehicleParams(), vdsim.TireParams()
    for src, dst in ((vp_src, vp), (tp_src, tp)):
        for k in dir(src):
            if k.startswith("_") or callable(getattr(src, k)):
                continue
            try:
                setattr(dst, k, getattr(src, k))
            except (AttributeError, TypeError):
                pass
    return vp, tp


def case(name, vp, tp):
    sp = vdsim.SolverParams()
    sp.max_substep_dt = CFG.max_substep_dt
    sp.max_substeps = CFG.max_substeps
    times = []
    for _ in range(A.repeat):
        v = vdsim.make_vec_session(A.envs, vp, tp, level=CFG.level,
                                   nominal_dt=CFG.dt, mu=1.0, solver=sp,
                                   threads=A.threads)
        v.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 15.0)])
        c = vdsim.CmdL4(); c.throttle = 0.3; c.steer_angle_wheel = 0.02
        v.set_input_all(c)
        v.tick(CFG.dt, 50)
        t0 = time.perf_counter(); v.tick(CFG.dt, A.steps)
        times.append(A.envs * A.steps / (time.perf_counter() - t0))
    med = statistics.median(times)
    print(f"{name:44s} {med:12,.0f} ticks/s   runs={[round(t) for t in times]}")
    return {"case": name, "ticks_per_s_median": med, "runs": times,
            "tire_backend": tp.backend, "plant_path": bool(vp.plant_path)}


B_VP, B_TP = vdsim.VehicleParams(), vdsim.TireParams()
rows = []
rows.append(case("A builtin vehicle + builtin tire (mf96)", *fresh(B_VP, B_TP)))
vp, tp = fresh(B_VP, TP_I)
rows.append(case("B builtin vehicle + preset tire (mf2002)", vp, tp))
vp, tp = fresh(B_VP, TP_I); tp.backend = B_TP.backend; tp.tir_path = ""
rows.append(case("C builtin vehicle + preset tire params, mf96", vp, tp))
vp, tp = fresh(VP_I, B_TP); vp.plant_path = False
rows.append(case("D preset vehicle (plant_path off) + builtin tire", vp, tp))
vp, tp = fresh(VP_I, B_TP)
rows.append(case("E preset vehicle (plant_path on) + builtin tire", vp, tp))
vp, tp = fresh(B_VP, B_TP); vp.plant_path = True
rows.append(case("F builtin vehicle, plant_path on + builtin tire", vp, tp))
rows.append(case("G preset vehicle + preset tire (as shipped)", *fresh(VP_I, TP_I)))

base = rows[0]["ticks_per_s_median"]
print()
for r in rows:
    r["slowdown_vs_A"] = base / r["ticks_per_s_median"]
    print(f"{r['case']:44s} x{r['slowdown_vs_A']:.2f}")
json.dump({"level": CFG.level, "dt": CFG.dt, "max_substep_dt": CFG.max_substep_dt,
           "envs": A.envs, "threads": A.threads, "steps": A.steps,
           "preset": PROV, "rows": rows}, open(A.out, "w"), indent=1)
print("COST DONE")
