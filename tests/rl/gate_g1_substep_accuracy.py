"""G1: does relaxing max_substep_dt (the RL preset) cost accuracy?

Reference = the same plant integrated with a much finer max_substep_dt
(``--ref-substep``, default 1 us), everything else identical: same outer tick
DT, same zero-order-hold input sampling, same manoeuvres.  Error metrics are
taken against that reference on two manoeuvres, and throughput is measured on
the same box so the trade is visible in one table.

Error definition (unchanged since 2026-09-17, so old and new tables compare):

    max_rel_pct(h) = 100 * max_t |x_h(t) - x_ref(t)| / max_t |x_ref(t)|

Convergence order between two grid points h_a < h_b (effective step sizes):

    p = log(E(h_b) / E(h_a)) / log(h_b / h_a)      (= log10 ratio for a decade)

The reference must really be integrated at ``--ref-substep``.  solver_substeps()
silently caps the substep count at SolverParams.max_substeps (params.cpp), so
``--max-substeps`` must cover DT / min(step); the script refuses to start
otherwise and, after the run, requires vdsim.solver_substep_clamp_count() == 0.
A run with a non-zero count is written with ``"discarded": true`` and exits 3.

Every grid row is also compared against the 0.1 ms run (``vs_0p1ms_pct``), the
reference used before 2026-09-22, so the old table can be printed next to the
new one without re-running anything.

The car is part of the measurement: ``--vehicle``/``--tire`` take the same
preset stems as ``EnvConfig`` and go through ``vdsim_rl.load_vehicle_preset``,
so the printed ``param_hash`` is the one an env built from that YAML reports.
Without ``--vehicle`` the C++ built-in generic car is used.

    python3 tests/rl/gate_g1_substep_accuracy.py --vehicle generic_sedan \\
        --tire generic_pacejka --level L2 --out /tmp/g1_generic_L2.json

A preset that lives outside the repository is measured the same way: point
$VDSIM_PRIVATE_CONFIGS at its directory and pass its stems.

Python paths: VDSIM_BUILD_PY (compiled module dir) and VDSIM_PY (repo python/)
default to the in-repo build tree.
"""
import argparse, json, math, os, sys, time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, os.environ.get("VDSIM_BUILD_PY", str(_REPO / "build" / "python")))
sys.path.insert(0, os.environ.get("VDSIM_PY", str(_REPO / "python")))
import numpy as np
import vdsim

DT = 0.005            # outer tick [s] (the RL control tick); never varied here
OLD_REF_SUBSTEP = 1e-4

ap = argparse.ArgumentParser()
ap.add_argument("--vehicle", default=None, help="configs/vehicles/<stem>.yaml")
ap.add_argument("--tire", default=None, help="configs/parts/tire/<stem>.yaml")
ap.add_argument("--level", default="L3")
ap.add_argument("--ref-substep", type=float, default=1e-6,
                help="reference max_substep_dt [s] (default 1 us)")
ap.add_argument("--grid", default="1e-5,1e-4,5e-4,1e-3,2.5e-3,5e-3",
                help="comma list of max_substep_dt [s] to evaluate")
ap.add_argument("--max-substeps", type=int, default=6000,
                help="SolverParams.max_substeps; must cover DT/min(step)")
ap.add_argument("--throughput-min-substep", type=float, default=5e-4,
                help="skip the 64-env throughput bench below this step [s]")
ap.add_argument("--out", default="/tmp/g1.json")
ARGS = ap.parse_args()
LEVEL = ARGS.level
REF_SUBSTEP = ARGS.ref_substep
CANDIDATES = sorted(float(x) for x in ARGS.grid.split(","))


def substeps(h):
    """Substep count solver_substeps() uses for this h (std::ceil(DT/h))."""
    return max(1, math.ceil(DT / h))


_need = substeps(min(CANDIDATES + [REF_SUBSTEP]))
if ARGS.max_substeps < _need:
    sys.exit(f"--max-substeps {ARGS.max_substeps} < {_need} needed for "
             f"h={min(CANDIDATES + [REF_SUBSTEP])} at DT={DT}: the solver would "
             f"clamp and the reference would not be what it claims to be")

if ARGS.vehicle is None and ARGS.tire is None:
    VP, TP = vdsim.VehicleParams(), vdsim.TireParams()
    PROV = {"vehicle": None, "tire": None, "param_hash": None}
else:
    from vdsim_rl import load_vehicle_preset
    VP, TP, PROV = load_vehicle_preset(ARGS.vehicle, ARGS.tire)


def session(max_substep_dt, n=1, threads=1):
    sp = vdsim.SolverParams()
    sp.max_substep_dt = max_substep_dt
    sp.max_substeps = ARGS.max_substeps
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

def rel_pct(g, r):
    return float(100.0 * np.abs(g - r).max() / max(np.abs(r).max(), 1e-12))

SIGNALS = {"step_steer": ["yaw_rate [rad/s]", "ay [m/s^2]", "roll [rad]", "vy [m/s]"],
           "brake":      ["vx [m/s]", "x [m]", "pitch [rad]", "Fz_FL [N]"]}
KEYS = [f"{m}:{n}" for m, ns in SIGNALS.items() for n in ns]

print(f"level={LEVEL} vehicle={PROV['vehicle']} tire={PROV['tire']} "
      f"param_hash={PROV['param_hash']} ref_substep={REF_SUBSTEP:g} s "
      f"max_substeps={ARGS.max_substeps}")
vdsim.reset_solver_substep_clamp_count()
t0 = time.perf_counter()
ref = {"step_steer": step_steer(REF_SUBSTEP), "brake": brake(REF_SUBSTEP)}
ref_wall = time.perf_counter() - t0
print(f"reference ({substeps(REF_SUBSTEP)} sub/tick) wall {ref_wall:.2f} s")

runs = {h: {"step_steer": step_steer(h), "brake": brake(h)} for h in CANDIDATES}
old = runs.get(OLD_REF_SUBSTEP)
rows = []
for h in CANDIDATES:
    got = runs[h]
    n_sub = substeps(h)
    rec = {"max_substep_dt": h, "substeps_per_tick": n_sub, "h_eff": DT / n_sub}
    for man, names in SIGNALS.items():
        r, g = ref[man], got[man]
        for j, nm in enumerate(names):
            err = g[:, j] - r[:, j]
            rec[f"{man}:{nm}"] = {
                "rmse": float(np.sqrt(np.mean(err ** 2))),
                "max_abs": float(np.abs(err).max()),
                "max_rel_pct": rel_pct(g[:, j], r[:, j]),
                "ref_peak": float(np.abs(r[:, j]).max()),
                "vs_0p1ms_pct": (rel_pct(g[:, j], old[man][:, j])
                                 if old is not None else None),
            }
    rec["steps_per_s_64env_22thr"] = (throughput(h)
                                      if h >= ARGS.throughput_min_substep else None)
    rows.append(rec)

clamp_count = int(vdsim.solver_substep_clamp_count())
print(f"solver_substep_clamp_count() = {clamp_count}")

# convergence order for every adjacent pair of the grid (h_a < h_b)
orders = []
for a, b in zip(rows[:-1], rows[1:]):
    pair = {"h_a": a["h_eff"], "h_b": b["h_eff"]}
    for k in KEYS:
        ea, eb = a[k]["max_rel_pct"], b[k]["max_rel_pct"]
        pair[k] = (math.log(eb / ea) / math.log(b["h_eff"] / a["h_eff"])
                   if ea > 0 and eb > 0 else None)
    orders.append(pair)

print()
print(f"{'max-rel error vs ref [%]':28s} " +
      " ".join(f"{h*1000:>10.3g}ms" for h in CANDIDATES))
for k in KEYS:
    print(f"{k:28s} " + " ".join(f"{r[k]['max_rel_pct']:11.4g}%" for r in rows))
print()
print(f"{'order p (adjacent pairs)':28s} " +
      " ".join(f"{o['h_a']*1000:>5.3g}/{o['h_b']*1000:<5.3g}" for o in orders))
for k in KEYS:
    print(f"{k:28s} " + " ".join(
        f"{o[k]:11.3f}" if o[k] is not None else f"{'n/a':>11s}" for o in orders))
print()
print("throughput [steps/s, 64 env x 22 thr]: " + ", ".join(
    f"{r['max_substep_dt']*1000:g}ms -> "
    + (f"{r['steps_per_s_64env_22thr']:,.0f}" if r['steps_per_s_64env_22thr'] else "skipped")
    for r in rows))

discarded = clamp_count != 0
json.dump({"reference_max_substep_dt": REF_SUBSTEP, "outer_dt": DT,
           "max_substeps": ARGS.max_substeps, "clamp_count": clamp_count,
           "discarded": discarded, "reference_wall_s": ref_wall,
           "level": LEVEL, "provenance": PROV, "rows": rows, "orders": orders},
          open(ARGS.out, "w"), indent=1)
if discarded:
    print("G1 DISCARDED: substep clamp fired; the reference is not the claimed one")
    sys.exit(3)
print("G1 DONE")
