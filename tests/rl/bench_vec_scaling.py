# R1 verification: VecSession correctness (bitwise vs serial SimSession) + core scaling.
import json, sys, time
import numpy as np
sys.path.insert(0, "/home/ailab-12/git/VDSim/build/python")
import vdsim

vp, tp = vdsim.VehicleParams(), vdsim.TireParams()
LEVEL, DT = "L2", 0.005

def cmd(th, st):
    c = vdsim.CmdL4(); c.throttle = th; c.steer_angle_wheel = st; return c

def snap(st):
    out = []
    for a in sorted(dir(st)):
        if a.startswith("_"): continue
        try:
            v = getattr(st, a)
        except TypeError:
            continue          # unregistered return type (quaternion accessor)
        if callable(v): continue
        if isinstance(v, float): out.append((a, v))
        elif isinstance(v, (list, tuple, np.ndarray)):
            try: out.append((a, tuple(np.asarray(v, dtype=float).ravel())))
            except Exception: continue
    return out

# ---------- 1. equivalence: 4 envs in a pool vs 4 serial sessions ----------
N, STEPS = 4, 2000
cmds = [cmd(0.2 + 0.1 * i, 0.01 * i) for i in range(N)]

serial = []
for i in range(N):
    s = vdsim.make_sim_session(vp, tp, level=LEVEL, nominal_dt=DT)
    s.reset(vdsim.make_init_state(0.0, 0.0, 0.0, 10.0))
    for _ in range(STEPS):
        s.set_input(cmds[i]); s.tick(DT)
    serial.append(snap(s.state()))

vec = vdsim.make_vec_session(N, vp, tp, level=LEVEL, nominal_dt=DT, threads=4)
vec.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 10.0)])
vec.set_inputs(cmds)
vec.tick(DT, STEPS)
par = [snap(s) for s in vec.states()]

diffs = []
for i in range(N):
    for (a, sv), (b, pv) in zip(serial[i], par[i]):
        if sv != pv:
            diffs.append((i, a, sv, pv))
print(f"equivalence: envs={N} steps={STEPS} fields={len(serial[0])} mismatches={len(diffs)}")
for d in diffs[:10]:
    print("   MISMATCH", d)

# ---------- 2. scaling ----------
def bench(num_envs, threads, steps=4000):
    v = vdsim.make_vec_session(num_envs, vp, tp, level=LEVEL, nominal_dt=DT, threads=threads)
    v.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 10.0)])
    v.set_input_all(cmd(0.3, 0.02))
    v.tick(DT, 50)                       # warm up
    t0 = time.perf_counter(); v.tick(DT, steps); el = time.perf_counter() - t0
    return num_envs * steps / el, v.threads

rows = []
base = None
for th in (1, 2, 4, 8, 12, 16, 20, 22):
    sps, eff = bench(max(th, 22), th)
    if base is None: base = sps
    rows.append({"threads": th, "effective": eff, "steps_per_s": sps, "speedup": sps / base})
    print(f"threads={th:2d} (eff {eff:2d})  {sps:12,.0f} steps/s   x{sps/base:5.2f}")

# single SimSession baseline through python loop (last turn's 13.6k number)
s = vdsim.make_sim_session(vp, tp, level=LEVEL, nominal_dt=DT)
s.reset(vdsim.make_init_state(0.0, 0.0, 0.0, 10.0))
c = cmd(0.3, 0.02)
t0 = time.perf_counter()
for _ in range(20000): s.set_input(c); s.tick(DT)
el = time.perf_counter() - t0
py_loop = 20000 / el
print(f"python-loop single session: {py_loop:,.0f} steps/s")

json.dump({"mismatches": len(diffs), "rows": rows, "py_loop_sps": py_loop},
          open("/tmp/r1_result.json", "w"), indent=1)
