# R1 ceiling: RL solver preset (max_substep_dt=1ms) + 64 envs, thread sweep.
import json, sys, time
sys.path.insert(0, "/home/ailab-12/git/VDSim/build/python")
import vdsim

vp, tp = vdsim.VehicleParams(), vdsim.TireParams()
LEVEL, DT, ENVS = "L2", 0.005, 64

def cmd(th, st):
    c = vdsim.CmdL4(); c.throttle = th; c.steer_angle_wheel = st; return c

def sweep(sp, tag, steps=4000):
    rows, base = [], None
    for th in (1, 2, 4, 8, 12, 16, 20, 22):
        v = vdsim.make_vec_session(ENVS, vp, tp, level=LEVEL, nominal_dt=DT,
                                   solver=sp, threads=th)
        v.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 10.0)])
        v.set_input_all(cmd(0.3, 0.02))
        v.tick(DT, 50)
        t0 = time.perf_counter(); v.tick(DT, steps); el = time.perf_counter() - t0
        sps = ENVS * steps / el
        if base is None: base = sps
        rows.append({"threads": th, "steps_per_s": sps, "speedup": sps / base})
        print(f"[{tag}] threads={th:2d}  {sps:12,.0f} steps/s  x{sps/base:5.2f}")
    return rows

sp_def = vdsim.SolverParams()
print(f"default solver: max_substep_dt={sp_def.max_substep_dt}  max_substeps={sp_def.max_substeps}")
rows_def = sweep(sp_def, "default")

sp_rl = vdsim.SolverParams(); sp_rl.max_substep_dt = 0.001; sp_rl.max_substeps = 64
rows_rl = sweep(sp_rl, "rl-preset")

# barrier cost: repeat=1 (one sync per 5 ms control step) vs repeat=4000
v = vdsim.make_vec_session(ENVS, vp, tp, level=LEVEL, nominal_dt=DT, solver=sp_rl, threads=22)
v.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 10.0)])
v.set_input_all(cmd(0.3, 0.02)); v.tick(DT, 50)
t0 = time.perf_counter()
for _ in range(4000): v.tick(DT, 1)
el = time.perf_counter() - t0
per_step = ENVS * 4000 / el
print(f"[rl-preset] threads=22 repeat=1 (4000 barriers): {per_step:,.0f} steps/s")

json.dump({"default": rows_def, "rl": rows_rl, "rl_repeat1_sps": per_step},
          open("/tmp/r1b_result.json", "w"), indent=1)
