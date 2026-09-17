# Where the 1.0M steps/s target actually sits: substep count and model level.
import json, sys, time
sys.path.insert(0, "/home/ailab-12/git/VDSim/build/python")
import vdsim
vp, tp = vdsim.VehicleParams(), vdsim.TireParams()
DT, ENVS, TH, STEPS = 0.005, 64, 22, 4000
def cmd():
    c = vdsim.CmdL4(); c.throttle = 0.3; c.steer_angle_wheel = 0.02; return c
def run(level, msd):
    sp = vdsim.SolverParams(); sp.max_substep_dt = msd; sp.max_substeps = 64
    v = vdsim.make_vec_session(ENVS, vp, tp, level=level, nominal_dt=DT, solver=sp, threads=TH)
    v.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 10.0)])
    v.set_input_all(cmd()); v.tick(DT, 50)
    t0 = time.perf_counter(); v.tick(DT, STEPS); el = time.perf_counter()-t0
    return ENVS*STEPS/el
out = []
for level in ("L1", "L2", "L3"):
    for msd in (0.001, 0.0025, 0.005):
        sps = run(level, msd)
        sub = max(1, round(DT/msd))
        out.append({"level": level, "max_substep_dt": msd, "substeps": sub, "steps_per_s": sps})
        print(f"{level} substep_dt={msd*1000:.2f}ms (~{sub} sub)  {sps:12,.0f} steps/s"
              f"  = {sps*DT:,.0f} sim-s/wall-s")
json.dump(out, open("/tmp/r1c_result.json","w"), indent=1)
