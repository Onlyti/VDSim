# R7/R8 verification: settle-on-ground respawn, and snapshot/restore fidelity.
import json, sys
sys.path.insert(0, "/home/ailab-12/git/VDSim/build/python")
sys.path.insert(0, "/home/ailab-12/git/VDSim/python")
import numpy as np
import vdsim

res = {}
DT = 0.005

# ---------------- R7: spawn settling on non-flat ground ----------------
GRADE = 0.10                       # 10 % climb -> surface z = 0.1 * x
vp = vdsim.VehicleParams()
vs = vdsim.make_vec_session(2, vp, vdsim.TireParams(), level="L2",
                            nominal_dt=DT, grade=GRADE, threads=1)
spawn = vdsim.make_init_state(100.0, 0.0, 0.0, 10.0)     # z left at 0
settled = vs.at(0).settle_on_ground(spawn)
surface_z = GRADE * 100.0
print(f"R7 spawn x=100 on a {GRADE*100:.0f}% grade: raw z={spawn.position[2]:.3f} -> "
      f"settled z={settled.position[2]:.3f} (surface {surface_z:.3f}, "
      f"cg_height {vp.cg_height:.3f}), vz={settled.velocity[2]:.3f}")
res["r7"] = {"raw_z": float(spawn.position[2]), "settled_z": float(settled.position[2]),
             "surface_z": surface_z, "cg_height": float(vp.cg_height)}
assert abs(settled.position[2] - (surface_z + vp.cg_height)) < 0.30, "settle missed the surface"
assert settled.velocity[2] == 0.0

# settled vs raw respawn: the raw one starts buried and kicks
def respawn_jolt(settle, level="L3"):
    # L2 is planar (z unused); the spawn height only matters from L3 (ride) up.
    v = vdsim.make_vec_session(1, vp, vdsim.TireParams(), level=level,
                               nominal_dt=DT, grade=GRADE, threads=1)
    v.reset_all([vdsim.make_init_state(100.0, 0.0, 0.0, 10.0)], settle)
    peak = 0.0
    for _ in range(100):
        v.tick(DT, 1)
        peak = max(peak, abs(v.outputs()[0].Fz[0]))
    return peak

j_raw, j_settled = respawn_jolt(False), respawn_jolt(True)
print(f"R7 (L3) first 0.5 s peak |Fz_FL|: raw spawn {j_raw:12,.0f} N | "
      f"settled {j_settled:9,.0f} N")
res["r7_peak_Fz"] = {"raw": j_raw, "settled": j_settled}
# Observation, not an assertion: the ride models re-seat their own suspension
# on reset, so peak Fz is the same either way -- what settling fixes is the
# reported pose (z, and therefore any z-dependent observation or reward).

# Settled z must track the surface at every x, not just one point.
errs = []
for x in (0.0, 25.0, 50.0, 100.0, 200.0):
    s = vdsim.make_init_state(x, 0.0, 0.0, 10.0)
    s = vs.at(0).settle_on_ground(s)
    errs.append(abs(s.position[2] - (GRADE * x + vp.cg_height)))
print("R7 settled z error vs (surface + cg_height) at x=0,25,50,100,200 m: "
      + ", ".join(f"{e:.3f}" for e in errs) + " m")
res["r7_settle_err_m"] = errs
# The moved algorithm searches z in 0.04 / 0.08 m steps and stops at first
# contact, so it lands within one step of the surface -- it is a respawn seed,
# not a static-equilibrium solve.
assert max(errs) < 0.20, "settled height does not track the surface"

# ---------------- R8: snapshot / restore ----------------
def make_session(level):
    tp = vdsim.TireParams()
    tp.relaxation_length_lat = 0.5       # tire transient state ON
    tp.relaxation_length_long = 0.3
    act = vdsim.ActuatorParams()
    act.steer.ch.dead_time_s = 0.04      # transport ring
    act.steer.ch.tau_s = 0.08            # lag memory
    act.steer.ch.rate_limit = 4.0        # rate-limit memory
    act.steer.friction.enabled = True    # LuGre bristle state
    act.throttle.dead_time_s = 0.03
    act.throttle.tau_s = 0.05
    act.brake.ch.dead_time_s = 0.02
    act.brake.thermal_enabled = True     # brake temperature state
    act.brake.heat_coeff = 0.5
    act.brake.cool_coeff = 0.02
    sens = vdsim.SensorParams(); sens.enabled = True
    v = vdsim.make_vec_session(1, vdsim.VehicleParams(), tp, level=level,
                               nominal_dt=DT, sensor_delay_s=0.08,   # delay line
                               actuator=act, sensors=sens, threads=1)
    return v

def actions(k):
    c = vdsim.CmdL4()
    c.steer_angle_wheel = 0.08 * np.sin(k * 0.05)
    c.throttle = 0.4 + 0.3 * np.sin(k * 0.03)
    return c

def probe(v):
    o = v.outputs()[0]
    s = o.state
    return np.array([s.position[0], s.position[1], s.vx(), s.vy(), s.yaw_rate(),
                     o.ay, o.steer_applied, o.throttle_applied,
                     *o.slip_ratio, *o.slip_angle, *o.Fz,
                     o.sensors.ax, o.sensors.steer])

res["r8"] = {}
for level in ("L1", "L2", "L3"):
    v = make_session(level)
    v.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 15.0)])
    for k in range(200):
        v.set_input_all(actions(k)); v.tick(DT, 1)

    snap = v.snapshots()
    state_only = v.states()[0]

    run_a = []
    for k in range(200, 300):
        v.set_input_all(actions(k)); v.tick(DT, 1); run_a.append(probe(v))
    run_a = np.stack(run_a)

    v.restore(snap)
    run_b = []
    for k in range(200, 300):
        v.set_input_all(actions(k)); v.tick(DT, 1); run_b.append(probe(v))
    run_b = np.stack(run_b)

    v.reset_all([state_only])          # control: State only, no memory
    run_c = []
    for k in range(200, 300):
        v.set_input_all(actions(k)); v.tick(DT, 1); run_c.append(probe(v))
    run_c = np.stack(run_c)

    bitwise = np.array_equal(run_a, run_b)
    state_only_err = float(np.abs(run_a - run_c).max())
    blob = len(snap[0])
    print(f"R8 {level}: snapshot {blob} doubles + rng {len(snap[0].rng)} chars | "
          f"restore bitwise = {bitwise} | State-only restore max err = {state_only_err:.4g}")
    res["r8"][level] = {"bitwise": bool(bitwise), "blob_doubles": blob,
                        "state_only_max_err": state_only_err}
    assert bitwise, f"{level} snapshot/restore is not bitwise"
    assert state_only_err > 0.0, f"{level}: State-only restore looked identical"

# pickling a snapshot (checkpoint to disk)
import pickle
v = make_session("L2")
v.reset_all([vdsim.make_init_state(0.0, 0.0, 0.0, 15.0)])
for k in range(50):
    v.set_input_all(actions(k)); v.tick(DT, 1)
blob = pickle.dumps(v.snapshots()[0])
v.restore([pickle.loads(blob)])
print(f"R8 pickle round-trip: {len(blob)} bytes, restore ok")
res["r8_pickle_bytes"] = len(blob)

json.dump(res, open("/tmp/r78.json", "w"), indent=1, default=str)
print("ALL CHECKS PASSED")
