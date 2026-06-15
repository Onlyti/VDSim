#!/usr/bin/env python3
"""FMU co-simulation: step-steer via VDSim L2 FMU.

Mimics what Simulink's FMU Block does: fmi2DoStep() at fixed intervals,
read outputs, feed inputs. Use this to verify the FMU before importing
into Simulink, or as a lightweight co-sim client without MATLAB.

Run:
    bash fmi_export/build_fmu.sh          # build the FMU once
    PYTHONPATH=build/python python3 examples/simulink/fmu_step_steer.py
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO / "build" / "python"), str(REPO / "python"),
                str(REPO / "fmi_export")]

from fmu_master import FMUMaster

FMU_PATH = REPO / "build" / "fmi_export" / "vdsim_l2.fmu"

if not FMU_PATH.exists():
    print(f"FMU not found: {FMU_PATH}\nRun: bash fmi_export/build_fmu.sh")
    sys.exit(1)

dt = 0.005          # co-simulation step [s]
T  = 8.0            # total time [s]
N  = round(T / dt)
V_TARGET  = 22.2    # [m/s]  ~80 km/h
STEER_AMP = 0.03    # [rad]  wheel steer, applied at t=2 s

fmu = FMUMaster.load(FMU_PATH)
fmu.initialize(0.0)

rows = []
vx_prev = 0.0
for k in range(N):
    t = k * dt
    steer    = STEER_AMP if t >= 2.0 else 0.0
    throttle = max(0.0, min(1.0, (V_TARGET - vx_prev) / 3.0 + 0.05))

    fmu.set("steer_angle_wheel", steer)
    fmu.set("throttle",          throttle)
    fmu.set("brake",             0.0)
    fmu.do_step(t, dt)

    st = fmu.get_many("x_world", "y_world", "yaw", "vx", "vy", "yaw_rate", "ay_body")
    vx_prev = st["vx"]
    rows.append((t, st["x_world"], st["y_world"], st["yaw"],
                 st["vx"], st["vy"], st["yaw_rate"], st["ay_body"]))

fmu.free()

# --- summary ---
peak_ay = max(abs(r[7]) for r in rows)
peak_r  = max(abs(r[6]) for r in rows)
vmax    = max(r[4] for r in rows)
print(f"Steps: {N}  |  vmax={vmax:.2f} m/s  peak_ay={peak_ay:.3f} m/s²  peak_r={peak_r:.4f} rad/s")
print("PASS" if peak_ay > 1.0 else "WARN: very low ay — check steer input")

# --- optional CSV ---
out = REPO / "results" / "fmu_step_steer.csv"
out.parent.mkdir(parents=True, exist_ok=True)
with open(out, "w") as f:
    f.write("t,x,y,yaw,vx,vy,r,ay\n")
    for row in rows:
        f.write(",".join(f"{v:.5f}" for v in row) + "\n")
print(f"CSV → {out}")
