#!/usr/bin/env python3
"""Compare default blend vs LuGre on a mild grade (parked hold / creep)."""
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

import vdsim
from vdsim_lab import Experiment, Maneuver, Road, Tire, Vehicle


def grade_creep(tp, grade=0.08, duration=3.0):
    tp.rolling_resistance = 0.0
    veh = Vehicle.preset("sedan")
    veh.vp.aero_drag_coeff = 0.0
    idle = Maneuver(lambda _k, _o, _vp: vdsim.CmdL4(), init_v=0.0)
    res = (Experiment(level="L2", dt=0.005)
           .vehicle(veh)
           .tire(Tire(tp))
           .road(Road.inclined(grade=grade, mu=1.0))
           .maneuver(idle)
           .run(duration))
    return abs(res.col("vx")[-1])


def main():
    tp_blend = Tire.preset().tp
    tp_lugre = Tire.preset().lugre(True, sigma0=3.0e5, sigma2=120.0).tp

    vx_blend = grade_creep(tp_blend)
    vx_lugre = grade_creep(tp_lugre)
    print("grade hold demo (8% grade, 3 s, zero throttle/brake)")
    print(f"  blend (default): |vx| = {vx_blend:.4f} m/s")
    print(f"  LuGre on:        |vx| = {vx_lugre:.4f} m/s")
    print()
    print("Realtime scene: build/bin/vdsim_realtime --scene=configs/scenes/lugre_grade_demo.yaml")
    print("CLI override:   ... --scene=configs/scenes/l3_sedan_kinematics.yaml --lugre")
    print("Python API:     Tire.preset().lugre(True)")


if __name__ == "__main__":
    main()
