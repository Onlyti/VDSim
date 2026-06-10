#!/usr/bin/env python3
"""Generate the Chrono Pac02 reference grid for the VDSim parity gate.

ISOLATION: this script is the ONLY place Chrono is touched. It runs in a separate
conda env, imports pychrono, and writes a plain CSV. VDSim's build/core never see
Chrono — the C++ gate (tests/parity/test_chrono_pac02_parity.cpp) only reads the CSV.

It loads the SAME public .tir that VDSim's evaluator uses (sample_pac02.tir), drives
Chrono's file-driven Pac02 tire over a (Fz, kappa, alpha) grid on a flat terrain, and
records Fx/Fy/Mz in the ISO tire frame.

ENV REQUIREMENT (important):
  Needs a pychrono with the *file-driven* Pac02 tire (ReadTireJSON / Pac02Tire(json)),
  i.e. pychrono >= 8.0. That conda binary requires GLIBC >= 2.32, so it does NOT run
  on Ubuntu 20.04 (GLIBC 2.31). On such a host, generate the CSV on a newer machine,
  or build Chrono from source (its locally-compiled libs match the host GLIBC).
  pychrono 7.0 (conda-forge, GLIBC-compatible with 20.04) exposes only abstract
  ChPac02Tire with no file loader, so it cannot consume an arbitrary .tir.

Usage:
  conda activate <env-with-pychrono>=8
  python gen_pac02_reference.py            # writes reference/pac02_reference.csv
"""
import csv
import json
import math
import os
from pathlib import Path

import pychrono as chrono
import pychrono.vehicle as veh

HERE = Path(__file__).resolve().parent
TIR = HERE / "sample_pac02.tir"
OUT = HERE / "reference" / "pac02_reference.csv"

# Grid: pure-long, pure-lat, and combined cells, at three vertical loads.
FZ_LIST = [2000.0, 4000.0, 6000.0]
KAPPA_LIST = [-0.15, -0.08, -0.03, 0.0, 0.03, 0.08, 0.15]
ALPHA_LIST = [math.radians(a) for a in (-8, -4, -1, 0, 1, 4, 8)]
VX = 16.0          # m/s, matches LONGVL in the .tir
R_UNLOADED = 0.31  # m, matches UNLOADED_RADIUS


def write_tire_json() -> Path:
    """Minimal Chrono tire JSON wrapper pointing at the shared .tir."""
    j = HERE / "_pac02_wrapper.json"
    j.write_text(json.dumps({
        "Name": "sample_pac02",
        "Type": "Tire",
        "Template": "Pac02Tire",
        "Tire Parameter File": str(TIR),
    }, indent=2))
    return j


def build_rig():
    """A single spindle body + flat terrain + file-driven Pac02 tire."""
    sys = chrono.ChSystemNSC()
    sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    spindle = chrono.ChBody()
    spindle.SetMass(20.0)
    sys.Add(spindle)
    wheel = veh.ChWheel(chrono.GetChronoDataFile("vehicle/generic/wheel/WheelSimple.json")) \
        if False else None

    tire = veh.ReadTireJSON(str(write_tire_json()))   # pychrono >= 8
    terrain = veh.RigidTerrain(sys)
    patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                             chrono.ChVectorD(200, 20, 0.1), True)
    terrain.Initialize()
    return sys, spindle, tire, terrain


def impose_state(spindle, Fz, kappa, alpha):
    """Set the spindle pose/velocity to realise (Fz via height, kappa, alpha).

    Fz is set by the vertical deflection: defl = Fz / k_vert; ride height = R - defl.
    kappa = (omega*R - Vx)/Vx  ->  omega = Vx*(1+kappa)/R
    alpha = atan(Vy/Vx)        ->  Vy = Vx*tan(alpha)
    """
    k_vert = 220000.0
    defl = Fz / k_vert
    z = R_UNLOADED - defl
    spindle.SetPos(chrono.ChVectorD(0, 0, z))
    Vy = VX * math.tan(alpha)
    spindle.SetPos_dt(chrono.ChVectorD(VX, Vy, 0))
    omega = VX * (1.0 + kappa) / R_UNLOADED
    spindle.SetWvel_loc(chrono.ChVectorD(0, omega, 0))


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    sys, spindle, tire, terrain = build_rig()
    tire.Initialize(spindle, veh.LEFT)

    rows = []
    t = 0.0
    dt = 1e-3
    for Fz in FZ_LIST:
        for kappa in KAPPA_LIST:
            for alpha in ALPHA_LIST:
                impose_state(spindle, Fz, kappa, alpha)
                # settle the internal relaxation a few steps at frozen state
                for _ in range(50):
                    tire.Synchronize(t, terrain)
                    tire.Advance(dt)
                    t += dt
                f = tire.ReportTireForce(terrain)
                force = f.force
                moment = f.moment
                rows.append([Fz, kappa, alpha, 0.0, force.x, force.y, moment.z])

    with open(OUT, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["Fz", "kappa", "alpha", "gamma", "Fx", "Fy", "Mz"])
        w.writerows(rows)
    print(f"[gen] wrote {len(rows)} rows -> {OUT}")


if __name__ == "__main__":
    main()
