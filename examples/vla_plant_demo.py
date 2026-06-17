#!/usr/bin/env python3
"""Quickstart: closed-loop VDSimPlant (trivial P controller, no MPC)."""
import csv
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

from vdsim_plant import VDSimPlant  # noqa: E402

V0 = 16.7
DT = 0.05
Y_REF = 3.5
KP_Y = 0.012
KP_V = 800.0


def main():
    plant = VDSimPlant(
        config="ioniq5_awd.yaml",
        friction_map=[(120.0, 200.0, 0.5)],
        base_mu=0.9,
        control_dt=DT,
        substep_dt=5e-4,
    )
    plant.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
    rows = []
    obs = None
    for k in range(100):
        if k == 0:
            delta, fx = 0.0, 0.0
        else:
            e_y = Y_REF - obs["Y"]
            delta = max(-0.35, min(0.35, KP_Y * e_y - 0.3 * obs["vy"] / max(V0, 1.0)))
            fx = KP_V * (V0 - obs["vx"])
        obs = plant.step([delta, fx])
        rows.append([
            k * DT, obs["X"], obs["Y"], obs["psi"], obs["vx"], obs["vy"], obs["r"],
            obs["ax"], obs["ay"], obs["beta"],
            *[obs["wheel"][i][k2] for i in range(4) for k2 in ("Fx", "Fy", "Fz")],
        ])
        if k % 20 == 0:
            w0 = obs["wheel"][0]
            print(f"t={k*DT:4.1f} X={obs['X']:6.1f} Y={obs['Y']:5.2f} "
                  f"vx={obs['vx']:5.2f} FL Fx={w0['Fx']:7.0f} Fy={w0['Fy']:7.0f}")

    out = Path("/tmp/vla_plant_demo.csv")
    hdr = ["t", "X", "Y", "psi", "vx", "vy", "r", "ax", "ay", "beta"]
    for name in ("FL", "FR", "RL", "RR"):
        for c in ("Fx", "Fy", "Fz"):
            hdr.append(f"{name}_{c}")
    with out.open("w", newline="") as f:
        csv.writer(f).writerow(hdr)
        csv.writer(f).writerows(rows)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
