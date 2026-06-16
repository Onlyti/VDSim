#!/usr/bin/env python3
"""VDSim vs CarMaker MF-Tyre/MF-Swift tire-force comparison.

Reads the SAME MF6.x .tir with both solvers over a (Fz, kappa, alpha) grid and
reports pure-longitudinal / pure-lateral relative error.

Prerequisites (a CarMaker-licensed machine):
    CMI=/opt/ipg/carmaker/linux64-12.0.1 ./build.sh    # builds mfs_grid_eval
A modern MF6.x .tir (FITTYP >= 61). MF-Swift 2212 rejects MF5.2 (FITTYP 6) files.
The CarMaker sample Data/Tire/Examples/.../Siemens_car205_60R15.tir works.

Usage:
    python3 compare_vdsim_carmaker.py <tir> [grid.csv]
    # grid.csv defaults to the Chrono parity grid (Fz,kappa,alpha,...).

Apples-to-apples slip: mfs_grid_eval reports the slip CarMaker ACTUALLY used
(varinf 6/7, computed from the effective rolling radius Re), not the prescribed
grid value. Feeding VDSim that same slip removes the Re-vs-unloaded-radius
mismatch, and the two solvers agree to machine precision on steady-state pure
slip (pure-long/pure-lat ~0%). Comparing at the prescribed grid slip instead
inflates the longitudinal error because CarMaker's kappa = (omega*Re - vx)/vx
differs from the unloaded-radius value the grid assumes.
"""
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
GRID_DEFAULT = REPO / "external/chrono_parity/reference/pac02_reference.csv"


def carmaker_eval(tir: str, grid_csv: str) -> dict:
    out = tempfile.NamedTemporaryFile(suffix=".csv", delete=False).name
    exe = HERE / "mfs_grid_eval"
    if not exe.exists():
        sys.exit("mfs_grid_eval not built — run: CMI=<carmaker> ./build.sh")
    # grid columns Fz,kappa,alpha[,...] — pass the file directly (extra cols ignored)
    r = subprocess.run([str(exe), tir, grid_csv, out], capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"CarMaker eval failed:\n{r.stderr}")
    return {(round(float(x['Fz'])), round(float(x['kappa']), 4), round(float(x['alpha']), 4)): x
            for x in csv.DictReader(open(out))}


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    tir = sys.argv[1]
    grid = sys.argv[2] if len(sys.argv) > 2 else str(GRID_DEFAULT)

    sys.path[:0] = [str(REPO / "build" / "python"), str(REPO / "python")]
    import vdsim

    cm = carmaker_eval(tir, grid)   # dict keyed (Fz,kappa,alpha) — actual CarMaker slip
    tire = vdsim.create_magic_formula_tire_from_tir(tir)
    tire.initialize(vdsim.TireParams())

    pl_fx, pl_fy = [], []
    for row in cm.values():
        # kappa/alpha here are the slip CarMaker ACTUALLY used (Re-based).
        Fz, k, a = float(row["Fz"]), float(row["kappa"]), float(row["alpha"])
        cfx, cfy = float(row["Fx"]), float(row["Fy"])
        ti = vdsim.TireInput()
        ti.Fz = Fz; ti.kappa = k; ti.alpha = a; ti.gamma = 0.0
        ti.mu_long = 1.0; ti.mu_lat = 1.0; ti.Vx_wheel = 15.0
        o = tire.compute(ti)
        if abs(a) < 0.02 and abs(k) > 0.02 and abs(cfx) > 200:        # pure longitudinal
            pl_fx.append(abs(o.Fx - cfx) / abs(cfx) * 100)
        if abs(k) < 0.025 and abs(a) > 0.03 and abs(cfy) > 120:       # pure lateral
            pl_fy.append(abs(o.Fy - cfy) / abs(cfy) * 100)

    def stat(lst):
        return (f"n={len(lst):3d}  mean={sum(lst)/len(lst):5.2f}%  max={max(lst):5.2f}%"
                if lst else "n=0")

    print(f"VDSim vs CarMaker MF-Swift (same .tir: {Path(tir).name}, same Re-based slip)")
    print(f"  Pure-long Fx: {stat(pl_fx)}")
    print(f"  Pure-lat  Fy: {stat(pl_fy)}")


if __name__ == "__main__":
    main()
