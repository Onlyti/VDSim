#!/usr/bin/env python3
"""Convert vdsim_lab CSV output to MATLAB .mat file for data exchange.

Usage:
    python3 tools/mat_export.py results/campaign/accel_brake/run.csv
    python3 tools/mat_export.py run.csv --out=run.mat
    python3 tools/mat_export.py run.csv --signals=t,vx,ay,r
"""
import argparse
import csv
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python")]


def csv_to_mat(csv_path: Path, mat_path: Path, signals: list | None = None):
    try:
        import scipy.io
    except ImportError:
        print("scipy not installed — run: pip install scipy")
        sys.exit(1)

    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        print(f"Empty CSV: {csv_path}"); sys.exit(1)

    available = list(rows[0].keys())
    cols = signals if signals else available
    missing = [c for c in cols if c not in available]
    if missing:
        print(f"Signals not in CSV: {missing}\nAvailable: {available}")
        sys.exit(1)

    data = {col: [float(r[col]) for r in rows] for col in cols}
    scipy.io.savemat(mat_path, data)
    print(f"Wrote {len(cols)} signals × {len(rows)} steps → {mat_path}")


def main():
    p = argparse.ArgumentParser(description="VDSim CSV → MATLAB .mat")
    p.add_argument("csv", help="input CSV (vdsim_lab output)")
    p.add_argument("--out", help="output .mat path (default: same stem)")
    p.add_argument("--signals", help="comma-separated signal names (default: all)")
    args = p.parse_args()

    csv_path = Path(args.csv)
    mat_path = Path(args.out) if args.out else csv_path.with_suffix(".mat")
    signals  = args.signals.split(",") if args.signals else None
    csv_to_mat(csv_path, mat_path, signals)


if __name__ == "__main__":
    main()
