"""
Convert a measurement CSV (e.g., ADMA / CarMaker export) into a VDSim
scenario YAML.

Usage:
    python3 csv_to_scenario.py <input.csv> <output.yaml> \
            --name my_scenario --vx-col vx --throttle-col throttle ...

Required CSV columns (configurable): time, throttle, brake, steer.
Optional: initial_vx (from first sample of vx_col).
"""
import argparse
import csv
from pathlib import Path
import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_in")
    ap.add_argument("yaml_out")
    ap.add_argument("--name", default="imported")
    ap.add_argument("--time-col", default="time")
    ap.add_argument("--vx-col",   default="vx")
    ap.add_argument("--throttle-col", default="throttle")
    ap.add_argument("--brake-col",    default="brake")
    ap.add_argument("--steer-col",    default="steer")
    ap.add_argument("--interp",  default="linear", choices=["linear", "zoh"])
    ap.add_argument("--subsample", type=int, default=1,
                    help="emit every Nth row (default 1)")
    args = ap.parse_args()

    rows = []
    with open(args.csv_in, newline="") as f:
        rdr = csv.DictReader(f)
        for i, r in enumerate(rdr):
            if i % args.subsample != 0:
                continue
            rows.append({
                "t":        float(r[args.time_col]),
                "throttle": float(r.get(args.throttle_col, 0.0)),
                "brake":    float(r.get(args.brake_col,    0.0)),
                "steer":    float(r.get(args.steer_col,    0.0)),
                "gear":     1,
            })
    if not rows:
        raise SystemExit("CSV produced no rows")

    initial_vx = 0.0
    if args.vx_col:
        with open(args.csv_in, newline="") as f:
            rdr = csv.DictReader(f)
            first = next(rdr)
            initial_vx = float(first.get(args.vx_col, 0.0))

    duration = rows[-1]["t"] - rows[0]["t"]
    dt = (rows[1]["t"] - rows[0]["t"]) if len(rows) > 1 else 0.005

    scenario = {
        "name": args.name,
        "initial_vx": initial_vx,
        "duration": duration,
        "dt": dt,
        "mu": 1.0,
        "interpolation": args.interp,
        "controls": rows,
    }
    Path(args.yaml_out).write_text(yaml.safe_dump(scenario, default_flow_style=None))
    print(f"wrote {len(rows)} samples to {args.yaml_out}")


if __name__ == "__main__":
    main()
