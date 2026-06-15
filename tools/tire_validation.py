#!/usr/bin/env python3
"""VDSim tire validation tool — compare MF2002 evaluator against reference data.

Usage:
    python3 tools/tire_validation.py --tir <file.tir> --ref <reference.csv>

    # Built-in example (Chrono Pac02 reference):
    python3 tools/tire_validation.py \\
        --tir external/chrono_parity/sample_pac02.tir \\
        --ref external/chrono_parity/reference/pac02_reference.csv

Reference CSV format (header row required):
    Fz,kappa,alpha,gamma,Fx,Fy,Mz
    2000,-0.15,-0.12,0,-2300,1800,-5.1

Regimes evaluated (same split as ctest ChronoPac02Parity):
    pure-long: |alpha| < 0.02 rad
    pure-lat:  |kappa| < 0.025, |alpha| > 0.03 rad
    combined:  |kappa| > 0.025 and |alpha| > 0.02 rad

Output:
    results/tire_validation/<stem>.csv    per-point comparison
    results/tire_validation/summary.json  regime error table
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO / "build" / "python"), str(REPO / "python")]


def load_reference(ref_path: Path):
    rows = list(csv.DictReader(open(ref_path)))
    if not rows:
        raise ValueError(f"Empty reference CSV: {ref_path}")
    required = {"Fz", "kappa", "alpha", "Fx", "Fy"}
    missing = required - set(rows[0].keys())
    if missing:
        raise ValueError(f"Reference CSV missing columns: {missing}")
    return rows


def evaluate_vdsim(tir_path: str, ref_rows):
    import vdsim
    tire = vdsim.create_magic_formula_tire_from_tir(tir_path)
    tire.initialize(vdsim.TireParams())

    results = []
    for r in ref_rows:
        Fz  = float(r["Fz"])
        k   = float(r["kappa"])
        a   = float(r["alpha"])
        ref_fx = float(r["Fx"])
        ref_fy = float(r["Fy"])

        ti = vdsim.TireInput()
        ti.Fz = Fz; ti.kappa = k; ti.alpha = a; ti.gamma = float(r.get("gamma", 0.0))
        ti.mu_long = 1.0; ti.mu_lat = 1.0; ti.Vx_wheel = 15.0
        o = tire.compute(ti)

        def pct(got, ref):
            return abs(got - ref) / max(abs(ref), 1.0) * 100

        results.append({
            "Fz":         Fz, "kappa": k, "alpha": a,
            "Fx_ref":     ref_fx, "Fy_ref": ref_fy,
            "Fx_vdsim":   o.Fx,   "Fy_vdsim": o.Fy,
            "err_Fx_pct": pct(o.Fx, ref_fx),
            "err_Fy_pct": pct(o.Fy, ref_fy),
        })
    return results


def compute_summary(results):
    pl_fx, pl_fy, cb_fx, cb_fy = [], [], [], []
    for r in results:
        k, a = r["kappa"], r["alpha"]
        ex, ey = r["err_Fx_pct"], r["err_Fy_pct"]
        if math.isnan(ex): continue
        rfx, rfy = abs(r["Fx_ref"]), abs(r["Fy_ref"])
        if abs(a) < 0.02  and rfx > 120:                         pl_fx.append(ex)
        if abs(k) < 0.025 and abs(a) > 0.03 and rfy > 120:      pl_fy.append(ey)
        if abs(k) > 0.025 and abs(a) > 0.02:
            if rfx > 120: cb_fx.append(ex)
            if rfy > 120: cb_fy.append(ey)

    def stat(lst, label):
        if not lst:
            return {"n": 0, "mean_pct": None, "max_pct": None, "gate_pct": None}
        gate = 6.0 if "pure" in label else None
        return {"n": len(lst), "mean_pct": round(sum(lst)/len(lst), 2),
                "max_pct": round(max(lst), 2), "gate_pct": gate}

    return {
        "pure_longitudinal_Fx": stat(pl_fx, "pure"),
        "pure_lateral_Fy":      stat(pl_fy, "pure"),
        "combined_Fx":          stat(cb_fx, "combined"),
        "combined_Fy":          stat(cb_fy, "combined"),
        "note": "gate_pct=6 means ctest enforces <6% rel error (floor 120N) for pure regimes",
    }


def main():
    p = argparse.ArgumentParser(description="VDSim MF2002 tire validation")
    p.add_argument("--tir", required=True, help=".tir file (Pacejka MF2002)")
    p.add_argument("--ref", required=True, help="Reference CSV (Fz,kappa,alpha,Fx,Fy,...)")
    p.add_argument("--out", default="results/tire_validation")
    args = p.parse_args()

    tir_path = Path(args.tir)
    ref_path = Path(args.ref)
    out_dir  = REPO / args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Tire:      {tir_path}")
    print(f"Reference: {ref_path}")

    ref_rows = load_reference(ref_path)
    print(f"Reference: {len(ref_rows)} points")

    results  = evaluate_vdsim(str(tir_path), ref_rows)
    summary  = compute_summary(results)

    stem = f"{tir_path.stem}_vs_{ref_path.stem}"
    out_csv = out_dir / f"{stem}.csv"
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader(); w.writerows(results)

    out_json = out_dir / "summary.json"
    payload  = {"tir": str(tir_path), "ref": str(ref_path), **summary}
    with open(out_json, "w") as f:
        json.dump(payload, f, indent=2)

    # Print table
    print("\nRegime               | n   | mean % | max %  | gate %")
    print("---------------------|-----|--------|--------|-------")
    for key, label in [("pure_longitudinal_Fx","Pure-long Fx"),
                       ("pure_lateral_Fy",     "Pure-lat  Fy"),
                       ("combined_Fx",         "Combined  Fx"),
                       ("combined_Fy",         "Combined  Fy")]:
        s = summary[key]
        mean = f"{s['mean_pct']:.1f}" if s['mean_pct'] is not None else "-"
        mx   = f"{s['max_pct']:.1f}"  if s['max_pct']  is not None else "-"
        gate = f"{s['gate_pct']:.0f}" if s['gate_pct']  is not None else "n/a"
        print(f"{label:20s} | {s['n']:3d} | {mean:6s} | {mx:6s} | {gate:6s}")

    print(f"\nCSV     → {out_csv}")
    print(f"Summary → {out_json}")


if __name__ == "__main__":
    main()
