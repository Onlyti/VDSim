#!/usr/bin/env python3
"""Multi-vehicle comparison — run the ISO maneuver set across several vehicle
presets and tabulate the objective metrics side by side (vehicle evaluation).

Multi-vehicle in VDSim is for *comparison*, not interaction: each vehicle runs the
same maneuver independently and the metrics are placed next to each other.

    python3 tools/vdsim_compare.py sedan sports race_car
    python3 tools/vdsim_compare.py sedan sports --tire sport_grip   # common tire
    python3 tools/vdsim_compare.py sedan sports --maneuvers step_steer skidpad \
        --out results/compare

Each preset may be a catalog stem (sedan / sports / race_car / fsk_formula) or a
full blueprint id (vehicle.<name>). By default each vehicle uses its own blueprint
tire (as-built); `--tire <stem>` forces a common tire so the comparison isolates the
chassis / mass / drivetrain differences.
"""
import argparse
import csv
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(REPO / "examples"))

import vdsim  # noqa: E402
import maneuvers as M  # noqa: E402

# maneuver key -> (callable, headline numeric metric for the chart)
MANEUVERS = {
    "step_steer": (M.step_steer, "r_ss[rad/s]"),
    "skidpad":    (M.skidpad_understeer, "K_us[deg/g]"),
    "dlc":        (M.double_lane_change, "peak_ay[g]"),
}


def resolve_vehicle(preset, tire=None, level="L2"):
    """Resolve a preset (stem or vehicle.<id>) to (VehicleParams, TireParams, level)."""
    from catalog import CatalogResolver
    from catalog.ids import blueprint_for_vehicle, tire_id_from_stem
    bid = str(preset) if str(preset).startswith("vehicle.") \
        else blueprint_for_vehicle(str(preset), level)
    r = CatalogResolver(REPO)
    inst = {"tire": tire_id_from_stem(tire)} if tire else None
    rv = r.resolve_blueprint(bid, instance_parts=inst,
                             out_dir=Path(tempfile.mkdtemp(prefix="vdsim_cmp_")))
    vp = vdsim.VehicleParams.from_yaml(str(rv.vehicle_yaml))
    tp = vdsim.TireParams.from_yaml(str(rv.tire_yaml))
    return vp, tp, rv.level


def run_compare(presets, maneuvers, tire=None, level="L2"):
    """Return a list of per-vehicle metric rows (dict keyed vehicle + maneuver.metric)."""
    rows = []
    for p in presets:
        vp, tp, lvl = resolve_vehicle(p, tire, level)
        rec = {"vehicle": str(p)}
        for mname in maneuvers:
            fn, _ = MANEUVERS[mname]
            res = fn(level=lvl, veh=(vp, tp))
            for k, v in res.items():
                if k == "maneuver":
                    continue
                rec[f"{mname}.{k}"] = v
        rows.append(rec)
    return rows


def run_traces(presets, tire=None, level="L2"):
    """Per-vehicle step-steer yaw-rate(t) trace for overlay charts:
    {vehicle: {"t": [...], "r": [...]}}."""
    out = {}
    for p in presets:
        vp, tp, lvl = resolve_vehicle(p, tire, level)
        res = M.step_steer(level=lvl, veh=(vp, tp), trace=True)
        if res.get("trace"):
            out[str(p)] = res["trace"]
    return out


def _columns(rows):
    cols = []
    for rec in rows:
        for k in rec:
            if k != "vehicle" and k not in cols:
                cols.append(k)
    return cols


def print_table(rows):
    cols = _columns(rows)
    w0 = max([len("vehicle")] + [len(r["vehicle"]) for r in rows])
    widths = {c: max(len(c), *(len(str(r.get(c, ""))) for r in rows)) for c in cols}
    head = "vehicle".ljust(w0) + "  " + "  ".join(c.rjust(widths[c]) for c in cols)
    print(head)
    print("-" * len(head))
    for r in rows:
        print(r["vehicle"].ljust(w0) + "  "
              + "  ".join(str(r.get(c, "")).rjust(widths[c]) for c in cols))


def write_csv(rows, path):
    cols = ["vehicle"] + _columns(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def write_figure(rows, maneuvers, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figs = [m for m in maneuvers if m in MANEUVERS]
    fig, axes = plt.subplots(1, len(figs), figsize=(4.2 * len(figs), 3.6), squeeze=False)
    names = [r["vehicle"] for r in rows]
    for ax, m in zip(axes[0], figs):
        metric = MANEUVERS[m][1]
        key = f"{m}.{metric}"
        vals = [float(r.get(key, float("nan"))) for r in rows]
        ax.bar(range(len(names)), vals, color="#4F81BD")
        ax.set_xticks(range(len(names)))
        ax.set_xticklabels(names, rotation=20, ha="right", fontsize=8)
        ax.set_title(f"{m}\n{metric}", fontsize=9)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("Vehicle comparison — ISO maneuver metrics", fontsize=11)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("presets", nargs="+", help="vehicle stems or vehicle.<id>")
    ap.add_argument("--maneuvers", nargs="+", default=list(MANEUVERS),
                    choices=list(MANEUVERS))
    ap.add_argument("--tire", default=None, help="common tire stem (else each as-built)")
    ap.add_argument("--level", default="L2")
    ap.add_argument("--out", default=None, help="output dir for compare.csv + compare.png")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    rows = run_compare(args.presets, args.maneuvers, args.tire, args.level)
    print_table(rows)
    if args.out:
        out = Path(args.out)
        write_csv(rows, out / "compare.csv")
        print(f"\n[compare] wrote {out / 'compare.csv'}")
        if not args.no_plot:
            try:
                write_figure(rows, args.maneuvers, out / "compare.png")
                print(f"[compare] wrote {out / 'compare.png'}")
            except Exception as e:  # matplotlib optional
                print(f"[compare] plot skipped: {e}")


if __name__ == "__main__":
    main()
