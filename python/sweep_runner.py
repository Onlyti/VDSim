"""
VDSim parameter sweep runner.

Usage:
    python3 sweep_runner.py <sweep.yaml> <out_dir>

Sweep YAML schema:
    catalog_vehicle: sedan
    catalog_tire:    default_pacejka
    base_scenario: configs/maneuvers/step_steer.yaml
    binary:        vdsim_l1_vs_l2          # or vdsim_scenario_run
    sweep:
      - param: vehicle.aero_drag_coeff      # dotted path; "vehicle" / "tire" / "scenario"
        values: [0.20, 0.30, 0.40]
      - param: scenario.initial_vx
        values: [5, 10, 15]
    output:
      filename_pattern: "{drag}_{vx}"        # uses sanitized param values
      keep_csv: true

Generates the Cartesian product of all sweep parameters and runs the chosen
binary once per cell, producing one CSV per cell.
"""
import argparse
import itertools
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml


def deep_merge(base: dict, override: dict) -> dict:
    out = dict(base)
    for k, v in override.items():
        out[k] = v
    return out


def _resolve_catalog_base(repo: Path, sweep: dict):
    if not sweep.get("catalog_vehicle"):
        veh = yaml.safe_load((repo / sweep["base_vehicle"]).read_text())
        tire_text = (repo / sweep["base_tire"]).read_text()
        return veh, tire_text
    sys.path.insert(0, str(repo / "python"))
    from catalog import CatalogResolver
    from catalog.ids import blueprint_for_vehicle, tire_id_from_stem
    tire_stem = sweep.get("catalog_tire", "default_pacejka")
    cache = repo / "configs" / ".resolve_cache" / f"sweep_{sweep['catalog_vehicle']}_{tire_stem}"
    cache.mkdir(parents=True, exist_ok=True)
    rv = CatalogResolver(repo).resolve_blueprint(
        blueprint_for_vehicle(sweep["catalog_vehicle"]),
        instance_parts={"tire": tire_id_from_stem(tire_stem)},
        out_dir=cache,
    )
    return yaml.safe_load(rv.vehicle_yaml.read_text()), rv.tire_yaml.read_text()


def set_path(d: dict, path: str, value):
    parts = path.split(".")
    cur = d
    for p in parts[:-1]:
        if p not in cur or not isinstance(cur[p], dict):
            cur[p] = {}
        cur = cur[p]
    cur[parts[-1]] = value


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sweep_yaml")
    ap.add_argument("out_dir")
    args = ap.parse_args()

    sweep = yaml.safe_load(Path(args.sweep_yaml).read_text())
    out_dir = Path(args.out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    repo = Path(__file__).resolve().parent.parent
    bin_path = repo / "build" / "bin" / sweep["binary"]
    if not bin_path.exists():
        print(f"binary not found: {bin_path}", file=sys.stderr)
        sys.exit(1)

    base_vehicle, base_tire = _resolve_catalog_base(repo, sweep)
    base_scenario = yaml.safe_load((repo / sweep["base_scenario"]).read_text())

    sweep_params = sweep["sweep"]
    value_lists = [p["values"] for p in sweep_params]
    names       = [p["param"] for p in sweep_params]

    runs = []
    for cell in itertools.product(*value_lists):
        tag = "_".join(re.sub(r"[^a-zA-Z0-9]", "", f"{v}") for v in cell)
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            veh = dict(base_vehicle); sc = dict(base_scenario)
            for name, value in zip(names, cell):
                target = veh if name.startswith("vehicle.") else \
                         sc  if name.startswith("scenario.") else None
                if target is None:
                    print(f"unsupported sweep target: {name}", file=sys.stderr); sys.exit(2)
                set_path(target, ".".join(name.split(".")[1:]), value)
            veh_path = td / "vehicle.yaml"; veh_path.write_text(yaml.safe_dump(veh))
            tir_path = td / "tire.yaml";    tir_path.write_text(base_tire)
            scn_path = td / "scenario.yaml"; scn_path.write_text(yaml.safe_dump(sc))

            run_out = out_dir / tag
            run_out.mkdir(exist_ok=True)
            if sweep["binary"] == "vdsim_l1_vs_l2":
                cmd = [str(bin_path), str(veh_path), str(tir_path), str(scn_path), str(run_out)]
            else:
                cmd = [str(bin_path), str(veh_path), str(tir_path), str(scn_path),
                       str(run_out / "out.csv")]
            res = subprocess.run(cmd, capture_output=True, text=True)
            runs.append({"tag": tag, "cell": dict(zip(names, cell)),
                         "returncode": res.returncode})
            if res.returncode != 0:
                print(f"FAILED {tag}: {res.stderr}", file=sys.stderr)

    summary_path = out_dir / "sweep_index.yaml"
    summary_path.write_text(yaml.safe_dump(runs))
    print(f"completed {len(runs)} runs; index -> {summary_path}")


if __name__ == "__main__":
    main()
