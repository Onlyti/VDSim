#!/usr/bin/env python3
"""Road-roughness presets: build a VDSim ground from a PSD preset YAML or a
measured (n, Gd) spectrum, and (demo) drive L3 over each to show the ride
response per surface.

Two PSD families (see configs/roads/*.yaml):
  - analytic: Gd(n) = gd_n0 (n/0.1)^-w, optional dual-slope (w below n_break,
    waviness_high above) for surface-specific short-wavelength content.
  - table:    measured (n, Gd) pairs, log-log interpolated (plug an RLDA spectrum).

Usage:
    python3 examples/road_profile.py                 # demo all configs/roads/*.yaml
    python3 examples/road_profile.py configs/roads/belgian_pave.yaml
    python3 examples/road_profile.py --csv my_rlda_psd.csv   # n,Gd columns
"""
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
import vdsim  # noqa: E402


def ground_from_preset(path, z=0.0, seed=1):
    """Build a contact-provider ground from a road preset YAML."""
    cfg = yaml.safe_load(open(path))
    mu = float(cfg.get("mu", 1.0))
    p = cfg["psd"]
    if p.get("type") == "table":
        return vdsim.create_psd_ground_table(
            z=z, mu=mu, n=[float(v) for v in p["n"]], gd=[float(v) for v in p["gd"]],
            n_max=float(p.get("n_max", 10.0)), seed=seed), mu
    return vdsim.create_psd_ground(
        z=z, mu=mu, gd_n0=float(p["gd_n0"]), waviness=float(p.get("waviness", 2.0)),
        n_break=float(p.get("n_break", 0.0)),
        waviness_high=float(p.get("waviness_high", 2.0)),
        n_max=float(p.get("n_max", 4.0)), seed=seed), mu


def ground_from_csv(path, mu=1.0, z=0.0, seed=1):
    """Build a ground from a measured PSD CSV with `n,Gd` columns (RLDA)."""
    import csv
    n, gd = [], []
    with open(path) as f:
        for row in csv.reader(f):
            try:
                n.append(float(row[0])); gd.append(float(row[1]))
            except (ValueError, IndexError):
                continue
        return vdsim.create_psd_ground_table(z=z, mu=mu, n=n, gd=gd, seed=seed)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if "--csv" in sys.argv:
        i = sys.argv.index("--csv")
        print("CSV ground built:", ground_from_csv(sys.argv[i + 1]) is not None)
        return
    paths = [Path(args[0])] if args else sorted((REPO / "configs/roads").glob("*.yaml"))
    print("=== ride response per road preset (L3 sedan @ 20 m/s straight, 6 s) ===")
    import statistics
    vp = vdsim.VehicleParams.from_yaml(str(REPO / "configs/vehicles/sedan.yaml"))
    tp = vdsim.TireParams.from_yaml(str(REPO / "configs/tires/default_pacejka.yaml"))
    for path in paths:
        cfg = yaml.safe_load(open(path)); p = cfg["psd"]; mu = float(cfg.get("mu", 1.0))
        if p.get("type") == "table":
            sess = vdsim.make_sim_session_psd(
                vp, tp, "L3", mu=mu, n=[float(v) for v in p["n"]],
                gd=[float(v) for v in p["gd"]], n_max=float(p.get("n_max", 10.0)),
                nominal_dt=0.002)
            desc = f"table[{len(p['n'])} pts]"
        else:
            sess = vdsim.make_sim_session_psd(
                vp, tp, "L3", mu=mu, gd_n0=float(p["gd_n0"]),
                waviness=float(p.get("waviness", 2.0)), n_break=float(p.get("n_break", 0.0)),
                waviness_high=float(p.get("waviness_high", 2.0)),
                n_max=float(p.get("n_max", 4.0)), nominal_dt=0.002)
            desc = (f"gd0={p['gd_n0']:.0e} w={p.get('waviness',2)}"
                    + (f"/{p['waviness_high']}@{p['n_break']}" if p.get("n_break") else ""))
        s = vdsim.make_init_state(x=0, y=0, yaw=0, v=20.0,
                                  wheel_radius=vp.wheel_radius_nominal)
        sess.reset(s)
        c = vdsim.CmdL4(); c.throttle = 0.25
        fz, az = [], []
        for k in range(3000):
            sess.set_input(c); sess.tick(0.002)
            if k > 500:
                o = sess.output(); fz.append(o.Fz[0]); az.append(o.ay)
        print(f"  {path.stem:16s} mu={mu:.2f}  {desc:26s} "
              f"FL Fz std {statistics.pstdev(fz):5.1f} N")


if __name__ == "__main__":
    main()
