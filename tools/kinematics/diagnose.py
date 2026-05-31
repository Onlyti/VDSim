"""
Hardpoint self-consistency diagnostic.

Loads a suspension YAML, dispatches to the appropriate 3D solver based on
the `type:` field, sweeps wheel travel ±range, and reports:
    - Solver success rate
    - Max wheel-z error
    - Smoothness check (residual derivatives)
    - Geometric sanity (IC location for DW, strut tilt for MP, etc.)
    - Range of camber/toe/track gains

Exit code 0 = consistent, 1 = problems detected.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml


def _import(name):
    """Lazy-import a solver module from tools/kinematics."""
    import importlib, sys
    sys.path.insert(0, str(Path(__file__).parent))
    return importlib.import_module(name)


SOLVER_MAP = {
    "double_wishbone": ("dw_3d_solver", "DW3DSolver"),
    "macpherson":      ("mp_3d_solver", "MPSolver"),
    "trailing_arm":    ("ta_3d_solver", "TASolver"),
    "five_link":       ("fivelink_3d_solver", "FiveLinkSolver"),
}


def diagnose(yaml_path: Path,
             travel_range: float = 0.08,
             n_travel: int = 17,
             tol_wz: float = 1e-4) -> int:
    with open(yaml_path) as f: hp = yaml.safe_load(f)
    typ = hp.get("type")
    if typ not in SOLVER_MAP:
        print(f"[diagnose] unknown suspension type: {typ}", file=sys.stderr)
        return 1
    mod_name, cls_name = SOLVER_MAP[typ]
    mod = _import(mod_name)
    solver = getattr(mod, cls_name)(hp)

    print(f"=== Hardpoint diagnostic: {yaml_path.name} ===")
    print(f"  type = {typ}, side = {hp.get('side','?')}")
    print(f"  sweep ±{travel_range*1000:.0f} mm, n = {n_travel}")
    print()

    travels = np.linspace(-travel_range, travel_range, n_travel)
    n_failed = 0
    max_wz_err = 0.0
    cambers, toes, tracks = [], [], []
    prev = None
    for t in travels:
        # Build kwargs depending on solver signature (some accept x0_*).
        kwargs = {}
        if "x0_axis_angle" in solver.solve.__code__.co_varnames:
            kwargs["x0_axis_angle"] = prev
        elif "x0_pose" in solver.solve.__code__.co_varnames:
            kwargs["x0_pose"] = prev
        r = solver.solve(t, 0.0, **kwargs) if kwargs else solver.solve(t, 0.0)
        if not r.get("valid"):
            n_failed += 1
            print(f"  travel={t*1000:+5.1f}mm: FAILED")
            continue
        actual_wz = r["wheel_pos"][2]
        target_wz = solver.wheel_static[2] + t
        wz_err = abs(actual_wz - target_wz)
        max_wz_err = max(max_wz_err, wz_err)
        cambers.append(r["camber"])
        toes.append(r["toe"])
        tracks.append(r["track_change"])
        # Carry continuation state
        if hasattr(solver, "_last_axis_angle"):
            prev = solver._last_axis_angle
        elif hasattr(solver, "_last_pose"):
            prev = solver._last_pose

    cambers = np.array(cambers); toes = np.array(toes); tracks = np.array(tracks)

    print(f"  Solver success rate     : {n_travel - n_failed} / {n_travel}")
    print(f"  Max wheel-z error       : {max_wz_err*1e6:.3f} μm")
    if n_failed == 0 and len(cambers) >= 3:
        # Monotone check (after sorting by travel)
        idx = np.argsort(travels[:len(cambers)])
        cam_sorted = cambers[idx]
        diffs = np.diff(cam_sorted)
        monotone = (diffs >= -1e-6).all() or (diffs <= 1e-6).all()
        print(f"  Camber monotone in travel: {monotone}")
        print(f"  Camber gain (avg)       : "
              f"{math.degrees(cambers[-1] - cambers[0]) / (2*travel_range*1000):.4f} °/mm")
        print(f"  Toe gain (avg)          : "
              f"{math.degrees(toes[-1] - toes[0]) / (2*travel_range*1000):.4f} °/mm")
        print(f"  Track gain (avg)        : "
              f"{(tracks[-1] - tracks[0])*1000 / (2*travel_range*1000):.4f} mm/mm")

    # Suspension-specific extras
    if typ == "double_wishbone":
        # IC check (2D approx)
        from double_wishbone import DWAnalyzer
        an = DWAnalyzer(hp)
        sm = an.summary()
        if sm["instant_center_valid"]:
            ic = sm["instant_center_yz"]
            rc = sm["roll_center_height_m"]
            print(f"  Instant center (y,z)    : ({ic[0]:.3f}, {ic[1]:.3f}) m")
            print(f"  Roll center height      : {rc:.3f} m" if rc is not None else "  RC: undefined")
        else:
            print("  Instant center          : UCA ∥ LCA — degenerate geometry")
    elif typ == "macpherson":
        # Tube tilt angle
        td = solver.tube_axis_body
        tilt = math.degrees(math.atan2(math.hypot(td[0], td[1]), td[2]))
        print(f"  Strut tilt from vertical: {tilt:.2f}°")
    elif typ == "trailing_arm":
        # Semi-trailing tilt
        ax = solver.arm_axis
        tilt_z = math.degrees(math.atan2(ax[0], ax[1]))
        tilt_x = math.degrees(math.atan2(ax[2], math.hypot(ax[0], ax[1])))
        print(f"  Semi-trailing angle z   : {tilt_z:.2f}° (around vertical)")
        print(f"  Semi-trailing angle x   : {tilt_x:.2f}° (anti-dive)")
    elif typ == "five_link":
        print(f"  Link lengths            : "
              f"{['%.3f' % L for L in solver.link_lengths]}")

    print()
    ok = (n_failed == 0 and max_wz_err < tol_wz)
    print(f"  Verdict: {'OK' if ok else 'PROBLEMS'}")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="suspension YAML")
    ap.add_argument("--travel", type=float, default=0.08)
    ap.add_argument("--n_travel", type=int, default=17)
    args = ap.parse_args()
    sys.exit(diagnose(Path(args.config), args.travel, args.n_travel))


if __name__ == "__main__":
    main()
