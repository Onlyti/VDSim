#!/usr/bin/env python3
"""Independent reference for gate P9 (work order 49a): contact-patch lateral slope.

dy_cp/dz_v at zero travel and zero rack for the left front double-wishbone corner,
from tools/kinematics/dw_3d_solver.py (pure numpy/yaml, no vdsim import).

  c(q), s_hat(q) = DW3DSolver.solve(q)["wheel_pos"], ["spin_axis"]   (left corner frame)
  d_hat(q)       = normalize(e_z - (e_z . s_hat) s_hat)               (wheel plane, down)
  cp(q)          = c(q) - R d_hat(q)                                  (lowest rim point)
  dy_cp/dz_v     = (cp_y(+h) - cp_y(-h)) / (2 h),  h = 1e-3 m
  dy_wc/dz_v     = (c_y(+h) - c_y(-h)) / (2 h)                        (information)

R is the vehicle wheel_radius_nominal (the radius the L5 tyre contact uses), not the
part YAML static_radius. The right corner is the y mirror, so its slope is the negative.

Usage: python3 tests/scripts/gen_dae_contact_slope_ref.py [--radius 0.32]
Prints one line per fixture; the values are copied into tests/unit/test_dae_contact_slope.cpp.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "kinematics"))
from dw_3d_solver import DW3DSolver  # noqa: E402

FIXTURES = {
    "antidive": ROOT / "tests/fixtures/dw_front_antidive_sample.yaml",
    "sports": ROOT / "configs/parts/susp_kinematics/kin/dw_front_sports.yaml",
}
H = 1e-3


def contact_point(sol: dict, radius: float) -> np.ndarray:
    """Lowest rim point of the wheel circle: centre - R * (in-plane unit vector toward -z).

    @param sol    DW3DSolver.solve() result (valid)
    @param radius wheel radius R [m]
    @return contact point [m], corner (left) frame
    """
    c = np.asarray(sol["wheel_pos"], dtype=float)
    s = np.asarray(sol["spin_axis"], dtype=float)
    s = s / np.linalg.norm(s)
    ez = np.array([0.0, 0.0, 1.0])
    d = ez - ez.dot(s) * s
    return c - radius * d / np.linalg.norm(d)


def slopes(path: Path, radius: float) -> tuple[float, float]:
    """dy_cp/dz_v and dy_wc/dz_v at zero travel and zero rack.

    @param path   double-wishbone hardpoint YAML
    @param radius wheel radius R [m]
    @return (dy_cp/dz_v, dy_wc/dz_v) [-]
    """
    solver = DW3DSolver(yaml.safe_load(path.read_text()))
    sp, sm = solver.solve(+H, 0.0), solver.solve(-H, 0.0)
    if not (sp["valid"] and sm["valid"]):
        raise SystemExit(f"solver failed for {path}")
    cp = (contact_point(sp, radius)[1] - contact_point(sm, radius)[1]) / (2 * H)
    wc = (sp["wheel_pos"][1] - sm["wheel_pos"][1]) / (2 * H)
    return cp, wc


def main() -> None:
    """Print the P9 reference table."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=0.32)
    a = ap.parse_args()
    for name, path in FIXTURES.items():
        cp, wc = slopes(path, a.radius)
        print(f"{name} R={a.radius:.4f} dy_cp/dz={cp:.12e} dy_wc/dz={wc:.12e}")


if __name__ == "__main__":
    main()
