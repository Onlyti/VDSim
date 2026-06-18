#!/usr/bin/env python3
"""Benchmark VDSim dynamics ladder L0–L5: ms/step (ioniq5_awd, dt=5e-4, 500 steps).

L0–L4: SimSession.tick() via pybind. L5 (Free3D stunt): C++ vdsim_bench_levels
(not wired into make_sim_session in pybind).
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python")]

import vdsim
from vdsim_plant import _load_tire_for_vehicle, resolve_vehicle_config

DT = 5e-4
N_STEPS = 500
WARMUP = 50
V0 = 20.0

PY_LEVELS = [
    ("L0", "L0"),
    ("L1", "L1"),
    ("L2", "L2"),
    ("L3", "L3"),
    ("L4", "L4"),
]

BENCH_BIN = REPO / "build" / "bin" / "vdsim_bench_levels"


def _load_ioniq5():
    vp_path = resolve_vehicle_config("ioniq5_awd")
    vp = vdsim.VehicleParams.from_yaml(str(vp_path))
    tp = _load_tire_for_vehicle(vp_path)
    return vp, tp


def bench_py(level_code: str) -> float:
    vp, tp = _load_ioniq5()
    solver = vdsim.SolverParams()
    sess = vdsim.make_sim_session(
        vp, tp, level_code, nominal_dt=DT, solver=solver, mu=tp.mu_nominal
    )
    sess.reset(vdsim.make_init_state(vp, tp, v=V0))
    cmd = vdsim.CmdL4()
    cmd.steer_angle_wheel = 0.02

    for _ in range(WARMUP):
        sess.set_input(cmd)
        sess.tick(DT)

    t0 = time.perf_counter()
    for _ in range(N_STEPS):
        sess.set_input(cmd)
        sess.tick(DT)
    return (time.perf_counter() - t0) / N_STEPS * 1e3


def bench_cpp(level_code: str) -> float:
    if not BENCH_BIN.is_file():
        raise FileNotFoundError(
            f"{BENCH_BIN} missing — run: cmake --build build -j"
        )
    proc = subprocess.run(
        [
            str(BENCH_BIN),
            f"--level={level_code}",
            f"--dt={DT}",
            f"--steps={N_STEPS}",
            f"--warmup={WARMUP}",
            f"--v0={V0}",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return float(proc.stdout.strip())


def main() -> int:
    results: dict[str, float] = {}
    for label, code in PY_LEVELS:
        ms = bench_py(code)
        results[label] = round(ms, 4)
        print(f"{label}: {ms:.4f} ms/step", flush=True)

    ms5 = bench_cpp("L5")
    results["L5"] = round(ms5, 4)
    print(f"L5: {ms5:.4f} ms/step", flush=True)

    print()
    print("| level | ms/step |")
    print("|-------|--------:|")
    for lv in ("L0", "L1", "L2", "L3", "L4", "L5"):
        print(f"| {lv} | {results[lv]:.4f} |")

    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
