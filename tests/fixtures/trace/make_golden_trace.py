#!/usr/bin/env python3
"""Regenerate the golden ``.vdtrace`` fixture.

The fixture is **synthetic**, not simulator output: it is written analytically
so that the container round-trip test and the renderer test run without the
compiled core, which is exactly what DoD 5 ("render from the fixture file
alone, no simulation") asks for. Nothing here should be read as validated
vehicle physics — its only job is to exercise every part of the v0.2 contract:

* all ten channels of §3.2
* a ``tire`` block whose ``mu_aniso`` is elliptic, so the friction-ellipse
  branch of the utilization formula is covered rather than the circular one
* one overlay of each of the three kinds a renderer draws
* the ``role`` field required from schema 0.2 on

``golden_v0_1.vdtrace`` is kept next to the output as a frozen legacy artefact
— it is what pins the reader's 0.1 fallback path, so it is never regenerated.

Run::

    python tests/fixtures/trace/make_golden_trace.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "python"))

import vdsim_trace  # noqa: E402

OUT = Path(__file__).resolve().parent / "golden_v0_2.vdtrace"

DT = 0.02             # 50 Hz record rate
N = 400               # 8 s
WHEELBASE = 2.97
TRACK = 1.635
STEER_RATIO = 14.0
RADIUS = 0.338
MASS = 2100.0
G = 9.81
BASE_MU = 0.9
LOW_MU = 0.45
PATCH_X = (55.0, 105.0)   # low-mu band in world X [m]
MU_ANISO = [1.0, 0.92]    # deliberately elliptic — see module docstring


def _mu_at(x: float) -> float:
    return LOW_MU if PATCH_X[0] <= x <= PATCH_X[1] else BASE_MU


def build():
    """Synthesise the fixture channels and write the container."""
    plant_params = {
        "config": "fixture_vehicle.yaml", "base_mu": BASE_MU,
        "friction_map": [[PATCH_X[0], PATCH_X[1], LOW_MU]],
        "substep_dt": 5e-4,
        "vehicle": {"mass": MASS, "wheelbase": WHEELBASE, "track_front": TRACK},
        "tire": {"backend": "fixture", "mu_aniso": MU_ANISO},
    }
    writer = vdsim_trace.TraceWriter(
        path=OUT,
        geometry={
            "wheelbase_m": WHEELBASE, "track_m": TRACK, "steer_ratio": STEER_RATIO,
            "track_front_m": TRACK, "track_rear_m": TRACK,
            "cg_to_front_m": 1.35, "cg_to_rear_m": WHEELBASE - 1.35,
            "wheel_radius_m": RADIUS,
        },
        tire={"friction_shape": "ellipse", "mu_aniso": MU_ANISO,
              "mu_aniso_source": "fixture"},
        repro={
            "vdsim_version": "fixture", "git_sha": "0" * 40,
            "param_hash": vdsim_trace.param_hash(plant_params),
            "seed": 12345, "dt_s": DT, "run_id": "golden_v0_2",
            "control_dt_s": DT, "substep_dt_s": 5e-4, "decimation": 1,
        },
        producer={"name": "make_golden_trace.py", "version": "0.2"},
        role="plant",
        decimation=1,
        extra={"tags": {"fixture": "golden", "synthetic": True}},
    )

    x, y, yaw = 0.0, 0.0, 0.0
    v = 16.7
    Fz_static = MASS * G / 4.0
    for k in range(N):
        t = k * DT
        # Straight, then a steady steer input that runs into the low-mu band.
        delta = 0.0 if t < 1.0 else 0.055
        fx = 0.0 if t < 1.0 else -2500.0

        mu = _mu_at(x)
        # Kinematic bicycle for the pose; enough to exercise the renderer.
        yaw_rate = v * math.tan(delta) / WHEELBASE
        vy = yaw_rate * 0.7
        ay = v * yaw_rate

        # Per-wheel loads: static + longitudinal and lateral transfer.
        long_shift = MASS * (fx / MASS) * 0.55 / WHEELBASE
        lat_shift = MASS * ay * 0.55 / TRACK
        Fz = [
            Fz_static - 0.5 * long_shift - 0.5 * lat_shift,   # FL
            Fz_static - 0.5 * long_shift + 0.5 * lat_shift,   # FR
            Fz_static + 0.5 * long_shift - 0.5 * lat_shift,   # RL
            Fz_static + 0.5 * long_shift + 0.5 * lat_shift,   # RR
        ]
        Fz = [max(z, 200.0) for z in Fz]
        total_z = sum(Fz)
        wheel_F, wheel_mu, wheel_kappa, wheel_alpha = [], [], [], []
        for w in range(4):
            share = Fz[w] / total_z
            Fx_w = fx * share
            Fy_w = MASS * ay * share * (1.25 if w < 2 else 0.8)
            cap = mu * Fz[w]
            # Clip to the friction ellipse so the fixture never claims a force
            # the tyre could not produce.
            u = math.hypot(Fx_w / (MU_ANISO[0] * cap), Fy_w / (MU_ANISO[1] * cap))
            if u > 1.05:
                Fx_w, Fy_w = Fx_w * 1.05 / u, Fy_w * 1.05 / u
            wheel_F.append((Fx_w, Fy_w, Fz[w]))
            wheel_mu.append(mu)
            wheel_kappa.append(-0.02 - 0.06 * (BASE_MU - mu) / BASE_MU)
            wheel_alpha.append((0.03 if w < 2 else 0.02) * (1.0 if delta else 0.0))

        writer.append({
            "t": t, "pose": (x, y, yaw), "v_body": (v, vy), "yaw_rate": yaw_rate,
            "u_steer": delta, "u_fx": fx, "wheel_F": wheel_F, "wheel_mu": wheel_mu,
            "wheel_kappa": wheel_kappa, "wheel_alpha": wheel_alpha,
        })

        x += (v * math.cos(yaw) - vy * math.sin(yaw)) * DT
        y += (v * math.sin(yaw) + vy * math.cos(yaw)) * DT
        yaw += yaw_rate * DT
        v = max(v + (fx / MASS) * DT, 4.0)

    path = writer.finalize()

    # Overlays are attached after the run, through the pass-through path.
    ref = [(float(i) * 2.0, 0.0) for i in range(90)]
    vdsim_trace.attach_overlay(path, {
        "kind": "path2d", "name": "ref_path", "xy": ref})
    vdsim_trace.attach_overlay(path, {
        "kind": "event", "name": "mu_entry", "t": [3.28], "label": ["low-mu entry"]})
    vdsim_trace.attach_overlay(path, {
        "kind": "region", "name": "mu_patch",
        "polygon": [[PATCH_X[0], -40.0], [PATCH_X[1], -40.0],
                    [PATCH_X[1], 40.0], [PATCH_X[0], 40.0]],
        "mu": LOW_MU})
    print("wrote %s (%d steps, %.1f KiB)"
          % (path, N, path.stat().st_size / 1024.0))
    return path


if __name__ == "__main__":
    build()
