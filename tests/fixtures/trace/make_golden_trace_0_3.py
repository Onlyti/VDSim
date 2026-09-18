#!/usr/bin/env python3
"""Generate ``golden_v0_3.vdtrace`` — the frozen L3 fixture of schema ``0.3``.

Why synthetic rather than a recorded plant run: the fixture has to be readable
and renderable on a machine that cannot build the core (that is the point of the
container contract, ``11`` §1), and it has to be bit-stable across compilers so
a byte-comparison regression gate means something.  The numbers below are a
kinematically consistent ride over a crowned, banked road; they are *not* a
physics result and nothing in the tree treats them as one.

The run deliberately exercises every ``0.3`` channel with non-trivial values:

* a bank that ramps in, so ``wheel_road_normal`` leaves ``(0,0,1)``
* a crown, so the left and right ``wheel_road_dz`` differ
* heave/roll/pitch in ``pose_zrp`` and matching ``wheel_travel``
* an ``a_body`` with a real ``az`` (the ride bounce), never a zero column

Regenerate with::

    python3 tests/fixtures/trace/make_golden_trace_0_3.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "python"))

import vdsim_trace  # noqa: E402

OUT = Path(__file__).resolve().parent / "golden_v0_3.vdtrace"

N = 240
DT = 0.02
V = 18.0
WHEELBASE = 2.70
TRACK = 1.60
STEER_RATIO = 15.0
RADIUS = 0.32
MASS = 1800.0
CG_HEIGHT = 0.55
MU = 0.95
MU_ANISO = [1.0, 0.92]
#: Peak bank of the road section [rad]; ramps in over the first third.
BANK_MAX = math.radians(8.0)
#: Road crown half-amplitude [m] between the left and right wheel tracks.
CROWN = 0.03


def bank_at(t: float) -> float:
    """Road bank [rad] at time ``t`` — zero, then a smooth ramp, then constant."""
    frac = min(1.0, max(0.0, (t - 0.6) / 1.8))
    return BANK_MAX * 0.5 * (1.0 - math.cos(math.pi * frac))


def build():
    plant_params = {
        "config": "fixture_vehicle_l3.yaml", "base_mu": MU, "substep_dt": 5e-4,
        "level": "L3",
        "vehicle": {"mass": MASS, "wheelbase": WHEELBASE, "track_front": TRACK},
        "tire": {"backend": "fixture", "mu_aniso": MU_ANISO},
    }
    writer = vdsim_trace.TraceWriter(
        path=OUT,
        geometry={
            "wheelbase_m": WHEELBASE, "track_m": TRACK, "steer_ratio": STEER_RATIO,
            "track_front_m": TRACK, "track_rear_m": TRACK,
            "cg_to_front_m": 1.25, "cg_to_rear_m": WHEELBASE - 1.25,
            "wheel_radius_m": RADIUS,
            "mass_kg": MASS, "cg_height_m": CG_HEIGHT,
            "wheel_width_m": 0.225, "body_lwh_m": [4.55, 1.85, 1.45],
        },
        tire={"friction_shape": "ellipse", "mu_aniso": MU_ANISO,
              "mu_aniso_source": "fixture"},
        repro={
            "vdsim_version": "fixture", "git_sha": "0" * 40,
            "param_hash": vdsim_trace.param_hash(plant_params),
            "seed": 20260918, "dt_s": DT, "run_id": "golden_v0_3",
            "control_dt_s": DT, "substep_dt_s": 5e-4, "decimation": 1,
        },
        producer={"name": "make_golden_trace_0_3.py", "version": "0.3"},
        role="plant",
        model_level="L3",
        contact_scope="C2",
        decimation=1,
        extra={"tags": {"fixture": "golden", "synthetic": True, "schema": "0.3"}},
    )

    x, y, yaw = 0.0, 0.0, 0.0
    # Longitudinal offsets of each wheel from the CG, FL FR RL RR.
    rx = [1.25, 1.25, -(WHEELBASE - 1.25), -(WHEELBASE - 1.25)]
    ry = [0.5 * TRACK, -0.5 * TRACK, 0.5 * TRACK, -0.5 * TRACK]

    for i in range(N):
        t = i * DT
        steer = 0.06 * math.sin(2.0 * math.pi * 0.25 * t)
        yaw_rate = V * math.tan(steer) / WHEELBASE
        bank = bank_at(t)
        # Body attitude: roll follows the bank plus a lateral-load term, pitch
        # follows the longitudinal force, heave bounces at the ride frequency.
        ay = V * yaw_rate
        ax = 0.8 * math.sin(2.0 * math.pi * 0.15 * t)
        roll = bank + 0.035 * ay / 9.80665
        pitch = -0.020 * ax / 9.80665
        heave = 0.012 * math.sin(2.0 * math.pi * 1.2 * t)
        az = -0.012 * (2.0 * math.pi * 1.2) ** 2 * math.sin(2.0 * math.pi * 1.2 * t)

        normal = []
        road_dz = []
        travel = []
        for w in range(4):
            # Road surface: banked plane plus a crown that lifts the outer track.
            dz = -ry[w] * math.tan(bank) + CROWN * (1.0 - abs(ry[w]) / (0.5 * TRACK)) ** 2
            road_dz.append(dz)
            n = (0.0, -math.sin(bank), math.cos(bank))
            normal.append(n)
            corner = heave + ry[w] * (roll - bank) - rx[w] * pitch
            travel.append(0.06 - corner)

        fz_front = MASS * 9.80665 * 0.52 * 0.5
        fz_rear = MASS * 9.80665 * 0.48 * 0.5
        lat = 0.5 * MASS * ay
        lon = 0.25 * MASS * ax
        wheel_F = [
            (lon, lat * 0.55, fz_front * (1.0 - 0.25 * ay / 9.80665)),
            (lon, lat * 0.55, fz_front * (1.0 + 0.25 * ay / 9.80665)),
            (lon, lat * 0.45, fz_rear * (1.0 - 0.25 * ay / 9.80665)),
            (lon, lat * 0.45, fz_rear * (1.0 + 0.25 * ay / 9.80665)),
        ]

        writer.append({
            "t": t,
            "pose": (x, y, yaw),
            "pose_zrp": (CG_HEIGHT + heave, roll, pitch),
            "v_body": (V, V * math.sin(0.3 * steer)),
            "a_body": (ax, ay, az),
            "yaw_rate": yaw_rate,
            "u_steer": steer,
            "u_fx": MASS * ax,
            "wheel_F": wheel_F,
            "wheel_mu": [MU] * 4,
            "wheel_kappa": [0.01 * math.sin(0.5 * t)] * 4,
            "wheel_alpha": [0.02 * steer] * 4,
            "wheel_road_dz": road_dz,
            "wheel_road_normal": normal,
            "wheel_travel": travel,
        })

        x += V * math.cos(yaw) * DT
        y += V * math.sin(yaw) * DT
        yaw += yaw_rate * DT

    path = writer.finalize()

    # A reference path overlay, so the 3D player's ref_path layer has input and
    # the fixture also covers the overlay pass-through contract at 0.3.
    vdsim_trace.attach_overlay(path, {
        "kind": "path2d", "name": "waypoint",
        "x": [k * 2.0 for k in range(40)],
        "y": [0.0] * 40,
    })
    print("wrote %s (%d steps)" % (path, N))


if __name__ == "__main__":
    build()
