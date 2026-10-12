#!/usr/bin/env python3
"""Generate ``golden_v0_5.vdtrace`` — the fixture of schema ``0.5``.

0.5 records ``rp_rate`` and ``wheel_travel`` at every level and states, per
channel, whether the model computed it (``channel_validity``). The fixture is an
L2 run because that is where the marker matters: both channels are exactly
``0.0`` and the manifest says ``not_modeled@L2``. A reader that ignores the
marker sees a plausible flat ride; a reader that honours it sees "not modeled".

The samples are synthetic (a constant-radius turn, no simulator involved) so the
file is byte-stable. The 0.3 and 0.4 fixtures stay untouched and keep exercising
their own read paths.

Regenerate with::

    python3 tests/fixtures/trace/make_golden_trace_0_5.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "python"))

import vdsim_trace  # noqa: E402

HERE = Path(__file__).resolve().parent
OUT = HERE / "golden_v0_5.vdtrace"

N = 60
DT = 0.05
SPEED = 12.0
YAW_RATE = 0.2


def build():
    """Write the 0.5 fixture.

    :returns: the written path.
    :raises SystemExit: when the module is not at 0.5.
    """
    if vdsim_trace.SCHEMA_VERSION != "0.5":
        raise SystemExit(
            "vdsim_trace is at %s; golden_v0_5.vdtrace is the 0.5 fixture and "
            "is frozen once the schema moves on." % vdsim_trace.SCHEMA_VERSION)
    names = vdsim_trace.channels_for_level("L2")
    marker = vdsim_trace.not_modeled_marker("L2")
    writer = vdsim_trace.TraceWriter(
        path=OUT,
        geometry={"wheelbase_m": 2.7, "track_m": 1.6, "steer_ratio": 15.0,
                  "mass_kg": 1800.0, "cg_height_m": 0.55, "wheel_radius_m": 0.32,
                  "wheel_width_m": 0.225, "body_lwh_m": [4.6, 1.9, 1.5]},
        tire={"friction_shape": "circle", "mu_aniso": [1.0, 1.0]},
        repro={"vdsim_version": "golden", "git_sha": "golden",
               "param_hash": "sha256:golden", "seed": 0, "dt_s": DT,
               "run_id": "golden_v0_5"},
        producer={"name": "make_golden_trace_0_5.py", "version": "0.5"},
        channels=names,
        extra={"tags": {"schema": "0.5"}},
        role="plant",
        model_level="L2",
        contact_scope="C2",
        kinematics_attached=False,
        channel_validity={"rp_rate": marker, "wheel_travel": marker},
    )
    for i in range(N):
        t = i * DT
        yaw = YAW_RATE * t
        radius = SPEED / YAW_RATE
        writer.append({
            "t": t,
            "pose": (radius * math.sin(yaw), radius * (1.0 - math.cos(yaw)), yaw),
            "v_body": (SPEED, 0.0),
            "yaw_rate": YAW_RATE,
            "u_steer": 0.04,
            "u_fx": 0.0,
            "wheel_F": [(150.0, 900.0, 4400.0)] * 4,
            "wheel_mu": [0.9] * 4,
            "wheel_kappa": [0.01] * 4,
            "wheel_alpha": [0.03] * 4,
            "a_body": (0.0, SPEED * YAW_RATE, 0.0),
            "rp_rate": (0.0, 0.0),
            "wheel_travel": [0.0, 0.0, 0.0, 0.0],
        })
    return writer.finalize()


if __name__ == "__main__":
    print(build())
