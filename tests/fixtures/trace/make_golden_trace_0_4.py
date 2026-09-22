#!/usr/bin/env python3
"""Generate ``golden_v0_4.vdtrace`` — the fixture of schema ``0.4``.

0.4 adds exactly one manifest field, ``kinematics_attached``, so the fixture
re-declares the frozen 0.3 ride under the 0.4 writer instead of synthesising a
second one: every channel is copied sample-for-sample from
``golden_v0_3.vdtrace``, and only the manifest changes. The 0.3 file stays in
place untouched; it is what exercises the 0.3 read path.

``kinematics_attached`` is ``false``: the ride is synthetic and no hardpoint
set was ever attached to anything, so ``false`` is the only value the file can
state truthfully.

Regenerate with::

    python3 tests/fixtures/trace/make_golden_trace_0_4.py
"""
from __future__ import annotations

import sys
import warnings
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "python"))

import vdsim_trace  # noqa: E402

HERE = Path(__file__).resolve().parent
SRC = HERE / "golden_v0_3.vdtrace"
OUT = HERE / "golden_v0_4.vdtrace"


def build():
    """Copy the 0.3 fixture's channels into a 0.4 container.

    :returns: the written path.
    :raises SystemExit: when the module is not at 0.4 (the fixture would then
        not be the 0.4 one) or the source fixture is missing.
    """
    if vdsim_trace.SCHEMA_VERSION != "0.4":
        raise SystemExit(
            "vdsim_trace is at %s; golden_v0_4.vdtrace is the 0.4 fixture and "
            "is frozen once the schema moves on." % vdsim_trace.SCHEMA_VERSION)
    if not SRC.is_file():
        raise SystemExit("source fixture missing: %s" % SRC)
    with warnings.catch_warnings():
        # The 0.3 source has no kinematics_attached; it is never read here.
        warnings.simplefilter("ignore")
        src = vdsim_trace.TraceReader(SRC)
    with src:
        m = src.manifest
        names = [c["name"] for c in m["channels"]]
        data = {n: src.channel(n) for n in names}
        repro = dict(m["repro"], run_id="golden_v0_4")
        tags = dict(m.get("tags", {}), schema="0.4")
        writer = vdsim_trace.TraceWriter(
            path=OUT,
            geometry=m["geometry"],
            tire=m["tire"],
            repro=repro,
            producer={"name": "make_golden_trace_0_4.py", "version": "0.4"},
            channels=names,
            extra={"tags": tags},
            role=m["role"],
            model_level=m["model_level"],
            contact_scope=m["contact_scope"],
            kinematics_attached=False,
        )
        for i in range(src.n_steps):
            writer.append({n: data[n][i] for n in names})
    return writer.finalize()


if __name__ == "__main__":
    print(build())
