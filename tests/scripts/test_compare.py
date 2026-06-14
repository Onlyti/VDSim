#!/usr/bin/env python3
"""Multi-vehicle comparison tool: run one maneuver across two presets and check
the side-by-side metric table has both vehicles with finite numeric metrics."""
import math
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(REPO / "tools"))

import vdsim_compare as C  # noqa: E402


def main():
    # step_steer only -> fast; two presets with distinct chassis.
    rows = C.run_compare(["sedan", "race_car"], ["step_steer"], tire=None, level="L2")
    assert len(rows) == 2, rows
    assert {r["vehicle"] for r in rows} == {"sedan", "race_car"}
    for r in rows:
        v = r.get("step_steer.r_ss[rad/s]")
        assert isinstance(v, (int, float)) and math.isfinite(v) and abs(v) > 1e-3, r
    # the two vehicles must not be byte-identical (different chassis -> different response)
    r0 = rows[0]["step_steer.r_ss[rad/s]"]
    r1 = rows[1]["step_steer.r_ss[rad/s]"]
    assert r0 != r1, "distinct presets should give distinct step-steer response"

    # CSV round-trip: every row keeps the vehicle column + the metric.
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "compare.csv"
        C.write_csv(rows, out)
        text = out.read_text()
        assert "vehicle" in text and "step_steer.r_ss[rad/s]" in text
        assert "sedan" in text and "race_car" in text

    # The GUI /api/compare returns rows as JSON, so every metric must be a native
    # type — the DLC `completed` flag was a numpy bool_ (not serializable). Lock it.
    import json
    dlc = C.run_compare(["sedan", "race_car"], ["dlc"], tire=None, level="L2")
    json.dumps(dlc)  # raises TypeError on a stray numpy scalar
    assert isinstance(dlc[0]["dlc.completed"], bool)

    print("test_compare: ok")


if __name__ == "__main__":
    main()
