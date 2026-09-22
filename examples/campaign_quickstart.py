#!/usr/bin/env python3
"""Quickstart: many runs from one declaration, then read the index.

The single-run seam is in examples/experiment_quickstart.py. This is the layer
above it: declare a set of runs, let the campaign runner record one .vdtrace
each, then look them up through the index.

    PYTHONPATH=build/python:python python3 examples/campaign_quickstart.py

Pip installs: `vdsim-campaign run <campaign.yaml>` does the same thing from the
command line.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python")]

import vdsim_campaign as vc


def main():
    spec = {
        "name": "quickstart_mu",
        "base": "step_steer",
        "seed": 20260922,
        "duration": 2.0,
        "sweep": {"grid": {"mu": [0.9, 0.6]}},
    }
    out = REPO / "results"
    index = vc.CampaignRunner(spec, out=out, jobs=1).run()

    print("index:", index)
    for row in vc.read_index(index):
        # The index answers "which run was which and how did it end" — metrics
        # are a consumer's job, not the index's (EX3).
        print("  %s  %-8s  axes=%s  trace=%s"
              % (row["run_id"], row["status"], row["axes"], row["trace_path"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
