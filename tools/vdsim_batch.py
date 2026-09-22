#!/usr/bin/env python3
"""Entry point for the campaign runner, kept at its historical path.

The batch/campaign logic this file used to hold was promoted into
``python/vdsim_campaign.py`` so that it ships in the wheel and can be imported
(``vdsim-campaign``, ``vdsim_campaign.read_index``). Everything that was here --
explicit scenarios, grid sweeps, Monte Carlo, process parallelism -- moved with
it; this file stays so existing invocations keep working.

    python3 tools/vdsim_batch.py run campaign.yaml
    python3 tools/vdsim_batch.py run campaign.yaml --dry

is the same as::

    vdsim-campaign run campaign.yaml

See docs/design/BATCH_RUNNER.md for the declaration schema, the run directory
layout and the index keys.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "python"))

import vdsim_campaign  # noqa: E402


def _translate(argv):
    """Map this script's historical flags onto the ``vdsim-campaign`` CLI.

    ``--parallel N`` was this tool's name for what the contract calls
    ``--jobs N`` (EX8). It is accepted here and nowhere else.

    :param argv: argument list without the program name.
    :returns: argument list for :func:`vdsim_campaign.main`.
    """
    out = []
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == "--parallel":
            out += ["--jobs", argv[i + 1]]
            i += 2
            continue
        if arg.startswith("--parallel="):
            out += ["--jobs", arg.split("=", 1)[1]]
            i += 1
            continue
        out.append(arg)
        i += 1
    return out


def main(argv=None):
    """Forward to :func:`vdsim_campaign.main`."""
    return vdsim_campaign.main(_translate(list(sys.argv[1:] if argv is None else argv)))


if __name__ == "__main__":
    sys.exit(main())
