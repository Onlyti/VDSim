#!/usr/bin/env python3
"""Deprecated: superseded by the campaign runner (``vdsim-campaign``).

This script ran a hand-written list of scenario names and reduced each to a
metrics CSV. The run-set layer it belonged to is now one contract
(21_experiment_runner_spec, 18_dev_briefing_0903 30.1): a campaign is declared
in one YAML, each run records a ``.vdtrace``, and the run list lives in
``index.jsonl`` rather than in a summary table.

Replacement -- write the scenario list as a campaign declaration::

    # campaign.yaml
    name: nightly
    base: step_steer
    sweep:
      list:
        - {}                       # the scenario as authored
        - {maneuver.v: 15.0}

    vdsim-campaign run campaign.yaml --jobs 4

Per-run metrics are not part of that contract: the index is a lookup table, not
a result database (EX3). Compute metrics in a consumer script that reads the
index with ``vdsim_campaign.read_index``.

The file is kept rather than deleted because callers outside this repository
cannot be found by grep; it will be removed after the P0 release.
"""
import sys
import warnings

MESSAGE = (
    "tools/campaign_runner.py is deprecated and does nothing. Use the campaign "
    "runner instead:\n"
    "    vdsim-campaign run <campaign.yaml> [--jobs N] [--render overview]\n"
    "Declaration schema and index keys: docs/design/BATCH_RUNNER.md"
)


def main(argv=None):
    """Warn, print the replacement command and fail.

    Failing rather than silently forwarding is deliberate: the old CLI took a
    list of scenario names and produced a metrics table, and the new one takes
    a declaration file and produces traces. A silent forward would hand the
    caller a different artefact under the same command.

    :returns: exit code 2.
    """
    warnings.warn(MESSAGE, DeprecationWarning, stacklevel=2)
    sys.stderr.write(MESSAGE + "\n")
    return 2


if __name__ == "__main__":
    sys.exit(main())
