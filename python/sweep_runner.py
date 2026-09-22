#!/usr/bin/env python3
"""Deprecated: superseded by the campaign runner (``vdsim-campaign``).

This script swept a cartesian grid of dotted parameters over a **C++ binary**
(``vdsim_l1_vs_l2``, ``vdsim_scenario_run``) and wrote one CSV per cell.

Two things changed (18_dev_briefing_0903 30.1):

* the run-set layer is now one contract -- one YAML declaration, one
  ``.vdtrace`` per run, one ``index.jsonl`` (21_experiment_runner_spec);
* that contract drives the Python ``Experiment`` API, **not a C++ binary**.
  Binary sweeps are therefore outside the new runner's scope. If you still need
  one, raise it as its own item rather than re-adding it here.

Grid sweeps of the Experiment API move across unchanged::

    # campaign.yaml
    name: drag_vs_speed
    base: step_steer
    sweep:
      grid:
        vehicle.aero_drag_coeff: [0.20, 0.30, 0.40]
        maneuver.v: [5, 10, 15]

    vdsim-campaign run campaign.yaml --jobs 4

The file is kept rather than deleted because callers outside this repository
cannot be found by grep; it will be removed after the P0 release.
"""
import sys
import warnings

MESSAGE = (
    "python/sweep_runner.py is deprecated and does nothing. For sweeps of the "
    "Experiment API use:\n"
    "    vdsim-campaign run <campaign.yaml> [--jobs N]\n"
    "Sweeping a C++ binary is out of that runner's scope -- raise it as its "
    "own item.\n"
    "Declaration schema and index keys: docs/design/BATCH_RUNNER.md"
)


def main(argv=None):
    """Warn, print the replacement command and fail.

    :returns: exit code 2.
    """
    warnings.warn(MESSAGE, DeprecationWarning, stacklevel=2)
    sys.stderr.write(MESSAGE + "\n")
    return 2


if __name__ == "__main__":
    sys.exit(main())
