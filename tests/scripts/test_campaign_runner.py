"""Campaign runner contract tests (21_experiment_runner_spec EG1-EG6).

The runner is the layer *above* a run: it decides how many runs there are, what
each one is seeded with, where it lands and what the index says about it. These
checks pin the six acceptance gates and the two rules that are easy to erode --
that the trace contract stays untouched (EX1) and that the index stays a lookup
table rather than a result database (EX3).

Each simulated run is short on purpose; what is under test is the layer, not
the physics.
"""
from __future__ import annotations

import json
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))
_BUILD = Path(__file__).resolve().parents[2] / "build" / "python"
if _BUILD.is_dir():
    sys.path.insert(0, str(_BUILD))

import vdsim_campaign as vc   # noqa: E402
import vdsim_trace as vt      # noqa: E402

#: Short enough that a dozen runs stay inside the ctest budget, long enough
#: that the step-steer input has actually been applied.
DURATION = 1.0

FAILURES = []


def check(cond, msg):
    if not cond:
        FAILURES.append(msg)
        print("FAIL: %s" % msg)
    else:
        print("ok  : %s" % msg)


def _write(tmp, name, spec):
    """Write a campaign declaration and return its path."""
    path = Path(tmp) / (name + ".yaml")
    path.write_text(yaml.safe_dump(spec, sort_keys=True))
    return path


def _run(spec_path, out, *extra):
    """Invoke the CLI the way a user does."""
    return vc.main(["run", str(spec_path), "--out", str(out)] + list(extra))


def _channels_digest(trace_path):
    """Bytes of every channel of a trace, keyed by name."""
    out = {}
    with vt.TraceReader(trace_path) as tr:
        for name in tr.channel_names():
            out[name] = np.asarray(tr.channel(name)).tobytes()
    return out


# --------------------------------------------------------------------------- #
# EG1 -- axis expansion
# --------------------------------------------------------------------------- #
def test_axis_expansion():
    grid = {"name": "g", "base": "step_steer",
            "sweep": {"grid": {"mu": [0.9, 0.7, 0.5], "maneuver.v": [10.0, 20.0]}}}
    runs = vc.expand(grid)
    check(len(runs) == 6, "grid 3x2 expands to 6 runs (got %d)" % len(runs))
    seen = [tuple(sorted(r["axes"].items())) for r in runs]
    check(len(set(seen)) == 6, "every grid cell is distinct")

    listed = {"name": "l", "base": "step_steer",
              "sweep": {"list": [{"mu": 0.9}, {"mu": 0.5, "maneuver.v": 12.0}]}}
    runs = vc.expand(listed)
    check(len(runs) == 2, "list expands to exactly the named combinations")

    rep = {"name": "r", "base": "step_steer", "seed": 7,
           "sweep": {"list": [{"mu": 0.9}], "repeat": 3}}
    runner = vc.CampaignRunner(rep)
    jobs = runner.plan()
    check(len(jobs) == 3, "repeat 3 expands to 3 runs")
    without_repeat = set(
        tuple(sorted((k, v) for k, v in j["axes"].items() if k != "repeat"))
        for j in jobs)
    check(len(without_repeat) == 1, "repeat keeps every parameter but the seed fixed")
    check(len({j["seed"] for j in jobs}) == 3, "repeat gives each run its own seed")
    check([j["run_id"] for j in jobs] == ["000", "001", "002"],
          "run_id is a zero-padded ordinal, sortable and readable")


def test_seed_is_a_function_of_the_declaration():
    a = [vc.derive_seed(20260922, i) for i in range(4)]
    b = [vc.derive_seed(20260922, i) for i in range(4)]
    check(a == b, "the same declaration derives the same seeds (EX4)")
    check(a != [vc.derive_seed(1, i) for i in range(4)],
          "a different root seed derives different seeds")
    check(all(isinstance(s, int) and s >= 0 for s in a), "seeds are non-negative ints")


# --------------------------------------------------------------------------- #
# EG2 / EG4 -- contract compliance and failure isolation in one campaign
# --------------------------------------------------------------------------- #
def test_contract_and_failure_isolation(tmp):
    spec = {"name": "isolate", "base": "step_steer", "seed": 20260922,
            "duration": DURATION,
            "sweep": {"list": [{"mu": 0.9},
                               {"vehicle.mass": 1.0e-9},
                               {"mu": 0.5}]}}
    out = Path(tmp) / "eg24"
    _run(_write(tmp, "isolate", spec), out)
    rows = vc.read_index(out / "isolate")
    check(len(rows) == 3, "every run has an index line, failures included (EG4)")

    by_id = {r["run_id"]: r for r in rows}
    check(by_id["001"]["status"] == "diverged",
          "the deliberately unstable run is recorded as diverged, got %r"
          % by_id["001"]["status"])
    check(by_id["000"]["status"] == "ok" and by_id["002"]["status"] == "ok",
          "one bad run does not stop the campaign (EG4)")

    missing = [k for k in vc.INDEX_KEYS if k not in by_id["000"]]
    check(not missing, "index line carries the required keys; missing %s" % missing)
    check(all(r["status"] in vc.STATUSES for r in rows),
          "every status is one of the declared dispositions")

    metric_like = [k for k in by_id["000"]
                   if k in ("metrics", "peak_ay", "lap_time", "cte_rms")]
    check(not metric_like,
          "the index stays a lookup table, not a result database (EX3): %s"
          % metric_like)

    trace = out / "isolate" / by_id["000"]["trace_path"]
    check(trace.is_file(), "a finished run leaves exactly one .vdtrace")
    with vt.TraceReader(trace) as tr:
        manifest = tr.manifest
        check(tr.role == "plant", "manifest role is plant")
        check(manifest["schema_version"] == vt.SCHEMA_VERSION,
              "the runner writes the current trace schema, not one of its own")
        check(manifest.get("producer", {}).get("name") == "vdsim_campaign",
              "the runner names itself in producer, the one allowed mention (EX1)")
        leaked = [k for k in manifest
                  if "campaign" in k.lower() or "axes" in k.lower()
                  or "experiment" in k.lower()]
        check(not leaked,
              "no campaign field leaks into the manifest (EG2): %s" % leaked)
        check(not manifest.get("tags"),
              "axis values live in the index, not in manifest tags (EX1)")
        check(tr.repro["param_hash"] == by_id["000"]["param_hash"],
              "index param_hash is the trace's own, not a parallel computation")
    return out / "isolate"


# --------------------------------------------------------------------------- #
# EG3 -- determinism, including under --jobs
# --------------------------------------------------------------------------- #
def test_determinism(tmp):
    spec = {"name": "determ", "base": "step_steer", "seed": 20260922,
            "duration": DURATION,
            "sweep": {"grid": {"mu": [0.9, 0.6]}}}
    path = _write(tmp, "determ", spec)
    first = Path(tmp) / "eg3a"
    second = Path(tmp) / "eg3b"
    _run(path, first)
    _run(path, second, "--jobs", "4")

    same = True
    for run_id in ("000", "001"):
        a = _channels_digest(first / "determ" / run_id / "run.vdtrace")
        b = _channels_digest(second / "determ" / run_id / "run.vdtrace")
        same = same and a.keys() == b.keys() and all(a[k] == b[k] for k in a)
    check(same, "same declaration, --jobs 1 and --jobs 4: identical channel bytes (EG3)")

    rows_a = vc.read_index(first / "determ")
    rows_b = vc.read_index(second / "determ")
    check([r["run_id"] for r in rows_a] == [r["run_id"] for r in rows_b],
          "index order is the declaration order, not the completion order")
    check([r["param_hash"] for r in rows_a] == [r["param_hash"] for r in rows_b],
          "param_hash does not depend on how the runs were scheduled")
    return path, first


# --------------------------------------------------------------------------- #
# EG5 -- resume
# --------------------------------------------------------------------------- #
def test_resume(tmp, spec_path, out):
    campaign = out / "determ"
    kept = campaign / "000" / "run.vdtrace"
    dropped = campaign / "001" / "run.vdtrace"
    kept_before = kept.stat().st_mtime_ns
    dropped.unlink()

    _run(spec_path, out, "--resume")
    rows = {r["run_id"]: r for r in vc.read_index(campaign)}
    check(rows["000"]["status"] == "skipped",
          "a complete, matching run is skipped on resume (EG5)")
    check(kept.stat().st_mtime_ns == kept_before,
          "the skipped run's trace is not rewritten")
    check(rows["001"]["status"] == "ok" and dropped.is_file(),
          "the run whose trace went missing is the one that re-executes (EG5)")

    # Same campaign name, different parameters: the runner must not let the two
    # parameter sets share one campaign directory.
    moved = yaml.safe_load(spec_path.read_text())
    moved["sweep"]["grid"]["mu"] = [0.9, 0.4]
    refused = False
    try:
        _run(_write(tmp, "determ_changed", moved), out, "--resume")
    except vc.CampaignError:
        refused = True
    check(refused,
          "resume is refused once the declaration moved: two parameter sets "
          "must not share one campaign directory (EG5)")


# --------------------------------------------------------------------------- #
# EG6 -- render connection
# --------------------------------------------------------------------------- #
def test_render_connection(tmp):
    spec = {"name": "rendered", "base": "step_steer", "seed": 3,
            "duration": 0.6, "sweep": {"list": [{"mu": 0.9}]}}
    out = Path(tmp) / "eg6"
    _run(_write(tmp, "rendered", spec), out, "--render", "overview")
    rows = vc.read_index(out / "rendered")
    check(rows[0]["status"] == "ok", "the rendered run itself succeeded")
    check(rows[0].get("render_status") == "ok",
          "render success is recorded on its own key, got %r"
          % rows[0].get("render_status"))
    check((out / "rendered" / "000" / "preview.png").is_file(),
          "the render CLI produced its output next to the run")

    bad = Path(tmp) / "eg6bad"
    _run(_write(tmp, "rendered_bad", dict(spec, name="rendered_bad")), bad,
         "--render", "no_such_preset")
    rows = vc.read_index(bad / "rendered_bad")
    check(rows[0]["status"] == "ok",
          "a render failure is not a run failure (EX7), got %r" % rows[0]["status"])
    check(str(rows[0].get("render_status", "")).startswith("error"),
          "the render failure is still reported, got %r"
          % rows[0].get("render_status"))


# --------------------------------------------------------------------------- #
# Promotion: the old entry points must not quietly do something else
# --------------------------------------------------------------------------- #
def test_superseded_entry_points():
    sys.path.insert(0, str(REPO / "tools"))
    import vdsim_batch                                     # noqa: E402
    check(vdsim_batch._translate(["run", "c.yaml", "--parallel", "4"])
          == ["run", "c.yaml", "--jobs", "4"],
          "tools/vdsim_batch.py forwards --parallel onto the contract's --jobs")

    import campaign_runner                                 # noqa: E402
    import sweep_runner                                    # noqa: E402
    check(campaign_runner.main([]) == 2 and sweep_runner.main([]) == 2,
          "the superseded runners fail loudly instead of forwarding silently")
    check("vdsim-campaign" in campaign_runner.MESSAGE
          and "vdsim-campaign" in sweep_runner.MESSAGE,
          "each deprecation names the replacement command")
    check("out of that runner's scope" in sweep_runner.MESSAGE,
          "the binary sweep is stated to be out of scope, not silently dropped")


# --------------------------------------------------------------------------- #
# C-1 (v): a level axis is accepted; a bare L4 run is refused by the seam
# --------------------------------------------------------------------------- #
def test_level_axis_after_q20(tmp):
    """``level`` sweeps, and cannot archive a false L4 label.

    The axis was refused while a bare L4 run recorded ``model_level: L4`` with
    L3 physics. Q20 moved the refusal to the session seam, so the axis is
    expanded on every form, the L3 run records ``kinematics_attached: false``
    and the bare L4 run is an ``error`` row with no trace at all.
    """
    forms = {
        "grid": {"base": "step_steer", "sweep": {"grid": {"level": ["L3", "L4"]}}},
        "list": {"base": "step_steer", "sweep": {"list": [{"level": "L4"}]}},
        "legacy": {"runs": [{"sweep": {"base": "step_steer",
                                       "grid": {"level": ["L3", "L4"]}}}]},
    }
    for name, spec in forms.items():
        try:
            runs = vc.expand(spec)
            check(len(runs) >= 1, "%s form expands a level axis" % name)
        except vc.CampaignError as exc:
            check(False, "%s form still refuses a level axis (%s)" % (name, exc))

    spec = {"name": "levels", "base": "step_steer", "seed": 20260922,
            "duration": DURATION, "sweep": {"grid": {"level": ["L3", "L4"]}}}
    out = Path(tmp) / "levels"
    _run(_write(tmp, "levels", spec), out)
    rows = {r["axes"]["level"]: r for r in vc.read_index(out / "levels")}
    check(rows["L3"]["status"] == "ok", "the L3 run completes (%s)" % rows["L3"]["status"])
    l3_trace = out / "levels" / rows["L3"]["trace_path"]
    with vt.TraceReader(l3_trace) as tr:
        check(tr.model_level == "L3" and tr.kinematics_attached is False,
              "the L3 trace states no hardpoints were attached")
    err = rows["L4"].get("error") or ""
    print("      bare L4 row: status=%s error=%s" % (rows["L4"]["status"], err))
    check(rows["L4"]["status"] == "error" and "level='L4'" in err,
          "the bare L4 run is refused by the seam, not run as L3")
    check(rows["L4"]["trace_path"] is None,
          "no trace is left behind for the refused L4 run")


def test_l4_with_kinematics_from_config(tmp):
    """Q20 (B): a declared ``kinematics`` block reaches the attach.

    ``Experiment.from_config`` reads ``kinematics: {front, rear}``, so an L4
    cell that names hardpoints runs as L4 physics. The L3 cell carries none;
    the two must both complete and must not be bit-identical, otherwise the
    block was dropped and L4 fell back to the bare-L3 trajectory.
    """
    spec = {"name": "kin", "base": "step_steer", "seed": 20260922,
            "duration": DURATION,
            "sweep": {"list": [
                {"level": "L3"},
                {"level": "L4", "kinematics.front": "mp_front_sedan",
                 "kinematics.rear": "ta_rear_sedan"}]}}
    out = Path(tmp) / "kin"
    _run(_write(tmp, "kin", spec), out)
    rows = {r["axes"]["level"]: r for r in vc.read_index(out / "kin")}
    for lv in ("L3", "L4"):
        check(rows[lv]["status"] == "ok",
              "the %s cell completes (%s %s)"
              % (lv, rows[lv]["status"], rows[lv].get("error") or ""))
    if rows["L3"]["status"] != "ok" or rows["L4"]["status"] != "ok":
        return
    paths = {lv: out / "kin" / rows[lv]["trace_path"] for lv in ("L3", "L4")}
    with vt.TraceReader(paths["L4"]) as tr:
        check(tr.model_level == "L4" and tr.kinematics_attached is True,
              "the L4 trace states the declared hardpoints were attached")
    a, b = (_channels_digest(paths[lv]) for lv in ("L3", "L4"))
    differ = sorted(k for k in a if k in b and a[k] != b[k])
    print("      channels that differ L3 vs L4+kinematics: %s" % differ)
    check(bool(differ), "L4 with hardpoints is not bit-identical to bare L3")


# --------------------------------------------------------------------------- #
# C-2: the friction-ellipse measurement is on the trace path, so it must run
# --------------------------------------------------------------------------- #
def test_mu_aniso_measurement_runs():
    """Execute ``measure_mu_aniso`` rather than only reading its source.

    This path had a latent call-signature fault that survived because nothing
    executed it: ``vdsim.create_pacejka_mf96`` takes no argument. A test that
    merely imports the module would not have caught it, so this one calls the
    function and uses its result.
    """
    import vdsim_lab as vl
    import vdsim_plant as vplant

    shape, aniso = vplant.measure_mu_aniso(vl.Tire.preset().tp)
    check(shape in ("circle", "ellipse"),
          "measure_mu_aniso returns a friction shape the manifest accepts (%s)"
          % shape)
    check(len(aniso) == 2 and all(np.isfinite(v) and v > 0.0 for v in aniso),
          "both mu multipliers are finite and positive (%r)" % (aniso,))
    check(abs(aniso[0] - aniso[1]) <= 0.01 * max(aniso)
          if shape == "circle" else True,
          "a 'circle' verdict means the two multipliers agree to 1 %")


def test_resolved_preset_is_private():
    """Two resolutions of one preset must not share files (Q10-c).

    Every run process resolves its vehicle and tire preset to YAML and reads it
    straight back. When those files lived at one path per preset, a second
    process rewriting them could hand the first a truncated file, parsed as
    C++ defaults -- EG3 then failed only when the timing lined up. A parallel
    stress check would pass most of the time on a broken tree, so this pins the
    structure instead: each call gets its own non-empty files.
    """
    import vdsim_lab as vl
    first = [Path(p) for p in vl._resolve_preset()]
    second = [Path(p) for p in vl._resolve_preset()]
    check(not {str(p) for p in first} & {str(p) for p in second},
          "each preset resolution writes its own files, none shared (Q10-c)")
    check(all(p.is_file() and p.stat().st_size > 0 for p in first + second),
          "every resolved preset file exists and is non-empty")


def main():
    with tempfile.TemporaryDirectory(prefix="vdsim_campaign_") as tmp:
        test_axis_expansion()
        test_mu_aniso_measurement_runs()
        test_resolved_preset_is_private()
        test_seed_is_a_function_of_the_declaration()
        test_contract_and_failure_isolation(tmp)
        test_level_axis_after_q20(tmp)
        test_l4_with_kinematics_from_config(tmp)
        spec_path, out = test_determinism(tmp)
        test_resume(tmp, spec_path, out)
        test_render_connection(tmp)
        test_superseded_entry_points()
    if FAILURES:
        print("\n%d check(s) failed" % len(FAILURES))
        for f in FAILURES:
            print("  - %s" % f)
        return 1
    print("\nall campaign-runner checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
