#!/usr/bin/env python3
"""vdsim_campaign -- the layer above one run: declare many, record each.

A *run* is one simulation and its product is one ``.vdtrace``; a *campaign* is
a set of runs declared by a single YAML file (21_experiment_runner_spec 0,
11.2). This module owns only that upper layer -- axis expansion, deterministic
seeds, the run directory and index conventions, failure isolation, resume and a
thin render hook. It adds nothing to the trace contract and changes nothing in
the render CLI.

    vdsim-campaign run campaign.yaml --jobs 4 --render overview
    python3 -m vdsim_campaign run campaign.yaml --resume

Declaration (YAML, one file)::

    name: mu_sweep
    base: skidpad                 # configs/experiments/<name>.yaml, or an inline dict
    seed: 20260922                # root seed; per-run seeds derive from it
    duration: 10.0                # optional override of the scenario duration
    jobs: 4
    render: overview              # optional render preset, one call per run
    sweep:
      grid:                       # orthogonal product of the listed axes
        mu: [0.9, 0.6]
        vehicle.mass: [1500, 1700]
      repeat: 2                   # each combination repeated; only the seed moves

``sweep.list`` names the combinations explicitly instead of taking a product,
and the legacy ``runs:`` block of ``tools/vdsim_batch.py`` (``scenario`` /
``sweep`` / ``monte_carlo``) is still accepted unchanged.

Directory layout (EX3)::

    campaigns/<campaign_id>/
    |-- campaign.yaml        # copy of the declaration actually executed
    |-- index.jsonl          # one line per run, query-only
    `-- <run_id>/run.vdtrace

The index is a lookup table, not a result database: it carries the axis values
and the run's disposition, never metrics (EX3).

A ``level`` axis is allowed since Q20 (18_dev_briefing_0903 §51). It was
refused while an ``L4`` run with no suspension hardpoints still produced a
trace claiming ``model_level: L4``; now the session seam refuses such a run
(it lands in the index as ``error``) and every trace states
``kinematics_attached``, so a level sweep can no longer archive a false label.
"""
import argparse
import copy
import hashlib
import itertools
import json
import os
import random
import subprocess
import sys
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent

#: Keys every index line carries (EX3). Producers may add keys; they may not
#: drop these.
INDEX_KEYS = ("run_id", "axes", "seed", "status", "param_hash", "role",
              "trace_path", "started_at", "wall_s")

#: Dispositions a run may end in (EX5). ``skipped`` is what ``--resume``
#: writes for a run it did not re-execute.
STATUSES = ("ok", "diverged", "error", "killed", "skipped")

#: Root seed used when the declaration names none. Chosen to reproduce the
#: Monte Carlo stream of the pre-promotion ``tools/vdsim_batch.py``
#: (``random.Random(1000 + i)``) so existing campaigns do not move.
DEFAULT_ROOT_SEED = 1000

#: Axes a campaign may not sweep, with the message the refusal carries.
#: Empty since Q20 lifted ``level`` (C-1 condition v): the refusal existed
#: because a bare L4 run recorded a false ``model_level``, and that run is now
#: rejected where the session is built. The mechanism stays so a future axis
#: with the same property is refused in the one place every form goes through.
FORBIDDEN_AXES = {}

#: Trace file name inside a run directory. Fixed, so a consumer can find the
#: artefact from the run id alone.
TRACE_NAME = "run.vdtrace"


class CampaignError(Exception):
    """Raised on a malformed declaration or a refused resume."""


# --------------------------------------------------------------------------- #
# Declaration -> run list
# --------------------------------------------------------------------------- #
def _config_root():
    """Scenario config root, mirroring ``vdsim_lab._conf_root``."""
    for c in (REPO / "configs", Path.cwd() / "configs",
              Path(__file__).resolve().parent / "vdsim_configs"):
        if c.is_dir():
            return c
    return REPO / "configs"


def _load_scenario(name_or_cfg):
    """Resolve a scenario reference to a fresh config dict.

    :param name_or_cfg: ``configs/experiments/<name>.yaml`` stem, or an inline
        scenario dict.
    :returns: a deep copy, so callers may mutate it freely.
    """
    if isinstance(name_or_cfg, dict):
        return copy.deepcopy(name_or_cfg)
    path = _config_root() / "experiments" / ("%s.yaml" % name_or_cfg)
    if not path.is_file():
        raise CampaignError("scenario %r not found at %s" % (name_or_cfg, path))
    with open(path) as fh:
        return yaml.safe_load(fh)


def _set(cfg, key, val):
    """Apply one dotted axis value onto a scenario config.

    ``vehicle.*`` / ``tire.*`` / ``mu`` are late overrides applied after the
    preset is loaded, so they land in ``_overrides``; everything else is a plain
    key path in the scenario document.

    :param cfg: scenario config dict, mutated in place.
    :param key: dotted key path, e.g. ``vehicle.mass`` or ``maneuver.v``.
    :param val: value to set.
    :raises CampaignError: if the key names a forbidden axis
        (:data:`FORBIDDEN_AXES`). Every expansion form routes its axis values
        through here, so the refusal cannot be reached around.
    """
    if key in FORBIDDEN_AXES:
        raise CampaignError(FORBIDDEN_AXES[key])
    if key.startswith(("vehicle.", "tire.")) or key == "mu":
        cfg.setdefault("_overrides", {})[key] = val
    else:
        d = cfg
        parts = key.split(".")
        for p in parts[:-1]:
            d = d.setdefault(p, {})
        d[parts[-1]] = val


def _sample(spec, rng):
    """Draw one Monte Carlo value from a ``vary`` entry."""
    dist = spec.get("dist", "uniform")
    if dist == "normal":
        return rng.gauss(spec["mean"], spec["std"])
    return rng.uniform(spec["lo"], spec["hi"])


def derive_seed(root_seed, run_index):
    """Per-run seed, derived only from the declaration (EX4).

    Wall-clock, pid and address-derived seeds are excluded by construction: the
    same declaration yields the same seeds on any machine, in any order.

    :param root_seed: campaign root seed.
    :param run_index: zero-based ordinal of the run in the expanded list.
    :returns: 32-bit non-negative seed.
    """
    digest = hashlib.sha256(("%d:%d" % (int(root_seed), int(run_index)))
                            .encode("utf-8")).digest()
    return int.from_bytes(digest[:4], "big")


def _expand_sweep(campaign):
    """Expand the ``base`` + ``sweep`` form into ``[{cfg, axes}, ...]``."""
    base = campaign.get("base")
    if base is None:
        raise CampaignError("declaration needs 'base' (or a legacy 'runs' block)")
    sweep = campaign.get("sweep") or {}
    if "grid" in sweep and "list" in sweep:
        raise CampaignError("sweep: use 'grid' or 'list', not both")

    if "grid" in sweep:
        grid = sweep["grid"]
        if not isinstance(grid, dict) or not grid:
            raise CampaignError("sweep.grid must be a non-empty mapping")
        keys = list(grid)
        combos = [dict(zip(keys, values))
                  for values in itertools.product(*[grid[k] for k in keys])]
    elif "list" in sweep:
        items = sweep["list"]
        if not isinstance(items, list) or not items:
            raise CampaignError("sweep.list must be a non-empty list")
        combos = [dict(item) for item in items]
    else:
        combos = [{}]

    repeat = int(sweep.get("repeat", 1))
    if repeat < 1:
        raise CampaignError("sweep.repeat must be >= 1")

    out = []
    for combo in combos:
        for r in range(repeat):
            cfg = _load_scenario(base)
            for key, value in combo.items():
                _set(cfg, key, value)
            axes = dict(combo)
            if repeat > 1:
                axes["repeat"] = r
            out.append({"cfg": cfg, "axes": axes})
    return out


def _expand_legacy(campaign, root_seed):
    """Expand the pre-promotion ``runs:`` block (scenario / sweep / monte_carlo).

    Kept identical in behaviour so campaigns written against
    ``tools/vdsim_batch.py`` keep producing the same run set, including the
    Monte Carlo draw sequence.
    """
    out = []
    for item in campaign.get("runs", []):
        if "scenario" in item:
            out.append({"cfg": _load_scenario(item["scenario"]),
                        "axes": {"scenario": item["scenario"]}})
        elif "sweep" in item:
            sw = item["sweep"]
            keys = list(sw["grid"])
            for values in itertools.product(*[sw["grid"][k] for k in keys]):
                cfg = _load_scenario(sw["base"])
                axes = dict(zip(keys, values))
                for key, value in axes.items():
                    _set(cfg, key, value)
                axes["scenario"] = sw["base"]
                out.append({"cfg": cfg, "axes": axes})
        elif "monte_carlo" in item:
            mc = item["monte_carlo"]
            for i in range(int(mc["n"])):
                rng = random.Random(root_seed + i)
                cfg = _load_scenario(mc["base"])
                axes = {k: _sample(spec, rng)
                        for k, spec in (mc.get("vary") or {}).items()}
                for key, value in axes.items():
                    _set(cfg, key, value)
                axes["scenario"] = mc["base"]
                axes["mc_draw"] = i
                out.append({"cfg": cfg, "axes": axes})
        else:
            raise CampaignError("runs[] entry needs scenario/sweep/monte_carlo: %r"
                                % (item,))
    return out


def expand(campaign):
    """Turn a campaign declaration into an ordered run list.

    :param campaign: parsed declaration dict.
    :returns: list of ``{"cfg": dict, "axes": dict}`` in declaration order. That
        order is the run numbering, so it must not depend on iteration that
        varies between interpreters.
    """
    root_seed = int(campaign.get("seed", DEFAULT_ROOT_SEED))
    if campaign.get("runs"):
        return _expand_legacy(campaign, root_seed)
    return _expand_sweep(campaign)


# --------------------------------------------------------------------------- #
# Index
# --------------------------------------------------------------------------- #
def read_index(path):
    """Read an ``index.jsonl`` into a list of dicts (5, the only query helper).

    :param path: index file, or the campaign directory holding it.
    :returns: one dict per run, in file order.
    """
    path = Path(path)
    if path.is_dir():
        path = path / "index.jsonl"
    rows = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _write_index(path, rows):
    """Write ``index.jsonl`` atomically, one compact JSON object per line."""
    tmp = Path(str(path) + ".tmp")
    with open(tmp, "w") as fh:
        for row in rows:
            fh.write(json.dumps(row, sort_keys=True) + "\n")
    tmp.replace(path)
    return path


# --------------------------------------------------------------------------- #
# Runner
# --------------------------------------------------------------------------- #
class CampaignRunner:
    """Execute a campaign declaration into ``campaigns/<id>/``.

    One run is one child process, so a run that takes the interpreter down is
    isolated to its own index line instead of ending the campaign (EX5, EX8).
    """

    def __init__(self, campaign, spec_path=None, out=None, jobs=None,
                 resume=False, render=None, retry=0):
        """
        :param campaign: parsed declaration dict.
        :param spec_path: path the declaration was read from, used for the
            default campaign id.
        :param out: parent directory for campaign directories. Defaults to the
            declaration's ``out``, else ``./campaigns`` (10-2).
        :param jobs: concurrent child processes; defaults to the declaration's
            ``jobs``, else 1 (EX8).
        :param resume: skip runs that are verifiably complete (EX6).
        :param render: render preset name, or ``None`` for no render (EX7).
        :param retry: re-execution attempts for a failed run; 0 by default,
            because silent retries bias a campaign toward the runs that happen
            to pass (EX5).
        """
        self.campaign = campaign
        self.spec_path = Path(spec_path) if spec_path else None
        self.campaign_id = str(campaign.get("name")
                               or (self.spec_path.stem if self.spec_path else "campaign"))
        parent = Path(out) if out else Path(campaign.get("out") or "campaigns")
        self.root = parent / self.campaign_id
        self.jobs = int(jobs if jobs else campaign.get("jobs") or 1)
        if self.jobs < 1:
            raise CampaignError("--jobs must be >= 1")
        self.resume = bool(resume)
        self.render = render if render is not None else campaign.get("render")
        self.retry = int(retry)
        self.root_seed = int(campaign.get("seed", DEFAULT_ROOT_SEED))
        self.duration = campaign.get("duration")

    # -- declaration bookkeeping ------------------------------------------
    def _declaration_text(self):
        """Canonical text of the declaration as executed."""
        return yaml.safe_dump(self.campaign, sort_keys=True, default_flow_style=False)

    def _check_declaration(self):
        """Refuse a resume whose declaration moved since the recorded run (EX6)."""
        copy_path = self.root / "campaign.yaml"
        if not copy_path.exists() or not self.resume:
            return
        if copy_path.read_text() != self._declaration_text():
            raise CampaignError(
                "declaration changed since %s was written; refusing --resume. "
                "Runs of two different parameter sets must not share one "
                "campaign directory (EX6)." % copy_path)

    # -- run planning ------------------------------------------------------
    def plan(self):
        """Expanded run list with ids, seeds and the param hash of each run.

        :returns: list of job dicts, ready to hand to a child process.
        """
        import vdsim_trace

        runs = expand(self.campaign)
        width = max(3, len(str(max(len(runs) - 1, 0))))
        jobs = []
        for i, item in enumerate(runs):
            run_id = str(i).zfill(width)
            params = {"scenario": item["cfg"], "duration": self.duration}
            jobs.append({
                "run_id": run_id,
                "axes": item["axes"],
                "seed": derive_seed(self.root_seed, i),
                "cfg": item["cfg"],
                "duration": self.duration,
                "param_hash": vdsim_trace.param_hash(params),
                "run_dir": str(self.root / run_id),
                "trace": str(self.root / run_id / TRACE_NAME),
                "render": self.render,
                "campaign_id": self.campaign_id,
            })
        return jobs

    # -- resume ------------------------------------------------------------
    def _reusable(self, job, previous):
        """Whether ``--resume`` may keep a run instead of re-executing it.

        The three conditions of EX6 are ANDed: the recorded disposition is
        ``ok``, the trace is still on disk, and its ``param_hash`` matches what
        this declaration now asks for.
        """
        row = previous.get(job["run_id"])
        if row is None or row.get("status") not in ("ok", "skipped"):
            return False
        trace = Path(row.get("trace_path") or "")
        if not str(trace):
            return False
        if not trace.is_absolute():
            trace = self.root / trace
        if not trace.is_file():
            return False
        return row.get("param_hash") == job["param_hash"]

    # -- execution ---------------------------------------------------------
    def _spawn(self, job):
        """Start one run as a child process; returns ``(popen, log_handle)``."""
        run_dir = Path(job["run_dir"])
        run_dir.mkdir(parents=True, exist_ok=True)
        job_path = run_dir / "job.json"
        job_path.write_text(json.dumps(job, sort_keys=True, indent=2))
        env = dict(os.environ)
        extra = [p for p in sys.path if p and Path(p).is_dir()]
        env["PYTHONPATH"] = os.pathsep.join(
            [p for p in extra + [env.get("PYTHONPATH", "")] if p])
        log = open(run_dir / "run.log", "w")
        proc = subprocess.Popen(
            [sys.executable, "-m", "vdsim_campaign", "exec", str(job_path)],
            stdout=log, stderr=subprocess.STDOUT, env=env)
        return proc, log

    def _collect(self, job, proc):
        """Turn a finished child into one index row."""
        run_dir = Path(job["run_dir"])
        result_path = run_dir / "result.json"
        row = {
            "run_id": job["run_id"],
            "axes": job["axes"],
            "seed": job["seed"],
            "param_hash": job["param_hash"],
            "role": "plant",
            "trace_path": job["run_id"] + "/" + TRACE_NAME,
            "started_at": None,
            "wall_s": None,
            "status": "error",
        }
        if result_path.is_file():
            row.update(json.loads(result_path.read_text()))
        if proc.returncode < 0:
            row["status"] = "killed"
            row["error"] = "child terminated by signal %d" % (-proc.returncode)
        elif proc.returncode != 0:
            row["status"] = "error"
            if not row.get("error"):
                tail = (run_dir / "run.log").read_text().strip().splitlines()[-1:]
                row["error"] = tail[0] if tail else ("child exited %d" % proc.returncode)
        if not Path(job["trace"]).is_file():
            row["trace_path"] = None
        return row

    def run(self):
        """Execute (or resume) the campaign.

        :returns: path of ``index.jsonl``. A result object tree is deliberately
            not returned -- consumers read the index (5).
        """
        self.root.mkdir(parents=True, exist_ok=True)
        self._check_declaration()
        previous = {}
        index_path = self.root / "index.jsonl"
        if self.resume and index_path.is_file():
            previous = {row["run_id"]: row for row in read_index(index_path)}
        (self.root / "campaign.yaml").write_text(self._declaration_text())

        jobs = self.plan()
        rows = {}
        pending = []
        for job in jobs:
            if self.resume and self._reusable(job, previous):
                row = dict(previous[job["run_id"]])
                row["status"] = "skipped"
                row["resumed_from"] = "ok"
                rows[job["run_id"]] = row
                continue
            pending.append(job)

        attempts = dict((job["run_id"], 0) for job in pending)
        queue = list(pending)
        running = []
        done = 0
        while queue or running:
            while queue and len(running) < self.jobs:
                job = queue.pop(0)
                attempts[job["run_id"]] += 1
                proc, log = self._spawn(job)
                running.append((job, proc, log))
            job, proc, log = running.pop(0)
            proc.wait()
            log.close()
            row = self._collect(job, proc)
            row["attempt"] = attempts[job["run_id"]]
            if row["status"] not in ("ok", "skipped") and attempts[job["run_id"]] <= self.retry:
                queue.append(job)
                continue
            rows[job["run_id"]] = row
            done += 1
            sys.stderr.write("[campaign] %s %s (%d/%d)\n"
                             % (job["run_id"], row["status"], done, len(pending)))
            sys.stderr.flush()

        ordered = [rows[job["run_id"]] for job in jobs]
        _write_index(index_path, ordered)
        ok = sum(1 for r in ordered if r["status"] in ("ok", "skipped"))
        bad = [r["run_id"] for r in ordered if r["status"] not in ("ok", "skipped")]
        sys.stderr.write("[campaign] %s: %d/%d ok%s -> %s\n"
                         % (self.campaign_id, ok, len(jobs),
                            (", failed: " + ",".join(bad)) if bad else "",
                            index_path))
        return index_path


# --------------------------------------------------------------------------- #
# Child process: one run
# --------------------------------------------------------------------------- #
def _render_one(job, trace_path, run_dir):
    """Call the existing render CLI on one trace (EX7).

    The runner owns no render code, and a failure here is a render failure and
    not a run failure, so it is reported on its own key.

    :returns: ``"ok"`` or ``"error: <last line>"``.
    """
    cmd = [sys.executable, "-m", "vdsim_render", str(trace_path),
           "--preset", str(job["render"]),
           "--png", str(run_dir / "preview.png")]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if proc.returncode == 0:
        return "ok"
    tail = proc.stdout.decode("utf-8", "replace").strip().splitlines()
    return "error: " + (tail[-1] if tail else "render exited %d" % proc.returncode)


def _diverged(trace_path):
    """Whether a finished run left a non-finite pose or body velocity behind."""
    import numpy as np
    import vdsim_trace
    with vdsim_trace.TraceReader(trace_path) as tr:
        for name in ("pose", "v_body"):
            if tr.has(name) and not np.isfinite(tr.channel(name)).all():
                return True
    return False


def exec_job(job_path):
    """Run one job declaration in this process and write ``result.json``.

    The child half of :meth:`CampaignRunner.run`. It exists as a subcommand so
    that a run which takes the interpreter down costs one index line rather
    than the campaign.

    :param job_path: path of the ``job.json`` written by the parent.
    :returns: process exit code.
    """
    job = json.loads(Path(job_path).read_text())
    run_dir = Path(job["run_dir"])
    trace_path = Path(job["trace"])
    started = time.time()
    result = {"status": "error", "started_at": _iso(started), "wall_s": 0.0,
              "role": "plant", "param_hash": job["param_hash"]}
    try:
        import vdsim_lab as lab

        exp = lab.Experiment.from_config(job["cfg"])
        if exp._sensors is not None:
            exp._sensors.sp.seed = int(job["seed"])
        if trace_path.exists():
            trace_path.unlink()
        exp.enable_trace(trace_path, seed=int(job["seed"]), run_id=job["run_id"],
                         params={"scenario": job["cfg"], "duration": job["duration"]},
                         producer={"name": "vdsim_campaign", "version": _version()})
        exp.run(job["duration"])
        exp.finalize_trace()
        result["status"] = "diverged" if _diverged(trace_path) else "ok"
        result["param_hash"] = _trace_param_hash(trace_path, job["param_hash"])
    except Exception as exc:                # noqa: BLE001 -- isolation is the point
        result["status"] = "error"
        result["error"] = "%s: %s" % (type(exc).__name__, exc)
    result["wall_s"] = round(time.time() - started, 4)
    if job.get("render") and result["status"] in ("ok", "diverged"):
        result["render_status"] = _render_one(job, trace_path, run_dir)
    (run_dir / "result.json").write_text(json.dumps(result, sort_keys=True, indent=2))
    return 0


def _trace_param_hash(trace_path, fallback):
    """Read back the hash the trace recorded, so index and trace cannot drift."""
    try:
        import vdsim_trace
        with vdsim_trace.TraceReader(trace_path) as tr:
            return tr.repro.get("param_hash", fallback)
    except Exception:
        return fallback


def _iso(epoch):
    """UTC ISO-8601 stamp for an epoch time."""
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(epoch))


def _version():
    """Runner version string, reported in the trace ``producer`` block."""
    try:
        import vdsim_trace
        return getattr(vdsim_trace, "SCHEMA_VERSION", "0")
    except Exception:
        return "0"


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main(argv=None):
    """CLI entry point (``vdsim-campaign``)."""
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd")

    run = sub.add_parser("run", help="expand and execute a campaign declaration")
    run.add_argument("campaign", type=Path, help="campaign declaration YAML")
    run.add_argument("--out", type=Path, default=None,
                     help="parent directory of campaign dirs (default: ./campaigns)")
    run.add_argument("--jobs", type=int, default=None,
                     help="concurrent runs, one process each (default: 1)")
    run.add_argument("--resume", action="store_true",
                     help="skip runs that are complete and still match the declaration")
    run.add_argument("--render", default=None,
                     help="render preset; one render CLI call per finished run")
    run.add_argument("--retry", type=int, default=0,
                     help="re-execution attempts per failed run (default: 0)")
    run.add_argument("--dry", action="store_true",
                     help="print the expanded run list and exit")

    ex = sub.add_parser("exec", help=argparse.SUPPRESS)
    ex.add_argument("job", type=Path)

    args = ap.parse_args(argv)
    if args.cmd == "exec":
        return exec_job(args.job)
    if args.cmd != "run":
        ap.print_help()
        return 2

    with open(args.campaign) as fh:
        campaign = yaml.safe_load(fh)
    runner = CampaignRunner(campaign, spec_path=args.campaign, out=args.out,
                            jobs=args.jobs, resume=args.resume,
                            render=args.render, retry=args.retry)
    if args.dry:
        for job in runner.plan():
            print(job["run_id"], json.dumps(job["axes"], sort_keys=True),
                  "seed=%d" % job["seed"])
        return 0
    runner.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
