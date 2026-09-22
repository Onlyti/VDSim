# Design: campaign runner

Status: **implemented, contract-bound**. The open decisions of the DRAFT are
closed by [21_experiment_runner_spec] §3 (EX1–EX8) and §11, and by
18_dev_briefing_0903 §30. This page documents what was built; the contract
document is the source of truth if the two ever disagree.

## 1. What this layer is

- A **run** is one simulation. Its product is one `.vdtrace`, and that artefact
  is the renderer's only input unit.
- A **campaign** is a set of runs declared by one YAML file.
- Before this layer existed the same question ("run many, how?") had three
  answers in the tree — `tools/vdsim_batch.py`, `tools/campaign_runner.py`,
  `python/sweep_runner.py`. They are now one: `python/vdsim_campaign.py`.

| file | today |
|---|---|
| `python/vdsim_campaign.py` | the runner — expansion, seeds, index, resume, render hook |
| `tools/vdsim_batch.py` | entry point kept at its historical path; forwards, translating `--parallel` to `--jobs` |
| `tools/campaign_runner.py` | `DeprecationWarning` shim; fails and names the replacement |
| `python/sweep_runner.py` | `DeprecationWarning` shim; C++ binary sweeps are out of scope |
| `tools/vdsim_compare.py` | untouched — its axis is vehicle preset and its product a comparison table, a different layer |

The two shims are kept rather than deleted because callers outside this
repository cannot be found by grep. Deletion is a post-P0 item.

## 2. Declaration

```yaml
name: mu_sweep                # campaign id, and the directory name
base: step_steer              # configs/experiments/<name>.yaml, or an inline dict
seed: 20260922                # root seed; per-run seeds derive from it
duration: 4.0                 # optional override of the scenario duration [s]
jobs: 4                       # default concurrency
render: overview              # optional; one render CLI call per finished run
out: campaigns                # optional; default is ./campaigns
sweep:
  grid:                       # orthogonal product
    mu: [0.9, 0.7, 0.5]
    vehicle.mass: [1500, 1700]
  # or: list: [{mu: 0.9}, {mu: 0.5, maneuver.v: 12.0}]
  repeat: 2                   # each combination repeated; only the seed moves
```

- Axis kinds are the three of EX2 and no more: `grid`, `list`, `repeat`.
  Random sampling is not an axis kind — it lives in the legacy `monte_carlo`
  block below.
- Axis keys are dotted paths into the scenario document. `mu`, `vehicle.*` and
  `tire.*` are applied after the preset resolves; everything else is a plain
  key path. There is no alias dictionary.
- YAML only (§10-1). Output is cwd-relative with `--out` overriding it (§10-2).

The pre-promotion `runs:` block still works unchanged, including Monte Carlo:

```yaml
runs:
  - scenario: step_steer
  - sweep:       { base: step_steer, grid: { mu: [0.9, 0.6] } }
  - monte_carlo: { base: step_steer, n: 50,
                   vary: { vehicle.mass: {dist: normal, mean: 1500, std: 50} } }
```

The Monte Carlo draw sequence is unchanged (`random.Random(seed + i)`, default
root seed 1000), so campaigns written against the old tool produce the same
run set.

## 3. Layout

```
campaigns/<campaign_id>/
├── campaign.yaml          # copy of the declaration actually executed
├── index.jsonl            # one line per run
└── <run_id>/
    ├── run.vdtrace        # the run's product
    ├── job.json           # what the child process was handed
    ├── run.log            # that run's stdout/stderr, never interleaved
    ├── result.json        # the child's own report
    └── preview.png        # only with --render
```

`run_id` is a zero-padded ordinal (`000`, `001`, …): sortable and readable.
Axis values are **not** encoded in the directory name — the index holds the
mapping.

## 4. Index keys

One JSON object per line. Required keys (EX3):

| key | meaning |
|---|---|
| `run_id` | zero-padded ordinal |
| `axes` | the axis values of this run |
| `seed` | derived seed, `derive(root_seed, run_index)` |
| `status` | `ok` \| `diverged` \| `error` \| `killed` \| `skipped` |
| `param_hash` | read back from the trace manifest, not recomputed |
| `role` | trace manifest role (`plant`) |
| `trace_path` | campaign-relative path, `null` when no trace was produced |
| `started_at` | UTC ISO-8601 |
| `wall_s` | run wall time [s] |

Also written: `attempt` (EX5), `render_status` (EX7), `error`, `resumed_from`.

**The index is a lookup table, not a result database.** Metrics do not belong
here (EX3, and §2 of the contract puts experiment tracking out of scope).
Compute them in a consumer that reads `vdsim_campaign.read_index(path)`.

## 5. Execution

1. Expand the declaration into an ordered run list; number and seed each run.
2. Run each one **in its own child process**, up to `--jobs` at a time
   (default 1). Process isolation, not threads: it keeps the C++ core's
   process state out of the question, and a run that takes the interpreter
   down costs one index line instead of the campaign (EX5, EX8).
3. Each child records its `.vdtrace` through the ordinary opt-in trace API and
   reports back in `result.json`.
4. Optionally call the existing render CLI on that one trace (EX7). A render
   failure is a render failure: `status` stays `ok` and `render_status`
   carries the error.
5. Write `index.jsonl` in **declaration order**, not completion order.

### Determinism

`seed = derive(root_seed, run_index)` — no wall clock, no pid, no address.
The same declaration yields the same seeds and the same channel bytes whether
it runs with `--jobs 1` or `--jobs 4`.

Caveat worth knowing: on the scenario path the seed currently reaches only the
sensor noise model, and the scenario autopilots steer from ground truth rather
than from measurements. A `repeat` axis therefore produces identical traces
today. The seed is still derived and recorded, so nothing has to change here
when a stochastic input is added.

### Resume

`--resume` skips a run only when all three of EX6 hold: the recorded status is
`ok`, the trace file is still there, and its `param_hash` matches what the
declaration now asks for. If the declaration itself moved since
`campaign.yaml` was written, the resume is refused outright — two parameter
sets must not end up mixed in one campaign directory.

### Retries

None by default. `--retry N` opts in and records `attempt`. Silent retries
would bias a campaign toward the runs that happen to pass; the SIGFPE hunt of
Q0 is the case in point.

## 6. What this layer does not touch

- **The trace contract.** No field is added. The campaign context (axis values,
  campaign id) lives in the index, never in the manifest. The one allowed
  mention is `producer.name = "vdsim_campaign"`, which the manifest already
  required (EX1).
- **The render CLI.** The runner owns no render code; it calls the CLI on one
  trace (EX7).
- **Overlay rendering and aggregate plots.** Those stay where they are; a
  consumer script that reads the index is the place for them.

## 7. A `level` axis is refused

`L4` and `L3` are the same physics unless suspension hardpoints are attached,
and the scenario path attaches none. A level sweep would therefore produce
identical runs whose manifests disagree about which model produced them, and
the false `model_level` outlives any warning printed at run time. So the runner
refuses the axis outright, on every declaration form:

```
CampaignError: 'level' is not a campaign axis until Q20: L4 is identical to L3
unless suspension hardpoints are attached, so a level sweep records traces
whose manifest model_level is false. Declare one level per campaign.
```

Declare one level per campaign instead. The refusal is lifted by Q20, which
attaches the hardpoints and adds a `kinematics_attached` field to the manifest
so a reader can tell the two apart. See 18_dev_briefing_0903 §30.4, §31 C-1.

## 8. CLI

```sh
vdsim-campaign run campaign.yaml                      # sequential
vdsim-campaign run campaign.yaml --jobs 4             # four at a time
vdsim-campaign run campaign.yaml --resume             # continue a campaign
vdsim-campaign run campaign.yaml --render overview    # render each run
vdsim-campaign run campaign.yaml --dry                # list the expansion only
python3 tools/vdsim_batch.py run campaign.yaml        # same, historical path
```
