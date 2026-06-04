# Design: experiment batch runner

Status: DRAFT for alignment. Run many experiments headless (explicit scenarios +
parameter sweeps + Monte Carlo), collect per-run data and a metrics table.

## 1. Why / what's already there

The builder authors `configs/experiments/*.yaml` and `vdsim_lab.Experiment.
from_config(name)` runs ONE. We have, but fragmented and not on the authored
configs:
- `python/sweep_runner.py` — cartesian sweep over a **C++ binary**, dotted-path params.
- `apps/doe/` — `metrics.py` (peak_yaw_rate, ss_yaw, …), `scenarios.py`, a DoE harness.
- `examples/monte_carlo.py` (#127) — stochastic sampling.

Gap: a **campaign runner** that expands explicit + swept + MC runs of the authored
experiment configs, runs them in parallel headless, and reduces each to metrics.

## 2. Campaign spec (one YAML)

```yaml
name: fdr_vs_surface
runs:
  - scenario: yongin_lap                    # configs/experiments/*.yaml, run as-is
  - sweep:                                  # base + grid -> cartesian product
      base: yongin_lap
      grid:
        vehicle.final_drive_ratio: [4.0, 5.0, 6.0]
        road.surface: [minor_road, belgian_pave]
        maneuver.v: [25, 30]
  - monte_carlo:                            # base + stochastic samples
      base: skidpad
      n: 200
      vary:
        vehicle.mass: { dist: normal, mean: 1500, std: 50 }
        mu:           { dist: uniform, lo: 0.7, hi: 1.0 }
metrics: [lap_time, peak_ay, understeer_K, max_Fz, dist]
output: results/fdr_vs_surface/             # per-run CSV + summary.csv + resolved/
parallel: 8
duration: 40
```

Overrides use **dotted paths** on the experiment config (`vehicle.*` / `tire.*` /
`road.*` / `maneuver.*` / `mu` / `level`). `vehicle.X` loads the vehicle preset,
overrides field X in-memory, runs.

## 3. Execution

1. **Expand** `runs` -> a flat list of `(run_id, resolved_config, params)`:
   sweep = itertools.product of the grid; monte_carlo = N seeded samples.
2. **Run** each with `vdsim_lab.Experiment.from_config(cfg, overrides=...)`, headless,
   in a `multiprocessing.Pool(parallel)` — runs are independent (embarrassingly
   parallel). Each worker writes `results/<name>/<run_id>.csv` (Result.to_csv) +
   the resolved config to `resolved/<run_id>.yaml` (reproducibility).
3. **Reduce** each Result to the requested `metrics` (registry name -> fn(Result)).
4. **Aggregate** -> `summary.csv`: one row per run = {run_id, params…, metrics…}.
   Failures are captured (error logged, row marked failed) and don't kill the batch.

## 4. Metrics

A name->function registry reusing/extending `apps/doe/metrics.py` on the `Result`:
`lap_time` (closed-loop return-to-start), `peak_ay`, `understeer_K`, `max_Fz`,
`dist`, `rms_slip`, `vmax`, `min_mu_margin`, … Users add their own.

## 5. CLI

```sh
python tools/vdsim_batch.py run campaign.yaml          # run the campaign
python tools/vdsim_batch.py run campaign.yaml --dry    # list the expanded runs
```
(Builder web-tool "Batch" tab is a later add; batch is headless automation first.)

## 6. Open decisions

1. **Run path = vdsim_lab Python + multiprocessing** (reuses authored configs; sim
   core is C++; perf fine) — agree? (vs the C++ sweep_runner binary path.)
2. **Output**: per-run CSV + summary.csv (metrics table) + resolved config per run.
   Add parquet later? CSV first — agree?
3. **Parallelism**: `multiprocessing.Pool(parallel)`, default = cpu_count. OK?
4. **Monte Carlo** folded into the same spec (reuse #127's sampling), or keep
   `examples/monte_carlo.py` separate and only do explicit+sweep here?
5. **CLI-first**, builder "Batch" tab later — agree?
6. Need **resume / caching** (skip runs whose output exists) in v1, or later?
