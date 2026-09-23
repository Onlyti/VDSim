# Positioning — when to use VDSim, and when not to

This page helps you pick the right tool, including *not* VDSim when a production suite is
what you need. It compares tool classes by **category and intent**, not by accuracy, and does not
name commercial products.

!!! warning "No parity claims"
    VDSim does not claim to match or replace commercial production tools. It makes no
    commercial tire-product parity claim, no real-vehicle validation claim, and no
    production sign-off claim. See [Validation](VALIDATION.md).

## By category

| Tool class | Examples | Strengths | Trade-off vs. VDSim |
|---|---|---|---|
| Commercial full-vehicle suites | — | production-validated full-vehicle fidelity, GUIs, vendor support | closed source, licensed, heavier to script and embed |
| Open multibody dynamics | Project Chrono | general multibody, broad physics | not control-focused; VDSim uses it as an **independent cross-check** |
| Game / AV simulation engines | CARLA and similar | sensors, rendering, scenarios, traffic | simplified vehicle dynamics; the ego is often friction-blind |
| **VDSim** | — | deterministic, per-wheel ground truth, scriptable, open core, control-research focus | experimental, pre-validation, not a production reference |

## Where VDSim is the right choice

- You need a **deterministic, scriptable plant** inside a control or estimation loop,
  not a GUI workbench.
- You need **per-wheel ground truth** (slip, load, force) as observable signals.
- You want an **open** core you can read, modify and cite, with every check
  reproducible from the repository.
- You study the **friction-limit gap** between a kinematic design model and tire
  dynamics.
- You need many fast, resettable copies of the plant for learning or sampling
  ([RL environment](RL_GUIDE.md), [Experiment API](EXPERIMENT_API.md)).

## Where it is not

- You need a **production-validated** full-vehicle model with vendor sign-off — use a
  commercial suite.
- You need **sensor simulation, rendering or traffic** — use an AV simulator, and
  consider VDSim as the ego plant inside it
  ([example](examples/external_benchmark_ego_swap.md)).
- You need validated **ride / NVH or detailed K&C** — out of scope.

## Relationship, not rivalry

VDSim's verification *uses* these tools: pure-slip tire forces are cross-checked against
a commercial Magic Formula tyre implementation and against Chrono's independent Pac02 on the same
public `.tir` parameters ([Validation](VALIDATION.md)). The framing is complementary: VDSim
is the open, deterministic, embeddable plant for control research, and it says so
without overclaiming.
