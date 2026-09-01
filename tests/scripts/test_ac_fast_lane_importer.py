#!/usr/bin/env python3
"""Separated contract and end-to-end tests for the AC M1 path importer."""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools" / "ac_fast_lane_importer.py"
FIXTURE = ROOT / "tests" / "fixtures" / "ac_fast_lane" / "synthetic_v7_points.json"
TRACE_FIXTURE = ROOT / "tests" / "fixtures" / "trace" / "golden_v0_1.vdtrace"
sys.path.insert(0, str(ROOT / "python"))

spec = importlib.util.spec_from_file_location("ac_fast_lane_importer", TOOL)
ac = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = ac
spec.loader.exec_module(ac)

from vdsim_render import LoadedTrace, render  # noqa: E402
from vdsim_trace import attach_overlay  # noqa: E402

CHECKS = 0


def check(condition, message):
    """Record one assertion with readable raw ctest output."""
    global CHECKS
    if not condition:
        raise AssertionError(message)
    CHECKS += 1
    print("ok  :", message)


def write_fast_lane(path: Path, fixture: dict, *, version=7, details=True):
    """Encode the synthetic JSON points in the documented v7 layout."""
    points = fixture["points"]
    payload = bytearray(struct.pack("<4i", version, len(points), 0, 0))
    source_s = 0.0
    previous = None
    for index, point in enumerate(points):
        xyz = tuple(float(v) for v in point["position"])
        if previous is not None:
            source_s += math.dist(previous, xyz)
        payload.extend(struct.pack("<4fi", *xyz, source_s, index))
        previous = xyz
    payload.extend(struct.pack("<i", len(points) if details else 0))
    if details:
        for point in points:
            values = [0.0] * 18
            values[5] = float(point["left_width_m"])
            values[6] = float(point["right_width_m"])
            values[9:12] = [0.0, 1.0, 0.0]
            values[12] = 1.0
            values[13:16] = [0.0, 0.0, 1.0]
            payload.extend(struct.pack("<18f", *values))
    payload.extend(struct.pack("<i", 0))
    path.write_bytes(payload)


def load_rows(path: Path):
    """Read the contract CSV without changing its values."""
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def test_roundtrip_and_contract(tmp: Path, fixture: dict):
    """Validate required columns, coordinate transform, widths and evidence."""
    source = tmp / "fast_lane.ai"
    write_fast_lane(source, fixture)
    mtime_ns = source.stat().st_mtime_ns
    out = tmp / "bundle"
    result = ac.import_fast_lane(
        source, out, source_track_id="synthetic/layout",
        transform_source_to_vdsim=fixture["transform_source_to_vdsim"],
        closed_loop=True, generated_at="2026-09-01T00:00:00Z",
    )
    check(source.stat().st_mtime_ns == mtime_ns,
          "source fast_lane.ai mtime is unchanged")
    rows = load_rows(result.reference_path)
    check(tuple(rows[0]) == ac.CSV_COLUMNS,
          "reference_path.csv has the exact seven mandatory columns")
    check([int(row["index"]) for row in rows] == list(range(4)),
          "source sample order and IDs are preserved")
    expected = [(40.0, 5.0, 1.0), (50.0, 9.0, 1.5),
                (60.0, 14.0, 2.0), (70.0, 17.0, 2.5)]
    actual = [(float(row["x_m"]), float(row["y_m"]), float(row["z_m"]))
              for row in rows]
    check(actual == expected,
          "explicit source-to-ISO transform preserves 3D height")
    check(all(math.isclose(actual, expected, abs_tol=1e-6, rel_tol=0.0)
              for actual, expected in zip(
                  [float(row["left_width_m"]) for row in rows],
                  [4.0, 4.2, 4.4, 4.6])),
          "left widths come from v7 detail records")
    check(all(math.isclose(actual, expected, abs_tol=1e-6, rel_tol=0.0)
              for actual, expected in zip(
                  [float(row["right_width_m"]) for row in rows],
                  [3.5, 3.7, 3.9, 4.1])),
          "right widths come from v7 detail records")
    s_values = [float(row["s_m"]) for row in rows]
    check(s_values[0] == 0.0 and all(b > a for a, b in zip(s_values, s_values[1:])),
          "s_m is a strictly increasing 3D arc-length station")

    manifest = yaml.safe_load(result.manifest.read_text(encoding="utf-8"))
    required = {"schema_version", "source", "source_track_id", "source_hash",
                "coordinate_frame", "units", "transform_source_to_vdsim",
                "generated_at", "included_layers"}
    check(required <= set(manifest), "manifest contains every section 5.1 field")
    check(manifest["source"] == "assetto_corsa"
          and manifest["coordinate_frame"]["standard"] == "ISO 8855",
          "manifest identifies Assetto Corsa source and ISO 8855 output")
    check(manifest["closed_loop"] is True
          and manifest["included_layers"] == ["reference_path"],
          "manifest records explicit loop state and M1 layer only")
    check(manifest["transform_source_to_vdsim"]
          == fixture["transform_source_to_vdsim"],
          "manifest preserves the exact 4x4 transform")
    report = result.report.read_text(encoding="utf-8")
    check("| Point count | 4 | 4 |" in report
          and "| Closed loop | declared: true | true |" in report,
          "report contains source/output point-count and loop comparison")
    check("Source XYZ" in report and "VDSim XYZ" in report,
          "report contains transform bounding boxes")
    return source, result


def test_determinism(tmp: Path, source: Path, fixture: dict):
    """The timestamp is the only allowed byte difference between runs."""
    outputs = []
    for index, timestamp in enumerate(("2026-09-01T00:00:00Z",
                                       "2026-09-01T00:00:01Z")):
        out = tmp / ("det-%d" % index)
        ac.import_fast_lane(
            source, out, source_track_id="synthetic/layout",
            transform_source_to_vdsim=fixture["transform_source_to_vdsim"],
            closed_loop=True, generated_at=timestamp,
        )
        outputs.append(out)
    check((outputs[0] / "reference_path.csv").read_bytes()
          == (outputs[1] / "reference_path.csv").read_bytes(),
          "CSV bytes are deterministic")
    check((outputs[0] / "report.md").read_bytes()
          == (outputs[1] / "report.md").read_bytes(),
          "report bytes are deterministic")
    manifests = [yaml.safe_load((out / "manifest.yaml").read_text(encoding="utf-8"))
                 for out in outputs]
    for manifest in manifests:
        manifest.pop("generated_at")
    check(manifests[0] == manifests[1],
          "manifest differs only in the permitted generated_at field")


def test_fail_closed_and_fallback(tmp: Path, fixture: dict):
    """Damage, guessed transforms and silent zero widths are rejected."""
    unsupported = tmp / "unsupported.ai"
    write_fast_lane(unsupported, fixture, version=6)
    try:
        ac.parse_fast_lane(unsupported)
    except ac.FastLaneImportError as exc:
        check("unsupported" in str(exc), "unsupported AI version fails immediately")
    else:
        raise AssertionError("unsupported version accepted")

    truncated = tmp / "truncated.ai"
    write_fast_lane(truncated, fixture)
    truncated.write_bytes(truncated.read_bytes()[:-20])
    try:
        ac.parse_fast_lane(truncated)
    except ac.FastLaneImportError as exc:
        check("truncated" in str(exc), "truncated detail sequence is a hard error")
    else:
        raise AssertionError("truncated input accepted")

    missing = tmp / "missing-width.ai"
    write_fast_lane(missing, fixture, details=False)
    kwargs = dict(
        source_track_id="synthetic/layout",
        transform_source_to_vdsim=fixture["transform_source_to_vdsim"],
        closed_loop=True, generated_at="2026-09-01T00:00:00Z",
    )
    try:
        ac.import_fast_lane(missing, tmp / "missing-reject", **kwargs)
    except ac.FastLaneImportError as exc:
        check("explicit fallback" in str(exc),
              "missing widths are never silently replaced with zero")
    else:
        raise AssertionError("missing widths accepted without fallback")
    result = ac.import_fast_lane(
        missing, tmp / "missing-explicit", fallback_half_width_m=3.25, **kwargs
    )
    rows = load_rows(result.reference_path)
    check(all(float(row["left_width_m"]) == 3.25
              and float(row["right_width_m"]) == 3.25 for row in rows),
          "explicit fallback width is applied to both missing sides")
    check(len(result.warnings) == 4 and "fallback" in result.warnings[0],
          "fallback use is explicit in warnings and report")

    non_rigid = [row[:] for row in fixture["transform_source_to_vdsim"]]
    non_rigid[0][0] = 2.0
    try:
        ac.validate_transform(non_rigid)
    except ac.FastLaneImportError as exc:
        check("orthonormal" in str(exc), "scale/shear transform is rejected")
    else:
        raise AssertionError("non-rigid transform accepted")


def test_cli_and_overlay(tmp: Path, source: Path, fixture: dict):
    """Exercise public CLI and pass CSV XY into path2d without modification."""
    transform_path = tmp / "transform.json"
    transform_path.write_text(json.dumps({
        "transform_source_to_vdsim": fixture["transform_source_to_vdsim"]
    }), encoding="utf-8")
    out = tmp / "cli"
    command = [
        sys.executable, str(TOOL), str(source), "--out-dir", str(out),
        "--track-id", "synthetic/layout", "--transform-json", str(transform_path),
        "--closed-loop", "yes",
    ]
    completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    check(completed.returncode == 0 and "points=4" in completed.stdout,
          "public CLI converts the synthetic v7 sample")

    rows = load_rows(out / "reference_path.csv")
    xy = [[float(row["x_m"]), float(row["y_m"])] for row in rows]
    trace = tmp / "overlay.vdtrace"
    shutil.copy2(TRACE_FIXTURE, trace)
    attach_overlay(trace, {"kind": "path2d", "name": "waypoint", "xy": xy})
    loaded = LoadedTrace(trace)
    overlays = loaded.path2d_overlays()
    check(overlays[-1]["xy"] == xy,
          "CSV XY reaches a path2d overlay without coordinate modification")
    preview = tmp / "overlay_preview.png"
    result = render(trace, out=tmp / "overlay.gif", png=preview,
                    fps=10, stride=40, quiet=True)
    check(result["png"].stat().st_size > 20_000,
          "overlay render writes a non-empty preview PNG")


def main():
    """Run all API, CLI, deterministic and overlay contract checks."""
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    check("synthetic" in fixture["license"].lower(),
          "fixture is synthetic and contains no AC game data")
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        source, _result = test_roundtrip_and_contract(tmp, fixture)
        test_determinism(tmp, source, fixture)
        test_fail_closed_and_fallback(tmp, fixture)
        test_cli_and_overlay(tmp, source, fixture)
    print("all AC M1 importer checks passed (%d checks)" % CHECKS)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
