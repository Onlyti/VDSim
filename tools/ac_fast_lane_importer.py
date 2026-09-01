#!/usr/bin/env python3
"""Convert an Assetto Corsa v7 ``fast_lane.ai`` to the VDSim M1 bundle.

The importer deliberately requires an explicit source-to-VDSim transform.  AC
track origins and axis conventions are asset-dependent, so guessing an axis
swap would violate the map-import contract.  This module is both the supported
Python API and the command-line implementation; it has no VDSim core imports.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
import struct
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Optional, Sequence

import numpy as np
import yaml

IMPORTER_VERSION = "1.0.0"
SCHEMA_VERSION = "1.0"
HEADER = struct.Struct("<4i")
BASE_POINT = struct.Struct("<4fi")
DETAIL_POINT = struct.Struct("<18f")
INT32 = struct.Struct("<i")
GRID_META = struct.Struct("<6fif")
CSV_COLUMNS = (
    "index", "s_m", "x_m", "y_m", "z_m",
    "left_width_m", "right_width_m",
)


class FastLaneImportError(ValueError):
    """Raised when an input cannot satisfy the M1 output contract."""


@dataclass(frozen=True)
class FastLanePoint:
    """One source sample in native AC coordinates and metres."""

    position: tuple[float, float, float]
    source_s_m: float
    point_id: int
    left_width_m: Optional[float]
    right_width_m: Optional[float]


@dataclass(frozen=True)
class FastLaneSpline:
    """Validated v7 spline payload needed by M1."""

    version: int
    points: tuple[FastLanePoint, ...]
    grid_entries: int


@dataclass(frozen=True)
class ImportResult:
    """Paths and scalar evidence produced by :func:`import_fast_lane`."""

    reference_path: Path
    manifest: Path
    report: Path
    point_count: int
    closed_loop: bool
    total_length_m: float
    warnings: tuple[str, ...]


def _take(data: bytes, offset: int, layout: struct.Struct, what: str):
    """Unpack one little-endian record or raise a contextual truncation error."""
    end = offset + layout.size
    if end > len(data):
        raise FastLaneImportError(
            f"damaged fast_lane.ai: truncated {what} at byte {offset}"
        )
    return layout.unpack_from(data, offset), end


def parse_fast_lane(path: os.PathLike | str) -> FastLaneSpline:
    """Parse and validate a standard Assetto Corsa v7 AI spline.

    Supported layout is the v7 little-endian header, 20-byte base points,
    detail-count marker, 72-byte detail points and the ACTools ``HasGrid``
    tail.  A present grid is structurally consumed even though M1 does not use
    it.  Detail widths may be non-finite or absent; :func:`import_fast_lane`
    then requires an explicit fallback rather than silently writing zero.

    :param path: input ``fast_lane.ai`` path.
    :returns: immutable validated spline samples.
    :raises FastLaneImportError: for unsupported or structurally damaged data.
    """
    source = Path(path)
    try:
        data = source.read_bytes()
    except OSError as exc:
        raise FastLaneImportError(f"cannot read {source}: {exc}") from exc

    header, offset = _take(data, 0, HEADER, "header")
    version, point_count, _reserved0, _reserved1 = header
    if version != 7:
        raise FastLaneImportError(
            f"unsupported fast_lane.ai version {version}; supported: 7"
        )
    if point_count < 2 or point_count > 10_000_000:
        raise FastLaneImportError(f"invalid point count: {point_count}")

    bases = []
    previous_s = -math.inf
    for index in range(point_count):
        values, offset = _take(data, offset, BASE_POINT, f"base point {index}")
        x, y, z, source_s, point_id = values
        if not all(math.isfinite(v) for v in (x, y, z, source_s)):
            raise FastLaneImportError(f"non-finite base point {index}")
        if point_id != index:
            raise FastLaneImportError(
                f"damaged point sequence: id {point_id} at index {index}"
            )
        if source_s < previous_s:
            raise FastLaneImportError(
                f"damaged point sequence: source distance decreases at {index}"
            )
        previous_s = source_s
        bases.append((x, y, z, source_s, point_id))

    (detail_count,), offset = _take(data, offset, INT32, "detail count")
    if detail_count not in (0, point_count):
        raise FastLaneImportError(
            f"detail count {detail_count} does not match point count {point_count}"
        )

    details = []
    for index in range(detail_count):
        values, offset = _take(data, offset, DETAIL_POINT, f"detail point {index}")
        details.append(values)

    (has_grid,), offset = _take(data, offset, INT32, "HasGrid flag")
    if has_grid not in (0, 1):
        raise FastLaneImportError(f"HasGrid flag must be 0 or 1, got {has_grid}")
    grid_entries = 0
    if has_grid:
        grid_entries, offset = _consume_grid(data, offset)
    if offset != len(data):
        raise FastLaneImportError(
            f"unexpected trailing bytes after AI spline: {len(data) - offset}"
        )

    points = []
    for index, (x, y, z, source_s, point_id) in enumerate(bases):
        left = right = None
        if details:
            raw_left, raw_right = details[index][5], details[index][6]
            left = raw_left if math.isfinite(raw_left) and raw_left > 0.0 else None
            right = raw_right if math.isfinite(raw_right) and raw_right > 0.0 else None
        points.append(FastLanePoint(
            position=(x, y, z), source_s_m=source_s, point_id=point_id,
            left_width_m=left, right_width_m=right,
        ))
    return FastLaneSpline(version=version, points=tuple(points),
                          grid_entries=grid_entries)


def _checked_count(data: bytes, offset: int, what: str,
                   minimum_record_bytes: int = 4):
    """Read a non-negative variable-array count bounded by remaining bytes."""
    (count,), offset = _take(data, offset, INT32, what)
    if count < 0:
        raise FastLaneImportError(f"negative {what}: {count}")
    if count > 10_000_000:
        raise FastLaneImportError(f"overflow {what}: {count}")
    remaining = len(data) - offset
    if minimum_record_bytes and count > remaining // minimum_record_bytes:
        raise FastLaneImportError(
            f"{what} {count} exceeds remaining payload ({remaining} bytes)"
        )
    return count, offset


def _consume_grid(data: bytes, offset: int):
    """Consume one ACTools ``AiSplineGrid`` payload and return its item count."""
    values, offset = _take(data, offset, GRID_META, "grid bounds/sampling")
    max_extreme = values[:3]
    min_extreme = values[3:6]
    neighbors = values[6]
    sampling_density = values[7]
    if not all(math.isfinite(v) for v in (*max_extreme, *min_extreme,
                                           sampling_density)):
        raise FastLaneImportError("non-finite grid bounds or sampling density")
    if any(maximum < minimum
           for maximum, minimum in zip(max_extreme, min_extreme)):
        raise FastLaneImportError("grid MaxExtreme is below MinExtreme")
    if neighbors < 0:
        raise FastLaneImportError(
            f"negative grid NeighborsConsideredNumber: {neighbors}"
        )
    if sampling_density <= 0.0:
        raise FastLaneImportError(
            f"grid SamplingDensity must be positive, got {sampling_density}"
        )

    item_count, offset = _checked_count(data, offset, "grid item count")
    for item_index in range(item_count):
        sub_count, offset = _checked_count(
            data, offset, f"grid item {item_index} subitem count"
        )
        for sub_index in range(sub_count):
            value_count, offset = _checked_count(
                data, offset,
                f"grid item {item_index} subitem {sub_index} value count"
            )
            byte_count = value_count * INT32.size
            if byte_count > len(data) - offset:
                raise FastLaneImportError(
                    f"grid item {item_index} subitem {sub_index} values "
                    "exceed remaining payload"
                )
            offset += byte_count
    return item_count, offset


def validate_transform(transform: Sequence[Sequence[float]]) -> np.ndarray:
    """Validate a metre-preserving homogeneous source-to-ISO transform.

    Reflections are permitted because converting AC's handedness to ISO 8855
    can require one.  Scale and shear are rejected by the orthonormality gate.

    :param transform: 4x4 nested numeric sequence.
    :returns: validated ``float64`` NumPy array.
    :raises FastLaneImportError: if the matrix is not finite and rigid.
    """
    try:
        matrix = np.asarray(transform, dtype=float)
    except (TypeError, ValueError) as exc:
        raise FastLaneImportError(f"invalid transform: {exc}") from exc
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise FastLaneImportError("transform_source_to_vdsim must be finite 4x4")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12, rtol=0.0):
        raise FastLaneImportError("transform last row must be [0, 0, 0, 1]")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0.0):
        raise FastLaneImportError("transform rotation must be orthonormal (no scale/shear)")
    if not math.isclose(abs(float(np.linalg.det(rotation))), 1.0,
                        abs_tol=1e-9, rel_tol=0.0):
        raise FastLaneImportError("transform rotation determinant must have magnitude 1")
    return matrix


def load_transform(path: os.PathLike | str) -> np.ndarray:
    """Load a 4x4 transform from JSON, accepting a direct array or manifest key."""
    transform_path = Path(path)
    try:
        payload = json.loads(transform_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise FastLaneImportError(f"cannot load transform JSON: {exc}") from exc
    if isinstance(payload, dict):
        if "transform_source_to_vdsim" not in payload:
            raise FastLaneImportError(
                "transform JSON object lacks transform_source_to_vdsim"
            )
        payload = payload["transform_source_to_vdsim"]
    return validate_transform(payload)


def _stable_float(value: float) -> str:
    """Format SI values deterministically while normalizing negative zero."""
    if abs(value) < 0.5e-9:
        value = 0.0
    return f"{value:.9f}"


def _bbox(points: np.ndarray) -> dict:
    """Return deterministic XYZ bounding-box scalars."""
    return {
        "min": [float(v) for v in points.min(axis=0)],
        "max": [float(v) for v in points.max(axis=0)],
    }


def _atomic_text(path: Path, text: str) -> None:
    """Replace one UTF-8 output without exposing a partial file."""
    tmp = path.with_name(path.name + ".part")
    with tmp.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
    os.replace(tmp, path)


def import_fast_lane(
    source_path: os.PathLike | str,
    output_dir: os.PathLike | str,
    *,
    source_track_id: str,
    transform_source_to_vdsim: Sequence[Sequence[float]],
    closed_loop: bool,
    fallback_half_width_m: Optional[float] = None,
    generated_at: Optional[str] = None,
) -> ImportResult:
    """Convert one v7 spline into deterministic M1 CSV, manifest and report.

    :param source_path: user-owned ``fast_lane.ai``; never modified.
    :param output_dir: destination directory for the three M1 files.
    :param source_track_id: stable user-provided AC track/layout identifier.
    :param transform_source_to_vdsim: explicit 4x4 source-to-ISO transform.
    :param closed_loop: explicit loop declaration; the binary has no reliable
        loop flag, so the importer does not infer it.
    :param fallback_half_width_m: explicit positive replacement for missing
        per-side widths.  ``None`` makes missing widths a hard error.
    :param generated_at: UTC timestamp override for reproducible tests.
    :returns: output paths and comparison evidence.
    :raises FastLaneImportError: when any M1 contract cannot be met.
    """
    source = Path(source_path)
    if not source_track_id or not source_track_id.strip():
        raise FastLaneImportError("source_track_id must be non-empty")
    if fallback_half_width_m is not None:
        if not math.isfinite(fallback_half_width_m) or fallback_half_width_m <= 0.0:
            raise FastLaneImportError("fallback_half_width_m must be finite and positive")
    matrix = validate_transform(transform_source_to_vdsim)
    before_stat = source.stat()
    source_bytes = source.read_bytes()
    spline = parse_fast_lane(source)

    raw_xyz = np.asarray([point.position for point in spline.points], dtype=float)
    homogeneous = np.column_stack((raw_xyz, np.ones(len(raw_xyz))))
    output_xyz = (matrix @ homogeneous.T).T[:, :3]
    segments = np.linalg.norm(np.diff(output_xyz, axis=0), axis=1)
    if not np.isfinite(segments).all() or np.any(segments <= 1e-9):
        raise FastLaneImportError("damaged point sequence: coincident consecutive samples")
    s_m = np.concatenate(([0.0], np.cumsum(segments)))
    closing_gap_m = float(np.linalg.norm(output_xyz[-1] - output_xyz[0]))
    total_length_m = float(s_m[-1] + (closing_gap_m if closed_loop else 0.0))

    warnings = []
    rows = []
    for index, (point, xyz) in enumerate(zip(spline.points, output_xyz)):
        left, right = point.left_width_m, point.right_width_m
        missing = []
        if left is None:
            missing.append("left")
            left = fallback_half_width_m
        if right is None:
            missing.append("right")
            right = fallback_half_width_m
        if missing:
            if fallback_half_width_m is None:
                raise FastLaneImportError(
                    f"point {index} missing {'/'.join(missing)} width; "
                    "pass an explicit fallback_half_width_m"
                )
            warnings.append(
                f"point {index}: missing {'/'.join(missing)} width; used explicit "
                f"fallback {fallback_half_width_m:.9f} m"
            )
        rows.append((index, float(s_m[index]), *map(float, xyz),
                     float(left), float(right)))

    csv_buffer = io.StringIO(newline="")
    writer = csv.writer(csv_buffer, lineterminator="\n")
    writer.writerow(CSV_COLUMNS)
    for row in rows:
        writer.writerow([row[0], *(_stable_float(v) for v in row[1:])])

    if generated_at is None:
        generated_at = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace(
            "+00:00", "Z"
        )
    manifest_data = {
        "schema_version": SCHEMA_VERSION,
        "source": "assetto_corsa",
        "source_track_id": source_track_id.strip(),
        "source_hash": hashlib.sha256(source_bytes).hexdigest(),
        "coordinate_frame": {
            "standard": "ISO 8855",
            "axes": {"x": "forward", "y": "left", "z": "up"},
        },
        "units": {"length": "m", "angle": "rad"},
        "transform_source_to_vdsim": matrix.tolist(),
        "generated_at": generated_at,
        "included_layers": ["reference_path"],
        "closed_loop": bool(closed_loop),
        "source_format": {"name": "assetto_corsa_ai_spline", "version": 7},
        "importer": {"name": "vdsim-ac-fast-lane", "version": IMPORTER_VERSION},
    }
    manifest_text = yaml.safe_dump(
        manifest_data, sort_keys=False, allow_unicode=True, default_flow_style=False
    )

    source_length_m = float(spline.points[-1].source_s_m)
    report_lines = [
        "# AC fast_lane.ai M1 import report",
        "",
        "## Comparison",
        "",
        "| Metric | Source | Output |",
        "|---|---:|---:|",
        f"| Point count | {len(spline.points)} | {len(rows)} |",
        f"| Closed loop | declared: {str(bool(closed_loop)).lower()} | {str(bool(closed_loop)).lower()} |",
        f"| Ordered-path length [m] | {source_length_m:.9f} | {float(s_m[-1]):.9f} |",
        f"| Closing gap [m] | n/a | {closing_gap_m:.9f} |",
        f"| Total length [m] | n/a | {total_length_m:.9f} |",
        "",
        "## Bounding boxes [m]",
        "",
        f"- Source XYZ: `{json.dumps(_bbox(raw_xyz), sort_keys=True)}`",
        f"- VDSim XYZ: `{json.dumps(_bbox(output_xyz), sort_keys=True)}`",
        "",
        "## Warnings",
        "",
    ]
    report_lines.extend(f"- {warning}" for warning in warnings)
    if not warnings:
        report_lines.append("- none")
    report_text = "\n".join(report_lines) + "\n"

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    reference_path = output / "reference_path.csv"
    manifest = output / "manifest.yaml"
    report = output / "report.md"
    _atomic_text(reference_path, csv_buffer.getvalue())
    _atomic_text(manifest, manifest_text)
    _atomic_text(report, report_text)

    after_stat = source.stat()
    if (before_stat.st_mtime_ns, before_stat.st_size) != (
        after_stat.st_mtime_ns, after_stat.st_size
    ):
        raise FastLaneImportError("source fast_lane.ai changed during import")
    return ImportResult(
        reference_path=reference_path, manifest=manifest, report=report,
        point_count=len(rows), closed_loop=bool(closed_loop),
        total_length_m=total_length_m, warnings=tuple(warnings),
    )


def _parser() -> argparse.ArgumentParser:
    """Build the public M1 command-line parser."""
    parser = argparse.ArgumentParser(
        description="Convert AC fast_lane.ai v7 to VDSim M1 reference_path.csv"
    )
    parser.add_argument("input", type=Path, help="path to user-owned fast_lane.ai")
    parser.add_argument("--out-dir", type=Path, required=True,
                        help="destination for reference_path.csv, manifest.yaml, report.md")
    parser.add_argument("--track-id", required=True,
                        help="stable Assetto Corsa track/layout identifier")
    parser.add_argument("--transform-json", type=Path, required=True,
                        help="JSON 4x4 source-to-ISO transform (array or manifest key)")
    parser.add_argument("--closed-loop", choices=("yes", "no"), required=True,
                        help="explicit loop declaration; never inferred")
    parser.add_argument("--fallback-half-width-m", type=float,
                        help="explicit positive fallback for absent left/right widths")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    """Run the M1 CLI, returning 0 on success and 2 on a contract error."""
    args = _parser().parse_args(argv)
    try:
        result = import_fast_lane(
            args.input, args.out_dir, source_track_id=args.track_id,
            transform_source_to_vdsim=load_transform(args.transform_json),
            closed_loop=args.closed_loop == "yes",
            fallback_half_width_m=args.fallback_half_width_m,
        )
    except (FastLaneImportError, OSError) as exc:
        print(f"ac_fast_lane_importer: {exc}", file=sys.stderr)
        return 2
    print(f"wrote {result.reference_path}")
    print(f"points={result.point_count} closed_loop={str(result.closed_loop).lower()} "
          f"total_length_m={result.total_length_m:.9f}")
    for warning in result.warnings:
        print(f"warning: {warning}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
