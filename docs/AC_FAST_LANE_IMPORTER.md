# Assetto Corsa fast_lane.ai M1 importer

The M1 importer converts a user-owned Assetto Corsa v7 AI spline into a
VDSim reference-path bundle. It does not parse KN5, modify the simulation core,
or redistribute Assetto Corsa data.

## Coordinate contract

- Output frame: ISO 8855, X forward, Y left, Z up.
- Length: metres. Angle metadata: radians.
- The importer never guesses the source axes or origin.
- `--transform-json` is mandatory and must contain a finite, orthonormal 4x4
  homogeneous transform. Handedness-changing orthonormal transforms are valid.
- `--closed-loop` is mandatory because the supported binary payload has no
  reliable loop flag.
- Missing widths are errors unless the user explicitly passes
  `--fallback-half-width-m`; every replacement is recorded as a warning.

## CLI

```bash
python3 tools/ac_fast_lane_importer.py /owned-track/ai/fast_lane.ai \
  --out-dir /tmp/track-m1 \
  --track-id track/layout \
  --transform-json /owned-track/source_to_vdsim.json \
  --closed-loop yes
```

The JSON file is either a 4x4 array or an object with the
`transform_source_to_vdsim` key. The command writes:

- `reference_path.csv`: ordered `index,s_m,x_m,y_m,z_m,left_width_m,right_width_m`.
- `manifest.yaml`: source hash, units, frame, exact transform, loop state and
  importer version.
- `report.md`: point-count/length comparison, source/output bounding boxes and
  explicit warnings.

## Python API

```python
from tools.ac_fast_lane_importer import import_fast_lane

result = import_fast_lane(
    "fast_lane.ai",
    "track-m1",
    source_track_id="track/layout",
    transform_source_to_vdsim=matrix_4x4,
    closed_loop=True,
)
```

`reference_path.csv` can be read directly and its XY columns attached as a
trace overlay object with `kind: path2d`; the importer performs no resampling.

## Supported input and legal boundary

- Supported: standard little-endian v7 AI spline with 20-byte base points and
  72-byte detail points.
- Unsupported versions and damaged point/detail sequences fail immediately.
- Run locally against the user's own Assetto Corsa installation.
- Do not commit or redistribute original AI files or converted track bundles.
- The repository fixture is fully synthetic and CC0-1.0.
