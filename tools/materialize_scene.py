#!/usr/bin/env python3
"""Materialize a catalog scene (fleet[] + blueprint) to cosim world YAML (vehicles[])."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))

from catalog.materialize import materialize_scene_file  # noqa: E402


def main():
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <scene.yaml> <out_world.yaml>", file=sys.stderr)
        return 2
    scene = Path(sys.argv[1])
    out = Path(sys.argv[2])
    materialize_scene_file(scene, out)
    print(f"materialize_scene: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
