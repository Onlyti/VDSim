#!/usr/bin/env python3
"""Verify wheel/sdist data packaging before PyPI publish (PO gate)."""
from __future__ import annotations

import sys
import zipfile
from pathlib import Path


def _check_member(names: list[str]) -> tuple[int, int, list[str]]:
  yaml = [n for n in names if n.endswith(".yaml") and "vdsim_configs/" in n]
  tir = [n for n in names if n.endswith(".tir")]
  public = [n for n in tir if n.endswith("ioniq5_pac2002.tir")]
  measured = [n for n in tir if not n.endswith("ioniq5_pac2002.tir")]
  return len(yaml), len(public), measured


def verify_wheel(path: Path) -> None:
  with zipfile.ZipFile(path) as zf:
    names = zf.namelist()
  ny, n_pub, bad = _check_member(names)
  if ny < 5:
    raise SystemExit(f"FAIL {path.name}: too few yaml presets ({ny})")
  if n_pub != 1:
    raise SystemExit(f"FAIL {path.name}: expected 1 public .tir, got {n_pub}")
  if bad:
    raise SystemExit(f"FAIL {path.name}: confidential .tir in wheel: {bad}")
  need = "vdsim_configs/parts/tire/ioniq5_pac2002.tir"
  if not any(n.endswith(need) for n in names):
    raise SystemExit(f"FAIL {path.name}: missing {need}")
  if not any("vdsim_configs/vehicles/ioniq5_awd.yaml" in n for n in names):
    raise SystemExit(f"FAIL {path.name}: missing ioniq5_awd.yaml preset")
  print(f"OK {path.name}: yaml={ny} public_tir=1 measured_tir=0")


def main(argv: list[str]) -> int:
  if len(argv) < 2:
    print("usage: verify_wheel_packaging.py <wheel> [wheel ...]", file=sys.stderr)
    return 2
  for p in argv[1:]:
    verify_wheel(Path(p))
  return 0


if __name__ == "__main__":
  raise SystemExit(main(sys.argv))
