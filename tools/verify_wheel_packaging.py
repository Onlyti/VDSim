#!/usr/bin/env python3
"""Verify wheel/sdist data packaging before PyPI publish (PO gate).

A published version can never be replaced, so this refuses an archive that
carries vehicle data or run outputs that must not leave the repository:

* a member whose path names a withheld source (``FORBIDDEN_NAME_PARTS``),
* any ``.tir`` (tyre coefficient files are measured data; none ships),
* a run-output directory (``ARTIFACT_DIRS``) anywhere in the path,

and refuses a wheel that lacks the public generic presets or the Python
modules the entry points import.  Usage: ``verify_wheel_packaging.py <file> ...``
with ``.whl`` and/or ``.tar.gz`` files.
"""
from __future__ import annotations

import sys
import tarfile
import zipfile
from pathlib import Path

#: Path fragments (lower case) no published member may contain.
FORBIDDEN_NAME_PARTS = ("ioniq", "carmaker", "private")
#: Directory names of build products and run outputs.
ARTIFACT_DIRS = ("build", "dist", "runs", "results", "logs")
#: Wheel members the default plant and the RL default car need.
GENERIC_PRESETS = ("vdsim_configs/vehicles/generic_sedan.yaml",
                   "vdsim_configs/parts/tire/generic_pacejka.yaml")
#: Wheel members the entry points and documented imports need.
REQUIRED_MODULES = ("vdsim_lab.py", "vdsim_plant.py", "catalog/__init__.py",
                    "quickstart.py")


def archive_members(path: Path) -> list[str]:
  """Member paths of a wheel (zip) or an sdist (tar.gz), ``/``-separated.

  :param path: the archive.
  :returns: every member name (directories included for tar).
  """
  if path.suffix == ".whl":
    with zipfile.ZipFile(path) as zf:
      return zf.namelist()
  with tarfile.open(path) as tf:
    return tf.getnames()


def forbidden_members(names: list[str]) -> list[str]:
  """Members that may not be published (rules in the module docstring).

  :param names: archive member paths.
  :returns: the offending paths, in archive order.
  """
  bad = []
  for n in names:
    low = n.lower()
    parts = low.split("/")
    if (any(f in low for f in FORBIDDEN_NAME_PARTS) or low.endswith(".tir")
        or any(p in ARTIFACT_DIRS for p in parts[:-1])):
      bad.append(n)
  return bad


def verify_archive(path: Path) -> None:
  """Raise ``SystemExit("FAIL ...")`` unless ``path`` passes the gate.

  :param path: a ``.whl`` or ``.tar.gz``; required members are checked for
    wheels only (an sdist is source, the build places them).
  """
  names = archive_members(path)
  bad = forbidden_members(names)
  if bad:
    raise SystemExit(f"FAIL {path.name}: {len(bad)} member(s) may not be published: "
                     f"{bad[:10]}")
  if path.suffix != ".whl":
    print(f"OK {path.name}: members={len(names)} forbidden=0")
    return
  yaml = [n for n in names if n.endswith(".yaml") and "vdsim_configs/" in n]
  if len(yaml) < 5:
    raise SystemExit(f"FAIL {path.name}: too few yaml presets ({len(yaml)})")
  for need in GENERIC_PRESETS:
    if not any(n.endswith(need) for n in names):
      raise SystemExit(f"FAIL {path.name}: missing generic preset {need}")
  for need in REQUIRED_MODULES:
    if not any(n == need or n.endswith("/" + need) for n in names):
      raise SystemExit(f"FAIL {path.name}: missing {need}")
  print(f"OK {path.name}: yaml={len(yaml)} forbidden=0 generic presets=yes "
        f"modules=yes")


def main(argv: list[str]) -> int:
  if len(argv) < 2:
    print("usage: verify_wheel_packaging.py <wheel|sdist> [...]", file=sys.stderr)
    return 2
  for p in argv[1:]:
    verify_archive(Path(p))
  return 0


if __name__ == "__main__":
  raise SystemExit(main(sys.argv))
