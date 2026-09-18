#!/usr/bin/env python3
"""Import every first-party module the wheel claims to ship, from the wheel.

Run by ``CIBW_TEST_COMMAND`` inside cibuildwheel's test environment, where the
wheel is installed and the source tree is present but not on ``sys.path``.

Why this exists
---------------
Measured 2026-09-18 (pre-flight C-2): the wheel shipped ``vdsim_render.py`` but
not ``vdsim_preset.py``, which it imports, so ``import vdsim_render`` raised
``ModuleNotFoundError`` for every pip user. The whole wheel matrix was green:
the old test command imported ``vdsim`` and built ``Sim`` twice, and
``tools/verify_wheel_packaging.py`` compares zip members against a hand-written
list that never named either module.

The module list is read from ``python/CMakeLists.txt`` — the file that decides
what ships — so the workflow never restates names that can drift.

Two properties are checked per module:

1. it imports at all;
2. it resolves to the installed distribution, not to the source tree. Without
   (2) this check would pass on a broken wheel whenever the working directory
   happened to be the repository.
"""
from __future__ import annotations

import importlib
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
MANIFEST = REPO / "python" / "CMakeLists.txt"


def shipped_modules():
    """Top-level importable names listed under ``if(DEFINED SKBUILD)``.

    ``install(FILES ...)`` entries ending in ``.py`` become module names;
    ``install(DIRECTORY .../python/<name> ...)`` entries become package names.
    Config directories (``configs/...``) are data, not importable, and are
    skipped.
    """
    text = MANIFEST.read_text()
    if "if(DEFINED SKBUILD)" not in text:
        raise SystemExit("FAIL: no SKBUILD install block in %s" % MANIFEST)
    body = text.split("if(DEFINED SKBUILD)", 1)[1].split("endif()", 1)[0]
    names = ["vdsim"]  # install(TARGETS vdsim_py ...) -> the extension module
    for rel in re.findall(r"\$\{CMAKE_SOURCE_DIR\}/([^\s)]+)", body):
        parts = rel.split("/")
        if rel.endswith(".py"):
            names.append(parts[-1][:-3])
        elif parts[0] == "python":
            names.append(parts[-1])
    out = []
    for n in names:  # stable order, no duplicates
        if n not in out:
            out.append(n)
    return out


def strip_source_tree_from_path():
    """Drop anything that could satisfy an import from the checkout.

    Returns the removed entries so the report can show what was excluded.
    """
    removed = []
    kept = []
    for entry in sys.path:
        try:
            resolved = Path(entry or ".").resolve()
        except OSError:
            kept.append(entry)
            continue
        if resolved == REPO or REPO in resolved.parents:
            removed.append(str(resolved))
        else:
            kept.append(entry)
    sys.path[:] = kept
    return removed


def main():
    removed = strip_source_tree_from_path()
    modules = shipped_modules()
    print("manifest: %s" % MANIFEST)
    print("removed from sys.path (source tree): %s" % (removed or "none"))
    print("modules to import (%d): %s" % (len(modules), ", ".join(modules)))

    failures = []
    for name in modules:
        try:
            mod = importlib.import_module(name)
        except Exception as exc:                       # noqa: BLE001 - report any
            failures.append("%s: %s: %s" % (name, type(exc).__name__, exc))
            print("FAIL %-16s %s: %s" % (name, type(exc).__name__, exc))
            continue
        origin = getattr(mod, "__file__", None)
        if origin is None:
            failures.append("%s: imported with no __file__" % name)
            print("FAIL %-16s no __file__" % name)
            continue
        origin_path = Path(origin).resolve()
        if origin_path == REPO or REPO in origin_path.parents:
            failures.append("%s: resolved to the source tree (%s)" % (name, origin_path))
            print("FAIL %-16s from source tree %s" % (name, origin_path))
            continue
        print("ok   %-16s %s" % (name, origin_path))

    if failures:
        print("\n%d module(s) failed:" % len(failures))
        for f in failures:
            print("  - %s" % f)
        print("The wheel does not ship a working copy of every module listed in "
              "python/CMakeLists.txt. Add the missing file to that install list.")
        return 1
    print("\nall %d shipped modules import from the installed wheel" % len(modules))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
