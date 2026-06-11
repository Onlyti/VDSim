#!/usr/bin/env python3
"""Module workshop: build, contract-check, and register a user C++ subsystem module.

The user writes a `.cpp` (from templates/modules/*) in their own editor. This tool then:
  1. build  — compile <folder>/<name>.cpp into a .so against libvdsim_core,
  2. check  — load it and probe that its I/O conforms to the interface contract,
  3. register — on pass, save it as a `module_plugin_v1` catalog part (kind = category).

Exposes plain functions (used by the GUI endpoints) plus a CLI. All functions return a
dict with at least {"status": "pass"|"fail", ...} and never raise on a user error — the
failure cause is in "cause".

CLI:
  python3 tools/module_workshop.py build_check --kind brake --folder DIR --name my_brake
  python3 tools/module_workshop.py register   --kind brake --name my_brake --so PATH [--axle 0]
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))

KINDS = ("brake", "steering", "drivetrain", "suspension", "antirollbar")


def _discover(repo: Path) -> dict:
    """Locate the include/lib/checker the build produced. Returns paths or an error."""
    core_inc = repo / "core" / "include"
    lib_dir = repo / "build" / "lib"
    checker = repo / "build" / "bin" / "vdsim_module_check"
    eigen = sorted((repo / "build" / "_deps").glob("eigen-src"))
    missing = []
    if not (lib_dir / "libvdsim_core.a").is_file():
        missing.append("build/lib/libvdsim_core.a (build the project first)")
    if not checker.is_file():
        missing.append("build/bin/vdsim_module_check (build the project first)")
    if not eigen:
        missing.append("Eigen headers under build/_deps/eigen-src")
    return {
        "core_inc": core_inc,
        "lib_dir": lib_dir,
        "checker": checker,
        "eigen": eigen[0] if eigen else None,
        "missing": missing,
    }


def build_module(cpp_path: Path, out_so: Path, repo: Path = REPO) -> dict:
    """Compile a single module .cpp into a shared object. {status, so, log, cause}."""
    cpp_path = Path(cpp_path)
    if not cpp_path.is_file():
        return {"status": "fail", "cause": f"source not found: {cpp_path}"}
    env = _discover(repo)
    if env["missing"]:
        return {"status": "fail", "cause": "missing build prerequisites: " + "; ".join(env["missing"])}
    out_so = Path(out_so)
    out_so.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "g++", "-std=c++17", "-shared", "-fPIC", "-O2",
        "-I", str(env["core_inc"]), "-I", str(env["eigen"]),
        str(cpp_path),
        "-L", str(env["lib_dir"]), "-lvdsim_core",
        "-o", str(out_so),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    log = (proc.stdout + proc.stderr).strip()
    if proc.returncode != 0:
        # First lines of the compiler diagnostics are the actionable cause.
        cause = "\n".join(log.splitlines()[:40]) or "compilation failed"
        return {"status": "fail", "so": None, "log": log, "cause": cause}
    return {"status": "pass", "so": str(out_so), "log": log}


def check_module(so_path: Path, kind: str, repo: Path = REPO) -> dict:
    """Contract-check a built .so against the requested kind. {status, kind, name, cause}."""
    env = _discover(repo)
    if not env["checker"].is_file():
        return {"status": "fail", "cause": "vdsim_module_check missing (build the project)"}
    proc = subprocess.run([str(env["checker"]), str(so_path), str(kind)],
                          capture_output=True, text=True)
    out = proc.stdout.strip()
    try:
        result = json.loads(out)
    except json.JSONDecodeError:
        # No JSON -> the checker crashed (e.g. the module segfaulted during the probe).
        return {"status": "fail", "kind": kind,
                "cause": f"checker crashed (rc={proc.returncode}): {(proc.stderr or out)[:300]}"}
    return result


def build_and_check(kind: str, folder: str, name: str, repo: Path = REPO) -> dict:
    """Auto-detect <folder>/<name>.cpp, build it, and contract-check it.

    Returns {status, kind, name, cpp, so, log, cause}. This is the GUI 'Build & Check' call.
    """
    if kind not in KINDS:
        return {"status": "fail", "cause": f"unknown kind '{kind}' (expected {list(KINDS)})"}
    cpp = Path(folder).expanduser() / f"{name}.cpp"
    if not cpp.is_file():
        return {"status": "fail", "kind": kind, "name": name,
                "cause": f"file not found: {cpp} — write {name}.cpp from the {kind} template"}
    out_so = repo / "build" / "workshop" / kind / f"{name}.so"
    b = build_module(cpp, out_so, repo)
    if b["status"] != "pass":
        return {"status": "fail", "kind": kind, "name": name, "cpp": str(cpp),
                "log": b.get("log", ""), "cause": "build failed:\n" + b["cause"]}
    c = check_module(out_so, kind, repo)
    c.setdefault("kind", kind)
    c["name"] = c.get("name") or name
    c["cpp"] = str(cpp)
    c["so"] = str(out_so)
    c["log"] = b.get("log", "")
    return c


def register(kind: str, name: str, so: str, repo: Path = REPO, axle: int = 0) -> dict:
    """Register a passed module .so as a catalog part. {status, part_id, ...}."""
    from catalog.part_store import save_module_plugin_part
    from catalog import CatalogError
    label = f"{kind} module: {name}"
    try:
        info = save_module_plugin_part(repo, kind, name, label, so, axle=axle)
    except CatalogError as e:
        return {"status": "fail", "cause": str(e)}
    info["status"] = "pass"
    return info


def _main() -> int:
    ap = argparse.ArgumentParser(description="VDSim C++ subsystem-module workshop")
    sub = ap.add_subparsers(dest="cmd", required=True)
    bc = sub.add_parser("build_check", help="build <folder>/<name>.cpp and contract-check it")
    bc.add_argument("--kind", required=True, choices=KINDS)
    bc.add_argument("--folder", required=True)
    bc.add_argument("--name", required=True)
    rg = sub.add_parser("register", help="register a passed .so as a catalog part")
    rg.add_argument("--kind", required=True, choices=KINDS)
    rg.add_argument("--name", required=True)
    rg.add_argument("--so", required=True)
    rg.add_argument("--axle", type=int, default=0)
    args = ap.parse_args()

    if args.cmd == "build_check":
        res = build_and_check(args.kind, args.folder, args.name)
    else:
        res = register(args.kind, args.name, args.so, axle=args.axle)
    print(json.dumps(res, indent=2))
    return 0 if res.get("status") == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(_main())
