#!/usr/bin/env python3
"""Code map for docs/modules/: module dependency graph generated from the source tree.

Each module page ``docs/modules/<name>.md`` starts with a YAML front-matter block
whose ``module:`` (M01..M19) and ``source:`` (files or directories) keys are the
single source of truth for "which file belongs to which module".  This script
scans those files for C++ ``#include "..."`` and Python imports, folds the
file-level edges into module-level edges, and prints them.

Modes
    --emit-graph   print the mermaid graph and the per-module Uses / Used by lines
    --coverage     list source files under core/, python/, cosim/ that no module claims
"""
import argparse
import ast
import re
import subprocess
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
MODULE_DIR = "docs/modules"
COVERED_ROOTS = ("core/src/", "core/include/", "python/", "cosim/")
CODE_SUFFIXES = (".cpp", ".hpp", ".h", ".py")
# External surfaces that consume modules but are not modules themselves (spec 3.5: at most 4).
EXTERNAL_GROUPS = {
    "EXT_SCRIPTS": ("examples & tools scripts", ("examples/", "tools/")),
    "EXT_APPS": ("GUI and apps", ("gui/", "apps/", "builder/")),
    "EXT_CARLA": ("CARLA plugin", ("carla_integration/",)),
}
INCLUDE_RE = re.compile(r'^\s*#\s*include\s*"([^"]+)"', re.M)


def tracked_files(repo):
    """Return the set of git-tracked paths (posix, repo-relative).

    @param repo  Repository root.
    @return      Set of tracked file paths.
    """
    out = subprocess.run(["git", "ls-files"], cwd=repo, check=True,
                         capture_output=True, text=True).stdout
    return set(out.split())


def read_front_matter(path):
    """Parse the leading ``---`` YAML block of a markdown page.

    @param path  Markdown file.
    @return      Dict of front-matter keys, or ``{}`` if the page has none.
    """
    text = path.read_text(encoding="utf-8")
    if not text.startswith("---\n"):
        return {}
    end = text.find("\n---", 4)
    if end < 0:
        return {}
    return yaml.safe_load(text[4:end]) or {}


def load_modules(repo, files):
    """Build module id -> (page name, claimed files) from the module pages.

    A ``source:`` entry ending in ``/`` claims every tracked file below it.

    @param repo   Repository root.
    @param files  Tracked file set.
    @return       (modules, owner) where modules maps id -> dict(name, files)
                  and owner maps file -> module id.
    """
    modules, owner = {}, {}
    for page in sorted((repo / MODULE_DIR).glob("*.md")):
        fm = read_front_matter(page)
        mid = fm.get("module")
        if not mid:
            continue
        claimed = set()
        for src in fm.get("source") or []:
            if src.endswith("/"):
                claimed |= {f for f in files if f.startswith(src)}
            elif src in files:
                claimed.add(src)
        modules[mid] = {"name": page.stem, "files": claimed}
        for f in claimed:
            owner.setdefault(f, mid)
    return modules, owner


def python_index(files):
    """Map importable Python module names to tracked files.

    ``vdsim`` is the compiled extension built from ``python/bindings.cpp``.

    @param files  Tracked file set.
    @return       Dict dotted-name -> file path.
    """
    idx = {"vdsim": "python/bindings.cpp"}
    for f in files:
        if not f.endswith(".py"):
            continue
        p = Path(f)
        if f.startswith("python/"):
            parts = list(p.relative_to("python").with_suffix("").parts)
        elif f.startswith(("tools/", "cosim/")):
            parts = [p.stem]
        else:
            continue
        if parts[-1] == "__init__":
            parts = parts[:-1]
        if parts:
            idx.setdefault(".".join(parts), f)
    return idx


def python_imports(path, rel):
    """Collect dotted module names imported by a Python file (static + importlib literals).

    @param path  Absolute file path.
    @param rel   Repo-relative path (for resolving relative imports).
    @return      Set of dotted names.
    """
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"))
    except (SyntaxError, UnicodeDecodeError):
        return set()
    pkg = list(Path(rel).relative_to("python").parent.parts) if rel.startswith("python/") else []
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            base = pkg[:len(pkg) - node.level + 1] if node.level else []
            mod = ".".join(base + ([node.module] if node.module else []))
            if mod:
                names.add(mod)
                names |= {mod + "." + a.name for a in node.names}
        elif (isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "import_module"
              and node.args and isinstance(node.args[0], ast.Constant)
              and isinstance(node.args[0].value, str)):
            names.add(node.args[0].value)
    return names


def resolve_python(names, idx):
    """Resolve dotted import names to tracked files (longest known prefix wins).

    @param names  Dotted names from python_imports().
    @param idx    python_index() result.
    @return       Set of file paths.
    """
    hits = set()
    for n in names:
        parts = n.split(".")
        for k in range(len(parts), 0, -1):
            key = ".".join(parts[:k])
            if key in idx:
                hits.add(idx[key])
                break
    return hits


def resolve_include(rel, inc, files):
    """Resolve a quoted C++ include against the including dir, core/include and cosim.

    @param rel    Including file (repo-relative).
    @param inc    Include string.
    @param files  Tracked file set.
    @return       Resolved file path or None.
    """
    for base in (str(Path(rel).parent), "core/include", "cosim"):
        cand = str(Path(base) / inc).replace("\\", "/")
        if cand in files:
            return cand
    return None


def file_deps(repo, rel, files, idx):
    """Files that @p rel depends on.

    @return  Set of repo-relative paths.
    """
    path = repo / rel
    if rel.endswith(".py"):
        return resolve_python(python_imports(path, rel), idx)
    if rel.endswith((".cpp", ".hpp", ".h")):
        text = path.read_text(encoding="utf-8", errors="replace")
        return {r for r in (resolve_include(rel, i, files) for i in INCLUDE_RE.findall(text)) if r}
    return set()


def external_group(rel):
    """External surface id for a file outside every module, or None."""
    for gid, (_, prefixes) in EXTERNAL_GROUPS.items():
        if rel.startswith(prefixes):
            return gid
    return None


def module_edges(repo):
    """Compute module-level dependency edges.

    @return  (modules, edges) with edges a sorted list of (user, used) ids;
             external surfaces appear only as users.
    """
    files = tracked_files(repo)
    modules, owner = load_modules(repo, files)
    idx = python_index(files)
    edges = set()
    for rel in sorted(files):
        if not rel.endswith(CODE_SUFFIXES):
            continue
        src = owner.get(rel) or external_group(rel)
        if not src:
            continue
        for dep in file_deps(repo, rel, files, idx):
            dst = owner.get(dep)
            if dst and dst != src:
                edges.add((src, dst))
    return modules, sorted(edges)


def mermaid(modules, edges):
    """Render the module graph as a mermaid flowchart (deterministic order)."""
    lines = ["```mermaid", "flowchart LR"]
    for mid in sorted(modules):
        lines.append('    %s["%s %s"]' % (mid, mid, modules[mid]["name"]))
    used_ext = sorted({s for s, _ in edges if s.startswith("EXT_")})
    for gid in used_ext:
        lines.append('    %s(["%s"])' % (gid, EXTERNAL_GROUPS[gid][0]))
    for s, d in edges:
        lines.append("    %s --> %s" % (s, d))
    lines.append("```")
    return "\n".join(lines)


def uses_lines(modules, edges):
    """Per-module ``Uses: ... / Used by: ...`` line (externals listed by label)."""
    out = {}
    for mid in sorted(modules):
        uses = sorted(d for s, d in edges if s == mid)
        users = sorted(s for s, d in edges if d == mid)
        label = [EXTERNAL_GROUPS[u][0] if u.startswith("EXT_") else u for u in users]
        out[mid] = "Uses: %s / Used by: %s" % (", ".join(uses) or "-", ", ".join(label) or "-")
    return out


def coverage(repo):
    """Tracked code files under COVERED_ROOTS that no module claims."""
    files = tracked_files(repo)
    _, owner = load_modules(repo, files)
    return sorted(f for f in files
                  if f.startswith(COVERED_ROOTS) and f.endswith(CODE_SUFFIXES) and f not in owner)


def main(argv=None):
    """CLI entry point."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--repo", type=Path, default=REPO)
    ap.add_argument("--emit-graph", action="store_true")
    ap.add_argument("--coverage", action="store_true")
    args = ap.parse_args(argv)
    if args.emit_graph:
        modules, edges = module_edges(args.repo)
        print(mermaid(modules, edges))
        print()
        for mid, line in uses_lines(modules, edges).items():
            print("%s %s" % (mid, line))
    if args.coverage:
        missing = coverage(args.repo)
        for f in missing:
            print("unclaimed:", f)
        print("unclaimed files:", len(missing))
        return 1 if missing else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
