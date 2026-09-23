#!/usr/bin/env python3
"""Code map for docs/modules/: module dependency graph generated from the source tree.

Each module page ``docs/modules/<name>.md`` starts with a YAML front-matter block
whose ``module:`` (M01..M19) and ``source:`` (files or directories) keys are the
single source of truth for "which file belongs to which module".  This script
scans those files for C++ ``#include "..."`` and Python imports, folds the
file-level edges into module-level edges, and prints them.

Modes
    --emit-graph   print the mermaid graph, reverse-edge table, Uses / Used by lines,
                   external-surface table, module cycles with their edges, file-level
                   cycles and rule-B1 violations (exit 1 on any file-level cycle)
    --write-edges  write the canonical edge list docs/modules/_graph_edges.txt
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
# Rule B2: M01+M02 are drawn as one foundation box.
FOUNDATION = ("M01", "M02")
FOUNDATION_ID = "BASE"
# Rule B1: Python files importing the compiled vdsim extension stay out of M01..M13.
CORE_LAYER = tuple("M%02d" % i for i in range(1, 14))
EDGES_FILE = MODULE_DIR + "/_graph_edges.txt"
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


def file_graph(repo):
    """File-level dependency graph over every code file owned by a module or an external surface.

    @param repo  Repository root.
    @return      (modules, owner, graph) with graph mapping file -> set of files it depends on.
    """
    files = tracked_files(repo)
    modules, owner = load_modules(repo, files)
    idx = python_index(files)
    graph = {}
    for rel in sorted(files):
        if rel.endswith(CODE_SUFFIXES) and (rel in owner or external_group(rel)):
            graph[rel] = file_deps(repo, rel, files, idx)
    return modules, owner, graph


def edge_evidence(repo):
    """File-level evidence behind every module edge.

    @param repo  Repository root.
    @return      (modules, evidence) with evidence a sorted list of
                 (user id, used id, user file, used file); external surfaces
                 appear only as users.
    """
    modules, owner, graph = file_graph(repo)
    rows = set()
    for rel, deps in graph.items():
        src = owner.get(rel) or external_group(rel)
        for dep in deps:
            dst = owner.get(dep)
            if dst and dst != src:
                rows.add((src, dst, rel, dep))
    return modules, sorted(rows)


def module_edges(repo):
    """Compute module-level dependency edges.

    @return  (modules, edges) with edges a sorted list of (user, used) ids;
             external surfaces appear only as users.
    """
    modules, rows = edge_evidence(repo)
    return modules, sorted({(s, d) for s, d, _, _ in rows})


def edges_text(rows):
    """Canonical edge list (``docs/modules/_graph_edges.txt``): one ``Mxx -> Myy <file>`` per user file.

    @param rows  edge_evidence() rows.
    @return      Text with a trailing newline.
    """
    lines = sorted({"%s -> %s %s" % (s, d, f) for s, d, f, _ in rows})
    return "\n".join(lines) + "\n"


def mermaid(modules, edges):
    """Render the module graph as a mermaid flowchart (deterministic order).

    M01 and M02 are drawn as one foundation box (rule B2): edges into it are
    left out of the figure, edges out of it go to foundation_table().
    External surfaces are left out of the figure and listed by external_table().
    """
    lines = ["```mermaid", "flowchart LR"]
    lines.append('    %s["%s"]' % (FOUNDATION_ID, "<br/>".join(
        "%s %s" % (m, modules[m]["name"]) for m in FOUNDATION if m in modules)))
    for mid in sorted(modules):
        if mid not in FOUNDATION:
            lines.append('    %s["%s %s"]' % (mid, mid, modules[mid]["name"]))
    for s, d in edges:
        if s not in FOUNDATION and d not in FOUNDATION and not s.startswith("EXT_"):
            lines.append("    %s --> %s" % (s, d))
    lines.append("```")
    return "\n".join(lines)


def foundation_note(edges):
    """One-line sentence standing in for the omitted edges into the foundation box."""
    users = sorted({s for s, d in edges if d in FOUNDATION and s not in FOUNDATION})
    label = [EXTERNAL_GROUPS[u][0] if u.startswith("EXT_") else u for u in users]
    return "Edges into %s are not drawn: %d modules/surfaces use it (%s)." % (
        "+".join(FOUNDATION), len(users), ", ".join(label))


def foundation_table(modules, rows):
    """Markdown table of edges leaving the foundation layer (reverse dependencies).

    @param rows  edge_evidence() rows.
    """
    out = ["| from | to | file | depends on |", "|---|---|---|---|"]
    for s, d, f, dep in rows:
        if s in FOUNDATION and d not in FOUNDATION:
            out.append("| %s | %s %s | %s | %s |" % (s, d, modules[d]["name"], f, dep))
    return "\n".join(out)


def external_table(rows):
    """Markdown table of edges from external surfaces into modules (README section 3).

    @param rows  edge_evidence() rows.
    """
    files = {}
    for s, d, f, _ in rows:
        if s.startswith("EXT_"):
            files.setdefault((s, d), set()).add(f)
    out = ["| surface | uses | files |", "|---|---|---|"]
    for (s, d), fs in sorted(files.items()):
        out.append("| %s | %s | %d |" % (EXTERNAL_GROUPS[s][0], d, len(fs)))
    return "\n".join(out)


def cycles(edges):
    """Strongly connected components of size > 1 among modules (externals excluded).

    @return  Sorted list of sorted module-id lists.
    """
    graph = {}
    for s, d in edges:
        if not s.startswith("EXT_"):
            graph.setdefault(s, set()).add(d)
            graph.setdefault(d, set())
    return scc(graph)


def cycle_table(modules, rows, comps):
    """Markdown table of the edges that make up each module cycle, with their files (rule B3).

    @param rows   edge_evidence() rows.
    @param comps  cycles() result.
    """
    out = ["| cycle | from | to | file | depends on |", "|---|---|---|---|---|"]
    for comp in comps:
        tag = " ".join(comp)
        for s, d, f, dep in rows:
            if s in comp and d in comp:
                out.append("| %s | %s | %s %s | %s | %s |" % (tag, s, d, modules[d]["name"], f, dep))
    return "\n".join(out)


def file_cycles(graph):
    """File-level dependency cycles (code defects under rule B3; D7 fails if any exist).

    A file depending on itself counts as a cycle of one.

    @param graph  file_graph() graph.
    @return       Sorted list of sorted file lists.
    """
    g = {f: {d for d in deps if d in graph} for f, deps in graph.items()}
    selfloops = [[f] for f, deps in g.items() if f in deps]
    return sorted(scc(g) + selfloops)


def scc(graph):
    """Strongly connected components of size > 1 (Tarjan).

    @param graph  Dict node -> set of successor nodes; every successor must be a key.
    @return       Sorted list of sorted node lists.
    """
    index, low, stack, on, comps = {}, {}, [], set(), []

    def visit(v):
        index[v] = low[v] = len(index)
        stack.append(v)
        on.add(v)
        for w in sorted(graph[v]):
            if w not in index:
                visit(w)
                low[v] = min(low[v], low[w])
            elif w in on:
                low[v] = min(low[v], index[w])
        if low[v] == index[v]:
            comp = []
            while True:
                w = stack.pop()
                on.discard(w)
                comp.append(w)
                if w == v:
                    break
            if len(comp) > 1:
                comps.append(sorted(comp))

    for v in sorted(graph):
        if v not in index:
            visit(v)
    return sorted(comps)


def b1_violations(repo):
    """Python files claimed by M01..M13 that import the compiled ``vdsim`` extension (rule B1).

    @return  Sorted list of (module id, file).
    """
    files = tracked_files(repo)
    _, owner = load_modules(repo, files)
    idx = python_index(files)
    return sorted((m, f) for f, m in owner.items()
                  if f.endswith(".py") and m in CORE_LAYER
                  and idx["vdsim"] in file_deps(repo, f, files, idx))


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
    ap.add_argument("--write-edges", action="store_true")
    ap.add_argument("--coverage", action="store_true")
    args = ap.parse_args(argv)
    rc = 0
    if args.emit_graph or args.write_edges:
        modules, rows = edge_evidence(args.repo)
        edges = sorted({(s, d) for s, d, _, _ in rows})
    if args.write_edges:
        (args.repo / EDGES_FILE).write_text(edges_text(rows), encoding="utf-8")
    if args.emit_graph:
        print(mermaid(modules, edges))
        print()
        print(foundation_note(edges))
        print()
        print(foundation_table(modules, rows))
        print()
        for mid, line in uses_lines(modules, edges).items():
            print("%s %s" % (mid, line))
        print()
        print(external_table(rows))
        print()
        comps = cycles(edges)
        for comp in comps:
            print("cycle:", " ".join(comp))
        print()
        print(cycle_table(modules, rows, comps))
        print()
        fcyc = file_cycles(file_graph(args.repo)[2])
        for comp in fcyc:
            print("file cycle:", " ".join(comp))
        print("file-level cycles:", len(fcyc))
        rc = 1 if fcyc else rc
        bad = b1_violations(args.repo)
        for mid, f in bad:
            print("B1 violation: %s %s" % (mid, f))
        print("B1 violations:", len(bad))
        rc = 1 if bad else rc
    if args.coverage:
        missing = coverage(args.repo)
        for f in missing:
            print("unclaimed:", f)
        print("unclaimed files:", len(missing))
        rc = 1 if missing else rc
    return rc


if __name__ == "__main__":
    sys.exit(main())
