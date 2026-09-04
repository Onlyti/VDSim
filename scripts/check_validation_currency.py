#!/usr/bin/env python3
"""Gate that keeps ``docs/VALIDATION.md`` from going stale.

The document records one number -- the ctest result of the *canonical*
configuration (the ``validation`` CMake preset).  Humans forget to refresh it,
so this script parses the machine-readable currency block out of the document
and compares every element against what was actually measured in CI.  Any
mismatch is a hard failure, not a warning.

Contract (VDSimMarketing ``07_p0_demo.md`` 6.3 / 6.5)::

    <!-- VALIDATION-CURRENCY BEGIN -->
    tests:    407/407
    config:   cmake --preset validation && ...
    presets:  CMakePresets.json@<blob-sha>
    toolchain: cmake 3.31.10
    commit:   <commit-sha>
    date:     2026-09-04
    excluded: gui_v3_e2e (reason -- link)
    flaky:    some_target (2/20 runs, unknown)
    failing:  other_target (reason)
    <!-- VALIDATION-CURRENCY END -->
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

BEGIN_MARKER = "<!-- VALIDATION-CURRENCY BEGIN -->"
END_MARKER = "<!-- VALIDATION-CURRENCY END -->"

#: Keys that may appear more than once inside the currency block.
MULTI_KEYS = ("excluded", "flaky", "failing")

#: The elements every recorded number must carry: the four of
#: 07_p0_demo.md 6.3 plus the pinned toolchain (18_dev_briefing_0903.md 10,
#: Q1a).  A count measured with a different CMake is a different measurement.
REQUIRED_KEYS = ("tests", "config", "commit", "date", "toolchain")

_KV_RE = re.compile(r"^\s*([a-z_]+)\s*:\s*(.+?)\s*$")
_TESTS_RE = re.compile(r"^(\d+)\s*/\s*(\d+)$")
_DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
_SUMMARY_RE = re.compile(
    r"(\d+)%\s+tests\s+passed,\s+(\d+)\s+tests\s+failed\s+out\s+of\s+(\d+)"
)
#: gtest value-parameterised names contain spaces ("... <00-00 00-00>"),
#: so the name runs to end of line rather than to the first blank.
_REGISTERED_RE = re.compile(r"^\s*Test\s*#\d+:\s*(.+?)\s*$")
_TOTAL_RE = re.compile(r"^\s*Total Tests:\s*(\d+)\s*$")
_CMAKE_VERSION_RE = re.compile(r"cmake\s+version\s+(\S+)", re.IGNORECASE)
_DOC_TOOLCHAIN_RE = re.compile(r"cmake\s+(\S+)", re.IGNORECASE)
_FENCE = chr(96) * 3

#: Historical measurements remain evidence rather than mutable status claims.
HISTORICAL_DOC_PREFIXES = ("tasks/", "evidence/")
HISTORICAL_DOC_PATHS = frozenset((
    "design/CURSOR_USAGE.md",
    "design/L5_6DOF_MULTIBODY.md",
    "design/LD4_MULTIBODY.md",
    "design/LOW_SPEED_HANDLING.md",
    "design/TIRE_INTERFACE_INVERSION.md",
    "design/TIRE_ROADMAP.md",
    "design/V0.2_PLAN.md",
    "design/V0.2_SUBSYSTEMS.md",
    "design/V0.4_PLAN.md",
    "design/V0.4_SLOPE_JUMP_DYNAMICS.md",
    "design/V0.5_TERRAIN_L5.md",
))
HISTORICAL_DOC_NAMES = frozenset((
    "CLEANUP_PLAN.md",
    "IMPROVEMENT_REPORT.md",
    "PLANT_DELIVERY_NOTE.md",
    "SUMMARY.md",
    "TIRE_VALIDATION.md",
))
_HARDCODED_TEST_COUNT_RE = re.compile(
    r"(?:\b\d+\s*/\s*\d+\b.{0,24}\bctests?\b|"
    r"\bctests?\b.{0,24}\b\d+\s*/\s*\d+\b|"
    r"\b\d{2,}\b\*{0,2}\s+(?:automated\s+)?ctests?\b|"
    r"\bctests?\b(?:\s+(?:suite|green|count|total))?\s*[:=]?\s*\*{0,2}\d{2,}\b)", re.IGNORECASE
)


def _is_historical_doc(relative):
    """Return whether @p relative is an immutable report or evidence page."""
    posix = Path(relative).as_posix()
    name = Path(posix).name
    return (posix.startswith(HISTORICAL_DOC_PREFIXES)
            or posix in HISTORICAL_DOC_PATHS
            or name in HISTORICAL_DOC_NAMES
            or name.startswith("HANDOFF")
            or name.startswith("STATUS_"))


def parse_currency_block(text):
    """Extract the currency block from a VALIDATION.md body.

    @param text  Full markdown source of ``docs/VALIDATION.md``.
    @return      Mapping of key to value; ``MULTI_KEYS`` map to a list of
                 strings, every other key to a single string.
    @throws ValueError  If the begin/end markers are missing or out of order.
    """
    start = text.find(BEGIN_MARKER)
    end = text.find(END_MARKER)
    if start < 0 or end < 0 or end < start:
        raise ValueError(
            "validation currency block not found; expected %r ... %r"
            % (BEGIN_MARKER, END_MARKER)
        )
    body = text[start + len(BEGIN_MARKER):end]

    parsed = {key: [] for key in MULTI_KEYS}
    for raw_line in body.splitlines():
        line = raw_line.strip()
        if not line or line.startswith(_FENCE) or line.startswith("<!--"):
            continue
        match = _KV_RE.match(line)
        if not match:
            continue
        key, value = match.group(1), match.group(2)
        if key in MULTI_KEYS:
            parsed[key].append(value)
        else:
            parsed[key] = value
    return parsed


def parse_tests_field(value):
    """Split a ``passed/total`` field into integers.

    @param value  The right-hand side of the ``tests:`` line.
    @return       ``(passed, total)``.
    @throws ValueError  If the field is not ``<int>/<int>``.
    """
    match = _TESTS_RE.match(value.strip())
    if not match:
        raise ValueError("tests field must be '<passed>/<total>', got %r" % value)
    return int(match.group(1)), int(match.group(2))


def parse_registered_tests(ctest_list_output):
    """Read the test names out of ``ctest -N`` output.

    @param ctest_list_output  Raw stdout of ``ctest -N``.
    @return                   Test names in registration order.
    @throws ValueError  If ctest reported tests but none could be parsed.
    """
    names = []
    for line in ctest_list_output.splitlines():
        match = _REGISTERED_RE.match(line)
        if match:
            names.append(match.group(1))
    if not names:
        total = _TOTAL_RE.search(ctest_list_output)
        if total and int(total.group(1)) > 0:
            raise ValueError("ctest -N reported tests but no names could be parsed")
    return names


def parse_run_summary(ctest_run_output):
    """Read ``(passed, total)`` out of a ``ctest`` run log.

    @param ctest_run_output  Raw stdout of a ``ctest`` run.
    @return                  ``(passed, total)`` counted by ctest itself.
    @throws ValueError       If the summary line is absent.
    """
    match = _SUMMARY_RE.search(ctest_run_output)
    if not match:
        raise ValueError("no ctest summary line ('N% tests passed, ...') found")
    failed, total = int(match.group(2)), int(match.group(3))
    return total - failed, total


def parse_cmake_version(cmake_version_output):
    """Read the CMake version out of ``cmake --version`` output.

    @param cmake_version_output  Raw stdout of ``cmake --version``.
    @return                      Version string, e.g. ``"3.31.10"``.
    @throws ValueError           If no version line is present.
    """
    match = _CMAKE_VERSION_RE.search(cmake_version_output)
    if not match:
        raise ValueError(
            "no 'cmake version <x.y.z>' line found in the toolchain record"
        )
    return match.group(1)


def parse_doc_toolchain(value):
    """Read the CMake version out of a ``toolchain:`` document line.

    @param value  Right-hand side of the ``toolchain:`` line.
    @return       Version string, or ``""`` if the line does not name cmake.
    """
    match = _DOC_TOOLCHAIN_RE.search(value or "")
    return match.group(1) if match else ""


def excluded_targets(entries):
    """Strip the reason text from ``excluded:`` lines.

    @param entries  Raw ``excluded:`` values, e.g. ``"gui_v3_e2e (deferred)"``.
    @return         Bare target names / prefixes.
    """
    stripped = [entry.split("(")[0].strip() for entry in entries]
    return [name for name in stripped if name]


def hardcoded_test_counts(docs_root):
    """Find duplicated full-suite counts in the live MkDocs tree.

    ``VALIDATION.md`` is the single source of truth. Historical task reports,
    evidence, design snapshots, status snapshots, and handoffs retain their
    measured values; current product and user-guide pages must link to the
    canonical document instead of copying its volatile count.

    @param docs_root  Path to the MkDocs document tree.
    @return           ``path:line: text`` records for every violation.
    """
    violations = []
    root = Path(docs_root)
    if not root.is_dir():
        return ["%s: MkDocs document tree not found" % root]
    for path in sorted(root.rglob("*.md")):
        relative = path.relative_to(root).as_posix()
        if relative == "VALIDATION.md" or _is_historical_doc(relative):
            continue
        for number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            if _HARDCODED_TEST_COUNT_RE.search(line):
                violations.append(
                    "%s:%d: %s" % (relative, number, line.strip())
                )
    return violations


def git_blob_sha(repo, relative_path, commit="HEAD"):
    """Return the git blob sha of a tracked file.

    @param repo           Repository root.
    @param relative_path  Path of the file relative to @p repo.
    @param commit         Commit-ish to resolve against.
    @return               Full blob sha, or "" if the file is untracked.
    """
    try:
        out = subprocess.run(
            ["git", "rev-parse", "%s:%s" % (commit, relative_path)],
            cwd=str(repo),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
        )
    except OSError:
        return ""
    if out.returncode != 0:
        return ""
    return out.stdout.strip()


def commit_relation(repo, doc_commit, built_commit):
    """Classify how the commit recorded in the document relates to the build.

    A document can never record the sha of the commit that contains it, so the
    contract is weaker than equality: the recorded commit must be the built
    commit or an ancestor of it.  Anything else means the number was copied
    from an unrelated tree.

    @param repo          Repository root.
    @param doc_commit    Sha recorded on the ``commit:`` line (may be short).
    @param built_commit  Sha that CI actually checked out and measured.
    @return              One of "same", "ancestor", "unrelated", "unknown".
    """
    if not built_commit:
        return "unknown"
    if built_commit.startswith(doc_commit) or doc_commit.startswith(built_commit):
        return "same"
    try:
        out = subprocess.run(
            ["git", "merge-base", "--is-ancestor", doc_commit, built_commit],
            cwd=str(repo),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
        )
    except OSError:
        return "unknown"
    if out.returncode == 0:
        return "ancestor"
    if out.returncode == 1:
        return "unrelated"
    return "unknown"


#: Paths that decide *which* tests get registered.  A change to any of them
#: can move the count without touching a single test body, so a recorded
#: number stays valid only while this surface is unchanged.  Test sources are
#: deliberately outside the surface -- their effect shows up in passed/total.
REGISTRATION_SURFACE = ("CMakePresets.json", "*CMakeLists.txt", "cmake/")

#: Printed whenever the gate fails.  An unexplained gate failure gets
#: "resolved" by deleting the gate unless the fix is spelled out.
REMEASURE_COMMAND = (
    "cmake --preset validation && "
    "cmake --build --preset validation -j && "
    "ctest --preset validation"
)


def registration_surface_diff(repo, doc_commit, built_commit):
    """List registration-surface paths that changed between two commits.

    A document can never record the sha of the commit that contains it, so
    commit: is allowed to name an ancestor.  Ancestry alone is too weak:
    any commit in between may have added or removed a test registration, which
    invalidates the recorded count while every other element still matches.

    @param repo          Repository root.
    @param doc_commit    Sha recorded on the commit: line.
    @param built_commit  Sha that was actually measured.
    @return              Sorted changed paths; empty when the surface is
                         unchanged or the comparison could not be made.
    """
    if not doc_commit or not built_commit:
        return []
    try:
        out = subprocess.run(
            ["git", "diff", "--name-only", doc_commit, built_commit, "--"]
            + list(REGISTRATION_SURFACE),
            cwd=str(repo),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
        )
    except OSError:
        return []
    if out.returncode != 0:
        return []
    return sorted(line.strip() for line in out.stdout.splitlines() if line.strip())


def check_currency(block, registered, measured, commit, presets_sha,
                   repo=None, cmake_version=""):
    """Compare a parsed currency block against the measured reality.

    @param block        Output of :func:`parse_currency_block`.
    @param registered   Test names from ``ctest -N`` of the canonical config.
    @param measured     ``(passed, total)`` from the canonical ctest run.
    @param commit       Commit sha that was actually built.
    @param presets_sha  Blob sha of ``CMakePresets.json`` at @p commit;
                        an empty string disables the presets comparison.
    @param cmake_version  CMake version that performed the measurement; an
                        empty string disables the toolchain comparison.
    @return             Human-readable problems; empty means the gate passes.
    """
    problems = []

    for key in REQUIRED_KEYS:
        if not block.get(key):
            problems.append(
                "missing required element '%s:' (07_p0_demo.md 6.3)" % key
            )
    if problems:
        return problems

    try:
        doc_passed, doc_total = parse_tests_field(str(block["tests"]))
    except ValueError as exc:
        return [str(exc)]

    run_passed, run_total = measured
    if registered and doc_total != len(registered):
        problems.append(
            "test count drift: document says total=%d, ctest -N registers %d"
            % (doc_total, len(registered))
        )
    if doc_total != run_total:
        problems.append(
            "test count drift: document says total=%d, ctest run reports %d"
            % (doc_total, run_total)
        )
    if doc_passed != run_passed:
        problems.append(
            "pass count drift: document says passed=%d, ctest run reports %d"
            % (doc_passed, run_passed)
        )

    config = str(block["config"])
    if "--preset" not in config:
        problems.append(
            "config must invoke the canonical CMake preset, got %r "
            "(07_p0_demo.md 6.4(1))" % config
        )

    doc_commit = str(block["commit"]).strip()
    if commit:
        relation = commit_relation(repo or Path("."), doc_commit, commit)
        if relation == "unrelated":
            problems.append(
                "commit mismatch: document records %s, which is not an "
                "ancestor of the built commit %s" % (doc_commit, commit)
            )
        elif relation == "ancestor":
            drifted = registration_surface_diff(
                repo or Path("."), doc_commit, commit
            )
            if drifted:
                problems.append(
                    "registration surface changed between the recorded "
                    "commit %s and the built commit %s: %s -- ancestry alone "
                    "does not keep the count valid, these files decide which "
                    "tests exist" % (doc_commit, commit, ", ".join(drifted))
                )

    doc_toolchain = parse_doc_toolchain(str(block.get("toolchain", "")))
    if not doc_toolchain:
        problems.append(
            "toolchain must name the pinned CMake, e.g. 'toolchain: cmake "
            "3.31.10', got %r" % block.get("toolchain", "")
        )
    elif cmake_version and doc_toolchain != cmake_version:
        problems.append(
            "toolchain drift: document says cmake %s, the measurement ran "
            "cmake %s (a toolchain bump is a re-measurement trigger; the "
            "version and the numbers belong in the same commit)"
            % (doc_toolchain, cmake_version)
        )

    if not _DATE_RE.match(str(block["date"]).strip()):
        problems.append("date must be YYYY-MM-DD, got %r" % block["date"])

    if presets_sha:
        recorded = str(block.get("presets", "")).strip()
        if "@" not in recorded:
            problems.append(
                "missing 'presets: CMakePresets.json@<sha>' line "
                "(07_p0_demo.md 6.5(1))"
            )
        else:
            recorded_sha = recorded.split("@", 1)[1].strip()
            if not (
                presets_sha.startswith(recorded_sha)
                or recorded_sha.startswith(presets_sha)
            ):
                problems.append(
                    "CMakePresets.json sha mismatch: document says %s, tree has %s"
                    % (recorded_sha, presets_sha)
                )

    lowered = [name.lower() for name in registered]
    for target in excluded_targets(block.get("excluded", [])):
        needle = target.lower()
        hits = [name for name in lowered if name == needle or needle in name]
        if hits:
            problems.append(
                "target listed as excluded but registered in the canonical "
                "config: %s (%s)" % (target, ", ".join(sorted(set(hits))))
            )

    if run_passed < run_total and not (block.get("flaky") or block.get("failing")):
        problems.append(
            "%d test(s) did not pass but no 'flaky:'/'failing:' line is "
            "recorded (07_p0_demo.md 6.5(2))" % (run_total - run_passed)
        )

    return problems


def main(argv=None):
    """CLI entry point.

    @param argv  Argument vector; ``None`` uses ``sys.argv[1:]``.
    @return      0 when the document matches reality, 1 otherwise.
    """
    parser = argparse.ArgumentParser(
        description="Compare docs/VALIDATION.md against measured ctest results."
    )
    parser.add_argument("--doc", required=True, type=Path,
                        help="path to docs/VALIDATION.md")
    parser.add_argument("--ctest-list", required=True, type=Path,
                        help="file holding 'ctest -N' output")
    parser.add_argument("--ctest-run", required=True, type=Path,
                        help="file holding the ctest run log")
    parser.add_argument("--commit", default="",
                        help="commit sha that was built")
    parser.add_argument("--repo", default=Path("."), type=Path,
                        help="repository root")
    parser.add_argument("--presets-file", default="CMakePresets.json",
                        help="preset file recorded in the document")
    parser.add_argument("--cmake-version-file", default=None, type=Path,
                        help="file holding 'cmake --version' output of the "
                             "toolchain that performed the measurement")
    args = parser.parse_args(argv)

    try:
        block = parse_currency_block(args.doc.read_text(encoding="utf-8"))
        registered = parse_registered_tests(
            args.ctest_list.read_text(encoding="utf-8")
        )
        measured = parse_run_summary(args.ctest_run.read_text(encoding="utf-8"))
        cmake_version = ""
        if args.cmake_version_file is not None:
            cmake_version = parse_cmake_version(
                args.cmake_version_file.read_text(encoding="utf-8")
            )
    except (OSError, ValueError) as exc:
        print("VALIDATION currency gate: FAIL -- %s" % exc, file=sys.stderr)
        return 1

    commit = args.commit.strip()
    presets_sha = git_blob_sha(args.repo, args.presets_file, commit or "HEAD")
    problems = check_currency(
        block, registered, measured, commit, presets_sha, repo=args.repo,
        cmake_version=cmake_version,
    )

    duplicated_counts = hardcoded_test_counts(args.doc.parent)
    if duplicated_counts:
        problems.append(
            "live documentation duplicates the canonical ctest count; "
            "remove the number and link to docs/VALIDATION.md: %s"
            % "; ".join(duplicated_counts)
        )

    if problems:
        print("VALIDATION currency gate: FAIL", file=sys.stderr)
        for problem in problems:
            print("  - %s" % problem, file=sys.stderr)
        print("", file=sys.stderr)
        print("Prescription -- re-measure the canonical configuration:",
              file=sys.stderr)
        print("  %s" % REMEASURE_COMMAND, file=sys.stderr)
        print("then rewrite tests:/commit:/date:/presets:/toolchain: in %s "
              "in the same commit as the change that moved them." % args.doc,
              file=sys.stderr)
        return 1
    print(
        "VALIDATION currency gate: OK (%s, %d registered, commit %s, %s)"
        % (block["tests"], len(registered), commit or "unchecked",
           block.get("toolchain", "toolchain unrecorded"))
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
