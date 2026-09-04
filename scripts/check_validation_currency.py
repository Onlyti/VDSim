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

#: The four elements every recorded number must carry (07_p0_demo.md 6.3).
REQUIRED_KEYS = ("tests", "config", "commit", "date")

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
_FENCE = chr(96) * 3


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


def excluded_targets(entries):
    """Strip the reason text from ``excluded:`` lines.

    @param entries  Raw ``excluded:`` values, e.g. ``"gui_v3_e2e (deferred)"``.
    @return         Bare target names / prefixes.
    """
    stripped = [entry.split("(")[0].strip() for entry in entries]
    return [name for name in stripped if name]


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


def check_currency(block, registered, measured, commit, presets_sha,
                   repo=None):
    """Compare a parsed currency block against the measured reality.

    @param block        Output of :func:`parse_currency_block`.
    @param registered   Test names from ``ctest -N`` of the canonical config.
    @param measured     ``(passed, total)`` from the canonical ctest run.
    @param commit       Commit sha that was actually built.
    @param presets_sha  Blob sha of ``CMakePresets.json`` at @p commit;
                        an empty string disables the presets comparison.
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
    args = parser.parse_args(argv)

    try:
        block = parse_currency_block(args.doc.read_text(encoding="utf-8"))
        registered = parse_registered_tests(
            args.ctest_list.read_text(encoding="utf-8")
        )
        measured = parse_run_summary(args.ctest_run.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        print("VALIDATION currency gate: FAIL -- %s" % exc, file=sys.stderr)
        return 1

    commit = args.commit.strip()
    presets_sha = git_blob_sha(args.repo, args.presets_file, commit or "HEAD")
    problems = check_currency(
        block, registered, measured, commit, presets_sha, repo=args.repo
    )

    if problems:
        print("VALIDATION currency gate: FAIL", file=sys.stderr)
        for problem in problems:
            print("  - %s" % problem, file=sys.stderr)
        return 1
    print(
        "VALIDATION currency gate: OK (%s, %d registered, commit %s)"
        % (block["tests"], len(registered), commit or "unchecked")
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
