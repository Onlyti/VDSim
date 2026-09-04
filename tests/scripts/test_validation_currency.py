#!/usr/bin/env python3
"""Unit tests for ``scripts/check_validation_currency.py``.

The gate exists to make a stale ``docs/VALIDATION.md`` fail CI, so every test
here pins one way the document can drift away from the measured reality.
"""

import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import check_validation_currency as cvc  # noqa: E402

DOC_TEMPLATE = """# VDSim validation

Some prose above the block.

<!-- VALIDATION-CURRENCY BEGIN -->
tests:    {tests}
config:   cmake --preset validation && ctest --preset validation
presets:  CMakePresets.json@{presets}
toolchain: cmake {toolchain}
commit:   {commit}
date:     {date}
{extra}<!-- VALIDATION-CURRENCY END -->

Prose below the block.
"""

CTEST_LIST = """Test #1: alpha
  Test #2: beta
  Test  #3: gamma

Total Tests: 3
"""

CTEST_RUN_ALL_PASS = """
100% tests passed, 0 tests failed out of 3

Total Test time (real) = 1.00 sec
"""

CTEST_RUN_ONE_FAIL = """
66% tests passed, 1 tests failed out of 3

Total Test time (real) = 1.00 sec
"""


def make_doc(tests="3/3", presets="a" * 40, commit="b" * 40,
             date="2026-09-04", extra="", toolchain="3.31.10"):
    """Build a VALIDATION.md body with a currency block.

    @param tests    Value for the ``tests:`` line.
    @param presets  Blob sha recorded on the ``presets:`` line.
    @param commit   Commit sha recorded on the ``commit:`` line.
    @param date     Value for the ``date:`` line.
    @param extra    Extra raw lines inserted before the end marker.
    @param toolchain  CMake version recorded on the ``toolchain:`` line.
    @return         Markdown source as a string.
    """
    return DOC_TEMPLATE.format(
        tests=tests, presets=presets, commit=commit, date=date, extra=extra,
        toolchain=toolchain,
    )


class ParseCurrencyBlockTest(unittest.TestCase):
    """The block parser must ignore surrounding prose and collect repeats."""

    def test_reads_all_required_elements(self):
        block = cvc.parse_currency_block(make_doc())
        self.assertEqual(block["tests"], "3/3")
        self.assertEqual(block["date"], "2026-09-04")
        self.assertIn("--preset validation", block["config"])
        self.assertEqual(block["commit"], "b" * 40)
        self.assertEqual(block["toolchain"], "cmake 3.31.10")

    def test_collects_repeated_keys(self):
        extra = ("excluded: gui_v3_e2e (deferred)\n"
                 "excluded: ergaccess (optional dep)\n"
                 "flaky:    delta (2/20)\n")
        block = cvc.parse_currency_block(make_doc(extra=extra))
        self.assertEqual(len(block["excluded"]), 2)
        self.assertEqual(len(block["flaky"]), 1)
        self.assertEqual(block["failing"], [])

    def test_missing_markers_raise(self):
        with self.assertRaises(ValueError):
            cvc.parse_currency_block("# no block here\n")


class ParseCtestOutputTest(unittest.TestCase):
    """ctest output parsing must survive ctest's variable indentation."""

    def test_registered_names(self):
        self.assertEqual(
            cvc.parse_registered_tests(CTEST_LIST), ["alpha", "beta", "gamma"]
        )

    def test_names_with_spaces_are_counted(self):
        """gtest parameterised names embed spaces; they are still tests."""
        lf = chr(10)
        listing = "".join([
            "  Test #1: alpha" + lf,
            "  Test #2: Suite/Case.Name/0-byte object <00-00 00-00>" + lf,
            "Total Tests: 2" + lf,
        ])
        self.assertEqual(
            cvc.parse_registered_tests(listing),
            ["alpha", "Suite/Case.Name/0-byte object <00-00 00-00>"],
        )

    def test_run_summary(self):
        self.assertEqual(cvc.parse_run_summary(CTEST_RUN_ALL_PASS), (3, 3))
        self.assertEqual(cvc.parse_run_summary(CTEST_RUN_ONE_FAIL), (2, 3))

    def test_missing_summary_raises(self):
        with self.assertRaises(ValueError):
            cvc.parse_run_summary("no summary line at all")

    def test_cmake_version_is_read_from_the_banner(self):
        banner = ("cmake version 3.31.10" + chr(10) + chr(10)
                  + "CMake suite maintained and supported by Kitware")
        self.assertEqual(cvc.parse_cmake_version(banner), "3.31.10")

    def test_missing_cmake_version_raises(self):
        with self.assertRaises(ValueError):
            cvc.parse_cmake_version("not a cmake banner")


class CheckCurrencyTest(unittest.TestCase):
    """The gate itself: each drift mode must produce a problem."""

    def setUp(self):
        self.registered = ["alpha", "beta", "gamma"]
        self.commit = "b" * 40
        self.presets = "a" * 40

    def run_gate(self, doc, registered=None, measured=(3, 3),
                 cmake_version="3.31.10"):
        """Parse @p doc and run the gate against the measured numbers."""
        block = cvc.parse_currency_block(doc)
        return cvc.check_currency(
            block,
            self.registered if registered is None else registered,
            measured,
            self.commit,
            self.presets,
            cmake_version=cmake_version,
        )

    def test_matching_document_passes(self):
        self.assertEqual(self.run_gate(make_doc()), [])

    def test_total_drift_fails(self):
        problems = self.run_gate(make_doc(tests="4/4"), measured=(3, 3))
        self.assertTrue(any("count drift" in p for p in problems))

    def test_pass_count_drift_fails(self):
        doc = make_doc(tests="3/3")
        problems = self.run_gate(doc, measured=(2, 3))
        self.assertTrue(any("pass count drift" in p for p in problems))

    def test_short_commit_prefix_is_accepted(self):
        self.assertEqual(self.run_gate(make_doc(commit=self.commit[:9])), [])

    def test_unrelated_commit_fails(self):
        original = cvc.commit_relation
        cvc.commit_relation = lambda repo, doc, built: "unrelated"
        try:
            problems = self.run_gate(make_doc(commit="c" * 40))
        finally:
            cvc.commit_relation = original
        self.assertTrue(any("commit mismatch" in p for p in problems))

    def test_ancestor_commit_passes(self):
        original = cvc.commit_relation
        cvc.commit_relation = lambda repo, doc, built: "ancestor"
        try:
            problems = self.run_gate(make_doc(commit="c" * 40))
        finally:
            cvc.commit_relation = original
        self.assertEqual(problems, [])

    def test_unknown_ancestry_does_not_fail(self):
        # Shallow CI checkouts cannot resolve ancestry; the number comparison
        # is the real gate, so unresolvable ancestry must not fail the build.
        original = cvc.commit_relation
        cvc.commit_relation = lambda repo, doc, built: "unknown"
        try:
            problems = self.run_gate(make_doc(commit="c" * 40))
        finally:
            cvc.commit_relation = original
        self.assertEqual(problems, [])

    def test_presets_sha_mismatch_fails(self):
        problems = self.run_gate(make_doc(presets="d" * 40))
        self.assertTrue(any("sha mismatch" in p for p in problems))

    def test_bad_date_fails(self):
        problems = self.run_gate(make_doc(date="Sep 2026"))
        self.assertTrue(any("date must be" in p for p in problems))

    def test_missing_element_fails(self):
        doc = make_doc().replace("date:     2026-09-04\n", "")
        problems = self.run_gate(doc)
        self.assertTrue(any("missing required element 'date:'" in p
                            for p in problems))

    def test_excluded_target_still_registered_fails(self):
        doc = make_doc(extra="excluded: beta (should be off)\n")
        problems = self.run_gate(doc)
        self.assertTrue(any("listed as excluded but registered" in p
                            for p in problems))

    def test_excluded_target_absent_passes(self):
        doc = make_doc(extra="excluded: gui_v3_e2e (deferred)\n")
        self.assertEqual(self.run_gate(doc), [])

    def test_silent_failure_needs_a_flaky_or_failing_line(self):
        doc = make_doc(tests="2/3")
        problems = self.run_gate(doc, measured=(2, 3))
        self.assertTrue(any("no 'flaky:'/'failing:' line" in p
                            for p in problems))

        doc_ok = make_doc(tests="2/3", extra="failing: gamma (known break)\n")
        self.assertEqual(self.run_gate(doc_ok, measured=(2, 3)), [])

    def test_toolchain_drift_fails(self):
        """A count measured with another CMake is another measurement."""
        problems = self.run_gate(make_doc(toolchain="4.0.3"))
        self.assertTrue(any("toolchain drift" in p for p in problems))

    def test_missing_toolchain_fails(self):
        doc = make_doc().replace("toolchain: cmake 3.31.10\n", "")
        problems = self.run_gate(doc)
        self.assertTrue(any("missing required element 'toolchain:'" in p
                            for p in problems))

    def test_toolchain_must_name_cmake(self):
        doc = make_doc().replace("toolchain: cmake 3.31.10",
                                 "toolchain: whatever the runner ships")
        problems = self.run_gate(doc)
        self.assertTrue(any("toolchain must name the pinned CMake" in p
                            for p in problems))

    def test_unmeasured_toolchain_does_not_fail(self):
        """Local runs without a version record still gate on the numbers."""
        self.assertEqual(self.run_gate(make_doc(), cmake_version=""), [])

    def test_ancestor_with_moved_registration_surface_fails(self):
        """Ancestry alone must not carry a count across a CMake change."""
        original_rel = cvc.commit_relation
        original_diff = cvc.registration_surface_diff
        cvc.commit_relation = lambda repo, doc, built: "ancestor"
        cvc.registration_surface_diff = (
            lambda repo, doc, built: ["tests/CMakeLists.txt"]
        )
        try:
            problems = self.run_gate(make_doc(commit="c" * 40))
        finally:
            cvc.commit_relation = original_rel
            cvc.registration_surface_diff = original_diff
        self.assertTrue(any("registration surface changed" in p
                            for p in problems))

    def test_identical_commit_skips_surface_check(self):
        """Same sha cannot differ from itself; do not shell out for it."""
        called = []
        original_diff = cvc.registration_surface_diff
        cvc.registration_surface_diff = (
            lambda repo, doc, built: called.append(1) or []
        )
        try:
            self.assertEqual(self.run_gate(make_doc()), [])
        finally:
            cvc.registration_surface_diff = original_diff
        self.assertEqual(called, [])

    def test_config_without_preset_fails(self):
        doc = make_doc().replace(
            "cmake --preset validation && ctest --preset validation",
            "cmake -B build -DCMAKE_BUILD_TYPE=Release",
        )
        problems = self.run_gate(doc)
        self.assertTrue(any("must invoke the canonical CMake preset" in p
                            for p in problems))


class RegistrationSurfaceDiffTest(unittest.TestCase):
    """The helper runs real git; stubs elsewhere would hide a wrong pathspec."""

    def setUp(self):
        self.repo = Path(tempfile.mkdtemp(prefix="vdsim_surface_"))
        self.addCleanup(shutil.rmtree, str(self.repo), True)
        if shutil.which("git") is None:
            self.skipTest("git not available")
        self._git("init", "-q")
        self._git("config", "user.email", "t@example.com")
        self._git("config", "user.name", "t")
        self._write("CMakeLists.txt", "add_test(NAME a COMMAND true)")
        self._write("tests/CMakeLists.txt", "add_test(NAME b COMMAND true)")
        self._write("cmake/helpers.cmake", "# helpers")
        self._write("tests/test_body.cpp", "int main() { return 0; }")
        self._git("add", "-A")
        self._git("commit", "-qm", "base")
        self.base = self._git("rev-parse", "HEAD").strip()

    def _git(self, *args):
        out = subprocess.run(
            ["git"] + list(args), cwd=str(self.repo),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True,
        )
        return out.stdout

    def _write(self, relative, text):
        path = self.repo / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text + chr(10), encoding="utf-8")

    def _commit(self, relative, text, message):
        self._write(relative, text)
        self._git("add", "-A")
        self._git("commit", "-qm", message)
        return self._git("rev-parse", "HEAD").strip()

    def test_unchanged_surface_is_empty(self):
        head = self._commit("README.md", "prose", "docs only")
        self.assertEqual(
            cvc.registration_surface_diff(self.repo, self.base, head), []
        )

    def test_nested_cmakelists_is_on_the_surface(self):
        head = self._commit("tests/CMakeLists.txt",
                            "add_test(NAME b COMMAND true)" + chr(10)
                            + "add_test(NAME c COMMAND true)", "new test")
        self.assertEqual(
            cvc.registration_surface_diff(self.repo, self.base, head),
            ["tests/CMakeLists.txt"],
        )

    def test_cmake_module_is_on_the_surface(self):
        head = self._commit("cmake/helpers.cmake", "# changed", "cmake module")
        self.assertEqual(
            cvc.registration_surface_diff(self.repo, self.base, head),
            ["cmake/helpers.cmake"],
        )

    def test_test_body_is_off_the_surface(self):
        head = self._commit("tests/test_body.cpp", "int main() { return 1; }",
                            "body only")
        self.assertEqual(
            cvc.registration_surface_diff(self.repo, self.base, head), []
        )

    def test_unresolvable_shas_do_not_raise(self):
        self.assertEqual(
            cvc.registration_surface_diff(self.repo, "d" * 40, "e" * 40), []
        )


class RealDocumentTest(unittest.TestCase):
    """The repository's own VALIDATION.md must carry a parsable block."""

    def test_repo_document_has_currency_block(self):
        doc = REPO_ROOT / "docs" / "VALIDATION.md"
        if not doc.exists():
            self.skipTest("docs/VALIDATION.md not present")
        block = cvc.parse_currency_block(doc.read_text(encoding="utf-8"))
        for key in cvc.REQUIRED_KEYS:
            self.assertTrue(block.get(key), "missing %s" % key)
        cvc.parse_tests_field(str(block["tests"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
