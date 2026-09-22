#!/usr/bin/env python3
"""Single check that the name ``vdsim`` resolves to VDSim's compiled core.

Another distribution on PyPI ships a top-level ``vdsim/`` *package*.  Installed
into the same environment as our wheel it wins the import over our
``vdsim.cpython-*.so`` -- pip neither warns nor overwrites, so the failure
surfaces much later as an ``AttributeError`` on an unrelated line.  The modules
that need the core therefore call :func:`load_core` instead of ``import
vdsim``; the decision itself lives here once so the modules cannot drift.

The check is structural on purpose: it asks whether the loaded module is a
compiled extension exposing the core API, never which distribution shipped it.
"""
from __future__ import annotations

import importlib
import importlib.machinery
from pathlib import Path

__all__ = ["CoreShadowedError", "check_core", "load_core", "diagnose"]


class CoreShadowedError(ImportError):
    """``vdsim`` resolved to something that is not VDSim's compiled core."""


#: Symbols every build of the core exposes (pybind11 ``cosim/bindings.cpp``).
#: Checked as a set so a foreign module cannot pass by coincidence on one name.
EXPECTED_SYMBOLS = ("SimSession", "VehicleParams", "TireParams", "make_sim_session")


def diagnose(mod):
    """Return ``None`` if *mod* is the compiled core, else a one-line reason.

    :param mod: the module object bound to the name ``vdsim``.
    :returns: ``None`` when the module passes both checks, otherwise a short
              phrase naming which property failed.
    """
    origin = getattr(mod, "__file__", None)
    if origin is None:
        return "it has no __file__ (a namespace package, not an extension module)"
    if not any(str(origin).endswith(s)
               for s in importlib.machinery.EXTENSION_SUFFIXES):
        return "it is Python source, not a compiled extension module"
    missing = [s for s in EXPECTED_SYMBOLS if not hasattr(mod, s)]
    if missing:
        return "it is missing core symbols: %s" % ", ".join(
            "vdsim." + s for s in missing)
    return None


def _message(mod, reason):
    origin = getattr(mod, "__file__", None) or "<no __file__>"
    return (
        "'import vdsim' loaded %s -- %s.\n"
        "Cause: another installed distribution provides a top-level 'vdsim' "
        "package, which shadows VDSim's vdsim.cpython-*.so on sys.path "
        "(pip does not warn about this).\n"
        "Fix: uninstall the shadowing distribution "
        "(e.g. `pip uninstall vehicle-dynamics-sim`) or install VDSim into "
        "its own virtual environment." % (origin, reason)
    )


def check_core(mod):
    """Return *mod* if it is the compiled core, else raise.

    :param mod: the module object bound to the name ``vdsim``.
    :raises CoreShadowedError: when :func:`diagnose` reports a mismatch.
    """
    reason = diagnose(mod)
    if reason is not None:
        raise CoreShadowedError(_message(mod, reason))
    return mod


def load_core(fallback_paths=()):
    """Import ``vdsim`` and return it only if it is the compiled core.

    *fallback_paths* are development build directories tried, in order, when
    the ambient import is missing **or** shadowed; this keeps a repo checkout
    working even in an environment that has the shadowing package installed.

    :param fallback_paths: directories prepended to ``sys.path`` on retry.
    :returns: the compiled ``vdsim`` extension module.
    :raises CoreShadowedError: the name resolves to a foreign module and no
            fallback path yields the core.
    :raises ImportError: ``vdsim`` is not importable at all.
    """
    import sys

    first_error = None
    try:
        mod = importlib.import_module("vdsim")
    except ImportError as exc:
        mod, first_error = None, exc
    if mod is not None and diagnose(mod) is None:
        return mod

    for raw in fallback_paths:
        path = str(raw)
        if not Path(path).is_dir():
            continue
        while path in sys.path:
            sys.path.remove(path)
        sys.path.insert(0, path)
        sys.modules.pop("vdsim", None)
        importlib.invalidate_caches()
        try:
            candidate = importlib.import_module("vdsim")
        except ImportError:
            continue
        if diagnose(candidate) is None:
            return candidate
        mod = candidate

    if mod is None:
        raise first_error
    return check_core(mod)
