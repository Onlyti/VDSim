"""vdsim_preset — render presets for the headless trace renderer.

A preset is **renderer configuration, not trace schema** (contract
``11_trace_contract_spec.md`` §6.3). It only decides which view of one
``.vdtrace`` is drawn; it never changes what the trace contains, which is why
adding presets does not move ``schema_version``.

Three presets are contracted — ``overview``, ``control``, ``tire_limit`` — and
the implementation order is fixed in that order. This module ships
``overview`` only; the other two names are deliberately absent rather than
stubbed, so ``--preset control`` fails loudly instead of silently rendering
something that is not the contracted screen.

Resolution order, highest first (§6.3)::

    CLI option  >  user preset file  >  built-in preset default

A preset declares panel kinds, channels, ranges and visibility — that is the
whole MVP surface. Free-form layout editing is out of scope: panel positions
and the BEV axis coordinate system are fixed by the renderer, and a preset
that pushed text or a panel outside the frame would be a failure, not a
layout choice.

The module depends on the standard library only. YAML presets additionally
need PyYAML; JSON always works.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

#: Preset resolution order, documented for callers that report it.
PRECEDENCE = ("cli", "user-preset", "builtin")

#: Preset names reserved by the contract but not implemented yet. ``road_contact``
#: is re-adjudicated only after the core C2 contact coupling lands (§6.3): naming
#: a preset after contact data the core does not yet resolve would advertise C2
#: while C1 is what actually runs.
RESERVED_PRESET_NAMES = ("road_contact", "control", "tire_limit")

#: Panel channels the renderer derives rather than reads from the trace.
#: ``speed`` is the magnitude of ``v_body``; it is not a recorded channel
#: because it is a function of one.
DERIVED_PANELS = ("speed",)

_BEV_DEFAULTS = {
    "show": True,              # the BEV axis itself
    "waypoint": True,          # path2d overlays (the global route)
    "regions": True,           # region overlays (e.g. a low-mu patch)
    "trail": True,             # the driven path so far
    "velocity_arrow": True,
    "utilization_color": True,  # per-wheel friction colouring
    "colorbar": True,
    "legend": True,
    "view_half_m": None,       # None keeps the geometry-derived window
}

_PANEL_DEFAULTS = {
    "channel": None,
    "label": None,      # None uses the channel name
    "ylim": None,       # None autoscales; autoscale is allowed, escaping is not
    "required": False,  # a missing optional channel drops its panel silently
}

#: Built-in presets. ``overview`` is the contracted "quick look at the driven
#: result": BEV + driven path + optional reference path + speed + steer/long
#: commands.
BUILTIN_PRESETS = {
    "overview": {
        "name": "overview",
        "description": "quick look at a driven result — BEV, driven path, "
                       "optional reference path, speed, steer and long. command",
        "bev": dict(_BEV_DEFAULTS),
        "panels": [
            {"channel": "speed", "label": "speed"},
            {"channel": "u_steer", "label": "steer cmd"},
            {"channel": "u_fx", "label": "long. cmd"},
        ],
    },
}

_TOP_KEYS = ("name", "description", "extends", "bev", "panels")


class PresetError(ValueError):
    """A preset file or name violates the §6.3 configuration contract."""


def builtin_names():
    """Names of the presets shipped with the renderer."""
    return tuple(sorted(BUILTIN_PRESETS))


def _deep_merge(base: dict, over: dict) -> dict:
    """Return ``base`` overlaid with ``over``; nested dicts merge key-wise."""
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _normalize_panels(panels) -> list:
    if not isinstance(panels, (list, tuple)):
        raise PresetError("preset 'panels' must be a list, got %r" % (type(panels).__name__,))
    out = []
    for i, p in enumerate(panels):
        if isinstance(p, str):
            p = {"channel": p}
        if not isinstance(p, dict):
            raise PresetError("panel %d must be a channel name or an object" % (i,))
        unknown = [k for k in p if k not in _PANEL_DEFAULTS]
        if unknown:
            raise PresetError(
                "panel %d has unknown key(s) %s; allowed: %s"
                % (i, ", ".join(sorted(unknown)), ", ".join(sorted(_PANEL_DEFAULTS))))
        entry = dict(_PANEL_DEFAULTS)
        entry.update(p)
        if not entry["channel"] or not isinstance(entry["channel"], str):
            raise PresetError("panel %d needs a non-empty string 'channel'" % (i,))
        ylim = entry["ylim"]
        if ylim is not None:
            if (not isinstance(ylim, (list, tuple)) or len(ylim) != 2
                    or not all(isinstance(v, (int, float)) for v in ylim)
                    or float(ylim[0]) >= float(ylim[1])):
                raise PresetError(
                    "panel %r 'ylim' must be [lo, hi] with lo < hi, got %r"
                    % (entry["channel"], ylim))
            entry["ylim"] = (float(ylim[0]), float(ylim[1]))
        out.append(entry)
    return out


def normalize_preset(raw, source: str = "<preset>") -> dict:
    """Validate one preset object and fill in the built-in defaults.

    A user preset is layered on the built-in named by ``extends`` (default
    ``overview``), so a file that only changes one panel range does not have
    to restate the whole screen.

    :param raw: parsed preset object.
    :param source: name used in error messages.
    :returns: a complete preset dict.
    :raises PresetError: on an unknown key, a reserved name or a malformed panel.
    """
    if not isinstance(raw, dict):
        raise PresetError("%s: a preset must be a JSON/YAML object" % (source,))
    unknown = [k for k in raw if k not in _TOP_KEYS]
    if unknown:
        raise PresetError(
            "%s: unknown preset key(s) %s; allowed: %s"
            % (source, ", ".join(sorted(unknown)), ", ".join(_TOP_KEYS)))

    name = raw.get("name") or Path(source).stem
    if name in RESERVED_PRESET_NAMES:
        raise PresetError(
            "%s: preset name %r is reserved by the render contract and not "
            "implemented yet — pick another name rather than occupying it"
            % (source, name))

    base_name = raw.get("extends", "overview")
    if base_name not in BUILTIN_PRESETS:
        raise PresetError(
            "%s: 'extends' must name a built-in preset %s, got %r"
            % (source, list(builtin_names()), base_name))

    merged = _deep_merge(BUILTIN_PRESETS[base_name], raw)
    merged.pop("extends", None)
    merged["name"] = name

    bev = merged.get("bev", {})
    if not isinstance(bev, dict):
        raise PresetError("%s: 'bev' must be an object" % (source,))
    unknown = [k for k in bev if k not in _BEV_DEFAULTS]
    if unknown:
        raise PresetError(
            "%s: unknown bev key(s) %s; allowed: %s"
            % (source, ", ".join(sorted(unknown)), ", ".join(sorted(_BEV_DEFAULTS))))
    view_half = bev.get("view_half_m")
    if view_half is not None and not (isinstance(view_half, (int, float)) and view_half > 0):
        raise PresetError("%s: bev.view_half_m must be a positive number or null, got %r"
                          % (source, view_half))
    merged["bev"] = _deep_merge(_BEV_DEFAULTS, bev)
    merged["panels"] = _normalize_panels(merged.get("panels", []))
    return merged


def load_preset(spec=None) -> dict:
    """Resolve ``spec`` to a complete preset.

    :param spec: built-in preset name, path to a ``.yaml``/``.yml``/``.json``
        preset file, an already-parsed dict, or ``None`` for ``overview``.
    :returns: a complete preset dict.
    :raises PresetError: on an unknown name, an unreadable file or an invalid
        preset body.
    """
    if spec is None:
        spec = "overview"
    if isinstance(spec, dict):
        return normalize_preset(spec, "<dict>")
    text = str(spec)
    if text in BUILTIN_PRESETS:
        return normalize_preset(copy.deepcopy(BUILTIN_PRESETS[text]), text)

    p = Path(text)
    if not p.is_file():
        raise PresetError(
            "unknown preset %r — built-ins are %s; anything else must be a path "
            "to a .yaml/.yml/.json preset file"
            % (text, list(builtin_names())))
    body = p.read_text(encoding="utf-8")
    if p.suffix.lower() in (".yaml", ".yml"):
        try:
            import yaml
        except ImportError:
            raise PresetError(
                "%s is YAML but PyYAML is not installed; install it or write "
                "the preset as JSON" % (p,))
        raw = yaml.safe_load(body)
    elif p.suffix.lower() == ".json":
        try:
            raw = json.loads(body)
        except ValueError as exc:
            raise PresetError("%s is not valid JSON: %s" % (p, exc))
    else:
        raise PresetError(
            "%s: a preset file must be .yaml, .yml or .json (got %r)"
            % (p, p.suffix))
    return normalize_preset(raw, str(p))


def apply_overrides(preset: dict, panels=None, view_half=None,
                    waypoint=None, legend=None) -> dict:
    """Layer CLI options on top of a resolved preset — the top of §6.3's order.

    Every argument is ``None`` when the caller did not pass the option, so an
    unset CLI flag never overwrites a preset value.

    :param preset: preset from :func:`load_preset`.
    :param panels: replacement channel list (names only).
    :param view_half: BEV half-window [m].
    :param waypoint: draw path2d overlays.
    :param legend: draw the BEV legend.
    :returns: a new preset dict; the input is not mutated.
    """
    out = copy.deepcopy(preset)
    if panels is not None:
        out["panels"] = _normalize_panels(list(panels))
    if view_half is not None:
        out["bev"]["view_half_m"] = float(view_half)
    if waypoint is not None:
        out["bev"]["waypoint"] = bool(waypoint)
    if legend is not None:
        out["bev"]["legend"] = bool(legend)
    return out
