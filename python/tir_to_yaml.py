"""
Convert an AVL/Pacejka-style .tir file into a VDSim TireParams YAML.

The .tir format uses [SECTION] headers and key = value lines.  We map a
small subset of fields to the simplified MF96 model used by VDSim.  The
full MF2002 expansion is Phase 2.

Usage:
    python3 tir_to_yaml.py <input.tir> <output.yaml>
"""
import argparse
import re
from pathlib import Path
import yaml


# Mapping from .tir field names -> VDSim TireParams keys.
# When the .tir has a different exponent / sign convention this is the place
# to fix it.
FIELD_MAP = {
    "BBX1":             "B_long",   # longitudinal stiffness factor
    "CFX1":             "C_long",
    "DFX1":             "D_long",
    "EFX1":             "E_long",
    "BBY1":             "B_lat",
    "CFY1":             "C_lat",
    "DFY1":             "D_lat",
    "EFY1":             "E_lat",
    "LMUX":             "mu_nominal",   # peak friction scaling
    "FNOMIN":           "Fz_nominal",
    "VERTICAL_STIFFNESS": "tire_vertical_stiffness",
    "QSY1":             "rolling_resistance",
    # Camber stiffness (MF combined) — varies by .tir vendor.
    "QSGZ1":            "camber_stiffness",
}


def parse_tir(path: str) -> dict:
    return parse_tir_text(Path(path).read_text(errors="ignore"))


def parse_tir_text(text: str) -> dict:
    fields = {}
    pat = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)")
    for line in text.splitlines():
        line = line.split("$", 1)[0]
        m = pat.match(line)
        if m:
            fields[m.group(1).upper()] = float(m.group(2))
    return fields


def tir_to_params(tir_fields: dict) -> dict:
    out = {}
    for src, dst in FIELD_MAP.items():
        if src in tir_fields:
            out[dst] = tir_fields[src]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tir_in")
    ap.add_argument("yaml_out")
    args = ap.parse_args()

    fields = parse_tir(args.tir_in)
    out = {}
    for src, dst in FIELD_MAP.items():
        if src in fields:
            out[dst] = fields[src]
    # Reasonable defaults for missing entries.
    out.setdefault("B_long", 10.0); out.setdefault("C_long", 1.65)
    out.setdefault("D_long", 1.0);  out.setdefault("E_long", 0.97)
    out.setdefault("B_lat",  8.0);  out.setdefault("C_lat",  1.30)
    out.setdefault("D_lat",  1.0);  out.setdefault("E_lat", -1.0)
    out.setdefault("mu_nominal", 1.0); out.setdefault("Fz_nominal", 4000.0)

    Path(args.yaml_out).write_text(yaml.safe_dump(out, default_flow_style=False))
    print(f"parsed {len(fields)} fields from {args.tir_in}, "
          f"mapped {sum(1 for k in FIELD_MAP if k in fields)} to {args.yaml_out}")


if __name__ == "__main__":
    main()
