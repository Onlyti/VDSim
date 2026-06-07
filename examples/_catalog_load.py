"""Resolve builtin catalog presets for examples (v0.3)."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent


def load_vehicle_tire(vehicle="sedan", tire="default_pacejka"):
    sys.path.insert(0, str(REPO / "build" / "python"))
    sys.path.insert(0, str(REPO / "python"))
    from vdsim_lab import Tire, Vehicle
    return Vehicle.preset(vehicle).vp, Tire.preset(tire).tp
