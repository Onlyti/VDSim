"""Blueprint part slots for vehicle assembly (catalog model)."""
from __future__ import annotations

from typing import List, Tuple

# (slot_key, GUI label, catalog part type)
BASE_SLOTS: List[Tuple[str, str, str]] = [
    ("chassis", "Chassis", "chassis"),
    ("tire", "Tire", "tire"),
    ("brake", "Brake", "brake"),
    ("steering", "Steering", "steering"),
    ("drivetrain", "Drivetrain", "drivetrain"),
]

L3_SLOTS: List[Tuple[str, str, str]] = [
    ("front_susp_kin", "Front suspension", "susp_kinematics"),
    ("rear_susp_kin", "Rear suspension", "susp_kinematics"),
]


def slots_for_level(level: str) -> List[Tuple[str, str, str]]:
    out = list(BASE_SLOTS)
    if str(level) in ("L3", "L4"):
        out.extend(L3_SLOTS)
    return out
