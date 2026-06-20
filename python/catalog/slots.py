"""Blueprint part slots for vehicle assembly (catalog model)."""
from __future__ import annotations

from typing import List, Tuple

# (slot_key, GUI label, catalog part type)
# Re-taxonomy step 2: the vehicle is body (mass/inertia/layout) + aero + ride
# (spring/damper/ARB) + chassis (suspension links/hardpoints/knuckle, was the
# susp_kinematics part) + tire/brake/steering/drivetrain. "chassis" now denotes
# the suspension linkage, not the mass bundle.
OPTIONAL_SLOTS: List[Tuple[str, str, str]] = [
    ("tire_rear", "Rear tire (axle)", "tire"),
    ("tire_fr", "Front-right tire", "tire"),
    ("tire_rl", "Rear-left tire", "tire"),
    ("tire_rr", "Rear-right tire", "tire"),
]

BASE_SLOTS: List[Tuple[str, str, str]] = [
    ("body", "Body", "body"),
    ("aero", "Aero", "aero"),
    ("ride", "Ride", "ride"),
    ("tire", "Tire", "tire"),
    ("brake", "Brake", "brake"),
    ("steering", "Steering", "steering"),
    ("drivetrain", "Drivetrain", "drivetrain"),
]

L3_SLOTS: List[Tuple[str, str, str]] = [
    ("front_chassis", "Front chassis", "chassis"),
    ("rear_chassis", "Rear chassis", "chassis"),
]


def slots_for_level(level: str) -> List[Tuple[str, str, str]]:
    out = list(BASE_SLOTS) + list(OPTIONAL_SLOTS)
    if str(level) in ("L3", "L4"):
        out.extend(L3_SLOTS)
    return out
