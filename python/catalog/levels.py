"""Dynamics ladder IDs (K, L1–L5) and human-readable names.

Runtime strings stay K/L1/…; theory docs use Ld0–Ld5 (Ld0 = K).
"""
from __future__ import annotations

from typing import Any, Dict, List, Mapping, Optional

LEVEL_LADDER: List[Dict[str, str]] = [
    {
        "id": "K",
        "ld": "Ld0",
        "name": "Kinematic",
        "title": "Kinematic bicycle",
        "dof": "2D pose; yaw_rate = v·tan(δ)/L",
        "summary": "No tire slip or Pacejka forces; path / controller smoke tests.",
    },
    {
        "id": "L1",
        "ld": "Ld1",
        "name": "Bicycle",
        "title": "Single-track bicycle",
        "dof": "5-DOF planar",
        "summary": "Axle-averaged Pacejka; linear understeer baseline.",
    },
    {
        "id": "L2",
        "ld": "Ld2",
        "name": "Seven-DOF",
        "title": "Planar seven-DOF (per-wheel)",
        "dof": "3 chassis + 4 wheel ω",
        "summary": "Per-wheel tire, Ackermann steer, differential, roll transfer.",
    },
    {
        "id": "L3",
        "ld": "Ld3",
        "name": "Fourteen-DOF",
        "title": "Ride fourteen-DOF",
        "dof": "sprung heave/roll/pitch + 4 unsprung z",
        "summary": "Dynamic ride, road profile, anti-dive/squat; L2 planar inner loop.",
    },
    {
        "id": "L4",
        "ld": "Ld4",
        "name": "Hardpoint",
        "title": "Hardpoint kinematic multibody",
        "dof": "L3 + attach kinematics",
        "summary": "Toe/camber/RC from suspension kin YAML; M4 bushing offline.",
    },
    {
        "id": "L5",
        "ld": "Ld5",
        "name": "Free-3D",
        "title": "Free 3D stunt",
        "dof": "6-DOF body + quaternion",
        "summary": "World-z hub contact, airborne, jump/loop/terrain.",
    },
]

LEVELS: List[str] = [e["id"] for e in LEVEL_LADDER]
LEVEL_BY_ID: Mapping[str, Dict[str, str]] = {e["id"]: e for e in LEVEL_LADDER}
LD_BY_LEVEL: Mapping[str, str] = {e["id"]: e["ld"] for e in LEVEL_LADDER}


def level_label(level_id: str) -> str:
    e = LEVEL_BY_ID.get(str(level_id))
    return f"{level_id} — {e['name']}" if e else str(level_id)


def level_title(level_id: str) -> str:
    e = LEVEL_BY_ID.get(str(level_id))
    return e["title"] if e else str(level_id)


def level_entry(level_id: str) -> Optional[Dict[str, Any]]:
    return LEVEL_BY_ID.get(str(level_id))
