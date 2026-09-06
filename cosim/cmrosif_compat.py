"""CarMaker cmrosif message ↔ VDS1 UDP conversions (no ROS dependency).

Topic names match IPG CMRosIF defaults in CMNode_ROS1.cpp so existing
controllers (e.g. carmaker_vds_client/cm_ablation_controller.py) can subscribe
to /carmaker/dynamic_info and publish /carmaker/control_signal unchanged.
"""
from __future__ import annotations

import math
from typing import Any, Mapping, MutableMapping

DEFAULT_TOPICS = {
    "control_signal": "/carmaker/control_signal",
    "dynamic_info": "/carmaker/dynamic_info",
    "uaq_out": "/carmaker/uaq_out",
    "gnss": "/carmaker/gnss",
    "cmremote": "/carmaker/cmremote",
}

DEFAULT_VEHICLE = {
    "steer_ratio": 14.46,
    "wheelbase_m": 3.0,
    "max_decel_mps2": 10.0,
    "max_steer_wheel_rad": 480.0 * math.pi / 180.0,
}


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def control_signal_to_vds1(
    steerangle: float,
    brake: float,
    gas: float,
    gear: int = 1,
    *,
    steer_ratio: float = DEFAULT_VEHICLE["steer_ratio"],
    max_steer_wheel_rad: float = DEFAULT_VEHICLE["max_steer_wheel_rad"],
) -> dict[str, float | int]:
    """carmaker_msgs/Control_Signal → VDS1 CMD fields."""
    sa = steerangle
    if not math.isfinite(sa):
        sa = 0.0
    sa = _clamp(sa, -max_steer_wheel_rad, max_steer_wheel_rad)
    g = _clamp(float(gas) if math.isfinite(gas) else 0.0, 0.0, 1.0)
    b = _clamp(float(brake) if math.isfinite(brake) else 0.0, 0.0, 1.0)
    return {
        "steer": sa / float(steer_ratio),
        "throttle": g,
        "brake": b,
        "gear": int(gear),
    }


def vds1_state_to_dynamic_info(
    st: Mapping[str, Any],
    *,
    steer_ratio: float = DEFAULT_VEHICLE["steer_ratio"],
    cycleno: int = 0,
    synthdelay: float = 0.0,
) -> dict[str, float | int]:
    """VDS1 decoded STATE → carmaker_msgs/DynamicsInfo field dict."""
    ws = list(st.get("wheel_spin") or [0.0, 0.0, 0.0, 0.0])
    while len(ws) < 4:
        ws.append(0.0)
    steer_wheel = float(st.get("steer_applied", 0.0)) * float(steer_ratio)
    out: dict[str, float | int] = {
        "cycleno": int(cycleno),
        "synthdelay": float(synthdelay),
        "latitude": 0.0,
        "longitude": 0.0,
        "altitude": float(st.get("z", 0.0)),
        "Car_Roll": float(st.get("roll", 0.0)),
        "Car_Pitch": float(st.get("pitch", 0.0)),
        "Car_Yaw": float(st.get("yaw", 0.0)),
        "Car_vx": float(st.get("vx", 0.0)),
        "Car_vy": float(st.get("vy", 0.0)),
        "Car_vz": float(st.get("vz", 0.0)),
        "Car_RollVel": float(st.get("roll_rate", 0.0)),
        "Car_PitchVel": float(st.get("pitch_rate", 0.0)),
        "Car_YawVel": float(st.get("yaw_rate", 0.0)),
        "Car_ax": float(st.get("ax", 0.0)),
        "Car_ay": float(st.get("ay", 0.0)),
        "Car_az": 0.0,
        "Car_RollAcc": 0.0,
        "Car_PitchAcc": 0.0,
        "Car_YawAcc": 0.0,
        "Steer_WhlAng": steer_wheel,
        "VC_Gas": float(st.get("throttle_applied", 0.0)),
        "VC_Brake": float(st.get("brake_applied", 0.0)),
        "VC_SelectorCtrl": 1.0,
        "Vhcl_FL_rotv": float(ws[0]),
        "Vhcl_FR_rotv": float(ws[1]),
        "Vhcl_RL_rotv": float(ws[2]),
        "Vhcl_RR_rotv": float(ws[3]),
        "Vhcl_FL_rz": 0.0,
        "Vhcl_FR_rz": 0.0,
        "Vhcl_RL_rz": 0.0,
        "Vhcl_RR_rz": 0.0,
        "PowerTrain_MotorIF_0_rotv": 0.0,
        "PowerTrain_MotorIF_0_Trq": 0.0,
        "PowerTrain_MotorIF_0_PwrElec": 0.0,
        "PowerTrain_MotorIF_1_rotv": 0.0,
        "PowerTrain_MotorIF_1_Trq": 0.0,
        "PowerTrain_MotorIF_1_PwrElec": 0.0,
    }
    _fill_imu_fields(out, st)
    return out


def vds1_state_to_uaq_out(
    st: Mapping[str, Any],
    *,
    steer_ratio: float = DEFAULT_VEHICLE["steer_ratio"],
    cycleno: int = 0,
    synthdelay: float = 0.0,
) -> dict[str, float | int]:
    base = vds1_state_to_dynamic_info(
        st, steer_ratio=steer_ratio, cycleno=cycleno, synthdelay=synthdelay)
    return {k: v for k, v in base.items()
            if not k.startswith(("latitude", "longitude", "altitude", "PowerTrain"))}


def _fill_imu_fields(out: MutableMapping[str, float], st: Mapping[str, Any]) -> None:
    vx, vy, vz = float(st.get("vx", 0.0)), float(st.get("vy", 0.0)), float(st.get("vz", 0.0))
    wx = float(st.get("roll_rate", 0.0))
    wy = float(st.get("pitch_rate", 0.0))
    wz = float(st.get("yaw_rate", 0.0))
    ax, ay = float(st.get("ax", 0.0)), float(st.get("ay", 0.0))
    for prefix in ("Sensor_Inertial_0", "Sensor_Inertial_1"):
        out[f"{prefix}_Vel_B_x"] = vx
        out[f"{prefix}_Vel_B_y"] = vy
        out[f"{prefix}_Vel_B_z"] = vz
        out[f"{prefix}_Omega_B_x"] = wx
        out[f"{prefix}_Omega_B_y"] = wy
        out[f"{prefix}_Omega_B_z"] = wz
        out[f"{prefix}_Acc_B_x"] = ax
        out[f"{prefix}_Acc_B_y"] = ay
        out[f"{prefix}_Acc_B_z"] = 0.0
        out[f"{prefix}_Aplha_B_x"] = 0.0
        out[f"{prefix}_Aplha_B_y"] = 0.0
        out[f"{prefix}_Aplha_B_z"] = 0.0


def fill_ros_msg(msg: Any, fields: Mapping[str, float | int]) -> None:
    for key, val in fields.items():
        if hasattr(msg, key):
            setattr(msg, key, val)


def parse_cmremote(req_type: str, req_msg: str) -> str | None:
    """Map cmrosutils/CMRemote request → plant action ('start'|'stop'|'pause')."""
    blob = f"{req_type or ''} {req_msg or ''}".strip().lower()
    if not blob:
        return None
    if any(k in blob for k in ("simstart", "start", "play", "run")):
        return "start"
    if any(k in blob for k in ("simstop", "stop", "halt", "end")):
        return "stop"
    if "pause" in blob:
        return "pause"
    return None
