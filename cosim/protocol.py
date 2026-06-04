"""Canonical VDSim co-sim wire protocol — Python mirror of cosim_protocol.hpp.

This is the single source of truth for the UDP wire format on the Python side.
Any Python participant in the comms layer (wheel/pedal clients, the viewer
bridge, HIL test harnesses) encodes CMD and decodes STATE through here, so the
bytes always match the C++ server. CRC32 is IEEE 802.3 via zlib, identical to
the C++ `crc32()`.

Layout (little-endian, tightly packed), see cosim_protocol.hpp:
  header (24): magic(I) version(H) msg_type(H) seq(I) _pad(I) timestamp(d)
  CMD payload (48): steer(d) throttle(d) brake(d) gear(i) handbrake(B) _pad(3x)
                    aux_accel(d) aux_speed(d)            -> 76 total incl. crc
  STATE payload (344): x y z roll pitch yaw vx vy vz roll_rate pitch_rate
                    yaw_rate ax ay (14d) wheel_spin[4] steer_applied
                    wheel_radius Fz[4]  (192, v1)
                    rack_torque slip_ratio[4] slip_angle[4] susp[4]
                    m_ax m_ay m_wz m_steer m_gnss_x m_gnss_y  (152, v2)
                                                         -> 372 total incl. crc
  trailing crc (4): crc32 over all preceding bytes
"""
import struct
import zlib

MAGIC = 0x56445331  # "VDS1"
VERSION = 2  # v2: STATE adds rack_torque/slip/susp/measured
MSG_CMD, MSG_STATE = 1, 2
CMD_BYTES, STATE_BYTES = 76, 372

# default ports (server listens CMD, emits STATE)
CMD_PORT, STATE_PORT = 7001, 7002

_HEADER = "<IHHIId"  # magic, version, msg_type, seq, pad, timestamp


def _crc(b):
    return zlib.crc32(b) & 0xFFFFFFFF


def pack_cmd(seq, steer=0.0, throttle=0.0, brake=0.0, gear=1,
             handbrake=0, aux_accel=float("nan"), aux_speed=float("nan"),
             timestamp=0.0):
    body = struct.pack(_HEADER, MAGIC, VERSION, MSG_CMD, seq, 0, timestamp)
    body += struct.pack("<ddd", steer, throttle, brake)
    body += struct.pack("<iB3x", gear, handbrake)
    body += struct.pack("<dd", aux_accel, aux_speed)
    assert len(body) == CMD_BYTES - 4, len(body)
    return body + struct.pack("<I", _crc(body))


def decode_cmd(buf):
    if len(buf) < CMD_BYTES:
        return None
    if _crc(buf[:CMD_BYTES - 4]) != struct.unpack_from("<I", buf, CMD_BYTES - 4)[0]:
        return None
    magic, ver, mtype, seq, _pad, ts = struct.unpack_from(_HEADER, buf, 0)
    if magic != MAGIC or ver != VERSION or mtype != MSG_CMD:
        return None
    steer, throttle, brake = struct.unpack_from("<ddd", buf, 24)
    gear, handbrake = struct.unpack_from("<iB", buf, 48)
    aux_accel, aux_speed = struct.unpack_from("<dd", buf, 56)
    return dict(seq=seq, timestamp=ts, steer=steer, throttle=throttle,
                brake=brake, gear=gear, handbrake=handbrake,
                aux_accel=aux_accel, aux_speed=aux_speed)


_STATE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz",
               "roll_rate", "pitch_rate", "yaw_rate", "ax", "ay")


def decode_state(buf):
    if len(buf) != STATE_BYTES:
        return None
    if _crc(buf[:STATE_BYTES - 4]) != struct.unpack_from("<I", buf, STATE_BYTES - 4)[0]:
        return None
    magic, ver, mtype, seq, _pad, ts = struct.unpack_from(_HEADER, buf, 0)
    if magic != MAGIC or ver != VERSION or mtype != MSG_STATE:
        return None
    vals = struct.unpack_from("<14d", buf, 24)
    out = dict(zip(_STATE_KEYS, vals))
    out["seq"] = seq
    out["timestamp"] = ts
    out["wheel_spin"] = list(struct.unpack_from("<4d", buf, 136))
    out["steer_applied"], out["wheel_radius"] = struct.unpack_from("<dd", buf, 168)
    out["Fz"] = list(struct.unpack_from("<4d", buf, 184))
    out["rack_torque"] = struct.unpack_from("<d", buf, 216)[0]
    out["slip_ratio"] = list(struct.unpack_from("<4d", buf, 224))
    out["slip_angle"] = list(struct.unpack_from("<4d", buf, 256))
    out["susp"] = list(struct.unpack_from("<4d", buf, 288))
    (out["m_ax"], out["m_ay"], out["m_wz"], out["m_steer"],
     out["m_gnss_x"], out["m_gnss_y"]) = struct.unpack_from("<6d", buf, 320)
    return out
