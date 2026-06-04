#!/usr/bin/env python3
"""Native force-feedback bridge for a racing wheel (Linux evdev).

WHY NATIVE: the browser Gamepad API only does rumble (dual/trigger), not the
directional constant-force a steering wheel needs, so true FFB cannot live in the
web viewer. This standalone process reads the wheel and writes real FFB effects.

WHAT IT DOES (loop ~100 Hz):
  - read wheel axes -> POST steer/throttle/brake to the running VDSim GUI
    (`/api/manual`), so the wheel drives the plant;
  - GET `/api/state` -> use `rack_torque` (VDSim's tire aligning moment / steering
    ratio — already physically computed) as the FF_CONSTANT level, so the wheel
    self-centers, lightens under understeer, and loads up with grip;
  - add an FF_SPRING autocenter and a short rumble on big suspension events (kerbs).

STATUS: UNTESTED without hardware. It follows the standard evdev FF API but I have
no FFB wheel to verify on — run it on your wheel and tune --gain / axis mapping.

Requires: python-evdev (`pip install evdev`), a wheel exposing EV_FF, and r/w on
its /dev/input/eventX (add your user to the `input` group or run with sudo).

Usage:
    python3 tools/wheel_ffb.py --url http://localhost:8100 [--device /dev/input/eventN]
                               [--gain 0.8] [--autocenter 0.15] [--max-steer 0.5]
"""
import argparse
import json
import sys
import time
import urllib.request

try:
    from evdev import InputDevice, ecodes, ff, list_devices
except ImportError:
    sys.exit("needs python-evdev:  pip install evdev")


def http_post(url, obj):
    req = urllib.request.Request(url, data=json.dumps(obj).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=0.2).read()
    except Exception:
        pass


def http_get(url):
    try:
        return json.loads(urllib.request.urlopen(url, timeout=0.2).read())
    except Exception:
        return {}


def find_wheel():
    for path in list_devices():
        d = InputDevice(path)
        caps = d.capabilities()
        if ecodes.EV_FF in caps and ecodes.EV_ABS in caps:
            return d
    return None


def norm_axis(dev, code):
    """current axis value normalized to [-1, 1] (or [0,1] for one-sided pedals)."""
    try:
        ai = dev.absinfo(code)
    except Exception:
        return None
    span = (ai.max - ai.min) or 1
    return (ai.value - ai.min) / span * 2.0 - 1.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8100")
    ap.add_argument("--device", default="")
    ap.add_argument("--gain", type=float, default=0.8)          # rack_torque -> FFB scale
    ap.add_argument("--autocenter", type=float, default=0.15)   # spring strength 0..1
    ap.add_argument("--max-steer", type=float, default=0.5)     # rad at full lock
    ap.add_argument("--rate", type=float, default=100.0)
    a = ap.parse_args()

    dev = InputDevice(a.device) if a.device else find_wheel()
    if dev is None:
        sys.exit("no FFB wheel found (need a device with EV_FF + EV_ABS)")
    print(f"[ffb] wheel: {dev.name} @ {dev.path}")

    # constant-force effect (we re-upload its level each tick)
    const = ff.Effect(ecodes.FF_CONSTANT, -1, 0,
                      ff.Trigger(0, 0), ff.Replay(0xFFFF, 0),
                      ff.EffectType(ff_constant_effect=ff.Constant(
                          level=0, envelope=ff.Envelope(0, 0, 0, 0))))
    cid = dev.upload_effect(const)
    dev.write(ecodes.EV_FF, cid, 1)

    # optional spring autocenter (device-side; constant level)
    if a.autocenter > 0 and hasattr(ff, "Condition"):
        try:
            sat = int(0x7FFF * min(1.0, a.autocenter))
            cond = ff.Effect(ecodes.FF_SPRING, -1, 0, ff.Trigger(0, 0), ff.Replay(0xFFFF, 0),
                             ff.EffectType(ff_condition_effect=[ff.Condition(
                                 right_saturation=sat, left_saturation=sat,
                                 right_coeff=sat, left_coeff=sat, deadband=0, center=0)]))
            sid = dev.upload_effect(cond)
            dev.write(ecodes.EV_FF, sid, 1)
        except Exception as e:
            print("[ffb] spring autocenter unavailable:", e)

    dt = 1.0 / a.rate
    prev_susp = 0.0
    while True:
        # --- drain pending input events (updates dev.absinfo current values) ---
        try:
            while dev.read_one() is not None:
                pass
        except Exception:
            pass
        steer = norm_axis(dev, ecodes.ABS_X) or 0.0
        # pedals: one-sided axes rest near min -> map (-1..1)->(0..1); device-specific
        thr_raw = norm_axis(dev, ecodes.ABS_Z)
        brk_raw = norm_axis(dev, ecodes.ABS_RZ)
        throttle = max(0.0, (thr_raw + 1) / 2) if thr_raw is not None else 0.0
        brake = max(0.0, (brk_raw + 1) / 2) if brk_raw is not None else 0.0
        http_post(a.url + "/api/manual",
                  {"steer": steer * a.max_steer, "throttle": throttle, "brake": brake})

        # --- FFB from VDSim aligning torque ---
        st = http_get(a.url + "/api/state")
        rack = float(st.get("rack_torque", 0.0))
        level = int(max(-32767, min(32767, -rack * a.gain * 300.0)))  # sign: oppose turn
        const.u.ff_constant_effect.level = level
        try:
            dev.upload_effect(const)   # same id -> updates level
        except Exception:
            pass
        # kerb bump: short rumble on a fast suspension change
        susp = st.get("susp") or [0, 0, 0, 0]
        jolt = abs(sum(susp) - prev_susp); prev_susp = sum(susp)
        if jolt > 0.02:
            try:
                rum = ff.Effect(ecodes.FF_RUMBLE, -1, 0, ff.Trigger(0, 0), ff.Replay(80, 0),
                                ff.EffectType(ff_rumble_effect=ff.Rumble(
                                    strong_magnitude=int(min(0xFFFF, jolt * 4e5)), weak_magnitude=0)))
                rid = dev.upload_effect(rum); dev.write(ecodes.EV_FF, rid, 1)
            except Exception:
                pass
        time.sleep(dt)


if __name__ == "__main__":
    main()
