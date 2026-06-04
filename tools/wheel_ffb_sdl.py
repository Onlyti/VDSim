#!/usr/bin/env python3
"""Cross-platform (Windows + Linux) racing-wheel force-feedback bridge via SDL2.

Same idea as wheel_ffb.py (Linux evdev) but uses SDL2's haptic API, which works
on Windows (DirectInput/XInput) and Linux. Run it on the CLIENT PC where the
wheel is plugged in; it talks to the VDSim server over HTTP.

  - read wheel axes -> POST steer/throttle/brake to <url>/api/manual
  - GET <url>/api/state -> rack_torque -> SDL_HAPTIC_CONSTANT level (self-centering,
    understeer lightening, grip load) + device-side autocenter spring (survives
    network hiccups).

STATUS: UNTESTED without an FFB wheel (no hardware here). Follows the standard
SDL2 haptic API; verify and tune --gain / axis indices on your wheel.

Talks to VDSim over UDP (one datagram round-trip per tick: command out,
rack_torque back) — lower latency than HTTP, important for FFB. The VDSim GUI
opens this UDP port at http-port + 1 (default 8091).

Install:  pip install pysdl2 pysdl2-dll        (pysdl2-dll bundles SDL2 on Windows)
Usage:    python wheel_ffb_sdl.py --server SERVER --udp-port 8091 [--gain 0.8]
          [--autocenter 0.15] [--steer-axis 0 --throttle-axis 2 --brake-axis 3]
"""
import argparse
import ctypes
import json
import socket
import sys
import time

try:
    import sdl2
except ImportError:
    sys.exit("needs PySDL2:  pip install pysdl2 pysdl2-dll")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="localhost", help="VDSim host")
    ap.add_argument("--udp-port", type=int, default=8091, help="VDSim UDP control/FFB port (http port + 1)")
    ap.add_argument("--gain", type=float, default=0.8)
    ap.add_argument("--autocenter", type=float, default=0.15)   # 0..1 device spring
    ap.add_argument("--max-steer", type=float, default=0.5)     # rad at full lock
    ap.add_argument("--steer-axis", type=int, default=0)
    ap.add_argument("--throttle-axis", type=int, default=2)
    ap.add_argument("--brake-axis", type=int, default=3)
    ap.add_argument("--rate", type=float, default=100.0)
    a = ap.parse_args()

    if sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_HAPTIC) != 0:
        sys.exit("SDL_Init failed: " + sdl2.SDL_GetError().decode())
    if sdl2.SDL_NumJoysticks() < 1:
        sys.exit("no joystick/wheel found")
    joy = sdl2.SDL_JoystickOpen(0)
    name = sdl2.SDL_JoystickName(joy)
    print("[ffb] wheel:", name.decode() if name else "?", "axes:", sdl2.SDL_JoystickNumAxes(joy))

    hap = sdl2.SDL_HapticOpenFromJoystick(joy)
    if not hap:
        sys.exit("wheel has no SDL haptic (FFB) support: " + sdl2.SDL_GetError().decode())
    if not (sdl2.SDL_HapticQuery(hap) & sdl2.SDL_HAPTIC_CONSTANT):
        print("[ffb] warning: device reports no CONSTANT force; trying anyway")
    sdl2.SDL_HapticSetAutocenter(hap, int(max(0, min(100, a.autocenter * 100))))

    eff = sdl2.SDL_HapticEffect()
    eff.type = sdl2.SDL_HAPTIC_CONSTANT
    eff.constant.type = sdl2.SDL_HAPTIC_CONSTANT
    eff.constant.direction.type = sdl2.SDL_HAPTIC_CARTESIAN
    eff.constant.direction.dir[0] = 1
    eff.constant.length = sdl2.SDL_HAPTIC_INFINITY
    eff.constant.level = 0
    eid = sdl2.SDL_HapticNewEffect(hap, ctypes.byref(eff))
    if eid < 0:
        sys.exit("SDL_HapticNewEffect failed: " + sdl2.SDL_GetError().decode())
    sdl2.SDL_HapticRunEffect(hap, eid, 1)

    def axis(i):  # -1..1
        return sdl2.SDL_JoystickGetAxis(joy, i) / 32767.0

    # one UDP round-trip per tick: command out, telemetry (rack_torque) back
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.05)
    dst = (a.server, a.udp_port)
    print(f"[ffb] udp -> {a.server}:{a.udp_port}")
    dt = 1.0 / a.rate
    try:
        while True:
            sdl2.SDL_JoystickUpdate()
            steer = max(-1.0, min(1.0, axis(a.steer_axis)))
            # pedals usually rest near +1, full near -1 -> (1-v)/2 in [0,1]
            thr = max(0.0, min(1.0, (1 - axis(a.throttle_axis)) / 2))
            brk = max(0.0, min(1.0, (1 - axis(a.brake_axis)) / 2))
            rack = 0.0
            try:
                sock.sendto(json.dumps({"steer": steer * a.max_steer,
                                        "throttle": thr, "brake": brk}).encode(), dst)
                rack = float(json.loads(sock.recv(1024)).get("rack_torque", 0.0))
            except OSError:
                pass
            lvl = int(max(-32767, min(32767, -rack * a.gain * 300.0)))   # oppose the turn
            eff.constant.level = lvl
            sdl2.SDL_HapticUpdateEffect(hap, eid, ctypes.byref(eff))
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        sdl2.SDL_HapticClose(hap)
        sdl2.SDL_JoystickClose(joy)
        sdl2.SDL_Quit()


if __name__ == "__main__":
    main()
