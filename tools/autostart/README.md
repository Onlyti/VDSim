# Wheel FFB — autostart (runs on the client PC, where the wheel is plugged in)

The force-feedback bridge must run **on the client** (it needs local hardware
access); only telemetry crosses the network to the VDSim server (`--url`). Set it
to autostart so it's always running — the browser viewer then just does cockpit
view + sound, and the wheel gets FFB in the background.

## Helpers
- `tools/wheel_ffb_sdl.py` — **cross-platform** (Windows + Linux) via SDL2 haptic.
  `pip install pysdl2 pysdl2-dll`
- `tools/wheel_ffb.py` — Linux-native via evdev. `pip install evdev` (+ `input` group).

Both are UNTESTED without an FFB wheel — verify and tune `--gain` / axis indices
on your hardware.

## Windows autostart
1. Edit `vdsim-ffb.bat` → set `SERVER` to your VDSim host.
2. `Win+R` → `shell:startup` → drop a shortcut to `vdsim-ffb.bat` there.
   (or `schtasks /create /tn VDSimFFB /tr "<path>\vdsim-ffb.bat" /sc onlogon`)

## Linux autostart (systemd user service)
```sh
mkdir -p ~/.config/systemd/user
cp tools/autostart/vdsim-ffb.service ~/.config/systemd/user/
# edit ExecStart (path + --url) in the copied file
systemctl --user daemon-reload
systemctl --user enable --now vdsim-ffb
```

## Note on the browser
The browser cannot launch native code or do real steering FFB itself (sandbox;
Gamepad API is rumble-only, WebHID can't reliably send constant-force). The
autostarted helper is the practical "automatic" path. If the FFB helper handles
input too, disable the browser's 🎮 Wheel toggle to avoid double-driving.
