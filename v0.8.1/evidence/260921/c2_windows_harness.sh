#!/usr/bin/env bash
# C-2 windows harness (pre-flight C, 2026-09-21).
# Declared console script name is vdsim-quickstart (pyproject [project.scripts]).
# The run logged in c2_windows.txt used a mistyped "quickstart.exe" and fell back to
# "python -m quickstart"; this is the corrected version. Run under Git Bash on win_lab.
set -e
BASE=/c/Users/jiwon/AppData/Local/Temp/q19b2win
WORK=$BASE/work
rm -rf "$WORK" && mkdir -p "$WORK/demo"
cp "$BASE/demo_grip_loss.py" "$WORK/demo/"
echo "== python"
py -3.12 --version
echo "== venv"
py -3.12 -m venv "$BASE/venv"
VPY="$BASE/venv/Scripts/python.exe"
"$VPY" -m pip install -q --upgrade pip
echo "== install wheel"
"$VPY" -m pip install -q "$BASE"/vdsim-0.7.0.dev134-cp312-cp312-win_amd64.whl matplotlib pillow
echo "== installed:"
"$VPY" -m pip show vdsim | head -3
echo "== source tree check (no VDSim source tree under work)"
ls "$WORK"
echo "src markers: $(ls "$WORK" | grep -E '^(python|core|CMakeLists.txt)$' || echo '[]')"
echo "== import smoke (all 13 shipped modules)"
cd "$WORK"
"$VPY" -c "
import importlib
mods='vdsim vdsim_lab vdsim_plant vdsim_trace vdsim_render vdsim_render3d vdsim_preset protocol opendrive rd5_route quickstart catalog vdsim_rl'.split()
for m in mods:
    mod=importlib.import_module(m)
    print(' ', m, getattr(mod,'__file__','builtin'))
print('all %d shipped modules import from the installed wheel' % len(mods))
"
echo "== 06 quickstart (console entry point, cwd=$WORK)"
"$BASE/venv/Scripts/vdsim-quickstart.exe"
ls -la "$WORK"/run.csv "$WORK"/run.png
echo "quickstart csv rows $(wc -l < "$WORK"/run.csv) cols $(head -1 "$WORK"/run.csv | tr ',' '\n' | wc -l)"
echo "== 07 demo (script copied outside any source tree)"
cd "$WORK/demo"
"$VPY" demo_grip_loss.py --trace demo.vdtrace
ls -la "$WORK/demo"
echo "== C2 WINDOWS OK"
