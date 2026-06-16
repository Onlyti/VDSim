#!/usr/bin/env bash
# Build the MF-Swift parity harnesses against the local CarMaker install.
set -euo pipefail
CMI="${CMI:-/opt/ipg/carmaker/linux64-12.0.1}"
[ -d "$CMI/include" ] || { echo "CarMaker not at $CMI (set CMI=...)"; exit 1; }
for src in mfs_init_probe mfs_grid_eval; do
    gcc -O2 -I "$CMI/include" "$src.c" -L "$CMI/lib" \
        -lmfswift_tire_interface -lm -Wl,-rpath,"$CMI/lib" -o "$src"
    echo "built $src"
done
