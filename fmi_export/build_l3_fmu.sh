#!/usr/bin/env bash
# Build VDSim L3 (FourteenDOF) as FMI 2.0 CoSimulation FMU.
# Optional: copy hardpoint kinematics CSVs into resources/kinematics/ so that
# the L3 FMU attaches them at runtime.
set -euo pipefail
cd "$(dirname "$0")/.."

REPO="$(pwd)"
OUT_DIR="$REPO/build/fmi_export"
STAGING="$OUT_DIR/staging_l3"
rm -rf "$STAGING"
mkdir -p "$STAGING/binaries/linux64" "$STAGING/resources/configs" "$STAGING/resources/kinematics"

YAMLCPP_LIB=$(find "$REPO/build" -name "libyaml-cpp*.a" | head -1)
SPDLOG_LIB=$(find  "$REPO/build" -name "libspdlog*.a" -not -name "*test*" | head -1)

g++ -O2 -std=c++17 -fPIC -shared -fvisibility=hidden \
    -I "$REPO/core/include" \
    -I "$REPO/build/_deps/eigen-src" \
    -I "$REPO/fmi_export/fmi2" \
    "$REPO/fmi_export/vdsim_l3_fmu.cpp" \
    "$REPO/build/lib/libvdsim_core.a" \
    "$YAMLCPP_LIB" "$SPDLOG_LIB" \
    -Wl,--version-script=<(echo '{ global: fmi2*; local: *; };') \
    -o "$STAGING/binaries/linux64/vdsim_l3.so"

cp "$REPO/fmi_export/modelDescription_l3.xml" "$STAGING/modelDescription.xml"

VEH="${VEH_YAML:-configs/vehicles/sports.yaml}"
TIRE="${TIRE_YAML:-configs/tires/default_pacejka.yaml}"
cp "$REPO/$VEH"  "$STAGING/resources/configs/vehicle.yaml"
cp "$REPO/$TIRE" "$STAGING/resources/configs/tire.yaml"
echo "[L3] configs : $VEH + $TIRE"

# Optional hardpoint kinematics — if user sets FRONT_KIN_CSV / REAR_KIN_CSV
if [ -n "${FRONT_KIN_CSV:-}" ] && [ -f "$REPO/$FRONT_KIN_CSV" ]; then
    cp "$REPO/$FRONT_KIN_CSV" "$STAGING/resources/kinematics/front.csv"
    echo "[L3] +front kinematics: $FRONT_KIN_CSV"
fi
if [ -n "${REAR_KIN_CSV:-}" ] && [ -f "$REPO/$REAR_KIN_CSV" ]; then
    cp "$REPO/$REAR_KIN_CSV" "$STAGING/resources/kinematics/rear.csv"
    echo "[L3] +rear kinematics : $REAR_KIN_CSV"
fi

FMU_OUT="$OUT_DIR/vdsim_l3.fmu"
rm -f "$FMU_OUT"
( cd "$STAGING" && zip -r "$FMU_OUT" . > /dev/null )
echo "[L3] -> $FMU_OUT  ($(du -h "$FMU_OUT" | cut -f1))"
