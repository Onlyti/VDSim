#!/usr/bin/env bash
# Build VDSim L2 as an FMI 2.0 Co-Simulation FMU.
#
# Output: build/fmi_export/vdsim_l2.fmu
#
# Inside the .fmu:
#   modelDescription.xml
#   binaries/linux64/vdsim_l2.so          (compiled shared library)
#   resources/configs/{vehicle,tire,solver}.yaml
set -euo pipefail
cd "$(dirname "$0")/.."

REPO="$(pwd)"
OUT_DIR="$REPO/build/fmi_export"
STAGING="$OUT_DIR/staging"
rm -rf "$STAGING"
mkdir -p "$STAGING/binaries/linux64" "$STAGING/resources/configs"

# 1. Compile shared library — link against libvdsim_core.a from existing build
PLATFORM=linux64
SO_NAME=vdsim_l2.so

YAMLCPP_LIB=$(find "$REPO/build" -name "libyaml-cpp*.a" | head -1)
SPDLOG_LIB=$(find  "$REPO/build" -name "libspdlog*.a" -not -name "*test*" | head -1)
if [ -z "$YAMLCPP_LIB" ] || [ -z "$SPDLOG_LIB" ]; then
    echo "ERROR: could not find static libs"; exit 1
fi
echo "[build] yaml-cpp = $YAMLCPP_LIB"
echo "[build] spdlog   = $SPDLOG_LIB"

g++ -O2 -std=c++17 -fPIC -shared -fvisibility=hidden \
    -I "$REPO/core/include" \
    -I "$REPO/build/_deps/eigen-src" \
    -I "$REPO/fmi_export/fmi2" \
    "$REPO/fmi_export/vdsim_l2_fmu.cpp" \
    "$REPO/build/lib/libvdsim_core.a" \
    "$YAMLCPP_LIB" \
    "$SPDLOG_LIB" \
    -Wl,--version-script=<(echo '{ global: fmi2*; local: *; };') \
    -o "$STAGING/binaries/$PLATFORM/$SO_NAME"

echo "[build] compiled $SO_NAME"

# 2. modelDescription.xml
cp "$REPO/fmi_export/modelDescription.xml" "$STAGING/"

# 3. Resources — vehicle / tire / solver YAML.  The FMU expects these under
#    resources/configs/{vehicle,tire,solver}.yaml regardless of source name.
#    Default: sports.yaml + default_pacejka.yaml.
VEH="${VEH_YAML:-configs/vehicles/sports.yaml}"
TIRE="${TIRE_YAML:-configs/tires/default_pacejka.yaml}"
cp "$REPO/$VEH"  "$STAGING/resources/configs/vehicle.yaml"
cp "$REPO/$TIRE" "$STAGING/resources/configs/tire.yaml"
echo "[build] resources: $VEH + $TIRE"

# 4. ZIP into .fmu
FMU_OUT="$OUT_DIR/vdsim_l2.fmu"
rm -f "$FMU_OUT"
( cd "$STAGING" && zip -r "$FMU_OUT" . > /dev/null )
echo "[build] -> $FMU_OUT  ($(du -h "$FMU_OUT" | cut -f1))"
