#!/usr/bin/env bash
# Build the MCU firmware in the espressif/idf:v5.5.5 container.
#
# The image publishes a linux/arm64 manifest, so it runs natively on an M4;
# --platform is passed anyway so an amd64 host and a QEMU fallback both get a
# deterministic answer. Binaries land in firmware/build/ on the host.
#
#   build.sh              release: no console on any interface (A37)
#   build.sh debug        console on USB-Serial/JTAG, caps bit 0 set
#   build.sh release clean, build.sh debug clean   full rebuild
set -euo pipefail

FIRMWARE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_DIR="$(cd "${FIRMWARE_DIR}/.." && pwd)"

IMAGE="${ROVER_IDF_IMAGE:-espressif/idf:v5.5.5}"
PLATFORM="${ROVER_IDF_PLATFORM:-linux/arm64}"

PROFILE="${1:-release}"
ACTION="${2:-build}"

case "${PROFILE}" in
  release) SDKCONFIG_DEFAULTS="sdkconfig.defaults" ;;
  debug)   SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.debug" ;;
  *) echo "usage: $0 [release|debug] [build|clean]" >&2; exit 2 ;;
esac

# A release and a debug build must never share a build directory OR an
# sdkconfig. ESP-IDF writes sdkconfig into the *project* directory, not into
# -B, and values already in it win over SDKCONFIG_DEFAULTS -- so without
# -DSDKCONFIG a debug build followed by a release build reuses the debug
# sdkconfig and ships CONFIG_ESP_CONSOLE_NONE=n: a live console on
# USB-Serial/JTAG in an image the operator believes is release, which is the
# second writer into the motor controller A37 and I-18 exist to forbid.
BUILD_DIR="build"
if [ "${PROFILE}" = "debug" ]; then
  BUILD_DIR="build-debug"
fi
SDKCONFIG="${BUILD_DIR}/sdkconfig"

IDF_CMD="idf.py -B ${BUILD_DIR} -DSDKCONFIG=${SDKCONFIG}"
IDF_CMD="${IDF_CMD} -DSDKCONFIG_DEFAULTS='${SDKCONFIG_DEFAULTS}' build"
if [ "${ACTION}" = "clean" ]; then
  IDF_CMD="rm -rf ${BUILD_DIR} sdkconfig && ${IDF_CMD}"
fi

exec docker run --rm \
  --platform "${PLATFORM}" \
  -v "${REPO_DIR}:/project" \
  -w /project/firmware \
  -e HOME=/tmp \
  -u "$(id -u):$(id -g)" \
  "${IMAGE}" \
  bash -c "${IDF_CMD}"
