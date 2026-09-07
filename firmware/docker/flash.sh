#!/usr/bin/env bash
# Flash the built firmware from the host, not from the container.
#
# Docker Desktop on macOS has no USB passthrough, so the IDF container cannot
# see the port; esptool runs natively through uvx instead. IDF writes
# flash_args with paths relative to the build directory, so this must run from
# inside it, and esptool 5.x renamed the entry point from esptool.py to
# esptool.
#
#   flash.sh /dev/tty.usbmodemXXXX            flashes firmware/build
#   flash.sh /dev/tty.usbmodemXXXX debug      flashes firmware/build-debug
set -euo pipefail

FIRMWARE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PORT="${1:-}"
PROFILE="${2:-release}"
if [ -z "${PORT}" ]; then
  echo "usage: $0 <serial-port> [release|debug]" >&2
  echo "ports: $(ls /dev/tty.usb* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ')" >&2
  exit 2
fi

BUILD_DIR="${FIRMWARE_DIR}/build"
if [ "${PROFILE}" = "debug" ]; then
  BUILD_DIR="${FIRMWARE_DIR}/build-debug"
fi
if [ ! -f "${BUILD_DIR}/flash_args" ]; then
  echo "no build in ${BUILD_DIR}; run firmware/docker/build.sh ${PROFILE} first" >&2
  exit 1
fi

cd "${BUILD_DIR}"
exec uvx --from 'esptool==5.*' esptool \
  --chip esp32s3 -p "${PORT}" -b 460800 write_flash "@flash_args"
