#!/usr/bin/env bash
# Flash the images that build.sh left in firmware/build/ to the General Driver
# board over USB, with esptool run on the host through uvx (Docker Desktop on
# macOS has no USB passthrough).
#
#   ./firmware/flash.sh /dev/tty.usbserial-XXXXXXXX
#
# Offsets, chip and flash parameters are the ones Waveshare's factory flash
# package uses for this board (ESP32-WROOM-32UE, 4 MB, DIO): bootloader 0x1000,
# partition table 0x8000, boot_app0.bin 0xE000, application 0x10000.
#
# The USB CP2102 drives EN and IO0, so no button is held. The Pi header UART is
# the same UART0 without those lines: unplug the Pi's TX/RX (or power the Pi
# down) while a computer is on the USB port, or the two talk over each other.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$HERE/build"
PORT="${1:?usage: flash.sh <serial port>, e.g. /dev/tty.usbserial-XXXXXXXX}"
BAUD="${BAUD:-460800}"

for f in General_Driver.ino.bootloader.bin General_Driver.ino.partitions.bin boot_app0.bin General_Driver.ino.bin; do
  [ -f "$BUILD/$f" ] || { echo "missing $BUILD/$f -- run firmware/build.sh first" >&2; exit 1; }
done

command -v uvx >/dev/null || { echo "uvx not found; install uv (https://docs.astral.sh/uv/)" >&2; exit 1; }

set -x
uvx --from esptool esptool \
  --chip esp32 \
  --port "$PORT" \
  --baud "$BAUD" \
  --before default_reset \
  --after hard_reset \
  write_flash -z \
  --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x1000  "$BUILD/General_Driver.ino.bootloader.bin" \
  0x8000  "$BUILD/General_Driver.ino.partitions.bin" \
  0xe000  "$BUILD/boot_app0.bin" \
  0x10000 "$BUILD/General_Driver.ino.bin"
