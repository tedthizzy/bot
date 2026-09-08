#!/usr/bin/env bash
# Compile firmware/General_Driver with arduino-cli inside Docker.
#
#   ./firmware/build.sh            # build; installs the core and libraries on first run
#   ./firmware/build.sh --setup    # only install the core and the libraries
#   FQBN=esp32:esp32:esp32:FlashMode=dio ./firmware/build.sh   # default partition table
#
# Output: firmware/build/General_Driver.ino.bin, .bootloader.bin, .partitions.bin
#         and boot_app0.bin — everything flash.sh needs.
# Cache:  firmware/.cache/ (gitignored) holds the esp32 core, the toolchains and
#         the libraries, about 1.5 GB after the first run.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/arduinojson.sh"
IMAGE="${BOT_ARDUINO_IMAGE:-bot-arduino-cli:1.5.1}"      # built from firmware/Dockerfile
CACHE="${BOT_ARDUINO_CACHE:-$HERE/.cache}"
# ESP32 Dev Module, DIO like the factory image, Huge APP partition table (3 MB
# app at 0x10000, 0xE0000 of SPIFFS/LittleFS at 0x310000) like Waveshare's own
# GitHub build. The fork also fits the default 1.2 MB slot; see README.md.
FQBN="${FQBN:-esp32:esp32:esp32:PartitionScheme=huge_app,FlashMode=dio}"

# 2.0.17 is the last 2.x core. The code uses the 2.x LEDC API (ledcSetup /
# ledcAttachPin), which 3.x removed. Do not bump the major version.
ESP32_CORE_VERSION="2.0.17"
ESP32_INDEX_URL="https://espressif.github.io/arduino-esp32/package_esp32_index.json"

# Every library is pinned to the version the reference build used (README.md).
# Two pins matter: ArduinoJson stays on the last 6.x because the sketch uses
# the v6 API (StaticJsonDocument, DynamicJsonDocument, containsKey), and
# INA219_WE stays on 1.3.8 because 1.4.0 renamed its enums (BIT_MODE_9 ->
# INA219_BIT_MODE_9) and the vendored battery_ctrl.h uses the old names.
LIBS=(
  "ArduinoJson@$ARDUINOJSON_VERSION"
  "Adafruit SSD1306@2.5.17"
  "Adafruit GFX Library@1.12.6"
  "Adafruit BusIO@1.17.4"
  "INA219_WE@1.3.8"
  "ESP32Encoder@5.0.0"
  "PID_v2@2.0.1"
  "SimpleKalmanFilter@0.2.0"
  "Adafruit ICM20X@2.0.7"
  "Adafruit Unified Sensor@1.1.15"
  "VL53L1X@1.3.1"
)

mkdir -p "$CACHE/data" "$CACHE/downloads" "$CACHE/user" "$CACHE/home" "$HERE/build"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "== building $IMAGE from $HERE/Dockerfile"
  # This toolchain image needs only its Dockerfile, not the sketch or cache.
  docker build -t "$IMAGE" - < "$HERE/Dockerfile"
fi

run() {
  docker run --rm \
    -u "$(id -u):$(id -g)" \
    -v "$HERE:/work" \
    -v "$CACHE:/cache" \
    -w /work \
    "$IMAGE" "$@"
}

setup() {
  echo "== installing esp32:esp32@$ESP32_CORE_VERSION"
  run core update-index --additional-urls "$ESP32_INDEX_URL"
  run core install "esp32:esp32@$ESP32_CORE_VERSION" --additional-urls "$ESP32_INDEX_URL"
  echo "== installing libraries"
  run lib install "${LIBS[@]}"
  run lib list
}

if ! run core list 2>/dev/null | grep -Eq "^esp32:esp32[[:space:]]+$ESP32_CORE_VERSION"; then
  setup
elif [ "${1:-}" = "--setup" ]; then
  setup
fi
[ "${1:-}" = "--setup" ] && exit 0

echo "== compiling $FQBN"
run compile \
  --fqbn "$FQBN" \
  --libraries /work/libraries \
  --build-path /work/build/arduino \
  --output-dir /work/build \
  --warnings default \
  /work/General_Driver

# boot_app0.bin selects ota_0 as the boot partition; the Arduino IDE flashes it
# at 0xe000 and so does flash.sh.
cp "$CACHE/data/packages/esp32/hardware/esp32/$ESP32_CORE_VERSION/tools/partitions/boot_app0.bin" "$HERE/build/boot_app0.bin"

echo "== done: $HERE/build"
ls -l "$HERE/build"/*.bin
