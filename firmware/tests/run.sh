#!/usr/bin/env bash
# Native regression checks against production headers; no serial devices or Docker run.
set -euo pipefail
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$TEST_DIR/../arduinojson.sh"
ARDUINO_CACHE="${BOT_ARDUINO_CACHE:-$TEST_DIR/../.cache}"
JSON_ROOT="$ARDUINO_CACHE/user/libraries/ArduinoJson"
JSON_INCLUDE="$JSON_ROOT/src"

json_ready() {
  [ -f "$JSON_INCLUDE/ArduinoJson.h" ] &&
    [ -f "$JSON_ROOT/library.properties" ] &&
    grep -qx "version=$ARDUINOJSON_VERSION" "$JSON_ROOT/library.properties"
}

case "${1:-}" in
  --setup)
    if json_ready; then
      echo "ArduinoJson $ARDUINOJSON_VERSION ready: $JSON_ROOT"
      exit 0
    fi
    if [ -e "$JSON_ROOT" ]; then
      echo "Refusing to overwrite incompatible cache: $JSON_ROOT" >&2
      echo 'Choose an empty BOT_ARDUINO_CACHE or move that library aside.' >&2
      exit 1
    fi
    mkdir -p "$ARDUINO_CACHE/user/libraries"
    NATIVE_SETUP="$(mktemp -d "$ARDUINO_CACHE/.arduinojson.XXXXXX")"
    # Only this invocation's fresh temporary directory is removed.
    trap 'rm -rf -- "$NATIVE_SETUP"' EXIT
    ARCHIVE="$NATIVE_SETUP/ArduinoJson.zip"
    curl --fail --location --silent --show-error --proto '=https' \
      --connect-timeout 15 --max-time 120 --retry 2 \
      "$ARDUINOJSON_URL" -o "$ARCHIVE"
    printf '%s  %s\n' "$ARDUINOJSON_SHA256" "$ARCHIVE" | shasum -a 256 -c -
    unzip -q "$ARCHIVE" -d "$NATIVE_SETUP"
    mv "$NATIVE_SETUP/ArduinoJson-$ARDUINOJSON_VERSION" "$JSON_ROOT"
    json_ready
    echo "ArduinoJson $ARDUINOJSON_VERSION installed: $JSON_ROOT"
    exit 0
    ;;
  '') ;;
  *) echo 'Usage: bash firmware/tests/run.sh [--setup]' >&2; exit 2 ;;
esac

if ! json_ready; then
  echo "ArduinoJson $ARDUINOJSON_VERSION headers missing or mismatched; run bash firmware/tests/run.sh --setup first." >&2
  exit 1
fi
TEST_BUILD="$(mktemp -d "${TMPDIR:-/tmp}/bot-firmware-test.XXXXXX")"
trap 'rm -f "$TEST_BUILD/runtime_test"; rmdir "$TEST_BUILD"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Wno-unused-parameter \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I "$JSON_INCLUDE" "$TEST_DIR/runtime_test.cpp" -o "$TEST_BUILD/runtime_test"
"$TEST_BUILD/runtime_test"
