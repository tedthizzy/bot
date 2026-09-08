# Upstream

Everything under `firmware/` is a fork of Waveshare's `ugv_base_general`, the
lower-computer firmware for the WAVE ROVER / UGV01 / UGV02 on the General
Driver for Robots board (classic ESP32).

| | |
| --- | --- |
| repository | <https://github.com/effectsmachine/ugv_base_general> (also published as `waveshareteam/ugv_base_general`) |
| commit | `d308df91b333a513f45a238bef9ba9a0a5edf64c` |
| commit date | 2025-11-28 10:54:04 +0800, "Update web_page.h" |
| licence | GPL-3.0 (`LICENSE`, copied verbatim from the upstream root) |

## Licence of this directory

Upstream is GPL-3.0-or-later ("either version 3 of the License, or (at your
option) any later version", per its README). Everything under `firmware/` -- the
vendored sources, the patches in `patches/`, `bot_config.h`, `bot_safety.h`,
the build and flash scripts and this documentation -- is therefore
**GPL-3.0-or-later** as well. The rest of this repository is Apache-2.0 and
never links or imports anything from here; the only thing that crosses the
boundary is JSON text over a serial line (see `THIRD_PARTY.md` at the repo
root).

## What was vendored

Source only, at the commit above:

- `General_Driver/*.ino`, `*.h`, `*.cpp` -> `firmware/General_Driver/`
- `SCServo/` (the bus-servo library the sketch needs, `*.h`, `*.cpp`,
  `library.properties`) -> `firmware/libraries/SCServo/`
- `LICENSE` -> `firmware/LICENSE`

Not vendored: `General_Driver/build/` (prebuilt binaries), `README_footage/`
(photos), `General_Driver/data/` (sample `wifiConfig.json` / `devConfig.json`
holding Waveshare's office Wi-Fi credentials), the upstream `README.md`,
`SCServo/examples/` (14 Arduino IDE example sketches; SCServo has no `src/`, so
the 1.0 library format applies and arduino-cli compiles the library root and
`utility/` only -- the recorded build output
`firmware/build/arduino/libraries/SCServo/` holds exactly `SCS.cpp.o`,
`SCSCL.cpp.o`, `SCSerial.cpp.o` and `SMS_STS.cpp.o` -- and the fork drives no
bus servo), and `SCServo/说明.txt`, a
GBK-encoded one-paragraph note whose file name is not valid UTF-8.

## What was changed

Nothing in the vendored files beyond the patches listed in
`patches/README.md`. The ordered unified diffs reconstruct the fork from upstream;
`patches/README.md` says how to re-derive the tree from them. Four sketch headers
are new: `General_Driver/bot_config.h` (constants),
`General_Driver/bot_safety.h` (sensor flags and banner),
`General_Driver/bot_runtime.h` (bounded serial storage), and
`General_Driver/bot_serial_ctrl.h` (serial dispatch and cooperative waits).
