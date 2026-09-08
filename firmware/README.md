# Firmware: Waveshare `ugv_base_general` with safety patches

This directory is a fork of Waveshare's `ugv_base_general` Arduino firmware for
the WAVE ROVER's General Driver for Robots board (ESP32-WROOM-32UE), plus eight
patches that make it safe to drive from an unattended host. The result
announces itself as `bot-wr-1` and implements `docs/protocol.md`; the reasoning
is `docs/adr/0013-wave-rover-open-loop.md`.

**Status: compiled, not flashed.** The build below has been run and links; it
has not yet been written to a board, so nothing in the "on the hardware"
sections has been observed. Treat those as the expected outcome, not a report.

Licence: GPL-3.0-or-later, like upstream (`UPSTREAM.md`, `LICENSE`). Nothing
outside `firmware/` links against this code; the Pi talks to it over a serial
line.

## Layout

| path | what |
| --- | --- |
| `General_Driver/` | the sketch: upstream sources plus `bot_config.h` (every added constant) and `bot_safety.h` (stop flags, ToF, bumper, battery latch, banner) |
| `libraries/SCServo/` | Waveshare's bus-servo library, vendored because it is not in the Library Manager |
| `patches/` | the eight patches as unified diffs, in apply order, with `patches/README.md` |
| `build.sh`, `Dockerfile` | reproducible compile in Docker (arduino-cli, esp32 core 2.0.17) |
| `flash.sh` | write the images over USB with esptool on the host |
| `UPSTREAM.md` | upstream URL, commit, date, licence, what was vendored |
| `build/`, `.cache/` | outputs and the arduino-cli cache; both gitignored |

## The patches

1. `bot_config.h`, one file for the whole envelope.
2. Heartbeat 300 ms; `T:136` can only lower it.
3. Wi-Fi, HTTP page and ESP-NOW compiled out; serial is the only control path.
4. No boot mission at power-on.
5. No encoder pins claimed (the rover has none); GPIO 27 is free for the bumper.
6. Stop flags from a front VL53L1X, a bumper and the INA219 pack voltage.
7. Power cap 0.30 on every path, forward block, low-battery refusal, coast flag.
8. Feedback fields `hb st tf bp cc`, `L`/`R` in host units, `T:1006` banner.

What each one changes and which safety property it serves: `patches/README.md`.

## Compile-time switches (`General_Driver/bot_config.h`)

| constant | default | meaning |
| --- | --- | --- |
| `BOT_HEARTBEAT_MS` | 300 | zero the motors after this long without a speed command |
| `BOT_POWER_CAP` | 0.30f | cap in Waveshare units (full scale 0.5); 60 percent duty |
| `BOT_WIFI_ENABLED`, `BOT_ESPNOW_ENABLED` | 0, 0 | radios compiled out |
| `BOT_BOOT_MISSION` | 0 | do not replay the flash "boot" mission |
| `BOT_ENCODERS` | 0 | do not attach GPIO 34/35/16/27 to the pulse counter |
| `BOT_TOF_ENABLED`, `BOT_TOF_STOP_MM`, `BOT_TOF_ADDR` | 1, 250, 0x29 | VL53L1X on the OLED's I2C bus; forward blocked below 250 mm |
| `BOT_TOF_REQUIRED` | 0 | 0: a missing or silent sensor (`tf` = -1) does not block forward, so the rover runs before the sensor arrives. **Set to 1 once the sensor is fitted**: then `tf` = -1 blocks forward and a missing sensor is never treated as clear. |
| `BOT_BUMPER_ENABLED`, `BOT_BUMPER_PIN` | 0, 27 | INPUT_PULLUP, active low, 20 ms debounce; off until the wiring is confirmed, `bp` still reported |
| `BOT_LOWBAT_V`, `BOT_LOWBAT_RECOVER_V` | 9.9f, 10.2f | below 9.9 V for 10 s refuses all motion until above 10.2 V for 30 s |

## Build

Needs Docker (arm64 or amd64) and about 2.5 GB of disk for the core, the
toolchains and the libraries, cached in `firmware/.cache/`. No Arduino IDE.

```sh
./firmware/build.sh
```

The first run builds a small Debian image with arduino-cli 1.5.1
(`Dockerfile`), installs `esp32:esp32@2.0.17` and the pinned libraries, and
compiles. Later runs only compile. The exact compile the script runs is

```sh
arduino-cli compile \
  --fqbn esp32:esp32:esp32:PartitionScheme=huge_app,FlashMode=dio \
  --libraries /work/libraries \
  --build-path /work/build/arduino --output-dir /work/build \
  --warnings default /work/General_Driver
```

with `/work` = `firmware/`. Output in `firmware/build/`:
`General_Driver.ino.bin`, `General_Driver.ino.bootloader.bin`,
`General_Driver.ino.partitions.bin`, `boot_app0.bin` (copied from the core),
plus the `.elf` and `.map`.

Reference build (esp32 2.0.17, huge_app, DIO):

```text
Sketch uses 885001 bytes (28%) of program storage space. Maximum is 3145728 bytes.
Global variables use 47760 bytes (14%) of dynamic memory, leaving 279920 bytes for local variables. Maximum is 327680 bytes.
```

With the default 4 MB partition table (`FQBN=esp32:esp32:esp32:FlashMode=dio
./firmware/build.sh`) the same sources give:

```text
Sketch uses 885001 bytes (67%) of program storage space. Maximum is 1310720 bytes.
Global variables use 47760 bytes (14%) of dynamic memory, leaving 279920 bytes for local variables. Maximum is 327680 bytes.
```

For comparison, the unpatched upstream sources with the same core, libraries
and default table compile to 992021 bytes (75%) of flash and 49756 bytes of
RAM; the fork is 107 KB smaller because the web page and the Wi-Fi/HTTP paths
are compiled out.

Why `huge_app` and DIO: Waveshare's own GitHub build of this sketch uses the
Huge APP table (3 MB app at 0x10000, 0xE0000 of SPIFFS/LittleFS at 0x310000)
and the module is a 4 MB DIO part; the factory flash package uses the default
table. The fork fits either. `build.sh` uses huge_app so that flipping
`BOT_WIFI_ENABLED` back on (which pulls the web page and Wi-Fi stack in) cannot
run the app slot out. Note that LittleFS lives at a different offset under the
two tables, so anything the stock firmware stored in flash (missions,
`wifiConfig.json`) is not visible after a table change; the fork does not
read it anyway.

Why esp32 core 2.0.17: the code uses the 2.x LEDC API (`ledcSetup`,
`ledcAttachPin`), which 3.x removed. Waveshare documents 2.0.11; 2.0.17 is the
last 2.x and has the same API.

Library pins (`build.sh`): ArduinoJson 6.21.5 (the sketch uses the v6 API),
INA219_WE 1.3.8 (1.4.0 renamed its enums, `BIT_MODE_9` became
`INA219_BIT_MODE_9`, and upstream `battery_ctrl.h` uses the old names -- pinning
keeps the vendored file untouched), Adafruit SSD1306 2.5.17, Adafruit GFX
1.12.6, Adafruit BusIO 1.17.4, ESP32Encoder 5.0.0, PID_v2 2.0.1,
SimpleKalmanFilter 0.2.0, Adafruit ICM20X 2.0.7, Adafruit Unified Sensor
1.1.15, VL53L1X 1.3.1 (Pololu). SCServo comes from `libraries/`.

The only warnings the build prints come from upstream `RoArm-M2_module.h`
(`control reaches end of non-void function`) and are unchanged by the patches.

## Flash from a Mac

1. Build first (`firmware/build/*.bin` must exist).
2. Power the rover off, **unplug the Pi's TX/RX from the board's UART header**
   (or power the Pi down): the Pi header is the same UART0 as the USB bridge,
   and the two would talk over each other. The Pi cannot reset the board; the
   USB bridge can.
3. Plug the board's USB-C into the Mac. The bridge is a CP2102 and appears as
   `/dev/tty.usbserial-XXXXXXXX` (macOS has the driver built in; if nothing
   appears, `ls /dev/tty.*` before and after plugging in tells you the name).
4. Hold no buttons. The CP2102's DTR/RTS lines drive EN and IO0, so esptool's
   default reset sequence enters the bootloader by itself.

```sh
./firmware/flash.sh /dev/tty.usbserial-XXXXXXXX
```

`flash.sh` runs `uvx --from esptool esptool --chip esp32 --port <port> --baud
460800 write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB` with
the bootloader at 0x1000, the partition table at 0x8000, `boot_app0.bin` at
0xE000 and the application at 0x10000 -- the offsets Waveshare's factory tool
uses. `BAUD=921600` also works on this board. esptool identifies the chip
(`Chip is ESP32-D0WD...`), prints one `Hash of data verified.` per image (four)
and `Hard resetting via RTS pin...`, and the board reboots into the new
firmware.

### What a successful boot looks like

The OLED (128x32, four lines) ends up showing:

```text
WAVE ROVER bot-wr-1
radio off ToF:none        <- "ToF:ok" once the VL53L1X answers at 0x29
MAC:XX:XX:XX:XX:XX:XX
UGV started               <- becomes "V:12.34" (pack volts) after ten seconds
```

Stock shows `AP:UGV` / `ST: OFF` on the first two lines instead; if you see
those, the stock firmware is still running.

On the serial line at 115200 the boot prints the stock text lines
(`Initialize LittleFS...`, `WiFi off (bot).`, `ToF VL53L1X: not found, tf =
-1.`, `Boot mission disabled (bot).`, the MAC) and then, as the last line of
`setup()`, the banner:

```json
{"T":1006,"fw":"bot-wr-1","hb_ms":300,"cap":0.3,"proto":1}
```

## Verify by hand with a serial terminal

Wheels off the ground. Battery pack switched on (see the note below about USB
only). Each command is one JSON object followed by a newline; `pyserial-miniterm`
sends CR LF on Enter, which the firmware accepts.

```sh
uvx --from pyserial pyserial-miniterm /dev/tty.usbserial-XXXXXXXX 115200
```

1. `{"T":143,"cmd":0}` -- stop echoing commands. `{"T":605,"cmd":0}` -- stop
   the debug prints. (Stock echoes every accepted command; you will see each
   line you type come back once until echo is off.)
2. `{"T":1007}` -- the banner comes back:
   `{"T":1006,"fw":"bot-wr-1","hb_ms":300,"cap":0.3,"proto":1}`.
   Stock firmware prints nothing.
3. `{"T":131,"cmd":1}` then `{"T":142,"cmd":200}` -- feedback at 5 Hz (the host
   uses 50 for 20 Hz). Lines look like
   `{"T":1001,"L":0,"R":0,"r":...,"p":...,"y":...,"temp":...,"v":11.9,"hb":0,"st":1,"tf":-1,"bp":0,"cc":0}`.
   `hb` is 0 and `st` is 1 because no speed command has arrived since boot.
   `tf` is -1 until a VL53L1X is fitted.
4. `{"T":1,"L":0.9,"R":0.9}` -- the wheels turn forward at 60 percent duty and
   the next feedback line shows `"L":0.3,"R":0.3,"hb":1,"st":0` with `cc`
   up by 2 (both sides were clamped). The requested 0.9 never reaches the
   motors.
5. Stop typing. Within 300 ms the wheels stop and feedback shows
   `"L":0,"R":0,"hb":0,"st":1`. At 5 Hz you see this on the second line after
   your command at the latest.
6. `{"T":136,"cmd":3000}` then `{"T":1,"L":0.1,"R":0.1}` and stop: the wheels
   still stop within 300 ms -- the longer heartbeat was ignored.
   `{"T":136,"cmd":100}` is accepted and stops them within 100 ms.
7. `{"T":11,"L":255,"R":255}` -- raw PWM is capped too: `L` and `R` read 0.3.
8. `{"T":1,"L":0.2,"R":0.2}` then `{"T":115}` -- coast: the motors freewheel
   and `st` shows 16 until the next `T:1`.
9. Hold a hand 10 cm in front of the VL53L1X, if fitted: `tf` drops below 250,
   `st` shows 2, a forward `T:1` is applied as `L:0,R:0`, and
   `{"T":1,"L":-0.2,"R":-0.2}` (reverse) or `{"T":1,"L":0.2,"R":-0.2}`
   (rotation) still moves.

Note on USB-only power: with the pack switched off the INA219 reads about 0 V,
so ten seconds after boot `st` gains bit 8 and every speed command is applied
as `L:0,R:0` (`cc` still counts the clamps). That is the low-battery refusal
doing its job, not a fault; switch the pack on for steps 4 to 9. The motors
have no power without the pack anyway.

## What has not been verified

- The firmware has been compiled (sizes above) but not flashed; none of the
  OLED, serial or motor behaviour above has been observed on a board.
- VL53L1X behaviour on this bus (shared with the OLED, INA219 and IMU at the
  stock 100 kHz) and the range-status mapping in `bot_safety.h` are untested on
  hardware. `BOT_TOF_REQUIRED` stays 0 until they are.
- The `/dev/tty.usbserial-*` name is the CP2102's usual one on macOS 12 and
  later; check with `ls /dev/tty.*`.
- `tests/unit/test_caps_match.py` (mentioned in `packages/rover_contracts/skills.py`)
  did not exist when this was written; `bot_config.h` keeps every define on one
  line with a plain literal so a regex test can read `BOT_POWER_CAP` and
  `BOT_HEARTBEAT_MS`.
