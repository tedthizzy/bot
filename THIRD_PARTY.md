# Third-party notices

First-party code in this repository is **Apache-2.0** (`LICENSE`). Everything
below is somebody else's work, with the terms it comes under and — where the
boundary matters — why it sits where it does.

Verify a line before relying on it. Licences change between releases; the
versions here are the ones `uv.lock` pins.

## Runtime dependencies

Four base packages, resolving on macOS **and** aarch64 Linux, and four speech
extras that are the Pi's. Every one permits Apache-2.0 distribution.

| package | licence | where |
|---|---|---|
| `pydantic` (+ `pydantic-core`) | MIT | base |
| `pyserial-asyncio-fast` (+ `pyserial`) | BSD-3-Clause | base |
| `openai` | Apache-2.0 | base |
| `aiohttp` (+ `yarl`, `multidict`, `frozenlist`, `aiosignal`, `propcache`, `aiohappyeyeballs`) | Apache-2.0 | base |
| `numpy` | BSD-3-Clause | `[speech]` |
| `sounddevice` | MIT | `[speech]` |
| `sherpa-onnx` | Apache-2.0 | `[speech]` |
| `pyopen-wakeword` | Apache-2.0 | `[speech]`, `sys_platform == "linux"` |

`numpy` sits in the speech extra rather than the base set for a technical
reason, not a licensing one: `rover-cam` runs in a `--system-site-packages`
virtual environment, and a base-set numpy would shadow apt's `python3-numpy`,
which apt's `python3-picamera2` and `python3-kms++` were built against.

## Piper, and why the subprocess boundary is load-bearing

**`piper-tts` (OHF-Voice/piper1-gpl) is GPL-3.0-or-later.** This repository is
public and Apache-2.0. Those two facts are compatible only because of where the
boundary is drawn, so it is drawn explicitly:

- Piper is **never imported.** The text-to-speech adapter runs
  `piper --output-raw` as a **subprocess** and reads its stdout (A28).
- Piper is **never a dependency of the published package.** It appears nowhere
  in `pyproject.toml`, in `uv.lock`, or in any wheel this repo produces.
- Piper is installed by `hosts/pi/deploy/install.sh` into **its own virtual environment**
  at `/opt/rover/.venv-tts`, which `[tts] bin` points at by absolute path.
  Nothing in `/opt/rover/.venv` can see it.
- On macOS the default is `[tts] backend = "say"`, so a developer machine never
  installs it at all.

Two processes exchanging PCM over a pipe are separate works. An import is not.
If that boundary ever moves — a Python `import piper` for convenience, or
vendoring the package — this repository can no longer be Apache-2.0. The
documented alternative, if `--output-raw` turns out to buffer rather than stream
(open item 10), is `wyoming-piper` as its own systemd unit: **the same boundary,
one process further out.**

## Voice and wake-word models

Model weights carry their own terms, separately from the code that runs them.

- **Piper voices** (`en_US-lessac-low` and the rest of `rhasspy/piper-voices`)
  are MIT, and are downloaded at install time into `/data/models/tts` rather
  than vendored here.
- **openWakeWord's pre-trained models are CC BY-NC-SA 4.0** — non-commercial,
  share-alike. **None of them is in this repository and none may be added.**
  `[wake] model = "rover.tflite"` is a **self-trained** model, produced in the
  project's own notebook (~1 hour) once the robot has a name (open item 11).
  Ship only that one.
- **Silero VAD** (`silero_vad.onnx`) is MIT, and reaches the robot through
  `sherpa_onnx.VoiceActivityDetector`, which vendors onnxruntime in its wheel.
- **Speech-recognition models** are chosen by the G3b bake-off and are not
  vendored. Check the licence of whichever wins before shipping it.

## The vision-language model

`philbert440/Qwen3.8-27B-W4A16-AWQ` and its two named fallbacks are served on a
machine you own, behind an OpenAI-compatible endpoint, and are not distributed
by this repository. Their weights carry their own terms — check the model card
before using one commercially. `box/prompts/system.md` is first-party and
Apache-2.0.

## Toolchain, containers, firmware

None of these is distributed by this repository; each is pulled at build time.

| thing | licence | note |
|---|---|---|
| ESP-IDF v5.5.5 and the `espressif/idf:v5.5.5` image | Apache-2.0 | the retired controller under `legacy/firmware-s3/` only; nothing current builds against it |
| arduino-cli, the `esp32` Arduino core 2.0.17 and the pinned Arduino libraries | GPL-3.0 (arduino-cli), LGPL-2.1 (core), various (see `firmware/build.sh`) | `make firmware` only; pulled at build time, never distributed here |
| vLLM | Apache-2.0 | `docker/compose.box.yml`, on the box |
| `wyoming-faster-whisper`, Kokoro-FastAPI | MIT / Apache-2.0 | opt-in speech profile, dark until G3b |
| `uv` | Apache-2.0 or MIT | installed to a fixed path by `hosts/pi/deploy/install.sh` |
| `esptool` | GPL-2.0-or-later | run via `uvx` at flash time only; never imported, never installed into the runtime environment |
| apt's `python3-picamera2`, `python3-libcamera`, `python3-simplejpeg` | BSD-2-Clause / LGPL-2.1+ | system packages on the Pi, imported through `--system-site-packages` |

`esptool` and `piper-tts` are the two copyleft tools in the build, and both are
kept at arm's length the same way: invoked as a program, never linked, never a
dependency of anything this repository publishes.

## Research notes

`docs/research/` contains fourteen notes written for this project. They quote
and cite third-party pages; the quotes belong to their authors and every one
carries its source URL. See `docs/research/README.md` for which notes were
independently verified.

## Firmware fork (`firmware/`)

**`firmware/` is GPL-3.0-or-later**, not Apache-2.0. It is a fork of
Waveshare's `ugv_base_general` (commit `d308df9`, 2025-11-28, GPL-3.0 -- see
`firmware/UPSTREAM.md`), carrying eight safety patches (`firmware/patches/`).
The boundary that keeps the rest of this repository Apache-2.0 is the same one
as for Piper: nothing here is imported or linked; the Pi and the board exchange
JSON text over a serial line. The earlier "ESP-IDF v5.5.5 / `firmware/main/`"
row above describes the retired ESP32-S3 design now under `legacy/firmware-s3/`
and does not apply to this directory.

Pulled by `firmware/build.sh` into the gitignored `firmware/.cache/`, never
committed:

| thing | version | licence (verify before relying on it) |
|---|---|---|
| arduino-esp32 core `esp32:esp32` | 2.0.17 | LGPL-2.1 (Arduino core), Apache-2.0 (ESP-IDF components) |
| arduino-cli (in the Docker image) | 1.5.1 | GPL-3.0 |
| ArduinoJson | 6.21.5 | MIT |
| Adafruit SSD1306, Adafruit GFX Library, Adafruit ICM20X | 2.5.17, 1.12.6, 2.0.7 | BSD-3-Clause |
| Adafruit BusIO | 1.17.4 | MIT |
| Adafruit Unified Sensor | 1.1.15 | Apache-2.0 |
| INA219_WE | 1.3.8 | MIT |
| ESP32Encoder | 5.0.0 | MIT |
| PID_v2 | 2.0.1 | MIT |
| SimpleKalmanFilter | 0.2.0 | MIT |
| VL53L1X (Pololu) | 1.3.1 | MIT |
| `esptool` (via `uvx`, flash time only) | latest | GPL-2.0-or-later |

Vendored in the tree: `firmware/libraries/SCServo/`, Waveshare's bus-servo
library, taken from the same GPL-3.0 repository.
