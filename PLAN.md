# Build plan

Tracker for the rover build. Checked items are done and verified on this machine unless marked `[hw]` (needs real hardware).

## 0. Inputs reconciled
- [x] Design brief (Pi 4 host, ESP32-S3 MCU, 27B VLM on 2× 3090)
- [x] Spec v1 (three Pi processes, serial + WS protocols, 9 invariants, gates)
- [x] Critique v2 (policy robots, Pi 5, teleop dataset path, LeRobot-shaped robotd)
- [x] Recommendations v3 (ESP-IDF, three timeouts, constrained decoding + validation, half-duplex first)
- [x] 2026 research notes per subsystem (13 notes, 78k words; 8 of 13 adversarially verified — 5 verifiers lost to a session limit, their notes are marked lower confidence)
- [x] Reconciled `ARCHITECTURE.md` — 21,094 words, 38 numbered decisions, 24 safety invariants, exact serial grammar with golden frame vectors, two adversarial critique rounds applied

## 1. Contracts (shared, written first)

Build running: contracts first, then nine components in parallel, integration to green, two adversarial code-review rounds.

- [x] Shared contracts: models, skill catalog, unit boundary, identifiers, config loader, append-only log writer
- [x] Serial protocol codec with golden frame vectors shared between the C and Python implementations
- [x] JSON schemas exported from the models, with a drift check in continuous integration

## 2. Firmware (ESP32-S3, ESP-IDF)
- [x] Freestanding C core: codec, timeout, caps, ramps, wheel loop, fault latch, session and arm
- [x] Host build under the address and undefined-behaviour sanitizers, 20 cases passing
- [x] Simulator: the same core over a pseudo terminal, with 13 fault injections
- [x] ESP-IDF application: encoders, motor driver, distance sensors, bumper, emergency stop, battery
- [x] Compiles in the Espressif container: 1087 objects, firmware image 234 KiB, 84% of the partition free
- [ ] `[hw]` G2 bench gate

## 3. robotd (Pi, Python)
- [x] Serial link (pyserial-asyncio), session handshake, telemetry parse, `serial_age_ms`
- [x] Validator: skill allowlist, strict pydantic, bounds, seq/ttl, source allowlist, per-instruction budget
- [x] Executor: trapezoid profiles at 20 Hz, one active skill, preemption, deadlines
- [x] Odometry from ticks
- [x] WS server `ws://127.0.0.1:8765`: state 20 Hz, results
- [x] JSONL logging
- [x] Unit tests + integration test against `mcu-sim`

## 4. brain (Pi, Python)
- [x] Box client: OpenAI-compatible, `response_format` json_schema, prefix-friendly prompt order, timeout, cancellation
- [x] FSM with explicit cancellation, instruction ids, late-response rejection
- [x] Input adapters: text (dev), push-to-talk, wake word (openWakeWord) + VAD (Silero) + STT (Vosk; Moonshine adapter)
- [x] TTS adapters: Piper resident; macOS `say` (dev); box TTS (optional)
- [x] Camera adapters: picamera2; file/webcam (dev)
- [x] Skills: say, describe_scene, find (stationary scan, observation schema), set_face
- [x] Scene memory (SQLite)
- [x] Unit tests with a fake box

## 5. web (Pi, FastAPI)
- [x] Face page (WS expressions, Wake Lock), teleop page → SkillCall `source=web`
- [x] Stubs: `POST /still`, `POST /cmd` (`source=phone`), rate limited

## 6. box
- [x] `docker-compose.yml`: vLLM with the chosen 27B, prefix caching, structured outputs; optional STT/TTS
- [x] `fakebox`: deterministic OpenAI-compatible stub for tests
- [x] Ollama profile for small-VLM tests on the Mac
- [x] System prompt ≤ 1k tokens; G1 prompt set

## 7. Dev/test on the Mac (M4)
- [x] `make test`: Python unit tests + firmware core host tests
- [x] `make sim`: mcu-sim ↔ robotd ↔ brain(text) ↔ fakebox end to end
- [x] Docker compose (linux/arm64) mirroring the Pi for the Python services
- [x] Gate scripts G1, G3, G4 (software parts) runnable in sim

## 8. Deploy (same day on the Pi)
- [x] `deploy/install.sh`: apt packages, venv with system site packages, systemd units, udev rule, config
- [x] Firmware flash procedure from the Mac
- [x] README quickstart: box → firmware → Pi → first voice turn

## 8b. Repo and account
- [x] Local git repo on `main`, identity set
- [x] Python 3.12 venv with test dependencies
- [x] GitHub CLI authenticated as `tedthizzy` (repo scope)
- [x] Public repo `tedthizzy/bot` created and pushed

## 9. Review and ship
- [x] Adversarial review against the safety invariants; fixes applied
- [x] Secret scan clean; `.gitignore` covers env/tokens/logs
- [x] Commit; public repo `tedthizzy/bot`; push


## Phase 2 — WAVE ROVER chassis (ADR-0013), Pi host first

The chassis decision retired the custom controller track. `v0-pi-sim` tags the last commit of it. Sequence A: the Pi stack runs on the WAVE ROVER first; the phone becomes a second host against the same gates later.

- [x] Tag `v0-pi-sim`; move Pi packages to `hosts/pi/`, ESP32-S3 track to `legacy/firmware-s3/`
- [x] `docs/protocol.md`: the Waveshare JSON link, the fork's added fields, stop flags, banner
- [x] `docs/adr/0013-wave-rover-open-loop.md` and the `ARCHITECTURE.md` amendment table
- [x] Contracts rewritten: `drive_for`/`turn_to`, heading world state, `wave_proto` codec, `[link]` config
- [ ] Firmware fork: vendored `ugv_base_general`, patches (heartbeat 300 ms, cap 0.30, radios off, boot mission off, ToF/bumper block, banner and feedback fields), compiled in the arduino-cli container
- [x] `rover-stub`: the simulator speaking the protocol over pty and TCP, with `--stock` and fault flags (38 tests)
- [ ] robotd reworked: link bring-up and firmware gate, 20 Hz stream from the goal loop, `drive_for`, `turn_to` on the fused yaw, budget in seconds
- [ ] brain reworked: router, validator, prompt, world state, `find` via `turn_to`
- [ ] devtools and web reworked: fakebox cases, wirecat, doctor, roverctl, teleop in power units
- [ ] Contracts tests, config files, caps-match test against `bot_config.h`
- [ ] Gates re-mapped: G2 becomes patch verification; G4 fault injection; G5 adds `turn_to` accuracy
- [ ] `docs/verification.md`: the 24 invariants re-mapped, changed ones listed
- [x] `docs/wiring.md`: WAVE ROVER, Pi powered from the UPS, three data wires, inline stop in the battery lead, ToF on the shared I2C bus
- [ ] `hosts/android/`: module skeleton, generated Kotlin data classes, one instrumentation test against the stub
- [x] Deploy scripts retargeted: `/dev/serial0`, Bluetooth off the UART, console off, preflight reads the JSON banner
- [ ] `make test`, `make lint`, `make sim`, `make gates-full` green; commit; push
- [ ] `[hw]` G2 on the real board: heartbeat, cap, boot state, e-stop, both wheels held
- [ ] `[hw]` G5 `turn_to(90)` within ±10° on hardwood and carpet; `yaw_sign` recorded

## Findings that changed the plan

Recorded here as they land, with the note they came from.

- **No echo cancellation in the BOM.** A plain USB mic plus a separate amp means the robot hears its own speech: no barge-in, and text-to-speech leaks into recognition. Replace with a ReSpeaker Lite or XVF3800 USB board with XMOS echo cancellation, driving the speaker from its own amp. (`pi_speech`)
- **Speech-to-text is slower than the brief assumed.** Vosk's own endpointer guidance puts final text 0.5 to 0.8 seconds after end of speech, not 0.3. The voice turn budget moves to roughly 2.7 to 5 seconds with an image. (`pi_speech`)
- **openWakeWord is unmaintained.** No release since February 2024, and its dependency pin breaks installation on Python 3.12 and 3.13. Use pymicro-wakeword, or install with `--no-deps` plus onnxruntime. Porcupine's free tier ended in June 2026. (`pi_speech`)
- **Piper is measured, not assumed, and relicensed.** The maintained project is OHF-Voice/piper1-gpl under GPL-3.0, with streaming that makes sub-second first audio possible. (`pi_speech`)
- **One TTL is really two mechanisms.** A 20 Hz host heartbeat that the MCU times out on its own monotonic clock, and a per-command validity window checked on the Pi clock. The two clocks must never be compared. (`safety_chain`)
- **Host reboot is a live hazard.** The Pi 4 power-cycles USB on reboot and dev boards reset when the host toggles the serial control lines. Power the MCU separately and make MCU reset default to motors off. (`safety_chain`)
- **Constrained decoding is not a safety layer.** It guarantees syntax, not intent, and 2026 work shows text in the camera frame hijacks vision models at a meaningful rate. The deterministic validator and the MCU limits remain the safety layer. (`safety_chain`)
