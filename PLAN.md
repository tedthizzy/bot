# Build plan

Tracker for the rover build. Checked items are done and verified on this machine unless marked `[hw]` (needs real hardware).

## 0. Inputs reconciled
- [x] Design brief (Pi 4 host, ESP32-S3 MCU, 27B VLM on 2× 3090)
- [x] Spec v1 (three Pi processes, serial + WS protocols, 9 invariants, gates)
- [x] Critique v2 (policy robots, Pi 5, teleop dataset path, LeRobot-shaped robotd)
- [x] Recommendations v3 (ESP-IDF, three timeouts, constrained decoding + validation, half-duplex first)
- [x] 2026 research notes per subsystem (13 notes, 78k words; 8 of 13 adversarially verified — 5 verifiers lost to a session limit, their notes are marked lower confidence)
- [~] Reconciled `ARCHITECTURE.md` (5 architects → chief architect → 4 critics → revision; running)

## 1. Contracts (shared, written first)
- [ ] `roverlib`: pydantic models for SkillCall, Observation, WorldState, WS messages, config
- [ ] Serial protocol spec + reference encoder/decoder (Python) with CRC test vectors
- [ ] JSON schemas exported from the models (`box/schema/*.json`)

## 2. Firmware (ESP32-S3, ESP-IDF)
- [ ] Portable core: protocol parser, TTL watchdog, caps, PID, fault latch, session/arm (no hardware deps)
- [ ] Host build of the core with clang++ + unit tests
- [ ] `mcu-sim`: core compiled for the host, speaks the serial protocol over a pty
- [ ] ESP-IDF app: PCNT encoders, MCPWM, ToF, bumper, e-stop input, ADC battery, USB-CDC
- [ ] Builds in the `espressif/idf` Docker image
- [ ] `[hw]` G2 bench gate

## 3. robotd (Pi, Python)
- [ ] Serial link (pyserial-asyncio), session handshake, telemetry parse, `serial_age_ms`
- [ ] Validator: skill allowlist, strict pydantic, bounds, seq/ttl, source allowlist, per-instruction budget
- [ ] Executor: trapezoid profiles at 20 Hz, one active skill, preemption, deadlines
- [ ] Odometry from ticks
- [ ] WS server `ws://127.0.0.1:8765`: state 20 Hz, results
- [ ] JSONL logging
- [ ] Unit tests + integration test against `mcu-sim`

## 4. brain (Pi, Python)
- [ ] Box client: OpenAI-compatible, `response_format` json_schema, prefix-friendly prompt order, timeout, cancellation
- [ ] FSM with explicit cancellation, instruction ids, late-response rejection
- [ ] Input adapters: text (dev), push-to-talk, wake word (openWakeWord) + VAD (Silero) + STT (Vosk; Moonshine adapter)
- [ ] TTS adapters: Piper resident; macOS `say` (dev); box TTS (optional)
- [ ] Camera adapters: picamera2; file/webcam (dev)
- [ ] Skills: say, describe_scene, find (stationary scan, observation schema), set_face
- [ ] Scene memory (SQLite)
- [ ] Unit tests with a fake box

## 5. web (Pi, FastAPI)
- [ ] Face page (WS expressions, Wake Lock), teleop page → SkillCall `source=web`
- [ ] Stubs: `POST /still`, `POST /cmd` (`source=phone`), rate limited

## 6. box
- [ ] `docker-compose.yml`: vLLM with the chosen 27B, prefix caching, structured outputs; optional STT/TTS
- [ ] `fakebox`: deterministic OpenAI-compatible stub for tests
- [ ] Ollama profile for small-VLM tests on the Mac
- [ ] System prompt ≤ 1k tokens; G1 prompt set

## 7. Dev/test on the Mac (M4)
- [ ] `make test`: Python unit tests + firmware core host tests
- [ ] `make sim`: mcu-sim ↔ robotd ↔ brain(text) ↔ fakebox end to end
- [ ] Docker compose (linux/arm64) mirroring the Pi for the Python services
- [ ] Gate scripts G1, G3, G4 (software parts) runnable in sim

## 8. Deploy (same day on the Pi)
- [ ] `deploy/install.sh`: apt packages, venv with system site packages, systemd units, udev rule, config
- [ ] Firmware flash procedure from the Mac
- [ ] README quickstart: box → firmware → Pi → first voice turn

## 8b. Repo and account
- [x] Local git repo on `main`, identity set
- [x] Python 3.12 venv with test dependencies
- [x] GitHub CLI authenticated as `tedthizzy` (repo scope)
- [ ] Public repo `tedthizzy/bot` created and pushed

## 9. Review and ship
- [ ] Adversarial review against the safety invariants; fixes applied
- [ ] Secret scan clean; `.gitignore` covers env/tokens/logs
- [ ] Commit; public repo `tedthizzy/bot`; push


## Findings that changed the plan

Recorded here as they land, with the note they came from.

- **No echo cancellation in the BOM.** A plain USB mic plus a separate amp means the robot hears its own speech: no barge-in, and text-to-speech leaks into recognition. Replace with a ReSpeaker Lite or XVF3800 USB board with XMOS echo cancellation, driving the speaker from its own amp. (`pi_speech`)
- **Speech-to-text is slower than the brief assumed.** Vosk's own endpointer guidance puts final text 0.5 to 0.8 seconds after end of speech, not 0.3. The voice turn budget moves to roughly 2.7 to 5 seconds with an image. (`pi_speech`)
- **openWakeWord is unmaintained.** No release since February 2024, and its dependency pin breaks installation on Python 3.12 and 3.13. Use pymicro-wakeword, or install with `--no-deps` plus onnxruntime. Porcupine's free tier ended in June 2026. (`pi_speech`)
- **Piper is measured, not assumed, and relicensed.** The maintained project is OHF-Voice/piper1-gpl under GPL-3.0, with streaming that makes sub-second first audio possible. (`pi_speech`)
- **One TTL is really two mechanisms.** A 20 Hz host heartbeat that the MCU times out on its own monotonic clock, and a per-command validity window checked on the Pi clock. The two clocks must never be compared. (`safety_chain`)
- **Host reboot is a live hazard.** The Pi 4 power-cycles USB on reboot and dev boards reset when the host toggles the serial control lines. Power the MCU separately and make MCU reset default to motors off. (`safety_chain`)
- **Constrained decoding is not a safety layer.** It guarantees syntax, not intent, and 2026 work shows text in the camera frame hijacks vision models at a meaningful rate. The deterministic validator and the MCU limits remain the safety layer. (`safety_chain`)
