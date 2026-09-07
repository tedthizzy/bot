# Safety architecture for an LLM-commanded indoor home rover

Research note, 2026-09-07. Dimension: layered validation, heartbeats, fail-safe, LLM-specific hazards, operational safety, observability.

## Summary

The brief's three-layer chain (box output -> Pi schema/bounds/TTL validator -> ESP32 hard limits) is the same shape a 2026 practitioner would build, and its numbers (300 ms MCU timeout, 0.3 m/s, 1 m per drive, 60 deg/s) sit inside the range used by ROS 2, LeRobot and hobby autopilots. Five things need changing or adding:

1. Split "TTL" into two mechanisms: a 20 Hz host heartbeat with sequence numbers that the ESP32 times out at 300 ms on its own monotonic clock, and a per-command validity window (issued_at + 2 s, checked on the Pi) that is never compared against the MCU clock.
2. The LLM must never be in the stop loop. A voice turn is 2.5–4.5 s; at 0.3 m/s that is 0.75–1.35 m of travel. The only fast reflexes are the ToF/bumper/current checks on the ESP32, so the ESP32, not the Pi, owns stop, slowdown and cliff.
3. Structured output (vLLM `guided_json` / `response_format: json_schema`, XGrammar backend, since vLLM 0.8.5) replaces "ask for JSON and hope"; it guarantees syntax and lets the skill name be a closed enum at decode time. It does not guarantee safety. 2026 papers show constrained decoding is itself an attack channel and that text in the camera frame hijacks GPT-4o-class VLMs 27–29% of the time. The deterministic Pi validator and the MCU limits remain the safety layer.
4. Host reboot is a live hazard: Pi 4 rev 1.4 power-cycles USB on every reboot, and ESP32 dev boards reset when a host toggles DTR/RTS. Power the ESP32 separately, use plain UART or cut the DTR/RTS auto-reset, and make MCU reset itself default to motors off.
5. Add: systemd `WatchdogSec` + `bcm2835_wdt` (15 s max) on the Pi, ESP-IDF Task WDT with panic-reset on the ESP32, a downward ToF for stairs, an observation-freshness gate, motion rate limiting, and a fault-injection matrix (cable pull, `kill -9`, `SIGSTOP`, Wi-Fi off, Pi reboot mid-drive, ToF unplugged).

## State of the art (2026)

### Layered safety in ROS 2 and hobby stacks

Nav2's Collision Monitor is the reference design for "a safety node below the planner". It runs as an independent filter on `cmd_vel`, with polygons around the robot that trigger `stop`, `slowdown`, `limit` or `approach` actions; when several trigger, the most aggressive wins; `VelocityPolygon` swaps zones by commanded speed. The `approach` model estimates time-to-collision from current velocity and slows the robot so it stays at least M seconds from contact (example config: `time_before_collision: 2.0`, `simulation_time_step: 0.1`). The shipped example uses `source_timeout: 5.0` (stale sensor data forces a stop) and `stop_pub_timeout: 2.0`; example zones are a 0.3 m stop box, a 0.4 m slowdown box at 30% speed, and a 0.5 m limit box (0.4 m/s, 0.5 rad/s) [VENDOR, Nav2 main, 2026]. The pattern to copy: the safety filter is dumb, fast, and independent of the planner, and stale sensor data is treated as a hazard.

Safe-ROS (FMAS 2025, AgileX Scout Mini) formalises the same split: an "intelligent control system" and a separate Safety System of formally verified Safety Instrumented Functions with independent oversight, one SIF being "stop the robot when too close to an obstacle". The ROS 2 Safety WG's `software_watchdogs` package uses DDS liveliness/deadline QoS and lifecycle nodes; its example is a 200 ms heartbeat with a 220 ms lease, and a windowed variant tolerates 3 missed deadlines [VENDOR, ros-safety, built on Rolling/Ubuntu 20.04, so dated].

Timeouts in shipping stacks cluster at 0.2–1 s:

- ros2_control `diff_drive_controller` `cmd_vel_timeout` default 0.5 s; `twist_mux` per-input `timeout` 0.5 s in the standard examples; `teleop_twist_joy` uses a deadman button [VENDOR, Rolling docs Jul 2026].
- LeRobot LeKiwi host: `watchdog_timeout_ms = 500`, `max_loop_freq_hz = 30`; when no ZMQ command arrives it logs "Command not received for more than … milliseconds. Stopping the base." and calls `robot.stop_base()` [VENDOR, huggingface/lerobot main]. This is the stack the brief plans to merge with, so 300 ms on the ESP32 sits comfortably beneath it.
- ArduPilot Rover: `FS_TIMEOUT` 1 s for RC loss, `FS_GCS_TIMEOUT` 5 s for lost MAVLink heartbeat, `FS_ACTION` Hold/RTL/Disarm [VENDOR].

### Standards, scaled to hobby size

ISO 13482:2014 (personal care robots: mobile servant, physical assistant, person carrier) requires speed and force limits, protective stops and emergency stop, and excludes robots faster than 20 km/h; a revision retitled "safety requirements for service robots" was at FDIS stage in 2025–2026 [VENDOR, iso.org / iTeh listing; I could not read the normative text]. ISO 3691-4 (driverless industrial trucks, 2020, amended 2023) defines operating/restricted/confined zones, requires PLd (ISO 13849) personnel detection able to see a 200 mm test piece, and requires braking to stop within the detection range; secondary sources state 0.3 m/s as the ceiling for vehicles without personnel detection and 0.3–1.2 m/s by zone [VENDOR, secondary; the 0.3 m/s clause came from search summaries of vendor blogs I could not open, treat as unverified]. ANSI/A3 R15.06-2025 (three parts) and ANSI/A3 R15.08-2 (industrial mobile robots) were published 2025; R15.08 adds an S1–S4 severity tiering. ISO/TS 15066 Annex A tabulates transient and quasi-static force/pressure limits per body region and forbids head/face contact; I could not obtain the numeric table from an open source.

Scaling: the standards' 0.3 m/s "no detection" limit is the strongest external anchor for the brief's speed cap. For a 3 kg rover, kinetic energy at 0.3 m/s is 0.135 J [INFERRED], far below any limb contact limit; the realistic hazards in a home are tripping (a person steps on it), toes/pet paws under wheels, stairs, cords, and battery/thermal faults, not impact energy.

### LLM-specific hazards, 2025–2026 evidence

- Physical prompt injection: text on a sheet of paper in the camera view redirected a VLM-controlled sorting task in 27.0% (GPT-4o), 29.4% (Gemini 2.5 Flash) and 5.0% (Qwen3-VL-32B) of 5,670 trials; authority-impersonation and negation prompts reached 80–100% for specific wordings; 99.9% of compromises were consciously acknowledged, i.e. the model read the sign and obeyed it. Defenses: a system-prompt rule to ignore in-scene text (75–98.9%), a second-model plan check (85–100%), OCR-based text masking before the VLM (100%) [MEASURED, arXiv 2608.05715, Aug 2026].
- RIPA (June 2026) injected through OCR'd text, Whisper speech transcripts, and poisoned LiDAR/obstacle context on LLM-controlled ROS 2 robots. Llama-3.3-70B hit 100% attack success across variants; Llama-3-8B and Qwen2.5-7B resisted direct override (0%); a hybrid semantic firewall blocked known patterns but showed a 10.2% bypass rate against obfuscated attacks [MEASURED, arXiv 2606.28649].
- RoboPAIR (Oct 2024) jailbroke a GPT-4o Jackal planner and a GPT-3.5 Unitree Go2 with success "often 100%"; BadRobot (ICLR 2025) did it through ordinary voice interaction against VoxPoser, Code-as-Policies and ProgPrompt [MEASURED].
- RoboGuard (RA-L, accepted Feb 2026): an offline-configured guard that grounds safety rules into LTL specs and synthesises compliant plans cut unsafe execution under worst-case jailbreaks from over 92% to under 3% [MEASURED, simulation + Spot]. Google DeepMind's ASIMOV/robot-constitution work (Mar 2025) reached 84.3% alignment with generated constitutions as a VLM guardrail [MEASURED].
- Constrained decoding is not a safety layer: "Exploiting Structured Generation to Bypass LLM Safety" (rev Aug 2026) shows schema-enforced logit masking can be used to inject a malicious prefix; DictAttack hit 94.3–99.5% on 13 models including GPT-5 and Gemini 2.5 Pro and kept 75.8% against jailbreak guardrails [MEASURED]. Schema key wording itself acts as an instruction channel (arXiv 2604.14862). Lesson: the grammar must be static and server-side; nothing from the user or the scene may shape it.
- Hallucinated tool calls: 2025–2026 tool-use benchmarks (BFCL, ToolScan, AgentHallu) classify wrong function name, empty/wrong parameter, and wrong parameter value as distinct error types and show sharp degradation on multi-turn/long-horizon sequences versus single-turn selection [MEASURED, various]. A closed enum of skill names plus per-field bounds removes the first three classes mechanically.

### Host and MCU watchdogs

systemd `WatchdogSec=`: the service must send `WATCHDOG=1` via `sd_notify` at least every interval (recommended: every half interval, `WATCHDOG_USEC` is passed in the environment); on miss the service gets `SIGABRT` (configurable via `WatchdogSignal=`) and `Restart=on-watchdog|on-failure|on-abnormal|always` restarts it [VENDOR, systemd.service(5)]. Python libraries: `sdnotify`, `systemd-watchdog`, `sd-notify`.

Kernel watchdog on Pi: driver `bcm2835_wdt`, `dtparam=watchdog=on`, `RuntimeWatchdogSec=` in `/etc/systemd/system.conf`, hardware maximum 15 s, kernel pets it every half interval; Pi kernels lack `CONFIG_WATCHDOG_SYSFS` so `/sys/class/watchdog` status is missing [MEASURED, Pi 4B, kernel 6.1.21-v8+, forum 2023-06-28]. The systemd watchdog does not care about load; the `watchdog` daemon adds `max-load-*` triggers.

Pi reboot vs ESP32: on Pi 4 rev 1.4 USB port power is cut during a software reboot; a Raspberry Pi engineer stated "external reset is linked to the chip reset so there's no way to prevent USB port power from being turned off at reboot"; `USB_MSD_PWR_OFF_TIME` can lengthen but not remove it [VENDOR/MEASURED, forum 2021-04-03]. Separately, esptool documents that dev boards wire RTS->EN and DTR->GPIO0, that Linux asserts RTS on unopened ports and "can hold the ESP32-S3 in a reset loop", and suggests `stty -F /dev/ttyUSB0 -hupcl` and a >=1 uF cap on EN [VENDOR]. For native USB-Serial/JTAG on S2/S3, an Espressif issue (Aug 2026) describes reset "on any falling RTS edge" when a terminal opens/closes the port, and USB not surviving reset [VENDOR, issue #18992]. Net: a Pi reboot or a serial-port reopen can reset the MCU mid-drive; the MCU must boot into motors-off and require a fresh heartbeat plus an explicit command before moving.

ESP32-S3 own watchdogs (ESP-IDF v6.1): Interrupt WDT panics then hard-resets on second stage; Task WDT prints a backtrace by default and only resets if `CONFIG_ESP_TASK_WDT_PANIC=y`; subscribe the motor-control task with `esp_task_wdt_add()` and feed it with `esp_task_wdt_reset()` [VENDOR].

### Sensing for stop and cliff

VL53L1X: short mode ~1.3 m max, 50 Hz max, mostly immune to ambient light; medium/long 3–4 m in the dark at 30 Hz; 27 deg FoV; 4 cm minimum; 1 mm resolution [VENDOR, Pololu #3415]. Timing budget 20 ms minimum; accuracy improves roughly with the square root of budget [VENDOR, ST community]. ST engineers endorse ToF pointed at the floor ahead for hole/stair detection: baseline the flat-floor distance at the chosen angle, alarm on deviation, and use signal rate as a second feature; specular floors can return nothing [VENDOR, ST community 2022–2023]. Robot-vacuum cliff sensors fail on dark rugs (absorb IR, look like a void), dirty windows, direct sunlight, and tilted chassis at rug edges [VENDOR, eufy 2025].

Hobby postmortems: the recurring pattern is "controller froze or link dropped, motors kept last command". A balancing-robot thread (Arduino Uno, MDD10A) traced freezes to motor noise and an I2C antenna loop; the fixes discussed were capacitors and wiring, and no timeout was added. RC-rover threads describe receivers holding last stick value on transmitter loss. None of these had a command TTL; this build does, which is the main thing that separates it from them.

## Recommendation for this build

### Layer responsibilities

| Layer | Checks | Must not do |
|---|---|---|
| Box (27B via vLLM) | Syntax only, enforced by `guided_json` against a static server-side schema; `skill` is an enum of the 7 names; numeric fields have schema `minimum`/`maximum` equal to the bounds. | Be trusted for safety; see prompt-injection and structured-generation results above. |
| Pi validator (pydantic, deterministic) | Schema re-validation; deny-by-default skill whitelist; bounds; units; `issued_at` within 2 s on `time.monotonic()`; observation freshness (frame used <= 3 s old, else refuse motion); one motion skill per turn; rate limits (max 1 motion per 3 s, max 4 direction reversals per minute); state gating (no `drive` while bumper pressed, ToF < 0.25 m, battery low, or e-stop latched); retry once on schema failure then `stop()`; log provenance. | Hold the stop loop; issue open-ended velocity commands; interpret free text from STT or OCR as instructions. |
| ESP32-S3 | Heartbeat timeout 300 ms on `esp_timer_get_time()`; sequence-number monotonicity; hard clamps (0.3 m/s, 60 deg/s, per-command distance <= 1 m and duration <= distance/speed + 0.5 s); ToF stop < 0.25 m, slowdown < 0.5 m, time-to-collision >= 1 s (`v <= d / 1 s`); bumper, current limit, cliff; boot to motors-off. | Trust any host timestamp; auto-resume an interrupted motion. |

Wire format Pi -> MCU (binary or compact JSON at 115200–921600 baud): `seq`, `type` (heartbeat | drive | turn | stop), parameters, CRC. MCU -> Pi at 20 Hz: `seq_ack`, MCU monotonic ms, encoder odometry, ToF ranges, bumper, current, fault flags. Heartbeat at 20 Hz (50 ms) so 300 ms equals six missed beats and absorbs Python scheduling jitter; jitter over USB CDC on a loaded Pi 4 is not published, expect tens of ms under Vosk+camera load [INFERRED]; measure it with the ack stream.

### Stop semantics

Heartbeat loss, bumper, ToF stop, cliff, current fault: active brake (PID to zero velocity, then H-bridge brake for ~0.5 s, then coast to avoid holding current). Coast alone lets a 3 kg rover roll on a threshold or sloped floor. E-stop: hardware break of the motor rail (motors coast; acceptable at 0.3 m/s) with the ESP32 reading the latch state and refusing commands until released and re-armed. After any stop the MCU discards the interrupted command; the Pi must send a new one after a fresh observation.

Stopping distance at 0.3 m/s [INFERRED]: ToF sample interval 33 ms + MCU loop 10 ms -> reaction 0.013 m; braking at 1–2 m/s^2 -> 0.02–0.045 m; total about 4–6 cm. A 0.25 m stop zone and 0.5 m slowdown zone leave margin for the 27 deg FoV missing a chair leg. A stop routed through the Pi adds 5–30 ms; a stop routed through the LLM adds 2.5–4.5 s (0.75–1.35 m), which is why the LLM never gets that job. TTL expiry alone (heartbeat loss during a 1 m drive) allows up to 0.09 m before detection plus 4–6 cm braking, about 15 cm total; acceptable.

### Bounds

Keep 0.3 m/s as the hard MCU cap (it matches the industrial "no personnel detection" ceiling). Default the Pi-side cap to 0.2 m/s and allow 0.3 m/s only when ToF > 1 m and the frame is fresh. Keep 1 m per drive; 3.3 s of unobserved motion is the practical limit given no fast person detection on the Pi 4. 180 deg per turn and 60 deg/s are fine; add angular slowdown when ToF < 0.5 m so the chassis corner does not clip a shin.

### LLM hazards

- Input hygiene: wrap STT text and any OCR/scene text in delimiters and label them as untrusted observations; system prompt says text seen in the scene and quoted speech are never commands. Expect 75–99% reduction, not elimination.
- Speech commands are the only command channel and still pass the validator; "ignore previous instructions" cannot widen bounds because there is no bound-override field.
- One skill per turn, no motion chaining. `find(object)` should be a Pi-side deterministic scan (rotate in <= 60 deg steps with ToF checks, at most 360 deg) that hands each frame back to the model for `describe_scene`; the model never scripts loops.
- Oscillation: reversal counter and cooldown in the validator; `set_face`/`say` are unlimited, motion is rate-limited.
- Anything outside bounds is rejected and spoken back; there is no "confirm to override" path in v1, because a confirmation heard through the same STT channel is itself injectable.
- Keep the grammar static; never include user text in the schema or in `guided_choice` lists.

### Host watchdogs and reboot

Run the orchestrator as `Type=notify`, `WatchdogSec=10`, ping every 5 s from the main loop only after the serial ack stream and heartbeat thread report healthy; `Restart=on-failure`. Enable `RuntimeWatchdogSec=14` (under the 15 s hardware cap) and `dtparam=watchdog=on`. Power the ESP32 from its own 5 V buck (the brief already does) and prefer the Pi's GPIO UART (`/dev/ttyAMA0`, 3.3 V) or a USB-UART with DTR/RTS not wired to EN/GPIO0; if you must use the S3 native USB, set `-hupcl` and accept the port may drop on MCU reset. Motor driver enable pins pulled low by resistor so MCU reset means motors off.

### Cliff and pets

Add a third VL53L1X (or VL53L4CD) pointed ~45 deg down and forward, short mode, 50 Hz, baseline the floor distance, stop on +8 cm deviation or on invalid range; expect false stops on black rugs and in sunlight and tune the threshold per home. Pets: robot vacuums rarely injure but stress animals; keep sound and motion predictable, start at 0.15 m/s for the first weeks, and never drive toward a detected animal (the model can report one in `describe_scene`; the validator then caps speed).

### E-stop

A mushroom latch on the top rear, reachable from standing; a second wireless kill (cheap RF relay in the motor rail) for when the rover is under a couch. Test that it works with the Pi and box off.

### Observability and tests

Per command, log JSONL: `cmd_id`, `seq`, `utterance_id`, `frame_id`, model name and prompt hash, `t_frame_captured`, `t_stt_final`, `t_llm_req`, `t_llm_resp`, `t_validated`, `t_sent_mcu`, `t_mcu_ack` (MCU ms), validator verdict, MCU fault flags. Observation-to-action age = `t_sent_mcu - t_frame_captured`; alert above 3 s. Record the ack stream at 20 Hz for replay; a replay tool feeds logged observations back through the validator to regression-test rule changes.

Fault-injection matrix (each pass/fail measured from encoders and MCU log): USB cable pull mid-drive; `kill -9` orchestrator; `kill -STOP` orchestrator (process alive, heartbeat silent); `nmcli radio wifi off` mid-turn; box returns malformed, oversize, out-of-range, expired (`issued_at` - 5 s), or replayed (`seq` rollback) commands; box latency 30 s; ToF I2C disconnected (treat as obstacle); bumper held; ESP32 test build with an infinite loop (Task WDT must reset to motors-off); Pi `sudo reboot` mid-drive; battery brownout at 3S low cutoff; e-stop with all logic off.

## Numbers

| quantity | value | hardware/context | tag | source URL |
|---|---|---|---|---|
| Nav2 collision monitor example `source_timeout` / `stop_pub_timeout` | 5.0 s / 2.0 s | shipped params YAML, Nav2 main | VENDOR | https://github.com/ros-navigation/navigation2/blob/main/nav2_collision_monitor/params/collision_monitor_params.yaml |
| Nav2 approach model `time_before_collision` | 2.0 s (step 0.1 s) | same | VENDOR | same |
| Nav2 example stop/slowdown/limit zones | 0.3 m box; 0.4 m box at 30%; 0.5 m box capped 0.4 m/s, 0.5 rad/s | same | VENDOR | same |
| ros-safety heartbeat / lease | 200 ms / 220 ms; windowed allows 3 misses | ROS 2 Rolling, Ubuntu 20.04 | VENDOR | https://github.com/ros-safety/software_watchdogs |
| `diff_drive_controller` `cmd_vel_timeout` | 0.5 s default | ros2_control Rolling, Jul 2026 | VENDOR | https://control.ros.org/rolling/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html |
| LeKiwi host watchdog / loop rate | 500 ms / 30 Hz | huggingface/lerobot main, Pi host over ZMQ | VENDOR | https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/lekiwi/config_lekiwi.py |
| ArduPilot Rover `FS_TIMEOUT` / `FS_GCS_TIMEOUT` | 1 s / 5 s | Rover docs | VENDOR | https://ardupilot.org/rover/docs/rover-failsafes.html |
| ISO 3691-4 speed without personnel detection | 0.3 m/s (0.3–1.2 m/s by zone) | industrial AGVs; secondary sources only | VENDOR (unverified) | https://blog.ansi.org/ansi/iso-3691-4-2023-driverless-industrial-trucks/ |
| ISO 3691-4 personnel detection test piece / PL | 200 mm, PLd | industrial AGVs | VENDOR | https://www.saphira.ai/blog/mobile-robot-safety-standards-understanding-iso-3691-4-(driverless-industrial-trucks)-and-r15-08-(industrial-mobile-robots)-implementation |
| ISO 13482:2014 scope ceiling | robots <= 20 km/h | personal care robots | VENDOR | https://www.iso.org/standard/53820.html |
| bcm2835_wdt max timeout | 15 s | Pi 4B, kernel 6.1.21-v8+ | MEASURED | https://forums.raspberrypi.com/viewtopic.php?t=353094 |
| systemd watchdog ping interval | half of `WatchdogSec` (`WATCHDOG_USEC`) | systemd.service(5) | VENDOR | https://man7.org/linux/man-pages/man5/systemd.service.5.html |
| Pi 4 USB power at reboot | cut, unavoidable on rev 1.4 | Pi engineer statement | MEASURED | https://forums.raspberrypi.com/viewtopic.php?t=308558 |
| ESP32 auto-reset wiring | RTS->EN, DTR->GPIO0; Linux asserts RTS on idle ports | esptool docs, ESP32-S3 | VENDOR | https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html |
| ESP32-S2/S3 native USB reset | on any falling RTS edge; USB does not survive reset | Espressif issue, 2026-08-18 | VENDOR | https://github.com/espressif/esp-idf/issues/18992 |
| VL53L1X short mode | ~1.3 m, 50 Hz, 27 deg FoV, 4 cm min | Pololu #3415 | VENDOR | https://www.pololu.com/product/3415 |
| VL53L1X timing budget | 20 ms minimum | ST community | VENDOR | https://community.st.com/t5/imaging-sensors/vl53l1x-timing-budget-and-intermeasurement-time/td-p/221497 |
| Physical prompt injection success | 27.0% GPT-4o, 29.4% Gemini 2.5 Flash, 5.0% Qwen3-VL-32B; 5,670 trials | sorting task, Aug 2026 | MEASURED | https://arxiv.org/html/2608.05715 |
| Prompt-injection defenses | prompt rule 75–98.9%; 2nd-model check 85–100%; OCR masking 100% | same | MEASURED | same |
| RIPA attack success | 100% on Llama-3.3-70B; 0% direct-override on Llama-3-8B, Qwen2.5-7B; firewall bypass 10.2% | LLM-controlled ROS 2 robots, Jun 2026 | MEASURED | https://arxiv.org/abs/2606.28649 |
| RoboGuard unsafe-plan rate | >92% -> <3% | worst-case jailbreaks, RA-L 2026 | MEASURED | https://arxiv.org/abs/2503.07885 |
| Structured-generation attack | 94.3–99.5% on 13 models; 75.8% vs guardrails | GPT-5, Gemini 2.5 Pro etc. | MEASURED | https://arxiv.org/abs/2503.24191 |
| vLLM structured outputs | `guided_json`/`guided_choice`/`guided_regex`/`guided_grammar`; XGrammar/Guidance/Outlines; since 0.8.5 V1 | Red Hat article 2025-06-03 | VENDOR | https://developers.redhat.com/articles/2025/06/03/structured-outputs-vllm-guiding-ai-responses |
| Rover kinetic energy | 0.135 J | 3 kg at 0.3 m/s | INFERRED | — |
| Stop distance from ToF trigger | 4–6 cm | 0.3 m/s, 33 ms sample + 10 ms loop, 1–2 m/s^2 brake | INFERRED | — |
| Travel during one LLM turn | 0.75–1.35 m | 0.3 m/s x 2.5–4.5 s (brief) | INFERRED | — |
| Travel on heartbeat loss | ~15 cm | 300 ms timeout + braking at 0.3 m/s | INFERRED | — |

## Corrections to the brief

- "300 ms command TTL -> stop": holds as a heartbeat timeout and is at the tight end of 2026 practice (LeKiwi 500 ms, ros2_control 0.5 s, ros-safety 220 ms). It conflates two things; make the heartbeat 20 Hz with sequence numbers timed on the MCU clock, and give each skill command its own 2 s validity checked on the Pi. Specify that "stop" means active brake and that the interrupted command is discarded.
- Gate "cable pull -> stop <= 300 ms": measure timeout plus braking (expect ~350 ms and ~15 cm); add `kill -9`, `kill -STOP`, Wi-Fi off, Pi reboot mid-drive, and MCU firmware hang to the gate.
- "Same validator chain as before: box output -> Pi schema/bounds/TTL -> ESP32 limits": correct shape; add observation-freshness, rate limiting and state gating to the Pi layer, and boot-to-motors-off plus sequence checks to the MCU.
- "Assume Gemma-class: don't rely on native tool-call parsing. Ask for one JSON object": in 2026 use vLLM `guided_json` with a static schema and an enum skill field; keep pydantic. The papers above mean this raises reliability, not safety.
- "ESP32-S3 ... Serial to Pi": a Pi 4 rev 1.4 reboot cuts USB power and dev-board DTR/RTS wiring resets the MCU on port open; use GPIO UART or cut the auto-reset, and rely on motors-off-at-boot.
- BOM "2x VL53L1X": forward pair covers two 27 deg cones with a gap; add a third downward unit for stairs, and keep the bumper as the last layer.
- Skill `find(object)` as written implies a multi-step plan; define it as a bounded Pi-side scan, not an LLM loop.
- Bounds (<= 1 m, <= 0.3 m/s, <= 180 deg, <= 60 deg/s) hold; 0.3 m/s matches the industrial no-detection ceiling. Default to 0.2 m/s and unlock 0.3 m/s only with fresh observation and clear ToF.
- Missing entirely: systemd `WatchdogSec`, `bcm2835_wdt`, ESP-IDF Task WDT, e-stop placement, cliff sensing, provenance logging.

## Alternatives considered and rejected

- Pi-side stop loop (camera or ToF over serial to Python): adds Python/USB jitter and dies with the orchestrator; the MCU already has the sensors. Rejected for stop; keep the Pi for policy gates only.
- Coast-only on timeout: rolls on slopes/thresholds and stops later; rejected in favour of brake-then-coast.
- LLM-in-the-loop confirmation for out-of-bounds requests: the confirmation channel (STT) is the injection channel; rejected for v1. Revisit with a physical confirm button.
- A second LLM as plan verifier (RoboGuard-style or the paper's D2): 85–100% effective in the benchmarks but adds 1–3 s per turn on the box and still fails open on novel attacks; deferred until a second container is available. The deterministic validator covers the seven-skill surface.
- OCR text masking before the VLM (100% in the benchmark): worth it later, but on a Pi 4 it costs CPU and breaks legitimate label reading; run it on the box if at all.
- ROS 2 / Nav2 collision monitor on the Pi now: correct long-term, heavy on a 4 GB Pi 4 with Vosk and camera; copy its zone/approach semantics into the ESP32 firmware and adopt the node when Nav2 arrives.
- Bumper-only proximity (no ToF): simplest, but contact-first at 0.3 m/s means every toe is a test; rejected.
- Hardware watchdog only (no systemd service watchdog): a hung orchestrator with a live kernel would not reboot; keep both.

## Open questions

- Actual USB-serial heartbeat jitter on this Pi 4 under Vosk + camera load; nobody has published it, measure it via the ack stream before fixing 300 ms.
- Braking deceleration of the chosen gearmotors/driver on carpet vs hardwood; needed to set the ToF stop zone precisely.
- Does the chosen ESP32-S3 board use native USB or a bridge, and does its DTR/RTS wiring reset on port open? Test with a scope before trusting the reboot behaviour.
- Which vLLM version and backend the box runs; `guided_json` semantics changed across 0.8.x–0.10.x and the auto backend picks per request.
- How often a Gemma-class 27B emits a wrong-but-schema-valid skill (e.g. `drive` when asked to describe); the retry-once policy needs a measured rate.
- Numeric ISO/TS 15066 and ISO 13482 limits were not verifiable from open sources; the 0.3 m/s ISO 3691-4 clause is from secondary sources.
- Downward ToF false-stop rate on this home's floors (dark rugs, sunlight).
- Whether `kill -STOP` on the orchestrator also freezes a separate heartbeat thread; if the heartbeat lives in a subprocess, add a health check so a wedged main loop stops the beat.

## Sources

- Nav2 Collision Monitor README, https://github.com/ros-navigation/navigation2/blob/main/nav2_collision_monitor/README.md, main, accessed 2026-09-07
- Nav2 collision_monitor_params.yaml, https://github.com/ros-navigation/navigation2/blob/main/nav2_collision_monitor/params/collision_monitor_params.yaml, main, accessed 2026-09-07
- nav2_collision_monitor Jazzy docs, https://docs.ros.org/en/jazzy/p/nav2_collision_monitor/, 1.3.11
- ros-safety/software_watchdogs, https://github.com/ros-safety/software_watchdogs, ROS 2 Rolling era
- ROS 2 Safety WG survey thread, https://discourse.openrobotics.org/t/safety-working-group-survey-on-ros-2-safety/23511
- Safe-ROS (FMAS 2025), https://arxiv.org/abs/2511.14433, 2025-11-18
- diff_drive_controller userdoc, https://control.ros.org/rolling/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html, Rolling Jul 2026
- LeKiwi config and host, https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/lekiwi/config_lekiwi.py and .../lekiwi_host.py, main
- ArduPilot Rover failsafes, https://ardupilot.org/rover/docs/rover-failsafes.html
- systemd.service(5), https://man7.org/linux/man-pages/man5/systemd.service.5.html
- Raspberry Pi forum, hardware watchdog [solved], https://forums.raspberrypi.com/viewtopic.php?t=353094, 2023-06-28
- Raspberry Pi forum, RPi4 v1.4 reboot USB powercycle, https://forums.raspberrypi.com/viewtopic.php?t=308558, 2021-04-03
- esptool Boot Mode Selection (ESP32-S3), https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html
- ESP-IDF v6.1 Watchdogs (ESP32-S3), https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html
- espressif/esp-idf issue #18992, https://github.com/espressif/esp-idf/issues/18992, 2026-08-18
- Pololu VL53L1X #3415, https://www.pololu.com/product/3415
- ST community, VL53L1X timing budget, https://community.st.com/t5/imaging-sensors/vl53l1x-timing-budget-and-intermeasurement-time/td-p/221497
- ST community, VL53L5CX for stairs, https://community.st.com/t5/imaging-sensors/is-the-vl53l5cx-tof-a-good-choice-considering-all-the-time-of/td-p/71677, 2022-04-11
- eufy, Do robot vacuums fall down stairs, https://www.eufy.com/blogs/robovac/do-robot-vacuums-fall-down-stairs
- Hijacking Robots with a Piece of Paper, https://arxiv.org/html/2608.05715, 2026-08-06
- RIPA, https://arxiv.org/abs/2606.28649, 2026-06-26
- RoboGuard, https://arxiv.org/abs/2503.07885, rev 2026-03-03 (RA-L accepted Feb 2026)
- Jailbreaking LLM-Controlled Robots (RoboPAIR), https://arxiv.org/abs/2410.13691, 2024-10-17
- BadRobot, https://arxiv.org/abs/2407.20242, ICLR 2025, rev 2026-06
- Generating Robot Constitutions & ASIMOV, https://arxiv.org/abs/2503.08663, 2025-03-11
- Exploiting Structured Generation to Bypass LLM Safety, https://arxiv.org/abs/2503.24191, rev 2026-08-10
- Schema Key Wording as an Instruction Channel, https://arxiv.org/pdf/2604.14862, 2026
- Red Hat, Structured outputs in vLLM, https://developers.redhat.com/articles/2025/06/03/structured-outputs-vllm-guiding-ai-responses, 2025-06-03
- ROS-RVFT guideline MTA1 (fault injection), https://ros-rvft.github.io/guidelines/guideline-mta1
- Arduino forum, freezing with motors running, https://forum.arduino.cc/t/arduino-freezing-serial-stops-and-motors-keep-running-indefinitely/540078
- Pololu Qik 2s9v1 guide (coast vs brake), https://www.pololu.com/docs/0J25/all
- ISO 13482:2014, https://www.iso.org/standard/53820.html; ISO/FDIS 13482 listing, https://standards.iteh.ai/catalog/standards/iso/02f350af-bc54-45f5-864e-9f71b2290f41/iso-fdis-13482
- ANSI blog, ISO 3691-4:2023, https://blog.ansi.org/ansi/iso-3691-4-2023-driverless-industrial-trucks/
- Saphira, ISO 3691-4 and R15.08, https://www.saphira.ai/blog/mobile-robot-safety-standards-understanding-iso-3691-4-(driverless-industrial-trucks)-and-r15-08-(industrial-mobile-robots)-implementation, 2025-02-25
- ANSI blog, ANSI/A3 R15.06-2025, https://blog.ansi.org/ansi/ansi-a3-r15-06-2025-robot-safety/; A3, R15.08-2, https://www.automate.org/robotics/news/ansi-a3-r15-08-2-safety-standard-for-industrial-mobile-robot-systems-and-applications-now-available
- ISO/TS 15066 (Annex A, numbers not open), https://docs.roboticks.io/standards/iso-ts-15066

## Verification (adversarial review)

Reviewed 2026-09-07. Method: every claim re-checked against a primary source fetched directly (raw source files, vendor docs, arXiv abstracts), not the note's summaries. Web search quota was exhausted for this session, so search-engine cross-checks were limited to Yahoo/Brave result pages; ISO normative text remains unread.

| claim | verdict | evidence | source URL | corrected claim |
|---|---|---|---|---|
| Nav2 Collision Monitor example: `source_timeout: 5.0`, `stop_pub_timeout: 2.0`, `time_before_collision: 2.0`, `simulation_time_step: 0.1`; zones 0.3 m stop, 0.4 m slowdown at 30%, 0.5 m limit (0.4 m/s, 0.5 rad/s); stale source forces stop; most aggressive action wins | confirmed | Raw params YAML matches every value (`PolygonStop` points 0.3, `PolygonSlow` 0.4 `slowdown_ratio: 0.3`, `PolygonLimit` 0.5 `linear_limit: 0.4` `angular_limit: 0.5`). `collision_monitor_node.cpp`: when `getData()` fails and `source_timeout != 0` the node sets `action_type = STOP`, polygon name "invalid source", warning "Robot to stop due to invalid source". README: "the most aggressive one is used (e.g. stop > slow 50% > slow 10%)". | https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_collision_monitor/params/collision_monitor_params.yaml ; https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_collision_monitor/src/collision_monitor_node.cpp | Add: stale-source stop is disabled when `source_timeout` is 0.0; `stop_pub_timeout` is how long zero-velocity keeps being published after a stop, not a sensor timeout. |
| Shipping-stack timeouts: `diff_drive_controller` `cmd_vel_timeout` 0.5 s; LeKiwi `watchdog_timeout_ms = 500`, `max_loop_freq_hz = 30`, logs "Command not received..." and calls `robot.stop_base()`; ArduPilot Rover `FS_TIMEOUT` 1 s, `FS_GCS_TIMEOUT` 5 s | needs_qualifier | `diff_drive_controller_parameter.yaml`: `cmd_vel_timeout` default 0.5, "0.0 disables the timeout". `config_lekiwi.py`: `watchdog_timeout_ms: int = 500`, `max_loop_freq_hz: int = 30`; `lekiwi_host.py` matches the quoted message and `robot.stop_base()`. ArduPilot: the failsafe docs page says "default = 1 sec", but `Rover/Parameters.cpp` on master and on the Rover-4.5 branch reads `GSCALAR(fs_timeout, "FS_TIMEOUT", 1.5)` (range 1–100 s); `FS_GCS_TIMEOUT` default 5 confirmed in code. | https://raw.githubusercontent.com/ros-controls/ros2_controllers/master/diff_drive_controller/src/diff_drive_controller_parameter.yaml ; https://raw.githubusercontent.com/huggingface/lerobot/main/src/lerobot/robots/lekiwi/lekiwi_host.py ; https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Rover/Parameters.cpp | ArduPilot Rover `FS_TIMEOUT` code default is 1.5 s (docs page is stale at 1 s); `FS_GCS_TIMEOUT` 5 s holds. The 0.2–1 s cluster and "300 ms sits beneath LeKiwi's 500 ms" still hold. |
| Structured output via vLLM `guided_json` / `response_format: json_schema`, XGrammar backend, "since vLLM 0.8.5" | refuted | vLLM structured-outputs docs: `guided_json`, `guided_regex`, `guided_choice`, `guided_grammar`, `guided_decoding_backend` were deprecated and "removed in v0.12.0"; replacement is `"structured_outputs": {"json": ...}` / `StructuredOutputsParams(json=...)`; default backend is `auto`. `response_format: {type: json_schema}` remains supported. v0.8.0 release (2025-03-18) is where V1 became default and V1 structured outputs with xgrammar landed; the v0.8.5 notes only add `structural_tag`. Latest release is v0.28.0 (2026-08-26). The note's "since 0.8.5" traces to the Red Hat article's phrasing, not vLLM's notes. | https://docs.vllm.ai/en/latest/features/structured_outputs.html ; https://github.com/vllm-project/vllm/releases/tag/v0.8.0 ; https://github.com/vllm-project/vllm/releases/latest | On any vLLM >= 0.12 (current is 0.28.x) send OpenAI `response_format: {"type":"json_schema", ...}` or `extra_body={"structured_outputs": {"json": schema}}`; `guided_json` is rejected. V1 structured outputs date from 0.8.0. Keep the "enum skill name at decode time" idea; drop `guided_decoding_backend`. |
| LLM-hazard numbers: physical prompt injection 27.0% GPT-4o / 29.4% Gemini 2.5 Flash / 5.0% Qwen3-VL-32B over 5,670 trials, 99.9% acknowledged, defenses 75–98.9% / 85–100% / 100%; DictAttack 94.3–99.5% on 13 models incl. GPT-5, 75.8% vs guardrails; RIPA 100% Llama-3.3-70B, 0% direct override on Llama-3-8B and Qwen2.5-7B, 10.2% firewall bypass; RoboGuard >92% -> <3% | confirmed | arXiv 2608.05715 abstract (submitted 2026-08-06) gives exactly 27.0 / 29.4 / 5.0%, 5,670 trials, 99.9%, defenses "75–100%", "85–100%", "100%". arXiv 2503.24191 v4 (2026-08-10): DictAttack 94.3–99.5% ASR, 13 models incl. GPT-5, Gemini-2.5-Pro, DeepSeek-R1, GPT-OSS-120B; 75.8% against guardrails. arXiv 2606.28649 (2026-06-26): Llama-3.3-70B 100%, Llama-3-8B and Qwen2.5-7B 0% direct override, 10.2% bypass over 19 obfuscated payloads. arXiv 2503.07885 v2 (2026-03-03): unsafe plan execution >92% -> <3%. | https://arxiv.org/abs/2608.05715 ; https://arxiv.org/abs/2503.24191 ; https://arxiv.org/abs/2606.28649 ; https://arxiv.org/abs/2503.07885 | Minor: the prompt-rule defense is "75–100%" in the abstract (note says 75–98.9%, presumably from the body). 2503.24191 is now titled "When Grammar Guides the Attack: Uncovering Control-Plane Vulnerabilities in LLMs with Structured Output"; the note cites the v1 title. All numbers are single-benchmark, sorting-task or simulation results, not robot-fleet field rates. |
| Pi kernel watchdog: `bcm2835_wdt` hardware max 15 s; Pi kernels lack `CONFIG_WATCHDOG_SYSFS` so `/sys/class/watchdog` status is missing; need `dtparam=watchdog=on` | needs_qualifier | Kernel driver: `PM_WDOG_TIME_SET 0x000fffff`, `WDOG_TICKS_TO_SECS(x) ((x) >> 16)` -> 0xfffff >> 16 = 15 s, `.timeout = WDOG_TICKS_TO_SECS(PM_WDOG_TIME_SET)`. Confirmed. But `bcm2711_defconfig` on rpi-6.12.y contains `CONFIG_WATCHDOG_SYSFS=y` (absent on rpi-6.1.y, which is the kernel in the 2023 forum post). Overlays README on rpi-6.12.y: `watchdog  Set to "off" to disable the hardware watchdog (default "on")`; on rpi-6.6.y it read `Set to "on" to enable ... (default "off")`. | https://raw.githubusercontent.com/raspberrypi/linux/rpi-6.6.y/drivers/watchdog/bcm2835_wdt.c ; https://raw.githubusercontent.com/raspberrypi/linux/rpi-6.12.y/arch/arm64/configs/bcm2711_defconfig ; https://raw.githubusercontent.com/raspberrypi/linux/rpi-6.12.y/arch/arm/boot/dts/overlays/README | 15 s max holds. On current Raspberry Pi OS kernels (rpi-6.12.y) the hardware watchdog is on by default and `/sys/class/watchdog/watchdog0/*` exists; `dtparam=watchdog=on` is only needed on <= 6.6 kernels. `RuntimeWatchdogSec=14` still fits under the cap. |
| Pi 4 rev 1.4 cuts USB port power on every software reboot; unavoidable; `USB_MSD_PWR_OFF_TIME` lengthens but cannot remove it | confirmed | Forum post by timg236 (Raspberry Pi Engineer, 2021-04-03): "external reset is linked to the chip reset so there's no way to prevent USB port power from being turned off at reboot"; `USB_MSD_PWR_OFF_TIME` "can be used to increase the power off time but can't get rid of the reset"; pre-1.4 boards also see "a very brief reset/glitch". No second independent source located (Pi docs pages fetched were truncated before the bootloader section). | https://forums.raspberrypi.com/viewtopic.php?t=308558 | Add: pre-1.4 Pi 4 boards also glitch USB power at reboot, just for less time. Single vendor-engineer source; still the strongest available. The design consequence (power ESP32 separately, boot to motors-off) stands regardless of board revision. |
| ESP32 dev boards reset on host DTR/RTS (RTS->EN, DTR->GPIO0); Linux asserts RTS on unopened ports and can hold the S3 in a reset loop; fix with `stty -hupcl` and >= 1 uF on EN. Issue #18992 (Aug 2026) shows S2/S3 native USB resets "on any falling RTS edge" and USB does not survive reset | needs_qualifier | esptool boot-mode docs confirm EN<-RTS, GPIO0<-DTR, "In Linux serial ports by default will assert RTS when nothing is attached to them. This can hold the ESP32-S3 in a reset loop", `sudo stty -F /dev/ttyUSB0 -hupcl`, and a 1–10 uF EN capacitor. esp-idf issue #18992 (opened 2026-08-18, "Selected for Development", no staff reply) is titled "ESP32-S31 missing USB (OTG) CDC Console support"; it mentions S2/S3 USB-OTG CDC "reset on any falling RTS edge" as background and says "USB does not survive reset" about the S31 rev 0 ROM, not the S3. ESP-IDF S3 USB-Serial/JTAG and USB-OTG console pages make no RTS-reset statement. | https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html ; https://github.com/espressif/esp-idf/issues/18992 | Bridge-chip auto-reset claims hold. Rewrite the #18992 sentence: it is an ESP32-S31 issue; the RTS-edge reset it describes is the S2/S3 USB-OTG CDC console (`CONFIG_ESP_CONSOLE_USB_CDC`), and "USB does not survive reset" applies to the S31, not the S3. The S3's USB-Serial/JTAG peripheral is not covered by that issue; test the actual board with a scope, as the Open questions already say. |
| VL53L1X: short mode ~1.3 m at 50 Hz, medium/long 3–4 m at 30 Hz, 27 deg FoV, 4 cm minimum, 1 mm resolution; timing budget 20 ms minimum | needs_qualifier | Pololu #3415 confirms ~130 cm/50 Hz short, ~300 cm/30 Hz medium, 400 cm/30 Hz long (in the dark), 27 deg FoV, 4 cm minimum, 1 mm resolution. Timing budget: the ST ULD driver (`VL53L1X_SetTimingBudgetInMs`, stm32duino port) accepts `case 15: /* only available in short distance mode */` then 20/33/50/100/200/500; Pololu's full-API library says "The minimum budget is 20 ms in short distance mode and 33 ms for medium and long distance modes". ST's own pages (st.com, UM2356 PDF) timed out. | https://www.pololu.com/product/3415 ; https://raw.githubusercontent.com/stm32duino/VL53L1X/main/src/vl53l1x_class.cpp ; https://raw.githubusercontent.com/pololu/vl53l1x-arduino/master/README.md | Minimum timing budget depends on driver and mode: 15 ms (ST ULD, short mode only), 20 ms (short mode, full API), 33 ms (medium/long, Pololu full API). The 50 Hz figure requires short mode and a <= 20 ms budget; 30 Hz for medium/long implies the 33 ms floor. Adjust the "33 ms sample" in the stopping-distance estimate to 20 ms if short mode at 50 Hz is used (saves ~4 mm; immaterial). |
| ISO 3691-4: 0.3 m/s ceiling for vehicles without personnel detection; 0.3–1.2 m/s by zone; PLd detection able to see a 200 mm test piece | needs_qualifier | Two secondary sources agree on the mechanism but not on the note's framing. fabrico.io (2026-07-09): "where personnel detection is muted or not fully effective, for example when docking, speed must not exceed 0.3 m/s"; test pieces are "a vertical cylinder 200 mm in diameter and 600 mm high, and a horizontal cylinder 70 mm in diameter"; PL "often Performance Level d". Robotnik (2022-01-10): "If these devices cannot work in the movement direction the maximum velocity must be less than 0.3 m/s"; 1.2 m/s appears only as one integrator's chosen max. Saphira confirms the three zones and PLd. Normative text still unread; iso.org and blog.ansi.org return 403. | https://www.fabrico.io/blog/iso-3691-4-driverless-industrial-trucks/ ; https://robotnik.eu/is-it-really-safe-to-share-workspace-between-robots-and-humans-robotnik-in-the-hr-recycler-project/ | 0.3 m/s is the ISO 3691-4 limit while personnel detection is muted or does not cover the direction of travel (docking, reversing), not a class limit for "vehicles without detection". Test pieces: 200 mm x 600 mm vertical and 70 mm horizontal cylinders. "0.3–1.2 m/s by zone" is not a clause of the standard as far as secondary sources show; drop it. The anchor for the rover's 0.3 m/s cap survives, with this wording. |
| ISO 13482:2014 covers mobile servant / physical assistant / person carrier robots and excludes robots faster than 20 km/h; a revision retitled "safety requirements for service robots" reached FDIS in 2025–2026 | confirmed | genorma.com scope text: does not apply to "robots travelling faster than 20 km/h", robot toys, water-borne and flying robots, industrial robots, medical devices, military robots; BSI lists it "Current, Under Review", published 2014. iss.rs project 83498: "ISO/FDIS 13482 Robotics — Safety requirements for service robots", stage 50.00, stage date 2025-07-24. | https://genorma.com/en/standards/iso-13482-2014 ; https://iss.rs/en/project/show/iso:proj:83498 | None. Note remains correct that the numeric force/speed clauses were not readable from open sources. |
| ESP-IDF Task WDT prints a backtrace by default and only resets if `CONFIG_ESP_TASK_WDT_PANIC=y`; subscribe with `esp_task_wdt_add()`, feed with `esp_task_wdt_reset()`; Interrupt WDT panics/resets | confirmed | ESP-IDF stable (S3) WDT page: "On a TWDT timeout the default behaviour is to simply print a warning and a backtrace before continuing running the app"; `CONFIG_ESP_TASK_WDT_PANIC` converts it to panic/reset; `esp_task_wdt_add()` subscribes, `esp_task_wdt_reset()` feeds. | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html | None. Also set `CONFIG_ESP_TASK_WDT_TIMEOUT_S` explicitly (default 5 s is long for a motor loop). |
| systemd `WatchdogSec=`: miss -> `SIGABRT` (or `WatchdogSignal=`), `WATCHDOG_USEC` passed in env, ping every half interval, `Restart=on-failure|on-abnormal|always` (and `on-watchdog`) restarts | confirmed | Debian bookworm systemd.service(5): "the service is placed in a failed state and it will be terminated with SIGABRT (or the signal specified by WatchdogSignal=)"; `WATCHDOG_USEC=` passed to the process; restarts with on-failure, on-abnormal, always. sd_watchdog_enabled(3): "It is recommended that a daemon sends a keep-alive notification message to the service manager every half of the time returned here." | https://manpages.debian.org/bookworm/systemd/systemd.service.5.en.html ; https://manpages.debian.org/bookworm/libsystemd-dev/sd_watchdog_enabled.3.en.html | None. The half-interval rule comes from sd_watchdog_enabled(3), not systemd.service(5). |
| ros-safety `software_watchdogs`: 200 ms heartbeat, 220 ms lease, windowed variant tolerates 3 misses, built on Rolling / Ubuntu 20.04 | confirmed | README: SimpleHeartbeat "200ms", watchdog "lease of 220ms", WindowedWatchdog "maximum of three deadline misses", tested on "ROS 2 Rolling Ridley" with "Ubuntu 20.04"; uses DDS liveliness and deadline QoS. | https://raw.githubusercontent.com/ros-safety/software_watchdogs/master/README.md | None; the note already flags it as dated. |
| Inferred arithmetic: 0.135 J at 3 kg / 0.3 m/s; 4–6 cm stop from ToF trigger; 0.75–1.35 m per LLM turn; ~15 cm on heartbeat loss | confirmed | 0.5 x 3 x 0.3^2 = 0.135 J. Reaction 0.043 s x 0.3 m/s = 1.3 cm; braking v^2/2a = 2.25–4.5 cm; total 3.5–5.8 cm. 0.3 x 2.5–4.5 s = 0.75–1.35 m. 0.3 x 0.3 s = 9 cm + 4–6 cm = 13–15 cm. | (arithmetic) | The braking deceleration 1–2 m/s^2 is an assumption; the Open questions already call for measuring it. |

### Stale or missing

Stale (pre-2025 facts that have since changed):
- vLLM `guided_json` / `guided_choice` / `guided_decoding_backend` request fields were removed in v0.12.0; current vLLM is 0.28.x. Use `response_format: json_schema` or `structured_outputs`. The note's Recommendation table and Corrections section both name `guided_json`.
- "Pi kernels lack `CONFIG_WATCHDOG_SYSFS`" is a kernel-6.1 (2023) observation; rpi-6.12.y ships `CONFIG_WATCHDOG_SYSFS=y`, and the `watchdog` dtparam defaults to "on" on 6.12 (was "off" through 6.6). `dtparam=watchdog=on` is harmless but no longer required.
- ArduPilot's hand-written failsafe page says `FS_TIMEOUT` default 1 s; the code default has been 1.5 s (Rover-4.5 and master).
- esp-idf issue #18992 is about the ESP32-S31 (2026 silicon), not the S3; the note attributes S31 behaviour ("USB does not survive reset") to S2/S3.
- arXiv 2503.24191 was retitled in later revisions; the note cites the 2025 title.
- ros-safety `software_watchdogs` targets Rolling on Ubuntu 20.04 (2020–2021) and shows no recent maintenance; copy the pattern, not the package.

Missing (a 2026 practitioner would insist on):
- Pi 4 UART wiring detail: `/dev/ttyAMA0` (PL011) is bound to Bluetooth by default; the note recommends it without saying `dtoverlay=disable-bt` is required ("On Pis prior to Pi 5 this restores UART0/ttyAMA0 over GPIOs 14 & 15", overlays README). Without it `serial0` is the mini-UART `/dev/ttyS0`, whose baud rate tracks the VPU core clock; a 115200–921600 baud safety link on the mini-UART with a throttling Pi is a known trap. `enable_uart=1` is also needed.
- A hardware-level motor-enable guard independent of firmware: the note relies on Task WDT + pull-down enable. A PWM-presence (retriggerable monostable) or heartbeat-driven enable on the motor driver covers a wedged MCU whose WDT is misconfigured or disabled during debugging.
- Encoder plausibility check on the MCU: commanded vs measured wheel velocity mismatch (stall, lift-off, slip on a rug edge) should fault, not just the current limit.
- ESP32-S3 brownout detector (`CONFIG_ESP_BROWNOUT_DET`) and Pi `get_throttled` under-voltage bits as motion-refusal inputs; the brief's Gate 3 logs `get_throttled`, the note's validator does not gate on it.
- Internal tension: the validator refuses motion if the frame is > 3 s old, but the brief's own turn budget is 2.5–4.5 s (image prefill 0.5–1 s + 1.5–2.5 s decode + Piper). `t_sent_mcu - t_frame_captured` will be 2–3.5 s on normal turns, so a 3 s gate rejects a share of legitimate commands at the upper end. Either capture the frame after STT-final and budget 4 s, or gate motion on ToF freshness and use frame age only for logging.
- `RuntimeWatchdogSec=14` plus `WatchdogSec=10`: state that the two watchdogs must not share a failure mode (systemd itself pets the hardware watchdog; a wedged Python process is caught by `WatchdogSec`, a wedged kernel by `bcm2835_wdt`). The note implies this but never says which watchdog catches which fault.
- Version pin: name the vLLM version and structured-output parameter style the box actually runs, and test that XGrammar handles the Gemma tokenizer with an `enum` skill field; the note lists this as an open question but the Recommendation table presents `guided_json` as settled.

Silent contradictions with the brief:
- Brief Gate 4: "Wi-Fi cut mid-drive -> stop." The note moves stop authority to the Pi<->ESP32 heartbeat, so a Wi-Fi cut no longer stops an in-flight drive; the bounded <= 1 m command completes. The fault matrix lists `nmcli radio wifi off` without stating the expected outcome. Decide and state it: either the Pi cancels active motion on box-link loss (keeps the brief's gate) or the gate is rewritten to "Wi-Fi cut -> current bounded command completes, no new motion".
- Brief Gate 2: "cable pull -> stop <= 300 ms." The note changes this to ~350 ms including braking; it says so in Corrections, but the Layer table still says "Heartbeat timeout 300 ms" without the braking allowance. Make the gate "motion ceases within 300 ms + measured braking time".
- Brief skill bound `drive(<= 0.3 m/s)`: the note lowers the default Pi-side cap to 0.2 m/s and unlocks 0.3 m/s conditionally. Flagged in Bounds, but the brief's skill list and Gate 1 ("10/10 correct skill calls") would need the new unlock condition reflected in the schema.
