# System walkthrough and cleanup record

The cleanup preserves the WAVE ROVER design and finishes several broken integration paths.
The objective is one owner for each decision, explicit execution, and trustworthy tests.
A 50% reduction is a stretch target, not a condition for removing required behavior.
A functionality-preserving 90% reduction is not supported by the source audit.

## 1. Scope and evidence

Baseline: commit `23267ed`, audited on 2026-09-07.
Sections 2–3 describe the updated implementation.
Section 4 separates source from generated output.
Section 5 retains all 32 baseline findings and states their current disposition.
Sections 6–7 record the implementation plan and executed evidence.

The audit traced contracts, Pi processes, firmware actuator paths, simulator,
development tools, deployment, CI, gates and the Android skeleton.
It inspected legacy boundaries without treating the retired controller as current.
It did not treat every generated object or historical research note as active code.

Evidence distinguishes executed checks, source inspection and unverified physical claims.
The implementation has not been deployed or flashed.
No hardware moved. Local logs, models, configuration and live sockets were preserved.
The old `PLAN.md` tracker was removed because it contradicted the current design;
its content remains in Git history. This document is the current cleanup/completion tracker.

## 2. What the system is trying to do

The goal is an indoor robot that accepts an instruction, observes its surroundings, proposes a bounded action, performs it, and reports the result.
The language model does not control motor pins.
The host and controller constrain what can happen when the model is wrong, slow, or unavailable.

The current path uses a WAVE ROVER chassis and driver board to avoid building a custom drivetrain and power system.
The Pi remains the first host because the Python implementation already exists.
Android is a later host replacement, not a prerequisite for testing the chassis.
These are the reasons recorded in [ADR-0013](docs/adr/0013-wave-rover-open-loop.md).

This choice already removed capabilities from the earlier proposed structure.
The current chassis has no wheel encoders.
The current system therefore commands power for time, not measured travel distance.
It uses IMU heading for turns, not encoder odometry.
That is an existing design decision, not a reduction proposed by this document.

The current architecture has four Pi processes:

| Component | Owns | Does not own |
| --- | --- | --- |
| `rover_brain` | Input, conversation state, model calls, scene memory, speech, composed skills | Motor pins or the serial port |
| `rover_cam` | Camera capture and frame publication | Motion decisions |
| `rover_robotd` | Command validation, arbitration, motion profiles, serial link, telemetry, motion budget | Speech recognition or model inference |
| `rover_web` | Browser input, face, telemetry display, teleop controls | Final motion permission |

The GPU box runs the model service separately.
The [box compose file](docker/compose.box.yml) configures vLLM with a 27B model, two-way tensor parallelism, and the OpenAI-compatible endpoint.
It also declares optional STT/TTS services under a speech profile.
Those declarations do not establish that the image tags, model, flags, authentication, or optional speech adapters work together on the real box.
The ESP32 on the driver board applies motor output and controller-side limits.
The physical STOP cuts driver-board power independently of software under the documented wiring arrangement.

```text
Text / Pi microphone / browser PTT
                  |
                  v
Camera -> rover-cam -> rover_brain <-> GPU box
          frames.sock      |
                           | bounded skill + host-owned metadata
Browser / CLI controls ----+----> rover_robotd
                                  robotd.sock
                                       |
                                 JSON lines, 20 Hz
                                       |
                                  ESP32 firmware
                                       |
                                     motors

ESP32 feedback -> robotd -> brain, browser, logs
Physical STOP -> cuts driver-board power independently
```

The browser also has a direct control path to robotd.
Speech and model planning are not prerequisites for a browser stop.

## 3. End-to-end operation

### 3.1 Install and configure the host

The proposed hardware sequence is build firmware, verify the board on the bench, configure the Pi, then enable the complete robot.
[install.sh](hosts/pi/deploy/install.sh) installs system packages, service users, virtual environments, systemd units, a UART rule, and an example configuration.
It enables services without starting the complete target automatically.
[preflight.sh](hosts/pi/deploy/preflight.sh) is intended to check the configuration, camera, audio, UART ownership, and firmware identity before use.
Sync now validates the repository root and destination and supports a non-mutating dry run.
Preflight reads current link/power configuration and uses the installed brain audio environment.
Neither deployment nor physical preflight was run during this cleanup.

The Pi uses separate Python environments for a reason.
The camera environment sees the distribution's camera packages.
The brain environment carries speech dependencies without replacing the camera environment's NumPy.
Piper runs as a subprocess from its own environment.
Combining those environments is not automatically a simplification.

[robot.example.toml](config/robot.example.toml) defines the reference configuration.
[robot.mac.toml](config/robot.mac.toml) supports development.
The local `config/robot.toml` is ignored by Git.
`ROVER__SECTION__KEY` overrides support machine-specific values.
Credentials belong in environment variables named by configuration, not literal configuration values.

The example defaults to text input.
Its hardware camera and Piper settings require installation and configuration.
`make sim` supplies fake adapters and local paths; the example is not independently a hardware-free launch command.

### 3.2 Bring up the controller link

Robotd starts the bus server, link task, validator, arbiter, heading tracker, budget ledger, and logs.
The default hardware link is `/dev/serial0` at 115200 baud.
Only robotd should own that port during normal operation.

The link configures feedback, disables command echo/debug output, and requests the firmware banner.
The patched firmware announces `T:1006` with its firmware identifier, protocol version, heartbeat ceiling, and power cap.
Robotd requires compatible firmware and fresh feedback before admitting motion under the default configuration.
Stock firmware is not an acceptable substitute for the fork's safety behavior.
See [link.py](hosts/pi/rover_robotd/link.py) and [protocol.md](docs/protocol.md).

The wire protocol is newline-delimited JSON without a checksum or controller session identifier.
Invalid JSON can be rejected.
A corrupted line that remains valid JSON is not necessarily detectable.
Host replay checks do not create replay protection on the physical UART.

### 3.3 Acquire an instruction

CLI text and browser text go to `brain.sock`.
Browser PTT sends button events; the Pi captures microphone audio.
It does not upload browser microphone audio in this path.

The speech path consists of capture, voice activity detection, speech recognition, and a transcript.
The intended wake path starts capture while the brain is idle.
Actual speech requires the chosen models, input device, and output device.
Several configured alternative backends remain unimplemented.

The brain assigns a `turn_id` to each instruction.
Its state machine associates planning and execution results with that instruction so an old result cannot silently become a new command.
Speech and listening operations also have their own identities, including STOP speech when no turn exists.
Late playback and transcription callbacks cannot complete or supersede a newer operation, even if cancellation fails.
Transcript length and confidence contribute to motion permission.
The policy treats absent confidence as authorized once the minimum length passes.
This is a policy choice, not proof that the transcript is correct.
See [fsm.py](hosts/pi/rover_brain/fsm.py) and [validate.py](hosts/pi/rover_brain/validate.py).


Explicit CLI speech, face, scene and search requests use a typed `request_skill` message on `brain.sock`.
They do not turn speech text back into an instruction; `say "stop"` means speak that word.
Brain uses the same executors as model-selected skills and returns a result whose `cmd_id` matches `request_id`.
Disconnect or a scoped cancellation cancels only that outstanding request.
Direct CLI motion obtains a camera observation, supplies operator provenance, maintains pings and waits for its own terminal result.
Acceptance alone is not successful execution.

### 3.4 Build the observation supplied to the model

The camera process publishes a JSON frame header followed by JPEG bytes over `frames.sock`.
The brain keeps the newest still instead of accumulating an unbounded image queue.
The separate process keeps camera work outside robotd's control loop.

Robotd publishes state derived from controller feedback.
The brain combines heading, battery estimate, range, stop flags, motion budget, and recent scene information into `WorldState`.
This is not a map or a localization system.
An unknown range is not a measured clear path.
Scene memory holds eight recent sightings in an in-memory ring and atomically writes `scene.jsonl` for inspection.
It does not reload that file after restart.
It is neither SQLite nor durable conversational memory.
See [scene.py](hosts/pi/rover_brain/scene.py).

Frame freshness uses the host's monotonic clock.
Controller heartbeat timing uses the controller's own clock.
The system must not subtract timestamps from those two clocks.
Wall-clock timestamps support logs, not command validity.
See [publisher.py](hosts/pi/rover_cam/publisher.py) and [world_from_state](hosts/pi/rover_brain/main.py).

Planning results carry the exact still's identity and timestamp.
An immutable dispatch effect also carries the instruction ID and captured motion permission.
Dispatch does not replace those values with the newest frame or current conversation state.
A stale or future observation is rejected before dispatch and independently by robotd.
Main-resolution requests now travel over the camera connection and wait for a fresh matching frame.
An explicitly configured fake JPEG keeps its original dimensions and bytes; the brain uses that fixture's dimensions rather than pretending it was resized.

### 3.5 Choose a skill

The local router handles recognized simple instructions before calling the GPU box.
Other instructions go to the model with the system prompt, `WorldState`, the utterance, and an available image.
The box client enforces response deadlines, validates structured output, and permits one corrective retry.
One planning request produces one proposed skill.
This is not an unrestricted autonomous tool loop.

The seven model-facing skills are:

| Skill | Meaning | Executor |
| --- | --- | --- |
| `drive_for` | Apply bounded power for a bounded duration | robotd |
| `turn_to` | Turn toward an absolute heading with a timeout | robotd |
| `stop` | Cancel motion through the stop path | robotd |
| `say` | Speak supplied text | brain |
| `describe_scene` | Obtain and speak a scene description | brain |
| `find` | Observe and make bounded heading turns while searching | brain, using robotd for turns |
| `set_face` | Change the displayed expression | brain/web |

`twist` is a separate teleop stream, not a model skill.
Production configuration disables that stream unless explicitly enabled.

[skills.py](packages/rover_contracts/skills.py) describes skill ownership and bounds.
Pydantic models define the message shapes.
Generated JSON schemas constrain model output.
Schema validity does not establish that an action is appropriate for the scene.
`describe_scene` and `find` use [BoxClient.observe](hosts/pi/rover_brain/box.py) with separate `SceneObservation` and `FindObservation` schemas.
Those observation schemas cannot contain a skill.
The brain must construct any subsequent turn and send it through normal motion admission.
`find` is a bounded local routine, not a workflow engine: at most eight scan positions and two centring turns.
A shared 60-second whole-operation timeout bounds every awaited adapter call in `find`.
Its eight-sweep limit and robotd's motion-time budget remain independent limits.

### 3.6 Validate and announce the proposed action

The brain rejects stale planning results and unauthorized motion.
It normally speaks the proposed intent before dispatching motion.
Stop bypasses that speech wait.
The current conversation design is half-duplex.
It does not continuously recognize spoken STOP while executing motion.

The brain adds `cmd_id`, sequence, `turn_id`, issuance time, deadline, observation metadata, and provenance.
The model does not choose those values.
This separates a proposed action from host permission to execute it.
See [dispatch_bus](hosts/pi/rover_brain/main.py).

### 3.7 Admit one motion command

Robotd independently checks the skill, arguments, operational bounds, source, sequence, replay history, instruction identity, observation age, firmware readiness, feedback age, stop state, arbitration, and budget.
This second validation is necessary because clients other than the brain can reach robotd.
The brain's checks also cannot guarantee that controller state remains unchanged before dispatch.

The arbiter allows one active motion command.
Source policy decides whether a new command starts, preempts, or is refused.
The instruction budget limits commanded motion time to 12 seconds by default.
It measures nonzero command periods, not physical distance or confirmed wheel travel.
See [validator.py](hosts/pi/rover_robotd/validator.py), [arbiter.py](hosts/pi/rover_robotd/arbiter.py), and [budget.py](hosts/pi/rover_robotd/budget.py).

### 3.8 Convert the admitted command into wheel power

`drive_for` applies power for time with 100 ms ramps.
Its default maximum duration is 2 seconds.
Equal requested power does not guarantee equal ground travel across floors or battery states.

`turn_to` compares target heading with fused yaw and applies opposing wheel powers using proportional control.
The default tolerance is 5 degrees for two consecutive samples.
The configured maximum timeout is 4 seconds.
All host callers now use the same positive-left heading convention.
Hardware yaw is converted at one boundary using the configured sign; physical direction still needs verification.
A drive deadline is `1.5 * duration + 0.5` seconds; a turn's timeout is already its complete deadline.
Brain, CLI and robotd use the shared calculation without changing the wire units.

The goal-owning loop sends `{"T":1,"L":l,"R":r}` at 20 Hz.
It sends zeros when idle.
There is no independent keep-alive loop that should continue a stale nonzero command after the goal loop freezes.
See [profiles.py](hosts/pi/rover_robotd/profiles.py) and [main.py](hosts/pi/rover_robotd/main.py).

### 3.9 Apply controller-side enforcement

The current firmware is the patched Waveshare Arduino sketch for the classic ESP32.
The portable ESP32-S3 core is retired under `legacy/firmware-s3/`.

The normal open-loop path clamps wire power to ±0.30 and writes the H-bridges.
The full-scale wire value is ±0.5, so 0.30 represents about 60% PWM duty, not 30%.
Feedback reports applied power, not measured wheel speed.

Firmware clears PWM, requested outputs and PID state directly when the 300 ms heartbeat expires.
A chassis-mode change also clears the previous output.
Serial input has a 512-byte line limit and a per-poll budget of 128 bytes and one line.
Vendor waits service heartbeat, sensors and STOP in short slices, with four queued commands.
Explicit STOP invalidates older queued work.
Queued wheel commands retain their receipt times; a vendor wait cannot refresh or restart expired motion.
These remain cooperative software checks, not a hardware timer or a proven worst-case latency bound.
A single blocking hardware-library call can still delay the next check.

Current sensor behavior is specific:

- A fitted ToF reading below 250 mm blocks pairs where both requested wheel powers are positive.
- The optional bumper applies the same forward block.
- Reverse and rotation remain available under that block.
- A missing or stale ToF does not block forward with the current `BOT_TOF_REQUIRED=0` default.
- The bumper is disabled by default.
- Low battery blocks all motion after 10 seconds continuously below 9.9 V.
- Recovery requires 30 seconds above 10.2 V.

These are not encoder stall detection, cliff detection, or measured stopping-distance guarantees.
See [bot_config.h](firmware/General_Driver/bot_config.h), [bot_safety.h](firmware/General_Driver/bot_safety.h), and [movtion_module.h](firmware/General_Driver/movtion_module.h).

### 3.10 Report completion or stop on failure

Controller feedback returns heading, voltage, applied powers, heartbeat state, and stop flags.
Robotd converts that into state, result, and event messages for the brain and web.
The brain reports completion or refusal and returns to its next conversation state.

Motion can terminate through explicit stop, software e-stop, client loss, stale feedback, expired command validity, exhausted budget, or a controller stop condition.
The default feedback age limit is 150 ms.
The default client ping gap is 400 ms.
Software e-stop latches until an authorized clear.
The normal stop path bypasses ordinary skill parsing, but still applies source policy.

If robotd freezes, the intended final software fallback is the firmware heartbeat.
The physical STOP cuts driver-board power without relying on either process.
The documented Pi power connection keeps the Pi running when that switch opens.
Actual wiring, output during reset, and coast distance still require bench verification.
See [stop handling](hosts/pi/rover_robotd/main.py) and [wiring.md](docs/wiring.md).

### 3.11 Run teleop and retain evidence

The web process offers text/PTT, face, telemetry, and explicit control endpoints.
Teleop uses a renewed wheel-power stream when enabled.
Robotd ends an unrenewed stream and records optional episodes containing commanded powers and observations.
The episode format no longer contains encoder pose.

Runtime command/event/telemetry logs go under the configured log directory.
Local development uses `data/logs`.
Gate results go under `logs/gates`.
`run` contains development sockets and process output.
Those directories serve different purposes even though two contain logs.
Retention and naming can be clarified without discarding evidence.
See [episodes.py](hosts/pi/rover_robotd/episodes.py) and [Makefile](Makefile).

### 3.12 Develop without hardware, then replace the host

`make sim` starts fakebox, `rover-stub`, robotd, a fake camera, text-mode brain, and web.
Fakebox produces deterministic model-compatible responses and fault cases.
The Python rover stub models the current wire protocol and approximate dynamics.
The real host processes run against those replacements.

The stub does not execute the Arduino firmware.
Simulation can test host decisions and modeled failures without establishing physical motor behavior.
The Linux/arm64 development container checks part of the Pi environment, not actual camera access or electrical behavior.
The GPU box's real model quality and latency require separate measurements.

Android currently contains generated contracts, three Gradle modules, placeholder brain/UI objects, a no-op `LinkService`, and a TCP instrumentation test against the stub.
Its README explicitly says it has not been built or run.
The eventual phone must implement input, camera, conversation, link ownership, validation, motion control, lifecycle handling, and UI.
Generated data classes do not implement those behaviors.
See [Android README](hosts/android/README.md), [LinkService.kt](hosts/android/link/src/main/kotlin/dev/rover/link/LinkService.kt), and [gen_kotlin.py](tools/gen_kotlin.py).

## 4. What the size numbers mean

The supplied `tree` contains 2,567 files.
Git tracks 321 files at the audit snapshot.
The generated builds, Python caches, runtime logs, virtual environment metadata, and local configuration in that tree are not all repository source.
The tracked gate-log directory contains only `.gitkeep`.
There is no evidence here that the large build trees were committed.

Local disk measurements include approximately 3.4 GB in `firmware/.cache`, 339 MB in `.venv`, 77 MB in the current firmware build, and 138 MB across the retired firmware builds.
Deleting reproducible outputs could reclaim substantial local space.
It would remove zero tracked application code lines.
Deleting models, logs, or live sockets is a different operation and needs explicit retention/lifecycle rules.

Tracked-only counts use `cloc` 2.04 with comments and blanks separated.
The categories below do not overlap.

| Category | Counted code lines |
| --- | ---: |
| Pi host Python | 6,287 |
| Shared contracts Python | 1,755 |
| Development tools Python | 2,649 |
| Current firmware `General_Driver` | 6,111 |
| Current firmware libraries/examples | 1,573 |
| Current firmware build/patch artifacts | 679 |
| Legacy, excluding Markdown | 6,780 |
| Android generated contracts | 655 |
| Android scaffold/test, excluding Markdown/generated | 272 |
| Tests and recognized fixtures | 14,806 |
| All tracked Markdown | 7,679 |
| Pi deployment scripts/config | 709 |
| Other build/config/schema/tools | 2,027 |
| Total | 51,982 |

`cloc` calls Markdown content "code."
Excluding Markdown leaves 44,303 counted lines, but that still includes configuration, schemas, and 583 patch lines describing changes already present in firmware source.
The tool recognizes 295 of the 321 tracked files.
It excludes binary fixtures, `uv.lock`, systemd units, and other unrecognized files from this count.
The web's HTML/CSS/JavaScript contributes 637 lines within the other category.

The most useful application baseline is 8,042 Python code lines for the Pi host and shared contracts.
Including development tools raises that to 10,691.
Neither baseline includes dependencies installed outside the repository.

### Measured cleanup outcome

Counts compare the final code with the independent checkout of `23267ed`.
They exclude comments and blank lines. No code was downloaded or generated merely to reduce its reported size.

| Scope | Baseline | After cleanup | Change |
| --- | ---: | ---: | ---: |
| Active Python: Pi + contracts + devtools | 10,691 | 11,266 | +575 (+5.4%) |
| Gate code and corpus | 5,236 | 3,598 | −1,638 (−31.3%) |
| All existing test areas and fixtures | 14,806 | 14,074 | −732 (−4.9%) |
| Tests plus new native firmware checks | 14,806 | 14,365 | −441 (−3.0%) |
| All recognized non-Markdown source | 44,303 | 45,027 | +724 (+1.6%) |
| Tracked source/artifact files | 321 | 327 | +6 (+1.9%) |

The 50% target was not achieved.
Gate consolidation removed a second implementation of control and protocol behavior.
Six thin pytest wrapper files became one, and the retired gate-link helper disappeared.
New regressions, explicit request completion, callback identities and firmware intake/expiry checks offset much of that reduction.
Runtime growth is a measured completion cost, not a claimed debloat win.

The shorter README, one current gate map and removal of the stale tracker reduce navigation.
The requested audit document adds documentation; it is excluded from the non-Markdown comparison.
All image/audio fixtures, vendor commands, Android scaffold and legacy reference source remain.
No caches, logs, models or live sockets were deleted.
An unrelated legacy test rename in the working tree is excluded from the commit and comparison.

A larger reduction would require further demonstrated duplication or an explicit narrower product scope.
This pass does not establish that half of the active implementation can disappear without losing behavior.
It does establish a working software baseline against which a later simplification can be tested.

### Can that shrink by 90% without losing functionality?

Theoretically, a different implementation could be much smaller.
This audit does not demonstrate an equivalent implementation or justify promising 90%.

A 90% reduction would leave approximately:

- 804 lines for Pi host and shared-contract Python.
- 1,069 lines when development tools are included.
- 4,430 lines across the broader non-Markdown count.

Those are arithmetic targets, not engineering estimates.
The current first-party code performs input handling, asynchronous cancellation, two IPC services, camera framing, model transport, strict contracts, motion scheduling, replay checks, arbitration, fault handling, logging, diagnostics, and simulation.
The audit found duplicated policy and transport mechanics, not evidence that nine out of ten application lines are unnecessary.

The following do not demonstrate a functionality-preserving 90% simplification:

- Moving vendor code to a downloaded dependency reduces checked-in code but retains the implementation and dependency.
- Generating source from templates reduces handwritten source, not necessarily generated or executed code.
- Removing tests removes verification functionality.
- Removing Android removes the planned host path and its existing scaffold.
- Removing legacy removes local reference/build capability even if the current WAVE ROVER still runs.
- Removing arm/gimbal/vendor commands changes the firmware's accepted command surface.
- Packing statements onto fewer lines reduces line count without reducing behavior or complexity.
- Removing every comment and blank would cut counted physical lines by 26.1%, remove no counted code, and harm the requested understandability.

Under the user's no-functionality-removal constraint, none of those removals counts as an approved shortcut.
A narrower rover-only firmware build could preserve the documented rover protocol, but it would not preserve every vendor feature.
Keep that distinction explicit if the scope is reconsidered later.

The practical target should be fewer independent policies, fewer contradictory interfaces, and a shorter path from an instruction to its result.
Measure any code reduction after proving those changes preserve behavior.

## 5. Baseline findings and current disposition

The paragraphs below describe the audited baseline, not defects presumed to remain.
Each **Current** line records the cleanup outcome or remaining gap.
Evidence links in this section point to the baseline so moved lines cannot misrepresent the finding.

### 5.1 Resolve before trusting hardware motion or deployment

**F01 — Retained firmware commands can defer or bypass the intended heartbeat stop. Source.**

**Current:** Corrected in software: mode-independent stops, cooperative vendor waits and bounded serial work. A separate review also tests deferred motion after heartbeat expiry. Physical timing remains open.

**Baseline:**
`T:111` reaches `RoArmM2_delayMillis`, which calls `delay(inputTime)` during serial dispatch.
A 3000 ms request can prevent the main-loop heartbeat check from running for that interval.
`T:900` can change `mainType` to 3 while output is nonzero.
Heartbeat expiration then calls `setGoalSpeed(0, 0)`, which updates PID targets in that mode while the PID execution loop is compiled out.
Existing motor output need not be cleared through that path.
The normal host does not send these commands, but the firmware still accepts them.
Investigate nonblocking execution while making emergency output clearing independent of mode.
Restricting the commands to a stopped maintenance state changes when they are accepted and must be recorded as a compatibility change.
Removing their handlers is a feature-scope change.
Evidence: [uart_ctrl.h](https://github.com/tedthizzy/bot/blob/23267ed/firmware/General_Driver/uart_ctrl.h#L199), [delay implementation](https://github.com/tedthizzy/bot/blob/23267ed/firmware/General_Driver/RoArm-M2_module.h#L916), [mode-dependent output](https://github.com/tedthizzy/bot/blob/23267ed/firmware/General_Driver/movtion_module.h#L374), [heartbeat](https://github.com/tedthizzy/bot/blob/23267ed/firmware/General_Driver/movtion_module.h#L480), [main loop](https://github.com/tedthizzy/bot/blob/23267ed/firmware/General_Driver/General_Driver.ino#L266).

**F02 — Firmware receive buffering has no line-length bound. Source.**

**Current:** Corrected: bounded line intake and per-poll work; production-header native regressions.

**Baseline:**
`serialCtrl()` appends to a `String` until newline.
The host codec and stub's 512-byte limit does not protect this independent receiver.
An unfinished or oversized input can grow the buffer; sustained input also keeps work inside the serial loop.
Add bounded framing and a per-loop work budget while preserving valid protocol commands.
Evidence: [uart_ctrl.h](https://github.com/tedthizzy/bot/blob/23267ed/firmware/General_Driver/uart_ctrl.h#L504).

**F03 — Brain and robotd disagree on heading direction. Source.**

**Current:** Corrected: one positive-left calculation; router/heading/profile regressions. Hardware yaw sign remains unverified.

**Baseline:**
Brain's `heading_left_of` subtracts leftward degrees because it documents a clockwise heading frame.
Robotd's shared `heading_error_deg` treats positive error as leftward rotation.
`world_from_state` forwards robotd heading without a sign conversion.
At heading 0, a requested left 90 becomes target 270 and negative heading error in robotd, producing the wheel pattern robotd defines as right.
Choose one convention, convert hardware yaw once, and add a router-to-wheel integration test.
Confirm physical direction on the bench rather than treating a `yaw_sign` adjustment as a fix for two conflicting host definitions.
Evidence: [brain heading](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/heading.py#L23), [shared heading](https://github.com/tedthizzy/bot/blob/23267ed/packages/rover_contracts/units.py#L64), [WorldState conversion](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/main.py#L846), [turn output](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_robotd/profiles.py#L132).

**F04 — Motion freshness does not refer to the image used for planning. Source.**

**Current:** Corrected: the immutable instruction carries its actual planning observation through dispatch.

**Baseline:**
Planning sends the image available at `_plan` time.
Dispatch later attaches whichever frame is newest then.
A slow plan can therefore carry a fresh timestamp for a different image.
Carry the planning observation identity through the result and enforce its age at dispatch.
Evidence: [planning](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/main.py#L628), [dispatch](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/main.py#L734).

**F05 — Deployment sync resolves the wrong source root. Source.**

**Current:** Corrected: repository-root validation, destination checks and dry-run isolation; shell regression tests. No live sync run.

**Baseline:**
`hosts/pi/deploy/sync.sh` resolves one parent directory, yielding `hosts/pi` rather than the repository root.
It passes that directory to `rsync --delete` targeting `/opt/rover`.
That can upload the wrong layout and delete destination files absent from that partial source.
Do not run the current sync script.
Correct the root, assert that it contains `pyproject.toml` and the expected packages, review exclusions, and inspect an rsync dry run before deployment.
Evidence: [sync.sh](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/deploy/sync.sh#L34).

**F06 — The secret scanner has the same moved-directory error. Source plus executed scope count.**

**Current:** Corrected: whole publishable-tree scope, redaction and exact upstream-line allowances; malicious changes remain detectable.

**Baseline:**
It changes to `hosts/pi` and invokes `git ls-files` there.
That lists 56 tracked files instead of all 321.
Its root-relative environment/config checks also run from the wrong directory.
The self-exclusion path still says `deploy/check-secrets.sh`.
Correct the repository root and test scanner coverage with safe fixtures outside `hosts/pi`.
Do not treat its current clean verdict as a repository-wide scan.
Evidence: [check-secrets.sh](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/deploy/check-secrets.sh#L21).

**F07 — `.env` files lack ignore rules, and Docker lacks a context boundary. Executed/source.**

**Current:** Corrected: environment ignore rules and private/generated Docker-context exclusions; firmware image uses an empty context.

**Baseline:**
This describes the audited commit; the Current line records the correction.
`git check-ignore` found no rule for `.env` or `box/.env`.
The repository has no `.dockerignore`.
The development image uses `COPY . .` from the repository context.
Git ignores do not exclude local configuration, private notes, logs, virtual environments, or the 3.4 GB firmware cache from that copy.
Add explicit environment-file ignores while preserving `.env.example`.
Add a Docker context allowlist or exclusions for private/runtime/build content.
This audit did not build or publish an image and does not establish that a secret leaked.
The development compose file also bind-mounts the checkout at runtime.
Build exclusions do not hide files in that runtime mount.
Evidence: [.gitignore](https://github.com/tedthizzy/bot/blob/23267ed/.gitignore), [Dockerfile.services](https://github.com/tedthizzy/bot/blob/23267ed/docker/Dockerfile.services#L53).

### 5.2 Complete the migration and restore credible evidence

**F08 — The unit suite cannot complete collection. Executed.**

**Current:** Corrected: migrated link/motion tests collect and the full current unit suite runs.

**Baseline:**
`test_robotd_link.py` imports nonexistent `LinkProtocol` from `rover_contracts.wave_proto`.
`test_robotd_motion.py` imports removed `SerialConfig` and also references the retired serial codec and odometry.
Port the integration expectations to WAVE ROVER rather than deleting those scenarios.
Evidence: [link tests](https://github.com/tedthizzy/bot/blob/23267ed/tests/unit/test_robotd_link.py#L31), [motion tests](https://github.com/tedthizzy/bot/blob/23267ed/tests/unit/test_robotd_motion.py#L33).

**F09 — Active gates still depend on the retired protocol. Executed/source.**

**Current:** Corrected: current corpus, wire checks and gate runners; retained scenarios map to current evidence in docs/gates.md.

**Baseline:**
G1 fails during import on removed `DriveArgs`.
G2, G4, and G6 fail during import on removed `SESSION_WILDCARD` through `gatelib.link`.
Their `--help` paths fail before any gate work begins.
G1 ground truth still expects distance/speed skills.
G2 still measures encoder travel.
G4 fuzz controls still use retired motion fields.
The gate link launches the retired `rover_devtools.mcu_sim` name.
CI still invokes G2/G4 through `make gates-sim`.
Migrate shared transport helpers, valid controls, fault cases, and gate criteria together.
Evidence: [G1 corpus](https://github.com/tedthizzy/bot/blob/23267ed/tests/gates/g1/corpus.py#L44), [gate link](https://github.com/tedthizzy/bot/blob/23267ed/tests/gates/gatelib/link.py#L31), [G2](https://github.com/tedthizzy/bot/blob/23267ed/tests/gates/g2/run_g2.py#L84), [G4 fuzz](https://github.com/tedthizzy/bot/blob/23267ed/tests/gates/g4/fuzz.py), [CI](https://github.com/tedthizzy/bot/blob/23267ed/.github/workflows/ci.yml#L47).

**F10 — Current verification claims exceed the available evidence. Source.**

**Current:** Corrected documentation: withdrawn blanket WAVE passing labels; current results and physical skips remain separate.

**Baseline:**
The WAVE ROVER table labels cases `M` while active gates remain unmigrated.
G4's late-response, reconnect, and box-loss cases include unconditional skips.
The historical `869 passed` record belongs to the superseded v0 section.
Keep planned enforcement, executed software checks, and physical measurements separate.
Require a commit and result artifact for each current measured claim.
Evidence: [verification table](https://github.com/tedthizzy/bot/blob/23267ed/docs/verification.md#L8), [G4 skipped cases](https://github.com/tedthizzy/bot/blob/23267ed/tests/gates/g4/run_g4.py#L1303), [Phase 2 tracker](https://github.com/tedthizzy/bot/blob/23267ed/PLAN.md#L81).

**F11 — Focused tests expose unfinished expectations, not five proven production defects. Executed/source.**

**Current:** Corrected fixtures and assertions. The tiny-negative-angle range defect is also fixed and its former expected-failure test passes.

**Baseline:**
The focused run returned 708 passed, 5 failed, and 1 expected failure.
One turn-profile assertion requires every sample to be nonzero, including the legitimate zero command while waiting for the second in-tolerance sample.
One validator fixture reduces `power_max` without reducing `twist_power`, so configuration correctly rejects the fixture before testing its intended case.
Three parameterized validator failures reuse an accepted `cmd_id` and reach replay rejection before the intended obstacle/budget assertion.
Fix the fixtures and assertions without weakening replay checks, limit ordering, or zero output at the target.
The expected failure separately documents `wrap_deg_360(-1e-20)` returning 360.0 outside its promised interval.
Evidence: [profile test](https://github.com/tedthizzy/bot/blob/23267ed/tests/unit/test_robotd_profiles.py#L105), [validator tests](https://github.com/tedthizzy/bot/blob/23267ed/tests/unit/test_robotd_validator.py#L414), [angle expected failure](https://github.com/tedthizzy/bot/blob/23267ed/tests/unit/test_units.py#L81).

**F12 — Preflight still reads retired configuration and the wrong audio environment. Source.**

**Current:** Corrected and covered by isolated preflight configuration tests.

**Baseline:**
The initial configuration probe reads `c.limits.speed_mps` and `c.serial.port`, which the current schema does not define.
The root audio probe imports `sounddevice` using the base environment, while installation puts speech dependencies in `.venv-brain`.
That probe can fail on the environment the installer intentionally creates.
Use the current `[link]`/power fields and the actual runtime environment for each probe.
Evidence: [configuration probe](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/deploy/preflight.sh#L65), [audio probe](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/deploy/preflight.sh#L151), [environment installation](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/deploy/install.sh#L169).

**F13 — The stub is not firmware-equivalent. Source.**

**Current:** Open modeling limitation: the stub still simplifies battery timing. Native firmware checks supply separate evidence, not equivalence.

**Baseline:**
The Python stub blocks low battery immediately below 9.9 V.
The firmware waits 10 seconds and requires a different recovery threshold and hold time.
The stub also bounds input lines where firmware does not.
Share test vectors and expected timing cases across the implementations, or explicitly mark modeled simplifications.
A passing stub test cannot prove the corresponding Arduino behavior.
Evidence: [stub](https://github.com/tedthizzy/bot/blob/23267ed/packages/rover_devtools/rover_stub.py#L456), [firmware battery logic](https://github.com/tedthizzy/bot/blob/23267ed/firmware/General_Driver/bot_safety.h#L129).

**F14 — Hardware verification remains outstanding. Unverified.**

**Current:** Open: no physical qualification or flashing was performed.

**Baseline:**
The firmware README records "compiled, not flashed."
The repository does not establish physical heartbeat timing, coast distance, STOP wiring, boot/reset motor state, turn accuracy, yaw sign, sensor failure behavior, stall behavior, battery behavior, or Pi thermal/power stability for this chassis.
Optional sensors do not become protection merely because configuration fields exist.
Keep those checks as prerequisites, not cleanup candidates.
Evidence: [firmware status](https://github.com/tedthizzy/bot/blob/23267ed/firmware/README.md#L9), [hardware tasks](https://github.com/tedthizzy/bot/blob/23267ed/PLAN.md#L101).

### 5.3 Reconcile behavior and advertised features

**F15 — Deadline policies disagree. Source.**

**Current:** Corrected: shared drive/turn deadline calculation with independent enforcement.

**Baseline:**
Brain and CLI apply `1.5 * timeout + 0.5` to turns, while robotd uses the turn timeout directly.
At defaults, the brain reduces a 4-second turn timeout to 3 seconds to fit the 5-second TTL ceiling.
Robotd would accept the 4-second turn.
Use one shared pure deadline policy while retaining independent enforcement at robotd.
Evidence: [brain conversion](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/validate.py#L180), [CLI conversion](https://github.com/tedthizzy/bot/blob/23267ed/packages/rover_devtools/roverctl.py#L223), [robotd deadline](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_robotd/validator.py#L440).

**F16 — `roverctl say` reaches an acceptance-only path. Source.**

**Current:** Corrected: CLI brain-owned skills reach brain's actual executors and require a matching terminal result.

**Baseline:**
Robotd accepts non-motion skills without executing them.
Brain normally performs speech after that acceptance.
The CLI sends `say` only to robotd, so acceptance does not cause speech through that path.
Route brain-owned work to brain and preserve CLI behavior with an explicit compatibility path if interfaces change.
Evidence: [robotd acceptance](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_robotd/main.py#L385), [brain execution](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/main.py#L669), [CLI say](https://github.com/tedthizzy/bot/blob/23267ed/packages/rover_devtools/roverctl.py#L313).

**F17 — Main-resolution still requests are not connected to brain. Source.**

**Current:** Corrected: brain requests a fresh main-plane capture and retains its observation identity; actual publisher/client regression.

**Baseline:**
`FrameSource.still(wide=True)` ignores `wide` and returns the latest subscribed still.
The camera publisher supports capture requests, but this client discards the connection writer.
Connect the request/response behavior or describe the actual resolution limitation.
Do not delete the advertised high-resolution path to claim equivalence.
Evidence: [FrameSource](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/main.py#L341), [camera requests](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_cam/publisher.py#L215).

**F18 — Continuous spoken STOP is not implemented. Source.**

**Current:** Open feature: half-duplex speech has no continuous spoken-STOP recognizer. Received stop transcripts now issue STOP before speech or planning.

**Baseline:**
Wake capture runs while IDLE, and listening stops for transcription.
The current execution path has no continuously active stop-word recognizer.
Browser/CLI stop and physical STOP are separate paths.
Do not describe half-duplex input as voice barge-in during motion.
Evidence: [wake loop](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/main.py#L520), [transcription transition](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/fsm.py#L443).

**F19 — Some backend options and future tools are placeholders. Source.**

**Current:** Open features: optional HTTP speech and alternate wake backends, real device/model setup, and offline episode conversion are incomplete.

**Baseline:**
The speech `openai_http` and wake `pymicro` factories raise `ConfigError` despite accepted configuration values.
Real STT/wake operation still requires models and hardware setup.
The episode module references `tools/to_lerobot.py`, which is absent from the tracked tree.
Preserve implemented episode recording and mark conversion as incomplete.
Keep supported, planned, and unavailable options distinct.
Evidence: [speech adapters](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/audio/stt.py), [wake adapters](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/audio/wake.py), [episode contract](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_robotd/episodes.py#L1).

**F20 — Android is a scaffold, not a working alternative host. Source/unverified.**

**Current:** Open feature: Android remains a scaffold. Generated-artifact checks are not an Android build or host-conformance result.

**Baseline:**
`LinkService` does not implement link ownership or motion validation.
The instrumentation test writes directly to the stub rather than exercising that service.
Generated contracts omit Pydantic cross-field validators.
The generator also omits several bus messages, including `TurnMessage`, `PingMessage`, `SubscribeMessage`, and `ClearMessage`.
Its drift check covers the selected models, not a complete phone-host protocol implementation.
The README records no Android build and no committed Gradle wrapper.
Keep the scaffold and its roadmap under the no-functionality-removal constraint.
Do not count it as delivered phone functionality.
Evidence: [Android status](https://github.com/tedthizzy/bot/blob/23267ed/hosts/android/README.md#L17), [cross-field rules](https://github.com/tedthizzy/bot/blob/23267ed/hosts/android/README.md#L76).

**F21 — Power terminology disagrees with the conversion. Source.**

**Current:** Corrected descriptions: power_pct remains hundredths of wire power, not a percentage of the vendor's full scale.

**Baseline:**
`power_pct` is described as percent of full scale, but `power_from_pct(30)` returns 0.30 while full scale is 0.5.
That is about 60% duty.
Make the model-facing description, prompt, CLI help, and conversion agree without silently changing existing wire behavior.
Evidence: [DriveForArgs](https://github.com/tedthizzy/bot/blob/23267ed/packages/rover_contracts/messages.py#L284), [conversion](https://github.com/tedthizzy/bot/blob/23267ed/packages/rover_contracts/skills.py#L66), [wire units](https://github.com/tedthizzy/bot/blob/23267ed/docs/protocol.md#L34).

**F22 — Box-loss detection can exceed its documented timing. Source.**

**Current:** Corrected: probes start immediately and use absolute monotonic cadence, including probe execution time. Four regressions cover failure timing, reset on success and cancellation.

**Baseline:**
`_watch_box` sleeps 0.8 seconds before each probe.
Each probe can itself take 0.8 seconds to time out.
Three consecutive timeout failures can therefore take about 4.8 seconds plus scheduling, not the documented 2.4 seconds.
Other motion deadlines can stop an individual command sooner.
Schedule probes against an absolute cadence and test delayed responses before claiming the 3-second box-loss bound.
Evidence: [watch loop](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/main.py#L720), [probe timeout](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/box.py#L367).

**F23 — Identity enforcement has a permissive fallback. Source.**

**Current:** Open policy: development identity permissiveness and unauthenticated web controls remain unchanged. Deployed identity/binding require explicit configuration review.

**Baseline:**
`ClientSession.bind` warns but proceeds when peer credentials are unavailable or the configured service user cannot be resolved.
That supports local development, but it is not unconditional kernel-backed source authentication.
Keep development permissiveness explicit and verify that deployed motion clients fail closed if required identity checks are unavailable.
The web defaults to loopback and exposes unauthenticated control endpoints; changing its bind address changes that exposure.
Evidence: [client binding](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_robotd/clients.py#L155), [web defaults](https://github.com/tedthizzy/bot/blob/23267ed/config/robot.example.toml#L124).

**F24 — Direct CLI motion does not satisfy robotd's contract. Executed/source.**

**Current:** Corrected: direct CLI motion supplies observation, provenance, pings, matching results and scoped cancellation.

**Baseline:**
`roverctl drive-for` and `turn-to` construct skill messages without observation metadata or authorization trace.
An in-memory check with otherwise-ready controller state returned `obs_stale` with detail `no observation attached`.
The CLI also lacks the ping loop required to keep a longer command alive beyond the 400 ms client gap.
Preserve these commands by supplying the required provenance and liveness behavior, or routing them through a suitable host executor.
Do not bypass robotd validation to make the CLI work.
Evidence: [CLI message construction](https://github.com/tedthizzy/bot/blob/23267ed/packages/rover_devtools/roverctl.py#L240), [observation check](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_robotd/validator.py#L402), [authorization check](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_robotd/validator.py#L215).

**F25 — Fakebox observation calls fail in `json_object` mode. Executed/source.**

**Current:** Corrected: fakebox reads the canonical observation schema in both json_schema and json_object modes.

**Baseline:**
Brain omits `json_schema.name` in this supported compatibility mode.
Fakebox uses that name alone to choose a skill response versus a find/scene observation.
In-memory checks confirmed that both observation requests become skill responses and fail their contracts.
Recognize the requested operation independently of the schema-name field and test both output modes.
Normal `json_schema` observation checks passed.
Evidence: [fakebox dispatch](https://github.com/tedthizzy/bot/blob/23267ed/packages/rover_devtools/fakebox.py#L570), [brain response format](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/box.py#L387).

**F26 — Doctor's box probe omits configured authentication. Source.**

**Current:** Corrected: doctor supplies configured authentication without logging the credential.

**Baseline:**
The probe checks whether the API-key environment variable exists but does not attach its value to the `/models` request.
An authenticated box can therefore produce a false availability warning.
Reuse the model transport's authentication behavior without logging credentials.
Evidence: [doctor probe](https://github.com/tedthizzy/bot/blob/23267ed/packages/rover_devtools/doctor.py#L381).

**F27 — Wirecat's direct serial reader is not a passive monitor. Source.**

**Current:** Corrected: both direct serial modes refuse a live robotd connection; file replay remains available.

**Baseline:**
The live-robotd guard applies only to `--request`.
The read-only path still opens the serial device, changes terminal settings, and consumes bytes from it.
A second reader can take feedback bytes robotd needs.
Provide live monitoring through robotd's bus/logs, and reserve direct serial access for an explicit maintenance mode with robotd stopped.
Preserve the diagnostic capability without implying that two readers see independent copies of the stream.
Evidence: [serial setup](https://github.com/tedthizzy/bot/blob/23267ed/packages/rover_devtools/wirecat.py#L148), [ownership guard and read loop](https://github.com/tedthizzy/bot/blob/23267ed/packages/rover_devtools/wirecat.py#L342).

### 5.4 Reduce maintenance cost without removing behavior

**F28 — Current and historical documentation are interleaved. Source.**

**Current:** Corrected entry points: shorter README, current ADR index, one cleanup tracker and one current gate map. Historical evidence remains labeled.

**Baseline:**
The architecture amendment requires readers to reinterpret a long frozen specification.
`PLAN.md` retains checked v0 claims about odometry, FastAPI, SQLite, speech backends, and build paths that do not describe the current system.
The ADR index omits ADR-0013.
The root quickstart links to a nonexistent `tests/gates/g2_bench.md`.
Write a concise current architecture through an ADR, retain historical documents behind explicit links, and make the current tracker authoritative about incomplete work.
Preserve rationale and evidence rather than deleting research solely to reduce words.
Evidence: [architecture amendment](https://github.com/tedthizzy/bot/blob/23267ed/ARCHITECTURE.md#L5), [tracker](https://github.com/tedthizzy/bot/blob/23267ed/PLAN.md), [ADR index](https://github.com/tedthizzy/bot/blob/23267ed/docs/adr/README.md), [README](https://github.com/tedthizzy/bot/blob/23267ed/README.md).

**F29 — Several policies and transport mechanics have multiple owners. Source.**

**Current:** Partly corrected: instruction permission/provenance, heading, deadlines and runtime permit() now have shared authorities. Similar but differently owned socket loops remain explicit.

**Baseline:**
Brain/robotd socket servers and brain/web reconnecting clients repeat lifecycle and bounded-queue code.
Heading and deadline rules already demonstrate the cost of policy duplication.
Skill bounds also appear in model fields, catalog rows, configuration ceilings, and firmware mirrors; generation and drift checks must cover those representations.
Extract small shared pure helpers and transport mechanics, not a general messaging framework.
Retain independent validation, source checks, queue limits, newest-only state delivery, and stop dispatch.
`permit()` currently has test callers but no runtime callers; consolidate its policy with the runtime path instead of maintaining a parallel tested implementation.
`StartPlanning` and `Dispatch` capture `authorized_motion`, but their handlers ignore that field and dispatch reads mutable FSM state instead.
Make the instruction-scoped permission authoritative through asynchronous work; do not delete the captured value merely because it is currently unused.
Keep robotd's independent authorization check.
Evidence: [brain bus](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/bus.py), [robotd bus](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_robotd/bus.py), [brain validation](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/validate.py#L282), [effect handlers](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/main.py#L573), [dispatch permission](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/main.py#L744), [caps checks](https://github.com/tedthizzy/bot/blob/23267ed/tests/unit/test_caps_match.py).

**F30 — Firmware source and patch reconstruction both need maintenance. Source.**

**Current:** Verified: ordered patches reproduce pinned source, native checks execute production headers, and the actual Arduino sketch compiles. Vendor commands/libraries were retained.

**Baseline:**
The build compiles the already-patched source tree.
It does not apply the patch series during each build.
Keeping both is useful only if reconstruction is checked and the series stays synchronized.
SCServo remains a real dependency because setup and handlers reference it.
Several other library includes appear unused, but removing their pins requires checking all supported build configurations.
Choose a tested vendoring/reconstruction policy without removing vendor functionality under the current constraint.
Evidence: [build.sh](https://github.com/tedthizzy/bot/blob/23267ed/firmware/build.sh), [patch policy](https://github.com/tedthizzy/bot/blob/23267ed/firmware/patches/README.md), [sketch setup](https://github.com/tedthizzy/bot/blob/23267ed/firmware/General_Driver/General_Driver.ino#L140).

**F31 — Build/deploy reproducibility and cleanup boundaries need clarification. Source.**

**Current:** Partly corrected: clean/distclean preserve runtime evidence; clean CI obtains only the checksum-pinned ArduinoJson headers needed for native tests. CMake and Ninja have left the development group; the retired controller's host build is reached by checking out the tag `v0-pi-sim`, as `legacy/firmware-s3/README.md` directs. Pi/container dependency resolution is still not fully lock-driven.

**Baseline:**
CI uses `uv sync --frozen`, but deployment and the development image use `uv pip install -e` rather than installing from the committed lock resolution.
They can resolve different dependency versions.
The default development group still includes CMake/Ninja for the retired controller's host build.
Those tools could move to a legacy-only environment while preserving that build capability.
`make clean` removes `run`, which may contain live sockets and useful logs.
`distclean` removes `data`, including models and runtime evidence.
Define cache cleanup separately from service shutdown, log retention, and model removal.
Do not run broad cleanup while services are active.
Evidence: [dependency group](https://github.com/tedthizzy/bot/blob/23267ed/pyproject.toml#L53), [installer](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/deploy/install.sh#L177), [image install](https://github.com/tedthizzy/bot/blob/23267ed/docker/Dockerfile.services#L57), [Makefile](https://github.com/tedthizzy/bot/blob/23267ed/Makefile).

**F32 — `find` does not enforce its claimed hard 60-second deadline. Source.**

**Current:** Corrected: shared whole-find timeout includes slow adapter awaits; cancellation regressions exercise it.

**Baseline:**
The local routine checks the clock between awaits.
It does not bound an already-running capture, observation, or turn by the remaining total time.
The docstring's claim that a slow box cannot stretch the scan past that deadline is therefore stronger than this implementation.
The outer FSM separately defaults to a 20-second execution timeout.
Define the intended whole-operation deadline and test delayed adapters and cancellation before consolidating these timers.
Evidence: [find loop](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/skills_local.py#L177), [centring](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/skills_local.py#L217), [FSM timeouts](https://github.com/tedthizzy/bot/blob/23267ed/hosts/pi/rover_brain/fsm.py#L350).

### 5.5 Additional findings during implementation

**F33 — The locked SDK does not provide the separately imported HTTP package. Corrected.**
Instantiating the actual OpenAI transport failed on `import httpx`.
The transport now uses the installed SDK's public `Timeout` export without adding another HTTP dependency.
The existing cancellation regression now exercises successful construction.

**F34 — Late speech and transcription callbacks could advance a different instruction. Corrected.**
STOP speech had no turn ID, allowing its late completion to dispatch a newer intent.
A transcription task could also outlive an explicit request that superseded it.
Speech and listening IDs now identify the exact operation.
Tests use cancellation-resistant adapters rather than assuming task cancellation is sufficient.

**F35 — Clean CI lacked the native test's ArduinoJson headers. Corrected.**
Native tests now have an explicit minimal setup command with an official archive and SHA-256.
The Arduino build and native runner share one version pin.
Tests fail when the dependency is missing; setup is not a silent runtime fallback.

**F36 — Two retained vendor arm functions have missing-return warnings. Open vendor limitation.**
The final Arduino build reports `RoArmM2_shoulderJointCtrlRad` and `maxNumInArray` paths without return values.
No arm feature was removed or redefined during this cleanup.
Wheel-path native tests do not qualify those arm operations or their hardware timing.

## 6. Refined implementation plan

The user authorized implementation, one final commit and a push after verification.
Preserve the seven skills, teleop, all current adapters, four process boundaries,
vendor commands, diagnostics, simulator and recoverable legacy reference/build capability.
Known defects are corrected intentionally; preserving behavior does not mean preserving defects.

1. Reproduce the baseline failures and inventory source independently of build output.
2. Keep the existing FSM and carry instruction permission and observation provenance through asynchronous work.
3. Correct mode-independent firmware stopping and bounded receive/wait handling.
4. Migrate obsolete tests to current semantics and share real boundary tests instead of duplicate gate implementations.
5. Repair CLI execution, camera requests, deployment paths, diagnostics and SDK compatibility.
6. Consolidate documentation and proven duplicate fixtures/wrappers without removing distinct evidence.
7. Run software checks, independent source review, firmware compile and isolated end-to-end positive-motion/STOP tests.
8. Commit the verified changes once and push the current branch. Record unavailable hardware and platform checks explicitly.

No protocol-version change, heartbeat-duration change, motor-unit conversion,
new language, generic state-machine framework or process collapse is part of the cleanup.
Safety checks at separate trust boundaries remain independent.
A future rover-only vendor command surface or retirement of the Pi/legacy host requires an explicit scope decision.

## 7. Verification and handoff record

These are the results for commit `354256d`. Section 8 supersedes the counts below
for the current tree; this section is kept unedited as the record of that commit.

Verification uses a clean locked Python 3.13.2 environment outside the cloud-backed checkout.
The original local environment and its data were preserved.
Checks use temporary sockets, OS-assigned loopback ports and owned subprocesses.
No live deployment or hardware service was contacted.

| Check | Executed result | Limit |
| --- | --- | --- |
| Full current unit suite | 1,375 passed, 0 skips, 0 expected failures in 62.45 s | Python 3.13.2; no physical devices |
| Strict mypy | No issues in 55 active Python files | Optional third-party hardware imports need their actual platform environments |
| Ruff and shell syntax | Passed | Not runtime or hardware evidence |
| Schema and Kotlin generation | Up to date after regeneration | Kotlin data classes do not implement validators or compile the Android app |
| Runtime dependency closure | No ML packages in the base runtime | Optional speech dependencies are separate |
| Whole publishable-tree secret scan | Passed with redacted, exact upstream provenance handling | Pattern-based scan, not proof that every possible secret format is recognized |
| Native firmware | ASan/UBSan production-header checks pass | Fake clock/GPIO; blocking hardware-library calls unmeasured |
| Arduino firmware | Final actual compile passes: 887,533 bytes flash; 50,332 bytes globals | Not flashed; two retained vendor missing-return warnings |
| Patch reconstruction | All nine patches reproduce the final sketch byte-for-byte with zero fuzz | Pinned upstream d308df91b333a513f45a238bef9ba9a0a5edf64c |
| G1 selftest | 6 criteria pass; 5 model-dependent criteria skip | Does not establish real model quality, injection resistance or latency |
| G2 full software sweep | 12 criteria pass; 10 physical criteria skip | Current WAVE semantics, not old encoder/CRC guarantees |
| G4 software faults | 23 criteria pass; 4 physical criteria skip | Includes actual owned robotd process freeze and raw-wire positive controls |
| G5 / G6 software | G5: 6 pass, 1 physical skip; G6: 2 pass | Physical find calibration and real episode conversion remain open |
| Full gate aggregate / public execution path | 11 passed, 1 skipped in 76.34 s | G3 physical soak skipped; individual gates also report their physical criteria as skips |

The SDK regression exposed an import of `httpx` although the locked OpenAI SDK
uses a different HTTP package. The transport now imports the SDK's public
`Timeout` export, without adding a redundant dependency.
The OpenAI Docs skill guided that compatibility check against the installed SDK
and [official Python reference](https://developers.openai.com/api/reference/python).

Run from a locked development environment:

```sh
make test
make lint typecheck schema-check kotlin-check deps-check secrets
bash firmware/tests/run.sh --setup
make gates-full
bash firmware/tests/run.sh
make firmware
git diff --cached --check -- . ':(exclude)firmware/patches/*.patch'
```

This run selected the clean environment with `PY` in the Make commands.
JUnit records are `/tmp/bot-final-units.xml` and `/tmp/bot-final-gates.xml`.
The end-to-end case executes CLI, brain, robotd and camera process entry points;
real fakebox, WAVE stub and web server implementations use isolated loopback ports.
It confirms direct CLI and model-planned motion, matching completion IDs, zero
output afterward, speech/face/scene/find execution, main-plane images, HTTP STOP
and refusal of later motion while stopped. TTS is null; no audible result is claimed.
The source whitespace check excludes literal patch files: their context records
upstream whitespace. Zero-fuzz, byte-for-byte reconstruction validates those artifacts.

Physical work still required: STOP power isolation, reset output, worst-case firmware
stop latency, coast distance, IMU sign/accuracy, fitted sensor failures, battery
hysteresis, stall current and Pi thermal/power stability.
Real speech, real GPU model quality/latency, Android build/behavior and episode conversion
remain separate completion work, not results inferred from passing simulation.

## 8. Structure and performance audit (2026-09-08)

Section 7 records commit `354256d`. Two commits landed after it, so the counts
above are that commit's, not this one's. This section records the audit pass and
restates the numbers; nothing in section 7 was edited.

Six auditors read the tree and measured the running system under three structure
lenses and three performance lenses. Each actionable finding was then attacked by
two independent skeptics, one looking for what the change would break or lose and
one judging whether it made the repo simpler or merely smaller. Only findings both
skeptics cleared were applied. Twenty-one of the eighty-one agents died mid-run on
a model quota and were re-run from cache; the completed pass had no errors.

### The defect

State reached the browser and brain at 7.8 Hz against a configured 10, with 150 ms
gaps, and the feedback log ran at 4.4 Hz against 5. Both decimators compared
elapsed time against the period (`main.py` `_publish_state`, `log.py`
`RobotdLog.feedback`) on a 50 ms tick grid, so roughly half a millisecond of wake
jitter made every second window miss its deadline and be skipped. Reproduced
deterministically: the same predicate fed a 20 Hz grid with 0-0.6 ms of jitter
yields 7.57 Hz and a p50 gap of 149.72 ms, and with zero jitter yields 9.98 Hz.
Both now bucket the timestamp by period, so a tick either lands in a new bucket or
it does not and jitter cannot move it. `subscribe.state_hz` was also accepted and
ignored.

### What was removed

Twenty-two tracked files, taking the tree from 327 files in 79 directories to 305
in 63: fourteen vendor example sketches for a bus-servo library this chassis never
drives and no build compiles, a five-line re-export shim, a three-file package
collapsed into one module, a redundant gate test configuration, two inert typing
markers, and a placeholder for a directory the gate runner creates itself.

The audit's own answer to whether the count was excessive: mostly it was not. The
firmware carries a vendored sketch whose nine patches must reconstruct it byte for
byte, the tests track one file per module, and the frozen architecture cites every
research note and decision record it rests on.

### Performance, measured

The control path is not CPU-bound. With a stub rover, three state subscribers, a
10 Hz teleop stream and episode recording, robotd used 1.65% of one M4 core
against 1.16% idle, 40 MB resident, with cProfile placing 96.8% of forty seconds
inside the event loop's wait and no function above 1% of wall. Per-tick and
per-request work measured 2-11 microseconds. The 20 Hz command stream measured at
the stub had p50 50.00 ms, p99 52.5 ms, max 59 ms, against the architecture's
requirement of p99 under 100 ms. A Pi 4 at five to eight times slower implies
roughly 6-13% of one core under the same load, which G3a on the Pi remains the
measurement that settles.

### Executed results at this commit

| Check | Executed result | Limit |
| --- | --- | --- |
| Full unit suite | 1,377 passed in 65.20 s | Python 3.13.2; no physical devices |
| Strict mypy | No issues in 52 source files | Three fewer files than section 7; the removed modules |
| Ruff and shell syntax | Passed | Not runtime or hardware evidence |
| Schema and Kotlin generation | Up to date, no regeneration diff | Kotlin classes carry no validators and compile nothing |
| Runtime dependency closure | No ML package in the base runtime | Optional speech dependencies are separate |
| Secret scan | Clean across 305 published files | Pattern-based, not proof against every secret format |
| Native firmware tests | Pass: motor stop, bounded serial intake, cooperative waits | Fake clock and GPIO; blocking library calls unmeasured |
| Arduino firmware compile | 887,533 bytes flash, 50,332 bytes globals | Not flashed; two retained vendor warnings |
| Full gate suite | 11 passed, 1 skipped in 76.73 s | G3 physical soak skipped; each gate reports its own physical skips |

### Not applied, and why

Three measured findings survived both skeptics only in narrowed form and were left
for a decision rather than applied blind. The acknowledgement tone is awaited
before the model request is spawned, adding about 200 ms to every box turn and
contradicting the comment above it. The OpenAI SDK costs roughly 580 ms of import
at each service start and its streaming path does not reuse connections. The
camera runs its stream encoder unconditionally although nothing consumes stream
frames. Each has a cost and a behavioural consequence that is not free to change.

### Environment

Tool caches now live beside the interpreter rather than in the checkout. This tree
sits inside a synced folder that evicts unread files and mints `name 2` conflict
copies; one such copy corrupted a branch reference and broke fetching, another
stalled the unit suite for over ten minutes on a single cached bytecode file, and
a third made mypy fail with an internal error until the cache was deleted by hand.
Moving the repository off the synced folder is the better fix and has not been
done.
