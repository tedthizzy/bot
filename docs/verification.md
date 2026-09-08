# Verification

Current software evidence and remaining gaps live in
[debloat.md](../debloat.md).
The [gate map](gates.md) connects retained scenarios to current tests.

## WAVE ROVER status

The migration now has executable unit, boundary and simulated fault checks.
Native C++ tests exercise production firmware headers with a fake clock and GPIO.
The Arduino fork compiles, and its ordered patches reproduce the pinned source.
Neither native tests nor the Python stub execute the assembled robot.

The earlier WAVE table's blanket `M` labels were not measured evidence and have
been withdrawn. In particular, simulation does not prove all-process stop timing,
model-image injection resistance, reset output levels, sensor failure behavior,
battery hysteresis, stalled-wheel current or physical power isolation.
The stub simplifies battery behavior. Firmware still uses cooperative checks,
so a blocking hardware call can delay them. Physical qualification remains open.

The 24 historical invariant identifiers remain useful references.
ADR-0013 changes their meaning where the topology changed: no encoder distance,
no CRC/session wire contract, no custom relay or encoder stall detector.
Use named current checks and their explicit limits rather than transferring old
passing counts to this chassis.

The transcript below is retained unchanged as historical evidence.

---

## v0 — ESP32-S3 controller (superseded, tag `v0-pi-sim`)

Integrator's record, 2026-09-07, on a MacBook Air M4 (macOS, arm64), no hardware
attached. Every command below was executed in this repository and every quoted
output is a real tail of that run, not a reconstruction. Where something was
**not** run, this document says so and says what it would take.

The claim this file supports is narrow and deliberate: **the whole rover runs in
simulation on one laptop, and 18 of the 24 safety invariants have a green,
reproducible test behind them there.** Five need hardware — the same five A3
names. The twenty-fourth, I-19, has the mechanism and no measurement, and
section 7 says so. A3 predicts 19 before hardware; this is 18, and the gap is
named rather than rounded up.

Environment: `.venv` is Python 3.12.13; `cmake` 3.31.10 and `ninja` come from the
venv (no Homebrew toolchain is required); the C core builds under host clang with
`-fsanitize=address,undefined`.

---

## 1. `make test` — unit, contract, and the C core

```
$ make test
```

```
828 passed in 45.85s
ninja: no work to do.
Internal ctest changing into directory: .../firmware/host/build
Test project <repo>/firmware/host/build
    Start 1: rover_host_tests
1/1 Test #1: rover_host_tests .................   Passed    0.19 sec

100% tests passed, 0 tests failed out of 1
```

828 Python tests, 0 skipped, 0 failed. The one test that used to skip — the
robotd link driving the real simulator over a pty — runs now that the host build
emits `mcu-sim` under the name `rover_devtools.mcu_sim` searches for.

The C half, run directly so its case list is visible:

```
$ firmware/host/build/rover_host_tests
PASS golden vectors agree with the Python codec
PASS the compiled caps are the numbers ARCHITECTURE states
PASS the controller boots disarmed and refuses motion until hello then arm
PASS a session change after reboot refuses motion
PASS TTL expiry stops the motors
PASS a duplicate or stale sequence does not renew the TTL
PASS a bad CRC is dropped, counted and does not renew the TTL
PASS an over-cap velocity is clamped and flagged
PASS the ramp limit holds
PASS an obstacle blocks forward while reverse and rotation still work
PASS stale ToF is treated as blockage
PASS a fault latches and only clears on an explicit clear once the cause is gone
PASS the PI controller converges
PASS garbage and mid-frame resets never crash the parser
14 cases, 0 failed assertions
```

**The golden vectors are shared, not duplicated.** `firmware/host/CMakeLists.txt`
compiles `ROVER_VECTORS_PATH` as an absolute path to
`tests/contract/serial_vectors.jsonl`, the same file `tests/unit/test_serial_codec.py`
reads. All fifteen frames of ARCHITECTURE 5.1 decode field-for-field and re-encode
byte-for-byte in **both** codecs, `rover_crc16("123456789")` is `0x29B1` in both,
and `rover_safety_hash()` computed in C from the seven compiled constants equals
`3381018647` — the value `config/robot.mac.toml`'s `[safety]` mirror keys produce
and `make doctor` prints.

## 2. `make lint` and the other static gates

```
$ make lint
.venv/bin/python -m ruff check .
All checks passed!
shell syntax ok

$ make schema-check
schemas up to date

$ make secrets
scanning 234 files
clean: no secret, LAN address, MAC address or home path in what git publishes

$ make deps-check
runtime closure clean: no ML package
```

`make typecheck` (mypy, strict) was **not** run to completion: on this
iCloud-backed checkout a full run does not finish in a usable time. This is
recorded as unverified rather than claimed.

## 3. `make sim` — the whole rover, no hardware

```
$ make sim
rover simulation — config config/robot.mac.toml, logs in run/

  start fakebox
  start mcu-sim
  ok    fakebox on :8000
  ok    mcu-sim pty at run/mcu.pty
  start robotd
  ok    robotd bus at run/robotd.sock
  start cam
  ok    cam frames at run/frames.sock
  start brain
  ok    brain bus at run/brain.sock
  start web
  ok    web on :8080

  All six up.
```

All six processes: the fake box endpoint, the C simulator behind a pseudo
terminal, robotd, the fake camera, brain in text mode, and web on `:8080`.
`make doctor` against the same running system:

```
  ok    config                      config/robot.mac.toml; safety_hash=3381018647, serial.backend=pty
  ok    mcu-sim                     .../firmware/host/build/mcu-sim
  ok    serial device               run/mcu.pty -> /dev/ttys007: char device, rw, mode crw--w----
  ok    socket robotd.sock          run/robotd.sock accepting, mode srw-rw----
  ok    socket frames.sock          run/frames.sock accepting, mode srw-rw----
  ok    socket brain.sock           run/brain.sock accepting, mode srw-rw----
  ok    box                         http://127.0.0.1:8000/v1/models serves ['rover-vlm']
effective limits: speed_mps=0.3 default=0.2 drive_m=1.0 turn_deg=180.0 rate_dps=60.0
                  budget=1.5m/12.0s twist=0.3mps/1.047radps stream=['teleop']
16 ok, 7 warn, 0 fail
```

The seven warnings are all "Pi only" or "optional": `picamera2`, `sounddevice`,
`sherpa_onnx`, `$ROVER_BOX_API_KEY`, `box_caps.json`, group `rover`, and
`serial_asyncio_fast` (pinned by ARCHITECTURE 12, imported by nothing in v1).

## 4. One full voice turn, end to end

```
$ .venv/bin/python -m rover_devtools.roverctl --config config/robot.mac.toml \
      utter --watch 20 "drive forward half a meter"
fsm     PLANNING
fsm     SPEAKING_INTENT
fsm     EXECUTING
fsm     SPEAKING_RESULT
fsm     IDLE
```

### 4.1 The utterance, the proposed skill, and validation passing

`run/brain.log`, one INFO line per decision point:

```
14:14:39,882 INFO rover.brain utterance: {"source": "cli", "text": "drive forward half a meter", "confidence": null}
14:14:40,784 INFO rover.brain plan: {"origin": "router", "skill": "drive", "args": {"distance_cm": 50, "speed_cms": 20}, "speech": "Going forward.", "authorized_motion": true}
14:14:40,786 INFO rover.brain speak: {"text": "Going forward."}
14:14:43,407 INFO rover.brain dispatch: {"cmd_id": "01M1YMPMJFKXNXNH7VYF4K4257", "turn_id": "01M1YMPH4A9W17FE43J4PGWW8G", "skill": "drive", "args": {"distance_m": 0.5, "speed_mps": 0.2}, "goal_ttl_ms": 4250, "obs": "cam-000016"}
```

Reading the chain: the router (not the box) answered, in integers as A11
requires — `distance_cm 50`, `speed_cms 20`. `authorized_motion` is `true`
because a `null` confidence counts as authorized (ARCHITECTURE 7). The intent
sentence *finishes playing* before dispatch: `speak` at 40.786, `dispatch` at
43.407, a 2.6 s gap that is macOS `say` actually speaking — this is A31's
dispatch-after-intent rule, observable. `goal_ttl_ms=4250` is the T2 estimate
`0.5/0.2 × 1.5 + 0.5 = 4.25 s`, computed from the profile and not from a
constant. `obs` names a real camera frame.

robotd's own decision record, `data/logs/commands-20260907.jsonl` — validation
passing is the `accepted`:

```json
{"t_utc_ns":1788808468322297000,"decision":"stop","source":"brain"}
{"t_utc_ns":1788808483408830000,"decision":"accepted","cmd_id":"01M1YMPMJFKXNXNH7VYF4K4257","reason":"","detail":""}
{"t_utc_ns":1788808486270975000,"decision":"done","cmd_id":"01M1YMPMJFKXNXNH7VYF4K4257","reason":"","detail":""}
```

The leading `stop` is brain's restart contract (4.3): a stop goes out on connect
before anything else can.

`data/logs/events-20260907.jsonl` for the same window:

```json
{"kind":"mcu_restart","detail":{"old_session":0,"new_session":40010,"reason":"port open"}}
{"kind":"session_reset","detail":{"arg":40010,"mcu_us":261425}}
{"kind":"cal_stored","detail":{"arg":98,"mcu_us":1631112}}
{"kind":"arm_ok","detail":{"arg":1492397617,"mcu_us":16929585}}
```

`cal_stored` is the MCU's own 50-sample cliff baseline at 98 mm, matching
`[safety] cliff_baseline_mm`; `arm_ok`'s `arg` is the `A` frame's random nonce
echoed back in `K.echo`.

### 4.2 The velocity frames on the wire

`make sim LOG_LEVEL=debug` puts every encoded down-frame in `run/robotd.log`.
This is the only way to see the wire while I-18 keeps robotd the sole writer of
the port, so `wirecat` cannot be attached to a live link.

```
14:14:43,408 tx $A,2,19,40010,1492397617*A48F
14:14:43,420 tx $V,2,20,40010,25,0,300,0*FCDE
14:14:43,470 tx $V,2,21,40010,50,0,300,0*67D9
14:14:43,520 tx $V,2,22,40010,75,0,300,0*CEED
14:14:43,571 tx $V,2,23,40010,100,0,300,0*E0FE
14:14:43,620 tx $V,2,24,40010,125,0,300,0*73A7
14:14:43,670 tx $V,2,25,40010,150,0,300,0*C68E
14:14:43,720 tx $V,2,26,40010,175,0,300,0*1DC8
14:14:43,770 tx $V,2,28,40010,200,0,300,0*7A3A
...
14:14:45,470 tx $V,2,63,40010,200,0,300,0*4EC8
14:14:45,520 tx $V,2,64,40010,200,0,300,0*74D7
...
14:14:46,168 tx $V,2,78,40010,157,0,300,0*9264
14:14:46,218 tx $V,2,79,40010,122,0,300,0*274D
14:14:46,270 tx $S,2,80,40010,0*C4B3
14:14:46,271 tx $V,2,81,40010,0,0,300,0*87A0
```

Every line is ARCHITECTURE 5.1 grammar: `$` body `*` CRC-16/CCITT-FALSE `\n`,
version 2, one seq counter per direction, session 40010, integers only. The
trapezoid is visible and correct: 25 mm/s per 50 ms step is exactly
`accel_mps2 = 0.5`; cruise at 200 mm/s is `speed_cms 20`; `frame_ttl_ms` is 300
on every frame; `flags` is 0. The stream is 20 Hz (`setpoint_hz`). The goal ends
with `S` mode 0 and then zero `V` frames.

### 4.3 Simulated odometry advancing

From the robotd bus (`roverctl state --json`), 23 `state` messages carried this
command; every fourth:

```
pose.x_m=0.000  twist=0.000 m/s  ticks=(0,0)        mcu=ARMED_IDLE   progress=0.00 deadline_in_ms=4238
pose.x_m=0.020  twist=0.142 m/s  ticks=(158,158)    mcu=ARMED_MOVING progress=0.04 deadline_in_ms=3788
pose.x_m=0.109  twist=0.209 m/s  ticks=(850,850)    mcu=ARMED_MOVING progress=0.22 deadline_in_ms=3338
pose.x_m=0.217  twist=0.192 m/s  ticks=(1692,1692)  mcu=ARMED_MOVING progress=0.43 deadline_in_ms=2788
pose.x_m=0.316  twist=0.191 m/s  ticks=(2461,2461)  mcu=ARMED_MOVING progress=0.63 deadline_in_ms=2288
pose.x_m=0.418  twist=0.185 m/s  ticks=(3251,3251)  mcu=ARMED_MOVING progress=0.84 deadline_in_ms=1788
```

Pose is integrated on the Pi side from raw encoder ticks, which is A7 and
principle 7. The MCU's own view, from the decimated telemetry log — `v_cmd`
tracking robotd's trapezoid, `v_meas` following the plant, ticks advancing:

```
T mcu_us=17139740 ack_seq=23 state=3 v_cmd=89  v_meas=28  ticks=(16,16)      fault=0x0 motion=1
T mcu_us=17549573 ack_seq=33 state=3 v_cmd=200 v_meas=216 ticks=(396,396)    fault=0x0 motion=1
T mcu_us=18380670 ack_seq=50 state=3 v_cmd=200 v_meas=192 ticks=(1692,1692)  fault=0x0 motion=1
T mcu_us=19631158 ack_seq=76 state=3 v_cmd=200 v_meas=186 ticks=(3622,3622)  fault=0x0 motion=1
T mcu_us=19840948 ack_seq=81 state=3 v_cmd=41  v_meas=153 ticks=(3927,3927)  fault=0x0 motion=1
```

`ack_seq` rising in step with the `V` seq numbers is the protocol's own
acknowledgement of the velocity stream (5.1: "`V` is not acked; `T.ack_seq` at
50 Hz is the acknowledgement"). `fault` stays 0 throughout.

### 4.4 The goal completing, and the spoken result

```json
{"type":"result","cmd_id":"01M1YMPMJFKXNXNH7VYF4K4257","seq":1,"status":"accepted","reason":"","detail":null}
{"type":"result","cmd_id":"01M1YMPMJFKXNXNH7VYF4K4257","seq":1,"status":"done","reason":"",
 "detail":{"traveled_m":0.4944152838419976,"turned_deg":null,"duration_ms":2862,
           "odom_delta":{"x_m":0.4944152838419977,"y_m":0.0,"yaw_rad":0.0},
           "speed_clamped_to_cms":null}}
```

0.494 m travelled against a 0.500 m goal, in 2862 ms, inside the 4250 ms
deadline; `speed_clamped_to_cms` is null because 0.20 m/s is under the cap in
force. And brain's side:

```
14:14:46,272 INFO rover.brain result: {"cmd_id": "01M1YMPMJFKXNXNH7VYF4K4257", "status": "done", "reason": "", "detail": {"traveled_m": 0.4944152838419976, "duration_ms": 2862, "odom_delta": {"x_m": 0.4944152838419977, "y_m": 0.0, "yaw_rad": 0.0}}}
14:14:46,272 INFO rover.brain executed: {"skill": "drive", "status": "done", "reason": ""}
14:14:46,272 INFO rover.brain speak: {"text": "Done."}
```

The spoken result is `"Done."`, from the fixed completion table, played in
SPEAKING_RESULT — after the executor reported, never before (A31). The FSM then
returns to IDLE.

## 5. Gates

```
$ .venv/bin/python -m pytest tests/gates -m gate -s
G1: PASS (INCOMPLETE) [fake]     7 passed, 0 failed, 4 skipped in 0.9s
G2: PASS (INCOMPLETE) [mcu-sim] 11 passed, 0 failed, 10 skipped in 15.6s
G2: PASS [mcu-sim]               2 passed, 0 failed, 0 skipped in 10.6s   (the --full flag sweep)
G3: PASS (INCOMPLETE) [mac]      3 passed, 0 failed, 11 skipped in 62.0s
G4: PASS (INCOMPLETE) [sim]     19 passed, 0 failed, 7 skipped in 39.5s
G5: PASS (INCOMPLETE) [sim]      4 passed, 0 failed, 3 skipped in 0.3s
G6: PASS (INCOMPLETE) [sim]      2 passed, 0 failed, 2 skipped in 2.4s
7 passed in 133.14s
```

48 gate cases pass, 0 fail, 37 skip. `make gates` (the fast G2+G4 subset) and
`make gates-full` both exit 0. Every gate writes a JSONL metrics file into
`logs/gates/`, so each number below has a file behind it.

The gates were run against a live `make sim`; the cases that spawn their own
simulator use `run/gate-mcu.pty` so they cannot pull the port out from under it.

### 5.1 G4 — fault injection. Every case shows the robot did not move.

| case | invariant | result | what it measured |
|---|---|---|---|
| G4-a | I-5 | **PASS** | obstacle at 231 mm: TOF_STOP set, **peak forward `v_cmd` 0 mm/s**, reverse clamped to exactly −150 mm/s, `\|w_cmd\|` clamped to exactly 500 mrad/s, state never entered FAULT — all on the same telemetry frame |
| G4-b | I-8 | **PASS** | 500 fuzz cases at the contract layer: 428 refused, 72 legal controls accepted |
| G4-b2 | I-8 | **PASS** | the same corpus through a live robotd: 423 rejected, **motion published: False** |
| G4-b3 | I-8 | **PASS** | `hello` as `brain` then a `teleop` twist → `source_not_allowed`, **motion=False** |
| G4-b4 | I-8 | **PASS** | a frozen teleop stream: **no motion published 450 ms after the last twist** |
| G4-c | I-9 | **PASS** | all 510 G1 records: 0 bound breaches, 0 accepted without a validated skill |
| G4-e2 | I-11 | **PASS** | a skill carrying a superseded `turn_id` → `stale_turn`, **motion=False** |
| G4-f | I-12 | **PASS** | verbatim resend → `stale_seq`; the same `cmd_id` at a fresh seq → `duplicate_cmd`. The repeat never executes |
| G4-h1 | I-14 | **PASS** | `rover-robotd` SIGSTOPped 1202 ms: **motion published: False** |
| G4-h2 | I-14 | **PASS** | `rover-brain` SIGSTOPped 1203 ms: **motion published: False** |
| G4-h3 | I-14 | **PASS** | `rover-cam` SIGSTOPped 1202 ms: **motion published: False**, next skill `obs_stale` |
| G4-i | I-15 | **PASS** | ten drives in one turn: 1.4 m accepted against `welcome.budget_path_m=1.5`, then `budget_exceeded` |
| G4-i17 | I-17 | **PASS** | grep over `firmware/` and `rover_robotd/`: only `O.echo_pi_mono_us` crosses |
| G4-j | I-16 | **PASS** | 65534 → 600 cm + `front_at_max`; 65535 → null. Never 6553 |
| G4-j2 | I-16 | **PASS** | front-L dead, front-R reporting a clear 2000 mm: TOF_STALE set, **peak forward `v_cmd` 0 mm/s, peak `\|v_meas\|` 0 mm/s**, `b8 tof_fl_ok=False b9 tof_fr_ok=True` |
| G4-l | I-22 | **PASS** | 11 stop-class mutations dispatchable on the raw type, before parsing |
| G4-l2 | I-22 | **PASS** | robotd never answered one `rejected`: 11 accepted or ignored |
| G4-l3 | I-22 | **PASS** | a `clear` from `brain` refused (`error source_not_allowed`), next skill still `estop_active` |
| G4-m | I-23 | **PASS** | an observation older than `obs_max_age_ms=5000` → `obs_stale` |

Every one of the nineteen asserts the negative directly — either `motion=False`
read off published `state`, or `v_cmd`/`v_meas` read off the wire. None of them
infers "did not move" from a rejection reason alone.

### 5.2 G2 — the MCU bench in simulation

| case | invariant | result |
|---|---|---|
| G2-brake | open item 1 | **PASS** 0.79 m/s² over 338 ms and 61 mm at 300 mm/s (A21's floor is 0.6); 0.68 m/s² at 150 mm/s, recorded not gated |
| G2-a | I-1 | **PASS** TTL bit in 302 ms (≤350), `v_cmd` zero in 452 ms (≤500), \|v\| < 10 mm/s in 553 ms (≤829), travel 133 mm (≤200) |
| G2-b | I-2 | **PASS** corruption raises `rx_drop` 0→3 and `ack_seq` does not move (44→44) |
| G2-b-codec | I-2 | **PASS** 6 invalid forms plus replay and a foreign session, all dropped and counted |
| G2-c | I-3 | **PASS** boots DISARMED, `v_cmd` 0 after an unarmed `V` |
| G2-c2 | I-3 | **PASS** `T.SESS` 40010→40011 under a still-open fd: re-seed + `H` + `A` accepted in 1727 ms (≤2000) |
| G2-d | I-4 | **PASS** 900 mm/s clamps to 300, `CAP_CLAMPED` raised |
| G2-d2 | I-4 | **PASS** all 256 `V.flags` values: worst `v_cmd` 300 mm/s. No bit widens anything |
| G2-d3 | I-4 | **PASS** no `config_set` path exists in the firmware |
| G2-h | I-20 | **PASS** travel after the hang 323 mm (≤400), banner `reset_reason` set, `v_cmd` 0 after the reboot |
| G2-i | I-24 | **PASS** `app_main` drives GPIO 4/5/6/7/12/21/39 low before peripheral init |

Braking was measured **first**, as ARCHITECTURE 13 requires, because I-1's own
time criteria derive from it.

### 5.3 G1, G3, G5, G6

* **G1** (510 requests through `rover_devtools.fakebox`): 100% schema-valid
  (510/510), 0 commands above the cap in force at dispatch, the Pi-side
  deterministic controls hold on every trial (bound 0, budget 0, unauthorized
  motion 0), a low-confidence transcript refuses all 23 motion skills
  `unauthorized_utterance` while `say` stays permitted, and no
  `FindObservation` may carry a model-supplied `bearing_deg`. The injection
  attack-success rate is measured and reported with a CI — **39/60 = 65.0%
  (95% CI 52.4–75.8%)** — and is a SKIP, not a pass, because fakebox is a regex
  router with no defence and I-21 forbids asserting zero. Skill and argument
  accuracy are likewise measured (85.3%, 75.0%) and SKIPped: they are claims
  about a model, and there is no model here.
* **G3a** in simulation: telemetry age p99 29 ms (≪ 200 ms), 0 arm/disarm cycles
  per hour, and utterance→SPEAKING_INTENT p50 0.89 s over 43 turns. That last
  one is a **proxy**, named as one in every record: a gate outside the process
  cannot hear audio, so it is not ARCHITECTURE 9's EOS→first-audio metric.
* **G5**: the Pi-computed bearing is within 0.04° over eight placements at
  `hfov_deg = 83.0`, and reads 45% high at 120° — which is why `hfov_deg` is a
  config key. A `FindObservation` refuses `bearing_deg`, `angle_deg`,
  `distance_m` and `skill`. Stop-utterance to a stationary MCU: 51 ms, reported
  and not gated (A29).
* **G6**: `allow_stream` contains `teleop` in the Mac profile, and `SERVO_EN`
  asserts on `V.flags` b1 and drops 272 ms after the last `V`.

---

## 6. Invariant status

**M** = verified in simulation on this machine, with the gate case named.
**R** = verified by code review only, no executing test.
**H** = requires hardware.

| # | invariant | status | evidence |
|---|---|---|---|
| I-1 | Motion requires a `V` accepted within `frame_ttl_ms` | **M** (+H for the real plant) | G2-a: TTL bit 302 ms, `v_cmd` 0 at 452 ms, \|v\|<10 mm/s at 553 ms, travel 133 mm. Deceleration on a simulated first-order plant, not a real chassis |
| I-2 | Bad CRC / stale seq / foreign session / unknown type / bad length / wrong version dropped, counted, TTL not renewed | **M** | G2-b, G2-b-codec, and `rover_host_tests` |
| I-3 | Boots DISARMED; no motion until `H`+`A`; a `T.SESS` change is handled like a port open | **M** (+H for a power cycle) | G2-c, G2-c2 |
| I-4 | Compiled caps cannot be raised by any frame, for any `V.flags` value | **M** | G2-d, G2-d2 (all 256 values), G2-d3 |
| I-5 | Obstacle/bumper/cliff/stale ToF zeroes forward, clamps \|w\| to 500 and reverse to 150 | **M** (+H for the coverage band and the halt distance) | G4-a on one telemetry frame; `rover_host_tests`. G2-bumper, G4-a2, G4-a3, G2-halt need hardware |
| I-6 | E-stop is a hardware coil break | **H** | G2-e and G2-rail skip. Nothing about relay contacts is simulable |
| I-7 | Low battery warn → refuse → disable on `V_oc`, debounced, never suspended | **H** | G2-f skips. The `vbat=<volts>` injection exists and the ladder is exercised by the core's own tests, but the 60 s sustained-load case needs a bench supply |
| I-8 | robotd rejects unknown skills, out-of-bounds, non-finite, extra fields, oversize TTL, stale seq — for `twist` too | **M** | G4-b (500 cases at the contract layer), G4-b2 (the same through a live robotd), G4-b3, G4-b4 |
| I-9 | Box output reaches the MCU only as a validated skill | **M** | G4-c over all 510 G1 records; G1-d |
| I-10 | Per-wheel slip/stall trips within 250 ms; per-channel I²t trips on its own | **H** | G2-g and G4-d skip. `stall_one_channel` and `slip_one_channel` injections exist, but the 250 ms and the I²t timetable are claims about a real motor |
| I-11 | A late model response cannot start motion | **M** (e2 only) | G4-e2 PASS (`stale_turn`, motion=False). **G4-e, the 8 s late-response race, is an unimplemented skip** |
| I-12 | A repeated `cmd_id` cannot execute motion twice | **M** | G4-f: verbatim resend `stale_seq`, same id at a fresh seq `duplicate_cmd` |
| I-13 | Serial reconnect never replays motion | **M** by unit test, **not** by gate | `tests/unit/test_robotd_link.py` covers the re-seed, the reconnect and the seq-desync replay. **G4-g is an unimplemented skip** |
| I-14 | Killing or freezing a process has a defined, per-process effect | **M** (`-STOP`) | G4-h1/h2/h3 all PASS with motion=False. The `kill -9` half is covered by the `-STOP` half plus `Restart=always`, not measured separately |
| I-15 | ≤1.5 m and ≤12 s of motion per instruction | **M** | G4-i: 1.4 m accepted against `welcome.budget_path_m`, then `budget_exceeded`. Read from `welcome`, not the file, as A33 requires |
| I-16 | Stale sensors are blockage; a no-target return is a clear path | **M** | G4-j (the rendering) and G4-j2 (one forward sensor dead, forward refused) |
| I-17 | No host timestamp is compared against the MCU clock | **M** | G4-i17 greps `firmware/` and `packages/rover_robotd/`; only `O.echo_pi_mono_us` crosses |
| I-18 | Only robotd writes the port; a release build has no writable console | **R + H** | Code review: robotd's `Link` is the only writer, and `sdkconfig.defaults` sets `CONFIG_ESP_CONSOLE_NONE=y`. G3-a-fuser needs `/dev/rover-mcu` and `psmisc`; the three-interface probe needs the board |
| I-19 | Box-link loss cancels an in-flight brain goal within 3 s; teleop unaffected | **R** | `_watch_box` implements 3 × `health_probe_s` = 2.4 s and is spawned only for a brain-owned motion command; the teleop path never reaches brain. **G4-k is an unimplemented skip** — the latency is not measured |
| I-20 | A control-loop hang reboots into DISARMED with motors off within 1 s + reset | **M** | G2-h: travel after the hang 323 mm (≤400), banner `reset_reason` set, `v_cmd` 0. See deviations entry 4 on FAULT vs DISARMED |
| I-21 | An injected instruction cannot exceed the budget, raise a bound, or move without a fresh authorizing utterance | **M** | G1-e (bound 0, budget 0, unauthorized motion 0 on every one of 510 trials) and G1-auth. The ASR itself is reported with a CI, never asserted |
| I-22 | Stop-class accepted from any allow-listed source in every state, never `rejected` | **M** | G4-l, G4-l2, G4-l3 |
| I-23 | An observation older than `obs_max_age_ms` cannot authorize motion | **M** | G4-m, and G4-h3 through a frozen `rover-cam` |
| I-24 | Driver inputs and `MOTOR_EN` low from the reset edge through `app_main` | **R + H** | G2-i checks the source: every named pin appears and `gpio_set_level(..., 0)` is called before peripheral init. G2-i2 needs a meter across the driver output |

**Eighteen invariants are green in simulation**: I-1, I-2, I-3, I-4, I-5, I-8,
I-9, I-11, I-12, I-14, I-15, I-16, I-17, I-20, I-21, I-22, I-23 by a gate case,
and I-13 by `tests/unit/test_robotd_link.py` rather than by a gate — the
coverage is real, the named gate case G4-g is not.

**Five are hardware-only**: I-6, I-7, I-10, I-18, I-24. That is exactly the five
A3 names, and no more.

**I-19 is the honest gap.** `_watch_box` implements 3 × `health_probe_s` = 2.4 s
and is spawned only for a brain-owned motion command, so the mechanism reads
correctly and teleop is exempt by construction. But G4-k has no implementation,
so nothing measured the 3 s. 18 + 5 + I-19 = 24.

---

## 7. What could not be verified without hardware

Grouped by what it would take. Nothing in this list is claimed as working.

**Needs the assembled rover.**

1. **E-stop electrics (I-6, G2-e, G2-rail).** That the coil break is a hardware
   path, that firmware can open the relay and cannot close it against an open
   button, that `ctrl_flags` b1 tracks the button with `MOTOR_EN` both asserted
   and deasserted, and that the MDD3A input stays under 16 V when the contacts
   open at 0.30 m/s. Needs the wiring, a scope and a person to press the button.
2. **The battery ladder (I-7, G2-f).** 11.5 → 9.4 V on a bench supply, with and
   without a stall, including the 60 s sustained 3 A load where the 9.9 V refusal
   must still fire; then the `UNDERVOLT_D` shutdown → poweroff → rail-cut order.
3. **Stall and over-current (I-10, G2-g, G4-d).** The 250 ms per-wheel stall
   trip with one wheel held, the 500 ms slip branch, and the per-channel I²t
   trip at a 3.5 A channel over its own seconds-scale timetable.
4. **Driver pin state through reset (I-24, G2-i2).** Zero motor current on
   assert-reset, power-cycle and entry to the download bootloader, with a meter
   or an LED across the driver output.
5. **Only robotd writes the port (I-18, G3-a-fuser).** `fuser -v /dev/rover-mcu`
   showing one pid, and probing USB-Serial/JTAG, UART0 and UART1 on a release
   build.
6. **Braking deceleration and obstacle-to-halt distance (G2-brake on a real
   chassis, G2-halt).** The simulated plant gives 0.79 m/s², above A21's 0.6
   floor, but that is a property of the model, not of carpet. **A21's fallback
   (stop 350 mm / slow zone 800 mm / hard cap 250 mm/s) triggers on the measured
   halt *distance*, which cannot be simulated at all** — half its error budget
   is detect latency.
7. **`R_pack`, `k_e` and `R_motor` (G2-rpack).** Measured once at G2 by stepping
   a known current. `rover_config.h` ships derived values.
8. **The ToF coverage band (G4-a2) and a real table edge (G4-a3).** Where the
   forward cone stops seeing a 70 mm cylinder at the chassis corner, and travel
   past a real cliff trigger point.
9. **Bumper wiring (G2-bumper).** Either switch alone, and a cut harness, all
   raising `BUMPER` — the series-NC-with-pull-down fail-safe OR.
10. **Thermals, clocks, rails, RSS and the 30-minute soak (G3a, G4-soak).**
    `vcgencmd get_throttled == 0x0`, ARM clock at `arm_freq` under load, SoC
    below 70 °C, header voltage ≥ 5.0 V, RSS below 1.6 GB, `rover-cam` against
    A1's core budget.
11. **`V` inter-frame jitter on a loaded Pi (G3-a-vgap, open item 3).** Nothing
    outside robotd can time the frames, because robotd owns the port. It needs a
    robotd-emitted writer metric or a logic analyser.

**Needs a microphone, a speaker, or recorded speech.**

12. **The STT bake-off and WER (G3b-stt, A27).** WER cannot be synthesized.
    Needs `tests/fixtures/audio/bakeoff/` populated with real recordings, and
    `tools/bakeoff_stt.py`, which does not exist.
13. **Piper first-audio latency (G3b-tts, open item 4)** and whether
    `piper --output-raw` actually streams (open item 10). The Mac runs
    `tts.backend="say"`.
14. **The wake word (G3b-wake, A30).** Needs a microphone and a self-trained
    `rover.tflite` (open item 11).
15. **ARCHITECTURE 9's real latency metric.** EOS → first audio. G3's proxy
    measures utterance-accepted → SPEAKING_INTENT and says so in every record.
16. **The stop word during motion (G5).** Measured here as text in at 51 ms to a
    stationary MCU. The spoken path adds wake, VAD and STT, and A29 counts none
    of it anyway.

**Needs a real model, or the GPU box.**

17. **G1-b and G1-c (A38).** ≥95% correct skill and ≥90% args in range are
    claims about a 27B VLM. Against fakebox they measure 85.3% and 75.0% and are
    recorded as SKIP so a fake can never look like a model measurement.
18. **G1-f, the injection ASR (I-21).** 65.0% (95% CI 52.4–75.8%) against a
    regex fake with no defence. The real number needs the real model and the
    real system prompt.
19. **G1-i, the prefix cache (A17).** fakebox reports `cached_tokens = 300` on a
    repeat, which proves the plumbing and nothing about vLLM. SKIP.
20. **A15's detection/counting spread at int4 with thinking disabled.**
21. **Whether the box supports `oneOf` and `json_schema` (open item 8).**
    `box_probe` answers it in 30 s on deploy day.

**Needs Docker, or a machine with disk headroom.**

22. **The ESP-IDF firmware has never been compiled.** `firmware/main/` is
    verified only by a mechanical cross-check that every `rover_*` symbol and
    every struct field it touches exists. The ESP-IDF API surface —
    `mcpwm_prelude.h`, `pulse_cnt.h`, `i2c_master.h`, `gptimer.h`,
    `adc_oneshot`/`adc_cali` — is unchecked, and that is where a first-compile
    error is most likely. `firmware/docker/build.sh release` on a machine with
    disk headroom is the check. This host is at 98% of a 926 GB volume.
23. **`make firmware`, `make flash`, `make docker-dev`, `make box`.** Not run.
24. **`deploy/install.sh`, `preflight.sh`, `sync.sh`.** `bash -n` only; they
    refuse to run off a Pi without `ROVER_FORCE=1`.
25. **Whether `espressif/idf:v5.5.5` publishes a linux/arm64 manifest**
    (open item 13) and whether `vllm serve` accepts
    `--default-chat-template-kwargs`.

**Not verified for other reasons, and said plainly.**

26. **`make typecheck`.** mypy strict does not finish in a usable time on this
    checkout. Not run to completion, not claimed.
27. **G4-e, G4-g, G4-k.** Three named gate cases are unconditional skips in
    `run_g4.py` with no implementation. I-11's other half, I-13 and I-19 are
    covered by unit tests and code review instead, and I-19's 3 s latency is not
    measured by anything.
28. **`tools/to_lerobot.py` and `tools/bakeoff_stt.py`** are named in
    ARCHITECTURE 12 and do not exist. G6-convert and G3b-stt skip naming them.
    Both belong to the G6 merge track and to G3b, neither of which is v1.
29. **The `text_in_frame` speed cap.** Measured and reported (30/30 trials at or
    below the default cap), never counted as a control — its only input is a
    field the model writes, which is A12's own principle.

---

## 8. Reproducing this

```bash
make test          # 828 Python tests + the C core under ASan/UBSan
make lint          # ruff over everything, bash -n over deploy/
make schema-check  # box/schema/*.json against rover_contracts
make secrets       # nothing publishable leaks
make sim           # the six processes; Ctrl-C stops them
# in another shell, with the sim up:
.venv/bin/python -m rover_devtools.roverctl --config config/robot.mac.toml \
    utter --watch 20 "drive forward half a meter"
make gates-sim     # the fast G2 + G4 subset, against a rover it boots itself
make gates         # the same selection against a rover already running
make gates-full    # all seven gate scripts
make doctor        # resolved config, devices, sockets, import map
```

`make sim LOG_LEVEL=debug` adds every encoded down-frame to `run/robotd.log`,
which is section 4.2's evidence. Gate metrics land in `logs/gates/*.jsonl`.

---

## 9. Review fixes, 2026-09-07 — what was changed and what was re-run

A review of the whole tree raised 12 critical/major invariant and correctness
findings, 10 major operability findings and 6 major simplicity findings. All of
them were applied except the two named at the end of `docs/deviations.md`'s
"Review fixes" section, which are argued rather than skipped. Every fix that
changes behaviour has a test that fails without it.

### The four that could stop the robot

**An escalated obstacle was a one-way trip.** Escalation set a private boolean
no `C` frame could name, and the boolean also blocked the five-clean-sample
self-clear of every obstacle bit — so the bit could never go, and the escalation
that keyed on it could never go either. Two ordinary wall approaches, or one dead
forward ToF for 30 s, latched the rover into FAULT until a power cycle, with
robotd still publishing `ready:true`. Escalation now raises
`ROVER_FAULT_OBSTACLE_LATCHED` (0x100000, inside `ROVER_FAULTS_LATCHED`), the
guards are gone, and `ClearableFault.OBSTACLE_LATCHED` gives an operator `clear`
a defined recovery.

**A failed INA226 read powered the host off.** `electrical_poll()` published 0 mV
on a failed read, the A25 ladder integrated it for 10 s and latched
`UNDERVOLT_W|S|D`: `MOTOR_EN` low, `PI_SHUTDOWN_REQ` high, the Pi's rail cut 60 s
later — on a full battery, because one I2C read failed. The snapshot now carries
`ok` + `stamp_us`, a stale reading publishes 65535, the ladder freezes, `V` is
refused with reason 15 `sensors_stale`, and `E I2C_ERROR` names the sensor.

**`OVERCURRENT` could never be cleared.** `cause_persists` read the latched I²t
integral, which has no decay term, so the reset behind it was dead code and A24's
designed protection trip bricked the controller.

**`cal_valid` could stick false for ever.** The 50-sample cliff window only
advanced while DISARMED and was discarded on the next entry, so an arm inside
1.5 s of a `D` left forward silently zeroed — no fault, no `K`, no `E`. The
window now accumulates in any state, and robotd refuses to arm while `ctrl_flags`
b6 is clear.

### `make test`

```
$ make test
851 passed in 49.60s
...
    Start 1: rover_host_tests
1/1 Test #1: rover_host_tests .................   Passed    0.43 sec
100% tests passed, 0 tests failed out of 1
```

828 → 851 Python tests, and 14 → 20 C cases. The six new C cases are the four
above plus the `E I2C_ERROR` producer and the obstacle bits' self-clear while
escalated:

```
$ ./firmware/host/build/rover_host_tests | tail -7
PASS an escalated obstacle recovers on an explicit clear
PASS obstacle bits still self-clear while escalated
PASS overcurrent clears once the current is gone
PASS a stale INA read never advances the undervoltage ladder
PASS the cliff calibration survives an arm inside the sample window
PASS a ToF bus error emits I2C_ERROR with its sensor index
20 cases, 0 failed assertions
```

### Each behavioural fix, falsified

Every one of these was run with the fix reverted and observed to fail, then run
again with it in place:

| fix | test | without the fix |
|---|---|---|
| wildcard session is a resync trigger | `test_the_reseed_falls_back_to_one_when_no_telemetry_arrives` | `AssertionError: condition not reached within the timeout` — the session is never learned, `arm()` never fires, no `V` is ever emitted |
| `odom_delta.yaw_rad` is wrapped | `test_a_turn_across_the_heading_wrap_reports_the_swept_angle` | `AssertionError` at the `turned_deg` comparison: −270.07° reported for a +89.93° turn |
| the box stream is closed | `test_abandoning_the_stream_early_closes_the_http_response` | `assert False … _FakeStream.closed` — the HTTP response, and its pooled connection, stay open |
| the four C cases above | `firmware/test/test_core.c` | written against the pre-fix behaviour and failing on it; see the case bodies |

The remaining tests are new coverage where none existed: `wirecat --ping`'s
second-writer guard and its read-only fd, robotd's arm precondition on
`CAL_VALID`, the sag-compensated `oc_v`, `state.budget`, the configured `twist`
bounds, the parse rejection reaching a sender that did not subscribe, the
episode recorder's fsync moving off the control loop, brain's reconnecting
robotd client, and `HalfDuplexTts` cutting a sentence short when the wheels
start.

### `make lint`, `make schema-check`, `make secrets`

```
$ make lint
All checks passed!
shell syntax ok

$ make schema-check
schemas up to date

$ make secrets
scanning 230 files
clean: no secret, LAN address, MAC address or home path in what git publishes
```

`box/schema/skillcall.json` is now generated from `rover_brain.box.SKILL_CALL_SCHEMA`
— the object brain actually puts in `response_format` — rather than from
`skill_call_adapter.json_schema()`, whose `$ref`/`discriminator` shape production
never sends. G1 scores the request body the robot sends, and `schema-check` guards
the one copy that ships.

### `make gates-sim` — the new target, and why it exists

`make gates` starts no rover. Every G4 case whose precondition is "something is
listening" therefore skipped, `run_gate` maps "nothing ran" to a pass, and eleven
enforcement cases never executed while CI stayed green. The new `gates-sim`
target boots the same six processes `make sim` does, runs the gates against them
and tears them down; `.github/workflows/ci.yml` runs it in place of `make gates`.

```
$ make gates-sim GATES_SIM_ARGS='-k "g2 or g4"'
  ok    fakebox on :8000
  ok    mcu-sim pty at run/mcu.pty
  ok    robotd bus at run/robotd.sock
  ok    cam frames at run/frames.sock
  ok    brain bus at run/brain.sock
  ok    web on :8080

running the gates against the simulated rover
...                                                                      [100%]
3 passed, 4 deselected in 69.85s (0:01:09)
```

G4's own record, from `logs/gates/g4-20260907T211441Z.jsonl`:

```
{"gate":"G4","record":"summary","mode":"sim","verdict":"PASS (INCOMPLETE) [sim]",
 "passed":19,"failed":0,"skipped":9,"elapsed_s":42.556,"exit_code":0}
```

**19 passed against 7** on the same script with no rover up. The nine skips are
the seven hardware-only cases plus the two `SIGKILL` halves, which are skipped
with a named reason: they need a supervisor that restarts the unit, and
`make sim` does not restart what it started.

`run_g4.py` now also refuses to report a clean pass over a hole. Twelve
sub-gates are tagged **M** in section 8; when any of them skips for a missing
peer the script exits 2 with no rover up (pytest reads that as a skip, not a
pass) and 1 with one up:

```
$ python tests/gates/g4/run_g4.py ; echo "exit=$?"
G4: PASS (INCOMPLETE) [sim]  7 passed, 0 failed, 21 skipped in 7.4s
G4: 12 case(s) tagged M in section 8 did not run: G4-b2 G4-b3 G4-b4 G4-e2 G4-f
    G4-h1 G4-h2 G4-h3 G4-i G4-l2 G4-l3 G4-m
  Boot the simulated rover first: make gates-sim
exit=2
```

### G4-h1 now reads its evidence off the controller

The old case SIGSTOPped robotd and then looked for motion in robotd's own
`state` stream — so the freeze silenced the witness, `client.states()` yielded
nothing, and the check could not fail. It never started a drive either. It now
starts a real drive, confirms the wheels are turning, opens a **read-only** tap
on the controller port (I-18: only robotd writes it) and measures `T.v_cmd_mm_s`
reaching zero with the brake asserted:

```
G4-h1  I-14  PASS  rover-robotd frozen: v_cmd reaches 0 on the wire within
  300 ms + t_brake -- pid 65297 SIGSTOPped; v_cmd reached 0 after 360 ms
  (budget 710 ms), brake asserted: True, motion seen before the freeze: True
```

360 ms is the MCU's own 300 ms `frame_ttl_ms` plus the abort ramp, measured on
the Pi's clock. The positive control — *motion seen before the freeze* — is what
the old case lacked.

### What was not re-run

`make gates-full` and `make gate-g1` were not re-run in this pass: neither
touches the code paths changed here beyond G4 and G2, and G1's 510 fakebox
requests measure the model, not the fixes. Nothing on the Pi was run at all —
every operability fix (`install.sh`, `preflight.sh`, `sync.sh`, the units, the
apt list, the README steps, the ESP-IDF `sdkconfig` isolation) is reviewed and
syntax-checked (`bash -n`, `make lint`) but is only exercised by a real deploy.
They are corrections to steps the review walked by hand; the first clean-card run
is still their test.

### The reconnect, end to end in the sim

With the six processes up, robotd was killed and restarted while brain kept
running:

```
run/brain.log
  16:21:37 INFO  rover.brain robotd bus disconnected; retrying
  16:21:38 WARN  rover.brain robotd bus ./run/robotd.sock unreachable ([Errno 61] Connection refused)
  16:21:52 INFO  rover.brain connected to robotd at ./run/robotd.sock
```

and the next voice turn executed:

```
$ python -m rover_devtools.roverctl --config config/robot.mac.toml \
      utter --watch 20 "turn left ninety degrees"
fsm     PLANNING / SPEAKING_INTENT / EXECUTING / SPEAKING_RESULT / IDLE

run/brain.log
  result: {"cmd_id":"01M1YVZR21N28T25F7G5F5202R","status":"done","reason":"",
           "detail":{"turned_deg":89.44,"duration_ms":2391,
                     "odom_delta":{"x_m":1.6e-05,"y_m":-1.2e-05,"yaw_rad":1.5611}}}
```

Before the fix, brain's reader task ended silently on the disconnect and every
later motion command died with an unretrieved `ConnectionResetError` inside the
execute task — no `Executed` event, the FSM parked in EXECUTING until its state
timeout, and no way back but restarting brain. `yaw_rad` 1.5611 rad = 89.44°
also shows `odom_delta` and `turned_deg` agreeing, which is the wrap fix.

---

## 10. Review fixes, second pass, 2026-09-07 — what was changed and re-run

Four reviews (invariants, correctness, operability, simplicity) produced two
critical and thirteen major findings plus a tail of minors. Every critical and
major is fixed at its root cause below, each with a test that fails without the
fix. The full suite, the lint gate, the C core's host tests and the whole
simulated-rover gate run were executed afterwards; the real output is quoted.

### Firmware core

**F1 (critical) — an escalated obstacle opened the 30 A e-stop relay.**
`ROVER_FAULT_OBSTACLE_LATCHED` is inside `ROVER_FAULTS_LATCHED`, and
`update_state()` used that same mask for the `MOTOR_EN` rule, so the 30 s
escalation branch of §5.1 — which needs only an obstacle-class bit and no
command at all — deasserted the relay-coil FET. Parking 250 mm from a wall, or
one dead forward ToF, opened the relay after 30 s; recovery closes it again, and
the BOM's 2.2 Ω NTC inrush limiter stays hot between rapid cycles, so each cycle
lands a 1000 µF inrush on the one contact the hardware e-stop depends on
opening. `firmware/core/rover_core.c:514` now excludes that one bit from the
relay rule only: the bit still enters FAULT, still refuses `V` with reason 8 and
still needs a `C`. New host case
`test_escalated_obstacle_does_not_open_the_relay` drives the 30 s branch and
asserts `motor_en` stays 1 across all 32 s, then raises `OVERCURRENT` and
asserts the relay does open — the exclusion is one bit, not the class.

**F2 (major) — every idle disarm blinded forward motion for 1.75 s.**
`enter_disarmed()` cleared `cal_valid` on every entry to DISARMED, and robotd
refuses to arm while `ctrl_flags` b6 is clear. robotd sends `D` routinely, after
`motion_idle_disarm_ms = 5000`, so a second instruction dispatched 5.00–6.75 s
after the last motion ended was answered `rejected reason=not_ready` with no
retry anywhere. §4.1 says `cal_valid` clears **on every reset**, not on every
disarm. `enter_disarmed()` now resets only `cliff_sample_n`; `cliff_step()` swaps
the new median in when the 50th sample lands, so the standing baseline keeps
guarding the edge while the window refills. New host case
`test_an_idle_disarm_keeps_the_standing_cliff_baseline`;
`test_calibration_survives_an_arm_inside_the_sample_window` was rewritten — it
had asserted the old behaviour verbatim.

### robotd and the bus

**ttl-vs-speed-clamp (major) — every model drive above 20 cm/s within a metre of
anything was refused.** `_prepare_motion` clamped the speed down and then
recomputed T2 from the *clamped* speed, while brain derives `goal_ttl_ms` from
the *requested* one. A slower drive always yields a larger T2, so
`goal_ttl_ms < t2_ms` always fired: `drive(distance_cm=60, speed_cms=30)` at
`front_range_mm=800` gave brain 3500 ms against robotd's 5000 ms and was
rejected `goal_ttl_too_short` instead of executed at the clamped speed with
`speed_clamped_to_cms` reported, which is what §6 and `AcceptedSkill` promise.
The too-short test now compares against the sender's T2, and the effective
deadline extends to cover the drive the clamp created — the `min(goal_ttl_ms,
T2)` rule of §4.2 is unchanged wherever no clamp applied.
`test_a_speed_above_the_cap_is_clamped_and_reported` is parametrised over
`front_mm` 400 and 800 and now builds `goal_ttl_ms` from the profile, which is
the `clamp-test-uses-constant-ttl` finding: the constant 5000 ms in the helper
was what certified the broken path.

**shutdown-stop-disarm-dead (major) — no `S` and no `D` on shutdown.**
`Robotd.run`'s finally cancelled the link task before `aclose()`; cancelling
`Link.run` runs its per-iteration `finally: self._teardown()`, which clears the
fd, so `Link.close`'s `connected and _ready` guard was false and neither frame
went out. `await self.link.close()` now runs first. Separately, no process
installed a SIGTERM handler, so under systemd the interpreter died without
unwinding and `aclose()`, `RobotdLog.close()` and `EpisodeRecorder.stop()`'s one
fsync never ran: `main()` now goes through a new `serve()` that installs
`add_signal_handler` for SIGTERM and SIGINT. Two new pty cases pin the order —
`…_brakes_and_disarms_before_the_link_task_is_cancelled` and
`…_cancelling_the_link_task_first_sends_neither_frame` — plus
`test_sigterm_unwinds_robotd_instead_of_killing_it`, which invokes the
registered callback directly rather than raising a real signal so a regression
fails one case instead of killing the run.

**mcu-rx-drop-wrong-source (major) — the bus reported the wrong direction.**
`StateMcu.rx_drop` was filled from robotd's own host-side count of malformed
*up*-direction lines while every other field of `mcu` came from the `T` frame,
and `telemetry.rx_drop` — the MCU's down-direction counter that `LINK_CRC`
latches on — was decoded and read nowhere. A noisy Pi→MCU link climbed on the
wire (G2-b reads `T` directly, so the gate stayed green) and stayed 0 for
`roverctl`, the web status strip and the event log. Now
`rx_drop=telemetry.rx_drop`, and `StateMcu.rx_drop` carries `le=0xFFFF`, the
wire's own bound. New case `test_state_publishes_the_mcus_own_rx_drop_counter`.

**F3 (major) — no latched MCU fault had a user-facing recovery.** `POST /clear`
sent `ClearableFault.ESTOP_SW` and nothing else, so I-20's own designed event —
Task-WDT panic, reboot with `WDT_REBOOT` latched, `ready:false reason="faulted"`
— left the rover dead to voice, web and teleop with the STOP page's only button
clearing a bit that was not set. `/clear` now takes an optional body naming
faults, validated against `ClearableFault`; `/status` publishes the latched-class
bit table from `rover_contracts`; and the face page grows a `clear <fault>`
button per set bit whenever `state.mcu.fault` shows one. Nothing new is trusted:
robotd still masks with `LATCHED_FAULTS` and the MCU still refuses a bit whose
cause persists. Three new cases in `tests/unit/test_web.py`; README documents
the panel and the `roverctl clear <fault> --source web` fallback.

**F7/F8/F9 and four more minors.** `authorized_motion` is now *required* on a
motion skill rather than merely respected when present (§7 makes a null STT
confidence authorized, not a missing flag); `_step()` consumes
`_pending_ds`/`_pending_dyaw` at the top so an abort cannot charge a previous
cycle's odometry to the next skill; an out-of-enum `T.state` maps to `FAULT`
with a WARN instead of raising inside `_publish_state` and silently stopping
state publication mid-drive; `_teardown` clears `self._out` so a truncated frame
from a dead session cannot eat the reconnect newline; brain's `FrameSource._read`
drops a malformed header instead of taking the whole TaskGroup down with it; an
unrecognised `E` code is logged rather than published as a `fault_set` that did
not happen; and Python's `RxCounters` gained `stale_seq` so both sides of
`T.rx_drop` sum the same wire events, pinned by
`test_rx_drop_sums_the_same_wire_events_on_both_sides`, which parses
`rover_rx_dropped()` out of the C.

**SIMP-1 (major) — config keys nothing read.** `[safety]
link_alive_max_age_ms` is documented as driving readiness and `is_connected`
(§5.8, §14) and was inert: both predicates derived from `cmd_gate_max_age_ms`.
`Link.link_alive` is new and reads it; `link_down` stays the §7 T0 command gate,
and the validator rule `cmd_gate_max_age_ms < link_alive_max_age_ms` now
separates two live thresholds. `[robot] name` is documented as the wake-word
identity, and a wake model is trained for exactly one name (open item 11), so a
`pyopen` backend whose model does not name the robot refuses startup instead of
logging an override at WARN that changes nothing. Two new cases.

### Gates

**F4 (major) — G4-b2 could not attribute the motion it scored.** It sampled a
0.5 s window for any non-zero twist and called it the corpus's, on a rover every
gate shares; G3's soak broke out of each turn at `SPEAKING_INTENT` with motion in
flight and ran immediately before it. It now records the odometry pose and
`mcu.last_ack_seq` before the corpus and asserts a pose delta within 5 mm /
0.02 rad after it, and `settle()` returns whether rest was actually reached so
the subtest fails attributably instead of scoring someone else's drive. G3's
soak drains to a stop — `stop`, then wait for `mcu.motion` false — before
closing its clients. The same treatment went to G4-h1/h2/h3, which had the same
defect and where h1's own resumed drive was being scored as h2's.

**F5 (major) — I-4's TTC criterion was never exercised.** G2-d2 ran against the
default simulator, where `--sim-flags` is empty: with a clear path
`rover_control_step` enters neither the obstacle branch nor the slow-zone
branch, b0 `require_slow_zone_stop` is a no-op for all 256 values, and the sweep
proved only the 300 mm/s clamp G2-d already proved with one frame. A regression
making b0 *widen* would have passed it unchanged. `sub_d_ttc` now runs on a
fresh simulator carrying `obstacle=400` — inside the 600 mm slow zone, outside
the 250 mm stop zone — and asserts `worst <= min(400 − tof_stop_mm, 150)`. The
three-value sweep (flags 0, 1, 2), each value held long enough for the
asymmetric slew to settle, always runs; the 256-value sweep stays behind
`--full` as `d4`.

```
G2-d2  I-4  PASS  the V.flags sweep against an obstacle, never above the TTC law
  -- obstacle 400 mm: flags 0 -> 150 mm/s, flags 2 -> 150 mm/s (both <= 150 =
  min(400 - 250, 150)), flags 1 (b0 require_slow_zone_stop) -> 0 mm/s (must be
  0: b0 narrows, never widens)
G2-d4  I-4  PASS  all 256 V.flags values against the same obstacle -- worst
  v_cmd over 256 flag values: 150 mm/s at flags 0x00 (<= 150; every bit is
  narrowing-only)
```

**F6 (major) — I-17's grep gate missed six of seven spellings.** Its
`suspicious` regex required the two clock tokens adjacent with at most a couple
of operator characters between them, so a call, a cast, a member access or a
unit conversion hid the violation; and its `allow` regex exempted every line
containing `pi_mono_us`, which exempts `drift = frame.mcu_us - pi_mono_us` — a
direct comparison of the two clocks. `tests/gates/gatelib/clockcheck.py` replaces
it: Python through `ast`, C through a token scan, flagging any arithmetic or
comparison operator one of whose operands carries an MCU-clock name while
another carries a host-clock name, at any nesting. The allow-list is keyed by
repo-relative file and qualified symbol (`Link._note_pong`), not by a substring
anywhere on a line. New sub-gate `G4-i17a` feeds the checker eight spellings and
asserts all eight are flagged, and asserts the same code in a sibling method of
the same file is flagged while `_note_pong` is exempt. `firmware/sim/mcu_sim.c`'s
`monotonic_us()` was renamed `sim_now_us()` — in mcu-sim there is one clock and
it stands in for `esp_timer_get_time()`, so a host name there read to the
checker, and to a reviewer, as the controller differencing a Pi timestamp.

### Operability

**OPS-1 (critical) — the wiring table drove a 5 V fan off a GPIO pad.**
`docs/wiring.md:107` connected a 40 mm 5 V fan directly to GPIO18 with no
switching device, no gate resistor and no flyback diode. A BCM2711 pad is 16 mA
absolute maximum per pin, 50 mA across the header, and swings 3.3 V; the fan
draws 80–150 mA running and more at start-up, and `gpio-fan` asserts that pin at
60 °C, which a Pi 4 with `arm_boost=1` reaches within minutes under G3a load.
The table now names the FET gate, the file carries the low-side switch drawing
(2N7002/AO3400, 100 Ω gate, 10 kΩ pull-down, 1N4148/SS14 flyback) and the reason,
and ARCHITECTURE §15's cooling row and §3's hardware sentence carry the parts.

**OPS-2, OPS-5, OPS-6, OPS-10 — preflight.** The empty `[audio] output_device`
is now a FAIL when `[tts] backend = "piper"`, printing `aplay -l` and the exact
line to add, rather than a skip that let a headless Pi pass every check and be
inaudible. A new block fails when `[audio] input` is not `text` while no usable
recogniser is configured, naming the four filenames `SherpaStt` opens. The
camera probe is re-run as `rover-cam` — the unit that actually opens the camera,
with only `SupplementaryGroups=video` — mirroring what the audio block already
does for `rover-brain`. And preflight now prints "re-run after `systemctl start
rover.target`" when it skipped the bus half, which on the documented one-pass
path never executed at all; README gains step 11b and ARCHITECTURE §11 the same.

**OPS-3, OPS-4 — the two Makefile targets the README sends a Pi user to.**
`make doctor` passed `--config config/robot.mac.toml` and so described the
simulator on the Pi — pty serial, sockets under `run/`, fake camera, loopback box
— and never looked at `/dev/rover-mcu` or `/run/rover`. It now resolves
`config/robot.toml` when one exists. `make gates-pi` ran `pytest` from a venv
that deliberately carries none; it now runs `tests/gates/g3/run_g3.py` directly,
which needs no test tooling, and README step 12 and ARCHITECTURE §11 step 12 say
so, with the separate-environment form for anyone who wants the wrapper.

**OPS-7, OPS-8, OPS-9, OPS-11, OPS-12.** `apt-mark hold` runs one package at a
time with the `libcameraN.M` name derived from the installed module rather than
hard-coded; `install.sh` symlinks `roverctl` into `/usr/local/bin`; the fixture
path in `box/README.md` and `box_probe.py` points at `tests/fixtures/frames/`
and a missing image is a WARN, not a silent 1×1 substitution; `Wants=` joins the
inert `After=network-online.target` on the two units that reach the network and
the ordering is deleted from the two that do not; `deploy/sync.sh` stops
stamping this Mac's uid/gid onto the Pi and excludes `firmware/host/build/`.

### What was run

```
$ make test
869 passed in 51.20s
ninja: no work to do.
    Start 1: rover_host_tests
1/1 Test #1: rover_host_tests .................   Passed    0.49 sec
100% tests passed, 0 tests failed out of 1

$ ./firmware/host/build/rover_host_tests | tail -3
PASS the cliff calibration survives an arm inside the sample window
PASS a ToF bus error emits I2C_ERROR with its sensor index
22 cases, 0 failed assertions

$ make lint
.venv/bin/python -m ruff check .
All checks passed!
shell syntax ok

$ make gates-sim            # boots the six sim processes, runs every gate
  ok    fakebox on :8000
  ok    mcu-sim pty at run/mcu.pty
  ok    robotd bus at run/robotd.sock
  ok    cam frames at run/frames.sock
  ok    brain bus at run/brain.sock
  ok    web on :8080
running the gates against the simulated rover
7 passed in 156.95s (0:02:36)

$ make secrets
clean: no secret, LAN address, MAC address or home path in what git publishes
$ make deps-check
runtime closure clean: no ML package
$ make schema-check
schemas up to date
```

The 869 unit and contract cases are 18 more than before this pass. `make
gates-sim` is the seven gate wrappers — G1, G2 (twice: the default subset and
the `--full` flag sweep), G3, G4, G5, G6 — against a live simulated rover, which
is the fault-injection gate the fixes above touch. The two fault-injection
results that changed shape:

```
G4-b2  I-8   PASS  robotd answers every fuzz case rejected and the pose does not
  move -- 423 rejected, pose moved 0.0 mm / 0.000 rad
G4-i17a I-17 PASS  the clock checker flags every spelling of the violation --
  8 spellings all flagged; the 5.1 allowance is one symbol, not a substring
  (Link._note_pong exempt, the same code in a sibling method flagged: True)
```

`make typecheck` still reports the eight pre-existing `import-not-found` errors
for the Pi-only optional dependencies (`sounddevice`, `sherpa_onnx`,
`pyopen_wakeword`, `picamera2`, `simplejpeg`) plus one from numpy's own stubs
under Python 3.12. None of them is touched by this pass and none is a gate.

### What was not run

Nothing on the Pi. Every operability fix — `install.sh`, `preflight.sh`,
`sync.sh`, the units, the README steps, the wiring table — is reviewed, shell
syntax-checked (`bash -n` through `make lint`) and reasoned about against the
documented deploy path, but a clean-card run is still their only real test. The
F1 relay fix is scored in the simulator against the same `firmware/core/` the S3
runs; the physical half is G2-e and needs a meter.
