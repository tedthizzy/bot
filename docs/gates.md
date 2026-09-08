# Current software gates

ADR-0013 supersedes the original S3 motion/wire design. These gates exercise
WAVE ROVER timed power, IMU heading, JSON feedback and the Pi host. They do not
establish physical stopping distance, power isolation, sensor coverage or
watchdog reset behavior on the assembled robot.

`test_gates.py` retains one pytest entry point per G1–G6 and the G2 full sweep.
Each CLI still emits case names, selectors, executed test names and JSONL
metrics. G2/G4/G6 call the existing boundary tests instead of carrying a second
implementation of robotd, the validator or the serial protocol. An empty test
selection, skipped required software test, missing dependency, timeout or
failure makes that software criterion fail. `--strict` also fails physical
bench skips. Hardware procedures remain explicitly incomplete.

Tests use temporary Unix sockets, loopback ports allocated by the OS, and owned
processes. G4 suspends only the robotd subprocess it starts. Default G3 and G5
never send commands to existing services. `G3 --hardware` enables the existing
live soak and audio bakeoff. `G5 --hardware` enables its live typed-stop timing
probe; that probe still does not measure spoken stop recognition.

## Evidence and migration map

| Previous scenario | Current check | Evidence retained or limit |
| --- | --- | --- |
| G2 CRC corruption, bounded lines | `test_wave_proto.py`, `test_robotd_link.py` malformed/reassembly tests | Current JSON parser and arrival freshness; CRC/session fields no longer exist. |
| G2 H/A bring-up and stock refusal | Link bring-up/banner tests | Actual Link over TCP/PTY; no WAVE arming handshake is invented. |
| G2 frame TTL and frozen writer | Link stopped-stream test; independent stub heartbeat test | No autonomous host keepalive; simulated board zeros after 300 ms. Physical braking stays pending. |
| G2 power cap and 256 `V.flags` values | Raw host cap test, stub clamp/stop-flag tests, full current protocol sweep | Host emission and board acceptance are asserted separately. WAVE has no input `V.flags` byte to sweep. |
| G2 watchdog/reset/source checks | `firmware/tests/run.sh`, `test_caps_match.py` | Native execution of production firmware headers with mocked GPIO/clock; separate source-constant comparison. Not an ESP32 execution claim. |
| G2 encoder braking, relay, current, sensor wiring | Named physical procedures | No measured-distance, encoder slip, S3 relay or current-based sag compensation claim. |
| G4 500-message fuzz corpus | `test_runtime.py::test_five_hundred_fuzz_lines_match_schema_and_invalid_lines_cannot_move` | All 500 classify against current contracts; every invalid line crosses the real bus and gets an error/rejection; successful drives before and after prevent an always-rejecting daemon from passing. |
| G4 obstacle, source spoof, depleted budget | `test_runtime.py` | Actual daemon/Unix bus/TCP raw commands; successful forward/reverse controls. Independent stub safety behavior remains separately tested by G2. |
| G4 replay, stale turn, stale observation, unauthorized motion | `test_robotd_motion.py::test_rejected_motion_has_positive_control_and_no_wire_output` | A successful drive precedes each refusal; later raw commands stay zero. |
| G4 STOP, estop, cancel, new turn, missing pings, feedback loss, reconnect/reboot | `test_robotd_motion.py::test_motion_faults_stop_raw_wire_and_never_resume` | Parameterized actual-runtime boundaries, persisted estop and no spontaneous motion after recovery. |
| G4 frozen robotd | `test_runtime.py::test_frozen_owned_robotd_stops_stream_and_expired_goal_does_not_resume` | Actual owned process suspension; independent observer sees the stream stop and simulated heartbeat expire. The goal expires during suspension; this is not a claim about every possible resume interval. |
| G4 late box reply / box-link loss | Named `test_brain_fsm.py` and `test_brain_box.py` cases | Actual FSM and HTTP-client component tests, paired with separate daemon stop/stale-turn tests; not a single microphone-to-motor integration test. Previously these criteria unconditionally skipped. |
| G4 mixed clocks | `gatelib/clockcheck.py`, eight regression spellings, current source-tree scan | Static evidence covers Pi hosts, contracts/devtools and current firmware. It is not a runtime synchronization test. |
| G4 clear authority and malformed stop fields | `test_robotd_bus.py` plus motion checks | Real bus tests; stop must not disappear in strict argument validation. |
| G5 bearing and observation schema | Direct eight-bearing calculation and forbidden-field checks | Formula/sign/schema only; physical camera calibration and 8/10 find trials stay pending. |
| G5 filler and scene memory | FSM cancellation, scene ring and world-state tests | Component evidence; audible overlap and model recall quality stay pending. |
| G6 teleop/episode keys | Web opt-in test and actual motion/episode recorder tests | Current action order is exactly `left.power`, `right.power`; observations retain applied power, heading and host arrival data. |
| G6 twenty episodes / conversion | `--episodes DIRECTORY`; explicit `--only convert` | Supplied data requires 20 nonempty, sequenced, bounded-power episodes. Offline conversion was never implemented; explicitly requesting it fails. |
| G6 `SERVO_EN` | Explicit superseded notice | The current WAVE topology has no such actuator channel. No success is claimed; additional actuators require a new topology decision and physical STOP confirmation. |

## Motion-suite invariants

The compact current suite also retains successful accepted-to-done execution,
drive ramps, time-budget charging, positive-left turning and heading wrap,
cancel-other-command isolation, teleop priority and renewal expiry, published
readiness/active-command/pack-voltage/budget fields, raw feedback logging,
unknown-feedback resilience, one fsync per episode, and SIGTERM unwinding.
The old encoder distance/pose assertions become timed-power/heading assertions
because the current chassis has no encoders. The old fabricated cap assertion
becomes a real raw-command cap assertion; the board's clamp cannot hide a host
violation. The old controller calibration-flag check becomes firmware-identity
refusal. Motor-current compensation and MCU clock/tick fields are not fabricated.

The former six thin wrapper files were consolidated without dropping their
seven invocations. The redundant rounding test was removed because the adjacent
parameterized cases already prove positive and negative half-away rounding.
Existing image/audio generation and all five JPEG fixtures remain. The three
world-state fixtures now use canonical headings, power units and time-only
budgets; an absent/out-of-range hallway reading is unknown, not clear.

## Running

Install the locked development environment, then run `make gates` or the
individual gate CLI with that environment's Python. G2 native firmware checks
also need a C++ compiler and the pinned ArduinoJson headers prepared by
`bash firmware/tests/run.sh --setup`. Missing required native-test inputs fail G2.
Direct-controller/process overrides (`--port`, `--sim-cmd`, `--pid-pattern`,
`--robotd`) are rejected by the isolated G2/G4 runners instead of attaching to
or signaling an unrelated running service. Historical S3 criteria are not
reinterpreted as passing WAVE hardware measurements.

## Result contract

Each CLI prints case results and writes JSONL to `logs/gates/` or `--metrics PATH`.
`PASS` means the named criterion ran and held.
`FAIL` includes missing, empty, skipped or timed-out required software.
`SKIP` identifies a physical or otherwise unavailable measurement.

Exit 0 means every executed criterion passed, not that every criterion executed.
Exit 1 means a failure. Exit 2 means all criteria skipped.
`--strict` promotes skips to failures.
Fakebox and selftest cannot establish model quality or attack resistance.

```sh
make gates             # isolated G2/G4 subset
make gates-full        # all gates plus isolated runtime integration
make gates-sim         # alias for gates-full
make gate-g1 BOX=real   # explicitly use the configured model service
make gates-pi          # explicitly run G3 on attached Pi hardware
```

Use each `tests/gates/gN/run_gN.py --help` for selectors.
Current executed evidence is in [debloat.md](../debloat.md).
The older [verification transcript](verification.md) remains historical.
