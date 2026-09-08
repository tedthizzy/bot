# Patches

Unified diffs against the pristine upstream sources (`UPSTREAM.md`: commit
`d308df9`), one per logical change, numbered in apply order. Together they turn
stock `ugv_base_general` into the `bot-wr-1` firmware that `docs/protocol.md`
describes. Fork runtime code lives in the four `General_Driver/bot_*.h` headers;
the vendor files contain the integration changes.

Apply from `firmware/` onto a fresh copy of the upstream `General_Driver/`
sources:

```sh
for p in patches/0*.patch; do patch -p1 < "$p"; done
```

Each intermediate state compiles on its own; the safety properties below are
cumulative.

| patch | changes | safety property |
| --- | --- | --- |
| `0001-bot-config.patch` | Adds `bot_config.h` with every constant the fork introduces, and includes it from `ugv_config.h`. | The whole envelope (heartbeat, cap, radio switches, sensor thresholds, battery limits, stop-flag bits) is readable in one file; host tests read `BOT_POWER_CAP` and `BOT_HEARTBEAT_MS` from it. |
| `0002-heartbeat-300ms.patch` | `HEART_BEAT_DELAY` starts from `BOT_HEARTBEAT_MS` (300) instead of 3000. `changeHeartBeatDelay()` (`T:136`) accepts only values in (0, 300]; higher values, zero and negatives are ignored (a negative would never fire because `heartBeatCtrl()` compares against an `unsigned long`). | A frozen host stops the wheels within 300 ms plus spin-down, and nothing sent over the wire can lengthen that. |
| `0003-radios-off.patch` | With `BOT_WIFI_ENABLED 0` / `BOT_ESPNOW_ENABLED 0`: `setup()` skips `initWifi()`, `initHttpWebServer()` and `initEspNow()`, calls `WiFi.mode(WIFI_OFF)`, and `loop()` skips `server.handleClient()`; `http_server.h` (and the 48 KB web page) is not compiled in; `espNowMode` is 0 and `ctrlByBroadcast` false; the `T:300`–`306` (ESP-NOW) and `T:401`–`408` (Wi-Fi) command cases are compiled out, so `T:402` cannot start an access point at run time. The OLED shows the fork tag and `radio off`. | The serial line is the only control path. Stock boots an open access point with a web control page and an ESP-NOW follower that accepts broadcast commands from any leader. |
| `0004-no-boot-mission.patch` | `setup()` no longer calls `createMission("boot", ...)` or `missionPlay("boot", 1)`. | Nothing stored in flash can move the robot at power-on; motion needs a live host. |
| `0005-no-encoders.patch` | With `BOT_ENCODERS 0`: `initEncoders()`, `pidControllerInit()` and the per-loop `getLeftSpeed()` / `getRightSpeed()` / PID compute calls are compiled out. | GPIO 34, 35, 16 and 27 are not claimed by the pulse counter (the WAVE ROVER has no encoders); 27 is the bumper input. The closed-loop path, which would run a PID on zero feedback if `T:900` switched `mainType`, is inert. |
| `0006-safety-sensors.patch` | Adds `bot_safety.h`: the `st` bitmask, `cc` counter, `tf` and `bp` state; VL53L1X init (short mode, 50 ms budget, continuous) and a non-blocking read every 10 ms with a 500 ms staleness timeout; bumper input with 20 ms debounce; INA219 sampling every 500 ms with the low-battery latch (below 9.9 V for 10 s sets flag 8, above 10.2 V for 30 s clears it). `setup()` calls `bot_safetyInit()`, `loop()` calls `bot_safetyUpdate()`. | The firmware knows, independently of the host, when forward is unsafe (ToF, bumper) and when all motion is (battery), and says so in `st`. A missing sensor reads `tf = -1`; with `BOT_TOF_REQUIRED 1` that blocks forward. |
| `0007-power-cap-forward-block.patch` | `setGoalSpeed()` clamps its inputs to ±`BOT_POWER_CAP`; `leftCtrl()` / `rightCtrl()` become wrappers that clamp every PWM to ±`BOT_PWM_CAP` and feed one chokepoint, `bot_driveMotors()`, which applies the forward-block rule (with `st` bit 2 or 4, a pair with L>0 and R>0 becomes zeros; reverse and rotation pass) and the low-battery refusal, then drives both H-bridges through the stock code. Every clamp increments `bot_clamp_count`. `bot_safetyUpdate()` re-drives the last request the moment a flag asserts. `T:115` zeroes the request pair and sets flag 16; `T:1`, `T:11`, `T:13` clear it. | No command, whichever `T` carries it (`T:1`, `T:11` raw PWM, `T:13`, the PID path), can exceed 0.30 (60 percent duty). Forward motion stops on an obstacle or bumper without waiting for the next host command; low battery stops everything. |
| `0008-feedback-banner.patch` | `baseInfoFeedback()` (`T:1001`) gains `hb`, `st`, `tf`, `bp`, `cc`, and for the open-loop rover reports `L`/`R` as the speed actually applied in host units (stock reports the raw PWM integer). Adds `FEEDBACK_BOT_BANNER 1006` and `CMD_BOT_BANNER 1007`; the banner `{"T":1006,"fw":"bot-wr-1","hb_ms":300,"cap":0.3,"proto":1}` is printed at the end of `setup()` and on `T:1007`. | The host can tell the fork from stock firmware and check that the compiled heartbeat and cap equal its own configuration before it moves anything; stock firmware is refused. |
| `0009-bounded-serial-cooperative-stops.patch` | Fixed 512-byte line storage, 128-byte/one-line poll budget, and a four-line deferred queue. Deferred wheel commands retain their receipt timestamps and expire at the heartbeat limit. Heartbeat and zero-speed stops clear PWM/PID state without depending on `mainType`. A chassis-mode change clears residual PWM. Vendor delays and file/mission loops service heartbeat, sensors, and immediate stops without overwriting the active JSON command. | Serial floods cannot grow memory indefinitely or keep the loop inside intake. Switching to the inactive PID mode cannot defeat motor stops. `T:111` retains its pause while safety runs; an urgent stop cancels older queued commands, and expired deferred motion cannot restart afterward. |

## What is not patched

The vendor command set remains available, except for the already-disabled radio
commands. Patch 9 changes chassis-mode transitions and how waits service safety;
it does not remove arm/gimbal, mission, file, reboot, or NVS operations. Module-only
changes retain their prior behaviour. The host still sends only the commands in
`docs/protocol.md`.

The cap and compiled heartbeat ceiling remain unchanged. Cooperative servicing
does not bound an individual blocking sensor, filesystem, servo-library, or
serial write. Those calls and physical stop latency still require hardware
verification; a compile or a Python simulator run does not establish them.

## Regression evidence

`bash firmware/tests/run.sh` compiles the production motor and serial/wait headers
with a fake clock and GPIO under ASan/UBSan. It covers the inactive-PID heartbeat
failure, a three-second wait, urgent stops, bounded intake, queue saturation,
deferred motion expiry and ordering, JSON isolation, and clock wrap. See the
[native regression checks](../README.md#native-regression-checks-no-docker-or-esp32-toolchain)
for the boundary between this harness and actual hardware.

All nine patches were applied in order with `patch --batch --fuzz=0 -p1` to the
source files checked out at `d308df91b333a513f45a238bef9ba9a0a5edf64c`.
The resulting `General_Driver/` matched the working sketch byte for byte.

## Regenerating

The patches were produced with `diff -ruN` between successive snapshots of
`General_Driver/`. To regenerate after editing, take the pristine sources as
step 0, apply the patches one by one saving a snapshot after each, and diff
neighbouring snapshots with `a/General_Driver` and `b/General_Driver` as the
path prefixes.
