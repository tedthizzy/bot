# ADR-0013 — WAVE ROVER chassis, open loop, Pi host first

Date: 2026-09-07. Status: accepted. Supersedes A2–A8, A21, A23–A25 and the BOM of
`ARCHITECTURE.md` v1; amends A1, A19, A35 and every invariant in section 8.
The tag `v0-pi-sim` marks the last commit of the superseded design.

## Decision

The rover is a Waveshare WAVE ROVER: a 4WD chassis with the General Driver for
Robots board (classic ESP32), a built-in UPS for three 18650 cells, a 9-axis IMU
and no wheel encoders. The host is the Raspberry Pi 4 the user already owns,
mounted on the chassis and wired to the board's UART header. The motor
controller runs Waveshare's own `ugv_base_general` firmware with a small set of
safety patches, kept as a fork under `firmware/`.

A phone host is planned as a second host against the same gates, after the Pi
stack has passed them on hardware. It is not part of this decision.

## Why

The custom ESP32-S3 controller, driver board, power tree and $434 bill of
materials were designed to be built from parts. A finished chassis with a
driver board, UPS and IMU replaces all of it for roughly the chassis price plus
three cells, an inline emergency stop and a time-of-flight sensor. The user has
ordered it. What the design loses is the encoder: there is no odometry, so
there is no `drive(distance)` and no pose, and every distance number in the
old skill catalog goes with it.

Sequencing the Pi first rather than going straight to the phone reuses the
tested host stack, gets a moving, gated robot within days of the chassis
arriving, and puts hardware truth before any rewrite. That was the strongest
lesson of v0: the one bug two review rounds missed was found by the compiler.

## What changes

| area | v0 (`v0-pi-sim`) | v1.1 (this ADR) |
| --- | --- | --- |
| controller | custom ESP32-S3 firmware, freestanding C core | Waveshare `ugv_base_general` fork with patches |
| link | ASCII frames, CRC-16, session, sequence, arm | Waveshare JSON lines, no checksum, heartbeat only |
| motion primitive | body velocity, wheel PI on encoders | left/right power, open loop |
| skills | `drive(distance)`, `turn(angle, rate)` | `drive_for(duration, power)`, `turn_to(heading)` closed on the IMU yaw |
| world state | pose in centimetres, speed cap | heading in degrees, power cap |
| stop path | hardware e-stop breaks the relay coil; Pi stays powered | inline switch between the UPS and the driver board; see `docs/wiring.md` for what it powers down |
| budget | metres and seconds per instruction | seconds per instruction |
| host processes | robotd, brain, cam, web | unchanged |
| bus, validator, gates, fake box, box | unchanged in shape | unchanged in shape |

## Safety chain, restated

1. The model proposes a skill. brain validates it strictly and attaches the
   host's own identifiers and expiry. Unchanged.
2. robotd validates again at dispatch: known skill, bounds, replay window,
   sequence, source, current turn, fresh observation, fresh feedback, budget,
   not stopped. Unchanged in shape; the bounds are the new catalog's.
3. robotd streams `{"T":1,"L":l,"R":r}` at 20 Hz from the loop that owns the
   goal, zeros when idle. There is no separate keep-alive thread, so a frozen
   robotd stops sending and the firmware zeroes the motors within its 300 ms
   heartbeat. This is I-1 and I-14 in one mechanism.
4. The firmware fork clamps every power value to 0.30 whichever command
   carried it, refuses to raise its heartbeat over the wire, blocks forward
   motion on the time-of-flight sensor and the bumper, refuses all motion on low
   battery, and boots with motors off and radios compiled out.
5. robotd refuses motion unless the firmware announces itself as the fork,
   with a heartbeat and cap equal to the host's configuration. Stock Waveshare
   firmware, with its 3 s heartbeat and no cap, is never driven.
6. The inline switch is the last authority and needs nothing from software.

What is weaker than v0, stated plainly: there is no checksum and no session on
the wire, so a corrupted line is dropped by the JSON parser rather than by a
CRC, and replay protection lives only in robotd. The link is a short cable
inside the chassis at 115200 baud; the cap bounds the damage of any value that
parses. There is no encoder, so a stall or a wheel off the ground is invisible
to the controller; the firmware's current limit is the driver's own, and the
per-instruction time budget is what bounds a drive that goes nowhere.

## Invariants

The 24 invariants keep their numbers. `docs/verification.md` carries the
re-mapping of each one to its new enforcement point and test, and marks the
ones whose meaning changed.

## What is retired

`legacy/firmware-s3/` holds the ESP32-S3 firmware, its host tests, its C
simulator, the CRC line codec and its golden vectors. Nothing imports it and
`make test` does not run it. It is kept because the safety logic in it is the
reference for the firmware patches, and because the compile-found bug in it is
the reason the fork gets compiled before it gets trusted.
