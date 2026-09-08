# Rover link protocol

The host talks to the WAVE ROVER's General Driver board over one serial line at
115200 baud, 8N1, using Waveshare's JSON-line protocol: one JSON object per
line, terminated by `\n`, UTF-8, no checksum. This document is the contract
between `rover_contracts.wave_proto`, the firmware fork in `firmware/`, and the
simulator in `rover_devtools.rover_stub`. Anything not listed here is not part
of the contract, even if the stock firmware accepts it.

On the Pi the line is `/dev/serial0`, the GPIO UART the board's Pi header is
wired to. In simulation it is a pseudo terminal or a TCP socket that carries the
same bytes.

## What the host sends

Only one command moves the robot. Everything else is configuration or a query.

| purpose | line | notes |
| --- | --- | --- |
| speed | `{"T":1,"L":l,"R":r}` | `l`, `r` are floats in Waveshare units, full scale ±0.5. The patched firmware clamps each to ±`cap` (0.30) and counts the clamp. Renews the heartbeat. Sent at 20 Hz whenever the link is up, zeros when idle. |
| coast | `{"T":115}` | All H-bridge inputs low. Motors freewheel. Sets stop flag 16 until the next speed command. |
| heartbeat | `{"T":136,"cmd":ms}` | The patched firmware accepts a value at or below its compiled default (300 ms) and ignores anything higher. Stock firmware accepts any value and defaults to 3000. |
| feedback on | `{"T":131,"cmd":1}` | Start the base feedback stream. |
| feedback interval | `{"T":142,"cmd":ms}` | Minimum gap between feedback lines. The host sets 50 for 20 Hz. |
| echo off | `{"T":143,"cmd":0}` | Stock firmware echoes every accepted command back; the host turns that off. |
| quiet | `{"T":605,"cmd":0}` | Suppress the firmware's human-readable debug prints. |
| banner | `{"T":1007}` | Ask the patched firmware to re-send its boot banner. Stock firmware ignores it. |
| oled | `{"T":3,"lineNum":n,"Text":"…"}` | Write a line on the board's display. Cosmetic. |

Anything else in Waveshare's command set (PWM input `T:11`, ROS twist `T:13`,
missions, files, Wi-Fi, ESP-NOW, servos) is deliberately never sent. The
firmware fork compiles the radio paths out, so they are not reachable either.

Units: a speed value of 0.30 is 60 percent PWM duty on the TB6612 driver
(0.5 is full scale, the firmware multiplies by 512 into an 8-bit duty). The
skill catalog calls this quantity `power`. Ground speed at a given power is a
property of the floor and the load and is measured, not assumed.

## What the rover sends

Every line is a JSON object with an integer `T`.

### `T:1001` base feedback, 20 Hz once enabled

Stock fields, always present:

| field | meaning |
| --- | --- |
| `L`, `R` | the speed values currently applied, after clamping, in the same units the host sends |
| `r`, `p`, `y` | roll, pitch, yaw in degrees from the on-board 9-axis fusion. `y` is in (−180, 180]. Its sign convention is confirmed on the real unit and recorded in `[link] yaw_sign`. |
| `temp` | board temperature, °C |
| `v` | bus voltage from the INA219, volts. This is the pack voltage. |

Fields added by the firmware fork. Their absence means stock firmware, and the
host refuses motion:

| field | meaning |
| --- | --- |
| `hb` | 1 while the heartbeat is alive, 0 after it has expired and the motors were zeroed |
| `st` | stop-flag bitmask, below |
| `tf` | front time-of-flight range in millimetres, or −1 when no sensor or no reading |
| `bp` | 1 while the bumper is pressed |
| `cc` | count of speed values clamped to the cap since boot, wraps at 65535 |

Stop flags in `st`:

| bit | value | meaning |
| --- | --- | --- |
| 0 | 1 | heartbeat expired; motors zeroed by the firmware |
| 1 | 2 | forward blocked by the time-of-flight sensor (reverse and rotation still allowed) |
| 2 | 4 | forward blocked by the bumper |
| 3 | 8 | all motion refused: low battery |
| 4 | 16 | coast requested by the host (`T:115`); cleared by the next speed command |

### `T:1002` IMU, on request

`r`, `p`, `y`, `ax`, `ay`, `az`, `gx`, `gy`, `gz`, `mx`, `my`, `mz`, `temp`.
Gyro rates are in degrees per second.

### `T:1006` banner, at boot and on request

`{"T":1006,"fw":"bot-wr-1","hb_ms":300,"cap":0.3,"proto":1}`

`fw` names the fork build, `hb_ms` and `cap` are the compiled safety constants,
`proto` is the version of this document the firmware implements. The host
compares `hb_ms` and `cap` with its own configuration and refuses to move if
they disagree.

### Everything else

Lines with any other `T`, lines that are not JSON, and lines over 512 bytes are
counted and dropped. The stock firmware's text prints (`"UGV started."` and the
like) fall in this class until the host has sent `quiet`.

## Timing

The firmware zeroes the motors when no speed command has arrived for the
heartbeat period, 300 ms in the fork. The host sends speed at 20 Hz, so the
firmware sees six commands per period and one dropped line costs nothing. A
frozen host process stops sending and the wheels stop within 300 ms plus the
motors' own spin-down; that is the whole of the Pi-side stop guarantee and it
requires nothing of the host to work.

The host treats feedback older than 150 ms as a dead link: it keeps sending
zeros, refuses new motion, and fails any goal in flight.

## Simulation

`rover-stub` implements this document on a pseudo terminal and on TCP, with a
first-order wheel model and an integrating heading. `--stock` makes it behave as
unpatched firmware: no banner, no added fields, a 3000 ms heartbeat, no clamp.
The gates use both.
