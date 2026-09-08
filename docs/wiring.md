# Wiring — WAVE ROVER, Pi 4, inline stop

The chassis arrives wired. This page covers the four things you add: how the
Pi is powered, the three data wires, the inline stop switch, and the optional
time-of-flight sensor and bumper. Every fact below comes from Waveshare's
General Driver schematic, the UPS module page, or the firmware source; items
marked *measure* are things the schematic cannot tell you about your unit.

## What the chassis already has

| part | detail |
| --- | --- |
| UPS module | three 18650 cells in series, protection and per-cell balancing on board, 12.6 V 2 A barrel-jack charging, a rear push-button that switches its outputs. Cells are not included and must be **unprotected** and under 67 mm; a protected cell does not fit or make contact. |
| UPS outputs | a 2-pin XH2.54 battery-voltage lead to the driver board, a Type-C 5 V pigtail from its own 5 A regulator, and an 8-pin I2C/power header. |
| General Driver for Robots | classic ESP32 (WROOM-32UE), one TB6612FNG driving the two sides (front and rear motors of a side are paralleled), INA219 reading the battery voltage, 9-axis IMU, OLED, a 5 A buck that feeds 5 V to the 40-pin header, CP2102 USB. No fuse on the input. |
| motors | four 12 V N20 gearmotors, 0.45 A each at stall, so 1.8 A with all four held. No encoders. |
| Pi header | a 40-pin header that passes 5 V (physical pins 2 and 4), ground, the ESP32's UART0 (Pi pin 8 TXD0 → ESP32 RX, Pi pin 10 RXD0 ← ESP32 TX) and the ESP32's I2C bus (Pi pins 3 and 5). Nothing else on it reaches the ESP32. *Measure* the pin mapping with a meter before connecting a Pi: the schematic's header symbol is numbered one column off from the Pi convention. |

The ESP32's UART0 is the same UART the CP2102 USB port uses. A PC on USB and a
Pi on the header both talk to it at once. **Never have both connected while the
robot is armed.** To flash, unplug the Pi's data wires or power the Pi down.
The Pi header carries no reset or boot lines, so opening the Pi's serial port
never resets the controller; the USB port does, which is one more reason the
Pi does not use it.

## Powering the Pi

Two ways. Choose one; never both, because both feed the same 5 V rail.

**Recommended: from the UPS's Type-C 5 V pigtail.** The Pi's power is then
independent of the driver board, and the inline stop switch below can cut the
motors and the controller while the Pi keeps running. That is what makes the
stop observable: robotd sees the link die, reports it, and re-runs its bring-up
when power returns. Use a short USB-C cable rated for 3 A. *Measure* whether the
pigtail is fitted on your unit and where it is routed under the belly plate; if
it is absent, a short lead from the UPS's 8-pin header's 5 V and ground to a
USB-C plug does the same job.

**Fallback: from the driver board's header 5 V.** Simpler, one ribbon cable,
and how Waveshare ships it. The cost is that the inline stop also drops the Pi,
so every stop is an unclean shutdown and the heartbeat behaviour cannot be
watched during one. If you must, mount the switch so a stop is rare and treat
the SD card accordingly.

Either way, `vcgencmd get_throttled` must read `0x0` under full load at G3.
*Measure* the header 5 V with the motors stalled; the board's buck is labelled
5 A but the wiki gives no rating.

## Data wires

With the Pi on its own power, run three wires from the driver board's 40-pin
header to the Pi's header, and nothing else:

| driver board header pin | Pi physical pin | signal |
| --- | --- | --- |
| 6 | 6 | ground |
| 8 | 8 | GPIO14 TXD0, Pi → ESP32 RX |
| 10 | 10 | GPIO15 RXD0, ESP32 TX → Pi |

Do not connect pins 2 or 4 (5 V) when the Pi is powered from the UPS. Do not
connect pins 3 and 5 unless you want the Pi on the ESP32's I2C bus; the OLED,
INA219 and IMU already live there and a second master with its own pull-ups is
a source of confusion, not a feature.

On the Pi, `enable_uart=1` and `dtoverlay=disable-bt` hand UART0 back to the
header, and the serial console must leave `cmdline.txt`; `deploy/install.sh`
does both and disables the Bluetooth UART service. The port is `/dev/serial0`
at 115200. Waveshare's own Pi setup makes the same changes.

## The inline stop switch

Insert a switch in the 2-pin XH2.54 lead between the UPS and the driver board.
Cut the positive wire, crimp or solder each end to the switch, insulate.

- **Type**: a latching mushroom-head emergency stop with a normally-closed
  contact, or a heavy toggle you can hit without looking. Rated for **at least
  5 A DC**; 10 A gives margin. The XH connector's own contacts are the 3 A weak
  point on this lead, and the four motors draw 1.8 A stalled plus whatever the
  board's 5 V branch carries.
- **What it stops**: the motors, the TB6612, the ESP32, the OLED, and the
  board's 5 V header. With the Pi on the UPS's Type-C, the Pi stays up.
- **What it does not do**: nothing in software knows about it directly. robotd
  learns of it as a dead link within 150 ms and streams zeros; when the switch
  is released the controller reboots with motors off and announces itself with
  the banner, and robotd re-runs its bring-up without resuming any goal. That
  is gate G2-e.
- Mount it on the top plate at the rear, reachable from the side the robot
  approaches you on. The UPS's own rear button also cuts everything including
  the Pi; leave it for storage, not for stops.

There is no fuse on the driver board input. A 5 A blade-fuse holder in the
same lead is cheap insurance against a shorted motor lead.

## Time-of-flight sensor (optional at first)

The firmware fork reads one VL53L1X on the ESP32's I2C bus and blocks forward
motion under 250 mm while still allowing reverse and rotation. It is optional:
with `BOT_TOF_REQUIRED 0` the rover runs without it and reports the range as
unknown, which the host treats as not-clear-but-not-blocked. Add it before you
let the rover drive toward anything you care about.

- Bus: 3.3 V I2C, SDA on ESP32 GPIO32, SCL on GPIO33, 4.7 kΩ pull-ups already on
  the board. The sensor's default address 0x29 does not collide with the OLED
  (0x3C), INA219 (0x42), IMU (0x6B, 0x0C) or the unpopulated BMP280 (0x77).
- Where to tap it: the board's single IIC connector is occupied by the OLED
  cable. Either splice a Y into that cable, or take 3V3 and ground from the
  7-pin P3 header and SDA/SCL from the 40-pin header's pins 3 and 5 on the
  driver-board side (they are the same bus). Keep the leads under 20 cm.
- Mount at the front centre, level, 40 to 60 mm above the floor, pointing
  forward. The sensor's cone is about 27°, so it will miss a chair leg until
  the rover is close; 250 mm is the margin for that.
- *Measure* at G4 that a box at 200 mm refuses forward and permits reverse.

## Bumper (optional)

A normally-open microswitch or a strip switch across the front, wired between
ground and ESP32 GPIO27, which the fork configures as an input with its internal
pull-up and reads active-low with a 20 ms debounce. GPIO27 is on the P3 7-pin
header and on the motor-B encoder connector, both with a ground pin beside it.
It is free because the WAVE ROVER has no encoders and the fork skips the encoder
setup. Enable it with `BOT_BUMPER_ENABLED 1` after you have confirmed the pin on
your board revision. Do not use GPIO34 or GPIO35 without an external pull-up;
they have none.

## Camera, microphone, speaker

The Camera Module 3 goes on the Pi's camera connector, mounted at the front of
the top plate, level, as high as the plate allows. The ReSpeaker Lite plugs into
a Pi USB port and drives the speaker from its own amplifier output; keep it as
far from the motors as the plate allows and mount the speaker facing up. The
plate's Pi hole pattern is 58 × 49 mm with 12.5 mm standoffs.

## Before power-on

1. Cells in, all three the same way; the UPS lights a per-cell LED if one is
   reversed. Do not charge in that state.
2. Press the UPS button once to wake the protection circuit, or plug the
   charger in.
3. Meter the Pi header: 5 V on pins 2 and 4 only, nothing on 8 and 10 beyond
   3.3 V logic. If 5 V appears anywhere else, stop and recheck the mapping.
4. Data wires: 6, 8, 10 only. USB cable to the board: unplugged.
5. Stop switch in the battery lead, in the released position; fuse in place.
6. Pi powered from the UPS Type-C; nothing on header pins 2 and 4.
7. Wheels off the floor for the first power-on. The OLED shows `WAVE ROVER`
   and the fork's version line; the motors do not move.
8. On the Pi: `deploy/preflight.sh` reads the banner and the first feedback
   line and refuses to continue if the firmware is stock.

## Numbers to record for `config/robot.toml`

| key | how |
| --- | --- |
| `[link] yaw_sign` | turn the rover left by hand 90° with feedback streaming; if `y` increases, 1, else −1 |
| `[limits] power_default` | at G5, the power that gives about 0.3 m/s on your floor; start at 0.20 |
| `[limits] turn_kp`, `power_min` | at G5, the smallest power that starts a turn on carpet, and the gain that lands within ±10° without oscillating |
