# Rover MCU firmware — ESP32-S3, ESP-IDF v5.5.5

`firmware/core/` is freestanding C11 and owns the protocol, the arm state
machine, the TTL, the caps, the slew limiter, the PI controller and the fault
classifier. `firmware/main/` — this half — owns pins, peripherals and three
tasks, and calls the four core entry points of ARCHITECTURE 4.1 and nothing
else. `mcu-sim` links the same core, which is why 19 of the 24 invariants go
green on a MacBook before this code is ever flashed.

| file | what it owns |
|---|---|
| `main/board.h` | every pin and hardware constant, one comment each |
| `main/app_main.c` | boot-safe outputs, the core instance, the 100 Hz control task |
| `main/motion.c` | MCPWM at 20 kHz, two OST fault inputs, PCNT quadrature |
| `main/sensors.c` | I²C, three ToF, INA226, driver NTC, bumper and e-stop |
| `main/vl53l4cx.c` | the distance-sensor driver (below) |
| `main/link.c` | UART1 at 921600 and the two byte rings |
| `main/power.c` | Pi rail, shutdown request, poweroff handshake |

Anything the core compiles in is used from `firmware/core/rover_config.h` by
its `ROVER_` name and never copied here: the `B` banner's `safety_hash` is a
CRC-32 over seven of those constants, so a second copy would be one the Pi
cannot check.

## Distance sensor: a driver, not a managed component

`main/vl53l4cx.c` is written here rather than pulled from the ESP component
registry. The registry carries no VL53L4CX driver, and ST's full ULD is a
multi-object-detection library an order of magnitude larger than the six
operations this design needs: address assignment, distance mode, timing
budget, inter-measurement period, a data-ready poll, and one distance with its
status. The register map is the VL53L1X family map the L4CX shares for that
subset.

ARCHITECTURE 4.1 already records the mode/budget pairing as `[UNVERIFIED]` for
this part, so the driver writes short mode at `ROVER_TOF_TIMING_BUDGET_MS` and
`ROVER_TOF_INTER_PERIOD_MS`, **reads both back, and refuses the sensor if they
differ**. A refused sensor sets its `tof_status` bit, which raises `TOF_STALE`
and refuses forward motion — the fail-safe direction, and the one an operator
can recognise. The stated fallback (long mode, 33 ms) is the only other
pairing in the table; nothing else is accepted, because a budget entry that is
never written is a magic number nothing can check.

Range status is mapped to the three-valued result of ARCHITECTURE 5.1 in the
driver, not in the core: `rover_in_t` carries a distance and one status bit
per sensor and has no `RangeStatus` field. A valid measurement is a distance,
signal-below-threshold is `65534` (no target — a clear path, I-16), and
everything else is `65535`.

## Build

Docker only; there is no host IDF install to keep in step. `espressif/idf:v5.5.5`
publishes a `linux/arm64` manifest (verified: `docker manifest inspect` lists
`arm64` beside `amd64`), so it runs natively on an M-series Mac.

```sh
firmware/docker/build.sh                # release: no console anywhere (A37)
firmware/docker/build.sh debug          # console on USB-Serial/JTAG
firmware/docker/build.sh release clean  # discard sdkconfig and rebuild
```

Release output lands in `firmware/build/`, debug in `firmware/build-debug/`, and
each profile owns its own `sdkconfig` inside that directory (`-DSDKCONFIG=`).
Both halves matter: ESP-IDF writes `sdkconfig` into the *project* directory by
default, not into `-B`, and values already in it win over `SDKCONFIG_DEFAULTS` —
so a shared one silently ships `CONFIG_ESP_CONSOLE_NONE=n` in an image the
operator believes is release, which is the second writer into the motor
controller A37 and I-18 forbid. `make firmware` goes through this script for the
same reason; a bare `idf.py build` reuses whatever the last debug build left
behind. `firmware/sdkconfig` is gitignored.

## Flash

From the host, not the container: Docker Desktop on macOS has no USB
passthrough, so the IDF container cannot see the port.

```sh
firmware/docker/flash.sh /dev/tty.usbmodem101          # release
firmware/docker/flash.sh /dev/tty.usbmodem101 debug
```

The script `cd`s into the build directory first, because IDF writes
`flash_args` with paths relative to it, and calls `esptool` (esptool 5.x
renamed the entry point from `esptool.py`).

## Monitor

A **release build prints nothing on any interface** — that is A37 and half of
I-18. There is no console on USB-Serial/JTAG, on the CP2102 UART0 bridge, or
on UART1. `idf.py monitor` on a release build shows an empty screen and that
is the correct result.

To see log output, flash the debug build and open the USB-Serial/JTAG port:

```sh
firmware/docker/build.sh debug && firmware/docker/flash.sh /dev/tty.usbmodem101 debug
screen /dev/tty.usbmodem101 115200      # or: idf.py -B build-debug monitor
```

A debug build advertises itself: `caps` bit 0 and `ctrl_flags` b7 are set, and
robotd refuses to arm an MCU that reports either. Never leave one in a robot
meant to move.

## Verifying the boot banner by hand

The banner is a `B` protocol frame on UART1, not console text. It repeats at
1 Hz until the first valid `H`, so there is no race to catch it.

On the Pi, after deploy step 8's `dtoverlay=uart5` and step 9's udev rule:

```sh
python -m rover_devtools.wirecat /dev/rover-mcu
```

Or from any machine with the adapter wired to UART1 (GPIO47 TX, GPIO48 RX):

```sh
python3 - <<'EOF'
import serial
port = serial.Serial('/dev/tty.usbserial-XXXX', 921600, timeout=2)
while True:
    line = port.readline().decode('ascii', 'replace').strip()
    if line.startswith('$B'):
        print(line)
        break
EOF
```

A release build on this board prints, with a new random session each boot:

```
$B,2,1,<session>,256,2,000A,1,3381018647*XXXX
```

Check, in order:

1. **`caps` = `000A`** — bit 1 cliff sensor, bit 3 INA226. Bit 0 clear is the
   release check: `000B` means a debug build with a live console, and I-18
   fails.
2. **`safety_hash` = `3381018647`**, the CRC-32 of `250,600,500,20,30,50,80`.
   `deploy/preflight.sh` asserts this against the `[safety]` mirror keys in
   `config/robot.toml` and prints both constant sets on mismatch. A difference
   means the firmware and the config disagree about the stop distance.
3. **`reset_reason`** — 1 is a power-on reset. A `T` frame's `fault` word
   carrying `0x20000` says the last boot was a Task-WDT panic (I-20) and
   `0x40000` says a brownout.
4. **`ctrl_flags` b7 clear** in the `T` frames that follow, which is the same
   release check read from the running telemetry rather than the banner.

`T` streams at 50 Hz from boot whether or not an `H` has arrived, so a
`wirecat` on a freshly powered MCU should show telemetry immediately. If it
shows nothing, the UART pins or the ground are wrong; if it shows `B` and no
`T`, the control task never started.

## Verifying the arm handshake by hand

Wheels off the ground. The MCU boots DISARMED and stays there until it sees
`H` then `A` in its own session (I-3), so this is the shortest sequence that
proves the link is alive in both directions.

Read one `T`, take its `SESS` and `ack_seq`, then send — seq strictly
increasing, one counter for the whole direction, CRC-16/CCITT-FALSE over the
body between `$` and `*`:

```
$H,2,<ack_seq+1>,0,3735928559*....     hello, wildcard session
$A,2,<ack_seq+2>,<SESS>,90210*....     arm, nonce 90210
$V,2,<ack_seq+3>,<SESS>,100,0,300,0*.. 100 mm/s forward, TTL 300 ms
```

`rover_devtools.wirecat` builds and CRCs these for you; by hand, the CRC is
the same function as `rover_contracts.serial_codec.crc16_ccitt_false`.

Expect:

- `$K` acking the `A` with `result` 0 and `echo` 90210 — the nonce echo is
  what makes a duplicate `A` detectable.
- `$E` event 1 `ARM_OK`.
- `T.state` moving 1 DISARMED → 2 ARMED_IDLE, then 3 ARMED_MOVING while `V`
  keeps arriving, with `v_cmd_mm_s` ramping toward 100 at the compiled slew
  rate rather than jumping.
- `MOTOR_EN` (GPIO12) going high on that first `A` **and staying high**. It
  must not follow arm state: G3a asserts fewer than ten relay actuations per
  hour, because cycling a 30 A relay into 1000 µF every disarm welds the one
  contact the e-stop depends on opening.
- Stopping the `V` stream: `v_cmd_mm_s` back to 0 within the 300 ms TTL, the
  driver inputs both **high** (MDD3A brake — PWM zero is not the terminal
  state I-1 asks for), `fault` bit `0x1` `TTL` set, and `ctrl_flags` b0 clear.
  Resume the stream and `TTL` clears on the next valid `V`.

If `A` is refused, `K.reason` says why: 9 `estop_asserted` (the mushroom is
pressed, or the divider on GPIO11 is not reading node A), 12 `undervoltage`,
15 `sensors_stale` (a ToF failed its read-back or the I²C bus is dead), 16
`arm_denied_moving`.

## What this half deliberately does not do

No goal is held here, no trajectory is integrated, no frame raises a cap, and
no host timestamp is ever compared against `esp_timer_get_time()` — `P`'s
`pi_mono_us` is echoed in `O` as an opaque token and is never read as a time
(I-17). Those rules live in the core; this file set gives it pins.
