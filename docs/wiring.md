# Wiring

Everything that gets soldered, crimped or plugged in, and the order to do it in.
Sourced from ARCHITECTURE 3, 4.1, 9 and 15; where the two disagree, the
architecture is right and this file is wrong — say so.

Read [Before power](#before-power-on) before you connect the battery. Two of
those checks exist because getting them wrong destroys the motor driver, and one
because getting it wrong drives the robot at full speed with no firmware
running.

---

## 1. Rails

Three rails, one star ground at the pack negative. **The motor return never
shares the Pi's ground path.**

```
             +--- 10 A fuse ---+--- D24V50F5 5V/5A --------------> Pi 5 V
             |                 |    (EN <- MCU PI_RAIL_EN)
 3S2P pack --+                 |
 + 20 A BMS  |                 +--- relay contacts --+-- 1000 uF + SMBJ16A --> MDD3A
 (common     |                                       |   (+ 2.2 ohm NTC in
  port)      |                                       |    series with the cap)
             +--- 5 V/2 A buck ---------------------------------> ESP32-S3, ToF,
                                                                  INA226, driver logic
```

| rail | source | feeds | notes |
|---|---|---|---|
| motor | pack → 10 A fuse → **relay contacts** → 1000 µF low-ESR + SMBJ16A TVS | MDD3A only | Nothing else on this rail. The cap and TVS are **not optional**: opening the contacts or tripping the BMS while the motors spin regenerates into a rail with no battery, and the MDD3A's absolute maximum input is 16 V. The 2.2 Ω NTC in series with the cap keeps the one contact closure per power cycle from being a welding event. |
| Pi | pack → 10 A fuse → **Pololu D24V50F5**, tapped **upstream of the relay** | Raspberry Pi 4 | 18 AWG soldered leads under 15 cm, 470 µF at the buck input. If you inject at header pins 2/4 rather than USB-C, add a 3 A polyfuse and an SMBJ5.0A. Upstream of the relay is the point: pressing the e-stop must not power down the Pi. |
| logic | pack → 5 V/2 A buck | ESP32-S3, 3× VL53L4CX, INA226, MDD3A logic side, NTC divider | Also upstream of the relay, for the same reason. |

**Pi rail enable.** `PI_RAIL_EN` (MCU GPIO1) drives the D24V50F5's `EN` pin
**open-drain, through a 10 kΩ series resistor, with no external pull-up.** The
module pulls `EN` to VIN internally, so a reset, unprogrammed or unpowered MCU
leaves the Pi rail **ON**; only an actively driven low kills it. Do not add a
pull-up to VIN: with the pin high-Z that sits the pad a diode drop above VDD and
pushes ~80 µA through the S3's ESD clamp into the 3.3 V rail, outside its
absolute maximum.

**Budget.** 5 V rails total 1.4–1.9 A typical, ~3.5 A peak, which is what sizes
the Pi buck at 5 A. Motors at 40% duty are ~2 × 0.45 A at 11 V; stall is
2 × 3.5 A. Whole robot ≈13 W average; 3S2P 35E ≈75 Wh nominal, ~60 Wh usable,
so ≈4.5 h.

**Cutoffs**, evaluated on the sag-compensated estimate
`V_oc = vbat_mv + imotor_ma × R_pack_mΩ/1000`, 10 s debounce, **never
suspended** — the research's ">2 A suspends the check" would disable it under
exactly the load it exists to survive:

| threshold | fault | what happens |
|---|---|---|
| 10.5 V | `UNDERVOLT_W` | warn only; advisory class |
| 9.9 V | `UNDERVOLT_S` | brake, DISARM, refuse `V`, refuse to `ARM` |
| 9.6 V | `UNDERVOLT_D` | deassert `MOTOR_EN`, assert `PI_SHUTDOWN_REQ`, wait for `PI_POWEROFF_IN` (≥3 edges in 2 s), then pull `PI_RAIL_EN` low |

Clearing warn or stop needs `V_oc` 0.3 V above the threshold for 30 s.
`R_pack_mΩ` is measured once at G2 by stepping a known current and compiled in.
`UNDERVOLT_D` is the **only** thing that removes the Pi rail (I-6).

---

## 2. ESP32-S3 pin map

DevKitC-1-N8R8. Pins that carry nothing, and why: **GPIO0/3/45/46** are strapping
pins; **GPIO19/20** are USB; **GPIO26–37** are octal flash and PSRAM;
**GPIO43/44** are the CP2102 bridge. Using GPIO39–42 forfeits hardware JTAG,
which USB-Serial/JTAG replaces.

| function | GPIO | dir | external pull | notes |
|---|---|---|---|---|
| MDD3A M1A / M1B / M2A / M2B | 4 / 5 / 6 / 7 | out | **10 kΩ to GND each** | both high = brake; both low at reset = brake, never drive |
| Encoder L A/B, R A/B | 15 / 16 / 17 / 18 | in | — | PCNT, 12.5 ns glitch filter |
| `MOTOR_EN` (relay coil FET gate) | 12 | out | **10 kΩ to GND** | floats low through reset → relay open |
| MCPWM FAULT0 — INA226 `ALERT` | 9 | in | 10 kΩ pull-up | active low, one-shot (OST) latch |
| Bumper, 2 NC switches **in series** | 10 | in | **10 kΩ to GND** | 3V3 → both NC contacts in series → GPIO10. Plain GPIO + ISR, **not** on the OST path |
| MCPWM FAULT2 — e-stop monitor | 11 | in | **100 kΩ / 33 kΩ divider to GND** | senses coil node A; 11.1 V → 2.75 V; 3.3 V clamp diode + 100 nF; `active_level = 0`, OST latch |
| I²C SDA / SCL | 13 / 14 | i/o | 4.7 kΩ pull-up | 3× ToF + INA226, 400 kHz |
| ToF XSHUT front-L / front-R / cliff | 40 / 41 / 42 | out | 10 kΩ to GND | see bring-up below |
| UART1 TX / RX to Pi | 47 / 48 | out / in | — | 921600 8N1 |
| `SERVO_EN` (**deferred to G6**; LED in v1) | 39 | out | 10 kΩ to GND | |
| `PI_RAIL_EN` → D24V50F5 `EN` | 1 | **open-drain** | **none**, 10 kΩ series | see §1; no pull-up to VIN |
| `PI_SHUTDOWN_REQ` → Pi GPIO17 | 21 | out | 10 kΩ to GND | driven high to request a clean halt |
| `PI_POWEROFF_IN` ← Pi GPIO26 | 8 | in | 10 kΩ to GND | the overlay's pulse train; **≥3 edges within 2 s confirms** |
| Driver NTC | 2 | ADC1 | 10 kΩ divider | raises `DRIVER_HOT` |

**Nine 10 kΩ pull-downs**, and they are load-bearing, not tidiness: four PWM
lines, `MOTOR_EN`, `SERVO_EN`, the bumper, `PI_SHUTDOWN_REQ`, `PI_POWEROFF_IN`.
Between the reset edge and the first instruction the S3's GPIOs are inputs and
the MDD3A documents no internal pulls, so an unpulled PWM line floating high is
full speed. That plus `app_main` driving GPIO4/5/6/7/12/21/39 low before any
peripheral init is I-24.

---

## 3. Pi side

| Pi pin | signal | to | overlay |
|---|---|---|---|
| GPIO12 (pin 32) | UART5 TXD | ESP32 GPIO48 (RX) | `dtoverlay=uart5` |
| GPIO13 (pin 33) | UART5 RXD | ESP32 GPIO47 (TX) | `dtoverlay=uart5` |
| GPIO17 (pin 11) | shutdown request in | ESP32 GPIO21 | `dtoverlay=gpio-shutdown,gpio_pin=17,active_low=0,gpio_pull=down` |
| GPIO26 (pin 37) | poweroff pulse out | ESP32 GPIO8 | `dtoverlay=gpio-poweroff,gpiopin=26` — **deploy step 17 only** |
| GPIO18 (pin 12) | fan FET gate | 2N7002/AO3400 gate, through 100 Ω | `dtoverlay=gpio-fan,gpiopin=18,temp=60000` |
| — | — | — | `camera_auto_detect=1`, `dtparam=watchdog=on`, `arm_boost=1` on a rev 1.4 board |

**The fan is switched, never driven.** GPIO18 drives a logic-level N-MOSFET
gate; the fan sits in the drain. A BCM2711 pad is rated 16 mA absolute maximum
per pin, 50 mA total across the header, and it swings 3.3 V, not 5 V — a 40 mm
5 V fan draws 80–150 mA running and more at start-up, and it is an inductive
load with no reverse-EMF protection on the pad but its own ESD diode. Wiring
the fan directly between pin 12 and ground either does not spin it or damages
the pad or the SoC, and the `gpio-fan` overlay asserts that pin at 60 °C, which
a Pi 4 with `arm_boost=1` reaches within minutes under G3a load.

```
+5 V (header pin 4) ──── fan + 
                          │
                   ┌──────┴──────┐        1N4148 or SS14 across the fan,
                   │   40 mm fan │◄───────cathode to +5 V, anode to the drain
                   └──────┬──────┘
                          │  fan −
GPIO18 (pin 12) ──100 Ω──┤G  ┌─ D
                          │  │      2N7002 or AO3400 (V_GS(th) well under 3.3 V)
                     10 kΩ┴  └─ S ──── GND (header pin 6, 9, 14, 20, …)
                     to GND
```

The 10 kΩ gate-to-ground pull-down holds the FET off through boot and reset,
when the pad is an input; the 100 Ω limits gate charge current; the flyback
diode clamps the motor's back-EMF at turn-off. Nothing in `preflight.sh` can
see this — it is a soldering-time decision, so it is stated here.

Ground the Pi to the ESP32 at one point. The UART is 3.3 V on both ends; no
level shifter.

**USB is flash and debug only.** The operational link is UART5, because the S3
has no `CHIP_RST_DIS` bit and a Pi 4 rev 1.4 cuts USB port power on every
reboot. GPIO14 is UART0 TXD and is why UART5 rather than a lower-numbered one.

`dtoverlay=gpio-poweroff` goes in **last**, after G2-f, on battery, with the
MCU's rail control wired. Installed earlier it bricks every `reboot` on the
bench: the overlay prevents the kernel resetting the SoC and requires an
external mechanism to remove power, which on the bench does not exist. This is
why the edit loop is `deploy/sync.sh`, not a reboot.

The udev rule keys on the device-tree node, not the tty number:

```
SUBSYSTEM=="tty", KERNELS=="fe201a00.serial", SYMLINK+="rover-mcu", GROUP="rover", MODE="0660"
```

Confirm the address with `ls -l /sys/class/tty/ttyAMA*/device`. Code names only
`/dev/rover-mcu`; the string `ttyAMA4` appears nowhere.

---

## 4. The emergency-stop path

**The button breaks the relay coil, not the motor rail.** A 22 mm mushroom is
not rated to break 10 A of inductive DC, and the Pi and the logic tap upstream
so pressing it never powers anything down.

```
  coil supply (pack, upstream of the contacts)
        |
   [ NC mushroom ]        <- the human
        |
     node A  ----+--- 100k ---+--- 33k --- GND      divider: 11.1 V -> 2.75 V
        |                     |                      + 3V3 clamp diode + 100 nF
   [ relay coil ]             +----------------> ESP32 GPIO11 (MCPWM FAULT2, OST)
        |
   [ N-MOSFET ]  <- gate = ESP32 GPIO12 MOTOR_EN, 100 ohm gate resistor,
        |            10 kohm gate-to-GND pull-down
       GND          flyback diode across the coil
```

Sense at node A, **above** the coil. There a closed button reads high and an
open button reads low whatever the FET is doing, so "the human pressed it" and
"firmware disarmed" cannot be confused. Below the coil they are
indistinguishable, and the BOM buys one NC contact block, so there is no second
contact to read. `ctrl_flags` b1 `estop_released` is exactly this input.

Use a logic-level FET — AO3400A or IRLZ44N, V_GS(th) well under 3.3 V. A
standard IRF-series part will not fully enhance at 3.3 V and will cook.

**`MOTOR_EN` lifecycle.** It asserts on the **first `A` of a power cycle and
then stays asserted.** It deasserts only on `UNDERVOLT_S`, `UNDERVOLT_D`, any
latched-class fault, and shutdown — explicitly **not** on `D`, `S`, TTL expiry
or an obstacle-class fault, all of which are handled by braking the PWM.
Tracking arm state would close and open a 30 A relay into 1000 µF every 5
seconds of idle and, over a day, weld the one contact the e-stop depends on
opening. G3a asserts fewer than 10 relay actuations per hour.

**Two MCPWM one-shot fault inputs, and only two: INA226 `ALERT` (GPIO9) and the
e-stop monitor (GPIO11).** An OST trip forces every generator bound to it to
its fault action — all four MDD3A inputs — and cannot be cleared while the
signal is still asserted. **The bumper is deliberately not on that path**: a
held bumper would kill reverse and rotation too and could never pass I-5 or
G4-a. The bumper is a plain GPIO with an ISR, and its forward-zero is enforced
in the 100 Hz control step, the same path as ToF and cliff.

**Bumper wiring is fail-safe by construction.** 3V3 → NC switch → NC switch →
GPIO10, with a 10 kΩ pull-down. At rest the line is **high**. Either switch
pressed, a broken wire, or an unseated connector pulls it **low** — a fail-safe
OR. G2 tests all three: open one switch, open the other, then cut the harness.

---

## 5. Sensors

**Forward ToF geometry.** Both VL53L4CX at **90 mm** height (80–100 mm
acceptable), **yawed ±9°** so their 18° cones abut on the centreline and
together span ±18°. At the 250 mm stop distance that is ±81 mm of half-width
against a ~100 mm chassis half-width: **the outer ~19 mm on each side is
bumper-only.** That is a permanent, measured gap, not a bug. Aiming them further
apart to reach the corners opens a blind cone straight ahead, which is worse.

**Bring-up.** All three power up at 0x29, so the addresses are assigned one at a
time: hold every XSHUT low, release front-L and write **0x30**, then front-R
**0x31**, then cliff **0x32**. XSHUT lines are GPIO40/41/42 with 10 kΩ
pull-downs.

v1 ships **short distance mode, timing budget 20 ms, inter-measurement period
30 ms, sensor task at 50 Hz** — the four `[safety]` numbers the detect-latency
budget derives from — accepting the ~1.3 m short-mode ceiling, which is still
2× the 600 mm slow zone. Anything beyond it maps to 65534. If the driver refuses
that pairing, the fallback is long mode at 33 ms / 40 ms. Preflight and G2 read
the achieved budget and period back **from each sensor** and fail if they differ
from `[safety]`.

**Three-valued, not two.** A real distance; **65534 = no target within range**
(forward allowed, no fault — an open room, a dark rug, a specular floor);
**65535 = sensor or I²C error, or a sample older than 200 ms** (`TOF_STALE`,
forward refused). Coverage is **not** a `min()`: if *either* forward sensor is
stale or erroring, forward is refused whatever the other reports. Per-sensor
state crosses the link as `ctrl_flags` b8 `tof_fl_ok` and b9 `tof_fr_ok`.

**Cliff.** One VL53L4CX downward. There is no `config_set` on the link, so the
baseline is never written from the Pi: on every entry to DISARMED the MCU takes
50 samples, stores the median as `cliff_baseline_mm`, sets `ctrl_flags` b6
`cal_valid`, and publishes it in an `E CAL_STORED` event. While `cal_valid` is
0 the MCU refuses all forward motion. `preflight.sh` **asserts** the reported
value against `[safety] cliff_baseline_mm`. `CLIFF` is raised when
`tof_cliff_mm > baseline + 80 mm` for two consecutive samples, **or on an
invalid read** — downward, an out-of-range return *is* the void, so the sign of
the conservative rule inverts.

**Current sense.** INA226 with a 2 mΩ 3 W shunt in the motor rail, `ALERT` to
GPIO9 as MCPWM FAULT0, tripping at 6.0 A. One INA226 on the pack cannot see a
4.0 A / 0.5 A split between channels, which is why there is also a per-channel
software I²t on a motor model and a per-wheel slip/stall detector. Put a 10 kΩ
NTC on the MDD3A heatsink into a divider on GPIO2.

**Deferred to G6, do not fit in v1:** ICM-20948 IMU and the `SERVO_EN` FET.
Nothing in v1 reads either. Their protocol fields (`gyro_z_mrad_s`, `rails`,
`V.flags` b1) stay reserved, so fitting them later needs no version bump.

---

## 6. Connectors and consumables

| where | connector | note |
|---|---|---|
| pack ↔ fuse ↔ relay | **XT60** pair | 14 AWG or heavier; the only high-current joint you will unplug |
| motor leads | JST-XH or soldered + heat-shrink | 22 AWG silicone |
| encoders | JST-PH 6-way | keep away from the motor leads |
| I²C bus + ToF | JST-PH 4-way | one 4.7 kΩ pull-up pair for the whole bus, not per device |
| bumper switches | JST-PH 2-way, in series | the series-NC chain is the fail-safe; do not parallel them |
| e-stop | screw terminals at the button | 22 AWG is fine — it is a coil, not the motor rail |
| Pi power | soldered leads under 15 cm, 18 AWG | 470 µF at the buck input |
| UART5 | Dupont is acceptable, JST-PH is better | 921600 over a long unshielded run is open item 7 |

Fusing: one **10 A** fuse between the pack and everything. The BMS is 20 A
common-port; the fuse is what protects the wiring, and the relay contacts are
30 A so they are not the limit.

Consumables: 22 AWG silicone wire, heat-shrink, standoffs, cable ties, **9×
10 kΩ pull-down resistors**, **1× 10 kΩ series for `PI_RAIL_EN`**, **2× 4.7 kΩ
I²C pull-ups**, 100 Ω gate resistor, 100 kΩ + 33 kΩ divider, 3.3 V clamp diode,
100 nF, flyback diode.

---

## Before power-on

Battery **disconnected** for all of it. A meter, not a guess.

1. **Continuity, pack negative to every ground.** One star point at the pack
   negative. Motor return does not share the Pi's ground path.
2. **Polarity at every buck input and output**, with a bench supply if you have
   one. Reversed into the D24V50F5 ends the day.
3. **The nine 10 kΩ pull-downs are fitted and measure ~10 kΩ to GND** on
   GPIO4, 5, 6, 7, 12, 21, 39, 10 and 8. This is the check that stops an
   unprogrammed MCU driving the motors at full speed.
4. **`PI_RAIL_EN` (GPIO1) has NO pull-up to VIN**, and has a 10 kΩ in series to
   the D24V50F5 `EN` pin. A pull-up here pushes current into the 3.3 V rail
   through the ESD clamp.
5. **1000 µF, SMBJ16A and the 2.2 Ω NTC are across the MDD3A supply,
   downstream of the relay contacts.** Without them the e-stop is the action
   most likely to destroy the driver.
6. **The e-stop divider reads at node A, above the coil**, and the clamp diode
   and 100 nF are fitted. With 11.1 V at node A the ESP32 pin must read ~2.75 V,
   never more than 3.3 V.
7. **The coil FET is logic-level** and its gate has a 100 Ω series resistor and
   a 10 kΩ pull-down. The flyback diode is across the coil, right way round.
8. **The two bumper switches are in series, NC**, and the line at GPIO10 is
   high at rest. Press each one in turn and watch it go low. Then unplug the
   harness and watch it go low.
9. **The 10 A fuse is fitted** and the XT60 polarity matches at both ends.
10. **ToF XSHUT lines are pulled down** and each sensor has power. They will all
    answer at 0x29 until firmware assigns 0x30/0x31/0x32.
11. **Nothing is on the motor rail except the MDD3A.**
12. **The Pi tap is upstream of the relay contacts.** Press the mushroom with a
    meter on the Pi 5 V rail: it must not move.

Then, with the wheels **off the ground** and staying off the ground until G2-e,
G2-i and G4-a pass:

13. Power the logic rail only. Confirm the MCU boots, `B` banner on UART1,
    `MOTOR_EN` low, all four MDD3A inputs low.
14. Power the motor rail. Meter or LED across a motor output: **zero current**
    with the MCU held in reset, through a power cycle, and in the download
    bootloader. That is I-24 and it is a hardware test, not a firmware one.
15. `deploy/preflight.sh`. It reads the `B` banner and asserts the firmware's
    `safety_hash` against the seven `[safety]` mirror keys, so a firmware built
    with different stop distances than the config claims cannot pass.
