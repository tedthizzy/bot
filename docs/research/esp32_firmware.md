# ESP32-S3 motor/safety controller: firmware and hardware practice, September 2026

Research date: 2026-09-07. Web-search budget for the session was exhausted after 18 searches; remaining checks used direct page fetches. Where I could not find a 2025–2026 measurement I say so and tag the number [INFERRED].

## Summary

- Toolchain: **ESP-IDF v5.5.5** (maintenance since July 2026, supported to 2028-01-21). v6.0/v6.1 are the service releases but Arduino-ESP32 for IDF 6 is only **v4.0.0-alpha1**. For Arduino sensor libraries use **Arduino-ESP32 3.3.11 as an IDF component** or **pioarduino 55.03.311**; official PlatformIO is frozen at core 2.0.17. Rust **esp-hal 1.2.0** still has MCPWM, PCNT and Timer behind `unstable` and needs the Xtensa fork toolchain on the S3; skip it.
- Firmware: 100 Hz control task on core 1 from a GPTimer, **MCPWM** at 20 kHz with driver fault / e-stop / bumper / INA226 ALERT on an **MCPWM GPIO fault (one-shot brake)**, encoders on **PCNT** with `accum_count` and a 1 µs glitch filter, per-wheel PI + feedforward, 0.5 m/s² ramp. The 300 ms TTL is a timestamp check inside the control loop; the Task Watchdog (`CONFIG_ESP_TASK_WDT_PANIC=y`, 1 s) guards the control task.
- Serial: **GPIO UART, not native USB**. The S3 USB-Serial/JTAG resets on the esptool DTR/RTS pattern and has no disable bit. Wire to the Pi 4 PL011 (`/dev/ttyAMA0`) or a CP2102/CH343 at 921600, **COBS + CRC-16 + sequence number**, 20 Hz commands, 50 Hz telemetry.
- micro-ROS runs on the S3 (humble/jazzy/kilted/rolling, IDF 5.4–6.0) but needs an agent on the Pi. Start custom; add a `ros2_control` hardware interface later. `diff_drive_controller` defaults its command timeout to 0.5 s, so 300 ms is stricter than ROS convention.
- Hardware: for 3–5 kg at 0.3 m/s, **JGB37-520 90:1 (~170 rpm, 11 PPR Hall, $20)** on a **Cytron MDD3A** (3 A/5 A, 20 kHz max), or **Pololu 37D 50:1 (200 rpm, 3200 CPR, $61)** on an **MDD10A** or 2× DRV8871. Not DRV8833 (10.8 V max on 3S), not L298N. Current sense: **INA226** on the motor rail, ALERT to the MCPWM fault pin. ToF: **VL53L4CX** ($14.95, 18°, 1 mm–6 m) or one **VL53L5CX** (8x8, 63°) beats two VL53L1X ($22.95, 27°, 1.3 m short mode). E-stop: NC latching mushroom in the relay-coil path on the motor rail; the ESP32 can open the relay but has no path around the button.

## State of the art (2026)

### Toolchain

**ESP-IDF.** The roadmap (fetched 2026-09-07): release/5.4 in maintenance from January 2026, release/5.5 in maintenance from July 2026, v6.0 released 2026-02-27, v6.1 released 2026-07-31 (GitHub tag late August; v6.0.3 on Sept 2). v5.5 bugfixes continue to v5.5.7 (January 2027) and support ends 2028-01-21. v5.2 is end-of-life August 2026, v5.3 January 2027. v5.5.x has the largest third-party ecosystem; v6.x changed build defaults and Arduino has not caught up.

**Arduino-ESP32.** v3.3.11 (2026-07-22) is built on IDF v5.5.5. v4.0.0-alpha1 (May 2026) is "the same core as 3.3.9 with pre-compiled libs from release/v6.0", missing Matter, RainMaker, ESP_SR, Insights and CBOR. The documented component path is `idf.py add-dependency "espressif/arduino-esp32^3.3.11"`, FreeRTOS tick 1000 Hz, and a choice between autostart `setup()/loop()` or `app_main()` + `initArduino()`. That hybrid is the 2026 hobbyist sweet spot: `idf.py menuconfig`, ESP-IDF MCPWM/PCNT/GPTimer drivers, and Arduino I2C libraries for the ToF and INA226 boards.

**PlatformIO.** The official `platformio/espressif32` still bundles Arduino core 2.0.17 (IDF 4.4). The community **pioarduino** platform tracks upstream: 55.03.311 (Jul 24, 2026) = Arduino 3.3.11 / IDF 5.5.5; scheme `<IDF major><minor>.<Arduino major>.<minor patch>`. Use `platform = https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip`. It is a volunteer project; Espressif Discussion #11308 confirms Espressif does not control PlatformIO releases.

**Rust.** esp-hal 1.0.0 shipped 2025-10-30 with GPIO, UART, SPI, I2C stable; docs.rs shows 1.2.0 (2026-09-02) adding RNG, eFuse, clock, system, interrupt and time. MCPWM, PCNT, Timer, DMA, LEDC and RMT remain `unstable`. The S3 is Xtensa and needs the `espup` fork toolchain. A motor controller uses exactly the unstable peripherals.

### Firmware architecture

**PWM: MCPWM.** ESP32-S3 MCPWM has two groups, each with 3 timers, 3 operators and 6 generators (12 outputs), per-operator dead time, sync, and a fault module that "detects the fault condition from outside, mainly via the GPIO matrix" with two brake modes: CBC (cycle-by-cycle, auto-recovers) and OST (one-shot, cleared by `mcpwm_operator_recover_from_fault()`). LEDC on the S3 has 8 channels, 4 timers, low-speed mode only, 80 MHz APB; 5 kHz gives 13 bits and 20 kHz gives 11 bits (4000 ticks < 4096) [INFERRED from the LEDC formula]. MCPWM from PLL_160M at 20 kHz gives ~8000 ticks [INFERRED]. Resolution is irrelevant at 0.3 m/s; the OST fault brake is not, because it forces outputs to a safe level within one PWM period when a fault GPIO asserts, independent of any task. Espressif's `examples/peripherals/mcpwm/mcpwm_bdc_speed_control` uses MCPWM + PCNT + a timer-driven PID from the `espressif/pid_ctrl` component (v0.3.1) with a DRV8848-class H-bridge. LEDC is fine via Arduino `ledcWrite` (linorobot2 uses it) but has no fault module.

**Encoders: PCNT.** PCNT decodes quadrature with edge actions on channel A and level actions on channel B. The hardware counter is 16-bit with `high_limit`/`low_limit`; `accum_count` accumulates overflow so reads are wide. The glitch filter runs on APB (12.5 ns/tick); the register is 10 bits, so the ceiling is ~12.8 µs [INFERRED]. Stock examples use `max_glitch_ns = 1000`. A Pololu 37D 50:1 at 200 rpm yields 3200 × 3.33 = 10.7 kcounts/s, ~94 µs per quadrature state, so a 1–5 µs filter removes Hall-line ringing without dropping edges. Disable DFS/light sleep: the filter is specified in APB cycles.

**Task layout.** Control task pinned to core 1, woken by a 10 ms GPTimer alarm (100 Hz; 200 Hz if the motors are stiff); comms task on core 0 for UART RX/COBS/CRC and telemetry TX; low-priority sensor task polling I2C ToF and INA226 at 20–50 Hz. Keep Wi-Fi off so core 0 is not shared with the radio stack. FreeRTOS tick 1000 Hz is required by Arduino-as-component anyway.

**Control law.** Velocity PI with static feedforward (duty ≈ k_v·v_target + k_static·sign(v_target)) handles gearmotor friction deadband better than bare PID; add D only for visible overshoot. Clamp the integrator to duty saturation. Ramp setpoints at 0.5 m/s² and 1 rad/s² so the PI never sees a step. Tune by streaming encoder velocity at 100 Hz to the Pi and stepping the setpoint; a 100–200 rpm gearmotor under a 4–5 kg rover typically settles in 150–300 ms [INFERRED, no 2025–2026 benchmark found]. `diff_drive_controller` exposes `max_velocity`, `max_acceleration`, `max_jerk` per axis; mirror those names to ease migration.

### Command TTL, watchdogs, faults

- The TTL is a check in the control loop: `if (now - last_valid_cmd_us > 300000) target = 0, fault |= TTL`. Commands arrive at 20 Hz so 300 ms is six missed frames. At 0.3 m/s the rover coasts 9 cm during the TTL plus a few cm of braking [INFERRED]. `diff_drive_controller` defaults `cmd_vel_timeout` to 0.5 s; 300 ms is stricter than the ROS default and consistent with the brief.
- On expiry: **brake** (both inputs high on MDD3A and TB6612; DRV8871 needs IN1=IN2=1 for brake, both low is coast). Braking a 0.3 m/s rover stops it in centimetres. On a **driver fault or e-stop** the motor rail is gone so the motors coast; gearbox friction stops the rover in a comparable distance at this speed [INFERRED].
- Latched faults: TTL expiry, stall, overcurrent, bumper, e-stop-observed, undervoltage. Latched means the control loop keeps targets at zero until the Pi sends an explicit `clear_faults` frame with a fresh sequence number and the physical cause is gone (e-stop released, bumper open). TTL expiry can auto-clear on the next valid command; the rest should not.
- **Task Watchdog** (`esp_task_wdt_add()` on the control task, timeout 1 s, `CONFIG_ESP_TASK_WDT_PANIC=y` so expiry resets instead of just printing a backtrace). It is built on MWDT in Timer Group 0, so a hung control task reboots the chip; MCPWM outputs return to their reset state (GPIO input, pulled low by the driver's pull-downs). Do not implement the TTL with it; the TWDT resets the whole system, which is the wrong response to a Pi hiccup.
- **MCPWM fault GPIO** in OST mode: OR together driver nFAULT, INA226 ALERT, bumper switch, and e-stop monitor with a couple of diodes into one active-low fault pin. The hardware brakes PWM before firmware notices; firmware then latches the fault.

### Current, stall, thermal

- INA226: 16-bit, 0.1% gain error, 36 V bus, averaging up to 1024 samples, ALERT pin for over-current on shunt voltage. INA219: 12-bit, 26 V, no averaging. ACS712: analog Hall, ±100 mA noise, and the S3 ADC is nonlinear; avoid. One INA226 on the motor rail with a 10 mΩ shunt (2 mA LSB at the 80 mV full-scale) gives total current and an ALERT trip at, say, 6 A. Per-motor current needs two INA226 (A0/A1 address pins) or a driver with a sense output (VNH5019 CS analog).
- DRV8871 has internal resistor-set current limiting: Adafruit's 30 kΩ default limits at ~2 A; formula from TI datasheet is I_trip ≈ 64 kΩ·A / R_ILIM [VENDOR, not re-verified this session].
- Stall detection: latch a stall when `|duty| > 40%` and `|encoder velocity| < 5% of commanded` for > 250 ms, or current > 2× rated for > 100 ms. No 2025–2026 measured reference found; this is standard practice in linorobot-class firmware [INFERRED].
- Thermal: MDD3A/TB6612 have thermal shutdown on the IC (TB6612 yes, MDD3A datasheet lists only reverse-polarity protection). Add an I²t-style software limit (integrate current above rated, trip after a budget) and a $1 NTC on the driver if you use MDD3A.

### Hardware e-stop

Use a latching (twist-to-release) 22 mm NC mushroom. Wire the NC contact in series with the relay coil supply (5 V coil from the logic buck, or 12 V automotive relay coil) so pressing the button drops the relay and opens the motor rail (VB+ to the driver). Logic (ESP32, Pi, sensors) is fed from a separate buck tapped upstream of the relay, so it stays on. Put the ESP32's "motor enable" GPIO in series with the coil through a small NPN/MOSFET: the ESP32 can open the relay, but it cannot bypass an open button. Monitor the coil side with a divider into a GPIO to know why the motors stopped. A 2018 hobby rover (mowerproject) used exactly this: NC FIT0156 button, $3 dual relay module rated 30 A / 24 V, motor power only, non-latching relay. Add a flyback diode, a 10–15 A fuse on the motor rail, and 30 A-rated wire. A smart high-side MOSFET (VN7040, BTS50055) is lighter and quieter but needs a gate path that the ESP32 does not control; the relay stays the practical hobbyist choice in 2026.

### Obstacle sensors

- VL53L1X: 27° FoV, short mode ≤ ~130 cm at 50 Hz (20 ms timing budget), medium/long ≤ 300/400 cm at 30 Hz, 400 kHz I2C, default address 0x29, $22.95 (Pololu). At 0.3 m/s a 20 ms budget is 6 mm of travel. Mounted 8–10 cm above the floor with 27° FoV (13.5° half-angle) the cone touches the floor at ~35–42 cm [INFERRED], so either tilt 5–10° up, shrink the ROI to the upper rows, or accept that far readings are floor returns.
- VL53L4CX: 18° FoV, 1 mm–6 m, linear down to 10 mm, multi-target, $14.95 (Adafruit). Newer, cheaper and better at close range than the L1X; a 2026 builder starting fresh picks it (or the cheaper VL53L4CD).
- VL53L5CX: 8x8 zones at 15 Hz or 4x4 at 60 Hz, 63° diagonal FoV, 4 m, ~$25–30. One forward unit gives a coarse depth strip plus cliff detection from the lower rows; ST's ULD driver carries an ~86 kB firmware blob, fine on the S3 (Hackaday, Feb 2026, shows an ESP32 streaming it in real time).
- Bumper: NC microswitch to GND with pull-up; read in the 100 Hz loop with a two-sample confirm (20 ms) or a 5 ms hardware RC; also feed the MCPWM fault pin.
- HC-SR04 ultrasonic: 15° cone, ~40 Hz, 2 cm–4 m, 5 V logic (level-shift Echo). Cheap forward backup for glass and black surfaces that ToF misses [INFERRED].

### Serial link

- ESP32-S3 USB-Serial/JTAG enumerates as `/dev/ttyACM*`. The ESP-IDF guide lists its limits: no output during boot loops, disappears in deep sleep, a 50 ms stall when its buffer fills. Forum threads and the esptool docs agree the S3 resets on the DTR/RTS sequence and, unlike C6/H2 (`USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS`), has no disable bit. Arduino issue #9316 (fixed in 3.0.0-RC1) shows the class of HWCDC RX-FIFO bugs. Pi OS's ModemManager also probes new ttyACM devices.
- UART: linorobot2's ESP32 port runs micro-ROS serial at 1,500,000 baud on CP2102 and warns about /dev/ttyUSB0 confusion with LiDAR dongles. Direct 3.3 V UART to the Pi 4 PL011 (`/dev/ttyAMA0`, `dtoverlay=disable-bt`, serial console off) avoids enumeration and reset issues; that is my recommendation.
- Framing: COBS-encoded `[type][seq][payload][CRC-16/CCITT]` terminated by 0x00. A 40-byte telemetry frame at 50 Hz is ~20 kbit/s; 921600 leaves 40× headroom. nanopb is optional; fixed little-endian structs suffice for four message types. Sequence numbers both ways; a repeated or skipped seq is a link fault.

### micro-ROS and reference firmware

- `micro_ros_espidf_component`: branches humble (22.x), jazzy (24.x), kilted (25.x, default in `micro_ros_platformio`), rolling (27.x); tested on IDF v5.4/5.5/6.0 and ESP32-S3; UART via `colcon.meta` `-DRMW_UXRCE_TRANSPORT=custom`; README says "not ready for production use". The agent runs on the host as Docker.
- linorobot2_hardware (hippo5329 fork, wiki revised 2026-09-05): ESP32/S2/S3 via PCNT, Generic 2-IN (TB6612/L298N) and BTS7960 drivers, cmd_vel → PID → odom over micro-ROS. Best reference for kinematics, PID structure and encoders.
- ESP-IDF `mcpwm_bdc_speed_control`: MCPWM + PCNT + `pid_ctrl`; the cleanest pure-IDF start.
- ROS2-ESP32-Serial-Bridge: text protocol at 115200, 30 Hz PID, 2 s timeout. Borrow the command set, not the framing or timeout.
- diffdrive_arduino: `ros2_control` hardware interface over serial; the later Nav2 seam without touching MCU firmware.
- OpenBot firmware (Nano/ESP32 dev kit): sonar and bumper plumbing only.

## Recommendation for this build

1. **Toolchain:** ESP-IDF v5.5.5 with Arduino-ESP32 3.3.11 as a component (or pioarduino 55.03.311 for VS Code). Pin versions in `idf_component.yml`. Revisit IDF 6.x when Arduino 4.0 leaves alpha.
2. **Peripherals:** MCPWM group 0, one 20 kHz timer, two operators, OST fault input fed by an OR of driver fault / INA226 ALERT / bumper / e-stop monitor. Two PCNT units, `accum_count`, glitch filter 1000 ns. GPTimer at 10 ms wakes the control task on core 1.
3. **Control:** per-wheel PI + feedforward, integrator clamp, ramps 0.5 m/s² and 1 rad/s², MCU hard caps 0.35 m/s and 1.2 rad/s regardless of frame contents.
4. **Safety:** 300 ms TTL → brake, latch `TTL`; stall (duty > 40%, speed < 5%, 250 ms) and overcurrent (INA226 ALERT) latch; `clear_faults` frame required; Task Watchdog 1 s, panic-reset, on the control task. E-stop: NC latching button in the relay-coil path on the motor rail only, ESP32 enable transistor in series, coil state read back.
5. **Sensors:** two VL53L4CX at the front corners angled ±15°, 8–10 cm high, 20 ms budget, stop at 25 cm, slow zone 50 cm; two microswitch bumpers; one HC-SR04 forward backup. One VL53L5CX can replace the two L4CX.
6. **Link:** GPIO UART to Pi PL011 at 921600, COBS + CRC-16 + seq; Pi→MCU 20 Hz (v, ω, seq, flags); MCU→Pi 50 Hz (counts, wheel velocities, current, fault bitmask, ToF, bumper, e-stop, seq echo). Fields map 1:1 to `geometry_msgs/Twist` and `nav_msgs/Odometry`.
7. **ROS seam:** custom protocol now; later a `ros2_control` hardware interface on the Pi (diffdrive_arduino pattern), or micro-ROS kilted if you accept running the agent.
8. **Motors/driver:** JGB37-520 90:1 12 V (~170 rpm, ~8.5 kg·cm rated, ≤3.5 A stall, 11 PPR → 3960 CPR at the wheel with 4× decoding) on MDD3A; or Pololu 37D 50:1 (200 rpm, 21 kg·cm stall, 5.5 A stall, 3200 CPR) on MDD10A or 2× DRV8871. 85–90 mm wheels put cruise at 64–67 rpm, a third of no-load speed.

## Numbers

| quantity | value | hardware/context | tag | source URL |
|---|---|---|---|---|
| ESP-IDF v5.5 support end | 2028-01-21; maintenance from July 2026 | release/5.5 | VENDOR | https://github.com/espressif/esp-idf/blob/master/ROADMAP.md |
| ESP-IDF v6.0 / v6.1 release | 2026-02-27 / 2026-07-31 (tag late Aug) | esp-idf | VENDOR | https://github.com/espressif/esp-idf/blob/master/ROADMAP.md |
| Arduino-ESP32 latest stable | v3.3.11, 2026-07-22, on IDF v5.5.5 | arduino-esp32 | VENDOR | https://github.com/espressif/arduino-esp32/releases/tag/3.3.11 |
| Arduino-ESP32 on IDF 6 | v4.0.0-alpha1 only, missing Matter/RainMaker/ESP_SR | arduino-esp32 | VENDOR | https://github.com/espressif/arduino-esp32/releases |
| pioarduino latest | 55.03.311 (Jul 24, 2026) = Arduino 3.3.11 / IDF 5.5.5 | PlatformIO fork | VENDOR | https://github.com/pioarduino/platform-espressif32/releases |
| Official PlatformIO espressif32 | Arduino core 2.0.17, no 3.x | platformio | VENDOR | https://github.com/espressif/arduino-esp32/discussions/10039 |
| esp-hal | 1.0.0 on 2025-10-30; 1.2.0 on 2026-09-02; MCPWM/PCNT/Timer `unstable` | Rust, ESP32-S3 | VENDOR | https://docs.rs/esp-hal/latest/esp_hal/ |
| Arduino-as-component requirement | FreeRTOS tick 1000 Hz; `idf.py add-dependency "espressif/arduino-esp32^3.3.11"` | IDF 5.5 | VENDOR | https://docs.espressif.com/projects/arduino-esp32/en/latest/esp-idf_component.html |
| LEDC S3 resources | 8 channels, 4 timers, low-speed only, 80 MHz APB; 5 kHz → 13-bit | ESP32-S3 | VENDOR | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/ledc.html |
| LEDC at 20 kHz | 11-bit max (4000 ticks) | ESP32-S3 | INFERRED | same |
| MCPWM at 20 kHz | ~8000 ticks from PLL_160M | ESP32-S3 | INFERRED | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/mcpwm.html |
| MCPWM fault | GPIO fault, CBC or OST brake, `mcpwm_operator_recover_from_fault()` | ESP32-S3 | VENDOR | same |
| PCNT counter | 16-bit hardware, `accum_count` extends; filter on APB clock 12.5 ns | ESP32-S3, IDF 6.1 doc | VENDOR | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/pcnt.html |
| PCNT glitch filter ceiling | ~12.8 µs (10-bit × 12.5 ns) | ESP32-S3 | INFERRED | same |
| pid_ctrl component | v0.3.1, float + IQmath | ESP-IDF registry | VENDOR | https://components.espressif.com/components/espressif/pid_ctrl |
| Task WDT | MWDT Timer Group 0; default action warn+backtrace; `CONFIG_ESP_TASK_WDT_PANIC` resets | ESP32-S3 | VENDOR | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html |
| ROS cmd_vel timeout | 0.5 s default | ros2_controllers diff_drive_controller | VENDOR | https://control.ros.org/rolling/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html |
| Coast distance during 300 ms TTL | ~9 cm at 0.3 m/s | any | INFERRED | — |
| USB-Serial/JTAG buffer stall | one-time 50 ms wait when buffer full; `/dev/ttyACM*` | ESP32-S3 | VENDOR | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/usb-serial-jtag-console.html |
| S3 DTR/RTS reset disable | not available; only C6/H2 have `USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS` | ESP32-S3 | MEASURED (forum) | https://www.esp32.com/viewtopic.php?t=37208 |
| linorobot2 serial | 1,500,000 baud CP2102; WiFi UDP preferred; PCNT encoders | ESP32/S3 | MEASURED | https://github.com/hippo5329/linorobot2_hardware/wiki |
| micro-ROS IDF support | IDF 5.4/5.5/6.0; branches humble/jazzy/kilted/rolling; UART via `-DRMW_UXRCE_TRANSPORT=custom` | ESP32-S3 | VENDOR | https://github.com/micro-ROS/micro_ros_espidf_component |
| VL53L1X | 27° FoV; short ≤130 cm @ 50 Hz; medium/long ≤300/400 cm @ 30 Hz; 20 mA; 400 kHz I2C; $22.95 | Pololu 3415 | VENDOR | https://www.pololu.com/product/3415 |
| VL53L1X timing budget | 20 ms–1000 ms | ST datasheet | VENDOR | https://www.st.com/resource/en/datasheet/vl53l1x.pdf |
| VL53L4CX | 18° FoV; 1 mm–6 m; linear to 10 mm; multi-target; $14.95 | Adafruit 5425 | VENDOR | https://www.adafruit.com/product/5425 |
| VL53L5CX | 8x8 @ 15 Hz, 4x4 @ 60 Hz, 63° diagonal, 4 m | ST datasheet / ST community | VENDOR | https://community.st.com/t5/imaging-sensors/vl53l5cx-set-ranging-frequency-hz-and-tof-sensors/td-p/79984 |
| VL53L1X floor intercept | ~35–42 cm at 8–10 cm height, 13.5° half-angle | rover | INFERRED | — |
| MDD3A | 4–16 V; 3 A cont / 5 A peak <5 s; logic high 1.7–12 V; PWM DC–20 kHz; reverse-polarity protection; both-high = brake | Cytron datasheet Rev 1.0 | VENDOR | https://cdn.robotshop.com/media/c/cyt/rb-cyt-260/pdf/cytron-3a-4-16v-dual-channel-dc-motor-driver-datasheet.pdf |
| MDD10A | 5–30 V; 10 A cont / 30 A 10 s peak per channel | Cytron | VENDOR | https://www.cytron.io/p-10amp-5v-30v-dc-motor-driver-2-channels |
| TB6612FNG carrier | 4.5–13.5 V; 1 A cont / 3 A peak; 100 kHz PWM; $4.95 | Pololu 713 | VENDOR | https://www.pololu.com/product/713 |
| DRV8871 breakout | 6.5–45 V; 3.6 A peak; 30 kΩ → ~2 A limit; 565 mΩ; $7.50 | Adafruit 3190 | VENDOR | https://www.adafruit.com/product/3190 |
| Pololu 37D 50:1 | 200 rpm, 21 kg·cm stall, 5.5 A stall, 3200 CPR, 10 kg·cm cont. limit, $60.95 | 12 V | VENDOR | https://www.pololu.com/product/4753 |
| Pololu 37D 30:1 | 330 rpm, 14 kg·cm stall, 1920 CPR, $60.95 | 12 V | VENDOR | https://www.pololu.com/product/4752 |
| JGB37-520 (NFP-GM37-520-EN) | ratios 6.3:1–450:1; 10–1530 rpm; rated 0.2–25 kg·cm; stall ≤2.5–3.5 A at 12 V; 11 PPR Hall; $20 | 12 V | VENDOR | https://microdcmotors.com/product/6v-12v-geared-dc-electric-motor-with-encoder-nfp-jgb37-520-en |
| Cruise wheel speed | 64–88 rpm at 0.3 m/s for 90–65 mm wheels | rover | INFERRED | — |
| Continuous torque need | ~1 kg·cm/wheel (5 kg, 0.5 m/s², Crr 0.05, r 42.5 mm); 3–5 kg·cm transient | rover | INFERRED | — |
| INA226 vs INA219 | 16-bit/0.1% vs 12-bit/1%; 36 V vs 26 V; INA226 averages ≤1024 samples | I2C monitors | VENDOR | https://hackaday.io/project/204686-mastering-the-ina219-ina226 |

## Corrections to the brief

- "ESP32-S3: wheel PID, 300 ms command TTL → stop, ToF/bumper stop, current limit, e-stop. Serial to Pi." Holds. Two additions: implement the TTL inside the control loop (not the Task Watchdog), and use the MCPWM fault input so bumper/e-stop/overcurrent cut PWM in hardware. ROS convention is 0.5 s; 300 ms is fine.
- "Serial to Pi" is underspecified. Use GPIO UART to the Pi's PL011 or a CP2102/CH343 bridge, not the S3's native USB: the S3 cannot disable the DTR/RTS reset and Pi OS's ModemManager probes ttyACM devices.
- BOM "motor driver $8–15": holds for MDD3A (~$12–15) or 2× DRV8871 ($15) with JGB37-class motors. If you choose Pololu 37D (5.5 A stall) you need MDD10A (~$28) or DRV8871 with its current limit; TB6612 (1 A cont) is undersized for 37D.
- BOM "2× encoded gearmotors + chassis + caster $50–100": holds with JGB37-520 ($20 each). Pololu 37D is $60.95 each, so that option is $150–200 with chassis.
- BOM "2× VL53L1X + bumper + e-stop $15–20": genuine VL53L1X boards are $22.95 each (Pololu), so two alone exceed the line. Either use $5–8 clones, or switch to VL53L4CX at $14.95 each; realistic line is $35–60.
- "3S Li-ion" (9.6–12.6 V) rules out DRV8833 (10.8 V max). MDD3A (16 V), TB6612 (13.5 V), DRV8871 (45 V), MDD10A (30 V) are fine.
- Gate 2 "cable pull → stop ≤300 ms": specify it as "PWM zeroed within 300 ms of the last valid frame; vehicle stationary within 0.5 s". Braking adds tens of ms and the encoder is the only witness.
- Gate 2 "e-stop kills motor rail with everything else off": confirm this means "with logic still powered". The relay-coil design does that; a battery main switch does not.
- Skills "drive(≤0.3 m/s), turn(≤60°/s)": also cap inside the MCU (0.35 m/s, 1.2 rad/s, 0.5 m/s²) so a bad Pi frame cannot exceed them.

## Alternatives considered and rejected

- **ESP-IDF v6.x now:** Arduino 4.0 is alpha; v5.5 is supported to 2028. Revisit in 2027.
- **Official PlatformIO:** frozen at Arduino 2.0.17.
- **Rust esp-hal:** motor peripherals unstable; Xtensa toolchain; no gain at this scope.
- **LEDC for motor PWM:** works (linorobot2 uses it) but no hardware fault brake.
- **Task Watchdog as TTL:** resets the whole chip; wrong granularity.
- **Native USB-Serial/JTAG:** DTR/RTS reset not disableable on S3; console lost in boot loops; ModemManager.
- **micro-ROS first:** needs a ROS 2 agent on a Pi already carrying Vosk/Piper and ties firmware to a distro line. Keep as the later seam.
- **L298N:** 1.8–2.5 V BJT drop. **DRV8833:** 10.8 V max. **BTS7960:** 43 A half-bridges are oversized and noisy for 5 kg. **VNH5019:** good (12 A, current sense) but $30+ per channel.
- **N20:** ~2–3 kg·cm stall and brush wear; too weak for 5 kg on carpet [INFERRED]. **JGA25-370:** fine for 3 kg on hard floors, marginal at 5 kg on carpet [INFERRED].
- **ACS712:** noise plus S3 ADC nonlinearity.
- **Software-only e-stop (GPIO to driver enable):** a hung task leaves motors running.
- **High-side MOSFET e-stop:** viable but needs a gate network the ESP32 cannot reach; relay is simpler.

## Open questions

- No 2025–2026 measurement of PI settling time or PWM-vs-encoder stall thresholds on JGB37/37D-class motors was found; tune on the bench and log at 100 Hz.
- Whether the Pi 4 PL011 at 921600 with the Pi under voice-pipeline load drops bytes over hours: test in Gate 3 with the seq counter.
- Exact DRV8871 I_trip formula and whether its "automagic" PWM handling introduces deadband at low duty: check the TI datasheet before choosing it over MDD3A.
- VL53L4CX minimum timing budget and max ranging rate were not confirmed from the datasheet (page fetch timed out); assume ~20 ms/50 Hz class like L1X and verify.
- MDD3A has no over-current or thermal protection listed; confirm whether the current Rev 2 board added any.
- micro-ROS agent CPU cost on a Pi 4 sharing cores with Vosk/Piper is unmeasured; matters only if you move to micro-ROS.

## Sources

- ESP-IDF ROADMAP.md — https://github.com/espressif/esp-idf/blob/master/ROADMAP.md — fetched 2026-09-07
- ESP-IDF releases — https://github.com/espressif/esp-idf/releases — fetched 2026-09-07 (v6.1, v6.0.3, v5.5.5)
- Arduino-ESP32 releases — https://github.com/espressif/arduino-esp32/releases — fetched 2026-09-07
- Arduino Release v3.3.11 — https://github.com/espressif/arduino-esp32/releases/tag/3.3.11 — 2026-07-22
- Arduino as ESP-IDF component — https://docs.espressif.com/projects/arduino-esp32/en/latest/esp-idf_component.html — fetched 2026-09-07
- 3.x Release to PlatformIO (Discussion #11308) — https://github.com/espressif/arduino-esp32/discussions/11308 — 2025
- Community PlatformIO support for core 3.x (Discussion #10039) — https://github.com/espressif/arduino-esp32/discussions/10039 — 2024–2025
- pioarduino platform-espressif32 releases — https://github.com/pioarduino/platform-espressif32/releases — 55.03.311, 2026-07-24
- esp-hal 1.0.0 announcement — https://developer.espressif.com/blog/2025/10/esp-hal-1/ — 2025-10-30
- esp-hal docs.rs — https://docs.rs/esp-hal/latest/esp_hal/ — 1.2.0, 2026-09-02
- ESP-IDF MCPWM (ESP32-S3, stable) — https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/mcpwm.html — v6.1 docs
- ESP-IDF LEDC (ESP32-S3, stable) — https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/ledc.html — v6.1 docs
- ESP-IDF PCNT (ESP32-S3, stable) — https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/pcnt.html — v6.1 docs
- ESP-IDF Watchdogs (ESP32-S3, stable) — https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html — v6.1 docs
- ESP-IDF USB Serial/JTAG console (ESP32-S3) — https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/usb-serial-jtag-console.html — v6.1 docs
- ESP32s3 Disable JTAG/Serial RTS Reset (forum) — https://www.esp32.com/viewtopic.php?t=37208 — 2024
- arduino-esp32 issue #9316 (HWCDC RX interrupt) — https://github.com/espressif/arduino-esp32/issues/9316 — 2024
- mcpwm_bdc_speed_control example — https://github.com/espressif/esp-idf/tree/master/examples/peripherals/mcpwm/mcpwm_bdc_speed_control — master, 2026
- espressif/pid_ctrl component — https://components.espressif.com/components/espressif/pid_ctrl — v0.3.1, 2026
- nexenne: rotary encoders on ESP32 with PCNT — https://nexenne.com/blog/rotary_encoders_esp32_esp_idf_pcnt/ — 2025
- micro_ros_espidf_component — https://github.com/micro-ROS/micro_ros_espidf_component — updated 2026-08-21
- micro_ros_platformio — https://github.com/micro-ROS/micro_ros_platformio — fetched 2026-09-07
- linorobot2_hardware wiki (hippo5329) — https://github.com/hippo5329/linorobot2_hardware/wiki — revised 2026-09-05
- linorobot2_hardware upstream — https://github.com/linorobot/linorobot2_hardware — README note July 2025
- ROS2-ESP32-Serial-Bridge — https://github.com/manojramesh-io/ROS2-ESP32-Serial-Bridge — fetched 2026-09-07
- diffdrive_arduino (ros2_control HW interface) — https://github.com/GabyK/diffdrive_arduino — fetched 2026-09-07
- OpenBot firmware README — https://github.com/isl-org/OpenBot/blob/master/firmware/README.md
- diff_drive_controller userdoc — https://control.ros.org/rolling/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html — rolling, 2026
- Pololu VL53L1X carrier — https://www.pololu.com/product/3415 — fetched 2026-09-07
- ST VL53L1X datasheet — https://www.st.com/resource/en/datasheet/vl53l1x.pdf
- Adafruit VL53L4CX — https://www.adafruit.com/product/5425 — fetched 2026-09-07
- ST community: VL53L5CX ranging frequency — https://community.st.com/t5/imaging-sensors/vl53l5cx-set-ranging-frequency-hz-and-tof-sensors/td-p/79984
- Hackaday: Real-time 3D room mapping with ESP32 + VL53L5CX — https://hackaday.com/2026/02/14/real-time-3d-room-mapping-with-esp32-vl53l5cx-sensor-and-imu/ — 2026-02-14
- Cytron MDD3A datasheet Rev 1.0 — https://cdn.robotshop.com/media/c/cyt/rb-cyt-260/pdf/cytron-3a-4-16v-dual-channel-dc-motor-driver-datasheet.pdf — March 2019
- Pololu TB6612FNG carrier — https://www.pololu.com/product/713 — fetched 2026-09-07
- Adafruit DRV8871 breakout — https://www.adafruit.com/product/3190 — fetched 2026-09-07
- TI DRV8871 — https://www.ti.com/product/DRV8871
- Pololu 37D 50:1 with encoder — https://www.pololu.com/product/4753 — fetched 2026-09-07
- Pololu 37D 30:1 with encoder — https://www.pololu.com/product/4752 — fetched 2026-09-07
- NFP-GM37-520-EN (JGB37-520) — https://microdcmotors.com/product/6v-12v-geared-dc-electric-motor-with-encoder-nfp-jgb37-520-en — fetched 2026-09-07
- Zbotic: L298N vs DRV8833 vs TB6612FNG — https://zbotic.in/motor-driver-ic-comparison-l298n-vs-drv8833-vs-tb6612fng/
- Hackaday.io: Mastering the INA219 & INA226 — https://hackaday.io/project/204686-mastering-the-ina219-ina226
- microcontrollerslab: Best current & voltage sensors (2026 guide) — https://microcontrollerslab.com/best-current-voltage-sensors-microcontrollers/
- Mower Project: The Emergency Stop Switch — https://mowerproject.com/2018/09/14/the-emergency-stop-switch/ — 2018
- Beningo: Embedded serial communication with protobuf (COBS + CRC16 + nanopb) — https://www.beningo.com/embedded-serial-communication-protobuf/

## Verification (adversarial review)

Reviewed 2026-09-07. WebSearch budget was already exhausted (200/200) when this review started, so every check below is a direct WebFetch of a primary page (GitHub releases/tags, ESP-IDF source headers on release/v5.5, ESP-IDF v6.1 docs, vendor product pages, TI/Cytron datasheets read via pdftotext). Pages that refused (esp32.com forum bot-wall, cytron.io 403, robotshop 403, pinout.xyz 403, Raspberry Pi docs truncated) are named where they mattered.

| claim | verdict | evidence | source URL | corrected claim |
|---|---|---|---|---|
| ESP-IDF v5.5.5 is the current v5.5 bugfix; release/5.5 in maintenance from July 2026; v6.1 released (roadmap 2026-07-31, tag late Aug); v6.0.3 on Sept 2 | confirmed | GitHub releases lists v5.5.5 (17 Jul), v6.1 (27 Aug, "latest"), v6.0.3 (2 Sep). ROADMAP.md: "push release/5.5 to maintenance period from July 2026", planned v5.5.6 2026-10-08, v5.5.7 2027-01-11, v6.0 released 2026-02-27, v6.1 planned 2026-07-31 | https://github.com/espressif/esp-idf/releases ; https://raw.githubusercontent.com/espressif/esp-idf/master/ROADMAP.md | — |
| v5.5 "supported to 2028-01-21" | needs_qualifier | No 2028 date appears in ROADMAP.md or on the v5.5.5 release page. The v5.5 tag is dated 21 Jul (2025) and the versions page states each minor release is supported 30 months (12 service + 18 maintenance); 2025-07-21 + 30 months = 2028-01-21 | https://github.com/espressif/esp-idf/releases/tag/v5.5 ; https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/versions.html | v5.5 EOL ≈ January 2028 is derived from the 30-month policy; Espressif publishes it only as a chart. Last planned bugfix is v5.5.7 (2027-01-11). |
| Arduino-ESP32 3.3.11 (2026-07-22) is built on IDF v5.5.5; 4.0.0-alpha1 (May 2026) is the only IDF-6 core and lacks Matter/RainMaker/ESP_SR/Insights/CBOR | confirmed | Releases page: 3.3.11 Jul 22 2026 on v5.5.5; 3.3.10/3.3.9 Jun 2026 on v5.5.4; 4.0.0-alpha1 May 27 2026 on v6.0.1+, "not yet available: Matter, RainMaker, ESP_SR, Insights, Cbor". Component doc: `idf.py add-dependency "espressif/arduino-esp32^3.3.11"`, CONFIG_FREERTOS_HZ=1000 required | https://github.com/espressif/arduino-esp32/releases ; https://docs.espressif.com/projects/arduino-esp32/en/latest/esp-idf_component.html | — |
| pioarduino 55.03.311 (Jul 24 2026) = Arduino 3.3.11 / IDF 5.5.5; official PlatformIO "frozen at Arduino core 2.0.17" | needs_qualifier | pioarduino releases: 55.03.311 Jul 24 = 3.3.11/5.5.5 (latest). Official platformio/platform-espressif32 v7.1.1 (Sep 5 2026; v7.1.0 Aug 31) bundles Arduino v2.0.17 (IDF 4.4.7) but also framework-espidf v6.1.0 | https://github.com/pioarduino/platform-espressif32/releases ; https://github.com/platformio/platform-espressif32/releases ; https://api.registry.platformio.org/v3/packages/platformio/platform/espressif32 | Official PlatformIO is frozen only for the Arduino framework (2.0.17). For pure ESP-IDF projects it ships IDF 6.1.0, so "official PlatformIO" is a valid IDF-only path. |
| Rust esp-hal 1.0.0 shipped 2025-10-30; 1.2.0 on 2026-09-02; MCPWM, PCNT, Timer (plus LEDC, RMT, DMA) still `unstable`; GPIO/UART/SPI/I2C stable | confirmed | docs.rs shows 1.2.0 (Sep 2 2026) with GPIO/UART/SPI/I2C stable and mcpwm/pcnt/timer/ledc/rmt/dma behind `unstable`. GitHub releases: v1.0.0 Oct 30, v1.1.0 Apr 24, v1.2.0 Sep 3; v1.2.0 "did not stabilize any new public APIs" | https://docs.rs/esp-hal/latest/esp_hal/ ; https://github.com/esp-rs/esp-hal/releases | Note omits 1.1.0 (Apr 2026). The "espup Xtensa fork toolchain" requirement was not re-verified (esp-rs book pages 404). |
| ESP32-S3 USB-Serial/JTAG resets on the DTR/RTS pattern and has no disable bit; only C6/H2 have `USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS` | confirmed | release/v5.5 `soc/esp32c6/register/soc/usb_serial_jtag_reg.h` defines `USB_SERIAL_JTAG_CHIP_RST_REG` ("CDC-ACM chip reset control") with bit `USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS` ("Set this bit to disable chip reset from usb serial channel"). The esp32s3 header defines 20 registers and none with CHIP_RST; neither chip's `usb_serial_jtag_ll.h` exposes a reset-disable on S3. Docs confirm the 50 ms TX-buffer stall, deep-sleep disconnect and light-sleep clock gating. The forum thread the note cites was behind a bot wall | https://raw.githubusercontent.com/espressif/esp-idf/release/v5.5/components/soc/esp32c6/register/soc/usb_serial_jtag_reg.h ; https://raw.githubusercontent.com/espressif/esp-idf/release/v5.5/components/soc/esp32s3/register/soc/usb_serial_jtag_reg.h ; https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/usb-serial-jtag-console.html | "No output during boot loops" is not in the current console guide; the other limits are. |
| MCPWM on S3: two groups, each 3 timers, 3 operators, "6 generators (12 outputs)"; GPIO fault module with CBC/OST brake and `mcpwm_operator_recover_from_fault()`; OST brake acts "within one PWM period" | needs_qualifier | release/v5.5 soc_caps.h: `SOC_MCPWM_GROUPS (2)`, `TIMERS_PER_GROUP (3)`, `OPERATORS_PER_GROUP (3)`, `GENERATORS_PER_OPERATOR (2)` → 6 outputs per group, 12 total; `SOC_MCPWM_GPIO_FAULTS_PER_GROUP (3)`. v6.1 docs confirm CBC auto-recover, OST needs `mcpwm_operator_recover_from_fault()`, `mcpwm_new_gpio_fault` with `active_level`/`pull_up`/`pull_down`. Docs do not state brake latency | https://raw.githubusercontent.com/espressif/esp-idf/release/v5.5/components/soc/esp32s3/include/soc/soc_caps.h ; https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/mcpwm.html | 6 PWM outputs per group (12 across both groups). Each group has 3 GPIO fault inputs, so up to three sources (e-stop, driver nFAULT, bumper) need no diode-OR. Brake latency is undocumented; "within one PWM period" is an inference. |
| PCNT: 16-bit hardware counter, `accum_count` widens it, glitch filter on APB (12.5 ns), 10-bit register → ~12.8 µs ceiling; disable DFS | confirmed | `pcnt_ll.h` (release/v5.5): `PCNT_LL_MAX_GLITCH_WIDTH 1023`, `PCNT_LL_MAX_LIM SHRT_MAX`, `PCNT_LL_MIN_LIM SHRT_MIN`; 1023 × 12.5 ns = 12.79 µs. v6.1 docs: filter on APB, DFS warning, and "the driver automatically installs a power management lock". soc_caps: 4 PCNT units, 2 channels each | https://raw.githubusercontent.com/espressif/esp-idf/release/v5.5/components/hal/esp32s3/include/hal/pcnt_ll.h ; https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/pcnt.html | Driver already holds a PM lock while the filter is enabled; the remaining hazard is light sleep, not DFS. |
| Task Watchdog is MWDT in Timer Group 0; default action warn + backtrace; `CONFIG_ESP_TASK_WDT_PANIC=y` → panic and reset | confirmed | v6.1 wdts page: TWDT "built using the MWDT_WDT watchdog timer in Timer Group 0"; default "print a warning and a backtrace before continuing"; PANIC option causes "a panic and a system reset" | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html | — |
| LEDC on S3: 8 channels, 4 timers, low-speed only, 80 MHz APB; 5 kHz → 13 bits; 20 kHz → 11 bits | confirmed | v6.1 LEDC page: 8 channels, 4 timers, "ESP32-S3 only supports ... low speed mode", APB 80 MHz, "5 kHz ... maximum duty resolution of 13 bits"; soc_caps `SOC_LEDC_CHANNEL_NUM (8)`, `SOC_LEDC_TIMER_BIT_WIDTH (14)`. 80e6/20e3 = 4000 ticks → floor(log2 4000) = 11 bits | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/ledc.html | — |
| `diff_drive_controller` defaults `cmd_vel_timeout` to 0.5 s and exposes max_velocity / max_acceleration / max_jerk per axis | confirmed | control.ros.org rolling userdoc: cmd_vel_timeout default 0.5 s, "A value of 0.0 disables the timeout"; limits are `linear.x.max_velocity`, `linear.x.max_acceleration`, `linear.x.max_jerk`, `angular.z.*` (default NaN) | https://control.ros.org/rolling/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html | — |
| JGB37-520 90:1 ≈ 170 rpm, 11 PPR Hall, $20, 8.5 kg·cm rated, ≤3.5 A stall; cruise 64–67 rpm is "a third of no-load speed" | refuted | The vendor page the note cites lists 12 V no-load speeds: 50:1 = 192 rpm, **90:1 = 107 rpm**, 150:1 = 64 rpm; 90:1 rated 8.5 kg·cm, stall ≤3.5 A; 11 PPR → 990 PPR at 90:1; $20 | https://microdcmotors.com/product/6v-12v-geared-dc-electric-motor-with-encoder-nfp-jgb37-520-en | JGB37-520 90:1 is ~107 rpm at 12 V, so 64–67 rpm cruise is ~60% of no-load, not a third. For one-third headroom pick 50:1 (192 rpm, 11×50×4 = 2200 CPR). Price, PPR, torque and stall current hold. |
| Cytron MDD3A: 4–16 V, 3 A continuous / 5 A peak <5 s, PWM DC–20 kHz, logic high 1.7–12 V, both-high = brake, reverse-polarity protection only | confirmed | Datasheet Rev 1.0 (text extracted): Vin 4–16 VDC; continuous 3 A, peak (<5 s) 5 A; logic high 1.7–12 V; PWM DC–20 kHz; truth table Low/Low = Brake, High/High = Brake; Protection section lists only reversed-polarity. cytron.io and robotshop product pages returned 403 so a Rev 2 check was not possible | https://cdn.robotshop.com/media/c/cyt/rb-cyt-260/pdf/cytron-3a-4-16v-dual-channel-dc-motor-driver-datasheet.pdf | No input pull-down is documented; see missing list. |
| Pololu 37D 50:1: 200 rpm, 21 kg·cm stall, 5.5 A stall, 3200 CPR, 10 kg·cm continuous limit, $60.95 | confirmed | Pololu #4753: "200 RPM, 200 mA", stall 21 kg·cm, 5.5 A, 64 CPR motor shaft / 3200 CPR output, continuous limit 10 kg·cm, $60.95 | https://www.pololu.com/product/4753 | — |
| VL53L1X (Pololu) $22.95, 27° FoV, short ≤130 cm @50 Hz, medium/long 300/400 cm @30 Hz, 400 kHz I2C, 20 mA; VL53L4CX (Adafruit) $14.95, 18° FoV, 1 mm–6 m, multi-target, linear to 10 mm | confirmed | Pololu #3415 and Adafruit #5425 pages state exactly these figures | https://www.pololu.com/product/3415 ; https://www.adafruit.com/product/5425 | — |
| VL53L5CX: 8x8 @15 Hz, 4x4 @60 Hz, 63° diagonal, 4 m, ~86 kB firmware blob, ~$25–30 | needs_qualifier | SparkFun #18642: 4x4 or 8x8 zones, 60 Hz frame-rate capability, 15 Hz for all (8x8) zones, 63° diagonal, up to 400 cm, firmware "~90KB" loaded over I2C at every power-on, $32.50 | https://www.sparkfun.com/products/18642 | Specs hold; boards are $32.50 at SparkFun (cheaper only from clone vendors) and the firmware upload is ~90 kB per power-up, which also adds start-up latency after every MCU reset. |
| DRV8833 max 10.8 V (rules out 3S); TB6612 4.5–13.5 V, 1 A/3 A, 100 kHz, $4.95; DRV8871 6.5–45 V, 3.6 A peak, I_trip ≈ 64 kΩ·A / R_ILIM (30 kΩ → ~2 A), IN1=IN2=1 brake, both low coast | confirmed | TI DRV8833 page: Vs min 2.7 V, absolute max 11.8 V (3S at 12.6 V exceeds it). Pololu #713: 4.5–13.5 V, 1 A cont/3 A peak, 100 kHz, $4.95. DRV8871 datasheet (text extracted): VM 6.5–45 V, ITRIP = VILIM (64 kV) / RILIM, "RILIM = 32 kΩ ... limits motor current to 2 A", min RILIM 15 kΩ, table: 0/0 = Coast (sleep after 1 ms), 1/1 = Brake (low-side slow decay) | https://www.ti.com/product/DRV8833 ; https://www.pololu.com/product/713 ; https://www.ti.com/lit/ds/symlink/drv8871.pdf | DRV8871 sleeps 1 ms after both-low; a coast command must be re-asserted or the driver needs 40–50 µs turn-on when leaving sleep. |
| INA226: 16-bit, 0.1% gain error, 36 V, averaging up to 1024, ALERT on shunt over-voltage; 10 mΩ shunt gives "2 mA LSB at the 80 mV full-scale" | needs_qualifier | Datasheet (text extracted): 16-bit native ADC, shunt input ±81.92 mV, shunt LSB 2.5 µV, bus LSB 1.25 mV, 0–36 V, gain error 0.1% max, AVG settings up to 1024, Alert pin with five selectable functions | https://www.ti.com/lit/ds/symlink/ina226.pdf | With a 10 mΩ shunt the shunt-register LSB is 0.25 mA and full scale is ±8.19 A; a "2 mA" current LSB only exists if the calibration register is set that coarsely. INA226 specs otherwise hold. |
| micro_ros_espidf_component: humble/jazzy/kilted/rolling branches; tested on IDF 5.4/5.5/6.0 and ESP32-S3; UART via `-DRMW_UXRCE_TRANSPORT=custom`; "not ready for production use" | confirmed | README: Humble 22.x, Iron 23.x, Jazzy 24.x, Kilted 25.x, Rolling 27.x; "ESP-IDF v5.4, v5.5, and v6.0"; boards incl. ESP32-S3; custom transport for UART; production-readiness disclaimer verbatim | https://github.com/micro-ROS/micro_ros_espidf_component | Note omits the Iron branch (irrelevant, Iron is EOL). |
| linorobot2's ESP32 port runs micro-ROS serial at 1,500,000 baud | needs_qualifier | hippo5329 wiki (edited Sep 5 2026): 921600 baud is the standard serial setting; 1,500,000 is an alternative for certain ESP32 configurations; ESP32/S2/S3 supported; PCNT encoders; TB6612/L298N generic 2-IN and BTS7960 drivers | https://github.com/hippo5329/linorobot2_hardware/wiki | linorobot2 defaults to 921600; 1.5 Mbaud is an optional ESP32 setting. |
| arduino-esp32 issue #9316 (HWCDC RX FIFO) fixed in 3.0.0-RC1; `espressif/pid_ctrl` is v0.3.1 with float + IQmath | confirmed | Issue #9316 "USB Serial JTAG Rx FiFo Interrupt Issue", ESP32-S3, reported on 2.0.6, milestone 3.0.0-RC1. Component registry: pid_ctrl 0.3.1, "float — pid_*_f() APIs" and "IQmath _iq — pid_*_iq() APIs" | https://github.com/espressif/arduino-esp32/issues/9316 ; https://components.espressif.com/components/espressif/pid_ctrl | — |
| Rust on ESP32-S3 needs the `espup` Xtensa fork toolchain | unverifiable | esp-rs book pages (docs.esp-rs.org and docs.espressif.com/projects/rust) returned 404; docs.rs only shows `xtensa-lx-rt` dependencies | https://docs.rs/esp-hal/latest/esp_hal/ | Treat as likely but unconfirmed this session. |

### Stale or missing

- **Motor speed error (load-bearing).** The recommended JGB37-520 90:1 runs ~107 rpm at 12 V, not ~170 rpm. Cruise at 64–67 rpm leaves ~40% headroom, not two-thirds. Either accept that or specify 50:1 (192 rpm, 2200 CPR at 4× decoding). The torque/price/PPR figures and the MDD3A pairing are unaffected.
- **Pi 4 has four spare PL011 UARTs.** UART2–UART5 (`dtoverlay=uart2` … `uart5`, GPIO 0/1, 4/5, 8/9, 12/13 → `/dev/ttyAMA1`–`ttyAMA4`) are PL011 blocks, so the ESP32 link does not need `dtoverlay=disable-bt` or GPIO14/15. UART5 on GPIO12/13 avoids I2C0/SPI clashes. Source: Raspberry Pi engineer post, https://forums.raspberrypi.com/viewtopic.php?t=244827 (the official docs page was truncated by the fetch tool). The note's `disable-bt` recommendation silently removes Bluetooth from the brief's Pi, which nothing in the brief needs but nothing in the brief authorises either.
- **Use the three MCPWM GPIO fault inputs, not a diode-OR.** `SOC_MCPWM_GPIO_FAULTS_PER_GROUP (3)`: e-stop monitor, driver nFAULT and bumper can each have their own fault object with per-input `active_level` and internal pull, so firmware learns which one tripped; only a fourth source (INA226 ALERT) needs sharing. Latency of the brake path is not documented; measure it on the bench before relying on "within one PWM period".
- **Driver input state during MCU reset/boot is asserted, not verified.** The note says outputs are "pulled low by the driver's pull-downs"; the MDD3A datasheet documents no input pulls, and the ESP-IDF GPIO guide warns not to rely on TRM default pin states because bootloader/startup code may change them. MDD3A and DRV8871 brake on both-high so any common pull state is safe; a PWM+DIR driver (MDD10A option) is not — a PWM pin floating high at reset means full speed. Add external pull-downs on every PWM/enable line and keep them off the S3 strapping pins GPIO0, GPIO3, GPIO45, GPIO46 (https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html).
- **ESP-IDF EOL date is derived, not published.** 2028-01-21 is 30 months after the v5.5 tag (2025-07-21); the roadmap only commits to v5.5.7 on 2027-01-11 and says nothing about 2028. Cite it as "≈ Jan 2028 per the 30-month policy".
- **Official PlatformIO is not frozen for IDF.** v7.1.x ships ESP-IDF 6.1.0; only the bundled Arduino core is stuck at 2.0.17. If the Arduino I2C libraries turn out unnecessary (the ToF and INA226 have IDF/ESP component drivers), plain PlatformIO + `framework = espidf` is a supported path the note dismisses.
- **INA226 arithmetic.** 10 mΩ → 0.25 mA per shunt LSB, ±8.19 A full scale; the "2 mA LSB" figure is wrong unless the calibration register is deliberately set coarse. The ALERT trip at 6 A is fine either way.
- **VL53L5CX boot cost.** ~90 kB firmware upload over I2C on every power-up (SparkFun); after a Task-WDT reset the sensor is blind for the upload time, so the fault latch must not depend on it. Price is $32.50 at SparkFun, above the note's $25–30.
- **Rover mass is an assumption.** The brief never states a mass; the note's "3–5 kg" drives motor, torque and driver sizing. Flag it as an input the user must confirm.
- **PCNT PM lock.** The driver already installs a power-management lock while the glitch filter is enabled, so the note's "disable DFS" is redundant; the remaining requirement is to keep light sleep off, which the 100 Hz control loop does anyway.
- **DRV8871 sleep.** Both-low for >1 ms enters sleep; the next drive edge pays a 40–50 µs turn-on. Irrelevant at 100 Hz but it means "coast" is really "coast then sleep", and the note's brake-vs-coast choice on TTL expiry should be brake for that driver too.
- Minor: esp-hal 1.1.0 (2026-04-24) and micro-ROS Iron branch are omitted; neither changes a recommendation. The USB Serial/JTAG console guide no longer lists "no output during boot loops" among its limitations; the 50 ms TX stall, deep-sleep disconnect and light-sleep clock gating are still there.
- Nothing in the note contradicts the brief silently beyond the two items above (Bluetooth removal, assumed mass); every other departure (VL53L4CX for VL53L1X, TTL in the control loop, MCU caps 0.35 m/s / 1.2 rad/s, Gate 2 rewording, BOM line changes) is listed under "Corrections to the brief".
