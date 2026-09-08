# Waveshare WAVE ROVER — hardware facts (researched 2026-09-07)

Sources read in full: Waveshare wiki pages (WAVE_ROVER, General_Driver_for_Robots, UPS_Module_3S, UGV01, UGV02, Arduino/INA219/motor/OLED tutorials), the three product pages, the General Driver schematic PDF (rendered and read), the UPS schematic PDF, the ugv_base_general and ugv_rpi sources, the WAVE_ROVER_FACTORY-25.zip flash package, the legacy WAVE_ROVER_demo.zip, jfjensen/wave-rover (README + photos), TB6612FNG and JST XH datasheets. Web search was mostly blocked (DuckDuckGo/Bing/Mojeek/Startpage bot challenges, Reddit/Amazon/RobotShop interstitials); Brave worked for a few queries. User-report threads exist but their text could not be retrieved.

Confidence: **high** = primary document, schematic net, or source code; **medium** = derived from primary data or a single secondary source; **low** = weak/indirect.

Schematic note used below: in `General_Driver_for_Robots.pdf` the two 40-pin header symbols (P4 inner, P2 outer; wired pin-for-pin) are numbered one column off from the Raspberry Pi convention. Symbol pins 1/3 carry 5V, 4 = IIC_SDA, 6 = IIC_SCL, 5 = GND, 7 = P_TX, 9 = P_RX. That pattern only makes sense as Pi physical pins 2/4 = 5V, 3 = SDA, 5 = SCL, 6 = GND, 8 = TXD, 10 = RXD. Verify with a meter before plugging in a Pi.

## Facts

### 1. UPS module

| fact | value | source URL | confidence |
|---|---|---|---|
| Cell count / arrangement | 3 × 18650 in series (3S1P), cells not included; "7800mAh" marketing figure is 3 cells' capacity summed, pack is still one cell's mAh at 3S | https://www.waveshare.com/wiki/UPS_Module_3S ; https://www.waveshare.com/wiki/WAVE_ROVER | high |
| Cell type | Unprotected: "battery length SHOULD be less than 67mm, some batteries with protection plate in the market are NOT supported"; UGV01/UGV02 FAQ: "Please use batteries without a protective plate"; WAVE ROVER wiki: "high discharge rate is recommended"; UGV02 wiki: ≥2200 mAh, 4C | https://www.waveshare.com/ups-module-3s.htm ; https://www.waveshare.com/wiki/UGV02 ; https://www.waveshare.com/wiki/UGV01 | high |
| Example cells a user fitted | Panasonic NCR18650GA flat-top, no protection circuit | https://github.com/jfjensen/wave-rover | high |
| Protection / balancing ICs | S-8254AA protection IC + AO4407A MOSFETs (over-charge/discharge, over-current, short, reverse); HY2213 per-cell balancing; SY8286 5 V buck; RT9193 3.3 V LDO; INA219 monitor; NDC7002N I2C level translator | https://www.waveshare.com/ups-module-3s.htm ; https://www.waveshare.com/w/upload/5/53/Ups01.pdf | high |
| Charging input | 12.6 V 2 A on a DC5521 (5.5 × 2.1 mm) female barrel pigtail; cannot charge from 5 V (FAQ); rover ships with the 12.6 V 2 A adapter; charge-while-running supported | https://www.waveshare.com/wiki/UPS_Module_3S ; https://www.waveshare.com/w/upload/0/04/WAVE_ROVER_Pack.png | high |
| First-use activation | Protection IC sleeps until BOOT button pressed or charger plugged in (UGV02 text: plug charger to activate) | https://www.waveshare.com/wiki/UPS_Module_3S ; https://www.waveshare.com/wiki/UGV02 | high |
| Own on/off switch | Yes: "Switch Button" = XH2.54 self-locking push-button pigtail switching the outputs (the rover's rear metal button). The driver board additionally has its own slide switch (SW1 → AO4407 P-MOSFET M3 on the input) | https://www.waveshare.com/wiki/UPS_Module_3S ; https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf | high |
| Outputs | (a) XH2.54 2-pin battery-voltage output ("12.6V 2A" per wiki) → driver board; (b) Type-C male pigtail 5 V (5 A regulator); (c) 8-pin header 5V/3V3/GND/SDA/SCL (I2C level 3.3 V default or 5 V). So not all power goes to the driver board | https://www.waveshare.com/wiki/UPS_Module_3S ; https://www.waveshare.com/ups-module-3s.htm | high |
| 5 V Type-C pigtail fitted/routed on the WAVE ROVER? | not found (UPS package includes the cable; rover panel routing not documented) | — | — |
| Reverse-polarity indicator | Per-cell LED lights if a cell is inserted reversed; do not charge in that state | https://www.waveshare.com/wiki/WAVE_ROVER | high |
| Run time | Waveshare: "no certain value" | https://www.waveshare.com/wiki/WAVE_ROVER (FAQ) | high |

### 2. Driver board ↔ UPS connection (E-stop insertion point)

| fact | value | source URL | confidence |
|---|---|---|---|
| Connector | XH2.54 2-pin on both ends (board H1 "XH2.54 power port", silkscreen "DC 9-12.6V"; UPS ships an "XH2.54 dual connector cable ~15cm") | https://www.waveshare.com/wiki/General_Driver_for_Robots ; https://www.waveshare.com/ups-module-3s.htm | high |
| Nominal voltage | 3S: 9.0–12.6 V; board input spec DC 7–13 V | https://www.waveshare.com/wiki/General_Driver_for_Robots | high |
| Wire gauge | not found; JST XH contacts accept AWG 30–22 and are rated 3 A (AWG 22) | https://www.jst-mfg.com/product/pdf/eng/eXH.pdf | medium |
| Motor stall load | 4 × 0.45 A stall = 1.8 A at 12 V (motor spec) | https://www.waveshare.com/wiki/WAVE_ROVER | high |
| Other loads on the same lead | 5 V buck branch designed for 5 A (≈2.5 A at 12 V input worst case) + bus-servo port (5 A limit per FAQ if servos fitted) | schematic "5V-5A" block; https://www.waveshare.com/wiki/General_Driver_for_Robots (FAQ) | medium |
| Switch rating needed | ≥5 A DC recommended (10 A for margin); the XH contact itself is the 3 A weak point; cutting this lead also drops the Pi (unclean shutdown) | derived from above | medium |
| Board-side power path | H1 → AO4407 P-MOSFET (M3, gate via SW1 + 2 kΩ) → DC_IN. A low-current E-stop could open SW1's gate path instead of the battery lead; verify on the unit | https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf | medium |
| Fuse | none on the driver board input (schematic) | schematic | high |

### 3. Raspberry Pi 4 power

| fact | value | source URL | confidence |
|---|---|---|---|
| Path | Driver board 5 V buck → 40-pin header (inner P4 / outer P2, paralleled) → Pi physical pins 2 & 4 (5 V) and GND pins 6, 20, 25, 30, 34, 39 | schematic; https://www.waveshare.com/wiki/General_Driver_for_Robots ("40PIN ... powering the host computer") | high |
| Regulator | MP8759 (alt. MP8756) synchronous buck, 1.5 µH 7×7×5 inductor, FB 75k/10k, 4 × 22 µF out; schematic block titled "5V-5A" / "5V Power for RPi/Jetson nano"; output passes through AO4407 P-MOSFET M1 | https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf | high |
| MP8759 IC rating | "26V, 8A, Low IQ, High-Current, Synchronous Step-Down Converter" | https://www.monolithicpower.com/en/mp8759.html ; https://www.mouser.com/datasheet/2/277/MP8759_r1.1-476926.pdf | medium (titles only; pages blocked scripted fetch) |
| Rated 5 V output current in docs | Wiki gives no number ("DC-DC 5V voltage regulator circuit: Power supply for host computers such as Raspberry Pi or Jetson nano"); 5 A is the schematic label | https://www.waveshare.com/wiki/General_Driver_for_Robots | medium |
| USB back-feed | Both Type-C VBUS lines are diode-ORed (MBR230LSFT1G D1/D2) into the same 5 V rail, so a PC USB cable also feeds the Pi header when the battery switch is off | schematic | high |
| Documented brownout issues | not found in Waveshare docs; Reddit r/robotics threads and a RobotShop forum thread exist but could not be read (bot walls) | https://www.reddit.com/r/robotics/comments/178uy1o/does_anyone_have_feedback_to_share_about_the/ ; https://www.reddit.com/r/robotics/comments/1imxjkd/wave_rover/ ; https://community.robotshop.com/forum/t/waveshare-wave-rover-yeah-i-really-have-no-idea-what-im-doing-challenge-was-accepted-but-not-understood/106641 | low |
| Alternative | UPS's own Type-C 5 V (SY8286, 5 A) is independent of the driver board | https://www.waveshare.com/ups-module-3s.htm | high |

### 4. INA219 and the "v" field

| fact | value | source URL | confidence |
|---|---|---|---|
| Which INA219 the firmware reads | The driver board's own INA219 (U2) at I2C 0x42 on GPIO32/33, shunt R11 0.01 Ω; not the UPS's INA219 | https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/battery_ctrl.h ; schematic | high |
| What it measures | Shunt is between DC_IN (switched battery input) and VIN (input of the 5 V buck only). Bus voltage = battery voltage for the whole robot; current = 5 V-branch current (host computer + ESP32/logic) only. Motors (TB6612 VM1/2/3) and bus-servo ports hang off DC_IN upstream of the shunt | https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf | high |
| "v" field | `jsonInfoHttp["v"] = loadVoltage_V` where `loadVoltage_V = busVoltage_V + shuntVoltage_mV/1000` → battery voltage (V) at DC_IN, in the T:1001 feedback JSON (also OLED line 4). Current is not in the feedback | https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/ugv_advance.h ; .../battery_ctrl.h ; .../oled_ctrl.h | high |
| Wiki wording | "The UPS power module on board contains an INA219" refers to the UPS's separate I2C header chip; tutorial IX says the driver board's INA219 "can detect the power supply voltage and current of the driver board" | https://www.waveshare.com/wiki/WAVE_ROVER ; https://www.waveshare.com/wiki/Tutorial_VII:_INA219_Voltage_And_Current_Monitoring_Demo | high |

### 5. Serial link to the Pi

| fact | value | source URL | confidence |
|---|---|---|---|
| Header pins | Pi pin 8 (GPIO14, TXD0) → net P_TX → ESP32 GPIO3 (U0RXD); Pi pin 10 (GPIO15, RXD0) ← net P_RX ← ESP32 GPIO1 (U0TXD). Header also passes Pi pins 3/5 (GPIO2/3) to the ESP32 I2C bus (GPIO32/33) | schematic (P4/P2 + net ties "U0TXD—P_RX", "U0RXD—P_TX") | high |
| Same UART as USB? | Yes. UART0 (GPIO1/3) is shared with CP2102N U11 through 1 kΩ series resistors R47/R48 (10 kΩ pull-ups R44/R45). A PC on USB and the Pi on the header both hear the ESP32 and can contend on RX | schematic | high |
| Firmware UART | Only `Serial` (UART0) is used for JSON; `Serial.begin(115200)`; servo bus is UART1 on GPIO18/19 | https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/General_Driver.ino ; .../ugv_config.h | high |
| ugv_rpi baud/device | `BaseController('/dev/ttyAMA0', 115200)` on Pi 5, `/dev/serial0` otherwise | https://github.com/waveshareteam/ugv_rpi/blob/main/app.py ; .../base_ctrl.py | high |
| UGV02 FAQ | "only UART and I2C interfaces are exposed on the 40Pin header" | https://www.waveshare.com/wiki/UGV02 | high |

### 6. Auto-reset

| fact | value | source URL | confidence |
|---|---|---|---|
| USB port | Yes: CP2102N DTR (pin 28) and RTS (pin 24) → 12 kΩ → two S8050 transistors → #EN and IO0 ("AUTO PROGRAM CIRCUIT", truth table: DTR=1/RTS=0 → RST=0; DTR=0/RTS=1 → GPIO0=0). Opening the port with DTR/RTS toggling resets or boot-modes the ESP32 | https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf ; https://www.waveshare.com/wiki/General_Driver_for_Robots ("Automatic download circuit") | high |
| Waveshare's mitigation | `serial.Serial(port, 115200, dsrdtr=None); ser.setRTS(False); ser.setDTR(False)` in serial_simple_ctrl.py | https://www.waveshare.com/wiki/WAVE_ROVER | high |
| Pi header | No EN/IO0 on the 40-pin header → no auto-reset from the Pi UART | schematic | high |
| Buttons | S2 KEY_RST/USER (EN, 10 kΩ + 100 nF), S1 KEY_FLASH (IO0, 10 kΩ) | schematic | high |

### 7. Pi configuration (ugv_rpi/setup.sh)

| fact | value | source URL | confidence |
|---|---|---|---|
| cmdline.txt | `sed -i "s/console=ttyAMA0,[0-9]\+ //"` and `s/console=serial0,[0-9]\+ //` | https://github.com/waveshareteam/ugv_rpi/blob/main/setup.sh | high |
| config.txt | `dtparam=uart0=on` (set_config_var), append `dtoverlay=disable-bt`; no explicit `enable_uart=1` | same | high |
| Services / groups | `systemctl disable hciuart.service`, `systemctl disable bluetooth.service`, `usermod -aG dialout $USER` | same | high |
| Header power | 5 V (pins 2, 4) and GND pass through; Pi 3V3 (pins 1/17) is not driven by the board | schematic | high |

### 8. Chassis and motion

| fact | value | source URL | confidence |
|---|---|---|---|
| Motor model | "GF12-N20 Motor 12V200rpm Gearbox", rated 12 V, rated current 0.055 A, stall 0.45 A, rated torque 0.09 kg·cm, stall torque 0.7 kg·cm, rated 1.5 W, motor 34 × 12 mm, shaft 4 × 10 mm | https://www.waveshare.com/wiki/WAVE_ROVER (Motor Specifications) | high |
| No-load speed | Wiki lists "66±10%RPM" in the same block, product page lists "200RPM × 4"; the two conflict | https://www.waveshare.com/wiki/WAVE_ROVER ; https://www.waveshare.com/wave-rover.htm | medium |
| Gear ratio | not found | — | — |
| Encoders | None on WAVE ROVER ("the motor used is without encoders"; CMD_ROS_CTRL/PID "only for UGV01"). Firmware mainType 1 runs open-loop PWM (`usePIDCompute=false`); L/R in T:1001 echo the commanded PWM, not measured speed | https://www.waveshare.com/wiki/WAVE_ROVER ; https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/movtion_module.h | high |
| Max speed | 1.25 m/s (both pages). 200 RPM × π × 0.08 m = 0.84 m/s, so speed and RPM are not mutually consistent | https://www.waveshare.com/wave-rover.htm | medium |
| Mass | 860 g without cells (cells not included); +~140 g for 3 × 18650 (estimate) | https://www.waveshare.com/wave-rover.htm | high / medium |
| Payload / climb / obstacle | 0.8 kg / 22° / 40 mm; ground clearance 33.7 mm | https://www.waveshare.com/wave-rover.htm | high |
| Wheel | 80 mm tire diameter, 42.5 mm tire width, nylon hub, rubber tire | https://www.waveshare.com/wave-rover.htm | high |
| Outline | Spec table 194 × 168 × 100 mm; drawing shows 194 mm long, 183 mm over tires, 100 mm high, 97 mm wheelbase, 81.6 mm between inner tire faces | https://www.waveshare.com/img/devkit/accBoard/WAVE-ROVER/WAVE-ROVER-details-size-1.jpg | high |
| Track width (centre-to-centre) | ≈124 mm derived (81.6 + 42.5); firmware `TRACK_WIDTH = 0.125` for mainType 1 | drawing above; https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/movtion_module.h | medium |
| Body | 2 mm 5052 aluminium | https://www.waveshare.com/wave-rover.htm | high |

### 9. Motor driver

| fact | value | source URL | confidence |
|---|---|---|---|
| Chip | One TB6612FNG (TB1); VM1/VM2/VM3 fed from DC_IN (switched battery, upstream of the INA219 shunt) | schematic; https://www.waveshare.com/wiki/General_Driver_for_Robots | high |
| Four motors on two channels | Channel A outputs (nets MA1/MA2) go to two PH2.0 2-pin connectors MOTOR-A1 and MOTOR-A2 in parallel (one side's front+rear motors); channel B (MB1/MB2) to MOTOR-B1 and MOTOR-B2. The 6-pin encoder connectors H3/H4 carry the same MA/MB nets plus 3V3, GND and encoder inputs | schematic; board photo items 14–17 | high |
| ESP32 pins | PWMA 25, AIN1 21, AIN2 17; PWMB 26, BIN1 22, BIN2 23; 100 kHz, 8-bit PWM | https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/ugv_config.h ; https://www.waveshare.com/wiki/Tutorial_II:_Motor_Without_Encoder_Control_Demo | high |
| TB6612FNG limits | Iout 1.2 A average / 3.2 A peak per channel; VM 15 V abs max, 4.5–13.5 V operating (Pololu quotes 1 A continuous / 3 A peak) | https://www.sparkfun.com/datasheets/Robotics/TB6612FNG.pdf ; https://www.pololu.com/product/713 | high |
| Margin | Two N20 stalled per channel = 0.9 A < 1.2 A average rating | derived | medium |

### 10. Firmware

| fact | value | source URL | confidence |
|---|---|---|---|
| What the factory package flashes | `WAVE_ROVER_FACTORY-25.zip` (server Last-Modified 2025-07-18): flash_download_tool_3.9.5 in Factory mode; `multi_download.conf` writes `General_Driver.ino.bootloader.bin` @0x1000, `General_Driver.ino.partitions.bin` @0x8000, `boot_app0.bin` @0xE000, `General_Driver.ino.bin` @0x10000 (built 2024-07-19; strings "version: 0.95", "WAVE ROVER"; DIO, 4 MB). The legacy `WAVE_ROVER_V0.9.ino.bin` (2024-03-22, "version: 0.90") is in bin/ but not selected | https://files.waveshare.com/wiki/common/WAVE_ROVER_FACTORY-25.zip | high |
| So which firmware ships | The General_Driver build = ugv_base_general (mainType 1 = WAVE ROVER, `screenLine_0 = "WAVE ROVER"`). Wiki still says OLED "Version: 0.9" means updated firmware | https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/General_Driver.ino ; https://www.waveshare.com/wiki/WAVE_ROVER | high (package) / medium (what a 2026 unit boots) |
| GitHub activity | ugv_base_general: code commits to 2024-07-19, web_page.h 2025-11-28; ugv_rpi last commit 2024-10-14 (pushed 2026-09-02) | https://github.com/waveshareteam/ugv_base_general/commits/main ; https://github.com/waveshareteam/ugv_rpi | high |
| "WAVE_ROVER_demo" source | `WAVE_ROVER_demo.zip` = folder WAVE_ROVER_V0.9 (same code lineage as ugv_base_general, version 0.90, includes build/ bins); Arduino tutorial links `WAVE_ROVER.zip` (WAVE_ROVER_v0.9.ino) | https://files.waveshare.com/upload/e/e6/WAVE_ROVER_demo.zip ; https://files.waveshare.com/upload/a/ae/WAVE_ROVER.zip ; https://www.waveshare.com/wiki/How_To_Install_Arduino_IDE | high |
| Arduino IDE settings documented | Board "ESP32 Dev Module"; ESP32 Arduino core 2.0.11 ("matches the version required" for WAVE ROVER/UGV01/UGV02); libraries `General-Libraries.zip` or ArduinoJson, LittleFS, Adafruit_SSD1306, INA219_WE, ESP32Encoder, PID_v2, SimpleKalmanFilter, Adafruit_ICM20X, Adafruit_ICM20948, Adafruit_Sensor + SCServo folder | https://www.waveshare.com/wiki/How_To_Install_Arduino_IDE ; https://github.com/waveshareteam/ugv_base_general | high |
| Partition scheme | Not documented. Factory bins use the default 4 MB table (app0 0x10000/0x140000, app1 0x150000/0x140000, spiffs 0x290000/0x160000 = "Default 4MB with spiffs 1.2MB APP/1.5MB SPIFFS"); the GitHub build folder uses Huge APP (app0 0x300000, spiffs 0x310000/0xE0000, coredump 0x3F0000) | factory zip partitions.bin; https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/build/esp32.esp32.esp32/General_Driver.ino.partitions.bin | high |
| Upload speed | Not documented for Arduino IDE; flash tool "ESP32 can use up to 921600" | https://www.waveshare.com/wiki/WAVE_ROVER | high |
| Flash / module | ESP32-WROOM-32UE (IPEX antenna), 4 MB, DIO | schematic; bin header | high |
| Robot type command | `{"T":900,"main":1,"module":0}` (main 1 = WAVE ROVER in this repo's ugv_config.h; README text copied from ugv_base_ros says 1 = RaspRover) | https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/ugv_config.h | high |

### 11. I2C expansion

| fact | value | source URL | confidence |
|---|---|---|---|
| Header | "IIC" 4-pin PH2.0-style white connector P1: 3V3, SDA (GPIO32), SCL (GPIO33), GND; 3.3 V logic | schematic; https://www.waveshare.com/wiki/General_Driver_for_Robots (item 4) | high |
| Pull-ups | R39/R40 4.7 kΩ to 3.3 V on the ESP32 bus; IMU segment sits behind an LSF0204PWR level shifter at 1.8 V with its own 4.7 kΩ (R37/R38) | schematic | high |
| Devices already on the bus | SSD1306 OLED 0x3C (plugged into the IIC header on the rover), INA219 0x42, QMI8658C 0x6B, AK09918C 0x0C, BMP280 0x77 (schematic "Addr 0xEE" 8-bit; unused by firmware; check if populated) | https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/oled_ctrl.h ; .../QMI8658.h ; .../AK09918.h ; schematic | high |
| VL53L1X | 0x29 default: no conflict; second sensor needs XSHUT + address change | derived | high |
| Second master | The same SDA/SCL go to Pi pins 3/5 (GPIO2/3); a Pi with I2C1 enabled shares the bus with the ESP32 and adds its 1.8 kΩ pull-ups | schematic | high |
| Free connector | The IIC header is occupied by the OLED; other 3V3/GND sources: P3 7-pin header, H3/H4 6-pin motor connectors | schematic | medium |

### 12. Bumper switch inputs

| fact | value | source URL | confidence |
|---|---|---|---|
| 40-pin header | No ESP32 GPIO, only UART and I2C (UGV02 FAQ) | https://www.waveshare.com/wiki/UGV02 | high |
| IO4 / IO5 on this board | 3.3 V logic pins through 10 Ω: IO4 on H2 3-pin PWM-servo header (IO4, 5V, GND); IO5 on P3 7-pin header (IO5, 3V3, GND, IO16, IO27, CP_RX, U0RX). They are not 12 V MOSFET switches on the General Driver (that is the ROS Driver board); firmware drives them as LED PWM (`led_pin_init`) | schematic; https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/ugv_led_ctrl.h | high |
| Unused encoder inputs | IO34/IO35 (A_C1/A_C2) and IO16/IO27 (B_C1/B_C2) on H3/H4 6-pin connectors with 3V3 and GND; 10 Ω series, clamp zeners D4–D7 marked NC. WAVE ROVER has no encoders, but `initEncoders()` attaches them to ESP32Encoder → firmware change needed; IO34/35 are input-only with no internal pull-up | schematic; https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/movtion_module.h | high |
| Easiest option | A spare Pi GPIO on the outer 40-pin header (all Pi pins pass through) | schematic | medium |

### 13. Mounting / expansion

| fact | value | source URL | confidence |
|---|---|---|---|
| Plate | Two-part mounting plate included; hole patterns 58 × 49 mm (Pi 4B / driver board), 58 × 86 mm (Jetson Nano), 34 × 34 mm (pan-tilt), 46.8 mm slot pair (LiDAR); holes Ø2.27 / Ø2.75 mm; standoff heights 12.5 / 13.5 mm | https://www.waveshare.com/img/devkit/accBoard/WAVE-ROVER/WAVE-ROVER-details-size-2.jpg ; ...details-27.jpg | high |
| Stock Pi position | Pi 4B on the plate at the rear top beside the power button and charge jack (product photo "Connect to Raspberry Pi 4B"); Pi Zero and Jetson Orin Nano photos also shown; connection to the driver board by cable to the 40-pin header (jfjensen: data cable into the header's low-numbered pins) | https://www.waveshare.com/img/devkit/accBoard/WAVE-ROVER/WAVE-ROVER-details-25-2.jpg ; https://github.com/jfjensen/wave-rover | medium |
| Camera options | Waveshare 2-axis pan-tilt (ST3215 bus servos, module type 2) on the 34 mm pattern; recommended IMX335 5MP USB Camera (B); LD19/STL-27L LiDAR via the board's LIDAR Type-C bridge | https://www.waveshare.com/wiki/WAVE_ROVER ; https://www.waveshare.com/wave-rover.htm | high |
| Upper surface area | 17,551 mm² | https://www.waveshare.com/wave-rover.htm | high |
| USB mic / speaker fit | not found | — | — |

### 14. Price (US, 2026-09-07)

| fact | value | source URL | confidence |
|---|---|---|---|
| WAVE ROVER | $89.99 at waveshare.com; only option is the charger plug (EU/UK/US); no Pi-included variant is offered (Pi kits are RaspRover/UGV Rover) | https://www.waveshare.com/wave-rover.htm | high |
| WAVE ROVER on Amazon | Listing B0CF55LM6Q; price page not retrievable (bot wall); a May-2025 review says "$99 ... on Amazon" | https://www.amazon.com/dp/B0CF55LM6Q ; https://www.youtube.com/watch?v=2faRUwIWVSA | medium |
| General Driver for Robots | $27.99 (volume $27.59 / $27.39 / $27.31) | https://www.waveshare.com/general-driver-for-robots.htm | high |
| UPS Module 3S | $21.99, charger optional | https://www.waveshare.com/ups-module-3s.htm | high |

## Unknowns to measure on the unit

1. Wire gauge and length of the XH2.54 battery lead between UPS and driver board (read insulation print or gauge the conductor).
2. Whether the UPS's Type-C 5 V pigtail is fitted and where it is routed (open the belly plate; count the XH2.54 pigtails on the UPS's 5-socket row).
3. 5 V at Pi header pins 2/4 under full load (Pi 4 + camera + USB mic, motors stalled); check `vcgencmd get_throttled` and scope the droop at motor start.
4. Header pin mapping with a meter before connecting a Pi: 5 V on physical 2/4, TX/RX on 8/10, SDA/SCL on 3/5 (schematic symbol numbering is offset one column).
5. Battery-lead current: motors stalled (expect ≈1.8 A at 12 V) plus the 5 V branch (`current_mA` from the INA219) to size the E-stop switch; check whether the UPS's XH2.54 output is really limited to "2 A".
6. Contents of the "Accessories pack" (is a 40-pin or 4-wire header cable for the Pi included).
7. `i2cdetect -y 1` from the Pi: is BMP280 (0x77) populated, and does the Pi's I2C1 collide with the ESP32 as bus master.
8. Motor gear ratio (gearbox marking or count turns), true no-load RPM and top speed (200 RPM vs 66 RPM vs 1.25 m/s do not agree).
9. Mass with cells and centre-to-centre track width (expect ≈124 mm).
10. Firmware on the unit: OLED line 2 "version: 0.90" vs "0.95"; `esptool read_flash 0x8000 0xC00` for the partition table.
11. Whether P3 (7-pin: IO5/3V3/GND/IO16/IO27/CP_RX/U0RX) and H2 (IO4/5V/GND) are populated on this board revision (Rev1.2 photo shows headers at the right edge and bottom).
12. Pi behaviour when a PC USB cable is attached for flashing with the battery switch off (USB VBUS diode-ORs into the Pi's 5 V rail).
13. Free plate area behind the Pi for a USB microphone board and speaker; cable exit through the top cover.
14. Whether the OLED's cable occupies the only IIC header and whether a Y-cable fits inside the cover.

## Discrepancies worth remembering

- Motor speed: wiki "66±10% RPM" vs "200rpm" model name vs 1.25 m/s (needs ~300 RPM at 80 mm).
- Outline width: spec 168 mm vs drawing 183 mm over tires.
- "IO4/IO5 12V switch": only true for the ROS Driver board; on the General Driver they are 3.3 V PWM pins.
- ugv_base_general README says main type 1 = RaspRover; its ugv_config.h says 1 = WAVE ROVER; the OLED line prints "RaspRover" for mainType 1 in `mm_settings()` while `setup()` prints "WAVE ROVER" on line 0.
- Wiki firmware-check text ("Version: 0.9") predates the v0.95 General_Driver build that the 2025 factory package flashes.
