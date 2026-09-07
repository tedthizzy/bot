# BOM part picks and US prices (checked 2026-09-07)

Method: the session's WebSearch budget was already exhausted, so every price is from a direct WebFetch of the vendor page on 2026-09-07 ([VENDOR]). Amazon, DigiKey, Mouser, Waveshare, Cytron, RobotShop and AutomationDirect block fetches; parts sold only there are [INFERRED] and flagged. The only [MEASURED] figure is pidramble's Pi 4 current table.

## Summary

- The brief's $200–300 total holds only at the top. A vendor-priced Pololu-Romi build (no IMU, no tools) sums to ≈ $295 before shipping/tax, ≈ $300–350 delivered. Clone/AliExpress parts can reach ~$200 with 3–5 week lead times.
- The largest error is electrical: a 3S pack (12.6 V full) matches none of the sub-$100 encoded drive options. Romi motors are 4.5 V nominal, TT encoder motors 6 V, the Romi driver board's DRV8838 stops at 11 V, DRV8833 at 10.8 V. Use 2S for a Romi/TT build, or keep 3S and buy 12 V 25D gearmotors (+$100–130).
- Under-priced lines: ESP32-S3 DevKitC ($15–20, not $10); 5 V/5 A buck (Pololu's is $32.95 and rationed; a $23.95 5 V/3.2 A part is right); "2× VL53L1X + bumper + e-stop $15–20" is really $45–60.
- Delete the small 5 V buck: power the ESP32-S3 from a Pi USB port, which is also the serial link.
- Missing: INA260 telemetry $9.95, fuse/holder, XT60, wire, Pi standoffs (~$34 together), optional IMU $20–30, tools $80–130.
- Ready-made kits (Hiwonder TurboPi $99.99, Yahboom Raspbot V2 $134.99, Elegoo V4 $75.99) have proprietary controllers and no advertised encoders. Only the Pololu Romi Robot Kit for FIRST ($159.95) maps onto the design, with an ATmega32U4 in place of the ESP32-S3.

## State of the art (2026)

**MCU.** ESP32-S3-DevKitC-1 is still the default S3 board. Espressif lists N8R8 at ~$15 and N32R16V (WROOM-2) at ~$17 [VENDOR]; Adafruit has both at $19.95, 62 N8R8 in stock [VENDOR]. The v1.1 user guide lists only N8R8, N32R16V and 1U-N8R8; N16R8 is now an AliExpress/Amazon clone SKU at $12–18 [INFERRED]. All R8 and WROOM-2 variants lose GPIO35/36/37 to the octal flash/PSRAM bus [VENDOR]. Seeed XIAO ESP32S3 is $7.49 with 8 MB flash / 8 MB PSRAM but only 11 GPIO [VENDOR]; this rover needs ~16 (4 encoder, 4 motor, 2 I2C, 2 UART, 2 bumper, 1 e-stop sense, 1 spare). Adafruit QT Py ESP32-S3 ($12.50, no PSRAM, 13 GPIO) and SparkFun Thing Plus ESP32-S3 ($24.95, backorder) are also out [VENDOR]. Adafruit's "only ESP-IDF, no Arduino" note on the DevKitC is a disclaimer; arduino-esp32 3.x targets it [INFERRED].

**Motor drivers.** Pololu TB6612FNG $4.95 (1 A cont / 3 A peak per channel, 4.5–13.5 V), DRV8833 $10.95 (1.2 A, 2.7–10.8 V), DRV8876 single $8.15 (1.3 A, 4.5–37 V, current sense, 2 A active limit), TB9051FTG single $12.95 (2.6 A, 4.5–28 V, 500 mV/A sense pin) [VENDOR]. Only TB6612FNG, DRV8876 and TB9051FTG accept a full 3S. Per-motor current sense (DRV8876/TB9051FTG) is what the brief's stall gate wants; with TB6612/DRV8833 the INA260 on the rail does it.

**Drive and chassis.** Romi Chassis Kit $39.95: two 120:1 HP mini plastic gearmotors, 70×8 mm wheels, ball caster, 6×AA contacts [VENDOR]. Romi Encoder Pair $9.95, 12 CPR motor-side ≈ 1440 counts per wheel rev [VENDOR]; pre-assembled motor+encoder $19.95 each [VENDOR]. Romi motor: 4.5 V nominal, 150 rpm / 130 mA no-load, 1.25 A stall, 1.8 kg·cm [VENDOR]; 150 rpm on 70 mm ≈ 0.55 m/s, above the 0.3 m/s cap. Cheaper: DFRobot FIT0450 TT motor with encoder, 6 V, 120:1, 960 counts/rev, $7.40 [VENDOR]; Adafruit #4416 7 V 20:1 encoder motor $13.50 [VENDOR]. Up-market: Pololu 25D HP 12 V 47:1 with 48 CPR encoder, 220 rpm, 5.0 A stall, 2249 counts/rev, $56.95 each; bracket pair $10.95; 90×10 mm wheels $9.49 [VENDOR].

**Battery.** Talentcell YB1203000-USB: 11.1 V 3S, 3000 mAh (33 Wh), 12 V out 3 A max (12.6–9 V), 5 V USB 2 A, DC5521, 12.6 V/0.5 A charger included, 190 g [VENDOR spec; price hidden, Amazon blocked, historically $25–30 INFERRED]. The 3 A cap rules out 25D motors (5 A stall each). DIY: Samsung INR18650-30Q 3000 mAh / 15 A at $6.99/cell [VENDOR], plus 2S/3S BMS-holder ($5–8) and charger ($10–15) [INFERRED].

**Pi 4 5 V rail.** Spec: 5 V 3 A minimum, or 2.5 A if USB draw < 500 mA [VENDOR]. pidramble measured a bare Pi 4B at 540 mA idle, 1280 mA full CPU, nothing on USB [MEASURED, Satechi USB-C meter]. Camera + USB mic + I2S amp add ~0.4–0.8 A [INFERRED]. Pololu D24V50F5 5 A is $32.95, "rationed", in the "older or supply-chain-constrained" list [VENDOR]. Current parts: D36V28F5 5 V/3.2 A $23.95 (5.3–50 V in) and D24V22F5 5 V/2.5 A $18.95 (new) [VENDOR].

**Sensors.** VL53L1X: Adafruit $14.95 (STEMMA QT, 4 m, 50 Hz, 27°), Pololu $22.95, SparkFun $29.95 [VENDOR]. VL53L4CD: Pololu $13.95, Adafruit $14.95 (1.2 m, 100 Hz, 18°) [VENDOR]. Bumper: Pololu 18.5 mm bump-lever snap switch, SPDT 5 A, $2.37 [VENDOR]. Telemetry: Adafruit INA260 $9.95 (36 V, 15 A integrated shunt), INA219 $9.95 (26 V, 3.2 A), INA228 $14.95 (85 V, 10 A) [VENDOR]; no US vendor stocks an INA226 breakout, only CJMCU clones at $6–9 [INFERRED]. IMU: BNO085 $29.50 (onboard fusion, UART-RVC), ICM-20948 $19.95 (raw) [VENDOR].

**Camera.** Camera Module 3 MSRP "from $25", in production until at least January 2030 [VENDOR]. US street: standard 75° $29.25, NoIR $27.50, Wide 120° $38.50 (PiShop and Adafruit agree) [VENDOR].

**Audio.** Adafruit Mini USB Microphone $5.95 [VENDOR]. Seeed ReSpeaker Lite (XMOS XU316, 2 mics, USB or I2S, on-chip AEC/NS) $26.99; ReSpeaker 2-Mics Pi HAT v2 $13.99; 4-Mic Array v2 $64 [VENDOR]. Output: MAX98357A I2S amp $5.95 (3.2 W @ 4 Ω) + 40 mm 4 Ω 5 W speaker $4.95; Adafruit's 3-inch speaker is out of stock; Mini USB Stereo Speaker $12.50 [VENDOR].

**Connectors, switches, tools.** Pololu XT60 pair (yellow) $2.95, black discontinued [VENDOR]. Pololu Mini Pushbutton Power Switch LV (2.2–20 V, 6 A) $4.95; Romi Power Distribution Board (reverse protection, latching button, busses) $14.95 [VENDOR]. Crimpers at Adafruit: Engineer PA-09 $49.95, PA-24 $29.95, ratcheting 18–28 AWG $34.95 [VENDOR]; IWISS SN-2549 ~$25 on Amazon [INFERRED]. SparkFun Big Dome Pushbutton $18.95 is a momentary arcade button, not an e-stop [VENDOR]; a 22 mm twist-release NC mushroom is $8–15 on Amazon/AutomationDirect [INFERRED].

**Kits.** Hiwonder TurboPi Standard without Pi $99.99 (mecanum, 2×18650, Hiwonder controller, no encoders stated); Yahboom Raspbot V2 Standard without Pi $134.99 (mecanum, Pi 5 expansion board); Elegoo Smart Robot Car V4.0 $75.99 (Arduino, no encoders); Pololu Romi Robot Kit for FIRST $159.95 (chassis with motors+encoders assembled, Romi 32U4 board, second caster, Pi standoffs; Pi not included) [all VENDOR]. Waveshare WAVE ROVER/UGV and General Driver for Robots, and Cytron MDD3A: 403, not priced.

## Recommendation for this build

Choose one coherent electrical system. Do not put a 3S pack on 4.5–6 V motors.

**Path A (recommended): Romi chassis, 2S Li-ion, ESP32-S3-DevKitC-1.** Motors run at the voltage Pololu designed for (6×AA ≈ 7.2–8.4 V), every board stays inside its rating, and it is the cheapest rigid chassis with real quadrature encoders.

| Line | Part | Price | Where |
|---|---|---|---|
| MCU | ESP32-S3-DevKitC-1-N8R8 | $19.95 (Espressif MSRP ~$15) | Adafruit #5336 |
| Chassis, motors, wheels, caster | Romi Chassis Kit | $39.95 | Pololu #3500 |
| Encoders | Romi Encoder Pair, 12 CPR | $9.95 | Pololu #3542 |
| Motor driver + 5 V/2.5 A rail + switch + reverse protection | Motor Driver and Power Distribution Board for Romi | $34.95 | Pololu #3543 |
| Pi deck | Romi Expansion Plate + standoffs | $6.49 + ~$8 | Pololu #3560, Amazon |
| Pi 5 V | D36V28F5 5 V/3.2 A | $23.95 | Pololu #3782 |
| ESP32 5 V | none; USB from Pi | $0 | — |
| Battery | 2× Samsung 30Q + 2S holder/BMS + 8.4 V charger | $13.98 + ~$20 [INFERRED] | 18650batterystore, Amazon |
| ToF ×2 | Adafruit VL53L1X (or Pololu VL53L4CD $13.95 ea.) | $29.90 | Adafruit #3967 |
| Bumpers ×2 | Snap-action bump-lever switch | $4.74 | Pololu #1405 |
| E-stop | 22 mm twist-release NC mushroom | ~$10 [INFERRED] | Amazon |
| Camera | Camera Module 3 Wide (standard $29.25) | $38.50 | PiShop / Adafruit |
| Mic | Mini USB Microphone | $5.95 | Adafruit #3367 |
| Speaker | MAX98357A + 40 mm 4 Ω 5 W | $10.90 | Adafruit #3006 + #3968 |
| Telemetry | INA260 | $9.95 | Adafruit #4226 |
| Connectors | XT60 pair; JST/pre-crimped wire; inline blade fuse holder + 5 A fuse | $2.95 + ~$13 [INFERRED] | Pololu #2175, Amazon |
| **Subtotal** | | **≈ $295** | |

Notes:
- The Romi board's DRV8838 (1.7 A continuous per channel) covers a 1.25 A-stall motor. Its 5 V/2.5 A rail can feed the ESP32 side and ToF sensors instead of Pi USB. To save $15, use TB6612FNG ($4.95) + Romi Power Distribution Board ($14.95) and lose the regulated rail.
- The Pi gets its own D36V28F5 from the battery bus; the Romi rail's 2.5 A will fail the brief's `get_throttled` gate once camera and USB audio are attached.
- Wide camera for `describe_scene` and `find` on a floor robot; take the 75° standard if the VLM needs fine detail.
- Start with the $5.95 mic. Upgrade to ReSpeaker Lite ($26.99, on-chip AEC) only if barge-in over the rover's own speaker fails. The $64 array is for far-field rooms, not this.
- I2S amp over USB speaker: same price, no USB audio device-ordering surprises, GPIO 18/19/21 with no UART conflict.
- INA260 over INA226: integrated 15 A shunt, 36 V bus, US stock. Put it on the battery bus on the ESP32's I2C so telemetry survives Pi loss.
- IMU later: BNO085 in UART-RVC mode gives heading with no fusion code; ICM-20948 saves $10 and costs you a filter.
- Controller alternative: Romi Robot Kit for FIRST ($159.95) covers MCU, chassis, encoders, driver and Pi standoffs in one order, $48 above Path A's equivalent lines, with an ATmega32U4 (2.5 KB RAM, Pi header, IMU, 5 V rail). PID + TTL + two ToF at 50 Hz fits; keep the ESP32-S3 for headroom.

**Path B (only if 3S is fixed): 12 V drive.** 2× 25D HP 12 V 47:1 encoder ($113.90), bracket pair ($10.95), 90×10 wheels ($9.49) + 4 mm hubs (~$8), 2× TB9051FTG ($25.90), plate/caster (~$20): drive ≈ $190 [VENDOR + INFERRED]. Needs a DIY 3S pack with a 10–15 A BMS; the Talentcell trips on stall. Whole build ≈ $400. Gains torque, 2249 counts/rev and per-motor current sense; none required at 0.3 m/s indoors.

## Numbers

| quantity | value | hardware/context | tag | source URL |
|---|---|---|---|---|
| DevKitC-1-N8R8 price | $19.95, 62 in stock | Adafruit | VENDOR | https://www.adafruit.com/product/5336 |
| DevKitC-1 MSRP | N8R8 ~$15; N32R16V ~$17 | Espressif | VENDOR | https://www.espressif.com/en/products/devkits/esp32-s3-devkitc-1 |
| GPIO lost on R8/WROOM-2 | GPIO35–37 | DevKitC-1 user guide v1.1 | VENDOR | https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html |
| XIAO ESP32S3 | $7.49; 11 GPIO; 8 MB/8 MB | Seeed | VENDOR | https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html |
| Romi Chassis Kit | $39.95 | 2 motors, wheels, caster | VENDOR | https://www.pololu.com/product/3500 |
| Romi Encoder Pair | $9.95; 12 CPR ≈ 1440/wheel rev | Pololu | VENDOR | https://www.pololu.com/product/3542 |
| Romi motor | 4.5 V; 150 rpm/130 mA; 1.25 A stall | Pololu #1520 | VENDOR | https://www.pololu.com/product/1520 |
| Romi Motor Driver + PDB | $34.95; DRV8838 ×2; 5 V 2.5 A | Pololu | VENDOR | https://www.pololu.com/product/3543 |
| Romi PDB / plate / 32U4 / FIRST kit | $14.95 / $6.49 / $74.95 / $159.95 | Pololu #3541, #3560, #3544, #4022 | VENDOR | https://www.pololu.com/product/4022 |
| TB6612FNG / DRV8833 | $4.95, 1 A, 4.5–13.5 V / $10.95, 1.2 A, 2.7–10.8 V | Pololu #713, #2130 | VENDOR | https://www.pololu.com/product/713 |
| DRV8876 / TB9051FTG | $8.15, 1.3 A, 4.5–37 V / $12.95, 2.6 A, 4.5–28 V | Pololu #4036, #2997 | VENDOR | https://www.pololu.com/product/2997 |
| 25D HP 12 V 47:1 encoder | $56.95; 220 rpm; 5.0 A stall; 2249 counts/rev | Pololu #4845 | VENDOR | https://www.pololu.com/product/4845 |
| 25D bracket / 90×10 wheels | $10.95 / $9.49 | Pololu #2676, #1435 | VENDOR | https://www.pololu.com/product/2676 |
| DFRobot FIT0450 | $7.40; 6 V; 960 counts/rev | TT motor w/ encoder | VENDOR | https://www.dfrobot.com/product-1457.html |
| D24V50F5 5 V/5 A | $32.95; rationed | Pololu | VENDOR | https://www.pololu.com/product/2851 |
| D36V28F5 5 V/3.2 A | $23.95; 5.3–50 V in | Pololu | VENDOR | https://www.pololu.com/product/3782 |
| D24V22F5 5 V/2.5 A | $18.95; 5.3–36 V in | Pololu | VENDOR | https://www.pololu.com/product/2858 |
| Pi 4 PSU spec | 5 V 3 A min; 2.5 A if USB < 500 mA | raspberrypi.com | VENDOR | https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/ |
| Pi 4B current | 540 mA idle; 1280 mA full CPU; no USB | Satechi USB-C meter | MEASURED | https://www.pidramble.com/wiki/benchmarks/power-consumption |
| Pi 4 + cam + USB mic + amp | ~1.7–2.2 A peak | estimate | INFERRED | — |
| Talentcell YB1203000-USB | 3S 3000 mAh; 12 V 3 A max; charger incl. | spec page, no price | VENDOR | https://talentcell.com/lithium-ion-battery/12v/yb1203000-usb.html |
| Talentcell street price | $25–30 | Amazon (blocked) | INFERRED | — |
| Samsung 30Q | $6.99; 3000 mAh; 15 A | 18650batterystore | VENDOR | https://www.18650batterystore.com/products/samsung-30q |
| BMS + holder + charger | $15–25 | Amazon/AliExpress | INFERRED | — |
| VL53L1X | Adafruit $14.95; Pololu $22.95; SparkFun $29.95 | 4 m, 50 Hz | VENDOR | https://www.adafruit.com/product/3967 |
| VL53L4CD | Pololu $13.95; Adafruit $14.95 | 1.2 m, 100 Hz | VENDOR | https://www.pololu.com/product/3692 |
| Bumper switch | $2.37 | Pololu #1405 | VENDOR | https://www.pololu.com/product/1405 |
| 22 mm mushroom e-stop | $8–15 | Amazon (blocked) | INFERRED | — |
| Camera Module 3 | MSRP from $25; production ≥ Jan 2030 | raspberrypi.com | VENDOR | https://www.raspberrypi.com/products/camera-module-3/ |
| Camera Module 3 street | $29.25 std; $38.50 Wide | PiShop, Adafruit | VENDOR | https://www.pishop.us/product/raspberry-pi-camera-module-3/ |
| Mini USB mic | $5.95 | Adafruit | VENDOR | https://www.adafruit.com/product/3367 |
| ReSpeaker Lite / 2-Mics HAT v2 / 4-Mic v2 | $26.99 / $13.99 / $64.00 | Seeed | VENDOR | https://www.seeedstudio.com/ReSpeaker-Lite-p-5928.html |
| MAX98357A + 40 mm speaker | $5.95 + $4.95 | Adafruit #3006, #3968 | VENDOR | https://www.adafruit.com/product/3006 |
| USB stereo speaker | $12.50 | Adafruit #3369 | VENDOR | https://www.adafruit.com/product/3369 |
| INA260 / INA219 / INA228 | $9.95 / $9.95 / $14.95 | Adafruit #4226, #904, #5832 | VENDOR | https://www.adafruit.com/product/4226 |
| INA226 clone | $6–9 | Amazon/AliExpress | INFERRED | — |
| BNO085 / ICM-20948 | $29.50 / $19.95 | Adafruit #4754, #4554 | VENDOR | https://www.adafruit.com/product/4754 |
| XT60 pair / pushbutton power switch | $2.95 / $4.95 | Pololu #2175, #2808 | VENDOR | https://www.pololu.com/product/2175 |
| Crimpers | PA-09 $49.95; PA-24 $29.95; ratcheting $34.95 | Adafruit | VENDOR | https://www.adafruit.com/?q=crimp |
| TurboPi / Raspbot V2 / Elegoo V4 (no Pi) | $99.99 / $134.99 / $75.99 | Shopify JSON | VENDOR | https://www.hiwonder.com/products/turbopi.json |
| Path A subtotal | ≈ $295 before shipping/tax | sum above | INFERRED | — |

## Corrections to the brief

1. **"ESP32-S3 $10"**: a genuine DevKitC-1 is $19.95 (Adafruit) or ~$15 (Espressif MSRP); $10–12 buys a clone N16R8 [INFERRED]. The $7.49 XIAO has 11 GPIO, too few. Budget $15–20.
2. **"3S Li-ion + BMS + charger $40–55"**: price fine (Talentcell ~$25–30 with charger; DIY ~$35). Voltage is wrong for the motor class the budget implies: Romi 4.5 V, TT 6 V, Romi driver board 11 V max, DRV8833 10.8 V max. Use 2S for Romi/TT; 3S only with 12 V motors and TB6612FNG/DRV8876/TB9051FTG. Talentcell's 12 V output is capped at 3 A; a stalled 25D pair (10 A) trips it and browns out the Pi.
3. **"Motor driver $8–15"**: holds ($4.95 TB6612FNG, $10.95 DRV8833, $8.15 DRV8876). Per-motor current sense costs 2× $8.15 or 2× $12.95.
4. **"2× encoded gearmotors + chassis + caster $50–100"**: holds for Romi ($56.39); a 12 V metal-gearmotor drive is $150–190.
5. **"5 V/5 A buck $20"**: Pololu's 5 A part is $32.95 and rationed. 5 A is unnecessary (spec 3 A; measured bare load 1.28 A). Use D36V28F5 5 V/3.2 A, $23.95.
6. **"Small 5 V buck $5"**: delete. DevKitC-1 runs from a Pi USB port (the serial cable) or the Romi board's 5 V rail. ToF carriers accept 2.6–5.5 V.
7. **"2× VL53L1X + bumper + e-stop $15–20"**: two VL53L1X are $29.90 (Adafruit) or $45.90 (Pololu); bumpers $4.74; e-stop ~$10. Budget $45–60, or ~$43 with VL53L4CD (1.2 m suffices for a 0.3 m/s stop).
8. **"Pi Cam 3 $25"**: MSRP. Street $29.25 standard, $38.50 Wide. In production until at least January 2030.
9. **"USB mic $10"**: holds at $5.95. ReSpeaker Lite $26.99 if AEC is needed.
10. **"Amp + speaker $15"**: holds; $10.90 I2S or $12.50 USB.
11. **Missing lines**: INA260 $9.95, fuse + holder ~$3, XT60 $2.95, wire/JST ~$10, standoffs ~$8 (≈ $34); optional IMU $20–30; tools $80–130.
12. **"~$200–300"**: Path A ≈ $295 before shipping/tax, $300–350 delivered, $330–380 with IMU. The upper bound holds; $200 needs clones and AliExpress motors.
13. **Stock**: Pololu's Romi category page showed the 32U4 board, Motor Driver+PDB, PDB, encoders and plate as out of stock / backorder on 2026-09-07; the product pages say "Active and Preferred", backorders allowed, in-house items ship "within a few days" [VENDOR]. Order at build step 2, not 4.

**Tools for a first-time builder**: JST-XH/PH + Dupont crimper (Engineer PA-09 $49.95 [VENDOR] or IWISS SN-2549 ~$25 [INFERRED]); temperature-controlled iron for XT60 and speaker leads (Pinecil v2 ~$26 [INFERRED]); ratcheting 18–28 AWG crimper $34.95 [VENDOR] only if you crimp the battery side; multimeter $20–40 [INFERRED]; USB-C inline power meter $15–25 [INFERRED] for the `get_throttled` gate; M2.5/M3 standoff kit ~$8; heat-shrink, 22 AWG silicone wire, zip ties, VHB tape ~$15 [INFERRED]. $80–130 total.

## Alternatives considered and rejected

- **XIAO ESP32S3 ($7.49)**: 11 GPIO. **QT Py ESP32-S3 ($12.50) / Thing Plus ESP32-S3 ($24.95)**: 13 GPIO, no PSRAM, Thing Plus on backorder.
- **DRV8833 ($10.95)**: 10.8 V max; TB6612FNG does the 2S job for $4.95.
- **25D 12 V encoded motors ($56.95 each)**: correct only if 3S is mandatory; doubles the drive line and forces a 10–15 A BMS pack.
- **FIT0450 TT encoder motors ($7.40) on a generic 2WD plate**: ~$30 drive, but plastic TT gearboxes and press-fit wheels give poor odometry; 960 counts/rev vs Romi's 1440. Budget build only.
- **Talentcell as the single pack**: 3S and 3 A cap conflict with every sub-$100 drive; fine as a Pi-only pack if rails are split.
- **D24V50F5 ($32.95)**: rationed, older family, over-spec.
- **ReSpeaker 4-Mic Array v2 ($64)**: far-field beamforming unneeded at 1–2 m. **2-Mics Pi HAT v2 ($13.99)**: 3.5 mm out still needs an amp and takes the I2S pins.
- **INA226 clone ($6–9)**: no US stock, external shunt to size; INA260 is $9.95 with 15 A shunt.
- **TurboPi ($99.99), Raspbot V2 ($134.99), Elegoo V4 ($75.99)**: proprietary controllers, mecanum, no encoders; cannot host the ESP32-S3 safety chain.
- **Romi Robot Kit for FIRST ($159.95)**: kept as an alternative, not rejected; +$48 and an ATmega32U4.
- **Waveshare WAVE ROVER/UGV, General Driver for Robots, Cytron MDD3A**: unpriced (403).

## Open questions

- Talentcell YB1203000-USB Amazon price, and the 12 V current limit of the newer PB-series (76 Wh, PD 45 W) pack.
- Exact input-voltage ceiling of the Romi Motor Driver and PDB; 11 V is the DRV8838 limit, not a fetched board spec.
- Pi 4 current with Camera Module 3 at 640×480/10 fps plus USB mic and I2S amp; pidramble has no such row. Measure at build step 3.
- Whether Pololu Romi backorders ship within days in September 2026.
- Waveshare General Driver for Robots (ESP32 + dual driver + 5 V + INA219 + IMU) pricing; it could collapse four lines if a non-S3 ESP32 is acceptable.
- Whether openWakeWord/Vosk reach the brief's accuracy at 2 m with the $5.95 mic while the rover speaks; decides the ReSpeaker Lite upgrade.

## Sources

All fetched 2026-09-07.

- Adafruit: ESP32-S3-DevKitC-1-N8R8 #5336 https://www.adafruit.com/product/5336 ; WROOM-2 N32R16V #5364 https://www.adafruit.com/product/5364 ; QT Py ESP32-S3 #5426 https://www.adafruit.com/product/5426 ; VL53L1X #3967 https://www.adafruit.com/product/3967 ; VL53L4CD #5396 https://www.adafruit.com/product/5396 ; MAX98357A #3006 https://www.adafruit.com/product/3006 ; 40 mm speaker #3968 https://www.adafruit.com/product/3968 ; 3-inch speaker #1314 https://www.adafruit.com/product/1314 ; USB speaker #3369 https://www.adafruit.com/product/3369 ; USB mic #3367 https://www.adafruit.com/product/3367 ; SPH0645 #3421 https://www.adafruit.com/product/3421 ; INA260 #4226 https://www.adafruit.com/product/4226 ; INA219 #904 https://www.adafruit.com/product/904 ; INA228 #5832 https://www.adafruit.com/product/5832 ; BNO085 #4754 https://www.adafruit.com/product/4754 ; ICM-20948 #4554 https://www.adafruit.com/product/4554 ; encoder gearmotor #4416 https://www.adafruit.com/product/4416 ; Camera Module 3 #5657 https://www.adafruit.com/product/5657 ; crimp tools https://www.adafruit.com/?q=crimp
- Espressif: DevKitC-1 devkit page https://www.espressif.com/en/products/devkits/esp32-s3-devkitc-1 ; user guide v1.1 https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html
- Seeed: XIAO ESP32S3 https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html ; XIAO wiki https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/ ; ReSpeaker Lite https://www.seeedstudio.com/ReSpeaker-Lite-p-5928.html ; 2-Mics Pi HAT https://www.seeedstudio.com/ReSpeaker-2-Mics-Pi-HAT.html ; 4-Mic Array v2 https://www.seeedstudio.com/ReSpeaker-Mic-Array-v2-0.html
- Pololu: Romi Chassis #3500 https://www.pololu.com/product/3500 ; Encoder Pair #3542 https://www.pololu.com/product/3542 ; Romi motor #1520 https://www.pololu.com/product/1520 ; motor+encoder #3675 https://www.pololu.com/product/3675 ; Motor Driver+PDB #3543 https://www.pololu.com/product/3543 ; PDB #3541 https://www.pololu.com/product/3541 ; Expansion Plate #3560 https://www.pololu.com/product/3560 ; 32U4 board #3544 https://www.pololu.com/product/3544 ; FIRST kit #4022 https://www.pololu.com/product/4022 ; Romi category https://www.pololu.com/category/202/romi-chassis-and-accessories ; TB6612FNG #713 https://www.pololu.com/product/713 ; DRV8833 #2130 https://www.pololu.com/product/2130 ; DRV8876 #4036 https://www.pololu.com/product/4036 ; TB9051FTG #2997 https://www.pololu.com/product/2997 ; 25D encoder motor #4845 https://www.pololu.com/product/4845 ; 25D bracket #2676 https://www.pololu.com/product/2676 ; 90×10 wheels #1435 https://www.pololu.com/product/1435 ; D24V50F5 #2851 https://www.pololu.com/product/2851 ; D36V28F5 #3782 https://www.pololu.com/product/3782 ; D24V22F5 #2858 https://www.pololu.com/product/2858 ; regulator category https://www.pololu.com/category/131/step-down-voltage-regulators ; VL53L1X #3415 https://www.pololu.com/product/3415 ; VL53L4CD #3692 https://www.pololu.com/product/3692 ; bump switch #1405 https://www.pololu.com/product/1405 ; power switch #2808 https://www.pololu.com/product/2808 ; XT60 yellow #2175 https://www.pololu.com/product/2175 (black #2158 discontinued)
- SparkFun: VL53L1X Qwiic https://www.sparkfun.com/products/14722 ; Thing Plus ESP32-S3 https://www.sparkfun.com/products/24408 ; Big Dome Pushbutton https://www.sparkfun.com/big-dome-pushbutton-red.html
- Raspberry Pi: Camera Module 3 https://www.raspberrypi.com/products/camera-module-3/ ; Pi 4 specifications https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/ ; PiShop Camera Module 3 https://www.pishop.us/product/raspberry-pi-camera-module-3/
- pidramble power consumption (measured; Pi 4 rows 2019–2020) https://www.pidramble.com/wiki/benchmarks/power-consumption
- Talentcell YB1203000-USB https://talentcell.com/lithium-ion-battery/12v/yb1203000-usb.html ; 12 V listing https://talentcell.com/lithium-ion-battery/12v/
- 18650 Battery Store Samsung 30Q https://www.18650batterystore.com/products/samsung-30q
- DFRobot FIT0450 https://www.dfrobot.com/product-1457.html
- Kits (Shopify JSON): Hiwonder TurboPi https://www.hiwonder.com/products/turbopi.json ; Hiwonder Pi collection https://www.hiwonder.com/collections/raspberry-pi-robot/products.json ; Yahboom Raspbot V2 https://category.yahboom.net/products/raspbot-v2.json ; Yahboom Pi collection https://category.yahboom.net/collections/rp-robotics ; Elegoo V4 https://us.elegoo.com/products/elegoo-smart-robot-car-kit-v-4-0.json
- Blocked, not used: DigiKey, Mouser, Amazon, Waveshare, Cytron, RobotShop, AutomationDirect, IWISS.
