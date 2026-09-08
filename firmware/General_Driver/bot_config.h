// bot_config.h -- compile-time constants of the bot fork of ugv_base_general.
//
// Included from ugv_config.h. Every constant the fork adds lives here so the
// safety envelope can be audited in one file. The wire contract these serve is
// docs/protocol.md; the reasoning is docs/adr/0013-wave-rover-open-loop.md.
// Host tests read BOT_POWER_CAP and BOT_HEARTBEAT_MS from this file, so keep
// each define on one line with a plain literal.
#ifndef BOT_CONFIG_H
#define BOT_CONFIG_H

// --- identity ------------------------------------------------------------
// Sent in the T:1006 banner at the end of setup() and on T:1007. The host
// refuses to move unless fw names a fork build and hb_ms and cap equal its own
// configuration.
#define BOT_FW_TAG "bot-wr-1"
#define BOT_PROTO 1

// --- heartbeat -----------------------------------------------------------
// Motors are zeroed when no speed command (T:1, T:11, T:13) has arrived for
// this long. Stock is 3000. The host sends speed at 20 Hz, so 300 ms rides out
// five dropped lines and still stops a frozen host within 300 ms plus motor
// spin-down. HEART_BEAT_DELAY starts from this value, and
// changeHeartBeatDelay() accepts only values in (0, BOT_HEARTBEAT_MS], so
// nothing on the wire can weaken it.
#define BOT_HEARTBEAT_MS 300

// Bound incoming lines, per-poll work, and commands queued during vendor waits.
// Oversized lines are discarded through LF. A full wait queue drops new lines.
#define BOT_SERIAL_LINE_MAX 512
#define BOT_SERIAL_BYTES_PER_POLL 128
#define BOT_SERIAL_PENDING_LINES 4

// --- power cap -----------------------------------------------------------
// Speed units are Waveshare's: full scale +-0.5, multiplied by 512 into an
// 8-bit duty. 0.30 is 60 percent duty on the TB6612. setGoalSpeed() clamps its
// inputs to +-BOT_POWER_CAP and leftCtrl()/rightCtrl() clamp every PWM write
// to +-BOT_PWM_CAP, so T:11 raw PWM and the PID path are capped as well. Each
// clamp increments bot_clamp_count (feedback field cc).
#define BOT_POWER_CAP 0.30f
#define BOT_PWM_CAP (BOT_POWER_CAP * 512.0f)   // 153.6 -> duty 154 of 255

// --- radios --------------------------------------------------------------
// 0 compiles out the Wi-Fi access point with its HTTP control page and the
// ESP-NOW follower (stock accepts broadcast commands from any leader). The
// serial line is then the only control path, as docs/protocol.md requires.
#define BOT_WIFI_ENABLED 0
#define BOT_ESPNOW_ENABLED 0

// --- boot mission --------------------------------------------------------
// Stock replays the "boot" mission from flash at the end of setup(). A stored
// speed step would move the robot at power-on with nobody in the loop, so the
// fork neither creates nor plays it.
#define BOT_BOOT_MISSION 0

// --- encoders ------------------------------------------------------------
// The WAVE ROVER has no wheel encoders; stock attaches GPIO 34/35 (motor A) and
// 27/16 (motor B) to the pulse counter anyway. 0 skips initEncoders(),
// pidControllerInit() and the per-loop encoder reads, leaving those pins free
// (27 is the bumper input).
#define BOT_ENCODERS 0

// --- front time-of-flight ------------------------------------------------
// VL53L1X on Wire (SDA 32 / SCL 33, shared with the OLED, INA219 and IMU),
// Pololu VL53L1X library, short distance mode, 50 ms timing budget, continuous
// ranging, read without blocking. Forward (L>0 and R>0) is applied as zeros
// while the range is below BOT_TOF_STOP_MM (stop flag 2); reverse and rotation
// still pass.
#define BOT_TOF_ENABLED 1
#define BOT_TOF_STOP_MM 250
// The board has one I2C connector and the OLED cable occupies it, so the
// sensor is spliced onto the same 3.3 V bus (4.7 k pull-ups on board). Taken
// addresses: 0x3C OLED, 0x42 INA219, 0x6B QMI8658, 0x0C AK09918, 0x77 BMP280
// (may be populated). 0x29, the VL53L1X power-up default, is free.
#define BOT_TOF_ADDR 0x29
// The feedback field tf is -1 when the sensor was absent at boot, has produced
// no valid range, or has been silent for BOT_TOF_STALE_MS.
//   BOT_TOF_REQUIRED 0: tf == -1 does not block forward, so the rover runs
//                       before the sensor arrives. Default until it is fitted.
//   BOT_TOF_REQUIRED 1: tf == -1 blocks forward. A missing, unplugged or
//                       silent sensor is never treated as clear.
#define BOT_TOF_REQUIRED 0
#define BOT_TOF_BUDGET_US 50000   // measurement timing budget
#define BOT_TOF_PERIOD_MS 50      // inter-measurement period, >= budget
#define BOT_TOF_STALE_MS 500      // no fresh sample for this long -> tf = -1
#define BOT_TOF_POLL_MS 10        // how often loop() polls dataReady()

// --- bumper --------------------------------------------------------------
// One switch to GND on BOT_BUMPER_PIN with the internal pull-up: LOW = pressed.
// Debounced over BOT_BUMPER_DEBOUNCE_MS. Pressed blocks forward (stop flag 4).
// Off until the wiring is confirmed; bp is still reported and reads 0.
// GPIO 27 is on the 7-pin P3 header (IO5, 3V3, GND, IO16, IO27, CP_RX, U0RX)
// and on the motor-B encoder connector H4 next to 3V3 and GND; BOT_ENCODERS 0
// is what frees it. GPIO 34 and 35 are input-only with no internal pull-up
// and would need an external resistor.
#define BOT_BUMPER_ENABLED 0
#define BOT_BUMPER_PIN 27
#define BOT_BUMPER_DEBOUNCE_MS 20

// --- low battery ---------------------------------------------------------
// INA219 bus voltage is the pack voltage; its current reading covers only the
// 5 V branch, so no motor-current rule is possible and voltage is the rule.
// Sampled every BOT_BATT_SAMPLE_MS. Below
// BOT_LOWBAT_V for BOT_LOWBAT_TRIP_MS without interruption latches stop flag 8
// and every motion command is applied as zeros; above BOT_LOWBAT_RECOVER_V for
// BOT_LOWBAT_RECOVER_MS without interruption clears it. 9.9 V is 3.3 V per
// cell on the three-cell pack. On USB power with the pack switched off the bus
// reads about 0 V and the flag latches after 10 s: expected, not a fault.
#define BOT_LOWBAT_V 9.9f
#define BOT_LOWBAT_RECOVER_V 10.2f
#define BOT_LOWBAT_TRIP_MS 10000
#define BOT_LOWBAT_RECOVER_MS 30000
#define BOT_BATT_SAMPLE_MS 500

// --- stop flags (feedback field st), docs/protocol.md -------------------
#define BOT_ST_HEARTBEAT 1    // heartbeat expired; motors zeroed by the firmware
#define BOT_ST_TOF 2          // forward blocked by the time-of-flight sensor
#define BOT_ST_BUMPER 4       // forward blocked by the bumper
#define BOT_ST_LOWBAT 8       // all motion refused: low battery
#define BOT_ST_COAST 16       // coast requested by the host (T:115)

// --- consistency ---------------------------------------------------------
#if BOT_TOF_REQUIRED && !BOT_TOF_ENABLED
#error "BOT_TOF_REQUIRED 1 needs BOT_TOF_ENABLED 1"
#endif
#if BOT_BUMPER_ENABLED && BOT_ENCODERS && (BOT_BUMPER_PIN == 27 || BOT_BUMPER_PIN == 16 || BOT_BUMPER_PIN == 34 || BOT_BUMPER_PIN == 35)
#error "BOT_BUMPER_PIN collides with an encoder pin; set BOT_ENCODERS 0"
#endif
#if BOT_TOF_PERIOD_MS * 1000 < BOT_TOF_BUDGET_US
#error "BOT_TOF_PERIOD_MS must cover BOT_TOF_BUDGET_US"
#endif

#endif  // BOT_CONFIG_H
