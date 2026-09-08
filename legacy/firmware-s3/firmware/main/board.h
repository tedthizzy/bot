#pragma once

/* board.h -- every pin and hardware constant for the v1 rover MCU, in one place.
 *
 * Target board: ESP32-S3-DevKitC-1-N8R8 (8 MB quad flash, 8 MB octal PSRAM).
 * Source of truth: ARCHITECTURE.md section 3, table "Pin map". A change here
 * without a change there is a bug.
 *
 * Anything the core compiles in -- caps, ToF timing, control rate, current and
 * battery thresholds -- lives in firmware/core/rover_config.h and is used from
 * there by its ROVER_ name, never copied. The B banner's safety_hash is a
 * CRC-32 over seven of those constants, so a second copy of one of them here
 * would be a copy the Pi cannot check.
 *
 * Pins that deliberately carry nothing:
 *   GPIO0, 3, 45, 46    strapping pins
 *   GPIO19, 20          USB D-/D+
 *   GPIO26..37          octal flash and PSRAM
 *   GPIO43, 44          CP2102 UART0 bridge
 * Using GPIO39..42 forfeits hardware JTAG, which USB-Serial/JTAG replaces.
 */

#include <stdint.h>

#include "rover_config.h"

/* ---------------------------------------------------- what is fitted */

/* Feeds the B banner's caps word. The ICM-20948 and the SERVO_EN FET are
 * deferred to the G6 merge track, so nothing in v1 consumes either. */
#define BOARD_HAS_CLIFF      1
#define BOARD_HAS_IMU        0
#define BOARD_HAS_INA        1
#define BOARD_HAS_SERVO_RAIL 0

/* ------------------------------------------------------- motor driver, PWM */

/* Cytron MDD3A, sign-magnitude: one input PWM and the other low drives that
 * direction; both high brakes; both low at reset brakes and never drives.
 * Each pin carries an external 10 kOhm pull-down (I-24). */
#define BOARD_GPIO_M1A 4 /* left  motor, forward  */
#define BOARD_GPIO_M1B 5 /* left  motor, reverse  */
#define BOARD_GPIO_M2A 6 /* right motor, forward  */
#define BOARD_GPIO_M2B 7 /* right motor, reverse  */

/* 20 kHz carrier, above audibility and inside the MDD3A's input rating.
 * 80 MHz / 4000 ticks = 20 kHz, so a duty step is 1/4000. */
#define BOARD_PWM_FREQ_HZ       20000
#define BOARD_PWM_RESOLUTION_HZ 80000000
#define BOARD_PWM_PERIOD_TICKS  (BOARD_PWM_RESOLUTION_HZ / BOARD_PWM_FREQ_HZ)

/* MOTOR_EN drives the low side of the 30 A relay coil through a logic-level
 * N-MOSFET. 10 kOhm to GND, so it floats low through reset and the relay is
 * open. Lifecycle in ARCHITECTURE 4.1: asserted on the first A of a power
 * cycle and held; deasserted only on UNDERVOLT_S/_D, a latched fault, or
 * shutdown -- never on D, S, TTL expiry or an obstacle-class fault. The core
 * owns that rule; main only applies rover_out_t.motor_en. */
#define BOARD_GPIO_MOTOR_EN 12

/* SERVO_EN is deferred to G6; in v1 the pin drives an LED. 10 kOhm to GND. */
#define BOARD_GPIO_SERVO_EN 39

/* ---------------------------------------------------------- MCPWM faults */

/* Two GPIO fault inputs, both one-shot (OST), and only these two: a trip
 * forces all four MDD3A inputs to their fault action and cannot be cleared
 * while the fault signal is asserted. The bumper is deliberately NOT here --
 * a held bumper would kill reverse and rotation too and could never pass I-5.
 * The FAULT0/FAULT2 labels are the wiring table's; the IDF driver allocates
 * the fault detector index itself. */
#define BOARD_GPIO_FAULT_INA_ALERT 9  /* INA226 ALERT, active low, ext 10k pull-up */
#define BOARD_GPIO_FAULT_ESTOP     11 /* e-stop coil node A via 100k/33k divider,
                                       * active low: 11.1 V -> 2.75 V, 3V3 clamp
                                       * diode + 100 nF. Sensed ABOVE the coil, so a
                                       * closed button reads high whatever the
                                       * MOTOR_EN FET is doing. ctrl_flags b1. */

/* --------------------------------------------------------------- encoders */

/* JGB37-520 50:1, 11 PPR quadrature -> ROVER_TICKS_PER_REV counts per output
 * revolution in x4 decoding. PCNT with accum_count, so the i32 tick totals in
 * T never wrap at the 16-bit hardware counter. */
#define BOARD_GPIO_ENC_L_A 15
#define BOARD_GPIO_ENC_L_B 16
#define BOARD_GPIO_ENC_R_A 17
#define BOARD_GPIO_ENC_R_B 18

#define BOARD_PCNT_HIGH_LIMIT 10000
#define BOARD_PCNT_LOW_LIMIT  (-10000)
/* 12.5 ns is one APB tick, the intended filter width. The driver computes
 * threshold = apb_mhz * ns / 1000 in integer arithmetic, so 12 rounds to 0
 * (filter off) and 13 is the smallest value that yields the one tick. */
#define BOARD_ENC_GLITCH_NS 13

/* ------------------------------------------------------ discrete sensors */

/* Two NC bumper switches in series: 3V3 -> both contacts -> GPIO10, 10 kOhm to
 * GND. At rest the line is high; either switch pressed, a broken wire or an
 * unseated connector pulls it low -- a fail-safe OR. Plain GPIO plus an ISR,
 * never an MCPWM fault. */
#define BOARD_GPIO_BUMPER 10

/* ------------------------------------------------- Pi rail and shutdown */

/* Open drain, no external pull: the D24V50F5 pulls its own EN to VIN through
 * a 10 kOhm series resistor, so a reset, unprogrammed or unpowered MCU leaves
 * the Pi rail ON and only an actively driven low kills it. An external
 * pull-up to VIN would sit the pad a diode drop above VDD. */
#define BOARD_GPIO_PI_RAIL_EN 1

/* Driven high to ask the Pi for a clean halt (its GPIO17, gpio-shutdown). */
#define BOARD_GPIO_PI_SHUTDOWN_REQ 21

/* The Pi's gpio-poweroff pulse train on its GPIO26: active 100 ms, inactive
 * 100 ms, active. Three edges inside 2 s confirms the halt completed; the
 * fallback if it never arrives is ROVER_POWEROFF_TIMEOUT_MS. */
#define BOARD_GPIO_PI_POWEROFF_IN 8
#define BOARD_POWEROFF_EDGES      3
#define BOARD_POWEROFF_WINDOW_US  2000000ULL

/* -------------------------------------------------------------- I2C bus */

#define BOARD_GPIO_I2C_SDA   13 /* 4.7 kOhm pull-up */
#define BOARD_GPIO_I2C_SCL   14 /* 4.7 kOhm pull-up */
#define BOARD_I2C_PORT       0
#define BOARD_I2C_FREQ_HZ    400000
#define BOARD_I2C_TIMEOUT_MS 20

/* ------------------------------------------------------- ToF, VL53L4CX x3 */

/* All three power up at 0x29. Hold every XSHUT low, release front-L and write
 * 0x30, front-R 0x31, cliff 0x32. Each XSHUT carries 10 kOhm to GND, so an
 * unprogrammed MCU leaves every sensor held in reset. */
#define BOARD_GPIO_TOF_XSHUT_FL 40
#define BOARD_GPIO_TOF_XSHUT_FR 41
#define BOARD_GPIO_TOF_XSHUT_CL 42

#define BOARD_TOF_ADDR_DEFAULT 0x29
#define BOARD_TOF_ADDR_FL      0x30
#define BOARD_TOF_ADDR_FR      0x31
#define BOARD_TOF_ADDR_CL      0x32

/* Index order is the E-frame arg convention of 5.1 and the bit order of
 * rover_in_t.tof_status: 0 front-L, 1 front-R, 2 cliff. */
#define BOARD_TOF_FL    0
#define BOARD_TOF_FR    1
#define BOARD_TOF_CLIFF 2
#define BOARD_TOF_COUNT 3

/* Both forward sensors mount at 90 mm and are yawed +-9 deg so their 18 deg
 * cones abut on the centreline. Short distance mode with
 * ROVER_TOF_TIMING_BUDGET_MS and ROVER_TOF_INTER_PERIOD_MS at
 * ROVER_TOF_POLL_HZ are the four numbers A21's detect latency derives from.
 * The mode/budget pairing is UNVERIFIED for the VL53L4CX, which is why the
 * driver reads both back and refuses the sensor if they differ. */
#define BOARD_TOF_SHORT_MODE 1

/* A sensor that has errored this many times in a row is re-addressed and
 * re-initialised, at most once per backoff window. */
#define BOARD_TOF_REINIT_ERRORS     10
#define BOARD_TOF_REINIT_BACKOFF_US 5000000ULL

/* ------------------------------------------------------- INA226 monitor */

/* 2 mOhm shunt on the pack. Bus voltage is vbat_mv; the current register is
 * imotor_ma (negative = regen). ALERT is wired to MCPWM FAULT0 and armed at
 * ROVER_OVERCURRENT_MA -- A24's first current-limit layer, in hardware, ahead
 * of the software I2t and slip detectors in the core. */
#define BOARD_INA226_ADDR       0x40
#define BOARD_INA226_SHUNT_UOHM 2000
/* Current LSB 1 mA: CAL = 0.00512 / (LSB_A * R_ohm) = 0.00512/(0.001*0.002). */
#define BOARD_INA226_CAL 2560
/* Bus voltage LSB is 1.25 mV; shunt voltage LSB is 2.5 uV. */
#define BOARD_INA226_BUS_LSB_UV 1250
#define BOARD_INA226_ALERT_RAW \
    ((ROVER_OVERCURRENT_MA * BOARD_INA226_SHUNT_UOHM) / 2500)

/* ---------------------------------------------------------- driver NTC */

/* 10 kOhm from 3V3 to the node, NTC from the node to GND. B25/50 = 3950,
 * R25 = 10 kOhm. Feeds the core's ROVER_DRIVER_HOT_C rule. */
#define BOARD_GPIO_NTC       2
#define BOARD_NTC_ADC_CHAN   1 /* GPIO2 is ADC1 channel 1 on the ESP32-S3 */
#define BOARD_NTC_SERIES_OHM 10000.0f
#define BOARD_NTC_R25_OHM    10000.0f
#define BOARD_NTC_BETA       3950.0f
#define BOARD_NTC_VDD_MV     3300.0f

/* ------------------------------------------------------------- Pi link */

#define BOARD_UART_PORT    1
#define BOARD_GPIO_UART_TX 47
#define BOARD_GPIO_UART_RX 48
#define BOARD_UART_BAUD    921600
/* A T line is ~100 bytes at 50 Hz = 5.4% of the link. The rings cover a
 * comms-task stall of several control periods without dropping a frame. */
#define BOARD_LINK_RX_RING 2048
#define BOARD_LINK_TX_RING 4096

/* --------------------------------------------------------------- tasks */

/* Control task on core 1 off a GPTimer alarm at ROVER_CTRL_HZ; comms and
 * sensors on core 0. */
#define BOARD_CONTROL_CORE  1
#define BOARD_IO_CORE       0
#define BOARD_CONTROL_PRIO  20
#define BOARD_COMMS_PRIO    15
#define BOARD_SENSOR_PRIO   10
#define BOARD_CONTROL_STACK 4096
#define BOARD_COMMS_STACK   3072
#define BOARD_SENSOR_STACK  4096
/* If the GPTimer alarm ever stops, the control task still runs at 20 Hz so
 * the TTL expires and the wheels brake rather than holding the last duty. */
#define BOARD_CONTROL_WAIT_MS 50
