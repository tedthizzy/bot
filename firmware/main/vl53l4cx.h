#pragma once

/* Minimal VL53L4CX driver.
 *
 * No managed component is used: the ESP component registry carries no
 * VL53L4CX driver, and ST's full ULD is a multi-object-detection library an
 * order of magnitude larger than the six operations this design needs
 * (address assignment, short/long distance mode, timing budget,
 * inter-measurement period, data-ready poll, one distance plus its status).
 * The register map below is the VL53L1X family map that the L4CX shares for
 * exactly that subset; ARCHITECTURE 4.1 already records the mode/budget
 * pairing as UNVERIFIED for this part, which is why every setting is read
 * back and asserted here rather than assumed.
 *
 * The three-valued result of ARCHITECTURE 5.1 is produced here, because
 * rover_in_t carries a distance and one status bit per sensor and no
 * RangeStatus field: a real distance, 65534 for no target within range
 * (forward allowed), 65535 for a sensor or I2C error (forward refused).
 */

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t address;
    int xshut_gpio;
} vl53l4cx_t;

/* Drives XSHUT low, releases it, waits for the boot flag, moves the sensor
 * from 0x29 to `address`, loads the default configuration and starts ranging
 * at the compiled distance mode, timing budget and inter-measurement period.
 * Fails if the sensor reports back a budget or period other than the
 * compiled one. */
esp_err_t vl53l4cx_start(vl53l4cx_t *s, i2c_master_bus_handle_t bus, int xshut_gpio,
                         uint8_t address, bool short_mode, uint16_t budget_ms,
                         uint16_t inter_period_ms);

/* Releases the device handle and holds the sensor in reset. */
void vl53l4cx_stop(vl53l4cx_t *s);

esp_err_t vl53l4cx_data_ready(vl53l4cx_t *s, bool *ready);

/* Reads one measurement and clears the interrupt. `*mm` is a distance, or
 * 65534 (no target) or 65535 (error). */
esp_err_t vl53l4cx_read(vl53l4cx_t *s, uint16_t *mm);

esp_err_t vl53l4cx_get_timing_budget_ms(vl53l4cx_t *s, bool short_mode, uint16_t *ms);
esp_err_t vl53l4cx_get_inter_period_ms(vl53l4cx_t *s, uint16_t *ms);
