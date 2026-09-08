#pragma once

/* Sensor acquisition: the I2C bus, three VL53L4CX, the INA226 pack monitor,
 * the driver NTC, and the two discrete inputs that are not on any bus.
 *
 * The sensor task runs on core 0 at [safety] tof_poll_hz and publishes one
 * snapshot; the control task on core 1 samples that snapshot into rover_in_t
 * once per 100 Hz step. Discrete levels are read directly in the control
 * task, since they are a register read each.
 */

#include <stdint.h>

#include "rover_core.h"

void sensors_init(void);

/* Fills the sensor half of rover_in_t: distances and their status bits, pack
 * voltage and current, driver temperature, bumper and e-stop levels. A ToF
 * sample older than 200 ms becomes 65535 and sets that sensor's status bit
 * (I-16). Encoder ticks are filled by motion_read_encoders(). */
void sensors_fill(rover_in_t *in, uint64_t now_us);
