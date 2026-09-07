#pragma once

/* Motor driver and encoders: MCPWM at 20 kHz on the four MDD3A inputs, two
 * one-shot GPIO fault inputs, and two PCNT units decoding quadrature with
 * hardware accumulation.
 */

#include <stdint.h>

#include "rover_core.h"

void motion_init(void);

/* Accumulated quadrature counts since boot, the raw ticks T publishes and
 * the Pi integrates odometry from. */
void motion_read_encoders(int32_t *left_ticks, int32_t *right_ticks);

/* Applies one control step: PWM duty and brake on the driver inputs,
 * MOTOR_EN on the relay coil FET, SERVO_EN on its (G6) rail. */
void motion_apply(const rover_out_t *out);
