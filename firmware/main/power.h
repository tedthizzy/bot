#pragma once

/* Pi rail control and the orderly-shutdown handshake.
 *
 * PI_RAIL_EN is open drain with no external pull: the buck holds its own EN
 * high, so a reset, unprogrammed or unpowered MCU leaves the Pi rail on and
 * only an actively driven low kills it (I-6).
 *
 * rover_in_t carries no poweroff-in field and rover_out_t asks for the rail
 * directly, so the pulse-train gate of ARCHITECTURE 9 lives here: the rail is
 * cut only after the Pi's gpio-poweroff train has been seen, or after the
 * 60 s fallback.
 */

#include <stdint.h>

#include "rover_core.h"

void power_init(void);

/* Applies rover_out_t's rail and shutdown outputs for this control step. */
void power_apply(const rover_out_t *out, uint64_t now_us);
