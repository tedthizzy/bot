#pragma once

/* UART1 to the Pi, 921600 8N1, and the two byte rings that keep rover_core_*
 * single-threaded.
 *
 * The comms task on core 0 moves bytes between the UART driver and the rings;
 * the control task on core 1 is the only caller of rover_core_feed() and
 * rover_core_drain_tx(). Nothing else opens this port: a second writer would
 * bypass session, seq and CRC (I-18), which is also why a release build has
 * no console on UART1, UART0 or USB-Serial/JTAG.
 */

#include <stddef.h>
#include <stdint.h>

void link_init(void);

/* Bytes received from the Pi, for rover_core_feed(). */
size_t link_rx_pop(uint8_t *dst, size_t cap);

/* Frames from rover_core_drain_tx(), for the wire. */
size_t link_tx_push(const uint8_t *src, size_t n);
