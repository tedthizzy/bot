#pragma once

/* Single-producer / single-consumer byte ring.
 *
 * It exists so that rover_core_* is only ever called from the control task:
 * the comms task on core 0 pushes received bytes and pops bytes to transmit,
 * the control task on core 1 does the opposite, and neither takes a lock. A
 * mutex shared with the 100 Hz task under a 1 s Task-WDT is the thing this
 * avoids.
 *
 * Capacity must be a power of two.
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t *buf;
    size_t mask;
    _Atomic size_t head; /* producer writes */
    _Atomic size_t tail; /* consumer writes */
} bytering_t;

static inline void bytering_init(bytering_t *r, uint8_t *storage, size_t capacity)
{
    r->buf = storage;
    r->mask = capacity - 1;
    atomic_store(&r->head, 0);
    atomic_store(&r->tail, 0);
}

/* Returns the number of bytes accepted; a full ring drops the remainder,
 * which is the same outcome as a corrupted line and is counted as one. */
static inline size_t bytering_push(bytering_t *r, const uint8_t *src, size_t n)
{
    size_t head = atomic_load_explicit(&r->head, memory_order_relaxed);
    size_t tail = atomic_load_explicit(&r->tail, memory_order_acquire);
    size_t free_bytes = r->mask - (head - tail);
    if (n > free_bytes) {
        n = free_bytes;
    }
    for (size_t i = 0; i < n; i++) {
        r->buf[(head + i) & r->mask] = src[i];
    }
    atomic_store_explicit(&r->head, head + n, memory_order_release);
    return n;
}

static inline size_t bytering_pop(bytering_t *r, uint8_t *dst, size_t cap)
{
    size_t tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
    size_t head = atomic_load_explicit(&r->head, memory_order_acquire);
    size_t avail = head - tail;
    if (avail > cap) {
        avail = cap;
    }
    for (size_t i = 0; i < avail; i++) {
        dst[i] = r->buf[(tail + i) & r->mask];
    }
    atomic_store_explicit(&r->tail, tail + avail, memory_order_release);
    return avail;
}
