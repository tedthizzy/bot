/* Internals shared between the core's translation units and its host tests.
 *
 * `firmware/main/`, `firmware/sim/` and the ctypes shim see only
 * `include/rover_core.h`; nothing outside `firmware/` includes this file.
 */
#ifndef ROVER_INTERNAL_H
#define ROVER_INTERNAL_H

#include "rover_core.h"

/* ---- integer formatting ------------------------------------------------
 * The wire is integers and uppercase hex only (A5), so the core needs no
 * printf: three small helpers keep it freestanding and keep the encoder's
 * cost independent of libc.  Each writes without a terminator and returns the
 * byte count.  `out` must have room for 20 bytes.
 */

static inline int32_t rover_iabs32(int32_t value)
{
    return value < 0 ? -value : value;
}

static inline size_t rover_fmt_udec(uint64_t value, char *out)
{
    char tmp[20];
    size_t n = 0;
    do {
        tmp[n++] = (char)('0' + (int)(value % 10u));
        value /= 10u;
    } while (value != 0);
    for (size_t i = 0; i < n; i++) {
        out[i] = tmp[n - 1 - i];
    }
    return n;
}

static inline size_t rover_fmt_dec(int64_t value, char *out)
{
    if (value < 0) {
        out[0] = '-';
        /* Negate in unsigned space so INT64_MIN does not overflow. */
        return 1 + rover_fmt_udec(~(uint64_t)value + 1u, out + 1);
    }
    return rover_fmt_udec((uint64_t)value, out);
}

static inline size_t rover_fmt_hex(uint64_t value, int width, char *out)
{
    static const char digits[] = "0123456789ABCDEF";
    char tmp[16];
    size_t n = 0;
    do {
        tmp[n++] = digits[value & 0xFu];
        value >>= 4;
    } while (value != 0);
    size_t pad = ((size_t)width > n) ? (size_t)width - n : 0;
    for (size_t i = 0; i < pad; i++) {
        out[i] = '0';
    }
    for (size_t i = 0; i < n; i++) {
        out[pad + i] = tmp[n - 1 - i];
    }
    return pad + n;
}

/* ---- core state -------------------------------------------------------- */

#define ROVER_TX_BYTES 2048
#define ROVER_LEFT 0
#define ROVER_RIGHT 1
#define ROVER_LOOP_WINDOW 100 /* control steps per loop_late_pct sample */

/** One wheel's slip/stall and plausibility timers (A24, 4.1). */
typedef struct {
    uint16_t hard_ms;      /**< |v_meas| < 5% of |v_cmd| while duty > 40%. */
    uint16_t slip_ms;      /**< |v_meas| < 40% of |v_cmd|, same duty gate. */
    uint16_t implaus_cycles;
    uint64_t i2t_ma2_ms;   /**< integral of max(0, I_ch - 3 A) squared. */
} rover_wheel_t;

struct rover_core {
    rover_cfg_t cfg;

    /* link */
    rover_line_reader_t reader;
    rover_rx_counters_t rx;
    uint16_t session;
    uint16_t last_down_seq;
    uint16_t up_seq;
    bool hello_seen;
    uint32_t host_boot_id;
    uint8_t tx[ROVER_TX_BYTES];
    uint16_t tx_len;

    /* clocks -- all the MCU's own; no host value is ever read as a time (I-17) */
    uint64_t now_us;
    uint64_t last_step_us;
    uint64_t next_telem_us;
    uint64_t next_banner_us;
    uint64_t last_clamp_ack_us;
    uint64_t link_window_us;
    uint32_t link_window_drops;

    /* commanded setpoint and its TTL (T1) */
    int32_t v_target_mm_s;
    int32_t w_target_mrad_s;
    uint8_t v_flags;
    uint64_t ttl_deadline_us;
    bool ttl_expired;
    uint64_t ttl_expired_since_us;
    uint64_t last_v_us; /**< heartbeat for SERVO_EN (5.1). */
    bool have_v;

    /* controller */
    int32_t v_cmd_mm_s;
    int32_t w_cmd_mrad_s;
    rover_pi_t pi[2];
    int16_t duty_q15[2];
    bool coast;

    /* odometry */
    bool have_ticks;
    int32_t last_ticks[2];
    int32_t wheel_mm_s[2];
    int32_t wheel_cmd_mm_s[2];
    int32_t v_meas_mm_s;
    int32_t w_meas_mrad_s;

    /* discrete state */
    uint8_t state;
    bool armed;
    uint32_t fault;
    uint16_t ctrl_flags;
    bool brake;
    bool motor_en;
    bool servo_en;
    bool pi_rail_en;
    bool pi_shutdown_req;
    uint64_t shutdown_since_us;

    /* forward ToF, cliff and bumper */
    uint32_t tof_front_mm;
    bool tof_ok[3]; /**< 0 front-L, 1 front-R, 2 cliff -- the E arg indices (5.1) */
    bool in_slow_zone;
    uint8_t tof_stop_count;
    uint8_t tof_clear_count;
    uint8_t tof_stale_clear_count;
    uint8_t cliff_count;
    uint8_t cliff_clear_count;
    uint8_t bumper_clear_count;
    uint64_t tof_good_us;
    uint64_t tof_sample_due_us; /**< one sensor sample per inter-measurement period */
    uint16_t cliff_sample[ROVER_CLIFF_SAMPLES];
    uint8_t cliff_sample_n;
    uint16_t cliff_baseline_mm;
    bool cal_valid;

    /* obstacle-class escalation (5.1); the escalated state itself is the
     * latched fault bit ROVER_FAULT_OBSTACLE_LATCHED, not a private flag. */
    uint64_t obstacle_since_us;
    uint8_t obstacle_tries;
    bool forward_requested;

    /* per-wheel protection */
    rover_wheel_t wheel[2];

    /* battery ladder (A25) */
    int32_t v_oc_mv;
    uint32_t below_ms[3];  /**< warn, stop, disable */
    uint32_t above_ms[2];  /**< warn, stop recovery */
    bool ina_stale;        /**< the INA226 read failed: freeze the ladder, stop motion */

    /* loop health */
    uint16_t loop_steps;
    uint16_t loop_late;
    uint8_t loop_late_pct;
    uint8_t loop_late_frames;
};

/* Shared between the core's translation units. */
void rover_core_emit(struct rover_core *core, const rover_frame_t *frame);
void rover_core_event(struct rover_core *core, rover_event_t event, int32_t arg);
void rover_core_set_fault(struct rover_core *core, uint32_t bits);
void rover_core_clear_fault(struct rover_core *core, uint32_t bits);

void rover_safety_step(struct rover_core *core, const rover_in_t *in, uint32_t dt_us);
void rover_control_step(struct rover_core *core, uint32_t dt_us);

#endif /* ROVER_INTERNAL_H */
