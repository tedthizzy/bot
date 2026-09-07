/* rover_core -- the freestanding half of the ESP32-S3 motion controller (A3).
 *
 * Zero ESP-IDF headers, no dynamic allocation, no I/O, and no floating point:
 * `firmware/main/` supplies the peripherals and `firmware/sim/` supplies a
 * wheel plant, and both link this same object code (A3), which is what lets
 * 19 of the 24 invariants go green on a MacBook before hardware exists.
 *
 * All state lives in an opaque `rover_core_t` with no file-scope storage, so a
 * host test, a sanitiser and the ctypes shim can hold several instances at
 * once.  Allocate `rover_core_size()` bytes with any alignment a `uint64_t`
 * accepts.
 *
 * Units are in the names: mm, mm/s, mrad/s, mV, mA, us.
 */
#ifndef ROVER_CORE_H
#define ROVER_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rover_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================== */
/* Wire protocol (ARCHITECTURE 5.1)                                      */
/* ===================================================================== */

/** Longest field list of any frame type: T carries 21. */
#define ROVER_MAX_FIELDS 21

/** T.state. */
typedef enum {
    ROVER_STATE_BOOT = 0,
    ROVER_STATE_DISARMED = 1,
    ROVER_STATE_ARMED_IDLE = 2,
    ROVER_STATE_ARMED_MOVING = 3,
    ROVER_STATE_FAULT = 4,
    ROVER_STATE_ESTOP = 5
} rover_state_t;

/** K.result. */
typedef enum {
    ROVER_ACK_OK = 0,
    ROVER_ACK_REJECT = 1,
    ROVER_ACK_CLAMPED = 2
} rover_ack_result_t;

/** K.reason, and the reason a decode or a session check failed. */
typedef enum {
    ROVER_REASON_NONE = 0,
    ROVER_REASON_BAD_SESSION = 1,
    ROVER_REASON_STALE_SEQ = 2,
    ROVER_REASON_BAD_CRC = 3,
    ROVER_REASON_BAD_LENGTH = 4,
    ROVER_REASON_UNKNOWN_TYPE = 5,
    ROVER_REASON_UNSUPPORTED_VERSION = 6,
    ROVER_REASON_NOT_ARMED = 7,
    ROVER_REASON_FAULT_LATCHED = 8,
    ROVER_REASON_ESTOP_ASSERTED = 9,
    ROVER_REASON_BUMPER_CLOSED = 10,
    ROVER_REASON_TOF_BLOCKED = 11,
    ROVER_REASON_UNDERVOLTAGE = 12,
    ROVER_REASON_CAP_EXCEEDED = 13,
    ROVER_REASON_FRAME_TTL_OUT_OF_RANGE = 14,
    ROVER_REASON_SENSORS_STALE = 15,
    ROVER_REASON_ARM_DENIED_MOVING = 16
} rover_reason_t;

/** T.ctrl_flags; bits 10-15 are reserved and stay 0. */
enum {
    ROVER_CF_TTL_OK = 1u << 0,
    ROVER_CF_ESTOP_RELEASED = 1u << 1,
    ROVER_CF_BUMPER_CLEAR = 1u << 2,
    ROVER_CF_TOF_CLEAR = 1u << 3,
    ROVER_CF_PWM_ENABLED = 1u << 4,
    ROVER_CF_IN_SLOW_ZONE = 1u << 5,
    ROVER_CF_CAL_VALID = 1u << 6,
    ROVER_CF_DEBUG_BUILD = 1u << 7,
    ROVER_CF_TOF_FL_OK = 1u << 8,
    ROVER_CF_TOF_FR_OK = 1u << 9
};

/** T.fault; bits 20-31 are reserved.  Three classes, below. */
enum {
    ROVER_FAULT_TTL = 0x1u,
    ROVER_FAULT_ESTOP = 0x2u,
    ROVER_FAULT_BUMPER = 0x4u,
    ROVER_FAULT_TOF_STOP = 0x8u,
    ROVER_FAULT_TOF_STALE = 0x10u,
    ROVER_FAULT_CLIFF = 0x20u,
    ROVER_FAULT_OVERCURRENT = 0x40u,
    ROVER_FAULT_STALL = 0x80u,
    ROVER_FAULT_UNDERVOLT_W = 0x100u,
    ROVER_FAULT_UNDERVOLT_S = 0x200u,
    ROVER_FAULT_UNDERVOLT_D = 0x400u,
    ROVER_FAULT_DRIVER_FAULT = 0x800u,
    ROVER_FAULT_ENC_IMPLAUS = 0x1000u,
    ROVER_FAULT_LOOP_OVERRUN = 0x2000u,
    ROVER_FAULT_LINK_CRC = 0x4000u,
    ROVER_FAULT_SESSION = 0x8000u,
    ROVER_FAULT_CAP_CLAMPED = 0x10000u,
    ROVER_FAULT_WDT_REBOOT = 0x20000u,
    ROVER_FAULT_BROWNOUT = 0x40000u,
    ROVER_FAULT_DRIVER_HOT = 0x80000u,
    /* Escalation moves an obstacle-class cause *into the latched class* (5.1).
     * A separate boolean no `C` could name is not that, so escalation raises a
     * real bit: it is visible on the wire, robotd's `clear` can name it, and
     * the obstacle bits themselves keep their five-clean-sample rule. */
    ROVER_FAULT_OBSTACLE_LATCHED = 0x100000u
};

/** Set and cleared freely; robotd's readiness gate ignores this class. */
#define ROVER_FAULTS_ADVISORY                                                  \
    (ROVER_FAULT_TTL | ROVER_FAULT_UNDERVOLT_W | ROVER_FAULT_SESSION |         \
     ROVER_FAULT_CAP_CLAMPED)
/** Blocking but self-clearing; the MCU is the sole clearer of these bits. */
#define ROVER_FAULTS_OBSTACLE                                                  \
    (ROVER_FAULT_TOF_STOP | ROVER_FAULT_TOF_STALE | ROVER_FAULT_BUMPER |       \
     ROVER_FAULT_CLIFF)
/** Everything else: enters FAULT, refuses V with reason 8, needs a C. */
#define ROVER_FAULTS_LATCHED                                                   \
    (0x001FFFFFu & ~(uint32_t)(ROVER_FAULTS_ADVISORY | ROVER_FAULTS_OBSTACLE))

/** E.event.  The text table lives on the Pi; the numbers live here. */
typedef enum {
    ROVER_EVENT_ARM_OK = 1,
    ROVER_EVENT_ARM_DENIED = 2,
    ROVER_EVENT_TTL_EXPIRED = 3,
    ROVER_EVENT_TTL_RECOVERED = 4,
    ROVER_EVENT_FAULT_SET = 5,
    ROVER_EVENT_FAULT_CLEARED = 6,
    ROVER_EVENT_CAP_CLAMP = 7,
    ROVER_EVENT_WDT_REBOOT = 8,
    ROVER_EVENT_BROWNOUT = 9,
    ROVER_EVENT_I2C_ERROR = 10,
    ROVER_EVENT_TOF_STATUS = 11,
    ROVER_EVENT_SESSION_RESET = 12,
    ROVER_EVENT_LOOP_OVERRUN = 13,
    ROVER_EVENT_STALL = 14,
    ROVER_EVENT_CAL_STORED = 15
} rover_event_t;

/** V.flags.  Every bit is narrowing-only (I-4); b2-7 reserved = 0. */
enum {
    ROVER_VFLAG_REQUIRE_SLOW_ZONE_STOP = 1u << 0,
    ROVER_VFLAG_SERVO_RAIL_EN = 1u << 1
};

/** B.caps feature bitmask. */
enum {
    ROVER_CAP_DEBUG_BUILD = 1u << 0,
    ROVER_CAP_CLIFF_SENSOR = 1u << 1,
    ROVER_CAP_IMU = 1u << 2,
    ROVER_CAP_INA = 1u << 3,
    ROVER_CAP_SERVO_RAIL = 1u << 4
};

/** rover_in_t.tof_status: one bit per sensor, 0 = ok. */
enum {
    ROVER_TOF_BIT_FL = 1u << 0,
    ROVER_TOF_BIT_FR = 1u << 1,
    ROVER_TOF_BIT_CLIFF = 1u << 2
};

/** E.arg for ROVER_EVENT_I2C_ERROR.  5.1 names 0..2; the INA226 shares the
 *  same 400 kHz bus and needs an index of its own to be diagnosable. */
enum {
    ROVER_I2C_INDEX_TOF_FL = 0,
    ROVER_I2C_INDEX_TOF_FR = 1,
    ROVER_I2C_INDEX_TOF_CLIFF = 2,
    ROVER_I2C_INDEX_INA = 3
};

/* Field indices, in wire order, for the frames the core reads and writes. */
enum { ROVER_H_HOST_BOOT_ID = 0 };
enum { ROVER_A_NONCE = 0 };
enum { ROVER_V_V_MM_S = 0, ROVER_V_W_MRAD_S, ROVER_V_FRAME_TTL_MS, ROVER_V_FLAGS };
enum { ROVER_S_MODE = 0 };
enum { ROVER_C_MASK = 0 };
enum { ROVER_P_PI_MONO_US = 0 };
enum {
    ROVER_B_FW_VER = 0,
    ROVER_B_PROTO_VER,
    ROVER_B_CAPS,
    ROVER_B_RESET_REASON,
    ROVER_B_SAFETY_HASH
};
enum {
    ROVER_K_ACK_TYPE = 0,
    ROVER_K_ACK_SEQ,
    ROVER_K_RESULT,
    ROVER_K_REASON,
    ROVER_K_ECHO
};
enum { ROVER_E_EVENT = 0, ROVER_E_ARG, ROVER_E_MCU_US };
enum { ROVER_O_ECHO_PI_MONO_US = 0, ROVER_O_MCU_US };
enum {
    ROVER_T_MCU_US = 0,
    ROVER_T_ACK_SEQ,
    ROVER_T_STATE,
    ROVER_T_CTRL_FLAGS,
    ROVER_T_FAULT,
    ROVER_T_LEFT_TICKS,
    ROVER_T_RIGHT_TICKS,
    ROVER_T_V_MEAS_MM_S,
    ROVER_T_W_MEAS_MRAD_S,
    ROVER_T_V_CMD_MM_S,
    ROVER_T_W_CMD_MRAD_S,
    ROVER_T_VBAT_MV,
    ROVER_T_IMOTOR_MA,
    ROVER_T_TOF_FRONT_MM,
    ROVER_T_TOF_CLIFF_MM,
    ROVER_T_SENSOR_AGE_MS,
    ROVER_T_LOOP_LATE_PCT,
    ROVER_T_RX_DROP,
    ROVER_T_GYRO_Z_MRAD_S,
    ROVER_T_RAILS,
    ROVER_T_MOTION
};

/** One wire field: its name, its integer range, and how it is written.
 *  `hex_width` is -1 for decimal, 0 for minimal-width uppercase hex, and n for
 *  hex zero-padded to n digits -- the widths the golden vectors use. */
typedef struct {
    const char *name;
    int64_t lo;
    int64_t hi;
    int8_t hex_width;
} rover_field_spec_t;

/** One frame type. */
typedef struct {
    char type;
    uint8_t nfields;
    bool down; /**< true for Pi->MCU. */
    const rover_field_spec_t *fields;
} rover_frame_spec_t;

/** A decoded or to-be-encoded frame.  Fields are held in wire order. */
typedef struct {
    char type;
    uint16_t seq;
    uint16_t session;
    uint8_t nfields;
    int64_t field[ROVER_MAX_FIELDS];
} rover_frame_t;

/** The spec for `type`, or NULL if there is no such frame type. */
const rover_frame_spec_t *rover_frame_spec(char type);
/** Index of `name` within `spec`, or -1. */
int rover_frame_field_index(const rover_frame_spec_t *spec, const char *name);

/** CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xor-out. */
uint16_t rover_crc16(const uint8_t *data, size_t n);
/** CRC-32 (the zlib/ISO-HDLC polynomial), used only for B.safety_hash. */
uint32_t rover_crc32(const uint8_t *data, size_t n);
/** CRC-32 over the seven compiled safety constants of 5.1, comma-joined. */
uint32_t rover_safety_hash(void);

/** `(int16_t)(seq - last) > 0` on uint16 counters (A8). */
bool rover_seq_is_newer(uint16_t seq, uint16_t last);
/** frame_ttl_ms is the one field clamping does not apply to: 0 or 50..500. */
bool rover_frame_ttl_ok(int64_t frame_ttl_ms);

/** Render `frame` as `$body*CRC\n` into `out`.
 *  Returns the byte count, or 0 if the frame is malformed or `cap` is short. */
size_t rover_encode_frame(const rover_frame_t *frame, uint8_t *out, size_t cap);

/** Decode one line, with or without its trailing newline.
 *  Returns ROVER_REASON_NONE and fills `out` on success; otherwise the reason
 *  the line was dropped.  Never touches `out` on failure and never traps. */
rover_reason_t rover_decode_line(const uint8_t *line, size_t n, rover_frame_t *out);

/** What T.rx_drop is built from. */
typedef struct {
    uint32_t ok;
    uint32_t bad_crc;
    uint32_t bad_length;
    uint32_t unknown_type;
    uint32_t unsupported_version;
    uint32_t bad_session;
    uint32_t stale_seq;
    uint32_t overlong;
} rover_rx_counters_t;

/** Total frames dropped: everything but `ok`, without double-counting. */
uint32_t rover_rx_dropped(const rover_rx_counters_t *c);

/** Newline framing over a byte stream.
 *  A leading newline is legal and is sent once after every port open, so an
 *  empty line is skipped silently.  A line longer than ROVER_MAX_LINE_BYTES is
 *  dropped to the next newline and counted exactly once, whatever its length,
 *  which is what makes a mid-frame reset harmless. */
typedef struct {
    uint8_t buf[ROVER_MAX_LINE_BYTES + 1];
    uint16_t len;
    bool skipping;
} rover_line_reader_t;

void rover_line_reader_init(rover_line_reader_t *reader);
/** Push one byte.  Returns true when `out`/`reason` describe a complete line:
 *  `*reason == ROVER_REASON_NONE` means `out` holds a valid frame. */
bool rover_line_reader_push(rover_line_reader_t *reader, uint8_t byte,
                            rover_frame_t *out, rover_reason_t *reason);

/* ===================================================================== */
/* Core                                                                  */
/* ===================================================================== */

/** Build-time facts and the constants G2 measures.  `rover_cfg_default()`
 *  fills every field from rover_config.h; nothing here raises a cap. */
typedef struct {
    uint32_t fw_ver;       /**< major<<16 | minor<<8 | patch. */
    uint16_t caps;         /**< B.caps feature bitmask. */
    uint8_t reset_reason;  /**< from the RTC reset reason. */
    bool debug_build;      /**< ctrl_flags b7; a release build clears it (I-18). */
    bool wdt_reboot;       /**< the RTC reset reason was a Task-WDT panic (I-20). */
    bool brownout;         /**< ... or the brownout detector (CONFIG_ESP_BROWNOUT_DET). */
    uint16_t wheel_radius_mm;
    uint16_t track_mm;
    uint16_t ticks_per_rev;
    uint16_t r_pack_mohm;  /**< A25 sag compensation; measured at G2. */
    uint16_t r_motor_mohm; /**< A24 current model; measured at G2. */
    uint16_t k_e_uv_per_mm_s;
} rover_cfg_t;

void rover_cfg_default(rover_cfg_t *cfg);

/** Everything the peripherals report.  One struct, so `firmware/main/` and the
 *  simulator differ only in how they fill it (principle 6). */
typedef struct {
    int32_t left_ticks;
    int32_t right_ticks;
    uint16_t tof_fl_mm;
    uint16_t tof_fr_mm;
    uint16_t tof_cliff_mm;
    uint8_t tof_status; /**< one bit per sensor, 0 = ok. */
    uint8_t bumper_clear;
    uint8_t estop_released;
    uint16_t vbat_mv;
    int16_t imotor_ma; /**< negative = regen. */
    int16_t ntc_c;
    int16_t gyro_z_mrad_s; /**< 32767 = absent. */
} rover_in_t;

/** Everything the peripherals are driven from. */
typedef struct {
    int16_t duty_l_q15;
    int16_t duty_r_q15;
    uint8_t brake; /**< MDD3A brake is both inputs high; PWM zero is not it. */
    uint8_t motor_en;
    uint8_t servo_en;
    uint8_t pi_rail_en;
    uint8_t pi_shutdown_req;
    uint8_t state;
    uint32_t fault;
    uint16_t ctrl_flags;
} rover_out_t;

typedef struct rover_core rover_core_t;

/** Bytes to allocate for one instance. */
size_t rover_core_size(void);

/** Boot DISARMED with the motors braked, mint the banner, start the 50 Hz
 *  telemetry.  `session` is truncated to the uint16 SESS field and 0 is
 *  replaced by 1, because the MCU never mints the wildcard. */
void rover_core_init(rover_core_t *core, const rover_cfg_t *cfg, uint32_t session,
                     uint64_t now_us);
/** Raw UART bytes in.  Nothing here can trap: garbage, a mid-frame reset and a
 *  200-byte line without a newline are all just dropped and counted (I-2). */
void rover_core_feed(rover_core_t *core, const uint8_t *rx, size_t n);
/** One 100 Hz control step.  `now_us` is the MCU's own monotonic clock and is
 *  never compared against anything the host sent (principle 5, I-17). */
void rover_core_step(rover_core_t *core, uint64_t now_us, const rover_in_t *in,
                     rover_out_t *out);
/** Copy queued frames out, whole lines only.  Returns the byte count. */
size_t rover_core_drain_tx(rover_core_t *core, uint8_t *out, size_t cap);

/* Read-only accessors, for `firmware/main/`'s logging and the host tests. */
uint16_t rover_core_session(const rover_core_t *core);
const rover_rx_counters_t *rover_core_counters(const rover_core_t *core);
int16_t rover_core_v_cmd_mm_s(const rover_core_t *core);
int16_t rover_core_w_cmd_mrad_s(const rover_core_t *core);
int16_t rover_core_v_meas_mm_s(const rover_core_t *core);
uint64_t rover_core_ttl_deadline_us(const rover_core_t *core);

/* ===================================================================== */
/* Pure control and safety helpers -- separately testable, no state       */
/* ===================================================================== */

/** Clamp to +-limit.  `*clamped` is set true if the value moved (A5: values
 *  outside a cap are clamped, never rejected, and raise CAP_CLAMPED). */
int32_t rover_clamp_sym(int32_t value, int32_t limit, bool *clamped);

/** Asymmetric slew (4.1): increases in magnitude are bounded by `up_step`,
 *  every decrease uses `down_step`, and a reversal stops at zero first so the
 *  abort ramp can never accelerate the other way. */
int32_t rover_slew(int32_t current, int32_t target, int32_t up_step, int32_t down_step);

/** Differential kinematics from body velocity (A6). */
void rover_body_to_wheels(int32_t v_mm_s, int32_t w_mrad_s, uint16_t track_mm,
                          int32_t *left_mm_s, int32_t *right_mm_s);
void rover_wheels_to_body(int32_t left_mm_s, int32_t right_mm_s, uint16_t track_mm,
                          int32_t *v_mm_s, int32_t *w_mrad_s);
/** Encoder delta to wheel speed.  `dt_us` of 0 returns 0. */
int32_t rover_ticks_to_mm_s(int32_t dticks, uint16_t ticks_per_rev,
                            uint16_t wheel_radius_mm, uint32_t dt_us);

/** One wheel's PI controller: integer state, anti-windup at full duty. */
typedef struct {
    int32_t integral;
} rover_pi_t;

void rover_pi_reset(rover_pi_t *pi);
int16_t rover_pi_step(rover_pi_t *pi, int32_t target_mm_s, int32_t meas_mm_s);

/** The A21 slow-zone law: the highest forward speed legal at `front_mm`.
 *  ROVER_TOF_NO_TARGET_MM is a clear path, not a fault (I-16). */
int32_t rover_slow_zone_v_cap(uint32_t front_mm);

#ifdef __cplusplus
}
#endif

#endif /* ROVER_CORE_H */
