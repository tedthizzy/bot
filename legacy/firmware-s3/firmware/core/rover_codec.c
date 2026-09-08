/* The line codec of ARCHITECTURE 5.1.
 *
 *   frame ::= "$" body "*" CRC "\n"
 *   body  ::= TYPE "," VER "," SEQ "," SESS [ "," FIELD ]*
 *
 * Every field is a signed decimal integer or an unsigned uppercase hex word,
 * which is what makes "\n" a deterministic resynchronisation point and is why
 * A5 could delete COBS and the free-text `L` frame.  One table drives encode,
 * decode and the host tests' lookup by field name, so the three cannot drift.
 *
 * Nothing here returns an error to a caller's loop by trapping: a corrupted
 * UART byte must not take down the 50 Hz telemetry task.
 */
#include "rover_internal.h"

#define U8_LO 0
#define U8_HI 0xFF
#define U16_HI 0xFFFF
#define U32_HI 0xFFFFFFFFLL
/* int64_t is the widest field store, so a u64 field is range-checked at
 * INT64_MAX.  Both u64 fields are microsecond counters; 292 000 years of
 * uptime is not a case the protocol needs. */
#define U64_HI INT64_MAX
#define I16_LO (-32768)
#define I16_HI 32767
#define I32_LO (-2147483648LL)
#define I32_HI 2147483647LL

#define DEC (-1)
#define HEXMIN 0

static const rover_field_spec_t f_hello[] = {
    {"host_boot_id", 0, U32_HI, DEC},
};
static const rover_field_spec_t f_arm[] = {
    {"nonce", 0, U32_HI, DEC},
};
static const rover_field_spec_t f_velocity[] = {
    {"v_mm_s", I16_LO, I16_HI, DEC},
    {"w_mrad_s", I16_LO, I16_HI, DEC},
    {"frame_ttl_ms", 0, U16_HI, DEC},
    {"flags", U8_LO, U8_HI, DEC},
};
static const rover_field_spec_t f_stop[] = {
    {"mode", U8_LO, U8_HI, DEC},
};
static const rover_field_spec_t f_clear[] = {
    {"mask", 0, U32_HI, 4},
};
static const rover_field_spec_t f_ping[] = {
    {"pi_mono_us", 0, U64_HI, DEC},
};
static const rover_field_spec_t f_boot[] = {
    {"fw_ver", 0, U32_HI, DEC},
    {"proto_ver", U8_LO, U8_HI, DEC},
    {"caps", 0, U16_HI, 4},
    {"reset_reason", U8_LO, U8_HI, DEC},
    {"safety_hash", 0, U32_HI, DEC},
};
static const rover_field_spec_t f_telemetry[] = {
    {"mcu_us", 0, U64_HI, DEC},
    {"ack_seq", 0, U16_HI, DEC},
    {"state", U8_LO, U8_HI, DEC},
    {"ctrl_flags", 0, U16_HI, HEXMIN},
    {"fault", 0, U32_HI, HEXMIN},
    {"left_ticks", I32_LO, I32_HI, DEC},
    {"right_ticks", I32_LO, I32_HI, DEC},
    {"v_meas_mm_s", I16_LO, I16_HI, DEC},
    {"w_meas_mrad_s", I16_LO, I16_HI, DEC},
    {"v_cmd_mm_s", I16_LO, I16_HI, DEC},
    {"w_cmd_mrad_s", I16_LO, I16_HI, DEC},
    {"vbat_mv", 0, U16_HI, DEC},
    {"imotor_ma", I16_LO, I16_HI, DEC},
    {"tof_front_mm", 0, U16_HI, DEC},
    {"tof_cliff_mm", 0, U16_HI, DEC},
    {"sensor_age_ms", U8_LO, U8_HI, DEC},
    {"loop_late_pct", U8_LO, U8_HI, DEC},
    {"rx_drop", 0, U16_HI, DEC},
    {"gyro_z_mrad_s", I16_LO, I16_HI, DEC},
    {"rails", U8_LO, U8_HI, DEC},
    {"motion", U8_LO, U8_HI, DEC},
};
static const rover_field_spec_t f_ack[] = {
    {"ack_type", U8_LO, U8_HI, DEC},
    {"ack_seq", 0, U16_HI, DEC},
    {"result", U8_LO, U8_HI, DEC},
    {"reason", U8_LO, U8_HI, DEC},
    {"echo", 0, U32_HI, DEC},
};
static const rover_field_spec_t f_event[] = {
    {"event", U8_LO, U8_HI, DEC},
    {"arg", I32_LO, I32_HI, DEC},
    {"mcu_us", 0, U64_HI, DEC},
};
static const rover_field_spec_t f_pong[] = {
    {"echo_pi_mono_us", 0, U64_HI, DEC},
    {"mcu_us", 0, U64_HI, DEC},
};

#define SPEC(letter, arr, is_down)                                             \
    {letter, (uint8_t)(sizeof(arr) / sizeof((arr)[0])), is_down, arr}

static const rover_frame_spec_t k_specs[] = {
    SPEC('H', f_hello, true),
    SPEC('A', f_arm, true),
    {'D', 0, true, NULL},
    SPEC('V', f_velocity, true),
    SPEC('S', f_stop, true),
    SPEC('C', f_clear, true),
    SPEC('P', f_ping, true),
    SPEC('B', f_boot, false),
    SPEC('T', f_telemetry, false),
    SPEC('K', f_ack, false),
    SPEC('E', f_event, false),
    SPEC('O', f_pong, false),
};

const rover_frame_spec_t *rover_frame_spec(char type)
{
    for (size_t i = 0; i < sizeof k_specs / sizeof k_specs[0]; i++) {
        if (k_specs[i].type == type) {
            return &k_specs[i];
        }
    }
    return NULL;
}

int rover_frame_field_index(const rover_frame_spec_t *spec, const char *name)
{
    if (spec == NULL) {
        return -1;
    }
    for (uint8_t i = 0; i < spec->nfields; i++) {
        const char *a = spec->fields[i].name;
        const char *b = name;
        while (*a != '\0' && *a == *b) {
            a++;
            b++;
        }
        if (*a == '\0' && *b == '\0') {
            return (int)i;
        }
    }
    return -1;
}

bool rover_seq_is_newer(uint16_t seq, uint16_t last)
{
    return (int16_t)(uint16_t)(seq - last) > 0;
}

bool rover_frame_ttl_ok(int64_t frame_ttl_ms)
{
    return frame_ttl_ms == ROVER_FRAME_TTL_IMMEDIATE ||
           (frame_ttl_ms >= ROVER_FRAME_TTL_MIN_MS &&
            frame_ttl_ms <= ROVER_FRAME_TTL_MAX_MS);
}

uint32_t rover_rx_dropped(const rover_rx_counters_t *c)
{
    return c->bad_crc + c->bad_length + c->unknown_type + c->unsupported_version +
           c->bad_session + c->stale_seq;
}

/* ---- parsing ----------------------------------------------------------- */

/** Decimal, optional leading '-', digits only, non-empty, no overflow. */
static bool parse_dec(const uint8_t *tok, size_t n, int64_t *out)
{
    bool neg = (n > 0 && tok[0] == '-');
    size_t i = neg ? 1u : 0u;
    if (i >= n) {
        return false;
    }
    uint64_t value = 0;
    for (; i < n; i++) {
        if (tok[i] < '0' || tok[i] > '9') {
            return false;
        }
        if (value > (uint64_t)INT64_MAX / 10u) {
            return false;
        }
        value = value * 10u + (uint64_t)(tok[i] - '0');
        if (value > (uint64_t)INT64_MAX) {
            return false;
        }
    }
    *out = neg ? -(int64_t)value : (int64_t)value;
    return true;
}

/** Unsigned hex, 1 to 8 uppercase digits, no 0x prefix. */
static bool parse_hex(const uint8_t *tok, size_t n, int64_t *out)
{
    if (n < 1 || n > 8) {
        return false;
    }
    uint64_t value = 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t c = tok[i];
        uint32_t digit;
        if (c >= '0' && c <= '9') {
            digit = (uint32_t)(c - '0');
        } else if (c >= 'A' && c <= 'F') {
            digit = (uint32_t)(c - 'A') + 10u;
        } else {
            return false;
        }
        value = (value << 4) | digit;
    }
    *out = (int64_t)value;
    return true;
}

/* ---- encode ------------------------------------------------------------ */

size_t rover_encode_frame(const rover_frame_t *frame, uint8_t *out, size_t cap)
{
    const rover_frame_spec_t *spec = rover_frame_spec(frame->type);
    if (spec == NULL || frame->nfields != spec->nfields) {
        return 0;
    }
    if (frame->session == ROVER_SESSION_WILDCARD && frame->type != 'H') {
        return 0; /* session 0 is the H wildcard and nothing else may send it. */
    }

    char body[ROVER_MAX_LINE_BYTES];
    size_t n = 0;
    body[n++] = frame->type;
    body[n++] = ',';
    n += rover_fmt_udec(ROVER_PROTO_VER, body + n);
    body[n++] = ',';
    n += rover_fmt_udec(frame->seq, body + n);
    body[n++] = ',';
    n += rover_fmt_udec(frame->session, body + n);
    for (uint8_t i = 0; i < spec->nfields; i++) {
        const rover_field_spec_t *fs = &spec->fields[i];
        int64_t value = frame->field[i];
        if (value < fs->lo || value > fs->hi) {
            return 0;
        }
        if (n + 22 > sizeof body) {
            return 0;
        }
        body[n++] = ',';
        n += (fs->hex_width < 0) ? rover_fmt_dec(value, body + n)
                                 : rover_fmt_hex((uint64_t)value, fs->hex_width,
                                                 body + n);
    }
    if (n > ROVER_MAX_LINE_BYTES - 6 || n + 7 > cap) {
        return 0;
    }

    uint16_t crc = rover_crc16((const uint8_t *)body, n);
    size_t out_n = 0;
    out[out_n++] = '$';
    for (size_t i = 0; i < n; i++) {
        out[out_n++] = (uint8_t)body[i];
    }
    out[out_n++] = '*';
    out_n += rover_fmt_hex(crc, 4, (char *)out + out_n);
    out[out_n++] = '\n';
    return out_n;
}

/* ---- decode ------------------------------------------------------------ */

rover_reason_t rover_decode_line(const uint8_t *line, size_t n, rover_frame_t *out)
{
    if (n > 0 && line[n - 1] == '\n') {
        n--;
    }
    if (n > 0 && line[n - 1] == '\r') {
        n--;
    }
    if (n > ROVER_MAX_LINE_BYTES) {
        return ROVER_REASON_BAD_LENGTH;
    }
    if (n == 0 || line[0] != '$') {
        return ROVER_REASON_BAD_LENGTH;
    }

    /* The CRC is the last '*'-delimited word; the body is everything between
     * '$' and it, exclusive of both. */
    size_t star = 0;
    bool found = false;
    for (size_t i = n; i > 1; i--) {
        if (line[i - 1] == '*') {
            star = i - 1;
            found = true;
            break;
        }
    }
    if (!found) {
        return ROVER_REASON_BAD_LENGTH;
    }
    const uint8_t *body = line + 1;
    size_t body_n = star - 1;
    const uint8_t *crc_text = line + star + 1;
    size_t crc_n = n - star - 1;
    int64_t crc_value = 0;
    if (crc_n != 4 || !parse_hex(crc_text, crc_n, &crc_value)) {
        return ROVER_REASON_BAD_LENGTH;
    }
    if ((uint16_t)crc_value != rover_crc16(body, body_n)) {
        return ROVER_REASON_BAD_CRC;
    }

    /* Split the body on commas.  4 header parts plus at most ROVER_MAX_FIELDS.
     * Anything past that is counted but not stored, so an over-long field list
     * still reaches the type, version and session checks below and reports the
     * same reason the Python codec does -- the two sides have to increment the
     * same RxCounters field for the same wire event, or T.rx_drop and robotd's
     * link stats cannot be compared. */
    const uint8_t *part[4 + ROVER_MAX_FIELDS];
    size_t part_n[4 + ROVER_MAX_FIELDS];
    const size_t part_cap = sizeof part / sizeof part[0];
    size_t parts = 0;
    size_t start = 0;
    for (size_t i = 0; i <= body_n; i++) {
        if (i == body_n || body[i] == ',') {
            if (parts < part_cap) {
                part[parts] = body + start;
                part_n[parts] = i - start;
            }
            parts++;
            start = i + 1;
        }
    }
    if (parts < 4) {
        return ROVER_REASON_BAD_LENGTH;
    }

    if (part_n[0] != 1) {
        return ROVER_REASON_UNKNOWN_TYPE;
    }
    const rover_frame_spec_t *spec = rover_frame_spec((char)part[0][0]);
    if (spec == NULL) {
        return ROVER_REASON_UNKNOWN_TYPE;
    }
    int64_t ver = 0;
    if (!parse_dec(part[1], part_n[1], &ver) || ver != ROVER_PROTO_VER) {
        return ROVER_REASON_UNSUPPORTED_VERSION;
    }
    int64_t seq = 0;
    int64_t session = 0;
    if (!parse_dec(part[2], part_n[2], &seq) || seq < 0 || seq > U16_HI) {
        return ROVER_REASON_BAD_LENGTH;
    }
    if (!parse_dec(part[3], part_n[3], &session) || session < 0 || session > U16_HI) {
        return ROVER_REASON_BAD_LENGTH;
    }
    if (session == ROVER_SESSION_WILDCARD && spec->type != 'H') {
        return ROVER_REASON_BAD_SESSION;
    }
    if (parts > part_cap || parts - 4 != spec->nfields) {
        return ROVER_REASON_BAD_LENGTH;
    }

    rover_frame_t frame;
    frame.type = spec->type;
    frame.seq = (uint16_t)seq;
    frame.session = (uint16_t)session;
    frame.nfields = spec->nfields;
    for (uint8_t i = 0; i < spec->nfields; i++) {
        const rover_field_spec_t *fs = &spec->fields[i];
        int64_t value = 0;
        bool ok = (fs->hex_width < 0) ? parse_dec(part[4 + i], part_n[4 + i], &value)
                                      : parse_hex(part[4 + i], part_n[4 + i], &value);
        if (!ok || value < fs->lo || value > fs->hi) {
            return ROVER_REASON_BAD_LENGTH;
        }
        frame.field[i] = value;
    }
    for (uint8_t i = spec->nfields; i < ROVER_MAX_FIELDS; i++) {
        frame.field[i] = 0;
    }
    *out = frame;
    return ROVER_REASON_NONE;
}

/* ---- newline framing --------------------------------------------------- */

void rover_line_reader_init(rover_line_reader_t *reader)
{
    reader->len = 0;
    reader->skipping = false;
}

bool rover_line_reader_push(rover_line_reader_t *reader, uint8_t byte,
                            rover_frame_t *out, rover_reason_t *reason)
{
    if (byte == '\n') {
        uint16_t len = reader->len;
        reader->len = 0;
        if (reader->skipping) {
            /* Tail of an over-length line, already counted once. */
            reader->skipping = false;
            return false;
        }
        if (len == 0 || (len == 1 && reader->buf[0] == '\r')) {
            return false; /* a leading newline is legal after every port open */
        }
        *reason = rover_decode_line(reader->buf, len, out);
        return true;
    }
    if (reader->len < ROVER_MAX_LINE_BYTES) {
        reader->buf[reader->len++] = byte;
        return false;
    }
    /* One byte past the longest legal line: drop to the next newline and
     * count it exactly once, whatever the length of the garbage (5.1). */
    reader->len = 0;
    if (!reader->skipping) {
        reader->skipping = true;
        *reason = ROVER_REASON_BAD_LENGTH;
        out->type = 0;
        out->nfields = 0;
        return true;
    }
    return false;
}
