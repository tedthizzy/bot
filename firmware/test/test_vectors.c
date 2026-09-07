/* The C codec against tests/contract/serial_vectors.jsonl.
 *
 * The vectors are the shared contract with `rover_contracts.serial_codec`:
 * fifteen frames verbatim from ARCHITECTURE 5.1, ten isolated rejections, the
 * CRC-16 check value and the seven-constant safety hash.  Firmware and the
 * Python codec are written by different people against one document, so the
 * file -- not either implementation -- is the arbiter.
 *
 * The JSON reader below is deliberately small: the vectors are flat objects of
 * strings, integers and integer-valued objects, and pulling a parser into the
 * firmware tree to read them would be the larger risk.
 */
#include <stdio.h>
#include <stdlib.h>

#include "test_util.h"

#ifndef ROVER_VECTORS_PATH
#define ROVER_VECTORS_PATH "tests/contract/serial_vectors.jsonl"
#endif

/* ---- a very small JSON reader ------------------------------------------ */

static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
        p++;
    }
    return p;
}

/** `p` at the opening quote; returns the byte after the closing quote. */
static const char *skip_string(const char *p)
{
    p++;
    while (*p != '\0' && *p != '"') {
        p += (*p == '\\' && p[1] != '\0') ? 2 : 1;
    }
    return *p == '"' ? p + 1 : p;
}

static const char *skip_value(const char *p)
{
    if (*p == '"') {
        return skip_string(p);
    }
    if (*p == '{' || *p == '[') {
        int depth = 0;
        while (*p != '\0') {
            if (*p == '"') {
                p = skip_string(p);
                continue;
            }
            if (*p == '{' || *p == '[') {
                depth++;
            } else if (*p == '}' || *p == ']') {
                if (--depth == 0) {
                    return p + 1;
                }
            }
            p++;
        }
        return p;
    }
    while (*p != '\0' && *p != ',' && *p != '}' && *p != ']') {
        p++;
    }
    return p;
}

static bool key_is(const char *start, const char *end, const char *key)
{
    while (start < end && *key != '\0' && *start == *key) {
        start++;
        key++;
    }
    return start == end && *key == '\0';
}

/** The value for `key` at the top level of `obj`, or NULL. */
static const char *json_find(const char *obj, const char *key)
{
    const char *p = skip_ws(obj);
    if (*p != '{') {
        return NULL;
    }
    p = skip_ws(p + 1);
    while (*p == '"') {
        const char *key_start = p + 1;
        const char *key_end = skip_string(p) - 1;
        p = skip_ws(skip_string(p));
        if (*p != ':') {
            return NULL;
        }
        p = skip_ws(p + 1);
        if (key_is(key_start, key_end, key)) {
            return p;
        }
        p = skip_ws(skip_value(p));
        if (*p != ',') {
            return NULL;
        }
        p = skip_ws(p + 1);
    }
    return NULL;
}

/** Decode a JSON string value into `out`; returns the byte count, or -1. */
static long json_str(const char *value, char *out, size_t cap)
{
    if (value == NULL || *value != '"') {
        return -1;
    }
    size_t n = 0;
    const char *p = value + 1;
    while (*p != '\0' && *p != '"') {
        char c = *p++;
        if (c == '\\') {
            char esc = *p++;
            switch (esc) {
            case 'n': c = '\n'; break;
            case 'r': c = '\r'; break;
            case 't': c = '\t'; break;
            case 'b': c = '\b'; break;
            case 'f': c = '\f'; break;
            case 'u':
                /* No vector needs a non-ASCII escape; consume and substitute. */
                for (int i = 0; i < 4 && *p != '\0'; i++) {
                    p++;
                }
                c = '?';
                break;
            default: c = esc; break;
            }
        }
        if (n + 1 >= cap) {
            return -1;
        }
        out[n++] = c;
    }
    out[n] = '\0';
    return (long)n;
}

static bool json_i64(const char *value, int64_t *out)
{
    if (value == NULL) {
        return false;
    }
    bool neg = *value == '-';
    const char *p = neg ? value + 1 : value;
    if (*p < '0' || *p > '9') {
        return false;
    }
    uint64_t v = 0;
    while (*p >= '0' && *p <= '9') {
        v = v * 10u + (uint64_t)(*p++ - '0');
    }
    *out = neg ? -(int64_t)v : (int64_t)v;
    return true;
}

/** Walk an object whose values are all integers.  `*cursor` starts at the
 *  object's '{' and is advanced; returns false at the closing brace. */
static bool json_next_int_member(const char **cursor, char *name, size_t cap,
                                 int64_t *value)
{
    const char *p = skip_ws(*cursor);
    if (*p == '{' || *p == ',') {
        p = skip_ws(p + 1);
    }
    if (*p != '"') {
        return false;
    }
    long n = json_str(p, name, cap);
    if (n < 0) {
        return false;
    }
    p = skip_ws(skip_string(p));
    if (*p != ':') {
        return false;
    }
    p = skip_ws(p + 1);
    if (!json_i64(p, value)) {
        return false;
    }
    *cursor = skip_value(p);
    return true;
}

/* ---- the cases --------------------------------------------------------- */

static void check_frame_vector(const char *obj, const char *name)
{
    char text[512];
    char body[512];
    if (json_str(json_find(obj, "line"), text, sizeof text) < 0 ||
        json_str(json_find(obj, "body"), body, sizeof body) < 0) {
        rover_test_fail(__FILE__, __LINE__, "%s: no line/body", name);
        return;
    }
    size_t line_n = 0;
    while (text[line_n] != '\0') {
        line_n++;
    }
    size_t body_n = 0;
    while (body[body_n] != '\0') {
        body_n++;
    }

    int64_t crc_u16 = 0;
    if (json_i64(json_find(obj, "crc_u16"), &crc_u16)) {
        CHECK_EQ(rover_crc16((const uint8_t *)body, body_n), crc_u16);
    }

    rover_frame_t frame;
    rover_reason_t reason =
        rover_decode_line((const uint8_t *)text, line_n, &frame);
    if (reason != ROVER_REASON_NONE) {
        rover_test_fail(__FILE__, __LINE__, "%s: decode rejected with reason %d",
                        name, (int)reason);
        return;
    }

    char type[8];
    int64_t ver = 0;
    int64_t seq = 0;
    int64_t session = 0;
    if (json_str(json_find(obj, "type"), type, sizeof type) == 1) {
        CHECK_EQ(frame.type, type[0]);
    }
    if (json_i64(json_find(obj, "ver"), &ver)) {
        CHECK_EQ(ver, ROVER_PROTO_VER);
    }
    if (json_i64(json_find(obj, "seq"), &seq)) {
        CHECK_EQ(frame.seq, seq);
    }
    if (json_i64(json_find(obj, "session"), &session)) {
        CHECK_EQ(frame.session, session);
    }

    const rover_frame_spec_t *spec = rover_frame_spec(frame.type);
    CHECK(spec != NULL);
    if (spec == NULL) {
        return;
    }

    const char *fields = json_find(obj, "fields");
    if (fields != NULL) {
        const char *cursor = fields;
        char field_name[64];
        int64_t value = 0;
        int seen = 0;
        while (json_next_int_member(&cursor, field_name, sizeof field_name, &value)) {
            int index = rover_frame_field_index(spec, field_name);
            if (index < 0) {
                rover_test_fail(__FILE__, __LINE__, "%s: no field %s on %c", name,
                                field_name, frame.type);
                continue;
            }
            CHECK_EQ(frame.field[index], value);
            seen++;
        }
        CHECK_EQ(seen, spec->nfields);
    }

    /* hex_fields carries the rendered width, which is what makes re-encoding
     * byte-exact rather than merely equivalent. */
    const char *hex_fields = json_find(obj, "hex_fields");
    if (hex_fields != NULL) {
        const char *cursor = hex_fields;
        char field_name[64];
        int64_t width = 0;
        while (json_next_int_member(&cursor, field_name, sizeof field_name, &width)) {
            int index = rover_frame_field_index(spec, field_name);
            if (index < 0) {
                rover_test_fail(__FILE__, __LINE__, "%s: no hex field %s", name,
                                field_name);
                continue;
            }
            CHECK_EQ(spec->fields[index].hex_width, width);
        }
    }

    uint8_t encoded[ROVER_MAX_LINE_BYTES + 8];
    size_t encoded_n = rover_encode_frame(&frame, encoded, sizeof encoded);
    CHECK_EQ(encoded_n, line_n);
    if (encoded_n == line_n) {
        for (size_t i = 0; i < line_n; i++) {
            if (encoded[i] != (uint8_t)text[i]) {
                rover_test_fail(__FILE__, __LINE__,
                                "%s: re-encode differs at byte %zu", name, i);
                break;
            }
        }
    }
}

static void check_reject_vector(const char *obj, const char *name)
{
    char text[512];
    int64_t reason_code = 0;
    if (json_str(json_find(obj, "line"), text, sizeof text) < 0 ||
        !json_i64(json_find(obj, "reason_code"), &reason_code)) {
        rover_test_fail(__FILE__, __LINE__, "%s: no line/reason_code", name);
        return;
    }
    size_t n = 0;
    while (text[n] != '\0') {
        n++;
    }
    rover_frame_t frame;
    rover_reason_t reason = rover_decode_line((const uint8_t *)text, n, &frame);
    if ((int)reason != (int)reason_code) {
        rover_test_fail(__FILE__, __LINE__, "%s: got reason %d, want %lld", name,
                        (int)reason, (long long)reason_code);
    }
}

void test_golden_vectors_agree_with_the_python_codec(void)
{
    FILE *file = fopen(ROVER_VECTORS_PATH, "rb");
    if (file == NULL) {
        rover_test_fail(__FILE__, __LINE__, "cannot open %s", ROVER_VECTORS_PATH);
        return;
    }
    char *line = malloc(65536);
    if (line == NULL) {
        fclose(file);
        rover_test_fail(__FILE__, __LINE__, "out of memory");
        return;
    }

    int frames = 0;
    int rejects = 0;
    int crc_checks = 0;
    int hash_checks = 0;
    while (fgets(line, 65536, file) != NULL) {
        const char *start = skip_ws(line);
        if (*start != '{') {
            continue;
        }
        char kind[32];
        char name[64];
        if (json_str(json_find(start, "kind"), kind, sizeof kind) < 0) {
            continue;
        }
        if (json_str(json_find(start, "name"), name, sizeof name) < 0) {
            name[0] = '\0';
        }
        if (rover_test_streq(kind, "frame")) {
            check_frame_vector(start, name);
            frames++;
        } else if (rover_test_streq(kind, "reject")) {
            check_reject_vector(start, name);
            rejects++;
        } else if (rover_test_streq(kind, "crc")) {
            char input[64];
            int64_t crc_u16 = 0;
            if (json_str(json_find(start, "input"), input, sizeof input) >= 0 &&
                json_i64(json_find(start, "crc_u16"), &crc_u16)) {
                size_t n = 0;
                while (input[n] != '\0') {
                    n++;
                }
                CHECK_EQ(rover_crc16((const uint8_t *)input, n), crc_u16);
                crc_checks++;
            }
        } else if (rover_test_streq(kind, "safety_hash")) {
            char input[128];
            int64_t crc32 = 0;
            if (json_str(json_find(start, "input"), input, sizeof input) >= 0 &&
                json_i64(json_find(start, "crc32"), &crc32)) {
                size_t n = 0;
                while (input[n] != '\0') {
                    n++;
                }
                CHECK_EQ(rover_crc32((const uint8_t *)input, n), crc32);
                /* And the compiled constants themselves: this is the assertion
                 * deploy/preflight.sh makes against the B banner. */
                CHECK_EQ(rover_safety_hash(), crc32);
                hash_checks++;
            }
        }
    }
    free(line);
    fclose(file);

    CHECK_EQ(frames, 16);
    CHECK_EQ(rejects, 12);
    CHECK_EQ(crc_checks, 1);
    CHECK_EQ(hash_checks, 1);
}

/* The safety_hash vector pins seven of the compiled constants.  These are the
 * rest: every number ARCHITECTURE states for a bound no frame may raise (I-4).
 * Without them the cap tests above would only prove that clamping happens at
 * whatever `rover_config.h` currently says, which is not what A21's stopping
 * distances were derived at.
 */
void test_compiled_caps_are_the_numbers_architecture_states(void)
{
    CHECK_EQ(ROVER_PROTO_VER, 2);          /* 5.1 */
    CHECK_EQ(ROVER_MAX_LINE_BYTES, 200);
    CHECK_EQ(ROVER_MAX_V_MM_S, 300);       /* 5.1 V ranges, the speed A21 assumes */
    CHECK_EQ(ROVER_MAX_W_MRAD_S, 1200);
    CHECK_EQ(ROVER_FRAME_TTL_MIN_MS, 50);
    CHECK_EQ(ROVER_FRAME_TTL_MAX_MS, 500);
    CHECK_EQ(ROVER_TTL_DISARM_MS, 5000);   /* T1 */
    CHECK_EQ(ROVER_SLOW_ZONE_V_CAP_MM_S, 150); /* A21 */
    CHECK_EQ(ROVER_REVERSE_CLAMP_MM_S, 150);   /* I-5: reverse is unsensed */
    CHECK_EQ(ROVER_TOF_STOP_SAMPLES, 2);
    CHECK_EQ(ROVER_TOF_CLEAR_SAMPLES, 5);
    CHECK_EQ(ROVER_CLIFF_SAMPLES, 50);
    CHECK_EQ(ROVER_ACCEL_MM_S2, 500);      /* [limits] accel_mps2 = 0.5 */
    CHECK_EQ(ROVER_ALPHA_MRAD_S2, 1000);   /* [limits] alpha_radps2 = 1.0 */
    CHECK_EQ(ROVER_ABORT_DECEL_MM_S2, 2000); /* 4.1 */
    CHECK_EQ(ROVER_CTRL_HZ, 100);
    CHECK_EQ(ROVER_TELEM_HZ, 50);
    CHECK_EQ(ROVER_STALL_HARD_MS, 200);    /* A24, so I-10's 250 ms holds */
    CHECK_EQ(ROVER_STALL_SLIP_MS, 500);
    CHECK_EQ(ROVER_ENC_IMPLAUS_CYCLES, 20);
    CHECK_EQ(ROVER_I2T_KNEE_MA, 3000);     /* A24 */
    CHECK_EQ(ROVER_I2T_LIMIT_MA2_MS, 6000000000ULL);
    CHECK_EQ(ROVER_VBAT_WARN_MV, 10500);   /* A25 */
    CHECK_EQ(ROVER_VBAT_STOP_MV, 9900);
    CHECK_EQ(ROVER_VBAT_DISABLE_MV, 9600);
    CHECK_EQ(ROVER_VBAT_DEBOUNCE_MS, 10000);
    CHECK_EQ(ROVER_OBSTACLE_ESCALATE_S, 30); /* [safety] obstacle_escalate_s */
    CHECK_EQ(ROVER_FW_VER, 256);           /* 0.1.0 packed, the B banner vector */

    /* And the three fault classes partition the twenty-one defined bits: the
     * twenty of 5.1 plus OBSTACLE_LATCHED, which is what escalation raises. */
    CHECK_EQ(ROVER_FAULT_OBSTACLE_LATCHED, 0x100000u);
    CHECK_EQ(ROVER_FAULT_OBSTACLE_LATCHED & ROVER_FAULTS_LATCHED,
             ROVER_FAULT_OBSTACLE_LATCHED);
    CHECK_EQ(ROVER_FAULTS_ADVISORY & ROVER_FAULTS_OBSTACLE, 0);
    CHECK_EQ(ROVER_FAULTS_ADVISORY & ROVER_FAULTS_LATCHED, 0);
    CHECK_EQ(ROVER_FAULTS_OBSTACLE & ROVER_FAULTS_LATCHED, 0);
    CHECK_EQ(ROVER_FAULTS_ADVISORY | ROVER_FAULTS_OBSTACLE | ROVER_FAULTS_LATCHED,
             0x001FFFFF);
}
