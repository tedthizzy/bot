/* CRC-16/CCITT-FALSE for the line protocol, CRC-32 for B.safety_hash.
 *
 * Both are bitwise: the link runs at 5.4% of 921600 (5.1) and the control step
 * has 10 ms, so a 256-entry table would buy nothing and cost RAM the S3 wants
 * for the frame buffers.
 */
#include "rover_internal.h"

uint16_t rover_crc16(const uint8_t *data, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint32_t rover_crc32(const uint8_t *data, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
        }
    }
    return ~crc;
}

uint32_t rover_safety_hash(void)
{
    /* The seven constants of 5.1 in their fixed order, comma-joined.  Built
     * from the macros rather than from a literal string, so the banner cannot
     * drift from the header the compiled behaviour actually uses. */
    static const int32_t values[] = {
        ROVER_TOF_STOP_MM,          ROVER_TOF_SLOW_MM,
        ROVER_SLOW_ZONE_W_MRAD_S,   ROVER_TOF_TIMING_BUDGET_MS,
        ROVER_TOF_INTER_PERIOD_MS,  ROVER_TOF_POLL_HZ,
        ROVER_CLIFF_DELTA_MM
    };
    char body[64];
    size_t n = 0;
    for (size_t i = 0; i < sizeof values / sizeof values[0]; i++) {
        if (i > 0) {
            body[n++] = ',';
        }
        n += rover_fmt_dec(values[i], body + n);
    }
    return rover_crc32((const uint8_t *)body, n);
}
