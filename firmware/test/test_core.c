/* The safety cases of ARCHITECTURE 8 that a MacBook can score: I-1 to I-5,
 * I-16 and the arm/session half of I-3, driven against a first-order wheel
 * plant so the same code that runs on the S3 is the code under test (A3).
 *
 * Each case is named after the invariant it stands for and prints one line.
 */
#include <stdlib.h>

#include "test_util.h"

#define SIM_MAX_FRAMES 512
#define PLANT_FULL_DUTY_MM_S 450 /* free-running wheel speed at full duty */
#define PLANT_TAU_US 150000
#define PLANT_BRAKE_TAU_US 30000
#define WHEEL_CIRCUMFERENCE_UM 282743

typedef struct {
    rover_core_t *core;
    rover_cfg_t cfg;
    rover_in_t in;
    rover_out_t out;
    uint64_t now_us;
    uint16_t session;
    uint16_t down_seq;
    /* plant */
    int64_t wheel_um_s[2];
    int64_t wheel_pos_um[2];
    bool hold_left;
    /* what the core has sent since the last reset */
    rover_frame_t frames[SIM_MAX_FRAMES];
    int nframes;
} sim_t;

static int32_t iabs32(int32_t v)
{
    return v < 0 ? -v : v;
}

/* ---- harness ----------------------------------------------------------- */

static void sim_collect(sim_t *s)
{
    uint8_t buf[4096];
    size_t n = rover_core_drain_tx(s->core, buf, sizeof buf);
    size_t start = 0;
    for (size_t i = 0; i < n; i++) {
        if (buf[i] != '\n') {
            continue;
        }
        rover_frame_t frame;
        if (rover_decode_line(buf + start, i - start + 1, &frame) ==
                ROVER_REASON_NONE &&
            s->nframes < SIM_MAX_FRAMES) {
            s->frames[s->nframes++] = frame;
        }
        start = i + 1;
    }
}

static void sim_frames_reset(sim_t *s)
{
    s->nframes = 0;
}

static sim_t *sim_new(uint16_t session)
{
    sim_t *s = calloc(1, sizeof *s);
    s->core = malloc(rover_core_size());
    s->session = session;
    rover_cfg_default(&s->cfg);
    s->in.tof_fl_mm = 2000;
    s->in.tof_fr_mm = 2000;
    s->in.tof_cliff_mm = 98;
    s->in.bumper_clear = 1;
    s->in.estop_released = 1;
    s->in.vbat_mv = 11620;
    s->in.ntc_c = 30;
    s->in.gyro_z_mrad_s = 32767;
    s->now_us = 1000000;
    rover_core_init(s->core, &s->cfg, session, s->now_us);
    sim_collect(s);
    return s;
}

static void sim_free(sim_t *s)
{
    free(s->core);
    free(s);
}

/** One control period of a first-order wheel model, integrating encoder ticks
 *  the way PCNT does on the real controller. */
static void sim_plant(sim_t *s, uint32_t dt_us)
{
    const uint32_t tau = s->out.brake ? PLANT_BRAKE_TAU_US : PLANT_TAU_US;
    const int16_t duty[2] = {s->out.duty_l_q15, s->out.duty_r_q15};
    for (int i = 0; i < 2; i++) {
        int64_t target_um_s =
            s->out.brake
                ? 0
                : ((int64_t)duty[i] * PLANT_FULL_DUTY_MM_S * 1000) / ROVER_DUTY_MAX_Q15;
        if (i == 0 && s->hold_left) {
            target_um_s = 0;
            s->wheel_um_s[i] = 0;
        } else {
            s->wheel_um_s[i] += ((target_um_s - s->wheel_um_s[i]) * dt_us) / tau;
        }
        s->wheel_pos_um[i] += (s->wheel_um_s[i] * dt_us) / 1000000;
    }
    s->in.left_ticks = (int32_t)((s->wheel_pos_um[0] * ROVER_TICKS_PER_REV) /
                                 WHEEL_CIRCUMFERENCE_UM);
    s->in.right_ticks = (int32_t)((s->wheel_pos_um[1] * ROVER_TICKS_PER_REV) /
                                  WHEEL_CIRCUMFERENCE_UM);
}

static void sim_step(sim_t *s, uint32_t us)
{
    while (us > 0) {
        uint32_t dt = us < ROVER_CTRL_PERIOD_US ? us : ROVER_CTRL_PERIOD_US;
        s->now_us += dt;
        sim_plant(s, dt);
        rover_core_step(s->core, s->now_us, &s->in, &s->out);
        sim_collect(s);
        us -= dt;
    }
}

/** Build a down frame and hand its bytes to the core, exactly as the UART
 *  would.  `seq` of 0 means "the next one". */
static size_t sim_encode(char type, uint16_t seq, uint16_t session,
                         const int64_t *fields, uint8_t n, uint8_t *out, size_t cap)
{
    rover_frame_t frame;
    for (uint8_t i = 0; i < ROVER_MAX_FIELDS; i++) {
        frame.field[i] = 0;
    }
    frame.type = type;
    frame.seq = seq;
    frame.session = session;
    frame.nfields = n;
    for (uint8_t i = 0; i < n; i++) {
        frame.field[i] = fields[i];
    }
    return rover_encode_frame(&frame, out, cap);
}

static void sim_send_full(sim_t *s, char type, uint16_t seq, uint16_t session,
                          const int64_t *fields, uint8_t n)
{
    uint8_t line[ROVER_MAX_LINE_BYTES + 8];
    size_t len = sim_encode(type, seq, session, fields, n, line, sizeof line);
    CHECK(len > 0);
    rover_core_feed(s->core, line, len);
}

static void sim_send(sim_t *s, char type, const int64_t *fields, uint8_t n)
{
    sim_send_full(s, type, ++s->down_seq, s->session, fields, n);
}

static void sim_velocity(sim_t *s, int32_t v_mm_s, int32_t w_mrad_s, int32_t ttl_ms,
                         int32_t flags)
{
    const int64_t fields[4] = {v_mm_s, w_mrad_s, ttl_ms, flags};
    sim_send(s, 'V', fields, 4);
}

/** Stream V at 20 Hz for `us`, the way robotd does. */
static void sim_drive(sim_t *s, int32_t v_mm_s, int32_t w_mrad_s, uint32_t us)
{
    while (us > 0) {
        uint32_t slice = us < 50000u ? us : 50000u;
        sim_velocity(s, v_mm_s, w_mrad_s, 300, 0);
        sim_step(s, slice);
        us -= slice;
    }
}

static const rover_frame_t *sim_find_ack(const sim_t *s, char acked, int result,
                                         int reason)
{
    for (int i = 0; i < s->nframes; i++) {
        const rover_frame_t *f = &s->frames[i];
        if (f->type == 'K' && f->field[ROVER_K_ACK_TYPE] == (unsigned char)acked &&
            f->field[ROVER_K_RESULT] == result && f->field[ROVER_K_REASON] == reason) {
            return f;
        }
    }
    return NULL;
}

static const rover_frame_t *sim_find_event(const sim_t *s, rover_event_t event)
{
    for (int i = 0; i < s->nframes; i++) {
        if (s->frames[i].type == 'E' && s->frames[i].field[ROVER_E_EVENT] == event) {
            return &s->frames[i];
        }
    }
    return NULL;
}

/** Boot, wait out the cliff calibration, hello, arm. */
static void sim_bring_up(sim_t *s)
{
    sim_step(s, 2000000);
    const int64_t hello[1] = {3735928559LL};
    sim_send(s, 'H', hello, 1);
    sim_step(s, 20000);
    const int64_t arm[1] = {90210};
    sim_send(s, 'A', arm, 1);
    sim_step(s, 20000);
}

/* ---- I-3: boot state, hello and arm ------------------------------------ */

void test_boots_disarmed_and_refuses_motion_until_hello_then_arm(void)
{
    sim_t *s = sim_new(40010);

    sim_step(s, 20000);
    CHECK_EQ(s->out.state, ROVER_STATE_DISARMED);
    CHECK_EQ(s->out.motor_en, 0);
    CHECK_EQ(s->out.brake, 1);
    CHECK(sim_find_event(s, ROVER_EVENT_ARM_OK) == NULL);

    /* The banner streams at 1 Hz until the first valid H (5.1). */
    CHECK(sim_find_ack(s, 'A', ROVER_ACK_OK, ROVER_REASON_NONE) == NULL);
    bool banner = false;
    for (int i = 0; i < s->nframes; i++) {
        if (s->frames[i].type == 'B') {
            banner = true;
            CHECK_EQ(s->frames[i].field[ROVER_B_SAFETY_HASH], rover_safety_hash());
            CHECK_EQ(s->frames[i].field[ROVER_B_PROTO_VER], ROVER_PROTO_VER);
        }
    }
    CHECK(banner);

    sim_step(s, 2000000);
    CHECK(s->core->cal_valid); /* 50 cliff samples stored on entry to DISARMED */

    /* A velocity before the arm is refused, not merely ignored. */
    sim_frames_reset(s);
    sim_velocity(s, 200, 0, 300, 0);
    sim_step(s, 50000);
    CHECK(sim_find_ack(s, 'V', ROVER_ACK_REJECT, ROVER_REASON_NOT_ARMED) != NULL);
    CHECK_EQ(s->out.duty_l_q15, 0);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);

    /* An arm before the hello is refused too: without one the MCU has no host. */
    sim_frames_reset(s);
    const int64_t arm[1] = {90210};
    sim_send(s, 'A', arm, 1);
    sim_step(s, 20000);
    CHECK(sim_find_ack(s, 'A', ROVER_ACK_REJECT, ROVER_REASON_BAD_SESSION) != NULL);
    CHECK_EQ(s->out.state, ROVER_STATE_DISARMED);

    sim_frames_reset(s);
    const int64_t hello[1] = {3735928559LL};
    sim_send(s, 'H', hello, 1);
    sim_step(s, 20000);
    sim_send(s, 'A', arm, 1);
    sim_step(s, 20000);
    const rover_frame_t *ack = sim_find_ack(s, 'A', ROVER_ACK_OK, ROVER_REASON_NONE);
    CHECK(ack != NULL);
    if (ack != NULL) {
        CHECK_EQ(ack->field[ROVER_K_ECHO], 90210); /* the nonce round-trips */
    }
    CHECK_EQ(s->out.motor_en, 1);
    CHECK(s->out.state == ROVER_STATE_ARMED_IDLE);

    sim_drive(s, 200, 0, 600000);
    CHECK(rover_core_v_cmd_mm_s(s->core) == 200);
    CHECK_EQ(s->out.state, ROVER_STATE_ARMED_MOVING);
    sim_free(s);
}

/* ---- I-3 / I-13: a session change is a reboot -------------------------- */

void test_session_change_after_reboot_refuses_motion(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);
    sim_drive(s, 200, 0, 500000);
    CHECK(rover_core_v_cmd_mm_s(s->core) > 0);

    /* The MCU panics and reboots: a new session from the hardware RNG, and the
     * host's frames are now foreign (4.1, I-3). */
    const uint16_t old_seq = s->down_seq;
    rover_core_init(s->core, &s->cfg, 51882, s->now_us);
    sim_frames_reset(s);
    sim_step(s, 20000);
    CHECK_EQ(rover_core_session(s->core), 51882);
    CHECK_EQ(s->out.state, ROVER_STATE_DISARMED);

    const uint32_t before = rover_core_counters(s->core)->bad_session;
    for (int i = 0; i < 10; i++) {
        sim_send_full(s, 'V', (uint16_t)(old_seq + 1 + i), 40010,
                      (const int64_t[]){250, 0, 300, 0}, 4);
        sim_step(s, 50000);
    }
    CHECK_EQ(rover_core_counters(s->core)->bad_session - before, 10);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);
    CHECK_EQ(s->out.duty_l_q15, 0);
    CHECK(sim_find_ack(s, 'V', ROVER_ACK_OK, ROVER_REASON_NONE) == NULL);

    /* An arm in the old session is equally invisible. */
    sim_send_full(s, 'A', (uint16_t)(old_seq + 20), 40010,
                  (const int64_t[]){90210}, 1);
    sim_step(s, 20000);
    CHECK(sim_find_ack(s, 'A', ROVER_ACK_OK, ROVER_REASON_NONE) == NULL);
    CHECK_EQ(s->out.state, ROVER_STATE_DISARMED);

    /* Re-seeding with the wildcard hello and the new session recovers. */
    s->session = 51882;
    s->down_seq = 1;
    sim_send_full(s, 'H', 1, ROVER_SESSION_WILDCARD, (const int64_t[]){1}, 1);
    sim_step(s, 20000);
    sim_frames_reset(s);
    s->down_seq = 2;
    sim_send(s, 'A', (const int64_t[]){77}, 1);
    sim_step(s, 20000);
    CHECK(sim_find_ack(s, 'A', ROVER_ACK_OK, ROVER_REASON_NONE) != NULL);
    sim_free(s);
}

/* ---- I-1: the command TTL ---------------------------------------------- */

void test_ttl_expiry_stops_the_motors(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);
    sim_drive(s, 300, 0, 1000000);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 300);

    const int64_t start_um = s->wheel_pos_um[0];
    const uint64_t last_v_us = s->now_us;

    /* The cable is pulled: no more V frames arrive. */
    sim_frames_reset(s);
    sim_step(s, 310000);
    CHECK(s->core->ttl_expired);
    CHECK((s->out.fault & ROVER_FAULT_TTL) != 0);
    CHECK_EQ(s->out.ctrl_flags & ROVER_CF_TTL_OK, 0);
    CHECK(sim_find_event(s, ROVER_EVENT_TTL_EXPIRED) != NULL);

    /* The reduction uses the 2000 mm/s^2 abort ramp, not accel_mps2, so 300
     * mm/s reaches zero in 150 ms (4.1, I-5). */
    uint64_t stopped_us = 0;
    for (int i = 0; i < 40 && stopped_us == 0; i++) {
        sim_step(s, ROVER_CTRL_PERIOD_US);
        if (rover_core_v_cmd_mm_s(s->core) == 0) {
            stopped_us = s->now_us;
        }
    }
    CHECK(stopped_us != 0);
    CHECK(stopped_us - last_v_us <= 300000u + 150000u + ROVER_CTRL_PERIOD_US);
    sim_step(s, 100000);
    CHECK_EQ(s->out.brake, 1);
    CHECK_EQ(s->out.duty_l_q15, 0);
    CHECK_EQ(s->out.duty_r_q15, 0);

    /* I-1's travel budget: 200 mm from the last accepted frame. */
    const int64_t travelled_um = s->wheel_pos_um[0] - start_um;
    CHECK(travelled_um < 200000);

    /* MOTOR_EN is not tracked to the TTL: it deasserts only on a latched
     * fault, undervoltage or shutdown (4.1). */
    CHECK_EQ(s->out.motor_en, 1);

    /* Five seconds continuously expired disarms (T1). */
    sim_step(s, 5000000);
    CHECK_EQ(s->out.state, ROVER_STATE_DISARMED);
    sim_free(s);
}

void test_stale_sequence_does_not_renew_the_ttl(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);

    const uint16_t seq = ++s->down_seq;
    sim_send_full(s, 'V', seq, s->session, (const int64_t[]){250, 0, 300, 0}, 4);
    sim_step(s, ROVER_CTRL_PERIOD_US);
    const uint64_t deadline = rover_core_ttl_deadline_us(s->core);
    CHECK(deadline > s->now_us);

    /* A replay of the same frame, and one with an older seq, at 20 Hz. */
    const uint32_t stale_before = rover_core_counters(s->core)->stale_seq;
    for (int i = 0; i < 6; i++) {
        sim_send_full(s, 'V', seq, s->session, (const int64_t[]){250, 0, 300, 0}, 4);
        sim_send_full(s, 'V', (uint16_t)(seq - 1), s->session,
                      (const int64_t[]){250, 0, 300, 0}, 4);
        sim_step(s, 50000);
    }
    CHECK_EQ(rover_core_counters(s->core)->stale_seq - stale_before, 12);
    CHECK_EQ(rover_core_ttl_deadline_us(s->core), deadline);
    CHECK(s->core->ttl_expired);
    CHECK((s->out.fault & ROVER_FAULT_TTL) != 0);

    sim_step(s, 200000);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);
    CHECK_EQ(s->out.duty_l_q15, 0);

    /* A newer seq renews it again and clears the advisory bit. */
    sim_frames_reset(s);
    s->down_seq = (uint16_t)(seq + 100);
    sim_velocity(s, 250, 0, 300, 0);
    sim_step(s, ROVER_CTRL_PERIOD_US);
    CHECK(!s->core->ttl_expired);
    CHECK_EQ(s->out.fault & ROVER_FAULT_TTL, 0);
    CHECK(sim_find_event(s, ROVER_EVENT_TTL_RECOVERED) != NULL);
    sim_free(s);
}

void test_bad_crc_is_dropped_counted_and_does_not_renew_the_ttl(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);

    sim_velocity(s, 250, 0, 300, 0);
    sim_step(s, ROVER_CTRL_PERIOD_US);
    const uint64_t deadline = rover_core_ttl_deadline_us(s->core);

    /* Frames with fresh sequence numbers and one corrupted CRC nibble. */
    const uint32_t before = rover_core_counters(s->core)->bad_crc;
    for (int i = 0; i < 6; i++) {
        uint8_t line[ROVER_MAX_LINE_BYTES + 8];
        size_t n = sim_encode('V', ++s->down_seq, s->session,
                              (const int64_t[]){250, 0, 300, 0}, 4, line, sizeof line);
        CHECK(n > 6);
        line[n - 2] = (uint8_t)(line[n - 2] == '0' ? '1' : '0');
        rover_core_feed(s->core, line, n);
        sim_step(s, 50000);
    }
    CHECK_EQ(rover_core_counters(s->core)->bad_crc - before, 6);
    CHECK_EQ(rover_core_ttl_deadline_us(s->core), deadline);
    CHECK(s->core->ttl_expired);

    /* A corrupted body under an unchanged CRC is caught the same way. */
    uint8_t line[ROVER_MAX_LINE_BYTES + 8];
    size_t n = sim_encode('V', ++s->down_seq, s->session,
                          (const int64_t[]){250, 0, 300, 0}, 4, line, sizeof line);
    line[6] = (uint8_t)(line[6] == '1' ? '2' : '1');
    rover_core_feed(s->core, line, n);
    sim_step(s, ROVER_CTRL_PERIOD_US);
    CHECK_EQ(rover_core_counters(s->core)->bad_crc - before, 7);
    /* Nothing renewed the TTL, so the abort ramp still runs to zero. */
    sim_step(s, 200000);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);
    CHECK_EQ(s->out.brake, 1);
    sim_free(s);
}

/* ---- I-4: compiled caps cannot be raised ------------------------------- */

void test_over_cap_velocity_is_clamped_and_flagged(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);

    sim_frames_reset(s);
    sim_velocity(s, 900, 0, 300, 0);
    sim_step(s, ROVER_CTRL_PERIOD_US);
    CHECK((s->out.fault & ROVER_FAULT_CAP_CLAMPED) != 0);
    /* A clamp is reported, never a rejection: a reject would leave the previous
     * command running (A5). */
    CHECK(sim_find_ack(s, 'V', ROVER_ACK_CLAMPED, ROVER_REASON_CAP_EXCEEDED) != NULL);
    CHECK(sim_find_ack(s, 'V', ROVER_ACK_REJECT, ROVER_REASON_CAP_EXCEEDED) == NULL);
    const rover_frame_t *clamp = sim_find_event(s, ROVER_EVENT_CAP_CLAMP);
    CHECK(clamp != NULL);
    if (clamp != NULL) {
        CHECK_EQ(clamp->field[ROVER_E_ARG], ROVER_MAX_V_MM_S);
    }

    int32_t peak_v = 0;
    int32_t peak_w = 0;
    for (int i = 0; i < 200; i++) {
        sim_velocity(s, 5000, -9000, 300, 0);
        sim_step(s, 50000);
        int32_t v = rover_core_v_cmd_mm_s(s->core);
        int32_t w = rover_core_w_cmd_mrad_s(s->core);
        if (v > peak_v) {
            peak_v = v;
        }
        if (-w > peak_w) {
            peak_w = -w;
        }
    }
    CHECK_EQ(peak_v, ROVER_MAX_V_MM_S);
    CHECK_EQ(peak_w, ROVER_MAX_W_MRAD_S);

    /* No value of V.flags widens anything: b0 only ever removes forward
     * motion, and b2-7 are reserved and ignored (I-4). */
    s->in.tof_fl_mm = 400;
    s->in.tof_fr_mm = 400;
    sim_drive(s, 0, 0, 400000); /* let the abort ramp settle before sweeping */
    for (int flags = 0; flags < 256; flags++) {
        sim_velocity(s, 300, 0, 300, flags);
        sim_step(s, 50000);
        int32_t v = rover_core_v_cmd_mm_s(s->core);
        CHECK(v <= rover_slow_zone_v_cap(400));
    }
    /* b0 is narrowing in the other direction: it refuses forward inside the
     * slow zone entirely. */
    for (int i = 0; i < 20; i++) {
        sim_velocity(s, 300, 0, 300, ROVER_VFLAG_REQUIRE_SLOW_ZONE_STOP);
        sim_step(s, 50000);
    }
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);
    sim_free(s);
}

/* ---- I-5: the asymmetric slew limiter ---------------------------------- */

void test_the_ramp_limit_holds(void)
{
    /* One 10 ms step of accel_mps2 = 0.5 from 250 mm/s is 255, which is the
     * v_cmd the golden telemetry frame carries. */
    CHECK_EQ(rover_slew(250, 300, 5, 20), 255);
    CHECK_EQ(rover_slew(300, 0, 5, 20), 280);
    /* A reversal stops at zero first, so the abort ramp can never accelerate
     * the other way in one step. */
    CHECK_EQ(rover_slew(10, -300, 5, 20), 0);
    CHECK_EQ(rover_slew(0, -300, 5, 20), -5);

    int32_t v = 300;
    int steps = 0;
    while (v != 0 && steps < 100) {
        v = rover_slew(v, 0, 5, 20);
        steps++;
    }
    CHECK_EQ(steps, 15); /* 150 ms, the golden decrease vector */

    sim_t *s = sim_new(40010);
    sim_bring_up(s);
    int32_t previous = rover_core_v_cmd_mm_s(s->core);
    int32_t worst_up = 0;
    for (int i = 0; i < 200; i++) {
        sim_velocity(s, 300, 0, 300, 0);
        sim_step(s, ROVER_CTRL_PERIOD_US);
        int32_t now = rover_core_v_cmd_mm_s(s->core);
        if (now - previous > worst_up) {
            worst_up = now - previous;
        }
        previous = now;
    }
    CHECK_EQ(worst_up, 5);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 300);

    int32_t worst_down = 0;
    for (int i = 0; i < 40; i++) {
        sim_velocity(s, 0, 0, 300, 0);
        sim_step(s, ROVER_CTRL_PERIOD_US);
        int32_t now = rover_core_v_cmd_mm_s(s->core);
        if (previous - now > worst_down) {
            worst_down = previous - now;
        }
        previous = now;
    }
    CHECK_EQ(worst_down, 20);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);

    /* S runs the same abort ramp.  Mode 1 coasts: duty zero with the brake
     * released, which is not the same terminal state as mode 0. */
    sim_drive(s, 300, 0, 800000);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 300);
    sim_frames_reset(s);
    sim_send(s, 'S', (const int64_t[]){1}, 1);
    sim_step(s, ROVER_CTRL_PERIOD_US);
    CHECK(sim_find_ack(s, 'S', ROVER_ACK_OK, ROVER_REASON_NONE) != NULL);
    CHECK_EQ(s->out.duty_l_q15, 0);
    CHECK_EQ(s->out.brake, 0);
    sim_step(s, 400000);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);
    CHECK_EQ(s->out.brake, 0);
    sim_send(s, 'S', (const int64_t[]){0}, 1);
    sim_step(s, 50000);
    CHECK_EQ(s->out.brake, 1);
    sim_free(s);
}

/* ---- I-5: obstacle class ----------------------------------------------- */

void test_obstacle_blocks_forward_reverse_and_rotation_still_work(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);
    sim_drive(s, 250, 0, 800000);
    CHECK(rover_core_v_cmd_mm_s(s->core) > 200);

    /* The golden telemetry frame's obstacle: 231 mm, inside the 250 mm stop. */
    s->in.tof_fl_mm = 231;
    s->in.tof_fr_mm = 231;
    sim_drive(s, 250, 0, 300000);
    CHECK((s->out.fault & ROVER_FAULT_TOF_STOP) != 0);
    CHECK_EQ(s->out.ctrl_flags & ROVER_CF_TOF_CLEAR, 0);
    CHECK((s->out.ctrl_flags & ROVER_CF_IN_SLOW_ZONE) != 0);
    /* The obstacle class does not enter FAULT and does not disarm. */
    CHECK(s->out.state == ROVER_STATE_ARMED_IDLE ||
          s->out.state == ROVER_STATE_ARMED_MOVING);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);

    /* Reverse stays legal, clamped to 150 mm/s because reverse is unsensed. */
    sim_drive(s, -300, 0, 800000);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), -ROVER_REVERSE_CLAMP_MM_S);
    CHECK(s->out.duty_l_q15 < 0);

    /* Rotation stays legal, clamped to the slow-zone angular cap. */
    sim_drive(s, 0, 1200, 1200000);
    CHECK_EQ(rover_core_w_cmd_mrad_s(s->core), ROVER_SLOW_ZONE_W_MRAD_S);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);

    /* The MCU is the sole clearer: five clean samples and the bit is gone. */
    s->in.tof_fl_mm = 2000;
    s->in.tof_fr_mm = 2000;
    sim_drive(s, 0, 0, 400000);
    CHECK_EQ(s->out.fault & ROVER_FAULT_TOF_STOP, 0);
    CHECK_EQ(s->out.ctrl_flags & ROVER_CF_IN_SLOW_ZONE, 0);
    sim_drive(s, 250, 0, 800000);
    CHECK(rover_core_v_cmd_mm_s(s->core) > 200);

    /* The slow zone itself: v <= (d - 250 mm) / 1.0 s, capped at 150 mm/s. */
    s->in.tof_fl_mm = 400;
    s->in.tof_fr_mm = 400;
    sim_drive(s, 300, 0, 600000);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 150);
    s->in.tof_fl_mm = 380;
    s->in.tof_fr_mm = 380;
    sim_drive(s, 300, 0, 600000);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 130);

    /* A bumper is the same class: forward refused, reverse allowed. */
    s->in.tof_fl_mm = 2000;
    s->in.tof_fr_mm = 2000;
    s->in.bumper_clear = 0;
    sim_drive(s, 250, 0, 300000);
    CHECK((s->out.fault & ROVER_FAULT_BUMPER) != 0);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);
    CHECK(s->out.state != ROVER_STATE_FAULT);
    sim_free(s);
}

/* ---- I-16: stale sensors are blockage, not clear path ------------------ */

void test_stale_tof_is_treated_as_blockage(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);
    sim_drive(s, 250, 0, 800000);
    CHECK(rover_core_v_cmd_mm_s(s->core) > 200);

    /* One forward sensor errors while the other still reports a clear path:
     * forward is refused anyway, because one dead sensor halves an already
     * marginal cone (4.1, I-16). */
    s->in.tof_fl_mm = ROVER_TOF_ERROR_MM;
    sim_drive(s, 250, 0, 300000);
    CHECK((s->out.fault & ROVER_FAULT_TOF_STALE) != 0);
    CHECK_EQ(s->out.ctrl_flags & ROVER_CF_TOF_FL_OK, 0);
    CHECK((s->out.ctrl_flags & ROVER_CF_TOF_FR_OK) != 0);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);
    CHECK(s->out.state != ROVER_STATE_FAULT);

    sim_drive(s, -300, 0, 800000);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), -ROVER_REVERSE_CLAMP_MM_S);

    /* An I2C status bit does the same without a sentinel distance. */
    s->in.tof_fl_mm = 2000;
    sim_drive(s, 0, 0, 400000);
    CHECK_EQ(s->out.fault & ROVER_FAULT_TOF_STALE, 0);
    s->in.tof_status = ROVER_TOF_BIT_FR;
    sim_drive(s, 250, 0, 300000);
    CHECK((s->out.fault & ROVER_FAULT_TOF_STALE) != 0);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);

    /* 65534 is the opposite case: no target within range is a clear path. */
    s->in.tof_status = 0;
    s->in.tof_fl_mm = ROVER_TOF_NO_TARGET_MM;
    s->in.tof_fr_mm = ROVER_TOF_NO_TARGET_MM;
    sim_drive(s, 0, 0, 400000);
    CHECK_EQ(s->out.fault & (ROVER_FAULT_TOF_STALE | ROVER_FAULT_TOF_STOP), 0);
    sim_drive(s, 300, 0, 900000);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 300);
    CHECK_EQ(rover_slow_zone_v_cap(ROVER_TOF_NO_TARGET_MM), ROVER_MAX_V_MM_S);
    sim_free(s);
}

/* ---- 5.1: the latched class -------------------------------------------- */

void test_fault_latches_and_clears_only_on_explicit_clear(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);
    sim_drive(s, 250, 0, 800000);

    /* A24's per-wheel detector: one wheel held while the other runs free.  A
     * body-frame test never trips here, because body v_meas is half of v_cmd. */
    const uint64_t held_us = s->now_us;
    s->hold_left = true;
    for (int i = 0; i < 20 && (s->out.fault & ROVER_FAULT_STALL) == 0; i++) {
        sim_drive(s, 250, 0, 50000);
    }
    CHECK((s->out.fault & ROVER_FAULT_STALL) != 0);
    /* I-10: the 200 ms window plus one control period. */
    CHECK(s->now_us - held_us <= 250000u + ROVER_CTRL_PERIOD_US);
    CHECK_EQ(s->out.state, ROVER_STATE_FAULT);
    CHECK_EQ(s->out.motor_en, 0);
    sim_frames_reset(s);
    sim_velocity(s, 250, 0, 300, 0);
    sim_step(s, 50000);
    CHECK(sim_find_ack(s, 'V', ROVER_ACK_REJECT, ROVER_REASON_FAULT_LATCHED) != NULL);
    const rover_frame_t *stall = sim_find_event(s, ROVER_EVENT_STALL);
    CHECK(stall == NULL || stall->field[ROVER_E_ARG] == ROVER_LEFT);

    s->hold_left = false;
    sim_step(s, 300000);
    sim_frames_reset(s);
    sim_send(s, 'C',
             (const int64_t[]){ROVER_FAULT_STALL | ROVER_FAULT_ENC_IMPLAUS}, 1);
    sim_step(s, 50000);
    CHECK(sim_find_ack(s, 'C', ROVER_ACK_OK, ROVER_REASON_NONE) != NULL);
    CHECK_EQ(s->out.fault & (ROVER_FAULT_STALL | ROVER_FAULT_ENC_IMPLAUS), 0);
    CHECK_EQ(s->out.state, ROVER_STATE_DISARMED);

    /* Recovery needs a fresh arm and a fresh cliff baseline. */
    sim_step(s, 2000000);
    sim_send(s, 'A', (const int64_t[]){90211}, 1);
    sim_step(s, 20000);
    sim_drive(s, 250, 0, 500000);
    CHECK(rover_core_v_cmd_mm_s(s->core) > 0);

    s->in.estop_released = 0;
    sim_step(s, 50000);
    CHECK((s->out.fault & ROVER_FAULT_ESTOP) != 0);
    CHECK_EQ(s->out.state, ROVER_STATE_ESTOP);
    CHECK_EQ(s->out.motor_en, 0);
    sim_step(s, 200000);
    CHECK_EQ(s->out.duty_l_q15, 0);
    CHECK_EQ(s->out.duty_r_q15, 0);

    sim_frames_reset(s);
    sim_velocity(s, 250, 0, 300, 0);
    sim_step(s, 50000);
    CHECK(sim_find_ack(s, 'V', ROVER_ACK_REJECT, ROVER_REASON_ESTOP_ASSERTED) != NULL);

    /* Refused while the cause persists. */
    sim_frames_reset(s);
    sim_send(s, 'C', (const int64_t[]){ROVER_FAULT_ESTOP}, 1);
    sim_step(s, 50000);
    CHECK(sim_find_ack(s, 'C', ROVER_ACK_REJECT, ROVER_REASON_FAULT_LATCHED) != NULL);
    CHECK((s->out.fault & ROVER_FAULT_ESTOP) != 0);

    /* Releasing the button does not clear it: a latched bit needs a C. */
    s->in.estop_released = 1;
    sim_step(s, 1000000);
    CHECK((s->out.fault & ROVER_FAULT_ESTOP) != 0);
    CHECK_EQ(s->out.state, ROVER_STATE_ESTOP);

    sim_frames_reset(s);
    sim_send(s, 'C', (const int64_t[]){ROVER_FAULT_ESTOP}, 1);
    sim_step(s, 50000);
    CHECK(sim_find_ack(s, 'C', ROVER_ACK_OK, ROVER_REASON_NONE) != NULL);
    CHECK_EQ(s->out.fault & ROVER_FAULT_ESTOP, 0);
    CHECK(sim_find_event(s, ROVER_EVENT_FAULT_CLEARED) != NULL);

    /* Recovery does not re-arm: the next motion needs a fresh A. */
    CHECK_EQ(s->out.state, ROVER_STATE_DISARMED);
    sim_frames_reset(s);
    sim_velocity(s, 250, 0, 300, 0);
    sim_step(s, 50000);
    CHECK(sim_find_ack(s, 'V', ROVER_ACK_REJECT, ROVER_REASON_NOT_ARMED) != NULL);

    /* And a C naming an obstacle-class bit clears nothing: the MCU owns those. */
    s->in.bumper_clear = 0;
    sim_step(s, 100000);
    CHECK((s->out.fault & ROVER_FAULT_BUMPER) != 0);
    sim_frames_reset(s);
    sim_send(s, 'C', (const int64_t[]){ROVER_FAULT_BUMPER}, 1);
    sim_step(s, 50000);
    CHECK((s->out.fault & ROVER_FAULT_BUMPER) != 0);
    sim_free(s);
}

/* ---- the wheel PI controller ------------------------------------------- */

static int32_t settle(int32_t target_mm_s, int steps, int64_t *out_um_s)
{
    rover_pi_t pi;
    rover_pi_reset(&pi);
    int64_t v_um_s = 0;
    int32_t worst_overshoot = 0;
    for (int i = 0; i < steps; i++) {
        int16_t duty = rover_pi_step(&pi, target_mm_s, (int32_t)(v_um_s / 1000));
        int64_t open_loop_um_s =
            ((int64_t)duty * PLANT_FULL_DUTY_MM_S * 1000) / ROVER_DUTY_MAX_Q15;
        v_um_s += ((open_loop_um_s - v_um_s) * ROVER_CTRL_PERIOD_US) / PLANT_TAU_US;
        int32_t excess = (int32_t)(v_um_s / 1000) - target_mm_s;
        if (target_mm_s < 0) {
            excess = -excess;
        }
        if (excess > worst_overshoot) {
            worst_overshoot = excess;
        }
    }
    *out_um_s = v_um_s;
    return worst_overshoot;
}

void test_pi_controller_converges(void)
{
    int64_t v_um_s = 0;
    int32_t overshoot = settle(300, 200, &v_um_s); /* 2 s */
    CHECK(iabs32((int32_t)(v_um_s / 1000) - 300) <= 5);
    CHECK(overshoot < 60);

    settle(-250, 200, &v_um_s);
    CHECK(iabs32((int32_t)(v_um_s / 1000) + 250) <= 5);

    settle(0, 200, &v_um_s);
    CHECK(iabs32((int32_t)(v_um_s / 1000)) <= 5);

    /* Anti-windup: a wheel held at zero must not leave the integrator able to
     * command full duty once it is released. */
    rover_pi_t pi;
    rover_pi_reset(&pi);
    for (int i = 0; i < 500; i++) {
        (void)rover_pi_step(&pi, 300, 0);
    }
    CHECK(rover_pi_step(&pi, 300, 0) == ROVER_DUTY_MAX_Q15);
    int64_t bound = ((int64_t)ROVER_PI_KI_Q8 * pi.integral) / 256;
    CHECK(bound <= ROVER_DUTY_MAX_Q15);
    sim_free(sim_new(40010)); /* the harness itself is leak-free under ASan */
}

/* ---- I-2: the parser survives anything --------------------------------- */

void test_garbage_and_mid_frame_resets_never_crash_the_parser(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);

    /* Pseudorandom bytes: an LCG, so the corpus is the same on every run. */
    uint32_t state = 0x12345678u;
    for (int block = 0; block < 200; block++) {
        uint8_t noise[1024];
        for (size_t i = 0; i < sizeof noise; i++) {
            state = state * 1664525u + 1013904223u;
            noise[i] = (uint8_t)(state >> 16);
        }
        rover_core_feed(s->core, noise, sizeof noise);
        sim_step(s, ROVER_CTRL_PERIOD_US);
    }

    /* A frame cut in half, then a reset into a new frame with no newline
     * between them -- what a mid-transmission MCU reboot looks like. */
    const char *fragments[] = {
        "$V,2,9,40010,25",
        "$",
        "$H,2,",
        "\n\n\n",
        "$T,2,1,40010,1,1,1,1,1*0000\n",
        "\r\n",
        "$V,2,3,40010,250.5,210,300,0*9482\n",
        "$Z,2,3,40010,1*EE1E\n",
        "$V,3,3,40010,250,210,300,0*F2EC\n",
    };
    for (size_t i = 0; i < sizeof fragments / sizeof fragments[0]; i++) {
        const char *p = fragments[i];
        size_t n = 0;
        while (p[n] != '\0') {
            n++;
        }
        rover_core_feed(s->core, (const uint8_t *)p, n);
        sim_step(s, ROVER_CTRL_PERIOD_US);
    }

    /* A line far longer than the 200-byte maximum is dropped to the next
     * newline and counted once, whatever its length (5.1). */
    const uint32_t overlong_before = rover_core_counters(s->core)->bad_length;
    uint8_t flood[5000];
    for (size_t i = 0; i < sizeof flood; i++) {
        flood[i] = '7';
    }
    flood[0] = '$';
    flood[sizeof flood - 1] = '\n';
    rover_core_feed(s->core, flood, sizeof flood);
    CHECK_EQ(rover_core_counters(s->core)->bad_length - overlong_before, 1);

    CHECK((int32_t)rover_core_counters(s->core)->bad_crc > 0);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);

    /* The link still works afterwards: a fresh core takes the next command. */
    sim_t *fresh = sim_new(40010);
    sim_bring_up(fresh);
    sim_drive(fresh, 250, 0, 800000);
    CHECK(rover_core_v_cmd_mm_s(fresh->core) > 200);
    sim_free(fresh);
    sim_free(s);
}


/* ---- 5.1: an escalated obstacle is latched, and a C releases it ---------- */

/** Re-arm after a recovery, waiting out the 50-sample cliff window first.
 *  A recovery from a latched fault passes through rover_core_init's state on
 *  the bench only when the core is rebuilt; here the wait simply lets the
 *  re-baseline finish, so the case reads against a settled baseline. */
static void sim_rearm(sim_t *s, uint32_t nonce)
{
    sim_step(s, (uint32_t)ROVER_CLIFF_SAMPLES * ROVER_TOF_INTER_PERIOD_MS * 1000u +
                    200000u);
    const int64_t arm[1] = {nonce};
    sim_send(s, 'A', arm, 1);
    sim_step(s, 20000);
}

void test_escalated_obstacle_recovers_on_an_explicit_clear(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);

    /* A wall at 200 mm, inside the 250 mm stop zone. */
    s->in.tof_fl_mm = 200;
    s->in.tof_fr_mm = 200;

    /* Two refused pushes forward, separated by a zero, is the escalation rule
     * of 5.1: ">=2 rejected escape attempts".  It must land in the *latched
     * class*, as a bit the Pi can name -- a private boolean no C can reach is
     * the one-way trip this case exists to catch. */
    sim_drive(s, 200, 0, 1000000);
    CHECK(s->out.fault & ROVER_FAULT_TOF_STOP);
    sim_drive(s, 0, 0, 500000);
    sim_drive(s, 200, 0, 1000000);

    CHECK(s->out.fault & ROVER_FAULT_OBSTACLE_LATCHED);
    CHECK_EQ(s->out.state, ROVER_STATE_FAULT);

    /* Every V is refused with reason 8 while it is set -- including reverse,
     * which is why the escalation has to have an exit. */
    sim_frames_reset(s);
    sim_velocity(s, -100, 0, 300, 0);
    sim_step(s, 50000);
    CHECK(sim_find_ack(s, 'V', ROVER_ACK_REJECT, ROVER_REASON_FAULT_LATCHED) != NULL);

    /* The widest mask robotd can build is ROVER_FAULTS_LATCHED, and it has to
     * be enough: the bit is in that class, and its cause is a past event, so
     * the clear is not refused while the wall is still there.  Refusing it
     * until the rover backs away, when backing away needs the clear, is a
     * deadlock with no exit but a power cycle. */
    sim_frames_reset(s);
    const int64_t clear[1] = {(int64_t)ROVER_FAULTS_LATCHED};
    sim_send(s, 'C', clear, 1);
    sim_step(s, 20000);
    CHECK(sim_find_ack(s, 'C', ROVER_ACK_OK, ROVER_REASON_NONE) != NULL);
    CHECK_EQ(s->out.fault & ROVER_FAULT_OBSTACLE_LATCHED, 0);
    CHECK(s->core->obstacle_tries < ROVER_OBSTACLE_ESCALATE_TRIES);

    /* Re-armed, the obstacle class is back to its own rule: reverse accepted
     * and clamped to 150 mm/s, forward still zeroed while the wall is there. */
    sim_rearm(s, 90211);
    CHECK_EQ(s->out.state, ROVER_STATE_ARMED_IDLE);
    sim_drive(s, -300, 0, 800000);
    CHECK(rover_core_v_cmd_mm_s(s->core) < 0);
    CHECK(rover_core_v_cmd_mm_s(s->core) >= -ROVER_REVERSE_CLAMP_MM_S);
    CHECK_EQ(s->out.fault & ROVER_FAULT_OBSTACLE_LATCHED, 0);

    sim_drive(s, 200, 0, 400000);
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);

    sim_free(s);
}

void test_obstacle_bits_still_self_clear_while_escalated(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);

    /* The 30 s branch: parked in front of a wall with the host streaming its
     * idle zero V, which is exactly what robotd does between goals. */
    s->in.tof_fl_mm = 200;
    s->in.tof_fr_mm = 200;
    sim_drive(s, 0, 0, 31000000);
    CHECK(s->out.fault & ROVER_FAULT_TOF_STOP);
    CHECK(s->out.fault & ROVER_FAULT_OBSTACLE_LATCHED);

    /* Take the wall away.  The five-clean-sample rule has to keep running:
     * gating it on the escalation is what made the obstacle bit unable to go,
     * which in turn made the escalation unable to go. */
    s->in.tof_fl_mm = 2000;
    s->in.tof_fr_mm = 2000;
    sim_drive(s, 0, 0, 2000000);
    CHECK_EQ(s->out.fault & ROVER_FAULTS_OBSTACLE, 0);
    /* The latched bit is still there -- that is what "latched" means -- and a
     * C is the defined recovery, which robotd's `clear` can now build. */
    CHECK(s->out.fault & ROVER_FAULT_OBSTACLE_LATCHED);

    sim_frames_reset(s);
    const int64_t clear[1] = {(int64_t)ROVER_FAULTS_LATCHED};
    sim_send(s, 'C', clear, 1);
    sim_step(s, 20000);
    CHECK_EQ(s->out.fault & ROVER_FAULT_OBSTACLE_LATCHED, 0);

    sim_rearm(s, 90212);
    CHECK_EQ(s->out.state, ROVER_STATE_ARMED_IDLE);
    sim_drive(s, 200, 0, 600000);
    CHECK(rover_core_v_cmd_mm_s(s->core) > 0);

    sim_free(s);
}

/* ---- 4.1: an escalated obstacle must not actuate the e-stop relay ------- */

void test_escalated_obstacle_does_not_open_the_relay(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);
    CHECK_EQ(s->out.motor_en, 1);

    /* Parked 200 mm from a wall with robotd's idle zero V, which is what the
     * 20 Hz writer emits between goals.  The 30 s branch of 5.1 escalates with
     * no command ever sent.  MOTOR_EN drives the coil FET of the 30 A relay,
     * and 4.1 says an obstacle-class cause never actuates it: the relay closes
     * once per power cycle so as not to weld the one contact the hardware
     * e-stop depends on opening, and the BOM's 2.2 ohm NTC inrush limiter is
     * sized for that single closure. */
    s->in.tof_fl_mm = 200;
    s->in.tof_fr_mm = 200;
    for (int i = 0; i < 64; i++) {
        sim_drive(s, 0, 0, 500000);
        CHECK_EQ(s->out.motor_en, 1);
    }
    CHECK(s->out.fault & ROVER_FAULT_OBSTACLE_LATCHED);
    CHECK_EQ(s->out.state, ROVER_STATE_FAULT);
    CHECK_EQ(s->out.motor_en, 1);
    /* It is still a latched fault in every other respect: V is refused with
     * reason 8 and only a C releases it. */
    sim_frames_reset(s);
    sim_velocity(s, -100, 0, 300, 0);
    sim_step(s, 50000);
    CHECK(sim_find_ack(s, 'V', ROVER_ACK_REJECT, ROVER_REASON_FAULT_LATCHED) != NULL);

    /* The exclusion is one bit, not the class: a genuinely latched fault still
     * opens the relay on the very next step. */
    s->in.imotor_ma = (int16_t)ROVER_OVERCURRENT_MA;
    sim_step(s, 20000);
    CHECK(s->out.fault & ROVER_FAULT_OVERCURRENT);
    CHECK_EQ(s->out.motor_en, 0);

    sim_free(s);
}

/* ---- 4.1: a routine idle disarm must not blind forward motion ----------- */

void test_an_idle_disarm_keeps_the_standing_cliff_baseline(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);
    CHECK(s->core->cal_valid);
    const uint16_t baseline = s->core->cliff_baseline_mm;
    CHECK(baseline > 0);

    /* robotd sends D after motion_idle_disarm_ms = 5000 with no active goal,
     * and refuses to arm while ctrl_flags b6 is clear.  4.1 says cal_valid
     * clears on every *reset*, not on every disarm -- clearing it here made
     * every ordinary conversational pause answer `rejected reason=not_ready`
     * for the 1.5 s the 50 samples take. */
    const int64_t none[1] = {0};
    sim_send(s, 'D', none, 0);
    sim_step(s, 20000);
    CHECK_EQ(s->out.state, ROVER_STATE_DISARMED);
    CHECK(s->core->cal_valid);
    CHECK_EQ(s->out.ctrl_flags & ROVER_CF_CAL_VALID, ROVER_CF_CAL_VALID);
    CHECK_EQ(s->core->cliff_baseline_mm, baseline);
    /* The window did restart, so the baseline really is retaken. */
    CHECK_EQ(s->core->cliff_sample_n, 0);

    /* b6 stays set for the whole re-baseline window, so an A landing anywhere
     * inside it is accepted and forward moves at once. */
    for (int i = 0; i < 40; i++) {
        sim_step(s, 50000);
        CHECK_EQ(s->out.ctrl_flags & ROVER_CF_CAL_VALID, ROVER_CF_CAL_VALID);
    }
    const int64_t arm[1] = {90213};
    sim_send(s, 'A', arm, 1);
    sim_step(s, 20000);
    CHECK_EQ(s->out.state, ROVER_STATE_ARMED_IDLE);
    sim_drive(s, 200, 0, 600000);
    CHECK(rover_core_v_cmd_mm_s(s->core) > 0);

    /* The new median swapped in when the 50th sample landed, and the floor
     * moving 40 mm further away is now the baseline the cliff rule uses. */
    s->in.tof_cliff_mm = 138;
    sim_step(s, 3000000);
    CHECK_EQ(s->core->cliff_sample_n, ROVER_CLIFF_SAMPLES);
    CHECK_EQ(s->core->cliff_baseline_mm, baseline);

    sim_free(s);
}

/* ---- A24: the I2t trip is a protection event, not a brick --------------- */

void test_overcurrent_clears_once_the_current_is_gone(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);

    /* What a real I2t trip leaves behind: the accumulator at the limit, with
     * no decay term to bring it down.  Judging the cause on that integral made
     * the reset behind the test dead code and the bit unclearable for ever. */
    s->core->wheel[ROVER_LEFT].i2t_ma2_ms = ROVER_I2T_LIMIT_MA2_MS;
    s->in.imotor_ma = (int16_t)ROVER_OVERCURRENT_MA;
    sim_step(s, 20000);
    CHECK(s->out.fault & ROVER_FAULT_OVERCURRENT);
    CHECK_EQ(s->out.state, ROVER_STATE_FAULT);
    CHECK_EQ(s->out.motor_en, 0);

    /* Wheels stopped, no current: a C naming the bit must clear it and reset
     * both accumulators. */
    s->in.imotor_ma = 0;
    sim_step(s, 500000);
    sim_frames_reset(s);
    const int64_t clear[1] = {(int64_t)ROVER_FAULT_OVERCURRENT};
    sim_send(s, 'C', clear, 1);
    sim_step(s, 20000);
    CHECK(sim_find_ack(s, 'C', ROVER_ACK_OK, ROVER_REASON_NONE) != NULL);
    CHECK_EQ(s->out.fault & ROVER_FAULT_OVERCURRENT, 0);
    CHECK_EQ(s->core->wheel[ROVER_LEFT].i2t_ma2_ms, 0);
    CHECK_EQ(s->core->wheel[ROVER_RIGHT].i2t_ma2_ms, 0);

    /* And the INA226 mirror re-raises it at once if the current really is
     * still over 6 A, so the clear does not disable the protection. */
    s->in.imotor_ma = (int16_t)ROVER_OVERCURRENT_MA;
    sim_step(s, 20000);
    CHECK(s->out.fault & ROVER_FAULT_OVERCURRENT);

    sim_free(s);
}

/* ---- A25: a failed INA read is blockage, not a flat pack ---------------- */

void test_a_stale_ina_read_never_advances_the_undervoltage_ladder(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);
    sim_drive(s, 200, 0, 500000);
    CHECK(rover_core_v_cmd_mm_s(s->core) > 0);

    /* What sensors_fill publishes when an INA226 read fails: the sentinel,
     * never 0 mV.  Publishing 0 mV made a bus stall indistinguishable from a
     * flat pack, and the response to a flat pack is to power the host off. */
    sim_frames_reset(s);
    s->in.vbat_mv = ROVER_VBAT_INVALID_MV;
    s->in.imotor_ma = 0;
    sim_drive(s, 200, 0, 70000000);

    CHECK_EQ(s->out.fault & (ROVER_FAULT_UNDERVOLT_W | ROVER_FAULT_UNDERVOLT_S |
                             ROVER_FAULT_UNDERVOLT_D),
             0);
    CHECK_EQ(s->out.pi_shutdown_req, 0);
    CHECK_EQ(s->out.pi_rail_en, 1);
    /* It stops motion rather than trusting the reading, and says why. */
    CHECK_EQ(rover_core_v_cmd_mm_s(s->core), 0);
    CHECK(sim_find_ack(s, 'V', ROVER_ACK_REJECT, ROVER_REASON_SENSORS_STALE) != NULL);
    const rover_frame_t *i2c = sim_find_event(s, ROVER_EVENT_I2C_ERROR);
    CHECK(i2c != NULL);
    if (i2c != NULL) {
        CHECK_EQ(i2c->field[ROVER_E_ARG], ROVER_I2C_INDEX_INA);
    }

    /* A real reading comes back and the controller is usable again. */
    s->in.vbat_mv = 11620;
    sim_rearm(s, 90214);
    sim_drive(s, 200, 0, 600000);
    CHECK(rover_core_v_cmd_mm_s(s->core) > 0);

    /* A genuinely flat pack still latches the ladder: the freeze is for a
     * reading the MCU cannot trust, not for a pack it can. */
    s->in.vbat_mv = 9500;
    sim_drive(s, 200, 0, 11000000);
    CHECK(s->out.fault & ROVER_FAULT_UNDERVOLT_D);

    sim_free(s);
}

/* ---- 4.1: the cliff window survives a short DISARMED bounce ------------- */

void test_calibration_survives_an_arm_inside_the_sample_window(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);
    CHECK(s->core->cal_valid);

    /* robotd disarms after motion_idle_disarm_ms and the user drives again
     * 0.5 s later -- inside the 50 x 30 ms the recalibration takes.  The
     * standing baseline holds throughout, so b6 never drops and the arm is
     * never answered `not_ready`. */
    sim_send(s, 'D', NULL, 0);
    sim_step(s, 20000);
    CHECK(s->core->cal_valid);
    CHECK_EQ(s->core->cliff_sample_n, 0);

    sim_step(s, 500000);
    CHECK(s->core->cal_valid);
    const int64_t arm[1] = {90213};
    sim_send(s, 'A', arm, 1);
    sim_step(s, 20000);
    CHECK_EQ(s->out.state, ROVER_STATE_ARMED_IDLE);

    /* The window keeps filling while armed, so cal_valid comes back and
     * forward is executed.  Gated on DISARMED it never came back, and the only
     * symptom was forward silently zeroed: no fault, no K reject, no event. */
    sim_drive(s, 200, 0, 3000000);
    CHECK(s->core->cal_valid);
    CHECK(rover_core_v_cmd_mm_s(s->core) > 0);
    CHECK_EQ(s->out.fault & ROVER_FAULTS_LATCHED, 0);

    sim_free(s);
}

/* ---- 5.1 event 10: an I2C failure is distinguishable from a blocked path - */

void test_a_tof_bus_error_emits_i2c_error_with_its_sensor_index(void)
{
    sim_t *s = sim_new(40010);
    sim_bring_up(s);

    sim_frames_reset(s);
    s->in.tof_status = ROVER_TOF_BIT_FL;
    s->in.tof_fl_mm = ROVER_TOF_ERROR_MM;
    sim_step(s, 200000);
    const rover_frame_t *i2c = sim_find_event(s, ROVER_EVENT_I2C_ERROR);
    CHECK(i2c != NULL);
    if (i2c != NULL) {
        CHECK_EQ(i2c->field[ROVER_E_ARG], ROVER_I2C_INDEX_TOF_FL);
    }

    /* An out-of-range return is not a bus error: it raises TOF_STALE and a
     * TOF_STATUS and no I2C_ERROR, which is the discrimination 4.1's coverage
     * rule and INV-02's battery ladder both turn on. */
    sim_frames_reset(s);
    s->in.tof_status = 0;
    s->in.tof_fl_mm = 2000;
    sim_step(s, 400000);
    sim_frames_reset(s);
    s->in.tof_fr_mm = ROVER_TOF_ERROR_MM;
    sim_step(s, 200000);
    CHECK(sim_find_event(s, ROVER_EVENT_TOF_STATUS) != NULL);
    CHECK(sim_find_event(s, ROVER_EVENT_I2C_ERROR) == NULL);

    sim_free(s);
}
