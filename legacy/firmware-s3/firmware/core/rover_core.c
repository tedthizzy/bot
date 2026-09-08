/* Session, arm, TTL, the state machine and telemetry assembly.
 *
 * The core owns no I/O and no clock: `now_us` is handed in, and it is always
 * the MCU's own monotonic microseconds.  No host timestamp is ever compared
 * against it -- `P.pi_mono_us` is stored and echoed as an opaque token and is
 * never read as a time (principle 5, I-17).
 */
#include "rover_internal.h"

#define ROVER_ARM_MOTION_V_MM_S 20
#define ROVER_ARM_MOTION_W_MRAD_S 100

static void zero_bytes(void *dst, size_t n)
{
    uint8_t *p = (uint8_t *)dst;
    for (size_t i = 0; i < n; i++) {
        p[i] = 0;
    }
}

size_t rover_core_size(void)
{
    return sizeof(struct rover_core);
}

void rover_cfg_default(rover_cfg_t *cfg)
{
    zero_bytes(cfg, sizeof *cfg);
    cfg->fw_ver = ROVER_FW_VER;
    cfg->caps = ROVER_CAP_CLIFF_SENSOR | ROVER_CAP_INA;
    cfg->reset_reason = 1;
    cfg->debug_build = false;
    cfg->wheel_radius_mm = ROVER_WHEEL_RADIUS_MM;
    cfg->track_mm = ROVER_TRACK_MM;
    cfg->ticks_per_rev = ROVER_TICKS_PER_REV;
    cfg->r_pack_mohm = ROVER_R_PACK_MOHM;
    cfg->r_motor_mohm = ROVER_R_MOTOR_MOHM;
    cfg->k_e_uv_per_mm_s = ROVER_K_E_UV_PER_MM_S;
}

/* ---- transmit queue ---------------------------------------------------- */

void rover_core_emit(struct rover_core *core, const rover_frame_t *frame)
{
    uint8_t line[ROVER_MAX_LINE_BYTES + 8];
    rover_frame_t sent = *frame;
    sent.seq = ++core->up_seq;
    sent.session = core->session;
    size_t n = rover_encode_frame(&sent, line, sizeof line);
    if (n == 0 || core->tx_len + n > ROVER_TX_BYTES) {
        core->up_seq--; /* nothing went out, so the counter must not advance */
        return;
    }
    for (size_t i = 0; i < n; i++) {
        core->tx[core->tx_len + i] = line[i];
    }
    core->tx_len = (uint16_t)(core->tx_len + n);
}

static void emit(struct rover_core *core, char type, const int64_t *fields,
                 uint8_t nfields)
{
    rover_frame_t frame;
    zero_bytes(&frame, sizeof frame);
    frame.type = type;
    frame.nfields = nfields;
    for (uint8_t i = 0; i < nfields; i++) {
        frame.field[i] = fields[i];
    }
    rover_core_emit(core, &frame);
}

void rover_core_event(struct rover_core *core, rover_event_t event, int32_t arg)
{
    const int64_t fields[3] = {(int64_t)event, arg, (int64_t)core->now_us};
    emit(core, 'E', fields, 3);
}

static void emit_ack(struct rover_core *core, char acked, uint16_t acked_seq,
                     rover_ack_result_t result, rover_reason_t reason, uint32_t echo)
{
    const int64_t fields[5] = {(int64_t)(unsigned char)acked, acked_seq,
                               (int64_t)result, (int64_t)reason, (int64_t)echo};
    emit(core, 'K', fields, 5);
}

static void emit_banner(struct rover_core *core)
{
    const int64_t fields[5] = {core->cfg.fw_ver, ROVER_PROTO_VER, core->cfg.caps,
                               core->cfg.reset_reason, (int64_t)rover_safety_hash()};
    emit(core, 'B', fields, 5);
}

size_t rover_core_drain_tx(struct rover_core *core, uint8_t *out, size_t cap)
{
    size_t take = 0;
    for (size_t i = 0; i < core->tx_len && i < cap; i++) {
        if (core->tx[i] == '\n') {
            take = i + 1; /* whole lines only */
        }
    }
    for (size_t i = 0; i < take; i++) {
        out[i] = core->tx[i];
    }
    for (size_t i = take; i < core->tx_len; i++) {
        core->tx[i - take] = core->tx[i];
    }
    core->tx_len = (uint16_t)(core->tx_len - take);
    return take;
}

/* ---- faults ------------------------------------------------------------ */

void rover_core_set_fault(struct rover_core *core, uint32_t bits)
{
    uint32_t added = bits & ~core->fault;
    if (added == 0) {
        return;
    }
    core->fault |= added;
    rover_core_event(core, ROVER_EVENT_FAULT_SET, (int32_t)added);
}

void rover_core_clear_fault(struct rover_core *core, uint32_t bits)
{
    uint32_t removed = bits & core->fault;
    if (removed == 0) {
        return;
    }
    core->fault &= ~removed;
    rover_core_event(core, ROVER_EVENT_FAULT_CLEARED, (int32_t)removed);
}

/** True while the named latched bit still has a live cause, so a `C` naming it
 *  must be refused (5.1). */
static bool cause_persists(const struct rover_core *core, uint32_t bit)
{
    switch (bit) {
    case ROVER_FAULT_ESTOP:
        return (core->ctrl_flags & ROVER_CF_ESTOP_RELEASED) == 0;
    case ROVER_FAULT_UNDERVOLT_S:
        return core->v_oc_mv < ROVER_VBAT_STOP_MV;
    case ROVER_FAULT_UNDERVOLT_D:
        return core->v_oc_mv < ROVER_VBAT_DISABLE_MV;
    case ROVER_FAULT_OVERCURRENT:
        /* The I2t integral has no decay term, so judging the cause on the
         * integral made A24's designed trip unclearable: the reset below sits
         * behind this test and could never run.  The INA226 mirror re-raises
         * the bit on the next step if the current really is still over 6 A. */
        return false;
    case ROVER_FAULT_STALL:
        return core->wheel[ROVER_LEFT].hard_ms >= ROVER_STALL_HARD_MS ||
               core->wheel[ROVER_RIGHT].hard_ms >= ROVER_STALL_HARD_MS ||
               core->wheel[ROVER_LEFT].slip_ms >= ROVER_STALL_SLIP_MS ||
               core->wheel[ROVER_RIGHT].slip_ms >= ROVER_STALL_SLIP_MS;
    case ROVER_FAULT_ENC_IMPLAUS:
        return core->wheel[ROVER_LEFT].implaus_cycles >= ROVER_ENC_IMPLAUS_CYCLES ||
               core->wheel[ROVER_RIGHT].implaus_cycles >= ROVER_ENC_IMPLAUS_CYCLES;
    case ROVER_FAULT_LOOP_OVERRUN:
        return core->loop_late_pct > ROVER_LOOP_LATE_PCT;
    case ROVER_FAULT_DRIVER_HOT:
        return false; /* the NTC re-raises it on the next step if it is still hot */
    default:
        /* LINK_CRC, WDT_REBOOT, BROWNOUT, DRIVER_FAULT, OBSTACLE_LATCHED: the
         * cause is a past event, so naming the bit is enough.  OBSTACLE_LATCHED
         * in particular must not be conditioned on the obstacle still being
         * there -- V is refused while it is set, so a clear refused until the
         * rover backs away is a deadlock with no exit. */
        return false;
    }
}

/* ---- arming ------------------------------------------------------------ */

static rover_reason_t arm_denied_reason(const struct rover_core *core)
{
    if (!core->hello_seen) {
        return ROVER_REASON_BAD_SESSION;
    }
    if ((core->fault & ROVER_FAULT_ESTOP) != 0) {
        return ROVER_REASON_ESTOP_ASSERTED;
    }
    if ((core->fault & (ROVER_FAULT_UNDERVOLT_S | ROVER_FAULT_UNDERVOLT_D)) != 0) {
        return ROVER_REASON_UNDERVOLTAGE;
    }
    if ((core->fault & ROVER_FAULTS_LATCHED) != 0) {
        return ROVER_REASON_FAULT_LATCHED;
    }
    if (rover_iabs32(core->v_meas_mm_s) > ROVER_ARM_MOTION_V_MM_S ||
        rover_iabs32(core->w_meas_mrad_s) > ROVER_ARM_MOTION_W_MRAD_S) {
        return ROVER_REASON_ARM_DENIED_MOVING;
    }
    return ROVER_REASON_NONE;
}

/** Why a V must be refused outright, as opposed to clamped (A5). */
static rover_reason_t velocity_denied_reason(const struct rover_core *core)
{
    if ((core->fault & ROVER_FAULT_ESTOP) != 0) {
        return ROVER_REASON_ESTOP_ASSERTED;
    }
    if ((core->fault & (ROVER_FAULT_UNDERVOLT_S | ROVER_FAULT_UNDERVOLT_D)) != 0) {
        return ROVER_REASON_UNDERVOLTAGE;
    }
    if ((core->fault & ROVER_FAULTS_LATCHED) != 0) {
        return ROVER_REASON_FAULT_LATCHED;
    }
    /* A pack reading the MCU cannot trust is blockage, not a clear rail: it
     * stops motion without advancing the undervoltage ladder (A25). */
    if (core->ina_stale) {
        return ROVER_REASON_SENSORS_STALE;
    }
    if (!core->armed) {
        return ROVER_REASON_NOT_ARMED;
    }
    return ROVER_REASON_NONE;
}

static void enter_disarmed(struct rover_core *core)
{
    core->armed = false;
    core->v_target_mm_s = 0;
    core->w_target_mrad_s = 0;
    core->v_flags = 0;
    core->have_v = false;
    core->coast = false;
    /* The cliff baseline is retaken on every *entry* to DISARMED, and forward
     * motion is refused while cal_valid is 0 (4.1).  A `D` or an `H` arriving
     * while already disarmed is not an entry, so it does not cost the 50
     * samples again.
     *
     * Restarting the window does NOT clear cal_valid: 4.1 says cal_valid
     * clears on every *reset*, which is rover_core_init.  Clearing it here
     * blinded forward motion for the 1.5 s the 50 samples take after every
     * routine motion_idle_disarm_ms disarm, and robotd refuses to arm while
     * b6 is clear -- so an ordinary conversational pause was answered
     * `not_ready`.  The standing baseline stays in force until the new median
     * swaps in (cliff_step).
     *
     * A part-filled window survives the bounce: discarding it is what left
     * cal_valid false for ever when robotd re-armed inside that window. */
    if (core->state != ROVER_STATE_DISARMED && core->cal_valid) {
        core->cliff_sample_n = 0;
    }
}

/* ---- down-frame handling ----------------------------------------------- */

static void handle_hello(struct rover_core *core, const rover_frame_t *f)
{
    const uint32_t boot_id = (uint32_t)f->field[ROVER_H_HOST_BOOT_ID];
    const bool new_host = boot_id != core->host_boot_id;
    core->host_boot_id = boot_id;
    core->hello_seen = true;
    /* A hello is a fresh link: the controller disarms and waits for an A, so a
     * robotd restart can never inherit a live setpoint (I-13). */
    enter_disarmed(core);
    if (new_host) {
        rover_core_event(core, ROVER_EVENT_SESSION_RESET, (int32_t)core->session);
    }
}

static void handle_arm(struct rover_core *core, const rover_frame_t *f)
{
    const uint32_t nonce = (uint32_t)f->field[ROVER_A_NONCE];
    rover_reason_t reason = arm_denied_reason(core);
    if (reason != ROVER_REASON_NONE) {
        emit_ack(core, 'A', f->seq, ROVER_ACK_REJECT, reason, nonce);
        rover_core_event(core, ROVER_EVENT_ARM_DENIED, (int32_t)reason);
        return;
    }
    core->armed = true;
    core->coast = false;
    /* MOTOR_EN asserts on the first A of a power cycle and then stays
     * asserted; it is not tracked to arm state, which would weld the one
     * contact the e-stop depends on opening (4.1). */
    core->motor_en = true;
    emit_ack(core, 'A', f->seq, ROVER_ACK_OK, ROVER_REASON_NONE, nonce);
    rover_core_event(core, ROVER_EVENT_ARM_OK, (int32_t)nonce);
}

static void handle_velocity(struct rover_core *core, const rover_frame_t *f)
{
    const int64_t ttl_ms = f->field[ROVER_V_FRAME_TTL_MS];
    if (!rover_frame_ttl_ok(ttl_ms)) {
        /* The sole field a clamp does not apply to: a silently clamped TTL
         * would move the safety deadline, so it is rejected and does not renew
         * the TTL (5.1). */
        emit_ack(core, 'V', f->seq, ROVER_ACK_REJECT,
                 ROVER_REASON_FRAME_TTL_OUT_OF_RANGE, 0);
        return;
    }
    rover_reason_t denied = velocity_denied_reason(core);
    if (denied != ROVER_REASON_NONE) {
        emit_ack(core, 'V', f->seq, ROVER_ACK_REJECT, denied, 0);
        return;
    }

    bool clamped = false;
    int32_t v = rover_clamp_sym((int32_t)f->field[ROVER_V_V_MM_S], ROVER_MAX_V_MM_S,
                                &clamped);
    int32_t w = rover_clamp_sym((int32_t)f->field[ROVER_V_W_MRAD_S],
                                ROVER_MAX_W_MRAD_S, &clamped);
    if (clamped) {
        rover_core_set_fault(core, ROVER_FAULT_CAP_CLAMPED);
        rover_core_event(core, ROVER_EVENT_CAP_CLAMP, v);
        if (core->now_us - core->last_clamp_ack_us >=
            (uint64_t)ROVER_CLAMP_ACK_MIN_MS * 1000u) {
            core->last_clamp_ack_us = core->now_us;
            emit_ack(core, 'V', f->seq, ROVER_ACK_CLAMPED, ROVER_REASON_CAP_EXCEEDED,
                     0);
        }
    } else {
        rover_core_clear_fault(core, ROVER_FAULT_CAP_CLAMPED);
    }

    core->v_target_mm_s = v;
    core->w_target_mrad_s = w;
    /* b2-7 are reserved and must be 0; an unknown bit is ignored rather than
     * honoured, because every defined bit is narrowing-only (I-4). */
    core->v_flags = (uint8_t)f->field[ROVER_V_FLAGS] &
                    (ROVER_VFLAG_REQUIRE_SLOW_ZONE_STOP | ROVER_VFLAG_SERVO_RAIL_EN);
    core->coast = false;
    core->have_v = true;
    core->last_v_us = core->now_us;

    if (ttl_ms == ROVER_FRAME_TTL_IMMEDIATE) {
        core->v_target_mm_s = 0;
        core->w_target_mrad_s = 0;
        core->ttl_deadline_us = core->now_us;
    } else {
        core->ttl_deadline_us = core->now_us + (uint64_t)ttl_ms * 1000u;
        if (core->ttl_expired) {
            core->ttl_expired = false;
            rover_core_clear_fault(core, ROVER_FAULT_TTL);
            rover_core_event(core, ROVER_EVENT_TTL_RECOVERED, 0);
        }
    }
}

static void handle_stop(struct rover_core *core, const rover_frame_t *f)
{
    core->v_target_mm_s = 0;
    core->w_target_mrad_s = 0;
    core->coast = f->field[ROVER_S_MODE] != 0;
    emit_ack(core, 'S', f->seq, ROVER_ACK_OK, ROVER_REASON_NONE, 0);
}

static void handle_clear(struct rover_core *core, const rover_frame_t *f)
{
    const uint32_t mask = (uint32_t)f->field[ROVER_C_MASK];
    /* robotd issues C only for latched bits; the MCU is the sole clearer of
     * the obstacle class, because it owns both the samples and the rule (5.1). */
    uint32_t wanted = mask & ROVER_FAULTS_LATCHED & core->fault;
    uint32_t clearable = 0;
    for (uint32_t bit = 1; bit != 0; bit <<= 1) {
        if ((wanted & bit) != 0 && !cause_persists(core, bit)) {
            clearable |= bit;
        }
    }
    if (clearable != 0) {
        if ((clearable & ROVER_FAULT_OVERCURRENT) != 0) {
            core->wheel[ROVER_LEFT].i2t_ma2_ms = 0;
            core->wheel[ROVER_RIGHT].i2t_ma2_ms = 0;
        }
        /* Releasing the escalation restarts the normal five-clean-sample rule
         * from scratch: the obstacle bits themselves are untouched, so forward
         * stays refused while the wall is still there and reverse is legal
         * again immediately. */
        if ((clearable & ROVER_FAULT_OBSTACLE_LATCHED) != 0) {
            core->obstacle_tries = 0;
            core->obstacle_since_us = 0;
            core->forward_requested = false;
        }
        rover_core_clear_fault(core, clearable);
    }
    const bool all_done = (wanted & ~clearable) == 0;
    emit_ack(core, 'C', f->seq, all_done ? ROVER_ACK_OK : ROVER_ACK_REJECT,
             all_done ? ROVER_REASON_NONE : ROVER_REASON_FAULT_LATCHED, 0);
}

static void handle_frame(struct rover_core *core, const rover_frame_t *f)
{
    const rover_frame_spec_t *spec = rover_frame_spec(f->type);
    if (spec == NULL || !spec->down) {
        core->rx.unknown_type++; /* an up frame is not a command */
        return;
    }
    const bool wildcard_hello =
        f->type == 'H' && f->session == ROVER_SESSION_WILDCARD;
    if (!wildcard_hello && f->session != core->session) {
        core->rx.bad_session++;
        rover_core_set_fault(core, ROVER_FAULT_SESSION);
        return;
    }
    /* last_down never moves backwards, so a replay is always stale -- and a
     * stale or duplicate frame does not renew the TTL (A8, I-2). */
    if (!rover_seq_is_newer(f->seq, core->last_down_seq)) {
        core->rx.stale_seq++;
        return;
    }
    core->last_down_seq = f->seq;
    core->rx.ok++;
    rover_core_clear_fault(core, ROVER_FAULT_SESSION);

    switch (f->type) {
    case 'H':
        handle_hello(core, f);
        break;
    case 'A':
        handle_arm(core, f);
        break;
    case 'D':
        enter_disarmed(core);
        emit_ack(core, 'D', f->seq, ROVER_ACK_OK, ROVER_REASON_NONE, 0);
        break;
    case 'V':
        handle_velocity(core, f);
        break;
    case 'S':
        handle_stop(core, f);
        break;
    case 'C':
        handle_clear(core, f);
        break;
    case 'P': {
        const int64_t fields[2] = {f->field[ROVER_P_PI_MONO_US],
                                   (int64_t)core->now_us};
        emit(core, 'O', fields, 2);
        break;
    }
    default:
        core->rx.unknown_type++;
        break;
    }
}

void rover_core_feed(struct rover_core *core, const uint8_t *rx, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        rover_frame_t frame;
        rover_reason_t reason = ROVER_REASON_NONE;
        if (!rover_line_reader_push(&core->reader, rx[i], &frame, &reason)) {
            continue;
        }
        switch (reason) {
        case ROVER_REASON_NONE:
            handle_frame(core, &frame);
            break;
        case ROVER_REASON_BAD_CRC:
            core->rx.bad_crc++;
            break;
        case ROVER_REASON_UNKNOWN_TYPE:
            core->rx.unknown_type++;
            break;
        case ROVER_REASON_UNSUPPORTED_VERSION:
            core->rx.unsupported_version++;
            break;
        case ROVER_REASON_BAD_SESSION:
            core->rx.bad_session++;
            break;
        default:
            core->rx.bad_length++;
            break;
        }
    }
}

/* ---- the step ---------------------------------------------------------- */

static void odometry(struct rover_core *core, const rover_in_t *in, uint32_t dt_us)
{
    const int32_t ticks[2] = {in->left_ticks, in->right_ticks};
    if (!core->have_ticks) {
        core->have_ticks = true;
        core->last_ticks[0] = ticks[0];
        core->last_ticks[1] = ticks[1];
        return;
    }
    for (int i = 0; i < 2; i++) {
        int32_t delta = (int32_t)((int64_t)ticks[i] - core->last_ticks[i]);
        core->last_ticks[i] = ticks[i];
        core->wheel_mm_s[i] = rover_ticks_to_mm_s(delta, core->cfg.ticks_per_rev,
                                                  core->cfg.wheel_radius_mm, dt_us);
    }
    rover_wheels_to_body(core->wheel_mm_s[ROVER_LEFT], core->wheel_mm_s[ROVER_RIGHT],
                         core->cfg.track_mm, &core->v_meas_mm_s,
                         &core->w_meas_mrad_s);
}

static void ttl_step(struct rover_core *core)
{
    if (!core->have_v) {
        return;
    }
    if (core->now_us > core->ttl_deadline_us) {
        if (!core->ttl_expired) {
            core->ttl_expired = true;
            core->ttl_expired_since_us = core->now_us;
            rover_core_set_fault(core, ROVER_FAULT_TTL);
            rover_core_event(core, ROVER_EVENT_TTL_EXPIRED, 0);
        }
        if (core->armed && core->now_us - core->ttl_expired_since_us >=
                               (uint64_t)ROVER_TTL_DISARM_MS * 1000u) {
            enter_disarmed(core);
        }
    }
}

static void update_state(struct rover_core *core, const rover_in_t *in)
{
    const bool blocking = (core->fault & ROVER_FAULTS_LATCHED) != 0;
    if (blocking) {
        core->armed = false;
    }
    /* MOTOR_EN deasserts only on UNDERVOLT_S/D, on any latched-class fault and
     * on shutdown -- not on D, S, TTL expiry or an obstacle-class fault, all of
     * which are handled by braking the PWM (4.1).  OBSTACLE_LATCHED is the one
     * latched bit whose *cause* is obstacle-class, so it is excluded here: it
     * still enters FAULT and still refuses V, but actuating the 30 A relay on
     * a wall park would cycle the one contact the hardware e-stop depends on
     * opening, which is exactly what 4.1's lifecycle rule forbids. */
    if ((core->fault & (ROVER_FAULTS_LATCHED & ~(uint32_t)ROVER_FAULT_OBSTACLE_LATCHED)) != 0 ||
        (core->fault & (ROVER_FAULT_UNDERVOLT_S | ROVER_FAULT_UNDERVOLT_D)) != 0) {
        core->motor_en = false;
    }

    if ((core->fault & ROVER_FAULT_ESTOP) != 0) {
        core->state = ROVER_STATE_ESTOP;
    } else if (blocking) {
        core->state = ROVER_STATE_FAULT;
    } else if (!core->armed) {
        enter_disarmed(core);
        core->state = ROVER_STATE_DISARMED;
    } else {
        core->state = (core->v_cmd_mm_s != 0 || core->w_cmd_mrad_s != 0)
                          ? ROVER_STATE_ARMED_MOVING
                          : ROVER_STATE_ARMED_IDLE;
    }

    /* The host's SERVO_EN request is ANDed with a live heartbeat -- the last
     * accepted V, never P/O, which are diagnostic-only (5.1). */
    core->servo_en = (core->v_flags & ROVER_VFLAG_SERVO_RAIL_EN) != 0 &&
                     core->have_v && core->now_us - core->last_v_us < 500000u &&
                     core->fault == 0 && in->estop_released != 0;

    /* UNDERVOLT_D: request a clean host halt, then cut the rail.  The >=3 edge
     * confirm on PI_POWEROFF_IN lives in the IDF glue, which owns the pin;
     * this is the 60 s fallback of section 9. */
    if ((core->fault & ROVER_FAULT_UNDERVOLT_D) != 0) {
        if (!core->pi_shutdown_req) {
            core->pi_shutdown_req = true;
            core->shutdown_since_us = core->now_us;
        }
        if (core->now_us - core->shutdown_since_us >=
            (uint64_t)ROVER_POWEROFF_TIMEOUT_MS * 1000u) {
            core->pi_rail_en = false;
        }
    }
}

static void publish(struct rover_core *core, const rover_in_t *in)
{
    if (!core->hello_seen && core->now_us >= core->next_banner_us) {
        core->next_banner_us = core->now_us + ROVER_BANNER_PERIOD_US;
        emit_banner(core);
    }
    if (core->now_us < core->next_telem_us) {
        return;
    }
    core->next_telem_us = core->now_us + ROVER_TELEM_PERIOD_US;

    uint64_t age_us = core->now_us - core->tof_good_us;
    uint32_t age_ms = (uint32_t)(age_us / 1000u);
    if (age_ms > 255u) {
        age_ms = 255u;
    }
    uint32_t drops = rover_rx_dropped(&core->rx);
    if (drops > 0xFFFFu) {
        drops = 0xFFFFu;
    }
    uint32_t front = core->tof_front_mm;
    if (front > 0xFFFFu) {
        front = 0xFFFFu;
    }

    const int64_t fields[ROVER_MAX_FIELDS] = {
        (int64_t)core->now_us,
        core->last_down_seq,
        core->state,
        core->ctrl_flags,
        core->fault,
        core->last_ticks[ROVER_LEFT],
        core->last_ticks[ROVER_RIGHT],
        core->v_meas_mm_s,
        core->w_meas_mrad_s,
        core->v_cmd_mm_s,
        core->w_cmd_mrad_s,
        in->vbat_mv,
        in->imotor_ma,
        front,
        in->tof_cliff_mm,
        age_ms,
        core->loop_late_pct,
        drops,
        in->gyro_z_mrad_s,
        core->servo_en ? 1 : 0,
        (core->v_cmd_mm_s != 0 || core->w_cmd_mrad_s != 0) ? 1 : 0,
    };
    emit(core, 'T', fields, ROVER_MAX_FIELDS);

    if (core->loop_late_pct > ROVER_LOOP_LATE_PCT) {
        if (++core->loop_late_frames >= ROVER_LOOP_LATE_FRAMES) {
            rover_core_set_fault(core, ROVER_FAULT_LOOP_OVERRUN);
            rover_core_event(core, ROVER_EVENT_LOOP_OVERRUN, core->loop_late_pct);
            core->loop_late_frames = 0;
        }
    } else {
        core->loop_late_frames = 0;
    }
}

void rover_core_init(rover_core_t *core, const rover_cfg_t *cfg, uint32_t session,
                     uint64_t now_us)
{
    zero_bytes(core, sizeof *core);
    core->cfg = *cfg;
    /* SESS is uint16 and the MCU never mints the wildcard. */
    core->session = (uint16_t)(session & 0xFFFFu);
    if (core->session == ROVER_SESSION_WILDCARD) {
        core->session = 1;
    }
    rover_line_reader_init(&core->reader);
    core->now_us = now_us;
    core->last_step_us = now_us;
    core->next_telem_us = now_us;
    core->next_banner_us = now_us;
    core->link_window_us = now_us;
    core->tof_good_us = now_us;
    core->tof_sample_due_us = now_us;
    core->state = ROVER_STATE_BOOT;
    core->brake = true;
    core->pi_rail_en = true; /* fail-safe: only an actively driven low kills it */
    core->tof_front_mm = ROVER_TOF_ERROR_MM;

    if (core->cfg.debug_build) {
        core->ctrl_flags |= ROVER_CF_DEBUG_BUILD;
    }
    if (core->cfg.wdt_reboot) {
        core->fault |= ROVER_FAULT_WDT_REBOOT;
    }
    if (core->cfg.brownout) {
        core->fault |= ROVER_FAULT_BROWNOUT;
    }
    emit_banner(core);
    core->next_banner_us = now_us + ROVER_BANNER_PERIOD_US;
    if (core->cfg.wdt_reboot) {
        rover_core_event(core, ROVER_EVENT_WDT_REBOOT, core->cfg.reset_reason);
    }
    if (core->cfg.brownout) {
        rover_core_event(core, ROVER_EVENT_BROWNOUT, core->cfg.reset_reason);
    }
}

void rover_core_step(rover_core_t *core, uint64_t now_us, const rover_in_t *in,
                     rover_out_t *out)
{
    uint64_t elapsed = now_us > core->last_step_us ? now_us - core->last_step_us : 0;
    uint32_t dt_us = elapsed > 1000000u ? 1000000u : (uint32_t)elapsed;
    if (dt_us == 0) {
        dt_us = ROVER_CTRL_PERIOD_US;
    }
    core->now_us = now_us;
    core->last_step_us = now_us;

    /* loop_late_pct over a rolling window of control steps; the latch needs
     * five consecutive telemetry frames above the threshold (5.1). */
    if (dt_us > ROVER_CTRL_PERIOD_US + ROVER_CTRL_PERIOD_US / 2) {
        core->loop_late++;
    }
    if (++core->loop_steps >= ROVER_LOOP_WINDOW) {
        core->loop_late_pct =
            (uint8_t)((core->loop_late * 100u) / ROVER_LOOP_WINDOW);
        core->loop_steps = 0;
        core->loop_late = 0;
    }

    if (core->state == ROVER_STATE_BOOT) {
        enter_disarmed(core);
        core->state = ROVER_STATE_DISARMED;
    }

    odometry(core, in, dt_us);
    rover_safety_step(core, in, dt_us);
    ttl_step(core);
    update_state(core, in);

    const bool blocking = (core->fault & ROVER_FAULTS_LATCHED) != 0;
    const bool want_zero = !core->armed || blocking || core->ttl_expired ||
                           core->ina_stale ||
                           (core->v_target_mm_s == 0 && core->w_target_mrad_s == 0);
    /* MDD3A brake is both inputs high, and PWM zero is not the terminal state
     * (I-1) -- so the brake asserts only once the abort ramp has reached zero. */
    core->brake = want_zero && !core->coast && core->v_cmd_mm_s == 0 &&
                  core->w_cmd_mrad_s == 0;

    rover_control_step(core, dt_us);

    uint16_t flags = 0;
    if (!core->ttl_expired) {
        flags |= ROVER_CF_TTL_OK;
    }
    if (in->estop_released) {
        flags |= ROVER_CF_ESTOP_RELEASED;
    }
    if (in->bumper_clear) {
        flags |= ROVER_CF_BUMPER_CLEAR;
    }
    if ((core->fault & (ROVER_FAULT_TOF_STOP | ROVER_FAULT_TOF_STALE)) == 0) {
        flags |= ROVER_CF_TOF_CLEAR;
    }
    if (core->motor_en && !core->brake) {
        flags |= ROVER_CF_PWM_ENABLED;
    }
    if (core->in_slow_zone) {
        flags |= ROVER_CF_IN_SLOW_ZONE;
    }
    if (core->cal_valid) {
        flags |= ROVER_CF_CAL_VALID;
    }
    if (core->cfg.debug_build) {
        flags |= ROVER_CF_DEBUG_BUILD;
    }
    if (core->tof_ok[0]) {
        flags |= ROVER_CF_TOF_FL_OK;
    }
    if (core->tof_ok[1]) {
        flags |= ROVER_CF_TOF_FR_OK;
    }
    core->ctrl_flags = flags;

    publish(core, in);

    out->duty_l_q15 = core->duty_q15[ROVER_LEFT];
    out->duty_r_q15 = core->duty_q15[ROVER_RIGHT];
    out->brake = core->brake ? 1u : 0u;
    out->motor_en = core->motor_en ? 1u : 0u;
    out->servo_en = core->servo_en ? 1u : 0u;
    out->pi_rail_en = core->pi_rail_en ? 1u : 0u;
    out->pi_shutdown_req = core->pi_shutdown_req ? 1u : 0u;
    out->state = core->state;
    out->fault = core->fault;
    out->ctrl_flags = core->ctrl_flags;
}

uint16_t rover_core_session(const rover_core_t *core)
{
    return core->session;
}

const rover_rx_counters_t *rover_core_counters(const rover_core_t *core)
{
    return &core->rx;
}

int16_t rover_core_v_cmd_mm_s(const rover_core_t *core)
{
    return (int16_t)core->v_cmd_mm_s;
}

int16_t rover_core_w_cmd_mrad_s(const rover_core_t *core)
{
    return (int16_t)core->w_cmd_mrad_s;
}

int16_t rover_core_v_meas_mm_s(const rover_core_t *core)
{
    return (int16_t)core->v_meas_mm_s;
}

uint64_t rover_core_ttl_deadline_us(const rover_core_t *core)
{
    return core->ttl_deadline_us;
}
