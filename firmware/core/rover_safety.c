/* Sensor interpretation and the fault classifier.
 *
 * Three classes, exactly as ARCHITECTURE 5.1 defines them: advisory bits are
 * set and cleared freely; obstacle-class bits block forward, clamp |w| and
 * reverse, do not enter FAULT and self-clear after five clean samples; every
 * other bit latches and needs an explicit `C` once its cause is gone.
 *
 * The ToF confirm and clear counters are gated to the sensor's own
 * inter-measurement period rather than to the 100 Hz control step: the core is
 * handed the same reading several times between samples, so counting steps
 * would turn A21's two-sample stop confirm into a 20 ms debounce that confirms
 * nothing.
 */
#include "rover_internal.h"

static uint32_t sat_add_ms(uint32_t value, uint32_t dt_ms, uint32_t ceiling)
{
    if (value >= ceiling) {
        return ceiling;
    }
    return (ceiling - value < dt_ms) ? ceiling : value + dt_ms;
}

/** Median of the cliff calibration window, by insertion sort on a copy. */
static uint16_t median_u16(const uint16_t *values, uint8_t n)
{
    uint16_t sorted[ROVER_CLIFF_SAMPLES];
    for (uint8_t i = 0; i < n; i++) {
        uint16_t v = values[i];
        uint8_t j = i;
        while (j > 0 && sorted[j - 1] > v) {
            sorted[j] = sorted[j - 1];
            j--;
        }
        sorted[j] = v;
    }
    return sorted[n / 2];
}

/* ---- forward ToF ------------------------------------------------------- */

static void tof_step(struct rover_core *core, const rover_in_t *in, bool sample)
{
    const bool fl_ok = (in->tof_status & ROVER_TOF_BIT_FL) == 0 &&
                       in->tof_fl_mm != ROVER_TOF_ERROR_MM;
    const bool fr_ok = (in->tof_status & ROVER_TOF_BIT_FR) == 0 &&
                       in->tof_fr_mm != ROVER_TOF_ERROR_MM;

    for (int i = 0; i < 2; i++) {
        bool ok = i == 0 ? fl_ok : fr_ok;
        if (ok != core->tof_ok[i]) {
            core->tof_ok[i] = ok;
            rover_core_event(core, ROVER_EVENT_TOF_STATUS, i);
            /* An I2C failure and a genuinely blocked path are the one
             * discrimination 4.1's coverage rule turns on, so the bus error
             * gets its own event rather than hiding inside TOF_STATUS. */
            if (!ok && (in->tof_status & (uint8_t)(1u << i)) != 0) {
                rover_core_event(core, ROVER_EVENT_I2C_ERROR, i);
            }
        }
    }

    /* tof_front_mm is a min() over the valid readings, but coverage is not:
     * one dead sensor halves an already-marginal cone, so forward is refused
     * whatever the survivor reports (4.1, I-16). */
    uint32_t front = ROVER_TOF_ERROR_MM;
    if (fl_ok && fr_ok) {
        front = in->tof_fl_mm < in->tof_fr_mm ? in->tof_fl_mm : in->tof_fr_mm;
    } else if (fl_ok) {
        front = in->tof_fl_mm;
    } else if (fr_ok) {
        front = in->tof_fr_mm;
    }
    core->tof_front_mm = front;

    const bool coverage = fl_ok && fr_ok;
    if (coverage) {
        core->tof_good_us = core->now_us;
    }
    core->in_slow_zone = coverage && front != ROVER_TOF_NO_TARGET_MM &&
                         front <= ROVER_TOF_SLOW_MM;

    if (!sample) {
        return;
    }

    /* TOF_STALE: either sensor stale or erroring refuses forward at once; it
     * clears only after five consecutive clean samples. */
    if (!coverage) {
        core->tof_stale_clear_count = 0;
        rover_core_set_fault(core, ROVER_FAULT_TOF_STALE);
    } else if ((core->fault & ROVER_FAULT_TOF_STALE) != 0) {
        if (++core->tof_stale_clear_count >= ROVER_TOF_CLEAR_SAMPLES) {
            rover_core_clear_fault(core, ROVER_FAULT_TOF_STALE);
            core->tof_stale_clear_count = 0;
        }
    }

    /* TOF_STOP: two samples to stop, five clean samples to clear (A21, 5.1). */
    const bool blocked = coverage && front != ROVER_TOF_NO_TARGET_MM &&
                         front <= ROVER_TOF_STOP_MM;
    if (blocked) {
        core->tof_clear_count = 0;
        if (core->tof_stop_count < ROVER_TOF_STOP_SAMPLES) {
            core->tof_stop_count++;
        }
        if (core->tof_stop_count >= ROVER_TOF_STOP_SAMPLES) {
            rover_core_set_fault(core, ROVER_FAULT_TOF_STOP);
        }
    } else if (coverage) {
        core->tof_stop_count = 0;
        if ((core->fault & ROVER_FAULT_TOF_STOP) != 0) {
            if (++core->tof_clear_count >= ROVER_TOF_CLEAR_SAMPLES) {
                rover_core_clear_fault(core, ROVER_FAULT_TOF_STOP);
                core->tof_clear_count = 0;
            }
        }
    }
}

/* ---- cliff ------------------------------------------------------------- */

static void cliff_step(struct rover_core *core, const rover_in_t *in, bool sample)
{
    const bool valid = (in->tof_status & ROVER_TOF_BIT_CLIFF) == 0 &&
                       in->tof_cliff_mm != ROVER_TOF_ERROR_MM &&
                       in->tof_cliff_mm != ROVER_TOF_NO_TARGET_MM;
    const bool bus_ok = (in->tof_status & ROVER_TOF_BIT_CLIFF) == 0;
    if (bus_ok != core->tof_ok[2]) {
        core->tof_ok[2] = bus_ok;
        rover_core_event(core, ROVER_EVENT_TOF_STATUS, 2);
        if (!bus_ok) {
            rover_core_event(core, ROVER_EVENT_I2C_ERROR, 2);
        }
    }
    if (!sample) {
        return;
    }

    /* The 50-sample median is retaken on every entry to DISARMED, and the new
     * median swaps in when the 50th sample lands.  The window accumulates
     * whenever it is not full and the reading is valid, whatever the state:
     * gating it on DISARMED made an arm inside the 1.5 s the window takes
     * leave cal_valid false for ever, and the only symptom was forward
     * silently zeroed (4.1, I-3).  The standing baseline keeps guarding the
     * edge while the new window fills, so a re-baseline costs no blind window
     * -- cal_valid goes false only at a reset. */
    if (core->cliff_sample_n < ROVER_CLIFF_SAMPLES) {
        if (valid) {
            core->cliff_sample[core->cliff_sample_n++] = in->tof_cliff_mm;
        }
        if (core->cliff_sample_n >= ROVER_CLIFF_SAMPLES) {
            core->cliff_baseline_mm =
                median_u16(core->cliff_sample, core->cliff_sample_n);
            core->cal_valid = true;
            rover_core_event(core, ROVER_EVENT_CAL_STORED,
                             (int32_t)core->cliff_baseline_mm);
        }
        if (!core->cal_valid) {
            return;
        }
    }

    /* Downward, an out-of-range return *is* the void, so the sign of the
     * conservative rule inverts relative to the forward sensors (4.1). */
    const bool over =
        !valid ||
        (uint32_t)in->tof_cliff_mm >
            (uint32_t)core->cliff_baseline_mm + ROVER_CLIFF_DELTA_MM;
    if (over) {
        core->cliff_clear_count = 0;
        if (!valid || ++core->cliff_count >= 2) {
            rover_core_set_fault(core, ROVER_FAULT_CLIFF);
        }
    } else {
        core->cliff_count = 0;
        if ((core->fault & ROVER_FAULT_CLIFF) != 0) {
            if (++core->cliff_clear_count >= ROVER_TOF_CLEAR_SAMPLES) {
                rover_core_clear_fault(core, ROVER_FAULT_CLIFF);
                core->cliff_clear_count = 0;
            }
        }
    }
}

/* ---- battery ladder (A25) ---------------------------------------------- */

static void battery_step(struct rover_core *core, const rover_in_t *in,
                         uint32_t dt_ms)
{
    /* A failed INA226 read publishes the sentinel, never 0 mV: a sensor
     * failure is indistinguishable from a flat pack otherwise, and the
     * response to a flat pack is to power the host off.  The ladder freezes
     * where it is -- a stale reading must never advance UNDERVOLT_D. */
    if (in->vbat_mv == ROVER_VBAT_INVALID_MV) {
        if (!core->ina_stale) {
            core->ina_stale = true;
            rover_core_event(core, ROVER_EVENT_I2C_ERROR, ROVER_I2C_INDEX_INA);
        }
        return;
    }
    core->ina_stale = false;

    /* Sag-compensated open-circuit estimate, evaluated at all times: >2 A is
     * normal on carpet, so a load-suspended check would be disabled by exactly
     * the condition it exists to survive. */
    core->v_oc_mv = (int32_t)in->vbat_mv +
                    (int32_t)(((int32_t)in->imotor_ma * core->cfg.r_pack_mohm) / 1000);

    static const int32_t thresholds[3] = {ROVER_VBAT_WARN_MV, ROVER_VBAT_STOP_MV,
                                          ROVER_VBAT_DISABLE_MV};
    static const uint32_t bits[3] = {ROVER_FAULT_UNDERVOLT_W, ROVER_FAULT_UNDERVOLT_S,
                                     ROVER_FAULT_UNDERVOLT_D};
    for (int i = 0; i < 3; i++) {
        if (core->v_oc_mv < thresholds[i]) {
            core->below_ms[i] =
                sat_add_ms(core->below_ms[i], dt_ms, ROVER_VBAT_DEBOUNCE_MS);
            if (core->below_ms[i] >= ROVER_VBAT_DEBOUNCE_MS) {
                rover_core_set_fault(core, bits[i]);
            }
        } else {
            core->below_ms[i] = 0;
        }
    }
    /* Clearing warn and stop needs V_oc 0.3 V above the threshold for 30 s (9).
     * UNDERVOLT_D is terminal: it ends in a host shutdown and a rail cut. */
    for (int i = 0; i < 2; i++) {
        if (core->v_oc_mv >= thresholds[i] + ROVER_VBAT_CLEAR_MARGIN_MV) {
            core->above_ms[i] =
                sat_add_ms(core->above_ms[i], dt_ms, ROVER_VBAT_CLEAR_MS);
            if (core->above_ms[i] >= ROVER_VBAT_CLEAR_MS) {
                rover_core_clear_fault(core, bits[i]);
                core->above_ms[i] = 0;
            }
        } else {
            core->above_ms[i] = 0;
        }
    }
}

/* ---- per-wheel slip, stall, I2t and encoder plausibility (A24, 4.1) ----- */

static void wheel_step(struct rover_core *core, const rover_in_t *in, uint32_t dt_ms)
{
    /* The per-channel current model.  A jammed wheel draws far more current at
     * the same duty as its free partner, so a duty-proportional split reports
     * the pair as equal and a single-channel overload is invisible; the motor
     * model inverts correctly because a jam is low omega at high duty. */
    int32_t model_ma[2] = {0, 0};
    int32_t model_sum = 0;
    const int32_t duty_sum =
        rover_iabs32(core->duty_q15[ROVER_LEFT]) + rover_iabs32(core->duty_q15[ROVER_RIGHT]);
    for (int i = 0; i < 2; i++) {
        int32_t applied_mv =
            (int32_t)(((int64_t)core->duty_q15[i] * in->vbat_mv) / 32768);
        int32_t bemf_mv = (int32_t)(((int64_t)core->cfg.k_e_uv_per_mm_s *
                                     core->wheel_mm_s[i]) /
                                    1000);
        int32_t across_mv = applied_mv - bemf_mv;
        model_ma[i] = core->cfg.r_motor_mohm == 0
                          ? 0
                          : rover_iabs32((int32_t)(((int64_t)across_mv * 1000) /
                                             core->cfg.r_motor_mohm));
        model_sum += model_ma[i];
    }

    const int32_t total_ma = rover_iabs32(in->imotor_ma);
    for (int i = 0; i < 2; i++) {
        int32_t i_ch = 0;
        if (duty_sum >= ROVER_I2T_DEAD_DUTY_Q15 && model_sum > 0) {
            /* Rescaled so the pair sums to the one current the INA226 sees. */
            i_ch = (int32_t)(((int64_t)model_ma[i] * total_ma) / model_sum);
        }
        int32_t excess = i_ch - ROVER_I2T_KNEE_MA;
        if (excess > 0) {
            core->wheel[i].i2t_ma2_ms +=
                (uint64_t)((int64_t)excess * excess) * dt_ms;
            if (core->wheel[i].i2t_ma2_ms >= ROVER_I2T_LIMIT_MA2_MS) {
                rover_core_set_fault(core, ROVER_FAULT_OVERCURRENT);
            }
        }

        const int32_t duty_pct = (rover_iabs32(core->duty_q15[i]) * 100) / ROVER_DUTY_MAX_Q15;
        const int32_t cmd = rover_iabs32(core->wheel_cmd_mm_s[i]);
        const int32_t meas = rover_iabs32(core->wheel_mm_s[i]);
        const bool stalling = duty_pct > ROVER_STALL_DUTY_PCT && cmd > 0 &&
                              meas * 100 < ROVER_STALL_HARD_PCT * cmd;
        if (duty_pct > ROVER_STALL_DUTY_PCT && cmd > 0) {
            if (meas * 100 < ROVER_STALL_HARD_PCT * cmd) {
                core->wheel[i].hard_ms =
                    (uint16_t)sat_add_ms(core->wheel[i].hard_ms, dt_ms, 65535);
            } else {
                core->wheel[i].hard_ms = 0;
            }
            if (meas * 100 < ROVER_STALL_SLIP_PCT * cmd) {
                core->wheel[i].slip_ms =
                    (uint16_t)sat_add_ms(core->wheel[i].slip_ms, dt_ms, 65535);
            } else {
                core->wheel[i].slip_ms = 0;
            }
        } else {
            core->wheel[i].hard_ms = 0;
            core->wheel[i].slip_ms = 0;
        }
        if (core->wheel[i].hard_ms >= ROVER_STALL_HARD_MS ||
            core->wheel[i].slip_ms >= ROVER_STALL_SLIP_MS) {
            if ((core->fault & ROVER_FAULT_STALL) == 0) {
                rover_core_event(core, ROVER_EVENT_STALL, i);
            }
            rover_core_set_fault(core, ROVER_FAULT_STALL);
        }

        /* Encoder plausibility: separate from the slip detector, and what
         * catches lift-off and a rug edge (4.1).  A wheel inside A24's stall
         * window is held out of it -- both rules run on 20 cycles, and the
         * looser 20% duty gate would otherwise always win the race and report
         * a jam as an encoder fault, which is not what I-10 asks for. */
        if (!stalling && duty_pct > ROVER_ENC_IMPLAUS_DUTY_PCT && cmd > 0 &&
            rover_iabs32(cmd - meas) * 100 > ROVER_ENC_IMPLAUS_PCT * cmd) {
            if (core->wheel[i].implaus_cycles < 0xFFFF) {
                core->wheel[i].implaus_cycles++;
            }
            if (core->wheel[i].implaus_cycles >= ROVER_ENC_IMPLAUS_CYCLES) {
                rover_core_set_fault(core, ROVER_FAULT_ENC_IMPLAUS);
            }
        } else {
            core->wheel[i].implaus_cycles = 0;
        }
    }

    if (total_ma >= ROVER_OVERCURRENT_MA) {
        rover_core_set_fault(core, ROVER_FAULT_OVERCURRENT);
    }
}

/* ---- obstacle-class escalation (5.1) ----------------------------------- */

static void escalation_step(struct rover_core *core)
{
    if ((core->fault & ROVER_FAULTS_OBSTACLE) == 0) {
        core->obstacle_since_us = 0;
        core->obstacle_tries = 0;
        core->forward_requested = false;
        return;
    }
    if (core->obstacle_since_us == 0) {
        core->obstacle_since_us = core->now_us;
        core->obstacle_tries = 0;
    }
    /* Escalation is not timed on the obstacle itself: stopping at 250 mm
     * leaves the cause present by definition, so a 2 s rule would latch every
     * wall approach.  It takes two distinct refused pushes forward, or 30 s
     * with no accepted reverse or rotate. */
    const bool forward = core->v_target_mm_s > 0;
    const bool escape = core->v_target_mm_s < 0 || core->w_target_mrad_s != 0;
    if (escape) {
        core->obstacle_since_us = core->now_us;
        core->obstacle_tries = 0;
    } else if (forward && !core->forward_requested) {
        if (core->obstacle_tries < 0xFF) {
            core->obstacle_tries++;
        }
    }
    core->forward_requested = forward;

    if (core->obstacle_tries >= ROVER_OBSTACLE_ESCALATE_TRIES ||
        core->now_us - core->obstacle_since_us >=
            (uint64_t)ROVER_OBSTACLE_ESCALATE_S * 1000000u) {
        rover_core_set_fault(core, ROVER_FAULT_OBSTACLE_LATCHED);
    }
}

/* ---- link and loop health ---------------------------------------------- */

static void health_step(struct rover_core *core, const rover_in_t *in)
{
    if (core->now_us - core->link_window_us >= 1000000u) {
        uint32_t dropped = rover_rx_dropped(&core->rx);
        /* A5's premise is that corrupted frames arrive routinely, so LINK_CRC
         * latches on a rate, never on one event. */
        if (dropped - core->link_window_drops > ROVER_LINK_CRC_DROPS_PER_S) {
            rover_core_set_fault(core, ROVER_FAULT_LINK_CRC);
        }
        core->link_window_drops = dropped;
        core->link_window_us = core->now_us;
    }
    if (in->ntc_c >= ROVER_DRIVER_HOT_C) {
        rover_core_set_fault(core, ROVER_FAULT_DRIVER_HOT);
    }
}

void rover_safety_step(struct rover_core *core, const rover_in_t *in, uint32_t dt_us)
{
    const uint32_t dt_ms = dt_us / 1000u;

    /* One sensor sample per inter-measurement period, whatever the control
     * rate; see the file header. */
    bool sample = false;
    if (core->now_us >= core->tof_sample_due_us) {
        sample = true;
        core->tof_sample_due_us =
            core->now_us + (uint64_t)ROVER_TOF_INTER_PERIOD_MS * 1000u;
    }

    if (!in->estop_released) {
        rover_core_set_fault(core, ROVER_FAULT_ESTOP);
    }

    /* Two NC switches in series with a pull-down: at rest the line is high, and
     * a press, a broken wire or an unseated connector all pull it low (4.1). */
    if (!in->bumper_clear) {
        core->bumper_clear_count = 0;
        rover_core_set_fault(core, ROVER_FAULT_BUMPER);
    } else if ((core->fault & ROVER_FAULT_BUMPER) != 0 && sample) {
        if (++core->bumper_clear_count >= ROVER_TOF_CLEAR_SAMPLES) {
            rover_core_clear_fault(core, ROVER_FAULT_BUMPER);
            core->bumper_clear_count = 0;
        }
    }

    tof_step(core, in, sample);
    cliff_step(core, in, sample);
    battery_step(core, in, dt_ms);
    wheel_step(core, in, dt_ms);
    escalation_step(core);
    health_step(core, in);
}
