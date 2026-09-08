/* Cap clamping, the asymmetric slew limiter, differential kinematics and the
 * per-wheel PI controller.
 *
 * Everything is integer: at 0.30 m/s a 90 mm wheel gives ~23 counts per 10 ms
 * window (4.1), so millimetres per second is already finer than the encoder
 * resolves and a float would buy only rounding differences between the S3 and
 * the host build.
 */
#include "rover_internal.h"

int32_t rover_clamp_sym(int32_t value, int32_t limit, bool *clamped)
{
    if (value > limit) {
        if (clamped != NULL) {
            *clamped = true;
        }
        return limit;
    }
    if (value < -limit) {
        if (clamped != NULL) {
            *clamped = true;
        }
        return -limit;
    }
    return value;
}

int32_t rover_slew(int32_t current, int32_t target, int32_t up_step,
                   int32_t down_step)
{
    if (target == current) {
        return current;
    }
    /* A reversal passes through zero.  Stopping there first is what keeps the
     * abort ramp from accelerating the other way in one step, and it costs one
     * control period. */
    if ((current > 0 && target < 0) || (current < 0 && target > 0)) {
        target = 0;
    }
    int32_t step = (rover_iabs32(target) > rover_iabs32(current)) ? up_step : down_step;
    if (step < 0) {
        step = 0;
    }
    if (target > current) {
        int32_t next = current + step;
        return next > target ? target : next;
    }
    int32_t next = current - step;
    return next < target ? target : next;
}

void rover_body_to_wheels(int32_t v_mm_s, int32_t w_mrad_s, uint16_t track_mm,
                          int32_t *left_mm_s, int32_t *right_mm_s)
{
    /* w is mrad/s and the track is mm, so half the differential speed is
     * w * track / 2000 mm/s. */
    int32_t half = (int32_t)(((int64_t)w_mrad_s * track_mm) / 2000);
    *left_mm_s = v_mm_s - half;
    *right_mm_s = v_mm_s + half;
}

void rover_wheels_to_body(int32_t left_mm_s, int32_t right_mm_s, uint16_t track_mm,
                          int32_t *v_mm_s, int32_t *w_mrad_s)
{
    *v_mm_s = (left_mm_s + right_mm_s) / 2;
    *w_mrad_s = track_mm == 0
                    ? 0
                    : (int32_t)(((int64_t)(right_mm_s - left_mm_s) * 1000) / track_mm);
}

int32_t rover_ticks_to_mm_s(int32_t dticks, uint16_t ticks_per_rev,
                            uint16_t wheel_radius_mm, uint32_t dt_us)
{
    if (dt_us == 0 || ticks_per_rev == 0) {
        return 0;
    }
    uint32_t circumference_um =
        (uint32_t)(((uint64_t)wheel_radius_mm * 6283185u) / 1000u);
    return (int32_t)(((int64_t)dticks * circumference_um * 1000) /
                     ((int64_t)ticks_per_rev * dt_us));
}

void rover_pi_reset(rover_pi_t *pi)
{
    pi->integral = 0;
}

int16_t rover_pi_step(rover_pi_t *pi, int32_t target_mm_s, int32_t meas_mm_s)
{
    const int32_t integral_max =
        (int32_t)(((int64_t)ROVER_DUTY_MAX_Q15 * 256) / ROVER_PI_KI_Q8);
    int32_t error = target_mm_s - meas_mm_s;

    pi->integral += error;
    if (pi->integral > integral_max) {
        pi->integral = integral_max;
    } else if (pi->integral < -integral_max) {
        pi->integral = -integral_max;
    }

    int64_t duty = ((int64_t)ROVER_PI_KP_Q8 * error +
                    (int64_t)ROVER_PI_KI_Q8 * pi->integral) /
                   256;
    if (duty > ROVER_DUTY_MAX_Q15) {
        duty = ROVER_DUTY_MAX_Q15;
    } else if (duty < -ROVER_DUTY_MAX_Q15) {
        duty = -ROVER_DUTY_MAX_Q15;
    }
    return (int16_t)duty;
}

int32_t rover_slow_zone_v_cap(uint32_t front_mm)
{
    /* 65534 is "no target within range": a clear path, not a fault (I-16). */
    if (front_mm == ROVER_TOF_NO_TARGET_MM || front_mm > ROVER_TOF_SLOW_MM) {
        return ROVER_MAX_V_MM_S;
    }
    if (front_mm <= ROVER_TOF_STOP_MM) {
        return 0;
    }
    /* A21's time-to-collision law: v <= (d - 250 mm) / 1.0 s, which in mm/s is
     * the millimetre difference itself, capped at 150 mm/s. */
    int32_t cap = (int32_t)front_mm - ROVER_TOF_STOP_MM;
    return cap > ROVER_SLOW_ZONE_V_CAP_MM_S ? ROVER_SLOW_ZONE_V_CAP_MM_S : cap;
}

/* ---- the 100 Hz control step ------------------------------------------- */

void rover_control_step(struct rover_core *core, uint32_t dt_us)
{
    const bool armed = core->state == ROVER_STATE_ARMED_IDLE ||
                       core->state == ROVER_STATE_ARMED_MOVING;
    const bool blocking = (core->fault & ROVER_FAULTS_LATCHED) != 0 ||
                          core->state == ROVER_STATE_ESTOP;

    int32_t v = core->v_target_mm_s;
    int32_t w = core->w_target_mrad_s;

    if (!armed || blocking || core->ttl_expired || core->ina_stale) {
        v = 0;
        w = 0;
    } else {
        /* cal_valid is 0 until the cliff baseline exists, and while it is 0 the
         * MCU refuses all forward motion (4.1). */
        if (!core->cal_valid && v > 0) {
            v = 0;
        }
        if ((core->fault & ROVER_FAULTS_OBSTACLE) != 0) {
            /* Forward zeroed, |w| clamped, reverse clamped -- reverse and
             * rotation stay legal so the rover can back out (I-5). */
            if (v > 0) {
                v = 0;
            }
            if (v < -ROVER_REVERSE_CLAMP_MM_S) {
                v = -ROVER_REVERSE_CLAMP_MM_S;
            }
            w = rover_clamp_sym(w, ROVER_SLOW_ZONE_W_MRAD_S, NULL);
        } else if (core->in_slow_zone) {
            if ((core->v_flags & ROVER_VFLAG_REQUIRE_SLOW_ZONE_STOP) != 0) {
                if (v > 0) {
                    v = 0; /* b0 = 1 refuses forward inside the slow zone entirely */
                }
            } else {
                int32_t cap = rover_slow_zone_v_cap(core->tof_front_mm);
                if (v > cap) {
                    v = cap;
                }
            }
            w = rover_clamp_sym(w, ROVER_SLOW_ZONE_W_MRAD_S, NULL);
        }
    }

    /* Asymmetric: increases at the compiled slew cap, every decrease at the
     * abort ramp -- TTC clamp, obstacle zeroing, TTL expiry, S and faults
     * alike (4.1). */
    int32_t up_v = (int32_t)(((int64_t)ROVER_ACCEL_MM_S2 * dt_us) / 1000000);
    int32_t down_v = (int32_t)(((int64_t)ROVER_ABORT_DECEL_MM_S2 * dt_us) / 1000000);
    int32_t up_w = (int32_t)(((int64_t)ROVER_ALPHA_MRAD_S2 * dt_us) / 1000000);
    int32_t down_w =
        (int32_t)(((int64_t)ROVER_ABORT_ALPHA_MRAD_S2 * dt_us) / 1000000);

    core->v_cmd_mm_s = rover_slew(core->v_cmd_mm_s, v, up_v, down_v);
    core->w_cmd_mrad_s = rover_slew(core->w_cmd_mrad_s, w, up_w, down_w);

    rover_body_to_wheels(core->v_cmd_mm_s, core->w_cmd_mrad_s, core->cfg.track_mm,
                         &core->wheel_cmd_mm_s[ROVER_LEFT],
                         &core->wheel_cmd_mm_s[ROVER_RIGHT]);

    /* S mode 1 is coast: duty zero with the brake released, so the PI must
     * not be allowed to decelerate the wheels actively. */
    const bool drive =
        core->motor_en && !core->brake && !core->coast && armed && !blocking;
    for (int i = 0; i < 2; i++) {
        if (!drive) {
            rover_pi_reset(&core->pi[i]);
            core->duty_q15[i] = 0;
            core->wheel_cmd_mm_s[i] = 0;
        } else {
            core->duty_q15[i] = rover_pi_step(&core->pi[i], core->wheel_cmd_mm_s[i],
                                              core->wheel_mm_s[i]);
        }
    }
}
