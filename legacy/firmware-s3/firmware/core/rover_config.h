/* Compiled caps and thresholds.  Every constant names the ARCHITECTURE
 * decision or section it comes from.
 *
 * No frame may raise any of these (I-4).  The wire grammar has no field that
 * widens a bound: over-cap values are clamped and flagged, never rejected
 * (A5), and every `V.flags` bit is narrowing-only.
 *
 * The seven constants marked SAFETY_HASH are the ones B.safety_hash is a
 * CRC-32 over, in that exact order (5.1).  Editing one here changes the banner
 * and makes deploy/preflight.sh fail against config/robot.toml's [safety]
 * mirror -- which is the point of the field.
 *
 * Trailing comments are `//` deliberately: tests/unit/test_caps_match.py reads
 * this header to prove every robotd bound sits inside a controller cap, and
 * that is the comment form its parser accepts.
 */
#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

/* ---- protocol (A5, 5.1) ------------------------------------------------ */
#define ROVER_PROTO_VER 2         // VER; a mismatch is reason 6, no negotiation
#define ROVER_MAX_LINE_BYTES 200  // longest legal line, excluding the newline
#define ROVER_SESSION_WILDCARD 0  // only H may carry it; the MCU never mints 0

/* ---- velocity caps (A6, 5.1 "V fields and MCU-accepted ranges") -------- */
#define ROVER_MAX_V_MM_S 300      // +x forward; the 0.30 m/s every A21 number derives at
#define ROVER_MAX_W_MRAD_S 1200   // +z up = CCW

/* ---- command TTL (A19 T1, 5.1) ---------------------------------------- */
#define ROVER_FRAME_TTL_IMMEDIATE 0 // 0 means stop now
#define ROVER_FRAME_TTL_MIN_MS 50   // outside {0} U [50,500] the frame is REJECTED,
#define ROVER_FRAME_TTL_MAX_MS 500  // reason 14 -- the one field a clamp cannot apply to
#define ROVER_TTL_DISARM_MS 5000    // TTL continuously expired this long -> DISARMED

/* ---- ToF zones.  SAFETY_HASH, in order (A21, 4.1) ---------------------- */
#define ROVER_TOF_STOP_MM 250          // SAFETY_HASH[0]
#define ROVER_TOF_SLOW_MM 600          // SAFETY_HASH[1]
#define ROVER_SLOW_ZONE_W_MRAD_S 500   // SAFETY_HASH[2]
#define ROVER_TOF_TIMING_BUDGET_MS 20  // SAFETY_HASH[3], the short-mode floor
#define ROVER_TOF_INTER_PERIOD_MS 30   // SAFETY_HASH[4]
#define ROVER_TOF_POLL_HZ 50           // SAFETY_HASH[5], the sensor task rate
#define ROVER_CLIFF_DELTA_MM 80        // SAFETY_HASH[6]

#define ROVER_SLOW_ZONE_V_CAP_MM_S 150 // A21 hard cap inside the slow zone
#define ROVER_TOF_STALE_MS 200         // I-16: a sample older than this is 65535
#define ROVER_TOF_NO_TARGET_MM 65534   // no target within range: a clear path (I-16)
#define ROVER_TOF_ERROR_MM 65535       // sensor/I2C error or a stale sample
#define ROVER_TOF_STOP_SAMPLES 2       // A21: two samples to stop
#define ROVER_TOF_CLEAR_SAMPLES 5      // 5.1: obstacle bits clear after 5 clean samples
#define ROVER_CLIFF_SAMPLES 50         // 4.1: median of 50 on every entry to DISARMED

/* ---- obstacle class (5.1 "Obstacle-class, blocking but self-clearing") - */
#define ROVER_REVERSE_CLAMP_MM_S 150    // I-5: reverse is unsensed, so it is clamped
#define ROVER_OBSTACLE_ESCALATE_S 30    // [safety] obstacle_escalate_s
#define ROVER_OBSTACLE_ESCALATE_TRIES 2 // ">=2 rejected escape attempts"

/* ---- control loop and slew (4.1, 5.8 [limits]) ------------------------- */
#define ROVER_CTRL_HZ 100
#define ROVER_CTRL_PERIOD_US 10000
#define ROVER_TELEM_HZ 50
#define ROVER_TELEM_PERIOD_US 20000
#define ROVER_BANNER_PERIOD_US 1000000 // B at 1 Hz until the first valid H

#define ROVER_ACCEL_MM_S2 500          // [limits] accel_mps2 = 0.5, increases only
#define ROVER_ALPHA_MRAD_S2 1000       // [limits] alpha_radps2 = 1.0, increases only
#define ROVER_ABORT_DECEL_MM_S2 2000   // 4.1: every decrease uses the abort ramp
#define ROVER_ABORT_ALPHA_MRAD_S2 4000 // the same 4x ratio applied to w (see deviations)

/* ---- wheel PI controller (4.1) ---------------------------------------- */
#define ROVER_PI_KP_Q8 6144        // 24.0 q15-duty per mm/s of wheel error
#define ROVER_PI_KI_Q8 1536        // 6.0 q15-duty per mm/s per 10 ms step
#define ROVER_DUTY_MAX_Q15 32767

/* ---- geometry defaults, [robot] in 5.8 -------------------------------- */
#define ROVER_WHEEL_RADIUS_MM 45
#define ROVER_TRACK_MM 150
#define ROVER_TICKS_PER_REV 2200

/* ---- per-wheel slip and stall (A24) ----------------------------------- */
#define ROVER_STALL_DUTY_PCT 40    // |duty_ch| above this arms the detector
#define ROVER_STALL_HARD_PCT 5     // |v_meas| < 5% of |v_cmd| ...
#define ROVER_STALL_HARD_MS 200    // ... for 200 ms, so I-10's 250 ms holds end to end
#define ROVER_STALL_SLIP_PCT 40    // |v_meas| < 40% of |v_cmd| ...
#define ROVER_STALL_SLIP_MS 500    // ... for 500 ms

/* ---- encoder plausibility (4.1, separate from the slip detector) ------- */
#define ROVER_ENC_IMPLAUS_PCT 50      // commanded vs measured differ by more than this
#define ROVER_ENC_IMPLAUS_CYCLES 20   // for 20 consecutive control cycles
#define ROVER_ENC_IMPLAUS_DUTY_PCT 20 // while |duty_ch| is above this

/* ---- per-channel I2t and the current model (A24) ----------------------- */
#define ROVER_I2T_KNEE_MA 3000      // the integrand is max(0, I_ch - 3.0 A)
#define ROVER_I2T_LIMIT_MA2_MS 6000000000ULL // 6 A^2.s; 3.5 A on one channel trips in 24 s
#define ROVER_I2T_DEAD_DUTY_Q15 328 // |duty_L|+|duty_R| < 0.01 forces both channels to 0
#define ROVER_OVERCURRENT_MA 6000   // the INA226 ALERT level, mirrored in software
#define ROVER_K_E_UV_PER_MM_S 13270 // back-EMF; measured at G2 (see deviations)
#define ROVER_R_MOTOR_MOHM 3430     // measured at G2 beside k_e
#define ROVER_DRIVER_HOT_C 80       // NTC -> DRIVER_HOT (see deviations)

/* ---- battery ladder (A25, 9) ------------------------------------------ */
#define ROVER_VBAT_WARN_MV 10500       // UNDERVOLT_W
#define ROVER_VBAT_STOP_MV 9900        // UNDERVOLT_S: brake, DISARM, refuse V and ARM
#define ROVER_VBAT_DISABLE_MV 9600     // UNDERVOLT_D: MOTOR_EN off, then the Pi rail
#define ROVER_VBAT_DEBOUNCE_MS 10000   // never suspended by load (A25)
#define ROVER_VBAT_CLEAR_MARGIN_MV 300 // 0.3 V above the threshold ...
#define ROVER_VBAT_CLEAR_MS 30000      // ... for 30 s clears warn and stop
#define ROVER_R_PACK_MOHM 65           // V_oc = vbat + imotor * R_pack/1000; measured at G2
#define ROVER_VBAT_INVALID_MV 65535    // INA226 read failed or stale: not a flat pack
#define ROVER_POWEROFF_TIMEOUT_MS 60000 // 9: fallback if the Pi's pulse train never arrives

/* ---- latching thresholds (5.1 "Latched") ------------------------------ */
#define ROVER_LINK_CRC_DROPS_PER_S 20 // rx_drop rising by more than this in 1 s latches
#define ROVER_LOOP_LATE_PCT 10        // loop_late_pct above this ...
#define ROVER_LOOP_LATE_FRAMES 5      // ... for 5 consecutive telemetry frames latches

/* ---- ack rate limiting (5.1: "a rate-limited K ... when a field is clamped") */
#define ROVER_CLAMP_ACK_MIN_MS 500

/* ---- boot banner (5.1) ------------------------------------------------- */
#define ROVER_FW_VER 256 // major<<16 | minor<<8 | patch, so 0.1.0

#endif /* ROVER_CONFIG_H */
