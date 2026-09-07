/* The host test harness: no external framework, one line of PASS or FAIL per
 * named case, non-zero exit on any failure. */
#ifndef ROVER_TEST_UTIL_H
#define ROVER_TEST_UTIL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "rover_internal.h"

/** Record a failure inside the running case; the case still runs to the end so
 *  one run reports every broken assertion, not just the first. */
void rover_test_fail(const char *file, int line, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            rover_test_fail(__FILE__, __LINE__, "%s", #cond);                  \
        }                                                                      \
    } while (0)

#define CHECK_EQ(actual, expected)                                             \
    do {                                                                       \
        long long a_ = (long long)(actual);                                    \
        long long e_ = (long long)(expected);                                  \
        if (a_ != e_) {                                                        \
            rover_test_fail(__FILE__, __LINE__, "%s == %s: got %lld, want %lld", \
                            #actual, #expected, a_, e_);                       \
        }                                                                      \
    } while (0)

#define CHECK_STR_EQ(actual, expected)                                         \
    do {                                                                       \
        const char *a_ = (actual);                                             \
        const char *e_ = (expected);                                           \
        if (rover_test_streq(a_, e_) == false) {                               \
            rover_test_fail(__FILE__, __LINE__, "%s: got \"%s\", want \"%s\"", \
                            #actual, a_, e_);                                  \
        }                                                                      \
    } while (0)

bool rover_test_streq(const char *a, const char *b);

/* Cases.  Each is registered in test_main.c. */
void test_golden_vectors_agree_with_the_python_codec(void);
void test_compiled_caps_are_the_numbers_architecture_states(void);
void test_boots_disarmed_and_refuses_motion_until_hello_then_arm(void);
void test_session_change_after_reboot_refuses_motion(void);
void test_ttl_expiry_stops_the_motors(void);
void test_stale_sequence_does_not_renew_the_ttl(void);
void test_bad_crc_is_dropped_counted_and_does_not_renew_the_ttl(void);
void test_over_cap_velocity_is_clamped_and_flagged(void);
void test_the_ramp_limit_holds(void);
void test_obstacle_blocks_forward_reverse_and_rotation_still_work(void);
void test_stale_tof_is_treated_as_blockage(void);
void test_fault_latches_and_clears_only_on_explicit_clear(void);
void test_pi_controller_converges(void);
void test_garbage_and_mid_frame_resets_never_crash_the_parser(void);
void test_escalated_obstacle_recovers_on_an_explicit_clear(void);
void test_obstacle_bits_still_self_clear_while_escalated(void);
void test_escalated_obstacle_does_not_open_the_relay(void);
void test_an_idle_disarm_keeps_the_standing_cliff_baseline(void);
void test_overcurrent_clears_once_the_current_is_gone(void);
void test_a_stale_ina_read_never_advances_the_undervoltage_ladder(void);
void test_calibration_survives_an_arm_inside_the_sample_window(void);
void test_a_tof_bus_error_emits_i2c_error_with_its_sensor_index(void);

#endif /* ROVER_TEST_UTIL_H */
