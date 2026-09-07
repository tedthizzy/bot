/* Host test runner for firmware/core.
 *
 * Built by firmware/host/CMakeLists.txt under
 * `clang -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined`, which is
 * the whole of A3's claim that the controller's safety logic is testable on a
 * MacBook before any hardware exists.
 */
#include <stdarg.h>
#include <stdio.h>

#include "test_util.h"

static int case_failures;
static int total_failures;

bool rover_test_streq(const char *a, const char *b)
{
    while (*a != '\0' && *a == *b) {
        a++;
        b++;
    }
    return *a == *b;
}

void rover_test_fail(const char *file, int line, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    fprintf(stderr, "    %s:%d: ", file, line);
    vfprintf(stderr, fmt, args);
    fputc('\n', stderr);
    va_end(args);
    case_failures++;
}

typedef struct {
    const char *name;
    void (*fn)(void);
} test_case_t;

static const test_case_t k_cases[] = {
    {"golden vectors agree with the Python codec",
     test_golden_vectors_agree_with_the_python_codec},
    {"the compiled caps are the numbers ARCHITECTURE states",
     test_compiled_caps_are_the_numbers_architecture_states},
    {"the controller boots disarmed and refuses motion until hello then arm",
     test_boots_disarmed_and_refuses_motion_until_hello_then_arm},
    {"a session change after reboot refuses motion",
     test_session_change_after_reboot_refuses_motion},
    {"TTL expiry stops the motors", test_ttl_expiry_stops_the_motors},
    {"a duplicate or stale sequence does not renew the TTL",
     test_stale_sequence_does_not_renew_the_ttl},
    {"a bad CRC is dropped, counted and does not renew the TTL",
     test_bad_crc_is_dropped_counted_and_does_not_renew_the_ttl},
    {"an over-cap velocity is clamped and flagged",
     test_over_cap_velocity_is_clamped_and_flagged},
    {"the ramp limit holds", test_the_ramp_limit_holds},
    {"an obstacle blocks forward while reverse and rotation still work",
     test_obstacle_blocks_forward_reverse_and_rotation_still_work},
    {"stale ToF is treated as blockage", test_stale_tof_is_treated_as_blockage},
    {"a fault latches and only clears on an explicit clear once the cause is gone",
     test_fault_latches_and_clears_only_on_explicit_clear},
    {"the PI controller converges", test_pi_controller_converges},
    {"garbage and mid-frame resets never crash the parser",
     test_garbage_and_mid_frame_resets_never_crash_the_parser},
    {"an escalated obstacle recovers on an explicit clear",
     test_escalated_obstacle_recovers_on_an_explicit_clear},
    {"obstacle bits still self-clear while escalated",
     test_obstacle_bits_still_self_clear_while_escalated},
    {"an escalated obstacle does not open the e-stop relay",
     test_escalated_obstacle_does_not_open_the_relay},
    {"an idle disarm keeps the standing cliff baseline",
     test_an_idle_disarm_keeps_the_standing_cliff_baseline},
    {"overcurrent clears once the current is gone",
     test_overcurrent_clears_once_the_current_is_gone},
    {"a stale INA read never advances the undervoltage ladder",
     test_a_stale_ina_read_never_advances_the_undervoltage_ladder},
    {"the cliff calibration survives an arm inside the sample window",
     test_calibration_survives_an_arm_inside_the_sample_window},
    {"a ToF bus error emits I2C_ERROR with its sensor index",
     test_a_tof_bus_error_emits_i2c_error_with_its_sensor_index},
};

int main(void)
{
    const size_t n = sizeof k_cases / sizeof k_cases[0];
    for (size_t i = 0; i < n; i++) {
        case_failures = 0;
        k_cases[i].fn();
        printf("%s %s\n", case_failures == 0 ? "PASS" : "FAIL", k_cases[i].name);
        fflush(stdout);
        total_failures += case_failures;
    }
    printf("%zu cases, %d failed assertions\n", n, total_failures);
    return total_failures == 0 ? 0 : 1;
}
