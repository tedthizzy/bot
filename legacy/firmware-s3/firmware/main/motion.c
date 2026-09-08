#include "motion.h"

#include <stddef.h>

#include "board.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

static const char *TAG = "motion";

enum { CH_LEFT = 0, CH_RIGHT = 1, CH_COUNT = 2 };
enum { GEN_A = 0, GEN_B = 1, GEN_COUNT = 2 };

static mcpwm_timer_handle_t s_timer;
static mcpwm_oper_handle_t s_oper[CH_COUNT];
static mcpwm_cmpr_handle_t s_cmp[CH_COUNT][GEN_COUNT];
static mcpwm_gen_handle_t s_gen[CH_COUNT][GEN_COUNT];
static pcnt_unit_handle_t s_pcnt[CH_COUNT];
/* One-shot recovery names the fault it clears, so the handles outlive fault_init. */
#define FAULT_COUNT 2
static mcpwm_fault_handle_t s_fault[FAULT_COUNT];

static const int k_gen_gpio[CH_COUNT][GEN_COUNT] = {
    {BOARD_GPIO_M1A, BOARD_GPIO_M1B},
    {BOARD_GPIO_M2A, BOARD_GPIO_M2B},
};

/* Last force level applied per generator, so a steady state is not rewritten
 * every 10 ms: -1 released, 0 low, 1 high. */
static int s_forced[CH_COUNT][GEN_COUNT] = {{-2, -2}, {-2, -2}};

static void force(int ch, int gen, int level)
{
    if (s_forced[ch][gen] == level) {
        return;
    }
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(s_gen[ch][gen], level, true));
    s_forced[ch][gen] = level;
}

static void pwm_init(void)
{
    const mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = BOARD_PWM_RESOLUTION_HZ,
        .period_ticks = BOARD_PWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &s_timer));

    for (int ch = 0; ch < CH_COUNT; ch++) {
        const mcpwm_operator_config_t oper_cfg = {.group_id = 0};
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &s_oper[ch]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_oper[ch], s_timer));

        for (int gen = 0; gen < GEN_COUNT; gen++) {
            const mcpwm_comparator_config_t cmp_cfg = {.flags.update_cmp_on_tez = true};
            ESP_ERROR_CHECK(mcpwm_new_comparator(s_oper[ch], &cmp_cfg, &s_cmp[ch][gen]));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_cmp[ch][gen], 0));

            const mcpwm_generator_config_t gen_cfg = {.gen_gpio_num = k_gen_gpio[ch][gen]};
            ESP_ERROR_CHECK(mcpwm_new_generator(s_oper[ch], &gen_cfg, &s_gen[ch][gen]));
            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                s_gen[ch][gen], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                             MCPWM_TIMER_EVENT_EMPTY,
                                                             MCPWM_GEN_ACTION_HIGH)));
            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                s_gen[ch][gen], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                               s_cmp[ch][gen],
                                                               MCPWM_GEN_ACTION_LOW)));
            /* Both inputs low is the MDD3A's stop state, and it is where the
             * pins already sit from app_main's pre-peripheral drive. */
            force(ch, gen, 0);
        }
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(s_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP));
}

static void fault_init(void)
{
    /* Both fault inputs are active low and latch one-shot, so a trip forces
     * all four MDD3A inputs high -- driver brake -- and cannot be cleared
     * while the signal is still asserted. Only these two are on this path:
     * an INA226 over-current ALERT and the e-stop monitor. */
    const int fault_gpio[FAULT_COUNT] = {BOARD_GPIO_FAULT_INA_ALERT, BOARD_GPIO_FAULT_ESTOP};
    const bool internal_pull_up[FAULT_COUNT] = {true, false};

    for (size_t f = 0; f < FAULT_COUNT; f++) {
        mcpwm_fault_handle_t fault = NULL;
        const mcpwm_gpio_fault_config_t cfg = {
            .group_id = 0,
            .gpio_num = fault_gpio[f],
            .flags.active_level = 0,
            .flags.pull_up = internal_pull_up[f],
        };
        ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&cfg, &fault));
        s_fault[f] = fault;

        for (int ch = 0; ch < CH_COUNT; ch++) {
            const mcpwm_brake_config_t brake = {
                .fault = fault,
                .brake_mode = MCPWM_OPER_BRAKE_MODE_OST,
            };
            ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(s_oper[ch], &brake));
            for (int gen = 0; gen < GEN_COUNT; gen++) {
                ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(
                    s_gen[ch][gen],
                    MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                 MCPWM_OPER_BRAKE_MODE_OST,
                                                 MCPWM_GEN_ACTION_HIGH)));
            }
        }
    }
}

static void encoder_init(void)
{
    const int chan_gpio[CH_COUNT][2] = {
        {BOARD_GPIO_ENC_L_A, BOARD_GPIO_ENC_L_B},
        {BOARD_GPIO_ENC_R_A, BOARD_GPIO_ENC_R_B},
    };

    for (int ch = 0; ch < CH_COUNT; ch++) {
        const pcnt_unit_config_t unit_cfg = {
            .high_limit = BOARD_PCNT_HIGH_LIMIT,
            .low_limit = BOARD_PCNT_LOW_LIMIT,
            .flags.accum_count = true,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_pcnt[ch]));

        const pcnt_glitch_filter_config_t filter = {.max_glitch_ns = BOARD_ENC_GLITCH_NS};
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_pcnt[ch], &filter));

        /* x4 decoding: each channel counts both edges of one phase and takes
         * its direction from the level of the other. */
        pcnt_channel_handle_t chan_a = NULL;
        pcnt_channel_handle_t chan_b = NULL;
        const pcnt_chan_config_t cfg_a = {
            .edge_gpio_num = chan_gpio[ch][0],
            .level_gpio_num = chan_gpio[ch][1],
        };
        ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt[ch], &cfg_a, &chan_a));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        const pcnt_chan_config_t cfg_b = {
            .edge_gpio_num = chan_gpio[ch][1],
            .level_gpio_num = chan_gpio[ch][0],
        };
        ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt[ch], &cfg_b, &chan_b));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        /* accum_count needs a watch point at each limit: the unit adds the
         * limit to a software accumulator as the hardware counter wraps. */
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_pcnt[ch], BOARD_PCNT_HIGH_LIMIT));
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_pcnt[ch], BOARD_PCNT_LOW_LIMIT));
        ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt[ch]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt[ch]));
        ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt[ch]));
    }
}

void motion_init(void)
{
    pwm_init();
    fault_init();
    encoder_init();
    ESP_LOGI(TAG, "mcpwm %d Hz, %d ticks", BOARD_PWM_FREQ_HZ, BOARD_PWM_PERIOD_TICKS);
}

void motion_read_encoders(int32_t *left_ticks, int32_t *right_ticks)
{
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(s_pcnt[CH_LEFT], &count));
    *left_ticks = count;
    ESP_ERROR_CHECK(pcnt_unit_get_count(s_pcnt[CH_RIGHT], &count));
    *right_ticks = count;
}

static void channel_apply(int ch, int16_t duty_q15, bool brake)
{
    if (brake) {
        /* Both inputs high is the MDD3A's brake. "PWM zero" is not the
         * terminal state I-1 asks for. */
        force(ch, GEN_A, 1);
        force(ch, GEN_B, 1);
        return;
    }
    if (duty_q15 == 0) {
        force(ch, GEN_A, 0);
        force(ch, GEN_B, 0);
        return;
    }

    const int driven = (duty_q15 > 0) ? GEN_A : GEN_B;
    const int idle = (duty_q15 > 0) ? GEN_B : GEN_A;
    const int32_t magnitude = (duty_q15 > 0) ? duty_q15 : -(int32_t)duty_q15;
    uint32_t ticks = (uint32_t)((magnitude * (int32_t)BOARD_PWM_PERIOD_TICKS) / ROVER_DUTY_MAX_Q15);
    if (ticks > BOARD_PWM_PERIOD_TICKS) {
        ticks = BOARD_PWM_PERIOD_TICKS;
    }

    force(ch, idle, 0);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_cmp[ch][driven], ticks));
    force(ch, driven, -1);
}

void motion_apply(const rover_out_t *out)
{
    /* A one-shot trip stays latched until the fault signal deasserts and the
     * operator is recovered; attempt it only when the core is asking to
     * drive, so recovery can never be what starts motion. Throttled, because
     * a still-asserted fault makes every attempt fail and log. */
    static int recover_countdown;
    if (out->motor_en && !out->brake) {
        if (--recover_countdown <= 0) {
            recover_countdown = ROVER_CTRL_HZ / 10;
            for (int ch = 0; ch < CH_COUNT; ch++) {
                for (size_t f = 0; f < FAULT_COUNT; f++) {
                    (void)mcpwm_operator_recover_from_fault(s_oper[ch], s_fault[f]);
                }
            }
        }
    } else {
        recover_countdown = 0;
    }

    channel_apply(CH_LEFT, out->duty_l_q15, out->brake != 0);
    channel_apply(CH_RIGHT, out->duty_r_q15, out->brake != 0);

    gpio_set_level(BOARD_GPIO_MOTOR_EN, out->motor_en ? 1 : 0);
    gpio_set_level(BOARD_GPIO_SERVO_EN, out->servo_en ? 1 : 0);
}
