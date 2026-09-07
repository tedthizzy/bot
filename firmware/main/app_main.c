/* Rover MCU application: the glue that binds firmware/core/ to the board.
 *
 * The core owns the protocol, the arm state machine, the TTL, the caps, the
 * slew limiter, the PI controller, the fault classifier and every safety
 * rule. This file owns pins, peripherals and the three tasks, and calls the
 * four core entry points of ARCHITECTURE 4.1 and nothing else.
 */

#include <string.h>

#include "board.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "link.h"
#include "motion.h"
#include "power.h"
#include "rover_core.h"
#include "sensors.h"

static const char *TAG = "rover";

static rover_core_t *s_core;
static TaskHandle_t s_control_task;

#if CONFIG_ROVER_DEBUG_BUILD
#define ROVER_DEBUG_BUILD 1
#else
#define ROVER_DEBUG_BUILD 0
#endif

/* Between the reset edge and the first instruction the S3's GPIOs are inputs
 * and the MDD3A documents no internal pulls, so an unpulled PWM line floating
 * high is full speed. This runs before any peripheral init and is half of
 * I-24; the external 10 kOhm pull-downs are the other half. */
static void safe_outputs(void)
{
    const int pins[] = {
        BOARD_GPIO_M1A, BOARD_GPIO_M1B, BOARD_GPIO_M2A,          BOARD_GPIO_M2B,
        BOARD_GPIO_MOTOR_EN, BOARD_GPIO_PI_SHUTDOWN_REQ, BOARD_GPIO_SERVO_EN,
    };
    uint64_t mask = 0;
    for (size_t i = 0; i < sizeof pins / sizeof pins[0]; i++) {
        gpio_set_level(pins[i], 0);
        mask |= 1ULL << pins[i];
    }
    const gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /* 10 kOhm externally on each */
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    for (size_t i = 0; i < sizeof pins / sizeof pins[0]; i++) {
        gpio_set_level(pins[i], 0);
    }
}

static bool IRAM_ATTR on_control_alarm(gptimer_handle_t timer,
                                       const gptimer_alarm_event_data_t *event, void *arg)
{
    (void)timer;
    (void)event;
    (void)arg;
    BaseType_t higher_priority_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task, &higher_priority_woken);
    return higher_priority_woken == pdTRUE;
}

static void control_timer_start(void)
{
    gptimer_handle_t timer = NULL;
    const gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &timer));

    const gptimer_event_callbacks_t cbs = {.on_alarm = on_control_alarm};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));

    const gptimer_alarm_config_t alarm = {
        .alarm_count = ROVER_CTRL_PERIOD_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
}

static void control_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    uint8_t rx[128];
    uint8_t tx[256];
    for (;;) {
        /* A missed alarm must not stop the loop: at 20 Hz the TTL still
         * expires and the wheels still brake, which is the fail-safe
         * direction if the GPTimer ever stops. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BOARD_CONTROL_WAIT_MS));
        const uint64_t now_us = (uint64_t)esp_timer_get_time();

        size_t n;
        while ((n = link_rx_pop(rx, sizeof rx)) > 0) {
            rover_core_feed(s_core, rx, n);
        }

        rover_in_t in;
        memset(&in, 0, sizeof in);
        sensors_fill(&in, now_us);
        motion_read_encoders(&in.left_ticks, &in.right_ticks);

        rover_out_t out;
        memset(&out, 0, sizeof out);
        rover_core_step(s_core, now_us, &in, &out);

        motion_apply(&out);
        power_apply(&out, now_us);

        while ((n = rover_core_drain_tx(s_core, tx, sizeof tx)) > 0) {
            link_tx_push(tx, n);
        }

        esp_task_wdt_reset();
    }
}

void app_main(void)
{
    safe_outputs();
    power_init();

    /* The session id is minted by the MCU at boot from the hardware RNG and
     * is never 0 (5.1); the core truncates it to the uint16 SESS field. */
    const uint32_t session = esp_random();

    /* The core compiles in every constant it can. What it cannot know is the
     * build it is part of and why the chip last reset, so those four fields
     * are all main sets: WDT_REBOOT and BROWNOUT are latched from here (I-20),
     * and the debug flag is what puts caps b0 and ctrl_flags b7 in the banner
     * so robotd can refuse to arm a console build (A37, I-18). */
    const esp_reset_reason_t reset = esp_reset_reason();

    rover_cfg_t cfg;
    rover_cfg_default(&cfg);
    cfg.reset_reason = (uint8_t)reset;
    cfg.debug_build = (ROVER_DEBUG_BUILD != 0);
    cfg.wdt_reboot = (reset == ESP_RST_TASK_WDT || reset == ESP_RST_INT_WDT ||
                      reset == ESP_RST_WDT);
    cfg.brownout = (reset == ESP_RST_BROWNOUT);
    cfg.caps = (uint16_t)((ROVER_DEBUG_BUILD ? ROVER_CAP_DEBUG_BUILD : 0u) |
                          (BOARD_HAS_CLIFF ? ROVER_CAP_CLIFF_SENSOR : 0u) |
                          (BOARD_HAS_IMU ? ROVER_CAP_IMU : 0u) |
                          (BOARD_HAS_INA ? ROVER_CAP_INA : 0u) |
                          (BOARD_HAS_SERVO_RAIL ? ROVER_CAP_SERVO_RAIL : 0u));

    /* rover_core_t is opaque and holds no file-scope storage. Internal RAM
     * only: the control task must never touch it across a PSRAM cache miss. */
    s_core = heap_caps_malloc(rover_core_size(), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_ERROR_CHECK(s_core != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    rover_core_init(s_core, &cfg, session, (uint64_t)esp_timer_get_time());

    motion_init();
    sensors_init();
    link_init();

    ESP_LOGI(TAG, "fw 0x%06lx caps 0x%04x reset %d session %u safety_hash %lu console=%s",
             (unsigned long)cfg.fw_ver, (unsigned)cfg.caps, (int)reset,
             (unsigned)rover_core_session(s_core), (unsigned long)rover_safety_hash(),
             ROVER_DEBUG_BUILD ? "usb-serial-jtag (DEBUG BUILD)" : "none");

    xTaskCreatePinnedToCore(control_task, "control", BOARD_CONTROL_STACK, NULL,
                            BOARD_CONTROL_PRIO, &s_control_task, BOARD_CONTROL_CORE);
    control_timer_start();
}
