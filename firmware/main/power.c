#include "power.h"

#include "board.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "power";

static uint64_t s_edge_us[BOARD_POWEROFF_EDGES];
static int s_edge_count;
static int s_last_poweroff_level = -1;
static bool s_rail_cut;

void power_init(void)
{
    /* Open drain, output level high = high impedance. No internal pull-up:
     * the D24V50F5 pulls EN to VIN itself through the 10 kOhm series
     * resistor, and a pull-up here would sit a 12 V node on a 3V3 pad. */
    const gpio_config_t rail = {
        .pin_bit_mask = 1ULL << BOARD_GPIO_PI_RAIL_EN,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_set_level(BOARD_GPIO_PI_RAIL_EN, 1));
    ESP_ERROR_CHECK(gpio_config(&rail));

    const gpio_config_t poweroff_in = {
        .pin_bit_mask = 1ULL << BOARD_GPIO_PI_POWEROFF_IN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /* 10 kOhm externally */
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&poweroff_in));
}

/* The overlay's train is active 100 ms, inactive 100 ms, active, so polling at
 * the control rate sees every transition. Confirmed when the oldest of the
 * last three edges is still inside the 2 s window. */
static bool poweroff_confirmed(uint64_t now_us)
{
    const int level = gpio_get_level(BOARD_GPIO_PI_POWEROFF_IN);
    if (level != s_last_poweroff_level) {
        s_last_poweroff_level = level;
        for (int i = BOARD_POWEROFF_EDGES - 1; i > 0; i--) {
            s_edge_us[i] = s_edge_us[i - 1];
        }
        s_edge_us[0] = now_us;
        if (s_edge_count < BOARD_POWEROFF_EDGES) {
            s_edge_count++;
        }
    }
    return s_edge_count >= BOARD_POWEROFF_EDGES &&
           (now_us - s_edge_us[BOARD_POWEROFF_EDGES - 1]) <= BOARD_POWEROFF_WINDOW_US;
}

void power_apply(const rover_out_t *out, uint64_t now_us)
{
    gpio_set_level(BOARD_GPIO_PI_SHUTDOWN_REQ, out->pi_shutdown_req ? 1 : 0);

    if (s_rail_cut) {
        return;
    }

    /* Two ways the rail goes: the core asks for it, having run out its own
     * ROVER_POWEROFF_TIMEOUT_MS fallback, or the Pi's gpio-poweroff train
     * confirms the halt finished and this cuts early. Waiting the full 60 s
     * at 9.6 V on a sagging pack is the worst time to keep the Pi drawing. */
    const bool confirmed = out->pi_shutdown_req && poweroff_confirmed(now_us);
    if (out->pi_rail_en == 0 || confirmed) {
        ESP_LOGW(TAG, "rail off, poweroff %s", confirmed ? "confirmed" : "core timeout");
        gpio_set_level(BOARD_GPIO_PI_RAIL_EN, 0);
        s_rail_cut = true;
    }
}
