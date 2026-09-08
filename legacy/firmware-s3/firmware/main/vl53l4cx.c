#include "vl53l4cx.h"

#include <string.h>

#include "board.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "tof";

/* Register map, VL53L1X family. */
#define REG_SOFT_RESET                 0x0000
#define REG_I2C_SLAVE_DEVICE_ADDRESS   0x0001
#define REG_VHV_CONFIG_TIMEOUT_BOUND   0x0008
#define REG_VHV_CONFIG_INIT            0x000B
#define REG_GPIO_HV_MUX_CTRL           0x0030
#define REG_GPIO_TIO_HV_STATUS         0x0031
#define REG_PHASECAL_CONFIG_TIMEOUT    0x004B
#define REG_RANGE_CONFIG_TIMEOUT_A_HI  0x005E
#define REG_RANGE_CONFIG_VCSEL_PERIOD_A 0x0060
#define REG_RANGE_CONFIG_TIMEOUT_B_HI  0x0061
#define REG_RANGE_CONFIG_VCSEL_PERIOD_B 0x0063
#define REG_RANGE_CONFIG_VALID_PHASE_HI 0x0069
#define REG_SYSTEM_INTERMEASUREMENT    0x006C
#define REG_SD_CONFIG_WOI_SD0          0x0078
#define REG_SD_CONFIG_INITIAL_PHASE    0x007A
#define REG_SYSTEM_INTERRUPT_CLEAR     0x0086
#define REG_SYSTEM_MODE_START          0x0087
#define REG_RESULT_RANGE_STATUS        0x0089
#define REG_RESULT_DISTANCE_MM         0x0096
#define REG_FIRMWARE_SYSTEM_STATUS     0x00E5
#define REG_RESULT_OSC_CALIBRATE_VAL   0x00DE

#define CONFIG_FIRST_REG 0x2D
#define CONFIG_LAST_REG  0x87

/* The 91-byte power-on configuration block written to 0x2D..0x87. Every byte
 * ST marks "not user-modifiable" is reproduced verbatim; the ones this design
 * depends on are commented. */
static const uint8_t k_default_config[CONFIG_LAST_REG - CONFIG_FIRST_REG + 1] = {
    /* 0x2d */ 0x00, 0x00, 0x00,
    /* 0x30 */ 0x01, /* interrupt active high: bit 4 clear */
    /* 0x31 */ 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00,
    /* 0x3c */ 0x00, 0x00, 0xff, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* 0x46 */ 0x20, /* interrupt on new sample ready */
    /* 0x47 */ 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00,
    /* 0x52 */ 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    /* 0x5d */ 0x00,
    /* 0x5e */ 0x01, 0xdb, /* range timeout macrop A, long-mode default */
    /* 0x60 */ 0x0f,       /* VCSEL period A, long-mode default */
    /* 0x61 */ 0x01, 0xf1, /* range timeout macrop B */
    /* 0x63 */ 0x0d,       /* VCSEL period B */
    /* 0x64 */ 0x01, 0x68, /* sigma threshold, 90 mm in 14.2 */
    /* 0x66 */ 0x00, 0x80, /* minimum count rate, 9.7 */
    /* 0x68 */ 0x08, 0xb8, 0x00, 0x00,
    /* 0x6c */ 0x00, 0x00, 0x0f, 0x89, /* inter-measurement period */
    /* 0x70 */ 0x00, 0x00,
    /* 0x72 */ 0x00, 0x00, 0x00, 0x00, /* distance thresholds, unused */
    /* 0x76 */ 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00, 0x00, 0x02,
    /* 0x7f */ 0xc7, /* ROI centre */
    /* 0x80 */ 0xff, /* ROI width and height */
    /* 0x81 */ 0x9B, 0x00, 0x00, 0x00, 0x01,
    /* 0x86 */ 0x00, /* clear interrupt */
    /* 0x87 */ 0x00, /* mode start: stopped */
};

_Static_assert(sizeof k_default_config == 91, "config block spans 0x2D..0x87");

/* Timing-budget register pairs, from ST's ULD table. Only the pairing this
 * design ships (short mode, 20 ms) and the fallback ARCHITECTURE 4.1 names
 * (33 ms in either mode) are carried: an entry that is never written is a
 * magic number nothing can check. */
typedef struct {
    uint16_t budget_ms;
    uint16_t macrop_a;
    uint16_t macrop_b;
} budget_entry_t;

static const budget_entry_t k_budget_short[] = {
    {20, 0x0051, 0x006E},
    {33, 0x00D6, 0x006E},
};

static const budget_entry_t k_budget_long[] = {
    {33, 0x0060, 0x006E},
};

static esp_err_t reg_write(vl53l4cx_t *s, uint16_t reg, const uint8_t *data, size_t n)
{
    uint8_t buf[6];
    if (n + 2 > sizeof buf) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)reg;
    memcpy(&buf[2], data, n);
    return i2c_master_transmit(s->dev, buf, n + 2, BOARD_I2C_TIMEOUT_MS);
}

static esp_err_t reg_read(vl53l4cx_t *s, uint16_t reg, uint8_t *data, size_t n)
{
    const uint8_t addr[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
    return i2c_master_transmit_receive(s->dev, addr, sizeof addr, data, n,
                                       BOARD_I2C_TIMEOUT_MS);
}

static esp_err_t write8(vl53l4cx_t *s, uint16_t reg, uint8_t v)
{
    return reg_write(s, reg, &v, 1);
}

static esp_err_t write16(vl53l4cx_t *s, uint16_t reg, uint16_t v)
{
    const uint8_t b[2] = {(uint8_t)(v >> 8), (uint8_t)v};
    return reg_write(s, reg, b, 2);
}

static esp_err_t write32(vl53l4cx_t *s, uint16_t reg, uint32_t v)
{
    const uint8_t b[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8),
                          (uint8_t)v};
    return reg_write(s, reg, b, 4);
}

static esp_err_t read8(vl53l4cx_t *s, uint16_t reg, uint8_t *v)
{
    return reg_read(s, reg, v, 1);
}

static esp_err_t read16(vl53l4cx_t *s, uint16_t reg, uint16_t *v)
{
    uint8_t b[2];
    esp_err_t err = reg_read(s, reg, b, sizeof b);
    if (err == ESP_OK) {
        *v = (uint16_t)((b[0] << 8) | b[1]);
    }
    return err;
}

static esp_err_t read32(vl53l4cx_t *s, uint16_t reg, uint32_t *v)
{
    uint8_t b[4];
    esp_err_t err = reg_read(s, reg, b, sizeof b);
    if (err == ESP_OK) {
        *v = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    }
    return err;
}

static esp_err_t attach(vl53l4cx_t *s, uint8_t address)
{
    if (s->dev != NULL) {
        i2c_master_bus_rm_device(s->dev);
        s->dev = NULL;
    }
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = BOARD_I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(s->bus, &cfg, &s->dev);
    if (err == ESP_OK) {
        s->address = address;
    }
    return err;
}

static esp_err_t wait_booted(vl53l4cx_t *s)
{
    for (int i = 0; i < 20; i++) {
        uint8_t status = 0;
        if (read8(s, REG_FIRMWARE_SYSTEM_STATUS, &status) == ESP_OK && (status & 1u) != 0) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t set_distance_mode(vl53l4cx_t *s, bool short_mode)
{
    if (short_mode) {
        ESP_RETURN_ON_ERROR(write8(s, REG_PHASECAL_CONFIG_TIMEOUT, 0x14), TAG, "phasecal");
        ESP_RETURN_ON_ERROR(write8(s, REG_RANGE_CONFIG_VCSEL_PERIOD_A, 0x07), TAG, "vcsel a");
        ESP_RETURN_ON_ERROR(write8(s, REG_RANGE_CONFIG_VCSEL_PERIOD_B, 0x05), TAG, "vcsel b");
        ESP_RETURN_ON_ERROR(write8(s, REG_RANGE_CONFIG_VALID_PHASE_HI, 0x38), TAG, "phase");
        ESP_RETURN_ON_ERROR(write16(s, REG_SD_CONFIG_WOI_SD0, 0x0705), TAG, "woi");
        ESP_RETURN_ON_ERROR(write16(s, REG_SD_CONFIG_INITIAL_PHASE, 0x0606), TAG, "phase0");
    } else {
        ESP_RETURN_ON_ERROR(write8(s, REG_PHASECAL_CONFIG_TIMEOUT, 0x0A), TAG, "phasecal");
        ESP_RETURN_ON_ERROR(write8(s, REG_RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F), TAG, "vcsel a");
        ESP_RETURN_ON_ERROR(write8(s, REG_RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D), TAG, "vcsel b");
        ESP_RETURN_ON_ERROR(write8(s, REG_RANGE_CONFIG_VALID_PHASE_HI, 0xB8), TAG, "phase");
        ESP_RETURN_ON_ERROR(write16(s, REG_SD_CONFIG_WOI_SD0, 0x0F0D), TAG, "woi");
        ESP_RETURN_ON_ERROR(write16(s, REG_SD_CONFIG_INITIAL_PHASE, 0x0E0E), TAG, "phase0");
    }
    return ESP_OK;
}

static const budget_entry_t *budget_table(bool short_mode, size_t *count)
{
    if (short_mode) {
        *count = sizeof k_budget_short / sizeof k_budget_short[0];
        return k_budget_short;
    }
    *count = sizeof k_budget_long / sizeof k_budget_long[0];
    return k_budget_long;
}

static esp_err_t set_timing_budget_ms(vl53l4cx_t *s, bool short_mode, uint16_t ms)
{
    size_t count;
    const budget_entry_t *table = budget_table(short_mode, &count);
    for (size_t i = 0; i < count; i++) {
        if (table[i].budget_ms == ms) {
            ESP_RETURN_ON_ERROR(write16(s, REG_RANGE_CONFIG_TIMEOUT_A_HI, table[i].macrop_a),
                                TAG, "macrop a");
            return write16(s, REG_RANGE_CONFIG_TIMEOUT_B_HI, table[i].macrop_b);
        }
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t vl53l4cx_get_timing_budget_ms(vl53l4cx_t *s, bool short_mode, uint16_t *ms)
{
    uint16_t macrop_a = 0;
    ESP_RETURN_ON_ERROR(read16(s, REG_RANGE_CONFIG_TIMEOUT_A_HI, &macrop_a), TAG, "macrop a");
    size_t count;
    const budget_entry_t *table = budget_table(short_mode, &count);
    for (size_t i = 0; i < count; i++) {
        if (table[i].macrop_a == macrop_a) {
            *ms = table[i].budget_ms;
            return ESP_OK;
        }
    }
    return ESP_ERR_INVALID_STATE;
}

/* The period register counts PLL clocks. ST's ULD scales by 1.075 writing and
 * by 1.065 reading; both factors are reproduced so a read-back of a value
 * this driver wrote returns the value it wrote. */
static esp_err_t set_inter_period_ms(vl53l4cx_t *s, uint16_t ms)
{
    uint16_t pll = 0;
    ESP_RETURN_ON_ERROR(read16(s, REG_RESULT_OSC_CALIBRATE_VAL, &pll), TAG, "pll");
    pll &= 0x03FFu;
    if (pll == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    const uint32_t ticks = ((uint32_t)pll * ms * 1075u) / 1000u;
    return write32(s, REG_SYSTEM_INTERMEASUREMENT, ticks);
}

esp_err_t vl53l4cx_get_inter_period_ms(vl53l4cx_t *s, uint16_t *ms)
{
    uint16_t pll = 0;
    uint32_t ticks = 0;
    ESP_RETURN_ON_ERROR(read16(s, REG_RESULT_OSC_CALIBRATE_VAL, &pll), TAG, "pll");
    ESP_RETURN_ON_ERROR(read32(s, REG_SYSTEM_INTERMEASUREMENT, &ticks), TAG, "period");
    pll &= 0x03FFu;
    if (pll == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    *ms = (uint16_t)((ticks * 1000u) / ((uint32_t)pll * 1065u));
    return ESP_OK;
}

static esp_err_t clear_interrupt(vl53l4cx_t *s)
{
    return write8(s, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

static esp_err_t set_ranging(vl53l4cx_t *s, bool on)
{
    return write8(s, REG_SYSTEM_MODE_START, on ? 0x40 : 0x00);
}

esp_err_t vl53l4cx_data_ready(vl53l4cx_t *s, bool *ready)
{
    uint8_t mux = 0;
    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(read8(s, REG_GPIO_HV_MUX_CTRL, &mux), TAG, "mux");
    ESP_RETURN_ON_ERROR(read8(s, REG_GPIO_TIO_HV_STATUS, &status), TAG, "status");
    const uint8_t polarity = (uint8_t)(((mux & 0x10u) >> 4) ^ 1u);
    *ready = ((status & 1u) == polarity);
    return ESP_OK;
}

esp_err_t vl53l4cx_read(vl53l4cx_t *s, uint16_t *mm)
{
    uint8_t raw_status = 0;
    uint16_t distance = 0;
    ESP_RETURN_ON_ERROR(read8(s, REG_RESULT_RANGE_STATUS, &raw_status), TAG, "range status");
    ESP_RETURN_ON_ERROR(read16(s, REG_RESULT_DISTANCE_MM, &distance), TAG, "distance");
    ESP_RETURN_ON_ERROR(clear_interrupt(s), TAG, "clear");

    /* ST's RangeStatus decoding, reduced to the three values the link
     * carries. 9 is a valid measurement. 4 is signal-below-threshold, which
     * is what an open room, a dark rug or a specular floor returns and which
     * ARCHITECTURE 5.1 requires be reported as a clear path, not a fault.
     * Everything else -- sigma failure, hardware failure, wrap-around, no
     * update -- is an error, and forward is refused on it. */
    switch (raw_status & 0x1Fu) {
    case 9:
        *mm = distance;
        break;
    case 4:
        *mm = ROVER_TOF_NO_TARGET_MM;
        break;
    default:
        *mm = ROVER_TOF_ERROR_MM;
        break;
    }
    return ESP_OK;
}

esp_err_t vl53l4cx_start(vl53l4cx_t *s, i2c_master_bus_handle_t bus, int xshut_gpio,
                         uint8_t address, bool short_mode, uint16_t budget_ms,
                         uint16_t inter_period_ms)
{
    s->bus = bus;
    s->dev = NULL;
    s->xshut_gpio = xshut_gpio;

    gpio_set_level(xshut_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(xshut_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_RETURN_ON_ERROR(attach(s, BOARD_TOF_ADDR_DEFAULT), TAG, "attach default");
    ESP_RETURN_ON_ERROR(wait_booted(s), TAG, "boot");
    ESP_RETURN_ON_ERROR(write8(s, REG_I2C_SLAVE_DEVICE_ADDRESS, address), TAG, "address");
    ESP_RETURN_ON_ERROR(attach(s, address), TAG, "attach");

    for (uint16_t reg = CONFIG_FIRST_REG; reg <= CONFIG_LAST_REG; reg++) {
        ESP_RETURN_ON_ERROR(write8(s, reg, k_default_config[reg - CONFIG_FIRST_REG]), TAG,
                            "config");
    }

    /* One throwaway measurement, then the VHV settings ST's init sequence
     * applies once the reference calibration has run. */
    ESP_RETURN_ON_ERROR(set_ranging(s, true), TAG, "start");
    for (int i = 0; i < 100; i++) {
        bool ready = false;
        ESP_RETURN_ON_ERROR(vl53l4cx_data_ready(s, &ready), TAG, "ready");
        if (ready) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    ESP_RETURN_ON_ERROR(clear_interrupt(s), TAG, "clear");
    ESP_RETURN_ON_ERROR(set_ranging(s, false), TAG, "stop");
    ESP_RETURN_ON_ERROR(write8(s, REG_VHV_CONFIG_TIMEOUT_BOUND, 0x09), TAG, "vhv bound");
    ESP_RETURN_ON_ERROR(write8(s, REG_VHV_CONFIG_INIT, 0x00), TAG, "vhv init");

    ESP_RETURN_ON_ERROR(set_distance_mode(s, short_mode), TAG, "mode");
    ESP_RETURN_ON_ERROR(set_timing_budget_ms(s, short_mode, budget_ms), TAG, "budget");
    ESP_RETURN_ON_ERROR(set_inter_period_ms(s, inter_period_ms), TAG, "period");

    /* ARCHITECTURE 4.1: the achieved budget and period are read back and a
     * difference from the compiled constants is a failure. The compiled pair
     * is what the B banner's safety_hash covers and what A21's detect-latency
     * budget is derived from, so a sensor running at something else must not
     * be allowed to authorise forward motion. */
    uint16_t achieved_budget = 0;
    uint16_t achieved_period = 0;
    ESP_RETURN_ON_ERROR(vl53l4cx_get_timing_budget_ms(s, short_mode, &achieved_budget), TAG,
                        "budget readback");
    ESP_RETURN_ON_ERROR(vl53l4cx_get_inter_period_ms(s, &achieved_period), TAG,
                        "period readback");
    if (achieved_budget != budget_ms || achieved_period != inter_period_ms) {
        ESP_LOGE(TAG, "0x%02x budget %u/%u period %u/%u", address, achieved_budget, budget_ms,
                 achieved_period, inter_period_ms);
        return ESP_ERR_INVALID_STATE;
    }

    return set_ranging(s, true);
}

void vl53l4cx_stop(vl53l4cx_t *s)
{
    if (s->dev != NULL) {
        i2c_master_bus_rm_device(s->dev);
        s->dev = NULL;
    }
    gpio_set_level(s->xshut_gpio, 0);
}
