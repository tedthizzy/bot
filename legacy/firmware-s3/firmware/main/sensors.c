#include "sensors.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l4cx.h"

static const char *TAG = "sensors";

/* INA226 registers. */
#define INA226_REG_CONFIG      0x00
#define INA226_REG_BUS_VOLTAGE 0x02
#define INA226_REG_CURRENT     0x04
#define INA226_REG_CALIBRATION 0x05
#define INA226_REG_MASK_ENABLE 0x06
#define INA226_REG_ALERT_LIMIT 0x07

/* 16 averages, 1.1 ms bus and shunt conversion, continuous: ~35 ms per
 * sample, comfortably inside the 20 ms sensor period at 50 Hz. */
#define INA226_CONFIG_VALUE 0x4527
/* Shunt over-limit, latched, ALERT active low -- A24's hardware layer, wired
 * to MCPWM FAULT0. */
#define INA226_MASK_SOL_LATCH 0x8001

typedef struct {
    uint16_t mm;
    uint64_t stamp_us;
    bool ok;
} tof_sample_t;

/* The electrical snapshot carries the same ok flag and stamp the ToF samples
 * do.  Publishing 0 mV on a failed read is indistinguishable from a flat pack,
 * and the response to a flat pack is to power the host off (A25). */
typedef struct {
    uint16_t vbat_mv;
    int16_t imotor_ma;
    uint64_t stamp_us;
    bool ok;
} electrical_sample_t;

typedef struct {
    tof_sample_t tof[BOARD_TOF_COUNT];
    electrical_sample_t ina;
    int16_t ntc_c;
} snapshot_t;

static snapshot_t s_snap;
static portMUX_TYPE s_snap_lock = portMUX_INITIALIZER_UNLOCKED;

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_ina;
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_adc_cali;

static vl53l4cx_t s_tof[BOARD_TOF_COUNT];
static bool s_tof_up[BOARD_TOF_COUNT];
static int s_tof_errors[BOARD_TOF_COUNT];
static uint64_t s_tof_retry_us[BOARD_TOF_COUNT];

static const int k_tof_xshut[BOARD_TOF_COUNT] = {
    BOARD_GPIO_TOF_XSHUT_FL,
    BOARD_GPIO_TOF_XSHUT_FR,
    BOARD_GPIO_TOF_XSHUT_CL,
};

static const uint8_t k_tof_addr[BOARD_TOF_COUNT] = {
    BOARD_TOF_ADDR_FL,
    BOARD_TOF_ADDR_FR,
    BOARD_TOF_ADDR_CL,
};

/* The bumper is a plain GPIO with an ISR (never an MCPWM fault): a press
 * shorter than one control period must still be seen. The latch is consumed
 * by the control task, and the core's obstacle class clears it after five
 * clean samples. */
static volatile bool s_bumper_hit;

static void IRAM_ATTR bumper_isr(void *arg)
{
    (void)arg;
    if (gpio_get_level(BOARD_GPIO_BUMPER) == 0) {
        s_bumper_hit = true;
    }
}

static esp_err_t ina226_write(uint8_t reg, uint16_t value)
{
    const uint8_t buf[3] = {reg, (uint8_t)(value >> 8), (uint8_t)value};
    return i2c_master_transmit(s_ina, buf, sizeof buf, BOARD_I2C_TIMEOUT_MS);
}

static esp_err_t ina226_read(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    esp_err_t err =
        i2c_master_transmit_receive(s_ina, &reg, 1, buf, sizeof buf, BOARD_I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        *value = (uint16_t)((buf[0] << 8) | buf[1]);
    }
    return err;
}

static void ina226_init(void)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BOARD_INA226_ADDR,
        .scl_speed_hz = BOARD_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &cfg, &s_ina));

    if (ina226_write(INA226_REG_CONFIG, INA226_CONFIG_VALUE) != ESP_OK ||
        ina226_write(INA226_REG_CALIBRATION, BOARD_INA226_CAL) != ESP_OK ||
        ina226_write(INA226_REG_ALERT_LIMIT, BOARD_INA226_ALERT_RAW) != ESP_OK ||
        ina226_write(INA226_REG_MASK_ENABLE, INA226_MASK_SOL_LATCH) != ESP_OK) {
        /* ALERT idles high through its 10 kOhm pull-up, so an absent monitor
         * leaves MCPWM FAULT0 inactive rather than latching the brake. The
         * pack reading then publishes the invalid sentinel, which freezes the
         * ladder and stops motion rather than counting down to a poweroff. */
        ESP_LOGE(TAG, "INA226 not responding");
    }
}

static void ntc_init(void)
{
    const adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &s_adc));

    const adc_oneshot_chan_cfg_t chan = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, BOARD_NTC_ADC_CHAN, &chan));

    const adc_cali_curve_fitting_config_t cali = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali, &s_adc_cali));
}

/* 10 kOhm from 3V3 to the node, NTC to ground, so a hotter driver reads a
 * lower voltage. Beta equation against the 25 C reference. */
static int16_t ntc_read_c(void)
{
    int raw = 0;
    int mv = 0;
    if (adc_oneshot_read(s_adc, BOARD_NTC_ADC_CHAN, &raw) != ESP_OK ||
        adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) != ESP_OK) {
        return 0;
    }
    if (mv <= 0 || mv >= (int)BOARD_NTC_VDD_MV) {
        return 0;
    }
    const float r = BOARD_NTC_SERIES_OHM * (float)mv / (BOARD_NTC_VDD_MV - (float)mv);
    const float inv_t = 1.0f / 298.15f + logf(r / BOARD_NTC_R25_OHM) / BOARD_NTC_BETA;
    return (int16_t)lrintf(1.0f / inv_t - 273.15f);
}

static void tof_bring_up(int index)
{
    s_tof_up[index] = false;
    s_tof_errors[index] = 0;
    const esp_err_t err =
        vl53l4cx_start(&s_tof[index], s_bus, k_tof_xshut[index], k_tof_addr[index],
                       BOARD_TOF_SHORT_MODE, ROVER_TOF_TIMING_BUDGET_MS,
                       ROVER_TOF_INTER_PERIOD_MS);
    if (err == ESP_OK) {
        s_tof_up[index] = true;
    } else {
        ESP_LOGE(TAG, "tof %d start: %s", index, esp_err_to_name(err));
        vl53l4cx_stop(&s_tof[index]);
    }
}

static void tof_poll(int index, uint64_t now_us)
{
    if (!s_tof_up[index]) {
        if (now_us - s_tof_retry_us[index] >= BOARD_TOF_REINIT_BACKOFF_US) {
            s_tof_retry_us[index] = now_us;
            tof_bring_up(index);
        }
        return;
    }

    bool ready = false;
    if (vl53l4cx_data_ready(&s_tof[index], &ready) != ESP_OK) {
        goto failed;
    }
    if (!ready) {
        return;
    }

    uint16_t mm = 0;
    if (vl53l4cx_read(&s_tof[index], &mm) != ESP_OK) {
        goto failed;
    }

    s_tof_errors[index] = 0;
    portENTER_CRITICAL(&s_snap_lock);
    s_snap.tof[index].mm = mm;
    s_snap.tof[index].stamp_us = now_us;
    s_snap.tof[index].ok = (mm != ROVER_TOF_ERROR_MM);
    portEXIT_CRITICAL(&s_snap_lock);
    return;

failed:
    if (++s_tof_errors[index] >= BOARD_TOF_REINIT_ERRORS) {
        ESP_LOGE(TAG, "tof %d re-init", index);
        vl53l4cx_stop(&s_tof[index]);
        s_tof_up[index] = false;
        s_tof_retry_us[index] = now_us;
    }
}

static void electrical_poll(uint64_t now_us)
{
    uint16_t bus_raw = 0;
    uint16_t current_raw = 0;

    const bool ok = ina226_read(INA226_REG_BUS_VOLTAGE, &bus_raw) == ESP_OK &&
                    ina226_read(INA226_REG_CURRENT, &current_raw) == ESP_OK;

    const int16_t ntc_c = ntc_read_c();

    portENTER_CRITICAL(&s_snap_lock);
    if (ok) {
        s_snap.ina.vbat_mv =
            (uint16_t)(((uint32_t)bus_raw * BOARD_INA226_BUS_LSB_UV) / 1000u);
        /* 1 mA per count, negative = regen. */
        s_snap.ina.imotor_ma = (int16_t)current_raw;
        s_snap.ina.stamp_us = now_us;
    }
    s_snap.ina.ok = ok;
    s_snap.ntc_c = ntc_c;
    portEXIT_CRITICAL(&s_snap_lock);
}

static void sensor_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    const TickType_t period = pdMS_TO_TICKS(1000 / ROVER_TOF_POLL_HZ);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        const uint64_t now_us = (uint64_t)esp_timer_get_time();
        for (int i = 0; i < BOARD_TOF_COUNT; i++) {
            tof_poll(i, now_us);
            /* A re-init walks the 91-byte config block and waits for a first
             * measurement, so the watchdog is fed between sensors. */
            esp_task_wdt_reset();
        }
        electrical_poll(now_us);
        esp_task_wdt_reset();
        xTaskDelayUntil(&last, period);
    }
}

void sensors_init(void)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = BOARD_GPIO_I2C_SDA,
        .scl_io_num = BOARD_GPIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, /* 4.7 kOhm externally */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    /* Every XSHUT low first: all three sensors answer at 0x29 until they are
     * addressed one at a time. */
    const gpio_config_t xshut = {
        .pin_bit_mask = (1ULL << BOARD_GPIO_TOF_XSHUT_FL) |
                        (1ULL << BOARD_GPIO_TOF_XSHUT_FR) |
                        (1ULL << BOARD_GPIO_TOF_XSHUT_CL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&xshut));
    for (int i = 0; i < BOARD_TOF_COUNT; i++) {
        gpio_set_level(k_tof_xshut[i], 0);
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    for (int i = 0; i < BOARD_TOF_COUNT; i++) {
        s_snap.tof[i].mm = ROVER_TOF_ERROR_MM;
        tof_bring_up(i);
    }

    ina226_init();
    ntc_init();

    const gpio_config_t bumper = {
        .pin_bit_mask = 1ULL << BOARD_GPIO_BUMPER,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /* 10 kOhm externally */
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&bumper));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BOARD_GPIO_BUMPER, bumper_isr, NULL));

    /* The e-stop node is also MCPWM FAULT2; reading its level is what
     * ctrl_flags b1 estop_released reports. */
    const gpio_config_t estop = {
        .pin_bit_mask = 1ULL << BOARD_GPIO_FAULT_ESTOP,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&estop));

    xTaskCreatePinnedToCore(sensor_task, "sensors", BOARD_SENSOR_STACK, NULL,
                            BOARD_SENSOR_PRIO, NULL, BOARD_IO_CORE);
}

void sensors_fill(rover_in_t *in, uint64_t now_us)
{
    snapshot_t snap;
    portENTER_CRITICAL(&s_snap_lock);
    snap = s_snap;
    portEXIT_CRITICAL(&s_snap_lock);

    uint16_t mm[BOARD_TOF_COUNT];
    uint8_t status = 0;
    for (int i = 0; i < BOARD_TOF_COUNT; i++) {
        const bool fresh = snap.tof[i].stamp_us != 0 &&
                           (now_us - snap.tof[i].stamp_us) <= ROVER_TOF_STALE_MS * 1000ULL;
        if (snap.tof[i].ok && fresh) {
            mm[i] = snap.tof[i].mm;
        } else {
            mm[i] = ROVER_TOF_ERROR_MM;
            status |= (uint8_t)(1u << i);
        }
    }

    in->tof_fl_mm = mm[BOARD_TOF_FL];
    in->tof_fr_mm = mm[BOARD_TOF_FR];
    in->tof_cliff_mm = mm[BOARD_TOF_CLIFF];
    in->tof_status = status;

    /* Same freshness test the ToF path gets: a bus stall that holds the last
     * good sample is a stale reading, not a discharging pack. */
    const bool ina_fresh =
        snap.ina.ok && snap.ina.stamp_us != 0 &&
        (now_us - snap.ina.stamp_us) <= ROVER_TOF_STALE_MS * 1000ULL;
    in->vbat_mv = ina_fresh ? snap.ina.vbat_mv : ROVER_VBAT_INVALID_MV;
    in->imotor_ma = ina_fresh ? snap.ina.imotor_ma : 0;
    in->ntc_c = snap.ntc_c;
    in->gyro_z_mrad_s = 32767; /* ICM-20948 deferred to G6; 32767 is "absent" */

    const bool hit = s_bumper_hit;
    s_bumper_hit = false;
    in->bumper_clear = (uint8_t)(!hit && gpio_get_level(BOARD_GPIO_BUMPER) == 1);

    /* Sensed above the coil: a closed mushroom reads high whatever the
     * MOTOR_EN FET is doing, so "the human pressed it" and "firmware
     * disarmed" cannot be confused. */
    in->estop_released = (uint8_t)(gpio_get_level(BOARD_GPIO_FAULT_ESTOP) == 1);
}
