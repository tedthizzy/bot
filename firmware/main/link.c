#include "link.h"

#include "board.h"
#include "bytering.h"
#include "driver/uart.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bytering_t s_rx;
static bytering_t s_tx;
static uint8_t s_rx_storage[BOARD_LINK_RX_RING];
static uint8_t s_tx_storage[BOARD_LINK_TX_RING];

size_t link_rx_pop(uint8_t *dst, size_t cap)
{
    return bytering_pop(&s_rx, dst, cap);
}

size_t link_tx_push(const uint8_t *src, size_t n)
{
    return bytering_push(&s_tx, src, n);
}

static void comms_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    uint8_t buf[128];
    for (;;) {
        const int n = uart_read_bytes(BOARD_UART_PORT, buf, sizeof buf, pdMS_TO_TICKS(5));
        if (n > 0) {
            bytering_push(&s_rx, buf, (size_t)n);
        }
        size_t m;
        while ((m = bytering_pop(&s_tx, buf, sizeof buf)) > 0) {
            uart_write_bytes(BOARD_UART_PORT, buf, m);
        }
        esp_task_wdt_reset();
    }
}

void link_init(void)
{
    bytering_init(&s_rx, s_rx_storage, sizeof s_rx_storage);
    bytering_init(&s_tx, s_tx_storage, sizeof s_tx_storage);

    const uart_config_t cfg = {
        .baud_rate = BOARD_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(BOARD_UART_PORT, BOARD_LINK_RX_RING,
                                        BOARD_LINK_TX_RING, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BOARD_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(BOARD_UART_PORT, BOARD_GPIO_UART_TX, BOARD_GPIO_UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreatePinnedToCore(comms_task, "comms", BOARD_COMMS_STACK, NULL, BOARD_COMMS_PRIO,
                            NULL, BOARD_IO_CORE);
}
