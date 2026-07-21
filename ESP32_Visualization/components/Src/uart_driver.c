#include "uart_driver.h"
#include "esp_log.h"

static const char *TAG = "UART_DRIVER";

/* Install UART driver, configure params and pins. */
void uart_driver_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART1 inicijaliziran: RX=GPIO%d, TX=GPIO%d, BAUD=%d",
             UART_RX_PIN, UART_TX_PIN, UART_BAUD_RATE);
}