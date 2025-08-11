#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uart_spam.h"


void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200, //9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
}

// void power_on_sim7080g() {
//     gpio_set_direction(PWRKEY_PIN, GPIO_MODE_OUTPUT);

//     gpio_set_level(PWRKEY_PIN, 1);
//     vTaskDelay(1000 / portTICK_PERIOD_MS); // Hold PWRKEY low for 1 second

//     gpio_set_level(PWRKEY_PIN, 0);
//     //vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait for the module to initialize
// }

void read_task(void *arg) {
    char data[128];
    for (;;) {
        int len = uart_read_bytes(UART_PORT, data, sizeof(data), 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            printf("Response: %s\n", data);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void spam_task(void *arg) {
    char *test_cmd = "AT\r\n";
    for (;;) {
        uart_write_bytes(UART_PORT, test_cmd, strlen(test_cmd));
        printf("Sent: %s", test_cmd);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}