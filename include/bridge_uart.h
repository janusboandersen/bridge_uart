 /**
  * bridge_uart.h / .c
  * 
  * Minimal UART bridge to exchange AT commands and responses with a modem.
  * PC <--jtag/swd/uart0 --> ESP32-S3 <-- uart1 --> Modem
  * 
  * TODO: Kconfig for setting ports, baud, etc.
  * 
  * Janus, August 2025.
  */
#pragma once

#include "esp_err.h"
#include "driver/uart.h"

// Modem on UART1
#define BRIDGE_MODEM_UART              UART_NUM_1
#define BRIDGE_MODEM_BAUD              115200
#define BRIDGE_MODEM_UART_RX_PIN       4                       // ESP32 Rx <- ... <- Modem Tx
#define BRIDGE_MODEM_UART_TX_PIN       5                       // ESP32 Tx -> ... -> Modem Rx
#define BRIDGE_MODEM_UART_RTS_PIN      UART_PIN_NO_CHANGE      // Flow control inactive
#define BRIDGE_MODEM_UART_CTS_PIN      UART_PIN_NO_CHANGE      // Same

// Runtime stacks and buffers
#define BRIDGE_TASK_STACK_SIZE         4096
//#define BRIDGE_PC_TO_MODEM_TASK_STACK   4096
//#define BRIDGE_MODEM_TO_PC_TASK_STACK   4096
#define BRIDGE_UART_BUFFER_SIZE         512

#define BRIDGE_Q_DEPTH                  256                     // FreeRTOS queue depth, num chars
#define BRIDGE_LINE_MAX                 256                     // Max input line length, num chars
#define BRIDGE_MB_SIZE                  (LINE_MAX + 16)         // Messages to modem, bytes
#define BRIDGE_CDC_PACKET_SIZE          64                      // CDC typically sends chunks of 64 bytes

// Initialize UART channel for modem
esp_err_t bridge_uart_init(void);

// FreeRTOS task looping UART from PC back to PC
//void bridge_usb_echo_task(void *arg);

// FreeRTOS task transmitting bytes from PC to modem
//void bridge_pc_to_modem_task(void *arg);

// FreeRTOS task transmitting bytes from modem to PC
//void bridge_modem_to_pc_task(void *arg);
