
#pragma once

#define TX_PIN 5
#define RX_PIN 4
// #define PWRKEY_PIN 41
#define UART_PORT UART_NUM_1

void init_uart();
// void power_on_sim7080g();
void spam_task(void *arg);
void read_task(void *arg);