/**
 * bridge_uart.h / .c
 * 
 * UART bridge PC <-> JTAG <-> ESP32-S3 <-> Modem.
 * 
 * Allows testing AT commands by communicating with the modem via the ESP.
 * Includes cooked-mode echoing back to user's terminal.
 * 
 * Producer and Tee: bridge_read_input_task
 * - reads char from USB-JTAG-UART (user input)
 * - writes char to q_echo
 * - writes char to q_parse
 * 
 * Consumer 1: bridge_echo_task
 * - reads char from q_echo
 * - echoes back to user terminal in cooked-mode
 * 
 * Consumer 2: bridge_parser_task
 * - reads char from q_parse
 * - updates linebuffer to create finalized messages to be sent to modem
 * - sends final messages to mb_message
 * 
 * Consumer 3: bridge_modem_tx_task
 * - reads final message from mb_message
 * - transmits complete message with CRLF to modem
 * 
 * The JTAG-SWD UART part is implemented via the USB CDC.
 * 
 * Producer: bridge_read_modem_task
 * - reads full buffer from modem (UART) and sends to USB.
 * 
 * Janus, August 2025.
 */

#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/usb_serial_jtag.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"

#include "bridge_uart.h"

// Logging
#define TAG "BRIDGE"

// Synchronization objects
static QueueHandle_t q_echo;
static QueueHandle_t q_parse;
static MessageBufferHandle_t mb_message;

// FreeRTOS tasks
static void bridge_read_input_task (void *arg);
static void bridge_echo_task(void *arg);
static void bridge_parser_task(void *arg);
static void bridge_read_modem_task(void *arg);
static void bridge_spam_at_task(void *arg);

// Maintain linebuffer state. Fills up waiting for EOL (commit).
typedef struct {
    size_t write_pos;
    size_t MAX_POS;     //  = BRIDGE_UART_BUFFER_SIZE - 1
    uint8_t line[BRIDGE_UART_BUFFER_SIZE];
} linebuf_t;

// Helper functions
static inline void usb_write(const void *p, size_t n);
static void init_linebuf(linebuf_t*);
static void reset_linebuf(linebuf_t*);


// Initialize UART channels
esp_err_t bridge_uart_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate      = BRIDGE_MODEM_BAUD,
        .data_bits      = UART_DATA_8_BITS,
        .parity         = UART_PARITY_DISABLE,
        .stop_bits      = UART_STOP_BITS_1,
        .flow_ctrl      = UART_HW_FLOWCTRL_DISABLE,
        .source_clk     = UART_SCLK_DEFAULT
    };

    // Install UART driver with memory allocation
    ESP_ERROR_CHECK(
    uart_driver_install(
        BRIDGE_MODEM_UART,              // UART device
        2*BRIDGE_UART_BUFFER_SIZE,      // Rx buffer
        2*BRIDGE_UART_BUFFER_SIZE,      // Tx buffer
        0,                              // Queue size
        NULL,                           // Queue handle
        0                               // Interrupt flags
    ));
    ESP_LOGI(TAG, "UART init done");

    usb_serial_jtag_driver_config_t jtag_cfg = {
        .rx_buffer_size     = BRIDGE_UART_BUFFER_SIZE,
        .tx_buffer_size     = BRIDGE_UART_BUFFER_SIZE
    };

    // Install serial JTAG driver
    ESP_ERROR_CHECK(
    usb_serial_jtag_driver_install(&jtag_cfg));
    ESP_LOGI(TAG, "USB_SERIAL_JTAG init done");

    // Configure UART
    ESP_ERROR_CHECK(
    uart_param_config(
        BRIDGE_MODEM_UART, 
        &uart_cfg
    ));

    // Configure pinmux for Tx Rx
    ESP_ERROR_CHECK(
    uart_set_pin(
        BRIDGE_MODEM_UART,
        BRIDGE_MODEM_UART_TX_PIN,
        BRIDGE_MODEM_UART_RX_PIN, 
        BRIDGE_MODEM_UART_RTS_PIN,      // Not used
        BRIDGE_MODEM_UART_CTS_PIN       // Not used
    ));

    // Initialize queues
    q_echo = xQueueCreate(BRIDGE_Q_DEPTH, sizeof(uint8_t));
    q_parse = xQueueCreate(BRIDGE_Q_DEPTH, sizeof(uint8_t));

    // Initialize message buffer
    mb_message = xMessageBufferCreate(BRIDGE_MB_SIZE);

    // Fail if plumbing not ok
    configASSERT(q_echo && q_parse && mb_message);

    // Register tasks
    xTaskCreate(bridge_read_input_task,     "bridge",   BRIDGE_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(bridge_echo_task,           "echo",     BRIDGE_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(bridge_parser_task,         "parser",   BRIDGE_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(bridge_read_modem_task,     "modem",    BRIDGE_TASK_STACK_SIZE, NULL, 10, NULL);
    //xTaskCreate(bridge_spam_at_task,        "spam",    BRIDGE_TASK_STACK_SIZE, NULL, 10, NULL);
    ESP_LOGI(TAG, "Tasks started");

    return ESP_OK;
}


// Send chars directly to JTAG-USB
static inline void usb_write(const void *p, size_t n) {
    usb_serial_jtag_write_bytes((const char *)p, n, pdMS_TO_TICKS(50));
}


static void init_linebuf(linebuf_t *lb) {
    lb->write_pos = 0;
    lb->MAX_POS = BRIDGE_UART_BUFFER_SIZE - 1;
}


static void reset_linebuf(linebuf_t *lb) {
    lb->write_pos = 0;
}


static void bridge_read_input_task (void *arg) {
    // In this task, it is essential that we ensure to get all of the data from USB (USB must not overflow).
    uint8_t in_buf[BRIDGE_CDC_PACKET_SIZE];
    size_t n;

    for (;;) {
        // Get user input
        n = usb_serial_jtag_read_bytes(in_buf, sizeof(in_buf), pdMS_TO_TICKS(20));
        if (n <= 0) continue;

        // Tee - Consume bytes by sending to queues
        for (size_t i = 0; i < n; i++) {
            // must deliver to parser
            while (xQueueSend(q_parse, &in_buf[i], pdMS_TO_TICKS(20)) != pdTRUE) {
                vTaskDelay(1);
            }

            // non-blocking best effort to echo
            (void) xQueueSend(q_echo, &in_buf[i], 0);
        }
    }
}


// Echoes input back to the user's terminal
static void bridge_echo_task(void *arg) {
    uint8_t b;

    for (;;) {
        // try forever to read a byte from the queue
        if (xQueueReceive(q_echo, &b, portMAX_DELAY) != pdTRUE) continue;

        switch (b) {
            case 0x08:  // backspace
            case 0x7F:  // del
                usb_write("\b \b", 3);  // erase one character
                break;
            case 0x15:  // kill line with Ctrl-U
                usb_write("\r\n", 2);   // next line, to show aborted
                break;
            case '\r':
            case '\n':
                usb_write("\r\n", 2);   // next line, to show committed
                break;
            default:
                usb_write(&b, 1);
                break;
        }
    }
}


// Parses input to finalized messages
static void bridge_parser_task(void *arg) {
    uint8_t b;
    linebuf_t lb;
    init_linebuf(&lb);
    bool eol_flag = false;  // up when user has committed

    for (;;) {
        // try forever to read a byte from the queue
        if (xQueueReceive(q_parse, &b, portMAX_DELAY) != pdTRUE) continue;

        // EOL, raise eol_flag but continue processing to discard any subsequent EOL characters.
        if (b == '\r' || b == '\n') {
            eol_flag = true;
            //continue; // discard remaining EOL characters
        }

        // Post-EOL - Commit message
        if (eol_flag) {
            // TODO: Sink into mb instead of echoing it
            if (lb.write_pos > 0) {
                usb_write("> ", 2);
                usb_write(lb.line, lb.write_pos);
                usb_write("\r\n", 2);
                uart_write_bytes(BRIDGE_MODEM_UART, lb.line, lb.write_pos);
                uart_write_bytes(BRIDGE_MODEM_UART, "\r\n", 2);
            }    
            reset_linebuf(&lb);
            eol_flag = false;
            continue;               // go get another char
        }

        // No EOL yet - Update and edit linebuffer
        if (!eol_flag) {
            switch (b) {
                case 0x08:  // backspace
                case 0x7F:  // del
                    if (lb.write_pos > 0) lb.write_pos--;
                    break;
                case 0x15:  // Ctrl-U kill line
                    reset_linebuf(&lb);
                    break;
                default:    // other chars, write to buffer
                    if (lb.write_pos > lb.MAX_POS) {
                        // overflow - no more capacity in linebuffer
                        usb_write(lb.line, lb.write_pos);
                        reset_linebuf(&lb);
                    }
                    lb.line[lb.write_pos++] = b;
                    break;
            } // switch
        } // if
    } // for
}


static void bridge_read_modem_task(void *arg) {
    char rx[128];
    size_t n;

    for (;;) {
        n = uart_read_bytes(BRIDGE_MODEM_UART, rx, sizeof(rx)-1, 100/portTICK_PERIOD_MS);
        if (n > 0) {
            rx[n] = '\0';
            // TODO: This is where we handle the messages coming back from the modem - wire it up
            usb_write("< ", 2);
            usb_write(rx, n);
            usb_write("\r\n", 2);
            taskYIELD();
        }

    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

static void bridge_spam_at_task(void *arg) {
    const char *AT = "AT\r\n";
    for (;;) {
        uart_write_bytes(BRIDGE_MODEM_UART, AT, strlen(AT));
        usb_write(AT, 4);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
