idf_component_register(
    SRCS
        "src/bridge_uart.c"
        #"src/uart_spam.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_common      # uses esp_err.h in public headers
        esp_driver_uart # same
    PRIV_REQUIRES
        esp_driver_usb_serial_jtag
        esp_driver_gpio
        freertos        # freertos only used in impl.
)
