idf_component_register(
    SRCS "dual_end_pairing.c"
    INCLUDE_DIRS "include"
    REQUIRES dual_end_runtime
        dual_end_common
        dual_end_espnow_link
        dual_end_ota
        esp_app_format
        esp_driver_usb_serial_jtag
        esp_hw_support
        esp_system
        esp_timer
        log
        mbedtls
)
