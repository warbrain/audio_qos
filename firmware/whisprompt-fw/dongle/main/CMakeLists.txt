idf_component_register(SRCS "app_main.c" "dongle_diagnostics.c" "dongle_diagnostics_format.c"
        "rgb/rgb_led.c"
    INCLUDE_DIRS "."
        "rgb"
    REQUIRES dual_end_runtime dual_end_common dual_end_audio dual_end_espnow_link dual_end_ota dual_end_pairing dongle_uac_bridge dongle_usb_device nvs_flash esp_driver_gpio esp_driver_ledc)
