idf_component_register(
    SRCS
        "app_main.c"
        "scepter_diagnostics.c"
        "scepter_diagnostics_format.c"
        "scepter_hardware.c"
        "battery/battery_monitor.c"
        "gpio/gpio_isr_service.c"
        "i2c/shared_i2c.c"
        "imu/imu_module.c"
        "keys/user_keys.c"
        "light_sleep/light_sleep_module.c"
        "lcd/anim_rle.c"
        "lcd/lcd_battery_anim_assets.c"
        "lcd/lcd_driver.c"
        "lcd/lcd_display.c"
        "lcd/lcd_font.c"
        "lcd/lcd_ui.c"
        "lcd/u8g2_font_big22.c"
        "lcd/u8g2_font_small.c"
        "mic/mic_audio.c"
        "mic/mic_analysis.c"
        "mic/mic_observability_telemetry.c"
        "mic/mic_pipeline_telemetry.c"
        "motor/motor_vibration.c"
        "rgb/rgb_led.c"
        "touchpad/touchpad_module.c"
        "wheel/wheel_module.c"
    INCLUDE_DIRS
        "."
        "battery"
        "gpio"
        "i2c"
        "imu"
        "keys"
        "light_sleep"
        "lcd"
        "mic"
        "motor"
        "rgb"
        "touchpad"
        "wheel"
    REQUIRES dual_end_runtime
        dual_end_common
        dual_end_audio
        dual_end_espnow_link
        dual_end_ota
        dual_end_pairing
        nvs_flash
        esp_event
        esp_netif
        esp_wifi
        esp_lcd
        esp_driver_gpio
        esp_driver_pcnt
        esp_driver_i2c
        esp_driver_i2s
        esp_driver_ledc
        esp_driver_spi
        heap
        esp_timer
        esp_adc
        esp_hw_support
        spi_flash
)

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    set(UI_ANIMATION_BIN "${CMAKE_CURRENT_LIST_DIR}/../animation.bin")
    if(NOT EXISTS "${UI_ANIMATION_BIN}")
        message(FATAL_ERROR "Missing UI animation image: ${UI_ANIMATION_BIN}")
    endif()

    partition_table_get_partition_info(UI_ANIMATION_OFFSET
        "--partition-name animation" "offset")
    if(NOT UI_ANIMATION_OFFSET)
        message(FATAL_ERROR "Partition table is missing the 'animation' partition")
    endif()

    esptool_py_flash_target_image(flash animation
        "${UI_ANIMATION_OFFSET}" "${UI_ANIMATION_BIN}")
endif()
