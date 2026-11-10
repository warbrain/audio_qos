idf_component_register(SRCS "dual_end_ota.c"
    INCLUDE_DIRS "include"
    REQUIRES dual_end_runtime app_update esp_partition esp_app_format esp_timer esp_wifi esp_driver_uart nvs_flash dual_end_common dual_end_espnow_link)
