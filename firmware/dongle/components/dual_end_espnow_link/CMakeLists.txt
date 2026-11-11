idf_component_register(SRCS "dual_end_espnow_link.c"
    INCLUDE_DIRS "include"
    REQUIRES dual_end_runtime esp_wifi esp_event esp_timer nvs_flash dual_end_common dual_end_secure_pairing dual_end_audio_recovery)
