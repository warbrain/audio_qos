idf_component_register(
    SRCS "dual_end_audio.c" "audio_rx_telemetry.c" "audio_rx_telemetry_format.c"
    INCLUDE_DIRS "include"
    REQUIRES dual_end_runtime esp_timer esp_rom freertos dual_end_common dual_end_espnow_link
             dual_end_ota dual_end_audio_recovery dongle_uac_bridge
             espressif__esp_audio_codec dual_end_audio_codec)
