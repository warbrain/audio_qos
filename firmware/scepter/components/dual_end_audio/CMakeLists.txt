idf_component_register(SRCS "dual_end_audio.c"
    INCLUDE_DIRS "include"
    REQUIRES dual_end_runtime esp_timer esp_rom dual_end_common dual_end_espnow_link dual_end_audio_recovery espressif__esp_audio_codec dual_end_audio_codec)
