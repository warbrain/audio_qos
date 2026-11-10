idf_component_register(SRCS "dual_end_opus.c" "dual_end_spatial.c" "dual_end_wiener.c"
    INCLUDE_DIRS "include"
    REQUIRES dual_end_common espressif__esp_audio_codec)
