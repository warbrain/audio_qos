idf_component_register(SRCS "dongle_uac_bridge.c"
    INCLUDE_DIRS "include"
    REQUIRES dual_end_common dual_end_audio_recovery esp_timer espressif__usb_device_uac)

# The host bridge regression intentionally uses the source default (legacy
# ring enabled). The production component accepts only absolute-timeline PCM,
# so do not reserve the 32 KiB compatibility ring in its static data.
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING=0)
