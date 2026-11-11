idf_component_register(SRCS "dongle_usb_device.c" "usb_descriptors.c"
    INCLUDE_DIRS "include" "tinyusb"
    REQUIRES dual_end_runtime app_update esp_app_format esp_timer dual_end_common dual_end_ota dual_end_pairing dongle_uac_bridge espressif__esp_tinyusb espressif__usb_device_uac)

idf_build_get_property(build_components BUILD_COMPONENTS)
if(tinyusb IN_LIST build_components)
    set(tusb_name tinyusb)
else()
    set(tusb_name espressif__tinyusb)
endif()

idf_component_get_property(tusb_lib ${tusb_name} COMPONENT_LIB)
idf_component_get_property(uac_dir espressif__usb_device_uac COMPONENT_DIR)
idf_component_get_property(uac_lib espressif__usb_device_uac COMPONENT_LIB)

# Keep the upstream 0.19 audio driver fixes in the project, not in the managed
# dependency cache: reset queued PCM before loading IN and correct FIFO sizing.
get_target_property(tusb_sources ${tusb_lib} SOURCES)
list(FILTER tusb_sources EXCLUDE REGEX "(^|/)audio_device\\.c$")
set_property(TARGET ${tusb_lib} PROPERTY SOURCES "${tusb_sources}")
target_sources(${tusb_lib} PRIVATE "${COMPONENT_DIR}/tinyusb/audio_device.c")

target_include_directories(${tusb_lib} BEFORE PUBLIC
    "${COMPONENT_DIR}/tinyusb"
    "${uac_dir}/tusb_uac")

target_include_directories(${uac_lib} BEFORE PUBLIC
    "${COMPONENT_DIR}/tinyusb"
    "${uac_dir}/tusb_uac")
