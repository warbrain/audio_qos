idf_component_register(SRCS "dual_end_perf_stats.c" "dual_end_perf_format.c" "dual_end_runtime.c" "dual_end_resource_format.c" "dual_end_telemetry_format.c" "dual_end_log_transport.c" "dual_end_log_health_format.c"
    INCLUDE_DIRS "include"
    REQUIRES dual_end_common esp_timer esp_hw_support esp_system esp_app_format heap freertos log
    PRIV_REQUIRES esp_driver_usb_serial_jtag)
