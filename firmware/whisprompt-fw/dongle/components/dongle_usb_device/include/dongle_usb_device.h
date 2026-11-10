#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DONGLE_HID_REPORT_ID_KEYBOARD 1
#define DONGLE_HID_REPORT_ID_MOUSE 2
#define DONGLE_HID_REPORT_ID_CONFIG 3
#define DONGLE_HID_CONFIG_REPORT_SIZE 16
#define DONGLE_HID_REPORT_TYPE_OUTPUT 2
#define DONGLE_HID_REPORT_TYPE_FEATURE 3
#define DONGLE_HID_CONFIG_CMD_SET_BUTTON 0x01
#define DONGLE_HID_CONFIG_CMD_RESET 0x02
#define DONGLE_HID_CONFIG_CMD_FORWARD_TO_SCEPTER 0x10

typedef struct {
    uint8_t modifier;
    uint8_t reserved;
    uint8_t keycode[6];
} dongle_hid_keyboard_report_t;

typedef struct {
    uint8_t buttons;
    int16_t x;
    int16_t y;
    int8_t vertical;
    int8_t horizontal;
} dongle_hid_mouse_report_t;

typedef struct {
    bool mounted;
    bool suspended;
    bool cdc_connected;
    bool cdc_dtr;
    uint32_t cdc_rx_commands;
    uint32_t cdc_rx_queue_overflows;
    uint32_t cdc_response_drops;
    uint64_t cdc_response_tx_queued_bytes;
    uint32_t cdc_response_tx_incomplete;
    uint32_t mouse_packets_received;
    uint32_t mouse_reports_sent;
    uint32_t mouse_hid_not_ready;
} dongle_usb_device_stats_t;

/* Boot-cumulative, saturating CDC mirror counters. attempted counts callbacks
 * eligible for the connected, enabled mirror; enqueued + dropped classifies
 * those callbacks. truncated is the dropped subset exceeding the line bound,
 * including missing final-LF space. No truncated JSON prefix is emitted.
 * write_failed counts queued records rejected by the TX worker or a backend
 * failure; accepted TinyUSB bytes are not acknowledgement by the host. */
typedef struct {
    uint32_t attempted;
    uint32_t enqueued;
    uint32_t dropped;
    uint32_t truncated;
    uint32_t write_failed;
} dongle_cdc_log_stats_t;

/* No device I/O, logging, heap scan, reset or counter clearing. */
void dongle_usb_cdc_get_log_stats(dongle_cdc_log_stats_t *out);

typedef esp_err_t (*dongle_usb_hid_config_report_cb_t)(uint8_t report_id,
                                                       uint8_t report_type,
                                                       const uint8_t *buffer,
                                                       uint16_t bufsize,
                                                       void *ctx);

/* Invoked by the CDC command worker after the legacy runtime commands. The
 * callback must only format bounded cached data: it runs with DTR possibly
 * low and must not take ownership of the input or response buffers. */
typedef bool (*dongle_usb_diagnostics_command_cb_t)(const char *line,
                                                     char *response,
                                                     size_t capacity,
                                                     void *ctx);

void dongle_usb_device_set_hid_config_report_cb(dongle_usb_hid_config_report_cb_t cb, void *ctx);
void dongle_usb_device_set_diagnostics_command_cb(dongle_usb_diagnostics_command_cb_t cb,
                                                   void *ctx);
esp_err_t dongle_usb_device_start(void);
void dongle_usb_device_reset_audio_session(void);
esp_err_t dongle_usb_device_send_keyboard_report(const dongle_hid_keyboard_report_t *report);
esp_err_t dongle_usb_device_send_mouse_report(const dongle_hid_mouse_report_t *report);
void dongle_usb_device_note_mouse_packet_received(void);
void dongle_usb_device_get_stats(dongle_usb_device_stats_t *out_stats);
void dongle_usb_device_cdc_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

void dongle_usb_device_set_config_reply(const uint8_t reply[16]);
