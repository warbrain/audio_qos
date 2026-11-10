#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "dual_end_link_config.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Optional native USB command extension.  The Scepter registers this before
 * starting the pairing USB task.  A callback must return false for commands
 * it does not own, preserving runtime/pairing/OTA command behavior. */
typedef bool (*dual_end_pairing_diagnostics_command_cb_t)(const char *line,
                                                           char *response,
                                                           size_t response_size,
                                                           void *ctx);

void dual_end_pairing_set_diagnostics_command_cb(
    dual_end_pairing_diagnostics_command_cb_t callback,
    void *ctx);

/**
 * Initialize the product pairing command service.
 *
 * Set start_native_usb=true on Scepter to own the ESP32-S3 USB Serial/JTAG
 * command channel. Dongle passes false and feeds commands received by its
 * existing TinyUSB composite CDC endpoint to dual_end_pairing_handle_command().
 */
esp_err_t dual_end_pairing_service_init(scepter_role_t role,
                                        bool start_native_usb);

/**
 * Process PAIR INFO/PREPARE/COMMIT/CONFIRM/ROLLBACK/CLEAR.
 *
 * Returns true when the line belongs to the PAIR command family. A complete
 * newline-terminated response is written to response.
 */
bool dual_end_pairing_handle_command(const char *line,
                                     char *response,
                                     size_t response_size);

/**
 * Return and clear the pending Scepter pairing confirmation.
 *
 * This becomes true after a successful PAIR COMMIT on Scepter and is consumed
 * when the ESP-NOW link first reaches connected state.
 */
bool dual_end_pairing_take_link_confirmation_pending(void);

#ifdef __cplusplus
}
#endif
