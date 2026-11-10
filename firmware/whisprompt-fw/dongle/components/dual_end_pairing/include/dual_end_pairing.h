#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "dual_end_link_config.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t dual_end_pairing_service_init(scepter_role_t role,
                                        bool start_native_usb);

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
