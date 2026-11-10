#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

/* Starts a low-priority cache worker after the Dongle services are ready. */
esp_err_t dongle_diagnostics_start(void);

/* CDC callback for DIAG_CONTEXT and DONGLE_TELEMETRY. It consumes only
 * cached snapshots, is safe with DTR low, and returns false for all other
 * commands so legacy command processing remains unchanged. Part zero freezes
 * one cached record for a token; subsequent parts and retries use that copy.
 * Each command family has one 10-second fixed lease, invalidated by context
 * changes. Expired/mismatched nonzero parts require a fresh part zero. */
bool dongle_diagnostics_handle_command(const char *line,
                                       char *response,
                                       size_t capacity,
                                       void *ctx);
