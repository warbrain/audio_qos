#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

/* Starts the optional, low-priority copy worker after microphone startup.
 * Failure is non-fatal: capture, radio, codec, pairing and legacy logging
 * continue unchanged. */
esp_err_t scepter_diagnostics_start(void);

/* Native USB callback for this bounded protocol:
 *   DIAG_CONTEXT <token> <run_id> <arm_id>
 *   SCEPTER_TELEMETRY <token> <kind> <part>
 * kind: 0=spatial, 1=pipeline, 2=capture_health, 3=owner_timing, 4=nlms,
 *       5=wiener (mode 4 cumulative state or mode 5 independent L/R state;
 *       unavailable outside Wiener modes), 6=dispatch (PCM slot/ring state),
 *       7=MVDR (mask/covariance/queue health; unavailable outside mode 6).
 * Successful part responses carry run/arm/ctx/cache plus boot/session/codec/
 * mode/generation/capture_generation before the original JSON event.  Hosts
 * must group multipart records by those identity fields and retry if any
 * member differs.  After a nonzero run/arm context ACK, queries return
 * context_pending until one new immutable microphone window closes after the
 * context boundary and binds one session/codec/mode/generation identity; an
 * older cache or a later cross-identity cache is never relabelled.  It reads
 * only completed cached snapshots
 * and returns false for every other command so legacy runtime/pairing/OTA
 * command handling is unchanged. */
bool scepter_diagnostics_handle_command(const char *line,
                                        char *response,
                                        size_t capacity,
                                        void *ctx);
