#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "dual_end_link_config.h"

#define DUAL_END_LOG_TRANSPORT_LINE_BYTES 512U
#define DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH 32U
typedef struct {
    bool enabled;
    uint32_t queue_drop, format_drop, short_write, backend_error, queue_highwater;
    /* Exclusive reasons: long includes missing LF space; chars includes
     * invalid/empty formatting, embedded NUL and fragmented/multiline records. */
    uint32_t format_drop_long, format_drop_chars;
} dual_end_log_transport_stats_t;

/* Optional LOG_VERSION1 primary USB Serial/JTAG sink, Scepter only. Other
 * roles/consoles keep their existing backend and return ESP_OK with disabled
 * status. Call once from startup after console/pairing initialization. Failure
 * leaves the previous log sink/USB driver ownership intact; audio is unchanged.
 * Enabled producers only format into a bounded stack record and enqueue with
 * zero wait: no heap allocation, USB/stdout lock or transport wait on owners.
 * The Core0 priority1 worker owns USB writes and shares stdout's line lock with
 * existing read-only/OTA replies. No RX, DTR, reset or USB-device changes. */
esp_err_t dual_end_log_transport_start(scepter_role_t role);
/* Thread-safe cumulative saturating counters; queue_drop is enqueue-full,
 * format_drop is invalid/overlength/fragmented records, short_write counts
 * incomplete write attempts, backend_error counts unavailable/failed backend
 * records; queue_highwater is the largest observed queued record count.
 * Snapshot does not log, scan heap, block on USB, or clear counters. */
void dual_end_log_transport_snapshot(dual_end_log_transport_stats_t *out);
