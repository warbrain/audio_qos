#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "mic_diagnostics_snapshot.h"

/* Query kinds are intentionally numeric and fixed so the CDC parser stays
 * bounded.  Each part keeps the existing event/schema contract used by the
 * console logger; this extension only exposes completed cache records. */
enum {
    SCEPTER_DIAGNOSTICS_KIND_SPATIAL = 0U,
    SCEPTER_DIAGNOSTICS_KIND_PIPELINE = 1U,
    SCEPTER_DIAGNOSTICS_KIND_CAPTURE_HEALTH = 2U,
    SCEPTER_DIAGNOSTICS_KIND_OWNER_TIMING = 3U,
    SCEPTER_DIAGNOSTICS_KIND_NLMS = 4U,
    SCEPTER_DIAGNOSTICS_KIND_WIENER = 5U,
    SCEPTER_DIAGNOSTICS_KIND_DISPATCH = 6U,
    SCEPTER_DIAGNOSTICS_KIND_COUNT = 7U,
};

/* Eight is the maximum: mode 5 emits separate L/R uint64 counter parts so
 * the bounded 180-byte telemetry frame remains valid at the full u64 range. */
#define SCEPTER_DIAGNOSTICS_WIENER_PARTS 8U

#define SCEPTER_DIAGNOSTICS_JSON_BUFFER_BYTES DUAL_END_TELEMETRY_JSON_BYTES

/* Returns zero for an invalid kind/part or for a spatial identity mismatch.
 * `active` is false only for an intentionally inactive NLMS window; its
 * zero-valued JSON is then state evidence, not a sampled performance zero. */
size_t scepter_diagnostics_format_part(
    const mic_audio_diagnostics_snapshot_t *snapshot,
    unsigned kind,
    unsigned part,
    char *buffer,
    size_t capacity,
    const char **event_name,
    unsigned *schema,
    bool *active);

unsigned scepter_diagnostics_part_count(unsigned kind);
