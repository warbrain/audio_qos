#pragma once
#include <stddef.h>
#include <stdint.h>
#include "dual_end_perf_stats.h"
#define DUAL_END_PERF_PART_COUNT 13U
#define DUAL_END_OWNER_OUTCOME_PART_COUNT 7U
#define DUAL_END_PERF_JSON_MAX_BYTES 180U
#define DUAL_END_PERF_JSON_BUFFER_BYTES (DUAL_END_PERF_JSON_MAX_BYTES + 1U)
typedef struct {
    uint8_t role, stage;
    uint32_t boot_id, window_seq;
    uint64_t start_us, end_us, resource_at_us;
    dual_end_perf_stats_t stats;
    uint32_t heap_internal, heap_min_internal, heap_psram, stack_min, telemetry_drop;
} dual_end_perf_record_t;
/* Full JSON per part, no prefix/newline. Returns 0 on invalid/truncated output;
 * never publish a partial JSON. All 13 parts identify boot/role/window. */
size_t dual_end_perf_format_part(const dual_end_perf_record_t *record,
    unsigned part, char *buffer, size_t capacity);
/* Additive event: legacy 13-part codec windows remain byte-compatible.
 * Join by boot/window/session/codec/stage; old firmware has unknown reasons. */
size_t dual_end_owner_outcome_format_part(const dual_end_perf_record_t *record,
    unsigned part, char *buffer, size_t capacity);
