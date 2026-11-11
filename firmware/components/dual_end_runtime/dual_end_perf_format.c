#include "dual_end_perf_format.h"
#include <inttypes.h>
#include <stdio.h>
size_t dual_end_perf_format_part(const dual_end_perf_record_t *r,
    unsigned part, char *buf, size_t cap)
{
    if (!r || !buf || !cap || part >= DUAL_END_PERF_PART_COUNT) return 0;
    buf[0] = '\0';
    int prefix = snprintf(buf, cap, "{\"event\":\"audio.codec_perf\",\"schema\":1,\"part\":%u,"
        "\"boot_id\":\"%08" PRIx32 "\",\"window_seq\":%" PRIu32 ",\"role\":%u",
        part, r->boot_id, r->window_seq, r->role);
    if (prefix < 0 || (size_t)prefix >= cap) { buf[0] = '\0'; return 0; }
    uint32_t mean = 0, p95 = 0;
    bool known = dual_end_perf_summary(&r->stats, &mean, &p95);
    const uint32_t minimum = r->stats.samples ? r->stats.min_us : 0;
    const uint32_t successful = r->stats.samples > r->stats.failures ?
        r->stats.samples - r->stats.failures : 0;
    const uint32_t *h = r->stats.histogram;
    char *out = buf + prefix;
    const size_t remaining = cap - (size_t)prefix;
    int count = -1;
    switch (part) {
    case 0: count = snprintf(out, remaining,
        ",\"part_count\":13,\"session_id\":%" PRIu32 ",\"codec\":%u,\"stage\":%u}",
        r->stats.session_id, r->stats.codec, r->stage); break;
    case 1: count = snprintf(out, remaining,
        ",\"start_us\":%" PRIu64 ",\"end_us\":%" PRIu64 "}", r->start_us, r->end_us); break;
    case 2: count = snprintf(out, remaining,
        ",\"samples\":%" PRIu32 ",\"failures\":%" PRIu32 ",\"invalid_duration\":%" PRIu32 "}",
        r->stats.samples, r->stats.failures, r->stats.invalid_duration); break;
    case 3: count = snprintf(out, remaining,
        ",\"sum_us\":%" PRIu64 ",\"budget_us\":20000,\"successful\":%" PRIu32 "}",
        r->stats.sum_us, successful); break;
    case 4: count = snprintf(out, remaining,
        ",\"mean_us\":%" PRIu32 ",\"min_us\":%" PRIu32 ",\"max_us\":%" PRIu32 "}",
        mean, minimum, r->stats.max_us); break;
    case 5: count = snprintf(out, remaining,
        ",\"p95_upper_us\":%" PRIu32 ",\"known\":%s,\"over_budget\":%" PRIu32 "}",
        p95, known ? "true" : "false", r->stats.over_budget); break;
    case 6: count = snprintf(out, remaining,
        ",\"expected_core\":%d,\"observed_core\":%d,\"mask\":%" PRIu32 "}",
        r->stats.expected_core, r->stats.observed_core, r->stats.observed_core_mask); break;
    case 7: count = snprintf(out, remaining,
        ",\"core_wrong\":%" PRIu32 ",\"invalid_core\":%" PRIu32 ",\"overflow\":%" PRIu32 "}",
        r->stats.core_wrong, r->stats.invalid_core, r->stats.overflow); break;
    case 8: count = snprintf(out, remaining,
        ",\"hist0\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]}",
        h[0], h[1], h[2], h[3], h[4]); break;
    case 9: count = snprintf(out, remaining,
        ",\"hist1\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]}",
        h[5], h[6], h[7], h[8]); break;
    case 10: count = snprintf(out, remaining,
        ",\"heap_internal\":%" PRIu32 ",\"heap_min_internal\":%" PRIu32 "}",
        r->heap_internal, r->heap_min_internal); break;
    case 11: count = snprintf(out, remaining,
        ",\"heap_psram\":%" PRIu32 ",\"stack_min\":%" PRIu32 "}",
        r->heap_psram, r->stack_min); break;
    case 12: count = snprintf(out, remaining,
        ",\"resource_at_us\":%" PRIu64 ",\"telemetry_drop\":%" PRIu32 "}",
        r->resource_at_us, r->telemetry_drop); break;
    }
    if (count < 0 || (size_t)count >= remaining ||
        (size_t)prefix + (size_t)count > DUAL_END_PERF_JSON_MAX_BYTES) {
        buf[0] = '\0'; return 0;
    }
    return (size_t)prefix + (size_t)count;
}
size_t dual_end_owner_outcome_format_part(const dual_end_perf_record_t *r,
    unsigned part, char *buf, size_t cap)
{
    if (buf && cap) buf[0] = '\0';
    if (!r || !buf || !cap || r->role != 0 || r->stage != 1 ||
        part >= DUAL_END_OWNER_OUTCOME_PART_COUNT) return 0;
    int prefix = snprintf(buf, cap, "{\"event\":\"audio.owner_outcome\",\"schema\":1,\"part\":%u,"
        "\"boot_id\":\"%08" PRIx32 "\",\"window_seq\":%" PRIu32 ",\"role\":%u",
        part, r->boot_id, r->window_seq, r->role);
    if (prefix < 0 || (size_t)prefix >= cap) { buf[0] = '\0'; return 0; }
    const uint32_t *o = r->stats.outcomes;
    char *out = buf + prefix; const size_t left = cap - (size_t)prefix;
    int n = -1;
    switch (part) {
    case 0: n = snprintf(out, left, ",\"part_count\":7,\"session_id\":%" PRIu32 ",\"codec\":%u,\"stage\":1}", r->stats.session_id, r->stats.codec); break;
    case 1: n = snprintf(out, left, ",\"start_us\":%" PRIu64 ",\"end_us\":%" PRIu64 "}", r->start_us, r->end_us); break;
    case 2: n = snprintf(out, left, ",\"ok\":%" PRIu32 ",\"operation_failed\":%" PRIu32 "}", o[0], o[1]); break;
    case 3: n = snprintf(out, left, ",\"wait_session\":%" PRIu32 ",\"wait_ack\":%" PRIu32 "}", o[2], o[3]); break;
    case 4: n = snprintf(out, left, ",\"deadline\":%" PRIu32 ",\"power\":%" PRIu32 "}", o[4], o[5]); break;
    case 5: n = snprintf(out, left, ",\"invalid_request\":%" PRIu32 ",\"unclassified\":%" PRIu32 "}", o[6], o[7]); break;
    case 6: n = snprintf(out, left, ",\"samples\":%" PRIu32 ",\"overflow\":%" PRIu32 "}", r->stats.samples, r->stats.overflow); break;
    }
    if (n < 0 || (size_t)n >= left || (size_t)prefix + (size_t)n > DUAL_END_PERF_JSON_MAX_BYTES) {
        buf[0] = '\0'; return 0;
    }
    return (size_t)prefix + (size_t)n;
}
