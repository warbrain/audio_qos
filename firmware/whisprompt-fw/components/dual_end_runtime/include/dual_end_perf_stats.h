#pragma once
#include <stdbool.h>
#include <stdint.h>
#define DUAL_END_PERF_HISTOGRAM_BUCKETS 9
#define DUAL_END_PERF_FRAME_BUDGET_US UINT32_C(20000)
extern const uint32_t dual_end_perf_histogram_upper_us[DUAL_END_PERF_HISTOGRAM_BUCKETS];
/* Owner admission is independent of elapsed-time overruns. A callback waiting
 * for a session/ACK did not attempt a codec operation; do not call it a codec
 * failure. These counters close with the same immutable performance window. */
typedef enum {
    DUAL_END_OUTCOME_OK = 0, DUAL_END_OUTCOME_OPERATION_FAILED,
    DUAL_END_OUTCOME_WAIT_SESSION, DUAL_END_OUTCOME_WAIT_ACK,
    DUAL_END_OUTCOME_DEADLINE, DUAL_END_OUTCOME_POWER,
    DUAL_END_OUTCOME_INVALID_REQUEST, DUAL_END_OUTCOME_UNCLASSIFIED,
    DUAL_END_OUTCOME_COUNT
} dual_end_owner_outcome_t;
typedef struct {
    uint8_t codec;
    uint32_t session_id;
    int expected_core, observed_core;
    uint32_t observed_core_mask;
    uint32_t samples, failures, over_budget, core_wrong, invalid_core;
    uint32_t invalid_duration, overflow;
    uint64_t sum_us;
    uint32_t min_us, max_us;
    uint32_t histogram[DUAL_END_PERF_HISTOGRAM_BUCKETS];
    uint32_t outcomes[DUAL_END_OUTCOME_COUNT];
} dual_end_perf_stats_t;
/* Single producer. Reset at each codec/session/window boundary, not per frame. */
void dual_end_perf_reset(dual_end_perf_stats_t *stats, uint8_t codec,
    uint32_t session_id, int expected_core);
/* Elapsed wall time includes preemption; this is not CPU utilization.
 * Negative duration is rejected. All counters saturate; overflow is explicit.
 * Invalid core records no valid core-mask bit and is separate from core_wrong. */
bool dual_end_perf_record(dual_end_perf_stats_t *stats, int64_t elapsed_us,
    int observed_core, bool success);
bool dual_end_perf_record_outcome(dual_end_perf_stats_t *stats, int64_t elapsed_us,
    int observed_core, dual_end_owner_outcome_t outcome);
/* False for no samples, all failed attempts, overflow or invalid summaries. Outputs then zero,
 * meaning unknown. p95 is the nearest-rank histogram upper bound, not an exact
 * quantile; UINT32_MAX is the unbounded final bucket. */
bool dual_end_perf_summary(const dual_end_perf_stats_t *stats,
    uint32_t *mean_us, uint32_t *p95_upper_us);
