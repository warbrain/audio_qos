#include "dual_end_perf_stats.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>
const uint32_t dual_end_perf_histogram_upper_us[DUAL_END_PERF_HISTOGRAM_BUCKETS] =
    {100, 250, 500, 1000, 2500, 5000, 10000, 20000, UINT32_MAX};
static void overflow_note(dual_end_perf_stats_t *s)
{
    if (s->overflow != UINT32_MAX) ++s->overflow;
}
static void counter_add(dual_end_perf_stats_t *s, uint32_t *counter)
{
    if (*counter == UINT32_MAX) overflow_note(s);
    else ++*counter;
}
void dual_end_perf_reset(dual_end_perf_stats_t *s, uint8_t codec,
    uint32_t session_id, int expected_core)
{
    if (!s) return;
    memset(s, 0, sizeof(*s));
    s->codec = codec;
    s->session_id = session_id;
    s->expected_core = expected_core;
    s->observed_core = -1;
    s->min_us = UINT32_MAX;
}
bool dual_end_perf_record(dual_end_perf_stats_t *s, int64_t elapsed_us,
    int core, bool success)
{
    return dual_end_perf_record_outcome(s, elapsed_us, core,
        success ? DUAL_END_OUTCOME_OK : DUAL_END_OUTCOME_OPERATION_FAILED);
}
bool dual_end_perf_record_outcome(dual_end_perf_stats_t *s, int64_t elapsed_us,
    int core, dual_end_owner_outcome_t outcome)
{
    if (!s) return false;
    if (elapsed_us < 0) {
        counter_add(s, &s->invalid_duration);
        return false;
    }
    if (s->samples == UINT32_MAX) {
        overflow_note(s);
        return false;
    }
    uint32_t duration;
    if ((uint64_t)elapsed_us > UINT32_MAX) {
        duration = UINT32_MAX;
        overflow_note(s);
    } else duration = (uint32_t)elapsed_us;
    ++s->samples;
    s->observed_core = core;
    if (core < 0 || core > 1) counter_add(s, &s->invalid_core);
    else {
        s->observed_core_mask |= UINT32_C(1) << (unsigned)core;
        if (core != s->expected_core) counter_add(s, &s->core_wrong);
    }
    if ((unsigned)outcome >= DUAL_END_OUTCOME_COUNT) outcome = DUAL_END_OUTCOME_UNCLASSIFIED;
    counter_add(s, &s->outcomes[outcome]);
    if (outcome != DUAL_END_OUTCOME_OK) counter_add(s, &s->failures);
    if (duration > DUAL_END_PERF_FRAME_BUDGET_US) counter_add(s, &s->over_budget);
    if (duration < s->min_us) s->min_us = duration;
    if (duration > s->max_us) s->max_us = duration;
    if (UINT64_MAX - s->sum_us < duration) {
        s->sum_us = UINT64_MAX;
        overflow_note(s);
    } else s->sum_us += duration;
    for (unsigned i = 0; i < DUAL_END_PERF_HISTOGRAM_BUCKETS; ++i) {
        if (duration <= dual_end_perf_histogram_upper_us[i]) {
            counter_add(s, &s->histogram[i]);
            break;
        }
    }
    return true;
}
bool dual_end_perf_summary(const dual_end_perf_stats_t *s,
    uint32_t *mean_us, uint32_t *p95_upper_us)
{
    if (mean_us) *mean_us = 0;
    if (p95_upper_us) *p95_upper_us = 0;
    if (!s || !mean_us || !p95_upper_us || !s->samples || s->failures >= s->samples || s->overflow) return false;
    const uint64_t mean = s->sum_us / s->samples;
    if (mean > UINT32_MAX) return false;
    const uint64_t rank = ((uint64_t)s->samples * 95U + 99U) / 100U;
    uint64_t count = 0;
    for (unsigned i = 0; i < DUAL_END_PERF_HISTOGRAM_BUCKETS; ++i) {
        count += s->histogram[i];
        if (count >= rank) {
            *mean_us = (uint32_t)mean;
            *p95_upper_us = dual_end_perf_histogram_upper_us[i];
            return true;
        }
    }
    return false;
}
