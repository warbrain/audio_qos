#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mic_observability_telemetry.h"

/* A bounded, optional diagnostic record, not an audio transport packet.
 * q: enqueue -> owner dequeue; a: raw block completed -> owner dequeue.
 * d excludes spatial and encoder callback; s is spatial only; w includes
 * local DSP, spatial and the bounded queue copy, excluding asynchronous
 * encoding/radio work. Nested d/s timings must NEVER be added to w again.
 * Closed windows do not overlap and end on session/config/capture changes.
 * U64 totals/timestamps use decimal strings; zero observations have null
 * extrema. Every part repeats c(codec), m(mode), g(spatial generation) and
 * boot/role/session/seq. cg is capture generation in part 3. Missing parts or
 * sequence gaps mean unknown, never a measured zero or a packet loss.
 * Dedicated audio.capture_health/audio.owner_timing/audio.nlms events repeat
 * the same boot/role/session/seq/c/m tuple but omit g to stay within the
 * logger line bound; join them to audio.pipeline for g and cg. */
#define MIC_PIPELINE_TELEMETRY_PARTS 32U
#define MIC_PIPELINE_TELEMETRY_BYTES 181U
#define MIC_PIPELINE_SPATIAL_MAX_MODE 5U
#define MIC_PIPELINE_WINDOW_US UINT64_C(5000000)
#define MIC_DIAGNOSTIC_INTERVAL_US UINT64_C(200000)
#define MIC_DIAGNOSTIC_BUDGET_US 40000U
#define MIC_DIAGNOSTIC_INITIAL_COST_US 8000U
#define MIC_DIAGNOSTIC_MAX_AGE_US UINT64_C(400000)
#define MIC_WIENER_TIMING_BUCKET_US 50U
#define MIC_WIENER_TIMING_MAX_US 10000U
#define MIC_WIENER_TIMING_BUCKET_COUNT \
    (MIC_WIENER_TIMING_MAX_US / MIC_WIENER_TIMING_BUCKET_US + 1U)

typedef struct { uint32_t n, lo, hi; uint64_t us; } mic_pipeline_metric_t;
static inline void mic_pipeline_metric_add(mic_pipeline_metric_t *m, uint32_t us)
{
    if (!m->n || us < m->lo) m->lo = us;
    if (us > m->hi) m->hi = us;
    if (m->n != UINT32_MAX) ++m->n;
    m->us = UINT64_MAX - m->us < us ? UINT64_MAX : m->us + us;
}

/* Fixed-size histogram for one five-second owner window. Buckets are 50 us
 * wide through 10 ms; the final bucket means >=10 ms. Quantiles are reported
 * as conservative bucket upper bounds, while max_us retains the exact max. */
typedef struct {
    uint32_t buckets[MIC_WIENER_TIMING_BUCKET_COUNT];
    uint32_t count, max_us;
} mic_wiener_timing_histogram_t;
typedef struct {
    uint32_t count, p50_us, p95_us, max_us;
    bool available, p50_overflow, p95_overflow;
} mic_wiener_timing_summary_t;
static inline void mic_wiener_timing_histogram_add(
    mic_wiener_timing_histogram_t *histogram, uint32_t elapsed_us)
{
    if (!histogram) return;
    const uint32_t overflow_bucket = MIC_WIENER_TIMING_BUCKET_COUNT - 1U;
    uint32_t bucket = elapsed_us / MIC_WIENER_TIMING_BUCKET_US;
    if (bucket >= overflow_bucket) bucket = overflow_bucket;
    if (histogram->buckets[bucket] != UINT32_MAX) ++histogram->buckets[bucket];
    if (histogram->count != UINT32_MAX) ++histogram->count;
    if (elapsed_us > histogram->max_us) histogram->max_us = elapsed_us;
}
static inline uint32_t mic_wiener_timing_histogram_percentile(
    const mic_wiener_timing_histogram_t *histogram, unsigned percentile,
    bool *overflow)
{
    if (overflow) *overflow = false;
    if (!histogram || !histogram->count || percentile == 0U || percentile > 100U)
        return 0U;
    const uint64_t rank = ((uint64_t)histogram->count * percentile + 99U) / 100U;
    uint64_t seen = 0;
    const uint32_t overflow_bucket = MIC_WIENER_TIMING_BUCKET_COUNT - 1U;
    for (uint32_t bucket = 0; bucket < MIC_WIENER_TIMING_BUCKET_COUNT; ++bucket) {
        seen += histogram->buckets[bucket];
        if (seen < rank) continue;
        if (bucket == overflow_bucket) {
            if (overflow) *overflow = true;
            return MIC_WIENER_TIMING_MAX_US;
        }
        return (bucket + 1U) * MIC_WIENER_TIMING_BUCKET_US;
    }
    return 0U;
}
static inline mic_wiener_timing_summary_t mic_wiener_timing_histogram_summary(
    const mic_wiener_timing_histogram_t *histogram)
{
    mic_wiener_timing_summary_t summary = {0};
    if (!histogram || !histogram->count) return summary;
    summary.available = true;
    summary.count = histogram->count;
    summary.max_us = histogram->max_us;
    summary.p50_us = mic_wiener_timing_histogram_percentile(
        histogram, 50U, &summary.p50_overflow);
    summary.p95_us = mic_wiener_timing_histogram_percentile(
        histogram, 95U, &summary.p95_overflow);
    return summary;
}

struct mic_pipeline_telemetry {
    uint32_t boot_id, session_id, sequence, capture_generation, window;
    uint8_t codec, mode;
    bool codec_known;
    /* Fixed output gain applied to the frames represented by this window.
     * Zero means unavailable/invalid and is never presented as a measured 0x. */
    uint8_t output_gain_l_x, output_gain_r_x;
    uint16_t generation;
    uint64_t sampled_us, start_us;
    mic_pipeline_metric_t q, a, d, s, w;
    mic_wiener_timing_summary_t wiener_timing[2];
    /* Additional windowed observability is emitted through dedicated events.
     * Keep audio.pipeline schema 1 and its 32-part contract unchanged. */
    mic_capture_health_t capture_health;
    mic_pipeline_metric_t owner_mutex_wait, owner_encode, owner_radio_submit;
    /* Worker queue, control-lock and complete-raw-block-to-callback ages.
     * These are separate, nested observations, not additive CPU work. */
    mic_pipeline_metric_t dispatch_queue_wait, dispatch_control_wait, dispatch_frame_age;
    /* Boot cumulative. Missing completion evidence remains unknown. */
    uint32_t dispatch_queue_full, dispatch_invalidated, dispatch_unavailable;
    uint32_t dispatch_event_lost, dispatch_queue_highwater, dispatch_late_events;
    /* Boot-cumulative reasons contributing to dispatch_late_events. The three
     * attribution reasons are mutually exclusive; epoch is reported separately. */
    uint32_t dispatch_late_epoch, dispatch_late_attribution;
    uint32_t dispatch_late_inactive, dispatch_late_identity, dispatch_late_before_window;
    mic_nlms_telemetry_t nlms;
    /* Diagnostic/global IO counters are boot cumulative, captured at window
     * close. They are NOT per-window deltas. FFT completion is independent. */
    uint32_t diag_q, diag_b, diag_o, diag_s, fft_hi;
    uint32_t win_lost, late, raw_seq, q_hi_all;
    uint64_t fft_us, fft_n, fft_u;
};

typedef struct {
    uint64_t window_started_us, last_started_us;
    uint32_t used_us, predicted_us;
    bool started;
} mic_diagnostic_budget_t;
/* Admission limits STARTS, not the time of an indivisible FFT. One unusually
 * long FFT may exceed the configured budget; its full cost is then charged,
 * so no further FFT starts in that second. No audio owner waits on this. */
static inline bool mic_diagnostic_budget_admit(mic_diagnostic_budget_t *b, uint64_t now_us)
{
    if (b->started && now_us < b->last_started_us) return false;
    if (!b->started || now_us - b->window_started_us >= UINT64_C(1000000)) {
        b->window_started_us = now_us; b->used_us = 0;
    }
    const uint32_t predicted = b->predicted_us ? b->predicted_us : MIC_DIAGNOSTIC_INITIAL_COST_US;
    if (b->started && now_us - b->last_started_us < MIC_DIAGNOSTIC_INTERVAL_US) return false;
    if (b->used_us >= MIC_DIAGNOSTIC_BUDGET_US ||
        predicted > MIC_DIAGNOSTIC_BUDGET_US - b->used_us) return false;
    b->last_started_us = now_us; b->started = true; return true;
}
static inline void mic_diagnostic_budget_charge(mic_diagnostic_budget_t *b, uint32_t cost_us)
{
    b->used_us = UINT32_MAX - b->used_us < cost_us ? UINT32_MAX : b->used_us + cost_us;
    /* Capped predictor permits a fresh window to try once after an outlier;
     * the actual outlier cost is never capped in the charged counter. */
    b->predicted_us = cost_us > MIC_DIAGNOSTIC_BUDGET_US ? MIC_DIAGNOSTIC_BUDGET_US : cost_us;
}
size_t mic_pipeline_telemetry_format(const mic_pipeline_telemetry_t *record,
    unsigned part, char *buffer, size_t capacity);
