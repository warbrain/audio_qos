#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* These records are copied from the microphone owner into a five-second
 * telemetry window. They are diagnostics only: no capture, codec, or DSP
 * control path reads them back. Capture-health includes a separate part for
 * the applied L/R fixed-gain setting; level samples remain pre-gain. */
#define MIC_OBSERVABILITY_TELEMETRY_BYTES 181U
#define MIC_CAPTURE_HEALTH_TELEMETRY_PARTS 18U
#define MIC_OWNER_TIMING_TELEMETRY_PARTS 17U
#define MIC_DISPATCH_TELEMETRY_PARTS 17U
#define MIC_OUTPUT_HEALTH_TELEMETRY_PARTS 7U
#define MIC_NLMS_TELEMETRY_PARTS 23U
/* Keep the legacy audio.nlms v1 23-part event unchanged. The separate
 * presence event splits explicit fields into two <=190-byte JSON parts so
 * a 64-byte ESP prefix plus LF and NUL still fits the 256-byte transport. */
#define MIC_NLMS_PRESENCE_TELEMETRY_PARTS 2U
#define MIC_NLMS_PRESENCE_TELEMETRY_BYTES 191U
#define MIC_CAPTURE_HEALTH_CHANNELS 2U
#define MIC_CAPTURE_HEALTH_QUIET_ABS 8U
#define MIC_CAPTURE_HEALTH_CLIP_ABS 32700U
#define MIC_CAPTURE_HEALTH_UNDER_RMS 128U
#define MIC_NLMS_REASON_COUNT 8U

/* Output is after fixed gain/filtering, spatial processing and the optional
 * gate. Samples count stereo frames per channel; limited_frames counts the
 * fixed-gain limiter's stereo sample instants, not encoded 20 ms packets. */
typedef struct {
    uint64_t samples, sum_sq[MIC_CAPTURE_HEALTH_CHANNELS], limited_frames;
    uint32_t peak[MIC_CAPTURE_HEALTH_CHANNELS];
} mic_output_health_t;

typedef struct {
    uint64_t samples[MIC_CAPTURE_HEALTH_CHANNELS];
    uint64_t sum_sq[MIC_CAPTURE_HEALTH_CHANNELS];
    uint64_t zero_samples[MIC_CAPTURE_HEALTH_CHANNELS];
    uint64_t quiet_samples[MIC_CAPTURE_HEALTH_CHANNELS];
    uint64_t clip_samples[MIC_CAPTURE_HEALTH_CHANNELS];
    uint32_t peak[MIC_CAPTURE_HEALTH_CHANNELS];
    uint32_t underlevel_blocks[MIC_CAPTURE_HEALTH_CHANNELS];
    uint32_t blocks;
    mic_output_health_t output;
} mic_capture_health_t;

/* A single 20 ms NLMS observation.  It is deliberately derived beside the
 * microphone owner rather than written into the spatial matcher state, so the
 * audio algorithm and its state layout remain unchanged. */
typedef struct {
    uint16_t reason;
    uint32_t updated_samples;
    uint32_t frozen_samples;
    uint32_t bounded_updates; /* Per-completed-block delta. */
    uint32_t resets;          /* Per-completed-block delta. */
    uint64_t convergence_frame;
    uint32_t update_streak;
    bool valid;
    bool converged;
    bool convergence_event;
    bool has_convergence_frame;
} mic_nlms_block_t;

typedef struct {
    uint64_t blocks;
    uint64_t updated_blocks;
    uint64_t frozen_blocks;
    uint64_t updated_samples;
    uint64_t frozen_samples;
    uint64_t converged_blocks;
    uint64_t first_converged_frame;
    uint64_t reasons[MIC_NLMS_REASON_COUNT];
    uint32_t convergence_events;
    uint32_t update_streak;
    uint32_t max_update_streak;
    uint32_t bounded_updates; /* Five-second window total. */
    uint32_t resets;          /* Five-second window total. */
    uint16_t last_reason;
    bool converged;
    /* Internal presence bit keeps a valid first frame distinct from the
     * zero-initialized no-convergence state. The published schema stays put. */
    bool has_first_converged_frame;
} mic_nlms_telemetry_t;

static inline uint64_t mic_observability_u64_add(uint64_t value, uint64_t add)
{
    return UINT64_MAX - value < add ? UINT64_MAX : value + add;
}

static inline uint32_t mic_observability_u32_add(uint32_t value, uint32_t add)
{
    return UINT32_MAX - value < add ? UINT32_MAX : value + add;
}

static inline void mic_capture_health_record(mic_capture_health_t *health,
                                               int16_t left, int16_t right)
{
    if (!health) return;
    const int16_t samples[MIC_CAPTURE_HEALTH_CHANNELS] = { left, right };
    for (unsigned channel = 0; channel < MIC_CAPTURE_HEALTH_CHANNELS; ++channel) {
        const int32_t sample = samples[channel];
        const uint32_t magnitude = (uint32_t)(sample < 0 ? -sample : sample);
        health->samples[channel] = mic_observability_u64_add(health->samples[channel], 1U);
        health->sum_sq[channel] = mic_observability_u64_add(
            health->sum_sq[channel], (uint64_t)(sample * sample));
        if (sample == 0) health->zero_samples[channel] = mic_observability_u64_add(
            health->zero_samples[channel], 1U);
        if (magnitude <= MIC_CAPTURE_HEALTH_QUIET_ABS)
            health->quiet_samples[channel] = mic_observability_u64_add(
                health->quiet_samples[channel], 1U);
        if (magnitude >= MIC_CAPTURE_HEALTH_CLIP_ABS)
            health->clip_samples[channel] = mic_observability_u64_add(
                health->clip_samples[channel], 1U);
        if (magnitude > health->peak[channel]) health->peak[channel] = magnitude;
    }
}

static inline void mic_capture_health_complete_block(mic_capture_health_t *health)
{
    if (!health) return;
    health->blocks = mic_observability_u32_add(health->blocks, 1U);
    const uint64_t under_square = (uint64_t)MIC_CAPTURE_HEALTH_UNDER_RMS *
                                  (uint64_t)MIC_CAPTURE_HEALTH_UNDER_RMS;
    for (unsigned channel = 0; channel < MIC_CAPTURE_HEALTH_CHANNELS; ++channel) {
        if (health->samples[channel] &&
            health->sum_sq[channel] / health->samples[channel] < under_square)
            health->underlevel_blocks[channel] = mic_observability_u32_add(
                health->underlevel_blocks[channel], 1U);
    }
}

static inline void mic_capture_health_merge(mic_capture_health_t *total,
                                             const mic_capture_health_t *block)
{
    if (!total || !block) return;
    total->blocks = mic_observability_u32_add(total->blocks, block->blocks);
    total->output.samples = mic_observability_u64_add(total->output.samples, block->output.samples);
    total->output.limited_frames = mic_observability_u64_add(
        total->output.limited_frames, block->output.limited_frames);
    for (unsigned channel = 0; channel < MIC_CAPTURE_HEALTH_CHANNELS; ++channel) {
        total->output.sum_sq[channel] = mic_observability_u64_add(
            total->output.sum_sq[channel], block->output.sum_sq[channel]);
        if (block->output.peak[channel] > total->output.peak[channel])
            total->output.peak[channel] = block->output.peak[channel];
    }
    for (unsigned channel = 0; channel < MIC_CAPTURE_HEALTH_CHANNELS; ++channel) {
        total->samples[channel] = mic_observability_u64_add(total->samples[channel], block->samples[channel]);
        total->sum_sq[channel] = mic_observability_u64_add(total->sum_sq[channel], block->sum_sq[channel]);
        total->zero_samples[channel] = mic_observability_u64_add(total->zero_samples[channel], block->zero_samples[channel]);
        total->quiet_samples[channel] = mic_observability_u64_add(total->quiet_samples[channel], block->quiet_samples[channel]);
        total->clip_samples[channel] = mic_observability_u64_add(total->clip_samples[channel], block->clip_samples[channel]);
        total->underlevel_blocks[channel] = mic_observability_u32_add(
            total->underlevel_blocks[channel], block->underlevel_blocks[channel]);
        if (block->peak[channel] > total->peak[channel]) total->peak[channel] = block->peak[channel];
    }
}

static inline void mic_nlms_telemetry_add(mic_nlms_telemetry_t *total,
                                           const mic_nlms_block_t *block)
{
    if (!total || !block || !block->valid) return;
    total->blocks = mic_observability_u64_add(total->blocks, 1U);
    if (block->updated_samples) total->updated_blocks = mic_observability_u64_add(total->updated_blocks, 1U);
    else total->frozen_blocks = mic_observability_u64_add(total->frozen_blocks, 1U);
    total->updated_samples = mic_observability_u64_add(total->updated_samples, block->updated_samples);
    total->frozen_samples = mic_observability_u64_add(total->frozen_samples, block->frozen_samples);
    if (block->converged) total->converged_blocks = mic_observability_u64_add(total->converged_blocks, 1U);
    if (block->convergence_event) {
        total->convergence_events = mic_observability_u32_add(total->convergence_events, 1U);
        if (block->has_convergence_frame && !total->has_first_converged_frame) {
            total->first_converged_frame = block->convergence_frame;
            total->has_first_converged_frame = true;
        }
    }
    if (block->reason < MIC_NLMS_REASON_COUNT)
        total->reasons[block->reason] = mic_observability_u64_add(total->reasons[block->reason], 1U);
    total->last_reason = block->reason;
    total->converged = block->converged;
    total->update_streak = block->update_streak;
    if (block->update_streak > total->max_update_streak)
        total->max_update_streak = block->update_streak;
    total->bounded_updates = mic_observability_u32_add(total->bounded_updates,
                                                        block->bounded_updates);
    total->resets = mic_observability_u32_add(total->resets, block->resets);
}

typedef struct mic_pipeline_telemetry mic_pipeline_telemetry_t;

size_t mic_capture_health_telemetry_format(const mic_pipeline_telemetry_t *record,
                                            unsigned part, char *buffer, size_t capacity);
size_t mic_owner_timing_telemetry_format(const mic_pipeline_telemetry_t *record,
                                          unsigned part, char *buffer, size_t capacity);
size_t mic_nlms_telemetry_format(const mic_pipeline_telemetry_t *record,
                                  unsigned part, char *buffer, size_t capacity);
/* Query-only variant.  It preserves the audio.nlms v1 JSON schema but may
 * encode a zero-block window when NLMS was intentionally inactive.  The CDC
 * response separately labels that state active=0; legacy console logging
 * continues to use mic_nlms_telemetry_format and emits only active NLMS. */
size_t mic_nlms_telemetry_cache_format(const mic_pipeline_telemetry_t *record,
                                        unsigned part, char *buffer, size_t capacity);
/* This deliberately does not change audio.nlms v1.  It distinguishes the
 * zero-initialized no-convergence state from a valid first frame at zero. */
size_t mic_nlms_presence_telemetry_format(const mic_pipeline_telemetry_t *record,
                                           unsigned part, char *buffer, size_t capacity);

/* Additive records preserve all existing v1 event part counts. */
size_t mic_dispatch_telemetry_format(const mic_pipeline_telemetry_t *record,
                                    unsigned part, char *buffer, size_t capacity);
size_t mic_output_health_telemetry_format(const mic_pipeline_telemetry_t *record,
                                         unsigned part, char *buffer, size_t capacity);
