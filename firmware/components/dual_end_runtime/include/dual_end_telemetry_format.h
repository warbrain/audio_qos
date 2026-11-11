#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Project telemetry over the existing debug/OTA log route; not an OTel SDK.
 * Decimal strings preserve uint64 counters for JavaScript consumers. Every
 * part carries boot/role/session/seq, so incomplete or mixed groups are unknown.
 * All formatting is low-priority work, never invoked by the audio producer. */
#define DUAL_END_SPATIAL_TELEMETRY_PARTS 15U
#define DUAL_END_RX_TELEMETRY_PARTS 6U
#define DUAL_END_TELEMETRY_JSON_BYTES 181U /* <=180 payload plus terminating NUL */
typedef struct {
    uint32_t boot_id, session_id, sequence;
    uint8_t role, codec;
    bool codec_known;
    uint64_t sampled_us;
    uint16_t generation, requested, applied, layout, fallback;
    bool polarity_inverted;
    float delay_samples, correlation, current_mix;
    uint64_t frames, learned_frames, frozen_frames, mixed_frames;
    uint64_t nlms_fallback_frames, mix_q16_sum, processing_us;
    uint32_t processing_frames, processing_max_us, bounded_updates, resets;
} dual_end_spatial_telemetry_t;
typedef struct {
    uint32_t boot_id, session_id, sequence;
    uint8_t role, codec;
    bool codec_known, active;
    uint64_t sampled_us, snapshot_age_us;
    uint32_t snapshot_dropped, event_dropped;
    uint32_t rx_stack_min_bytes, telemetry_stack_min_bytes;
} dual_end_rx_telemetry_t;
size_t dual_end_spatial_telemetry_format(const dual_end_spatial_telemetry_t *snapshot,
    unsigned part, char *buffer, size_t capacity);
size_t dual_end_rx_telemetry_format(const dual_end_rx_telemetry_t *snapshot,
    unsigned part, char *buffer, size_t capacity);
