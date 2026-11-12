#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "dual_end_telemetry_format.h"
#include "mic_pipeline_telemetry.h"

/*
 * Immutable diagnostics consumed by the optional Scepter CDC cache worker.
 *
 * `pipeline` and `spatial` are captured together by the microphone owner at
 * its existing five-second pipeline-window close.  The raw spatial sequence
 * remains its own producer sequence; `cache_sequence` is the only join key
 * for this immutable bundle.  A mismatched spatial identity is retained but
 * marked not-ready instead of being relabelled to match an older pipeline
 * window.  This is deliberately not an audio transport structure and no
 * capture/DSP/encoder path reads it back.
 */
typedef struct {
    bool available;
    uint8_t channels;
    bool noise_valid[2];
    uint64_t processed_hops[2];
    uint64_t queue_underflows[2];
    uint64_t queue_overflows[2];
} mic_wiener_diagnostics_t;

typedef struct {
    bool available, noise_valid;
    uint16_t fallback, speech_mask_q15;
    uint64_t processed_hops, valid_bins, fallback_bins, fallback_hops;
    uint64_t queue_underflows, queue_overflows;
} mic_mvdr_diagnostics_t;

typedef struct {
    bool ready;
    bool spatial_ready;
    uint32_t cache_sequence;
    mic_pipeline_telemetry_t pipeline;
    dual_end_spatial_telemetry_t spatial;
    /* Captured by the microphone owner at the same close boundary as spatial.
     * Per-channel counters are since the selected mode's generation reset and
     * remain unavailable outside applied Wiener modes 4 and 5. */
    mic_wiener_diagnostics_t wiener;
    /* Captured at the same immutable window boundary; populated only for
     * applied spatial mode 6. */
    mic_mvdr_diagnostics_t mvdr;
} mic_audio_diagnostics_snapshot_t;
