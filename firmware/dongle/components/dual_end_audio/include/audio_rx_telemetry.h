#pragma once

#include "audio_recovery_rx.h"
#include "esp_err.h"

/* Values are copied by the sole RX owner; the logger never reads RX state. */
typedef struct {
    audio_rx_stats_t stats;
    int64_t sampled_us;
    int64_t gap_deadline_us;
    int64_t hid_wait_peak_us;
    uint32_t session_id;
    /* RX owner session ordinal and the UAC generation observed while this
     * immutable owner snapshot was created. They make cross-cache joins
     * rejectable instead of silently mixing a new UAC session with old RX. */
    uint32_t session_generation;
    uint32_t uac_session_generation;
    uint8_t codec; /* RX owner codec, UINT8_MAX means not bound yet. */
    bool active;
    uint32_t delivered_frames, masked_frames, crc_error_frames, decode_error_frames;
    uint32_t invalid_frames, feedback_queue_failures, nack_queued, recovered_decoded_frames;
    uint32_t timed_sink_drop_frames; /* Decoded frames rejected by bounded timed UAC slots. */
    uint32_t concealed_repeat_frames; /* Missing frames filled by a faded repeat. */
    uint32_t hid_dispatches, hid_wait_max_us, decode_frame_max_us;
    uint32_t poll_max_us, poll_audio_max, hid_wait_peak_sequence, rx_stack_min_bytes;
} audio_rx_telemetry_snapshot_t;

/* The logger publishes this immutable copy after it has dequeued an RX-owner
 * snapshot. Diagnostic readers receive the cache only; they never inspect the
 * recovery owner or wait for a new audio frame. */
typedef struct {
    audio_rx_telemetry_snapshot_t snapshot;
    uint32_t sequence;
    uint32_t snapshots_dropped;
    uint32_t events_dropped;
    /* Written by the telemetry worker after it dequeues the immutable RX
     * owner copy. These are diagnostic-cache facts, never live RX reads. */
    uint64_t cache_published_us;
    uint32_t telemetry_stack_min_bytes;
} audio_rx_telemetry_cache_t;

typedef enum {
    AUDIO_RX_LOG_DECODERS_READY,
    AUDIO_RX_LOG_SESSION_BEGIN,
    AUDIO_RX_LOG_DECODER_OPEN_FAILED,
    AUDIO_RX_LOG_DECODER_NOT_READY,
    AUDIO_RX_LOG_DECODE_FAILED,
} audio_rx_telemetry_event_kind_t;

/* Start once from app initialization before registering the RX callbacks.
 * Allocation/task failure is logged and leaves telemetry disabled; it never
 * blocks audio startup. publish/event are called only by the RX task. No
 * formatting, allocation, waits, or application callbacks occur on producer
 * paths. */
esp_err_t audio_rx_telemetry_start(void);
bool audio_rx_telemetry_enabled(void);
bool audio_rx_telemetry_publish(const audio_rx_telemetry_snapshot_t *snapshot);
bool audio_rx_telemetry_get_latest(audio_rx_telemetry_cache_t *out);
void audio_rx_telemetry_event(audio_rx_telemetry_event_kind_t kind,
    uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e);
uint32_t audio_rx_telemetry_stack_min(void);

/* Formatting is separate for host schema tests; production calls only on logger. */
void audio_rx_telemetry_log_snapshot(const audio_rx_telemetry_snapshot_t *snapshot);
