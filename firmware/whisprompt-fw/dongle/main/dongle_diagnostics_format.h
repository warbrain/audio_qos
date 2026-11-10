#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* `DONGLE_TELEMETRY` is deliberately a compact multipart cache response so it
 * fits one CDC response record without fragmentation or heap allocation. */
#define DONGLE_DIAGNOSTICS_PARTS 13U
#define DONGLE_DIAGNOSTICS_SCHEMA 1U
/* `DONGLE_TELEMETRY_V2` leaves the v1 wire contract untouched and appends one
 * strict RX-cache evidence part. v2 is an opt-in command. */
#define DONGLE_DIAGNOSTICS_V2_PARTS 14U
#define DONGLE_DIAGNOSTICS_V2_SCHEMA 2U
#define DONGLE_DIAGNOSTICS_PERF_STAGES 4U
#define DONGLE_CDC_LOG_HEALTH_PARTS 6U
#define DONGLE_DIAGNOSTICS_JSON_MAX_BYTES 340U
#define DONGLE_DIAGNOSTICS_JSON_BUFFER_BYTES \
    (DONGLE_DIAGNOSTICS_JSON_MAX_BYTES + 1U)

typedef struct {
    bool runtime_ready;
    bool resource_cache_ready;
    bool rx_cache_ready;
    bool context_set;
    uint8_t role;
    uint8_t codec; /* UINT8_MAX means no admitted decoder has bound yet. */
    uint32_t boot_id;
    uint32_t cache_sequence;
    uint32_t run_id;
    uint32_t arm_id;
    uint32_t context_sequence;
    /* v1 keeps its legacy immediate-label context. v2 has a separate context
     * that becomes queryable only after a post-context RX owner snapshot.
     * HMI protocol note: in a v2 response `co=1` means this strict gate is
     * armed, not that run/arm IDs are nonzero. Thus `DIAG_CONTEXT <t> 0 0`
     * clears v1's legacy label (`co=0` there) but deliberately leaves v2
     * pending until one new immutable RX record is published. */
    uint32_t v2_run_id;
    uint32_t v2_arm_id;
    uint32_t v2_context_sequence;
    bool v2_context_set;
    bool v2_context_ready;
    /* Strict-v2 identity is copied from immutable RX/UAC/runtime caches. It
     * is intentionally internal to the query gate so the v1 wire schema and
     * its 13 response parts remain unchanged. */
    bool v2_identity_ready;
    bool v2_identity_mixed;
    uint32_t v2_rx_session_id;
    uint32_t v2_rx_session_generation;
    uint32_t v2_uac_session_generation;
    uint32_t v2_rx_cache_sequence;
    uint32_t session_id;
    uint64_t sampled_us;
    uint32_t resource_sequence;
    uint64_t resource_sampled_us;
    uint8_t perf_ready_mask;
    uint32_t perf_window_sequence[DONGLE_DIAGNOSTICS_PERF_STAGES];

    bool rx_active;
    uint32_t rx_accepted;
    uint32_t rx_output_frames;
    uint32_t rx_permanent_loss;
    uint32_t rx_gap_events;
    uint32_t rx_recovered;
    uint32_t rx_duplicates;
    uint32_t rx_late;
    uint32_t rx_rejected;
    uint32_t rx_abort_count;
    uint32_t rx_nack_requests;
    uint32_t rx_nack_accepted;
    uint32_t rx_nack_cancelled;
    uint32_t rx_nack_expired;
    uint32_t rx_nack_rejected;
    uint32_t rx_resync_requests;
    uint32_t rx_delivered_decoded;
    uint32_t rx_masked_frames;
    uint32_t rx_recovered_decoded;
    uint32_t rx_crc_failures;
    uint32_t rx_decode_failures;
    uint32_t rx_invalid_frames;
    uint32_t rx_feedback_queue_failures;
    uint32_t rx_nack_queued;
    uint32_t rx_poll_max_us;
    uint32_t rx_decode_max_us;
    uint32_t rx_poll_audio_max;

    bool uac_rx_active;
    bool uac_start_pending;
    uint32_t uac_frames_in;
    uint32_t uac_sequence_lost;
    uint32_t uac_sequence_reordered;
    uint32_t uac_timeouts;
    uint32_t uac_silence_fills;
    uint32_t uac_buffer_samples;
    uint32_t uac_overflow_drops;
    uint32_t uac_level_drop_samples;
    uint32_t uac_level_dup_samples;
    bool uac_timeline_enabled;
    bool uac_timeline_started;
    uint64_t uac_timeline_cursor_samples;
    uint64_t uac_timeline_late_samples;
    uint64_t uac_timeline_local_retired_samples;
    uint64_t uac_timeline_resume_skipped_samples;
    uint32_t uac_timeline_pause_count;
    uint32_t uac_timeline_max_gap_us;
    uint64_t uac_timeline_start_us;
    uint64_t uac_timeline_first_output_us;

    bool link_connected;
    bool channel_pending;
    uint8_t channel;
    uint8_t channel_state;
    uint32_t rx_queue_drops;
    uint32_t rx_invalid_lengths;
    uint32_t tx_accepted;
    uint32_t tx_callbacks;
    uint32_t tx_immediate_failures;
    uint32_t tx_soft_timeouts;
    uint32_t tx_late_callbacks;
    uint32_t tx_hard_recoveries;
    uint32_t tx_recovery_failures;
    uint32_t rx_stack_min_bytes;
    uint32_t tx_stack_min_bytes;
    uint32_t link_stack_min_bytes;
    uint32_t rx_cache_sequence;
    uint32_t rx_snapshot_drops;
    uint32_t rx_event_drops;
    /* Timing is valid only when rx_owner_timing_ready is true. `age` is the
     * diagnostics cache sample time minus the immutable RX-owner timestamp;
     * it is not an estimate at CDC query time. All 64-bit timestamps are
     * formatted as decimal JSON strings in the v2 evidence part. */
    bool rx_owner_timing_ready;
    uint64_t rx_owner_sampled_us;
    uint64_t rx_owner_age_us;
    uint64_t rx_cache_published_us;
    uint32_t rx_owner_stack_min_bytes;
    uint32_t rx_telemetry_stack_min_bytes;
    uint32_t diagnostics_stack_min_bytes;
    /* Cumulative USB CDC control-path counters, sampled from the device task.
     * Queued TX bytes mean TinyUSB accepted bytes, not that the host received them. */
    bool cdc_connected;
    bool cdc_dtr;
    uint32_t cdc_rx_commands;
    uint32_t cdc_rx_queue_overflows;
    uint32_t cdc_response_drops;
    uint64_t cdc_response_tx_queued_bytes;
    uint32_t cdc_response_tx_incomplete;
    /* Independent mirror evidence, not appended to legacy DIAG wire parts.
     * truncated counts rejected oversize records; write_failed means queued
     * data could not be fully submitted, not host delivery acknowledgment. */
    uint32_t cdc_log_attempted,cdc_log_enqueued,cdc_log_dropped;
    uint32_t cdc_log_truncated,cdc_log_write_failed;
} dongle_diagnostics_snapshot_t;

/* Writes one complete JSON document or returns zero after clearing buffer[0].
 * Callers must never transmit a partial document. */
size_t dongle_diagnostics_format_part(const dongle_diagnostics_snapshot_t *snapshot,
                                      unsigned part,
                                      char *buffer,
                                      size_t capacity);

/* v2 preserves parts 0..12 semantically, adds cumulative CDC status to part 1,
 * sets v=2/n=14, and adds part 13 for RX-owner cache timing and stack evidence. */
size_t dongle_diagnostics_format_v2_part(const dongle_diagnostics_snapshot_t *snapshot,
                                         unsigned part,
                                         char *buffer,
                                         size_t capacity);

/* Independent bounded event; legacy v1/v2 part counts remain unchanged. */
size_t dongle_diagnostics_cdc_log_format_part(const dongle_diagnostics_snapshot_t *snapshot,
                                              unsigned part, char *buffer, size_t capacity);
