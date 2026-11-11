#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool rx_active;
    bool start_pending;
    uint32_t frames_in;
    uint32_t sequence_lost;
    uint32_t sequence_reordered;
    uint32_t timeouts;
    uint32_t silence_fills;
    uint32_t level_drop_samples;
    uint32_t level_dup_samples;
    uint32_t overflow_drop_samples;
    uint32_t concealed_frames;
    /* Interleaved sample values, preserving the existing diagnostics:
     * buffer_ms = values * 1000 / (sample_rate * channels). */
    uint32_t buffer_samples;
    uint32_t session_generation;
    bool timeline_enabled;
    bool timeline_started;
    int64_t timeline_start_us;
    int64_t timeline_first_output_us;
    uint64_t timeline_cursor_samples;
    uint64_t timeline_late_samples;
    uint64_t timeline_local_retired_samples; /* PCM past deadline while USB was not consuming. */
    uint64_t timeline_resume_skipped_samples; /* Source positions skipped when USB starts/resumes. */
    uint32_t timeline_pause_count;
    uint32_t timeline_max_gap_us;
} dongle_uac_bridge_stats_t;

esp_err_t dongle_uac_bridge_start(void);
void dongle_uac_bridge_reset_session(void);
void dongle_uac_bridge_end_session(void);
bool dongle_uac_bridge_session_drained(void);
/* Compatibility-only legacy sink. Interleaved L=denoised, R=raw; the
 * standard dual_end_audio legacy callback counts interleaved int16_t values.
 * For existing direct bridge callers, one 320-sample L/R-pair frame remains
 * accepted too. Production Dongle builds compile the legacy ring out and
 * return ESP_ERR_NOT_SUPPORTED from this entry point; use the fixed-timeline
 * sink below. */
esp_err_t dongle_uac_bridge_write_pcm(const int16_t *pcm,
                                       size_t samples,
                                       uint32_t sequence,
                                       int rssi,
                                       void *ctx);

/* Fixed-timeline path. playout_us is the first sample's absolute USB time,
 * supplied by the same recovery clock that decides repair expiry. */
esp_err_t dongle_uac_bridge_write_timed_pcm(const int16_t *pcm,
    size_t samples, uint32_t sequence, int rssi, int64_t playout_us, void *ctx);
bool dongle_uac_bridge_playout_clock(uint32_t *sequence, int64_t *playout_us,
                                    void *ctx);
size_t dongle_uac_bridge_read_pcm(uint8_t *buf, size_t len);
void dongle_uac_bridge_get_stats(dongle_uac_bridge_stats_t *out_stats);
/* Small coherent identity read for the RX telemetry owner. It does not scan
 * PCM/timeline state or allocate. */
uint32_t dongle_uac_bridge_session_generation(void);
/* Compile-time playout ring capacity in milliseconds, for field telemetry.
 * It never scans timeline state and allocates nothing. */
uint32_t dongle_uac_bridge_timeline_capacity_ms(void);

#ifdef __cplusplus
}
#endif
