#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "dual_end_link_config.h"
#include "esp_err.h"
#include "dual_end_spatial.h"
#include "mic_diagnostics_snapshot.h"

typedef struct {
    uint32_t input_frames;
    uint32_t output_frames;
    uint32_t bytes_read;
    uint32_t left_raw_rms;
    uint32_t right_raw_rms;
    int32_t left_raw_peak;
    int32_t right_raw_peak;
    uint32_t left_pcm_rms;
    uint32_t right_pcm_rms;
    int32_t left_pcm_peak;
    int32_t right_pcm_peak;
    uint32_t soft_limited_frames;
    uint16_t minimum_gain_permille;
    /* Internal diagnostic snapshot, not an ESP-NOW/HID packet. Legacy
     * output_gain_x reports the left gain; the right gain can differ. */
    uint8_t output_gain_x;
    uint8_t right_output_gain_x;
    bool vad_enabled;
    bool vad_open;
    uint32_t packets_per_sec;
    uint32_t sample_values_per_sec;
    uint32_t analysis_frames;
    uint32_t analysis_average_us;
    uint32_t analysis_max_us;
    dual_end_audio_analysis_metadata_t latest_analysis;
    /* Device-processed UAC may be dual_mono, not two original mic signals. */
    dual_end_spatial_stats_t spatial;
    uint16_t spatial_requested_mode;
    uint32_t spatial_processing_average_us, spatial_processing_max_us;
} mic_audio_stats_t;

typedef void (*mic_audio_frame_cb_t)(
    const int16_t *samples,
    size_t sample_count,
    const dual_end_audio_analysis_metadata_t *analysis,
    void *user_ctx);
typedef void (*mic_audio_stats_cb_t)(const mic_audio_stats_t *stats, void *user_ctx);
/* Stats callback executes on optional low-priority Core0 telemetry, not the
 * real-time Core1 audio owner. Consumers must not mutate DSP/encoder state.
 * Callback snapshots are interval summaries; absent telemetry is unknown. */

esp_err_t mic_audio_init(void);
esp_err_t mic_audio_load_output_gain(void);
esp_err_t mic_audio_set_output_gain(uint8_t gain_x, bool persist);
uint8_t mic_audio_get_output_gain(void);
void mic_audio_set_frame_callback(mic_audio_frame_cb_t callback, void *user_ctx);
void mic_audio_set_stats_callback(mic_audio_stats_cb_t callback, void *user_ctx);
esp_err_t mic_audio_start(void);
esp_err_t mic_audio_set_enabled(bool enabled);
void mic_audio_set_motor_active(bool active);

/* Items 0..14 are DSP v1. Codec 15: 3=Opus/4=G722, session-only, applied
 * by the bounded control worker; the 20 ms audio owner only commits local DSP
 * state at a frame boundary. Capabilities 16: bit0 G722/bit1 Opus, read-only v2.
 * DSP v3:17 capabilities;18 requested mode RW;19 applied mode;20 layout;
 * 21 current fallback;22 applied spatial generation. Mode 6 is soft-mask MVDR
 * and reports dual-mono layout; default mode 0 preserves stereo. */
esp_err_t mic_audio_parameter(bool write, uint8_t item, uint16_t value, uint16_t *actual);

/* io0..7: depth, highwater, DMA overflow, overload stops, diag drops, deadlines, IO/processing stack bytes. */
bool mic_audio_get_runtime_io(uint32_t values[8]);

/*
 * Copies a completed, immutable pipeline window for the optional CDC
 * diagnostics cache.  It never consumes the logger report and never changes
 * capture, DSP, encoder, radio, or spatial state.  The caller must treat a
 * false spatial_ready flag as an identity mismatch rather than a measured
 * zero.  This accessor is intended for a low-priority task, not a callback.
 */
bool mic_audio_copy_diagnostics_snapshot(mic_audio_diagnostics_snapshot_t *snapshot);
