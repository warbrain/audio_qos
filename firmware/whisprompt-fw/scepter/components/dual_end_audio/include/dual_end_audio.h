#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "dual_end_link_config.h"
#include "dual_end_perf_stats.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*dual_end_audio_pcm_sink_t)(const int16_t *pcm,
                                              size_t samples,
                                              uint32_t sequence,
                                              int rssi,
                                              void *ctx);

typedef void (*dual_end_input_event_sink_t)(const dual_end_hid_input_payload_t *payload,
                                           uint32_t sequence,
                                           int rssi,
                                           void *ctx);

esp_err_t dual_end_audio_sender_start(void);
/* Runtime switch is owned by the mic audio task at a complete PCM boundary. */
esp_err_t dual_end_audio_sender_set_codec(uint8_t codec);
uint8_t dual_end_audio_sender_get_codec(void);
/* Owner-only layout transition: new recovery stream discards old queued PCM. */
esp_err_t dual_end_audio_sender_restart_processing(void);
/* Applied microphone-owner layout: 0 stereo, 1 dual mono. Every call starts a
 * new recovery session, including DAS <-> NLMS, and resets both codec states.
 * Commit/ACK only on ESP_OK; failure leaves the previous layout applied. */
esp_err_t dual_end_audio_sender_set_processing_layout(uint8_t layout);
typedef struct {
    uint32_t session_id;
    uint8_t layout;
    uint64_t frames, process_calls, reused_frames, failures, failed_process_calls;
    uint64_t layout_mismatches;
} dual_end_audio_encode_stats_t;
/* Low-rate diagnostic snapshot. Do not add a per-frame sender-lock acquisition. */
void dual_end_audio_sender_get_encode_stats(dual_end_audio_encode_stats_t *stats);
/* Owner-only one-shot copy made immediately after the PCM callback returns.
 * It reports one fresh sender admission without taking the sender mutex:
 * mw=mutex wait, enc=codec work, rs=radio queue submit. A later copy without
 * another sender invocation returns false. It is diagnostic only; `outcome`
 * is populated by the consuming handoff API. */
typedef struct {
    uint32_t session_id;
    uint32_t mutex_wait_us;
    uint32_t encode_us;
    uint32_t radio_submit_us;
    uint8_t codec;
    bool valid;
    bool encode_attempted;
    bool encode_ok;
    bool radio_submitted;
    bool radio_ok;
    dual_end_owner_outcome_t outcome;
} dual_end_audio_owner_timing_t;
bool dual_end_audio_sender_copy_last_owner_timing(dual_end_audio_owner_timing_t *timing);
/* Sender-callback-worker-only handoff.  It consumes one completed sender
 * admission and its classified outcome without taking the sender mutex.  A
 * separate microphone owner may later record this immutable value; callers
 * must not call record_mic_work for the same admission. */
bool dual_end_audio_sender_take_owner_timing(dual_end_audio_owner_timing_t *timing);
/* DSP+analysis+callback wall work, excluding I2S waits; nested encode is not additive. */
void dual_end_audio_sender_record_mic_work(int64_t elapsed_us);
/* 0.4.2 recovery control ingress. The legacy name remains as an application
 * compatibility alias because app_main routes the same receive callback. */
bool dual_end_audio_sender_queue_control(const uint8_t *data, int len);
bool dual_end_audio_sender_handle_retry_request(const uint8_t *data, int len);
void dual_end_audio_sender_note_link_state(bool connected);
void dual_end_audio_sender_set_ota_paused(bool paused);
esp_err_t dual_end_audio_sender_send_pcm(const int16_t *pcm, size_t samples);
esp_err_t dual_end_audio_sender_send_pcm_with_hardware(const int16_t *pcm,
                                                       size_t samples,
                                                       const dual_end_hid_input_payload_t *hardware_payload);
esp_err_t dual_end_audio_sender_send_pcm_with_hardware_and_analysis(
    const int16_t *pcm,
    size_t samples,
    const dual_end_hid_input_payload_t *hardware_payload,
    const dual_end_audio_analysis_metadata_t *analysis_metadata);
esp_err_t dual_end_audio_receiver_set_input_event_sink(dual_end_input_event_sink_t sink, void *ctx);
esp_err_t dual_end_audio_receiver_start(dual_end_audio_pcm_sink_t sink, void *ctx);
esp_err_t dual_end_audio_preencode_test_loop(void);

#ifdef __cplusplus
}
#endif
