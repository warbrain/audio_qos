#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dual_end_link_config.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Legacy double-mic sink: samples counts interleaved int16_t values. */
typedef esp_err_t (*dual_end_audio_pcm_sink_t)(const int16_t *pcm,
                                              size_t samples,
                                              uint32_t sequence,
                                              int rssi,
                                              void *ctx);
typedef esp_err_t (*dual_end_audio_timed_pcm_sink_t)(const int16_t *pcm,
    size_t samples, uint32_t sequence, int rssi, int64_t playout_us, void *ctx);
typedef bool (*dual_end_audio_playout_clock_t)(uint32_t *sequence,
    int64_t *playout_us, void *ctx);
typedef void (*dual_end_input_event_sink_t)(
    const dual_end_hid_input_payload_t *payload,
    uint32_t sequence, int rssi, void *ctx);
typedef void (*dual_end_audio_session_reset_cb_t)(void *ctx);

/* Compile-time decode capability mask: G722 bit 0, Opus bit 1. */
uint16_t dual_end_audio_receiver_codec_capabilities(void);
esp_err_t dual_end_audio_sender_start(void);
void dual_end_audio_sender_set_ota_paused(bool paused);
esp_err_t dual_end_audio_sender_send_pcm(const int16_t *pcm, size_t samples);
esp_err_t dual_end_audio_sender_send_pcm_with_hardware(
    const int16_t *pcm, size_t samples,
    const dual_end_hid_input_payload_t *hardware_payload);
esp_err_t dual_end_audio_receiver_set_input_event_sink(
    dual_end_input_event_sink_t sink, void *ctx);
void dual_end_audio_receiver_set_session_reset_cb(
    dual_end_audio_session_reset_cb_t cb, void *ctx);
esp_err_t dual_end_audio_receiver_start(dual_end_audio_pcm_sink_t sink, void *ctx);
esp_err_t dual_end_audio_receiver_start_timed(
    dual_end_audio_timed_pcm_sink_t sink,
    dual_end_audio_playout_clock_t clock, void *ctx);
esp_err_t dual_end_audio_preencode_test_loop(void);

#ifdef __cplusplus
}
#endif
