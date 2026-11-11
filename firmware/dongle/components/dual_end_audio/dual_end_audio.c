#include <math.h>
#ifndef DUAL_END_PCM_LEVEL_DIAG
#define DUAL_END_PCM_LEVEL_DIAG 0
#endif

#include "dual_end_audio.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "audio_recovery_rx.h"
#include "audio_rx_telemetry.h"
#include "dongle_uac_bridge.h"
#include "dual_end_audio_recovery_protocol.h"
#include "dual_end_espnow_link.h"
#include "dual_end_ota.h"
#include "esp_check.h"
#include "esp_g722_dec.h"
#include "esp_g722_enc.h"
#include "dual_end_opus.h"
#include "dual_end_pcm_layout.h"
#include "dual_end_runtime.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "dual_end_audio";

#define AUDIO_TX_LOG_INTERVAL_FRAMES (1000 / DUAL_END_AUDIO_FRAME_MS)

typedef struct {
    dual_end_packet_header_t packet;
    dual_end_audio_frame_header_t audio;
    dual_end_audio_hardware_payload_t payload;
} __attribute__((packed)) dual_end_audio_packet_t;

_Static_assert(sizeof(dual_end_audio_packet_t) == DUAL_END_AUDIO_PACKET_BYTES,
               "combined ESP-NOW packet must match configured wire size");
_Static_assert(sizeof(dual_end_audio_packet_t) <= DUAL_END_ESPNOW_MAX_DATA_BYTES,
               "combined packet exceeds ESP-NOW payload limit");

static dual_end_audio_pcm_sink_t s_pcm_sink;
static dual_end_audio_timed_pcm_sink_t s_timed_pcm_sink;
static dual_end_audio_playout_clock_t s_playout_clock;
static void *s_pcm_sink_ctx;
static dual_end_input_event_sink_t s_input_event_sink;
static void *s_input_event_sink_ctx;
static dual_end_audio_session_reset_cb_t s_session_reset_cb;
static void *s_session_reset_ctx;

static void *s_g722_encoder[DUAL_END_AUDIO_CHANNELS];
static void *s_g722_decoder[DUAL_END_AUDIO_CHANNELS];
_Alignas(16) static int16_t s_encode_channel_pcm[DUAL_END_AUDIO_CHANNELS][DUAL_END_AUDIO_FRAME_SAMPLES];
_Alignas(16) static int16_t s_decode_channel_pcm[DUAL_END_AUDIO_CHANNELS][DUAL_END_AUDIO_FRAME_SAMPLES];
_Alignas(16) static int16_t s_decoded_interleaved_pcm[DUAL_END_AUDIO_FRAME_SAMPLE_VALUES];
/* Last fully decoded frame: replayed under a one-frame fade when a packet is
 * missing, so concealment keeps spectral continuity instead of a hole. */
_Alignas(16) static int16_t s_last_frame_pcm[DUAL_END_AUDIO_FRAME_SAMPLE_VALUES];
static uint32_t s_concealed_repeat_frames;
static bool s_last_frame_valid;
_Static_assert(DUAL_END_AUDIO_CHANNEL_FRAME_BYTES % 16 == 0, "codec PCM row alignment");
static dual_end_audio_packet_t s_tx_packet;
static uint32_t s_tx_sequence;
static volatile bool s_ota_paused;

static audio_recovery_rx_t s_rx_recovery;
static uint32_t s_feedback_sequence;
static bool s_nack_pending;
static audio_rx_event_t s_nack_event;
static dual_end_audio_nack_packet_t s_nack_packet;
static bool s_hid_sequence_valid;
static uint32_t s_hid_sequence;
static uint32_t s_crc_error_frames;
static uint32_t s_decode_error_frames;
/* Bound only after a valid packet in the current recovery session. */
static uint8_t s_rx_codec = UINT8_MAX;
static uint32_t s_invalid_frames;
static uint32_t s_feedback_queue_failures;
static uint32_t s_delivered_frames;
static uint32_t s_timed_sink_drop_frames;
static uint32_t s_recovered_decoded_frames;
static uint32_t s_masked_frames;
static uint32_t s_nack_queued;
static uint32_t s_hid_dispatches;
static uint32_t s_hid_wait_max_us;
static uint32_t s_hid_wait_peak_sequence;
static int64_t s_hid_wait_peak_us;
static uint32_t s_decode_frame_max_us;
static uint32_t s_poll_max_us;
static uint32_t s_poll_audio_max;
/* Incremented only by the RX owner on a accepted new audio session. This is
 * separate from the UAC bridge generation, which can also change on USB-side
 * timeout/reset. Both are captured in every immutable RX telemetry record. */
static uint32_t s_rx_session_generation;
static int64_t s_last_rx_snapshot_us;
static int64_t s_last_rx_stack_scan_us;
static uint32_t s_rx_stack_min_bytes;
static bool s_rx_snapshot_ready;

static uint32_t increment_nonzero(uint32_t value)
{
    ++value;
    return value ? value : 1U;
}

static void log_analysis_metadata(const dual_end_audio_analysis_metadata_t *analysis,
                                  uint32_t sequence)
{
    if (!analysis || analysis->version != DUAL_END_AUDIO_ANALYSIS_VERSION) {
        return;
    }
    ESP_LOGI(TAG,
             "mic_analysis seq=%" PRIu32 " flags=0x%02x"
             " tdoa=%d/256sample gcc_peak=%u conf=%u coh=%u"
             " quality=%u/%u hf=%u motor=%u clip=%u/%u",
             sequence, analysis->flags, analysis->tdoa_samples_q8_8,
             analysis->gcc_peak_q15, analysis->confidence_q15,
             analysis->coherence_q15, analysis->left_quality_q15,
             analysis->right_quality_q15, analysis->high_frequency_q15,
             analysis->motor_q15, analysis->clip_left_count,
             analysis->clip_right_count);
}

static uint32_t audio_payload_crc32(const uint8_t *payload, size_t len)
{
    return esp_rom_crc32_le(UINT32_MAX, payload, len) ^ UINT32_MAX;
}

static esp_err_t ensure_g722_encoder(void)
{
    bool initialized = false;
    for (size_t channel = 0; channel < DUAL_END_AUDIO_CHANNELS; ++channel) {
        if (s_g722_encoder[channel]) {
            continue;
        }
        esp_g722_enc_config_t cfg = ESP_G722_ENC_CONFIG_DEFAULT();
        cfg.sample_rate = DUAL_END_AUDIO_SAMPLE_RATE_HZ;
        cfg.channel = 1;
        cfg.bits_per_sample = DUAL_END_AUDIO_BITS_PER_SAMPLE;
        cfg.bitrate = DUAL_END_AUDIO_G722_BITRATE_BPS;
        cfg.packed = true;
        cfg.frame_duration = DUAL_END_AUDIO_FRAME_MS;
        esp_audio_err_t err = esp_g722_enc_open(&cfg, sizeof(cfg), &s_g722_encoder[channel]);
        ESP_RETURN_ON_FALSE(err == ESP_AUDIO_ERR_OK && s_g722_encoder[channel],
                            ESP_FAIL, TAG, "open G722 encoder ch=%u err=%d",
                            (unsigned)channel, err);
        int in_size = 0;
        int out_size = 0;
        err = esp_g722_enc_get_frame_size(s_g722_encoder[channel], &in_size, &out_size);
        ESP_RETURN_ON_FALSE(err == ESP_AUDIO_ERR_OK &&
                                in_size == DUAL_END_AUDIO_CHANNEL_FRAME_BYTES &&
                                out_size == DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES,
                            ESP_FAIL, TAG,
                            "G722 encoder frame size ch=%u err=%d in=%d out=%d",
                            (unsigned)channel, err, in_size, out_size);
        initialized = true;
    }
    if (initialized) {
        ESP_LOGI(TAG,
                 "stereo G722 encoders ready: channels=%d bitrate/ch=%d frame_out/ch=%d",
                 DUAL_END_AUDIO_CHANNELS, DUAL_END_AUDIO_G722_BITRATE_BPS,
                 DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES);
    }
    return ESP_OK;
}

static esp_err_t ensure_g722_decoder(void)
{
    bool initialized = false;
    for (size_t channel = 0; channel < DUAL_END_AUDIO_CHANNELS; ++channel) {
        if (s_g722_decoder[channel]) {
            continue;
        }
        esp_g722_dec_cfg_t cfg = {
            .sample_rate = DUAL_END_AUDIO_SAMPLE_RATE_HZ,
            .bitrate = DUAL_END_AUDIO_G722_BITRATE_BPS,
            .packed = true,
        };
        esp_audio_err_t err = esp_g722_dec_open(&cfg, sizeof(cfg), &s_g722_decoder[channel]);
        if (err != ESP_AUDIO_ERR_OK || !s_g722_decoder[channel]) {
            audio_rx_telemetry_event(AUDIO_RX_LOG_DECODER_OPEN_FAILED,
                                     (uint32_t)channel, (uint32_t)err, 0, 0, 0);
            return ESP_FAIL;
        }
        initialized = true;
    }
    if (initialized) {
        audio_rx_telemetry_event(AUDIO_RX_LOG_DECODERS_READY, 0, 0, 0, 0, 0);
    }
    return ESP_OK;
}

static void reset_g722_decoder(void)
{
    for (size_t channel = 0; channel < DUAL_END_AUDIO_CHANNELS; ++channel) {
        if (s_g722_decoder[channel]) {
            (void)esp_g722_dec_close(s_g722_decoder[channel]);
            s_g722_decoder[channel] = NULL;
        }
    }
    memset(s_decode_channel_pcm, 0, sizeof(s_decode_channel_pcm));
    memset(s_decoded_interleaved_pcm, 0, sizeof(s_decoded_interleaved_pcm));
    memset(s_last_frame_pcm, 0, sizeof(s_last_frame_pcm));
    s_last_frame_valid = false;
}

static esp_err_t g722_encode_frame(const int16_t *pcm, uint8_t *payload,
                                   size_t payload_len, uint32_t *encoded_bytes)
{
    ESP_RETURN_ON_ERROR(ensure_g722_encoder(), TAG, "ensure G722 encoder");
    ESP_RETURN_ON_FALSE(payload_len >= DUAL_END_AUDIO_G722_FRAME_BYTES,
                        ESP_ERR_INVALID_SIZE, TAG, "G722 output buffer too small");
    dual_end_pcm_split(pcm, s_encode_channel_pcm[0], s_encode_channel_pcm[1], DUAL_END_AUDIO_FRAME_SAMPLES);
    for (size_t channel = 0; channel < DUAL_END_AUDIO_CHANNELS; ++channel) {
        esp_audio_enc_in_frame_t in_frame = {
            .buffer = (uint8_t *)s_encode_channel_pcm[channel],
            .len = DUAL_END_AUDIO_CHANNEL_FRAME_BYTES,
        };
        esp_audio_enc_out_frame_t out_frame = {
            .buffer = payload + channel * DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES,
            .len = DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES,
        };
        esp_audio_err_t err = esp_g722_enc_process(s_g722_encoder[channel],
                                                   &in_frame, &out_frame);
        ESP_RETURN_ON_FALSE(err == ESP_AUDIO_ERR_OK &&
                                out_frame.encoded_bytes == DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES,
                            ESP_FAIL, TAG, "G722 encode ch=%u err=%d encoded=%" PRIu32,
                            (unsigned)channel, err, out_frame.encoded_bytes);
    }
    *encoded_bytes = DUAL_END_AUDIO_G722_FRAME_BYTES;
    return ESP_OK;
}

static esp_err_t g722_decode_frame(const uint8_t *payload, size_t encoded_bytes,
                                   int16_t *pcm, size_t pcm_bytes)
{
    ESP_RETURN_ON_ERROR(ensure_g722_decoder(), TAG, "ensure G722 decoder");
    ESP_RETURN_ON_FALSE(encoded_bytes == DUAL_END_AUDIO_G722_FRAME_BYTES &&
                            pcm_bytes >= DUAL_END_AUDIO_FRAME_BYTES,
                        ESP_ERR_INVALID_SIZE, TAG, "invalid stereo G722 frame size");
    for (size_t channel = 0; channel < DUAL_END_AUDIO_CHANNELS; ++channel) {
        esp_audio_dec_in_raw_t raw = {
            .buffer = (uint8_t *)payload + channel * DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES,
            .len = DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES,
        };
        esp_audio_dec_out_frame_t out_frame = {
            .buffer = (uint8_t *)s_decode_channel_pcm[channel],
            .len = DUAL_END_AUDIO_CHANNEL_FRAME_BYTES,
        };
        esp_audio_dec_info_t dec_info = {0};
        esp_audio_err_t err = esp_g722_dec_decode(s_g722_decoder[channel],
                                                  &raw, &out_frame, &dec_info);
        if (err != ESP_AUDIO_ERR_OK ||
            raw.consumed != DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES ||
            out_frame.decoded_size != DUAL_END_AUDIO_CHANNEL_FRAME_BYTES) {
            audio_rx_telemetry_event(AUDIO_RX_LOG_DECODE_FAILED,
                                     (uint32_t)channel, (uint32_t)err,
                                     raw.consumed, out_frame.decoded_size,
                                     out_frame.needed_size);
            return ESP_FAIL;
        }
    }
    dual_end_pcm_interleave(s_decode_channel_pcm[0], s_decode_channel_pcm[1], pcm, DUAL_END_AUDIO_FRAME_SAMPLES);
    return ESP_OK;
}

esp_err_t dual_end_audio_sender_start(void)
{
    s_tx_sequence = 0;
    return ensure_g722_encoder();
}

void dual_end_audio_sender_set_ota_paused(bool paused)
{
    s_ota_paused = paused;
}

esp_err_t dual_end_audio_sender_send_pcm(const int16_t *pcm, size_t samples)
{
    return dual_end_audio_sender_send_pcm_with_hardware(pcm, samples, NULL);
}

esp_err_t dual_end_audio_sender_send_pcm_with_hardware(
    const int16_t *pcm, size_t samples,
    const dual_end_hid_input_payload_t *hardware_payload)
{
    if (s_ota_paused) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_FALSE(pcm, ESP_ERR_INVALID_ARG, TAG, "pcm is null");
    ESP_RETURN_ON_FALSE(samples == DUAL_END_AUDIO_FRAME_SAMPLE_VALUES,
                        ESP_ERR_INVALID_SIZE, TAG, "pcm samples=%u expected=%u",
                        (unsigned)samples, DUAL_END_AUDIO_FRAME_SAMPLE_VALUES);
    uint32_t encoded_bytes = 0;
    memset(&s_tx_packet, 0, sizeof(s_tx_packet));
    ESP_RETURN_ON_ERROR(g722_encode_frame(pcm, s_tx_packet.payload.audio,
                                          sizeof(s_tx_packet.payload.audio),
                                          &encoded_bytes),
                        TAG, "encode real PCM");
    if (hardware_payload) {
        s_tx_packet.payload.hardware = *hardware_payload;
    } else {
        s_tx_packet.payload.hardware.version = DUAL_END_HID_INPUT_VERSION;
    }
    s_tx_packet.packet.protocol_version = SCEPTER_PROTOCOL_VERSION;
    s_tx_packet.packet.source_role = SCEPTER_ROLE_SCEPTER;
    s_tx_packet.packet.message_type = SCEPTER_MSG_AUDIO_FRAME;
    s_tx_packet.packet.flags = (uint8_t)SCEPTER_ESPNOW_TX_POWER_QDBM;
    s_tx_packet.packet.sequence = s_tx_sequence;
    s_tx_packet.packet.payload_len =
        sizeof(dual_end_audio_frame_header_t) + sizeof(s_tx_packet.payload);
    s_tx_packet.audio.codec = DUAL_END_AUDIO_CODEC_G722;
    s_tx_packet.audio.channels = DUAL_END_AUDIO_CHANNELS;
    s_tx_packet.audio.sample_rate_hz = DUAL_END_AUDIO_SAMPLE_RATE_HZ;
    s_tx_packet.audio.samples_per_channel = DUAL_END_AUDIO_FRAME_SAMPLES;
    s_tx_packet.audio.pcm_bytes_after_decode = DUAL_END_AUDIO_FRAME_BYTES;
    s_tx_packet.audio.payload_crc32 = audio_payload_crc32(
        (const uint8_t *)&s_tx_packet.payload, sizeof(s_tx_packet.payload));
    s_tx_packet.audio.previous_encoded_bytes = 0;
    s_tx_packet.audio.current_encoded_bytes = (uint16_t)encoded_bytes;
    const size_t packet_len = sizeof(dual_end_packet_header_t) +
                              s_tx_packet.packet.payload_len;
    esp_err_t ret = dual_end_espnow_link_send_audio(&s_tx_packet, (int)packet_len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "audio send seq=%" PRIu32 " ret=%s",
                 s_tx_sequence, esp_err_to_name(ret));
    } else if ((s_tx_sequence % AUDIO_TX_LOG_INTERVAL_FRAMES) == 0) {
        ESP_LOGI(TAG, "audio tx seq=%" PRIu32 " bytes=%u codec=g722-stereo",
                 s_tx_sequence, (unsigned)packet_len);
    }
    ++s_tx_sequence;
    return ret;
}

esp_err_t dual_end_audio_preencode_test_loop(void)
{
    return dual_end_audio_sender_start();
}

static bool audio_format_valid(const dual_end_audio_frame_header_t *audio)
{
    const bool codec_size =
        (audio->codec == DUAL_END_AUDIO_CODEC_G722 &&
         audio->current_encoded_bytes == DUAL_END_AUDIO_G722_FRAME_BYTES) ||
        (audio->codec == DUAL_END_AUDIO_CODEC_OPUS &&
         audio->current_encoded_bytes == DUAL_END_AUDIO_OPUS_FRAME_BYTES) ||
        (audio->codec == DUAL_END_AUDIO_CODEC_OPUS_62 &&
         audio->current_encoded_bytes == DUAL_END_AUDIO_OPUS62_FRAME_BYTES);
    return codec_size &&
           audio->sample_rate_hz == DUAL_END_AUDIO_SAMPLE_RATE_HZ &&
           audio->channels == DUAL_END_AUDIO_CHANNELS &&
           audio->samples_per_channel == DUAL_END_AUDIO_FRAME_SAMPLES &&
           audio->pcm_bytes_after_decode == DUAL_END_AUDIO_FRAME_BYTES &&
           audio->previous_encoded_bytes == 0;
}

/* All callers are on the single receiver owner task. Recovery slots do not
 * duplicate metadata: one session binds one codec after CRC/length validation.
 * Only admitted frames call this gate. Old, out-of-window or late packets
 * cannot bind a codec or mutate decoder state. */
static bool receiver_bind_codec(uint8_t codec, uint32_t session_id)
{
    if ((codec != DUAL_END_AUDIO_CODEC_G722 && !dual_end_audio_codec_is_opus(codec)) ||
        !s_rx_recovery.has_session || !s_rx_recovery.active ||
        session_id != s_rx_recovery.session_id) return false;
    if (s_rx_codec != UINT8_MAX) return codec == s_rx_codec;
    esp_err_t ret = dual_end_audio_codec_is_opus(codec) ?
        dual_end_opus_decoder_prepare() : ensure_g722_decoder();
    if (ret != ESP_OK) return false;
    s_rx_codec = codec;
    ESP_LOGI(TAG, "receiver codec=%u session=%08" PRIx32, codec, session_id);
    return true;
}
uint16_t dual_end_audio_receiver_codec_capabilities(void)
{
    return DUAL_END_AUDIO_CODEC_CAPABILITIES;
}

static void feedback_header(dual_end_packet_header_t *header, uint8_t type,
                            size_t length)
{
    header->protocol_version = SCEPTER_PROTOCOL_VERSION;
    header->source_role = SCEPTER_ROLE_DONGLE;
    header->message_type = type;
    header->sequence = s_feedback_sequence++;
    header->payload_len = (uint16_t)(length - sizeof(*header));
}

static void send_session_feedback(uint8_t command)
{
    dual_end_audio_session_packet_t packet = {
        .version = DUAL_END_AUDIO_RECOVERY_VERSION,
        .command = command,
        .session_id = s_rx_recovery.has_session ? s_rx_recovery.session_id : 0,
        .first_sequence = s_rx_recovery.has_session ? s_rx_recovery.first_sequence : 0,
    };
    feedback_header(&packet.packet, SCEPTER_MSG_AUDIO_SESSION, sizeof(packet));
    if (dual_end_espnow_link_queue_audio_feedback(&packet, sizeof(packet)) != ESP_OK) {
        ++s_feedback_queue_failures;
    }
}

static void update_pending_nack(int64_t now_us)
{
    if (!s_nack_pending) {
        return;
    }
    int64_t latest_send_us = 0;
    if (!audio_recovery_rx_refresh_nack(&s_rx_recovery, &s_nack_event,
                                        now_us, &latest_send_us)) {
        if (dual_end_espnow_link_cancel_audio_nack(s_nack_event.nack_token)) {
            audio_recovery_rx_finish_nack(
                &s_rx_recovery, s_nack_event.nack_token,
                latest_send_us && now_us >= latest_send_us ?
                    AUDIO_RX_NACK_EXPIRED : AUDIO_RX_NACK_CANCELLED,
                now_us);
            s_nack_pending = false;
        } else if (!s_rx_recovery.nack_pending ||
                   s_rx_recovery.nack_token != s_nack_event.nack_token) {
            s_nack_pending = false;
        }
        return;
    }
    s_nack_packet.base_sequence = s_nack_event.sequence;
    s_nack_packet.missing_mask = s_nack_event.missing_mask;
    s_nack_packet.remaining_ms = s_nack_event.remaining_ms;
    dual_end_nack_snapshot_t snapshot = {
        .token = s_nack_event.nack_token,
        .latest_send_us = latest_send_us,
        .packet = s_nack_packet,
    };
    if (dual_end_espnow_link_publish_audio_nack(&snapshot) != ESP_OK) {
        ++s_feedback_queue_failures;
        audio_recovery_rx_finish_nack(&s_rx_recovery,
                                      s_nack_event.nack_token,
                                      AUDIO_RX_NACK_REJECTED, now_us);
        s_nack_pending = false;
    }
}

static void consume_nack_admissions(void)
{
    dual_end_nack_result_t result;
    while (dual_end_espnow_link_take_nack_result(&result)) {
        audio_rx_nack_result_t outcome = AUDIO_RX_NACK_REJECTED;
        switch (result.kind) {
        case DUAL_END_NACK_ACCEPTED: outcome = AUDIO_RX_NACK_ACCEPTED; break;
        case DUAL_END_NACK_CANCELLED: outcome = AUDIO_RX_NACK_CANCELLED; break;
        case DUAL_END_NACK_EXPIRED: outcome = AUDIO_RX_NACK_EXPIRED; break;
        case DUAL_END_NACK_REJECTED: break;
        }
        audio_recovery_rx_finish_nack(&s_rx_recovery, result.token,
                                      outcome, result.submitted_us);
        if (s_nack_pending && s_nack_event.nack_token == result.token) {
            s_nack_pending = false;
        }
    }
}

static void note_duration_max(uint32_t *maximum, int64_t duration_us)
{
    if (duration_us > 0 && (uint64_t)duration_us > *maximum) {
        *maximum = duration_us > UINT32_MAX ? UINT32_MAX : (uint32_t)duration_us;
    }
}

/* A missing burst may use the last real frame once. Consuming its validity
 * prevents every subsequent 20 ms hole from restarting at full amplitude. */
static void conceal_missing_frame(void)
{
    if (!s_last_frame_valid) {
        memset(s_decoded_interleaved_pcm, 0, sizeof(s_decoded_interleaved_pcm));
        return;
    }
    memcpy(s_decoded_interleaved_pcm, s_last_frame_pcm, sizeof(s_last_frame_pcm));
    for (size_t sample = 0; sample < DUAL_END_AUDIO_FRAME_SAMPLES; ++sample) {
        const int32_t gain = (int32_t)(DUAL_END_AUDIO_FRAME_SAMPLES - 1U - sample);
        for (size_t channel = 0; channel < DUAL_END_AUDIO_CHANNELS; ++channel) {
            int16_t *value = &s_decoded_interleaved_pcm[
                sample * DUAL_END_AUDIO_CHANNELS + channel];
            *value = (int16_t)((int32_t)*value * gain /
                               (int32_t)(DUAL_END_AUDIO_FRAME_SAMPLES - 1U));
        }
    }
    s_last_frame_valid = false;
    ++s_concealed_repeat_frames;
}

static void publish_rx_telemetry(int64_t now_us)
{
    if (!audio_rx_telemetry_enabled()) return;
    static audio_rx_telemetry_snapshot_t snapshot;
    const bool changed = !s_rx_snapshot_ready ||
        snapshot.session_generation != s_rx_session_generation ||
        snapshot.active != s_rx_recovery.active || snapshot.codec != s_rx_codec;
    if (!changed && now_us - s_last_rx_snapshot_us < 1000000LL) return;
    /* Stack watermark scanning stays at 10 s even when cache copies are 1 Hz.
     * A failed bounded enqueue retries at the next interval, never every poll. */
    if (!s_rx_snapshot_ready || now_us - s_last_rx_stack_scan_us >= 10000000LL) {
        s_rx_stack_min_bytes = audio_rx_telemetry_stack_min();
        s_last_rx_stack_scan_us = now_us;
    }
    s_last_rx_snapshot_us = now_us;
    s_rx_snapshot_ready = true;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.stats = s_rx_recovery.stats;
    snapshot.sampled_us = now_us;
    snapshot.session_id = s_rx_recovery.session_id;
    snapshot.session_generation = s_rx_session_generation;
    snapshot.uac_session_generation = dongle_uac_bridge_session_generation();
    snapshot.codec = s_rx_codec;
    snapshot.active = s_rx_recovery.active;
    snapshot.gap_deadline_us = s_rx_recovery.gap_active ?
        audio_recovery_rx_playout_us(&s_rx_recovery, s_rx_recovery.next_sequence) -
            DUAL_END_AUDIO_DECODE_MARGIN_US : 0;
    snapshot.delivered_frames = s_delivered_frames;
    snapshot.timed_sink_drop_frames = s_timed_sink_drop_frames;
    snapshot.masked_frames = s_masked_frames;
    snapshot.concealed_repeat_frames = s_concealed_repeat_frames;
    snapshot.crc_error_frames = s_crc_error_frames;
    snapshot.decode_error_frames = s_decode_error_frames;
    snapshot.invalid_frames = s_invalid_frames;
    snapshot.feedback_queue_failures = s_feedback_queue_failures;
    snapshot.nack_queued = s_nack_queued;
    snapshot.recovered_decoded_frames = s_recovered_decoded_frames;
    snapshot.hid_dispatches = s_hid_dispatches;
    snapshot.hid_wait_max_us = s_hid_wait_max_us;
    snapshot.hid_wait_peak_sequence = s_hid_wait_peak_sequence;
    snapshot.hid_wait_peak_us = s_hid_wait_peak_us;
    snapshot.decode_frame_max_us = s_decode_frame_max_us;
    snapshot.poll_max_us = s_poll_max_us;
    snapshot.poll_audio_max = s_poll_audio_max;
    snapshot.rx_stack_min_bytes = s_rx_stack_min_bytes;
    (void)audio_rx_telemetry_publish(&snapshot);
}

static void audio_poll_cb(void *ctx)
{
    (void)ctx;
    const int64_t poll_started_us = esp_timer_get_time();
    int64_t now_us = poll_started_us;
    uint32_t audio_events = 0;
    uint32_t cursor_sequence = 0;
    int64_t cursor_playout_us = 0;
    if (s_playout_clock &&
        s_playout_clock(&cursor_sequence, &cursor_playout_us, s_pcm_sink_ctx)) {
        audio_recovery_rx_set_playout_anchor(&s_rx_recovery,
                                             cursor_sequence,
                                             cursor_playout_us);
    }
    consume_nack_admissions();
    update_pending_nack(now_us);
    audio_rx_event_t event;
    while ((now_us = esp_timer_get_time()),
           audio_recovery_rx_next(&s_rx_recovery, now_us, &event)) {
        if (event.kind == AUDIO_RX_END) {
            continue;
        }
        if (event.kind == AUDIO_RX_NACK) {
            s_nack_event = event;
            s_nack_packet = (dual_end_audio_nack_packet_t){
                .version = DUAL_END_AUDIO_RECOVERY_VERSION,
                .session_id = event.session_id,
                .base_sequence = event.sequence,
                .missing_mask = event.missing_mask,
            };
            feedback_header(&s_nack_packet.packet,
                            SCEPTER_MSG_AUDIO_NACK,
                            sizeof(s_nack_packet));
            s_nack_pending = true;
            update_pending_nack(now_us);
            if (s_nack_pending) {
                ++s_nack_queued;
            }
            continue;
        }
        if (event.kind == AUDIO_RX_RESYNC) {
            send_session_feedback(DUAL_END_AUDIO_SESSION_RESYNC);
            continue;
        }

        ++audio_events;
        if (event.kind == AUDIO_RX_FRAME) {
            const int64_t decode_started_us = esp_timer_get_time();
            esp_err_t decode_ret = dual_end_audio_codec_is_opus(s_rx_codec) ?
                dual_end_opus_decode(event.encoded, sizeof(event.encoded),
                    s_decoded_interleaved_pcm, sizeof(s_decoded_interleaved_pcm)) :
                g722_decode_frame(event.encoded, sizeof(event.encoded),
                    s_decoded_interleaved_pcm, sizeof(s_decoded_interleaved_pcm));
            const int64_t decode_us = esp_timer_get_time() - decode_started_us;
            note_duration_max(&s_decode_frame_max_us, decode_us);
            dual_end_runtime_record(DUAL_END_STAGE_DECODE, s_rx_codec, event.session_id,
                                    decode_us, decode_ret == ESP_OK);
            if (decode_ret != ESP_OK) {
                /* One channel may have advanced before the other failed.
                 * Reset both before abandoning this synchronized session. */
                if (dual_end_audio_codec_is_opus(s_rx_codec)) dual_end_opus_decoder_reset();
                ++s_decode_error_frames;
                audio_recovery_rx_abort(&s_rx_recovery, now_us);
                continue;
            }
#if DUAL_END_PCM_LEVEL_DIAG
            /* Diagnostic: level of the decoded PCM handed to the UAC sink. */
            {
                static int64_t level_log_us;
                const int64_t level_now = esp_timer_get_time();
                if (level_now - level_log_us >= 2000000) {
                    level_log_us = level_now;
                    int32_t peak = 0;
                    int64_t sum = 0;
                    for (size_t i = 0; i < DUAL_END_AUDIO_FRAME_SAMPLE_VALUES; ++i) {
                        const int32_t v = s_decoded_interleaved_pcm[i];
                        const int32_t a = v < 0 ? -v : v;
                        if (a > peak) peak = a;
                        sum += (int64_t)v * v;
                    }
                    ESP_LOGI(TAG, "decoded pcm peak=%d rms=%d", (int)peak,
                             (int)sqrt((double)sum /
                                       (double)DUAL_END_AUDIO_FRAME_SAMPLE_VALUES));
                }
            }
#endif
            ++s_delivered_frames;
            memcpy(s_last_frame_pcm, s_decoded_interleaved_pcm,
                   sizeof(s_last_frame_pcm));
            s_last_frame_valid = true;
            if (event.repaired) {
                ++s_recovered_decoded_frames;
            }
        } else {
            /* Opus predictor reset is distinct from our bounded PCM fade. */
            if (dual_end_audio_codec_is_opus(s_rx_codec)) dual_end_opus_decoder_reset();
            conceal_missing_frame();
            ++s_masked_frames;
        }

        esp_err_t sink_result = ESP_OK;
        if (s_timed_pcm_sink) {
            sink_result = s_timed_pcm_sink(
                s_decoded_interleaved_pcm, DUAL_END_AUDIO_FRAME_SAMPLES,
                event.sequence, event.rssi, event.playout_us, s_pcm_sink_ctx);
        } else if (s_pcm_sink) {
            sink_result = s_pcm_sink(
                s_decoded_interleaved_pcm, DUAL_END_AUDIO_FRAME_SAMPLE_VALUES,
                event.sequence, event.rssi, s_pcm_sink_ctx);
        }
        if (sink_result != ESP_OK) {
            if (sink_result == ESP_ERR_NO_MEM &&
                s_timed_pcm_sink == dongle_uac_bridge_write_timed_pcm) {
                /* The fixed-timeline sink rejected only this frame because
                 * its bounded slot is still occupied. Keep decoder/recovery
                 * state intact; the bridge separately records dropped PCM. */
                if (s_timed_sink_drop_frames != UINT32_MAX)
                    ++s_timed_sink_drop_frames;
            } else {
                audio_recovery_rx_abort(&s_rx_recovery, now_us);
            }
        }
        break;
    }
    now_us = esp_timer_get_time();
    update_pending_nack(now_us);
    if (audio_events > s_poll_audio_max) {
        s_poll_audio_max = audio_events;
    }
    note_duration_max(&s_poll_max_us, now_us - poll_started_us);
    if (s_rx_codec != UINT8_MAX && s_rx_recovery.has_session)
        dual_end_runtime_record(DUAL_END_STAGE_RX_POLL, s_rx_codec, s_rx_recovery.session_id,
                                now_us - poll_started_us, true);
    publish_rx_telemetry(now_us);
}

static void begin_audio_session(const dual_end_audio_session_packet_t *session,
                                int64_t now_us)
{
    /* begin validates retired/duplicate identities before touching the live
     * recovery state. A delayed old BEGIN must never stop the current stream. */
    audio_rx_session_result_t result = audio_recovery_rx_begin(
        &s_rx_recovery, session->session_id, session->first_sequence, now_us);
    if (result == AUDIO_RX_SESSION_REJECTED) {
        ++s_invalid_frames;
        return;
    }
    /* Retire old queued NACK evidence only after acceptance, and before any
     * decoder/UAC reset callback. Claimed TX requests keep their old tokens. */
    update_pending_nack(now_us);
    if (result == AUDIO_RX_SESSION_NEW) {
        reset_g722_decoder();
        dual_end_opus_decoder_reset();
        s_rx_codec = UINT8_MAX;
        s_rx_session_generation = increment_nonzero(s_rx_session_generation);
        s_hid_sequence_valid = false;
        if (s_session_reset_cb) {
            s_session_reset_cb(s_session_reset_ctx);
        }
        audio_rx_telemetry_event(AUDIO_RX_LOG_SESSION_BEGIN,
                                 session->session_id,
                                 session->first_sequence, 0, 0, 0);
    }
    send_session_feedback(DUAL_END_AUDIO_SESSION_ACK);
}

static void audio_recv_cb(const uint8_t *src_addr, const uint8_t *data,
                          int len, int rssi, void *ctx)
{
    (void)src_addr;
    (void)ctx;
    if (dual_end_ota_handle_scepter_espnow_packet(data, len)) {
        return;
    }
    if (!data || len < (int)sizeof(dual_end_packet_header_t)) {
        return;
    }
    const dual_end_packet_header_t *header =
        (const dual_end_packet_header_t *)data;
    if (header->protocol_version != SCEPTER_PROTOCOL_VERSION ||
        header->source_role != SCEPTER_ROLE_SCEPTER) {
        return;
    }
    const int64_t now_us = esp_timer_get_time();
    const int64_t received_us = dual_end_espnow_link_rx_received_us();

    if (header->message_type == SCEPTER_MSG_AUDIO_SESSION) {
        audio_poll_cb(NULL);
        if (len != sizeof(dual_end_audio_session_packet_t) ||
            !dual_end_audio_recovery_header_valid(
                header, (size_t)len, SCEPTER_MSG_AUDIO_SESSION,
                SCEPTER_ROLE_SCEPTER)) {
            ++s_invalid_frames;
            return;
        }
        dual_end_audio_session_packet_t session;
        memcpy(&session, data, sizeof(session));
        if (session.version != DUAL_END_AUDIO_RECOVERY_VERSION ||
            session.command != DUAL_END_AUDIO_SESSION_BEGIN ||
            session.reserved != 0 || session.session_id == 0) {
            ++s_invalid_frames;
            return;
        }
        begin_audio_session(&session, now_us);
        return;
    }

    if (header->message_type == SCEPTER_MSG_AUDIO_REPAIR) {
        audio_poll_cb(NULL);
        if (len != sizeof(dual_end_audio_repair_packet_t) ||
            !dual_end_audio_recovery_header_valid(
                header, (size_t)len, SCEPTER_MSG_AUDIO_REPAIR,
                SCEPTER_ROLE_SCEPTER)) {
            ++s_invalid_frames;
            return;
        }
        dual_end_audio_repair_packet_t repair;
        memcpy(&repair, data, sizeof(repair));
        if (!audio_format_valid(&repair.audio) ||
            !dual_end_codec_payload_valid(repair.audio.codec,
                repair.audio.current_encoded_bytes, repair.encoded, sizeof(repair.encoded))) {
            ++s_invalid_frames;
            return;
        }
        if (repair.audio.payload_crc32 !=
            audio_payload_crc32(repair.encoded, sizeof(repair.encoded))) {
            ++s_crc_error_frames;
            return;
        }
        if (s_rx_codec != UINT8_MAX && repair.audio.codec != s_rx_codec) {
            ++s_invalid_frames;
            return;
        }
        const audio_rx_push_result_t pushed = audio_recovery_rx_push(
            &s_rx_recovery, repair.session_id, repair.packet.sequence,
            repair.encoded, rssi, true, received_us);
        if (pushed == AUDIO_RX_ACCEPTED &&
            !receiver_bind_codec(repair.audio.codec, repair.session_id)) {
            ++s_decode_error_frames;
            audio_recovery_rx_abort(&s_rx_recovery, now_us);
        }
        audio_poll_cb(NULL);
        return;
    }

    if (header->message_type != SCEPTER_MSG_AUDIO_FRAME) {
        return;
    }
    if (len != sizeof(dual_end_audio_packet_t) ||
        !dual_end_audio_recovery_header_valid(
            header, (size_t)len, SCEPTER_MSG_AUDIO_FRAME,
            SCEPTER_ROLE_SCEPTER)) {
        ++s_invalid_frames;
        return;
    }
    dual_end_audio_packet_t packet;
    memcpy(&packet, data, sizeof(packet));
    if (!audio_format_valid(&packet.audio) || packet.payload.stream_id == 0 ||
        !dual_end_codec_payload_valid(packet.audio.codec,
            packet.audio.current_encoded_bytes, packet.payload.audio, sizeof(packet.payload.audio))) {
        ++s_invalid_frames;
        return;
    }
    if (packet.audio.payload_crc32 != audio_payload_crc32(
            (const uint8_t *)&packet.payload, sizeof(packet.payload))) {
        ++s_crc_error_frames;
        return;
    }

    audio_poll_cb(NULL);
    if (!s_rx_recovery.has_session || !s_rx_recovery.active ||
        packet.payload.stream_id != s_rx_recovery.session_id ||
        received_us < s_rx_recovery.started_us ||
        dual_end_audio_seq_before(packet.packet.sequence,
                                  s_rx_recovery.first_sequence)) {
        ++s_rx_recovery.stats.rejected;
        if (!s_rx_recovery.has_session) {
            audio_recovery_rx_request_session(&s_rx_recovery, now_us);
        }
        audio_poll_cb(NULL);
        return;
    }

    if (s_rx_codec != UINT8_MAX && packet.audio.codec != s_rx_codec) {
        ++s_invalid_frames;
        return;
    }
    dual_end_espnow_link_note_audio_received(received_us);
    bool new_input = !s_hid_sequence_valid ||
        dual_end_audio_seq_before(s_hid_sequence, packet.packet.sequence);
    if (new_input) {
        s_hid_sequence_valid = true;
        s_hid_sequence = packet.packet.sequence;
        ++s_hid_dispatches;
        int64_t wait_us = esp_timer_get_time() - received_us;
        if (wait_us > 0 && (uint64_t)wait_us > s_hid_wait_max_us) {
            s_hid_wait_max_us = wait_us > UINT32_MAX ?
                UINT32_MAX : (uint32_t)wait_us;
            s_hid_wait_peak_sequence = packet.packet.sequence;
            s_hid_wait_peak_us = esp_timer_get_time();
        }
        if (s_input_event_sink) {
            dual_end_hid_input_payload_t hardware;
            memcpy(&hardware, &packet.payload.hardware, sizeof(hardware));
            s_input_event_sink(&hardware, packet.packet.sequence,
                               rssi, s_input_event_sink_ctx);
        }
        if ((packet.packet.sequence % 250U) == 0U) {
            dual_end_audio_analysis_metadata_t analysis;
            memcpy(&analysis, &packet.payload.analysis, sizeof(analysis));
            log_analysis_metadata(&analysis, packet.packet.sequence);
        }
    }

    /* Push synchronously copies the same 320 byte representation into its
     * recovery slot before this stack packet expires. No borrowed buffer. */
    const audio_rx_push_result_t pushed = audio_recovery_rx_push(&s_rx_recovery,
                                 packet.payload.stream_id,
                                 packet.packet.sequence,
                                 packet.payload.audio, rssi, false, received_us);
    if (pushed == AUDIO_RX_ACCEPTED &&
        !receiver_bind_codec(packet.audio.codec, packet.payload.stream_id)) {
        ++s_decode_error_frames;
        audio_recovery_rx_abort(&s_rx_recovery, now_us);
    }
    audio_poll_cb(NULL);
}

esp_err_t dual_end_audio_receiver_set_input_event_sink(
    dual_end_input_event_sink_t sink, void *ctx)
{
    s_input_event_sink = sink;
    s_input_event_sink_ctx = ctx;
    return ESP_OK;
}

void dual_end_audio_receiver_set_session_reset_cb(
    dual_end_audio_session_reset_cb_t cb, void *ctx)
{
    s_session_reset_cb = cb;
    s_session_reset_ctx = ctx;
}

static esp_err_t receiver_start_common(void)
{
    audio_recovery_rx_init(&s_rx_recovery);
    s_rx_codec = UINT8_MAX;
    dual_end_opus_decoder_reset();
    s_feedback_sequence = 0;
    s_nack_pending = false;
    s_hid_sequence_valid = false;
    s_crc_error_frames = 0;
    s_decode_error_frames = 0;
    s_invalid_frames = 0;
    s_feedback_queue_failures = 0;
    s_delivered_frames = 0;
    s_timed_sink_drop_frames = 0;
    s_recovered_decoded_frames = 0;
    s_masked_frames = 0;
    s_concealed_repeat_frames = 0;
    s_nack_queued = 0;
    s_hid_dispatches = 0;
    s_hid_wait_max_us = 0;
    s_decode_frame_max_us = 0;
    s_poll_max_us = 0;
    s_poll_audio_max = 0;
    s_rx_session_generation = 0;
    s_last_rx_snapshot_us = 0;
    s_last_rx_stack_scan_us = 0;
    s_rx_stack_min_bytes = 0;
    s_rx_snapshot_ready = false;
    memset(s_last_frame_pcm, 0, sizeof(s_last_frame_pcm));
    s_last_frame_valid = false;
    /* RX telemetry is optional: allocation failure must never prevent the
     * timed audio receiver from registering its callbacks or starting UAC. */
    (void)audio_rx_telemetry_start();
    ESP_RETURN_ON_ERROR(ensure_g722_decoder(), TAG, "init G722 decoder");
    ESP_RETURN_ON_ERROR(dual_end_espnow_link_register_recv_cb(audio_recv_cb, NULL),
                        TAG, "register ESP-NOW audio RX");
    ESP_RETURN_ON_ERROR(dual_end_espnow_link_register_poll_cb(audio_poll_cb, NULL),
                        TAG, "register audio recovery poll");
    ESP_LOGI(TAG,
             "audio receiver started: stereo G722 %d Hz recovery_window=%u playout=%lldms",
             DUAL_END_AUDIO_SAMPLE_RATE_HZ,
             (unsigned)DUAL_END_AUDIO_RECOVERY_WINDOW,
             (long long)(DUAL_END_AUDIO_PLAYOUT_DELAY_US / 1000LL));
    return ESP_OK;
}

esp_err_t dual_end_audio_receiver_start(dual_end_audio_pcm_sink_t sink, void *ctx)
{
    s_pcm_sink_ctx = ctx;
    s_timed_pcm_sink = NULL;
    s_playout_clock = NULL;
    /* Keep app_main unchanged while selecting the 0.4.2 absolute USB timeline
     * whenever it supplies the standard Dongle bridge callback. */
    if (sink == dongle_uac_bridge_write_pcm) {
        s_pcm_sink = NULL;
        s_timed_pcm_sink = dongle_uac_bridge_write_timed_pcm;
        s_playout_clock = dongle_uac_bridge_playout_clock;
    } else {
        s_pcm_sink = sink;
    }
    return receiver_start_common();
}

esp_err_t dual_end_audio_receiver_start_timed(
    dual_end_audio_timed_pcm_sink_t sink,
    dual_end_audio_playout_clock_t clock, void *ctx)
{
    s_pcm_sink = NULL;
    s_timed_pcm_sink = sink;
    s_playout_clock = clock;
    s_pcm_sink_ctx = ctx;
    return receiver_start_common();
}
