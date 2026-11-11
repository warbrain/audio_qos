#include <math.h>
#ifndef DUAL_END_PCM_LEVEL_DIAG
#define DUAL_END_PCM_LEVEL_DIAG 0
#endif

#include "dual_end_audio.h"
#include "dual_end_runtime.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "esp_g722_dec.h"
#include "esp_g722_enc.h"
#include "dual_end_opus.h"
#include "dual_end_pcm_layout.h"
#include "dual_end_spatial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "dual_end_link_config.h"
#include "dual_end_espnow_link.h"
#include "audio_recovery_tx.h"
#include "audio_recovery_tx_transport.h"
#include "audio_session_gate.h"

static const char *TAG = "dual_end_audio";

static const char *codec_profile_name(uint8_t codec)
{
    switch (codec) {
    case DUAL_END_AUDIO_CODEC_OPUS:
        return "opus32";
    case DUAL_END_AUDIO_CODEC_G722:
        return "g72264";
    case DUAL_END_AUDIO_CODEC_OPUS_62:
        return "opus62";
    default:
        return "unknown";
    }
}

static const char *codec_fec_state(uint8_t codec)
{
    return dual_end_audio_codec_is_opus(codec) ? "0" : "na";
}

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

typedef struct {
    uint64_t frames_rx;
    uint64_t lost_frames;
    uint64_t loss_events;
    uint64_t duplicate_frames;
    uint64_t out_of_order_frames;
    uint64_t crc_error_frames;
    int64_t last_rx_us;
    int32_t jitter_min_us;
    int32_t jitter_max_us;
    int64_t jitter_abs_sum_us;
    uint64_t jitter_abs_samples;
    int64_t rssi_sum;
    int rssi_min;
    int rssi_max;
    uint32_t next_sequence;
    bool has_sequence;
} dual_end_audio_power_stats_t;

static dual_end_audio_pcm_sink_t s_pcm_sink;
static void *s_pcm_sink_ctx;
static dual_end_input_event_sink_t s_input_event_sink;
static void *s_input_event_sink_ctx;
static void *s_g722_encoder[DUAL_END_AUDIO_CHANNELS];
static void *s_g722_decoder[DUAL_END_AUDIO_CHANNELS];
_Alignas(16) static int16_t s_encode_channel_pcm[DUAL_END_AUDIO_CHANNELS][DUAL_END_AUDIO_FRAME_SAMPLES];
_Alignas(16) static int16_t s_decode_channel_pcm[DUAL_END_AUDIO_CHANNELS][DUAL_END_AUDIO_FRAME_SAMPLES];
_Alignas(16) static int16_t s_decoded_interleaved_pcm[DUAL_END_AUDIO_FRAME_SAMPLE_VALUES];
_Static_assert(DUAL_END_AUDIO_CHANNEL_FRAME_BYTES % 16 == 0, "codec PCM row alignment");
static dual_end_audio_packet_t s_tx_packet;
static uint32_t s_tx_sequence;
static uint8_t s_tx_codec = DUAL_END_AUDIO_CODEC_G722;
static uint8_t s_tx_layout = DUAL_END_LAYOUT_STEREO;
static dual_end_audio_encode_stats_t s_encoding_stats;
static uint32_t s_encode_max_us;
static uint32_t s_last_encoded_bytes;
static bool s_encode_attempt_pending, s_encode_attempt_success;
static dual_end_owner_outcome_t s_encode_attempt_outcome = DUAL_END_OUTCOME_UNCLASSIFIED;
static uint8_t s_encode_attempt_codec;
static uint32_t s_encode_attempt_session;
/* The PCM sender publishes one timing snapshot per admitted PCM call. The
 * microphone owner consumes it immediately after that callback, without an
 * extra sender-mutex acquisition. The generations prevent a callback that
 * bypasses the sender (suspend/recovery/OTA) from reusing stale timings. */
static dual_end_audio_owner_timing_t s_last_owner_timing;
static uint32_t s_owner_timing_produced_generation;
static uint32_t s_owner_timing_consumed_generation;
static uint32_t s_tx_capture_sample_index;
static int8_t s_last_tx_power_qdbm = INT8_MIN;
static volatile bool s_ota_paused;
static bool s_channel_paused;
static dual_end_espnow_link_stats_t s_last_tx_log_stats;
static int64_t s_last_tx_log_us;
static dual_end_audio_power_stats_t s_segment_stats;
static dual_end_audio_power_stats_t s_total_stats;
static int8_t s_current_segment_power_qdbm = INT8_MIN;
static uint32_t s_segment_index;

typedef union {
    dual_end_audio_session_packet_t session;
    dual_end_audio_nack_packet_t nack;
} audio_recovery_control_t;

typedef struct {
    audio_recovery_control_t control;
    int64_t received_us;
} audio_recovery_control_event_t;

typedef struct {
    uint32_t sessions;
    uint32_t begins;
    uint32_t acks;
    uint32_t resync;
    uint32_t nack;
    uint32_t repair_queued;
    uint32_t repair_submitted;
    uint32_t repair_cancelled;
    uint32_t repair_mac_ok;
    uint32_t repair_mac_fail;
    uint32_t cache_miss;
    uint32_t cache_expired;
    uint32_t wrong_session;
    uint32_t duplicate_nack;
    uint32_t budget_drop;
    uint32_t queue_drop;
    uint32_t waiting_frames;
    uint32_t nack_deadline_expired;
} audio_recovery_tx_stats_t;

static SemaphoreHandle_t s_sender_mutex;
static QueueHandle_t s_recovery_queue;
/* The dual-mic cache is almost 12 KiB and is never used for DMA.  Keep it in
 * PSRAM so LCD and Wi-Fi retain their contiguous internal DMA headroom. */
static audio_recovery_tx_t *s_recovery_cache;
static audio_recovery_tx_stats_t s_recovery_stats;
static uint32_t s_session_id;
static uint32_t s_session_first_sequence;
static bool s_session_ready;
static bool s_sender_link_connected;
static int64_t s_session_last_begin_us;
static int64_t s_session_gate_open_us;
static uint8_t s_pending_input_flags;
static volatile uint32_t s_recovery_rx_queue_drop;

static uint32_t audio_payload_crc32(const uint8_t *payload, size_t len)
{
    return esp_rom_crc32_le(UINT32_MAX, payload, len) ^ UINT32_MAX;
}

static esp_err_t ensure_g722_encoder(void)
{
    bool initialized = false;
    for (size_t channel = 0; channel < DUAL_END_AUDIO_CHANNELS; channel++) {
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
        if (err != ESP_AUDIO_ERR_OK && s_g722_encoder[channel]) {
            esp_g722_enc_close(s_g722_encoder[channel]);
            s_g722_encoder[channel] = NULL;
        }
        ESP_RETURN_ON_FALSE(err == ESP_AUDIO_ERR_OK && s_g722_encoder[channel],
                            ESP_FAIL, TAG, "open G722 encoder ch=%u err=%d",
                            (unsigned)channel, err);
        initialized = true;

        int in_size = 0;
        int out_size = 0;
        err = esp_g722_enc_get_frame_size(s_g722_encoder[channel], &in_size, &out_size);
        if (err != ESP_AUDIO_ERR_OK ||
            in_size != DUAL_END_AUDIO_CHANNEL_FRAME_BYTES ||
            out_size != DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES) {
            /* A handle with unchecked geometry must never survive a failed
             * prepare and later be mistaken for a successfully prepared one. */
            esp_g722_enc_close(s_g722_encoder[channel]);
            s_g722_encoder[channel] = NULL;
            ESP_LOGE(TAG, "G722 encoder frame size ch=%u err=%d in=%d out=%d",
                     (unsigned)channel, err, in_size, out_size);
            return ESP_FAIL;
        }
    }

    if (initialized) {
        ESP_LOGI(TAG,
                 "stereo G722 encoders ready: channels=%d bitrate/ch=%d frame_in/ch=%d frame_out/ch=%d",
                 DUAL_END_AUDIO_CHANNELS,
                 DUAL_END_AUDIO_G722_BITRATE_BPS,
                 DUAL_END_AUDIO_CHANNEL_FRAME_BYTES,
                 DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES);
    }
    return ESP_OK;
}

static esp_err_t ensure_g722_decoder(void)
{
    bool initialized = false;
    for (size_t channel = 0; channel < DUAL_END_AUDIO_CHANNELS; channel++) {
        if (s_g722_decoder[channel]) {
            continue;
        }

        esp_g722_dec_cfg_t cfg = {
            .sample_rate = DUAL_END_AUDIO_SAMPLE_RATE_HZ,
            .bitrate = DUAL_END_AUDIO_G722_BITRATE_BPS,
            .packed = true,
        };

        esp_audio_err_t err = esp_g722_dec_open(&cfg, sizeof(cfg), &s_g722_decoder[channel]);
        ESP_RETURN_ON_FALSE(err == ESP_AUDIO_ERR_OK && s_g722_decoder[channel],
                            ESP_FAIL, TAG, "open G722 decoder ch=%u err=%d",
                            (unsigned)channel, err);
        initialized = true;
    }
    if (initialized) {
        ESP_LOGI(TAG, "stereo G722 decoders ready: channels=%d bitrate/ch=%d packed=true",
                 DUAL_END_AUDIO_CHANNELS, DUAL_END_AUDIO_G722_BITRATE_BPS);
    }
    return ESP_OK;
}

static esp_err_t g722_encode_frame(const int16_t *pcm, uint8_t *payload,
    size_t payload_len, uint32_t *encoded_bytes, bool dual_mono,
    uint32_t *process_calls)
{
    if (process_calls) *process_calls = 0;
    if (encoded_bytes) *encoded_bytes = 0;
    ESP_RETURN_ON_FALSE(pcm && payload && encoded_bytes, ESP_ERR_INVALID_ARG,
                        TAG, "G722 argument is null");
    if (dual_mono) {
        ESP_RETURN_ON_FALSE(dual_end_pcm_extract_dual_mono(pcm,
            s_encode_channel_pcm[0], DUAL_END_AUDIO_FRAME_SAMPLES),
            ESP_ERR_INVALID_ARG, TAG, "G722 owner dual-mono layout mismatch");
    } else {
        dual_end_pcm_split(pcm, s_encode_channel_pcm[0], s_encode_channel_pcm[1],
                            DUAL_END_AUDIO_FRAME_SAMPLES);
    }
    ESP_RETURN_ON_ERROR(ensure_g722_encoder(), TAG, "ensure G722 encoder");
    ESP_RETURN_ON_FALSE(payload_len >= DUAL_END_AUDIO_G722_FRAME_BYTES,
                        ESP_ERR_INVALID_SIZE, TAG, "G722 output buffer too small");

    const size_t channels_to_encode = dual_mono ? 1 : DUAL_END_AUDIO_CHANNELS;
    for (size_t channel = 0; channel < channels_to_encode; channel++) {
        esp_audio_enc_in_frame_t in_frame = {
            .buffer = (uint8_t *)s_encode_channel_pcm[channel],
            .len = DUAL_END_AUDIO_CHANNEL_FRAME_BYTES,
        };
        esp_audio_enc_out_frame_t out_frame = {
            .buffer = payload + channel * DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES,
            .len = DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES,
        };

        if (process_calls) ++*process_calls;
        esp_audio_err_t err = esp_g722_enc_process(s_g722_encoder[channel],
                                                   &in_frame,
                                                   &out_frame);
        ESP_RETURN_ON_FALSE(err == ESP_AUDIO_ERR_OK &&
                                out_frame.encoded_bytes == DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES,
                            ESP_FAIL, TAG,
                            "G722 encode ch=%u err=%d encoded=%" PRIu32,
                            (unsigned)channel, err, out_frame.encoded_bytes);
    }
    if (dual_mono) {
        memcpy(payload + DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES, payload,
               DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES);
    }

    *encoded_bytes = DUAL_END_AUDIO_G722_FRAME_BYTES;
    return ESP_OK;
}

static esp_err_t g722_decode_frame(const uint8_t *payload, size_t encoded_bytes, int16_t *pcm, size_t pcm_bytes)
{
    ESP_RETURN_ON_ERROR(ensure_g722_decoder(), TAG, "ensure G722 decoder");
    ESP_RETURN_ON_FALSE(encoded_bytes == DUAL_END_AUDIO_G722_FRAME_BYTES &&
                            pcm_bytes >= DUAL_END_AUDIO_FRAME_BYTES,
                        ESP_ERR_INVALID_SIZE, TAG, "invalid stereo G722 frame size");

    for (size_t channel = 0; channel < DUAL_END_AUDIO_CHANNELS; channel++) {
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
                                                  &raw,
                                                  &out_frame,
                                                  &dec_info);
        ESP_RETURN_ON_FALSE(err == ESP_AUDIO_ERR_OK &&
                                raw.consumed == DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES &&
                                out_frame.decoded_size == DUAL_END_AUDIO_CHANNEL_FRAME_BYTES,
                            ESP_FAIL, TAG,
                            "G722 decode ch=%u err=%d consumed=%" PRIu32
                            " decoded=%" PRIu32 " needed=%" PRIu32,
                            (unsigned)channel, err, raw.consumed,
                            out_frame.decoded_size, out_frame.needed_size);
    }

    dual_end_pcm_interleave(s_decode_channel_pcm[0], s_decode_channel_pcm[1], pcm, DUAL_END_AUDIO_FRAME_SAMPLES);
    return ESP_OK;
}

static void reset_stats(dual_end_audio_power_stats_t *stats)
{
    memset(stats, 0, sizeof(*stats));
    stats->rssi_min = 127;
    stats->rssi_max = -127;
}

static void update_stats(dual_end_audio_power_stats_t *stats,
                         uint32_t sequence,
                         bool crc_error,
                         int rssi)
{
    int64_t now_us = esp_timer_get_time();

    if (!stats->has_sequence) {
        stats->has_sequence = true;
        stats->next_sequence = sequence + 1;
    } else if (sequence == stats->next_sequence) {
        stats->next_sequence++;
    } else if (sequence > stats->next_sequence) {
        stats->lost_frames += sequence - stats->next_sequence;
        stats->loss_events++;
        stats->next_sequence = sequence + 1;
    } else if ((stats->next_sequence - sequence) == 1) {
        stats->duplicate_frames++;
    } else {
        stats->out_of_order_frames++;
    }

    if (stats->last_rx_us != 0) {
        int32_t jitter_us = (int32_t)((now_us - stats->last_rx_us) -
                                      (DUAL_END_AUDIO_FRAME_MS * 1000));
        if (stats->jitter_abs_samples == 0 || jitter_us < stats->jitter_min_us) {
            stats->jitter_min_us = jitter_us;
        }
        if (stats->jitter_abs_samples == 0 || jitter_us > stats->jitter_max_us) {
            stats->jitter_max_us = jitter_us;
        }
        stats->jitter_abs_sum_us += jitter_us < 0 ? -jitter_us : jitter_us;
        stats->jitter_abs_samples++;
    }
    stats->last_rx_us = now_us;
    stats->frames_rx++;
    if (crc_error) {
        stats->crc_error_frames++;
    }
    stats->rssi_sum += rssi;
    if (rssi < stats->rssi_min) {
        stats->rssi_min = rssi;
    }
    if (rssi > stats->rssi_max) {
        stats->rssi_max = rssi;
    }
}

static void log_stats(const char *prefix,
                      uint32_t segment_index,
                      int8_t tx_power_qdbm,
                      const dual_end_audio_power_stats_t *stats)
{
    if (stats->frames_rx == 0) {
        return;
    }

    uint64_t expected = stats->frames_rx + stats->lost_frames;
    uint64_t loss_ppm = expected ? (stats->lost_frames * 1000000ULL) / expected : 0;
    uint64_t crc_ppm = expected ? (stats->crc_error_frames * 1000000ULL) / expected : 0;
    int64_t jitter_avg_abs = stats->jitter_abs_samples ?
                             stats->jitter_abs_sum_us / (int64_t)stats->jitter_abs_samples : 0;
    int rssi_avg = (int)(stats->rssi_sum / (int64_t)stats->frames_rx);

    ESP_LOGI(TAG,
             "%s segment=%" PRIu32 " tx_power_qdbm=%d rx=%" PRIu64
             " expected=%" PRIu64 " lost=%" PRIu64 " loss_events=%" PRIu64
             " dup=%" PRIu64 " ooo=%" PRIu64 " crc_err=%" PRIu64
             " loss=%" PRIu64 "ppm crc=%" PRIu64 "ppm"
             " jitter_us[min=%" PRId32 " max=%" PRId32 " avg_abs=%" PRId64 "]"
             " rssi[min=%d max=%d avg=%d]",
             prefix,
             segment_index,
             tx_power_qdbm,
             stats->frames_rx,
             expected,
             stats->lost_frames,
             stats->loss_events,
             stats->duplicate_frames,
             stats->out_of_order_frames,
             stats->crc_error_frames,
             loss_ppm,
             crc_ppm,
             stats->jitter_min_us,
             stats->jitter_max_us,
             jitter_avg_abs,
             stats->rssi_min,
             stats->rssi_max,
             rssi_avg);
}

static void sender_reset_encoders_locked(void)
{
    for (size_t channel = 0; channel < DUAL_END_AUDIO_CHANNELS; ++channel) {
        if (s_g722_encoder[channel] != NULL) {
            if (esp_g722_enc_reset(s_g722_encoder[channel]) != ESP_AUDIO_ERR_OK) {
                esp_g722_enc_close(s_g722_encoder[channel]);
                s_g722_encoder[channel] = NULL;
            }
        }
    }
}

static void sender_invalidate_session_locked(void)
{
    if (s_encoding_stats.frames || s_encoding_stats.failures) {
        ESP_LOGI(TAG, "codec_session_summary session=%08" PRIx32
                 " codec=%u profile=%s layout=%u encoded_frames=%" PRIu64
                 " process_calls=%" PRIu64 " reused_frames=%" PRIu64
                 " failures=%" PRIu64 " failed_process_calls=%" PRIu64
                 " layout_mismatches=%" PRIu64 " encode_max_us=%" PRIu32
                 " encoded_bytes_last=%" PRIu32 " opus_fec_enabled=%s",
                 s_session_id, s_tx_codec, codec_profile_name(s_tx_codec),
                 s_tx_layout, s_encoding_stats.frames,
                 s_encoding_stats.process_calls, s_encoding_stats.reused_frames,
                 s_encoding_stats.failures, s_encoding_stats.failed_process_calls,
                 s_encoding_stats.layout_mismatches, s_encode_max_us,
                 s_last_encoded_bytes, codec_fec_state(s_tx_codec));
    }
    memset(&s_encoding_stats, 0, sizeof(s_encoding_stats));
    /* Timing and last-packet size belong to this codec/session, not the whole
     * period since the last manual codec selection. Keep the old values in the
     * summary above, then start the next session with an empty measurement. */
    s_encode_max_us = 0;
    s_last_encoded_bytes = 0;
    s_session_ready = false;
    s_session_id = 0;
    s_session_first_sequence = s_tx_sequence;
    s_session_last_begin_us = 0;
    s_tx_capture_sample_index = 0;
    audio_recovery_tx_session_reset(s_recovery_cache, 0);
    dual_end_espnow_link_discard_audio();
    sender_reset_encoders_locked();
    dual_end_opus_encoder_reset();
}

static void sender_new_session_locked(void)
{
    uint32_t previous = s_session_id;
    sender_invalidate_session_locked();
    do {
        s_session_id = esp_random();
    } while (s_session_id == 0 || s_session_id == previous);
    s_session_first_sequence = s_tx_sequence;
    audio_recovery_tx_session_reset(s_recovery_cache, s_session_id);
    s_recovery_stats.sessions++;
    ESP_LOGI(TAG, "audio session new id=%08" PRIx32 " first=%" PRIu32,
             s_session_id, s_session_first_sequence);
}

static bool sender_session_available_locked(void)
{
    return s_sender_link_connected && !s_ota_paused && !s_channel_paused;
}

static void sender_update_session_gate_locked(bool connected, bool ota_paused,
                                              bool channel_paused,
                                              int64_t now_us)
{
    const audio_session_gate_action_t action = audio_session_gate_transition(
        s_sender_link_connected, s_ota_paused, s_channel_paused,
        connected, ota_paused, channel_paused);
    s_sender_link_connected = connected;
    s_ota_paused = ota_paused;
    s_channel_paused = channel_paused;
    if (action == AUDIO_SESSION_GATE_INVALIDATE) {
        s_session_gate_open_us = 0;
        sender_invalidate_session_locked();
    } else if (action == AUDIO_SESSION_GATE_START) {
        s_session_gate_open_us = now_us;
        sender_new_session_locked();
    }
}

static void sender_send_begin_locked(int64_t now_us)
{
    if (!sender_session_available_locked() || s_session_id == 0 ||
        s_session_ready ||
        (s_session_last_begin_us != 0 &&
         now_us - s_session_last_begin_us < DUAL_END_AUDIO_SESSION_RETRY_US)) {
        return;
    }
    dual_end_audio_session_packet_t begin = {
        .packet = {
            .protocol_version = SCEPTER_PROTOCOL_VERSION,
            .source_role = SCEPTER_ROLE_SCEPTER,
            .message_type = SCEPTER_MSG_AUDIO_SESSION,
            .sequence = s_session_first_sequence,
            .payload_len = sizeof(begin) - sizeof(begin.packet),
        },
        .version = DUAL_END_AUDIO_RECOVERY_VERSION,
        .command = DUAL_END_AUDIO_SESSION_BEGIN,
        .session_id = s_session_id,
        .first_sequence = s_session_first_sequence,
    };
    s_session_last_begin_us = now_us;
    if (dual_end_espnow_link_send_audio_control(&begin, sizeof(begin)) == ESP_OK) {
        s_recovery_stats.begins++;
    } else {
        s_recovery_stats.queue_drop++;
    }
}

static void sender_handle_nack_locked(const dual_end_audio_nack_packet_t *nack,
                                      int64_t received_us, int64_t now_us)
{
    s_recovery_stats.nack++;
    if (!s_session_ready || nack->session_id != s_session_id) {
        s_recovery_stats.wrong_session++;
        return;
    }
    const int64_t deadline_us = received_us + (int64_t)nack->remaining_ms * 1000 -
                                DUAL_END_AUDIO_NACK_TRANSIT_RESERVE_US;
    if (now_us < received_us ||
        deadline_us - now_us < DUAL_END_AUDIO_REPAIR_SEND_RESERVE_US) {
        s_recovery_stats.nack_deadline_expired++;
        return;
    }
    for (unsigned bit = 0; bit < DUAL_END_AUDIO_RECOVERY_WINDOW; ++bit) {
        if ((nack->missing_mask & (1U << bit)) == 0) {
            continue;
        }
        const uint32_t sequence = nack->base_sequence + bit;
        const audio_recovery_tx_entry_t *entry = NULL;
        uint32_t reservation_id = 0;
        const audio_recovery_tx_result_t result = audio_recovery_tx_reserve(
            s_recovery_cache, nack->session_id, sequence, now_us,
            &reservation_id, &entry);
        if (result != AUDIO_RECOVERY_TX_HIT) {
            if (result == AUDIO_RECOVERY_TX_EXPIRED) {
                s_recovery_stats.cache_expired++;
            } else if (result == AUDIO_RECOVERY_TX_LIMIT ||
                       result == AUDIO_RECOVERY_TX_TOO_SOON ||
                       result == AUDIO_RECOVERY_TX_PENDING) {
                s_recovery_stats.duplicate_nack++;
            } else if (result == AUDIO_RECOVERY_TX_BUDGET) {
                s_recovery_stats.budget_drop++;
            } else {
                s_recovery_stats.cache_miss++;
            }
            continue;
        }
        dual_end_audio_repair_packet_t repair = {
            .packet = {
                .protocol_version = SCEPTER_PROTOCOL_VERSION,
                .source_role = SCEPTER_ROLE_SCEPTER,
                .message_type = SCEPTER_MSG_AUDIO_REPAIR,
                .sequence = sequence,
                .payload_len = sizeof(repair) - sizeof(repair.packet),
            },
            .session_id = s_session_id,
            .audio = entry->audio,
        };
        memcpy(repair.encoded, entry->encoded, sizeof(repair.encoded));
        repair.audio.payload_crc32 = audio_payload_crc32(repair.encoded,
                                                         sizeof(repair.encoded));
        esp_err_t ret = dual_end_espnow_link_send_audio_repair(
            &repair, sizeof(repair), entry->created_us, reservation_id, deadline_us);
        if (ret == ESP_OK) {
            s_recovery_stats.repair_queued++;
        } else {
            (void)audio_recovery_tx_complete(s_recovery_cache, nack->session_id,
                                             sequence, reservation_id);
            s_recovery_stats.queue_drop++;
        }
    }
}

bool dual_end_audio_repair_tx_prepare(uint32_t session_id, uint32_t sequence,
                                      uint32_t reservation_id, int64_t now_us)
{
    if (s_sender_mutex == NULL) {
        return false;
    }
    xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
    now_us = esp_timer_get_time();
    if (!sender_session_available_locked() || !s_session_ready ||
        !audio_recovery_tx_prepare(s_recovery_cache, session_id, sequence,
                                   reservation_id, now_us)) {
        if (audio_recovery_tx_complete(s_recovery_cache, session_id, sequence,
                                       reservation_id)) {
            s_recovery_stats.repair_cancelled++;
        }
        xSemaphoreGive(s_sender_mutex);
        return false;
    }
    return true;
}

void dual_end_audio_repair_tx_submitted(uint32_t session_id, uint32_t sequence,
                                        uint32_t reservation_id, bool accepted,
                                        int64_t now_us)
{
    if (accepted) {
        if (audio_recovery_tx_accepted(s_recovery_cache, session_id, sequence,
                                       reservation_id, now_us)) {
            s_recovery_stats.repair_submitted++;
        }
    } else if (audio_recovery_tx_complete(s_recovery_cache, session_id, sequence,
                                          reservation_id)) {
        s_recovery_stats.repair_cancelled++;
    }
    xSemaphoreGive(s_sender_mutex);
}

void dual_end_audio_repair_tx_complete(uint32_t session_id, uint32_t sequence,
                                       uint32_t reservation_id, bool submitted,
                                       bool success, int64_t now_us)
{
    (void)now_us;
    if (s_sender_mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
    if (audio_recovery_tx_complete(s_recovery_cache, session_id, sequence,
                                   reservation_id)) {
        if (!submitted) {
            s_recovery_stats.repair_cancelled++;
        } else if (success) {
            s_recovery_stats.repair_mac_ok++;
        } else {
            s_recovery_stats.repair_mac_fail++;
        }
    }
    xSemaphoreGive(s_sender_mutex);
}

static void sender_process_control_locked(const audio_recovery_control_event_t *event,
                                          int64_t now_us)
{
    const audio_recovery_control_t *control = &event->control;
    if (control->session.packet.message_type == SCEPTER_MSG_AUDIO_SESSION) {
        const bool cold_resync =
            control->session.command == DUAL_END_AUDIO_SESSION_RESYNC &&
            control->session.session_id == 0 &&
            control->session.first_sequence == 0;
        if (cold_resync) {
            if (s_session_ready) {
                s_recovery_stats.resync++;
                sender_new_session_locked();
            }
        } else if (control->session.session_id == s_session_id &&
                   control->session.first_sequence == s_session_first_sequence &&
                   s_session_id != 0) {
            if (control->session.command == DUAL_END_AUDIO_SESSION_ACK) {
                if (!s_session_ready) {
                    s_session_ready = true;
                    s_recovery_stats.acks++;
                    ESP_LOGI(TAG, "audio session ready id=%08" PRIx32,
                             s_session_id);
                }
            } else {
                s_recovery_stats.resync++;
                sender_new_session_locked();
            }
        } else {
            s_recovery_stats.wrong_session++;
        }
    } else {
        sender_handle_nack_locked(&control->nack, event->received_us, now_us);
    }
}

static void sender_recovery_task(void *arg)
{
    (void)arg;
    audio_recovery_control_event_t event;
    int64_t last_log_us = 0;
    while (true) {
        const bool received =
            xQueueReceive(s_recovery_queue, &event, pdMS_TO_TICKS(2)) == pdTRUE;
        const bool connected = dual_end_espnow_link_is_connected();
        const bool channel_paused = dual_end_espnow_link_is_audio_paused();
        xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
        const int64_t now_us = esp_timer_get_time();
        sender_update_session_gate_locked(connected, s_ota_paused,
                                          channel_paused, now_us);
        if (sender_session_available_locked() && s_session_id == 0) {
            sender_new_session_locked();
        }
        if (received && sender_session_available_locked() &&
            event.received_us >= s_session_gate_open_us) {
            sender_process_control_locked(&event, now_us);
        }
        sender_send_begin_locked(now_us);
        if (now_us - last_log_us >= 10000000LL) {
            ESP_LOGI(TAG,
                     "recovery_tx session=%08" PRIx32 " ready=%d"
                     " sessions/begin/ack/resync=%" PRIu32 "/%" PRIu32
                     "/%" PRIu32 "/%" PRIu32
                     " nack=%" PRIu32 " wrong_session=%" PRIu32
                     " repair[q/sub/ok/fail/cancel]=%" PRIu32
                     "/%" PRIu32 "/%" PRIu32 "/%" PRIu32 "/%" PRIu32
                     " cache[miss/expired/dup/budget]=%" PRIu32 "/%" PRIu32
                     "/%" PRIu32 "/%" PRIu32 " queue_drop=%" PRIu32
                     " rx_queue_drop=%" PRIu32 " deadline_drop=%" PRIu32,
                     s_session_id, s_session_ready ? 1 : 0,
                     s_recovery_stats.sessions, s_recovery_stats.begins,
                     s_recovery_stats.acks, s_recovery_stats.resync,
                     s_recovery_stats.nack, s_recovery_stats.wrong_session,
                     s_recovery_stats.repair_queued,
                     s_recovery_stats.repair_submitted,
                     s_recovery_stats.repair_mac_ok,
                     s_recovery_stats.repair_mac_fail,
                     s_recovery_stats.repair_cancelled,
                     s_recovery_stats.cache_miss,
                     s_recovery_stats.cache_expired,
                     s_recovery_stats.duplicate_nack,
                     s_recovery_stats.budget_drop,
                     s_recovery_stats.queue_drop,
                     s_recovery_rx_queue_drop,
                     s_recovery_stats.nack_deadline_expired);
            last_log_us = now_us;
        }
        xSemaphoreGive(s_sender_mutex);
    }
}

bool dual_end_audio_sender_queue_control(const uint8_t *data, int len)
{
    if (data == NULL || len < (int)sizeof(dual_end_packet_header_t)) {
        return false;
    }
    dual_end_packet_header_t header;
    memcpy(&header, data, sizeof(header));
    if (header.message_type != SCEPTER_MSG_AUDIO_SESSION &&
        header.message_type != SCEPTER_MSG_AUDIO_NACK) {
        return false;
    }
    if (len != (int)sizeof(audio_recovery_control_t) || s_recovery_queue == NULL ||
        !dual_end_audio_recovery_header_valid(&header, (size_t)len,
                                              header.message_type,
                                              SCEPTER_ROLE_DONGLE)) {
        return true;
    }
    audio_recovery_control_t control;
    memcpy(&control, data, sizeof(control));
    if (header.message_type == SCEPTER_MSG_AUDIO_SESSION) {
        const bool cold_resync =
            control.session.command == DUAL_END_AUDIO_SESSION_RESYNC &&
            control.session.session_id == 0 &&
            control.session.first_sequence == 0;
        if (control.session.version != DUAL_END_AUDIO_RECOVERY_VERSION ||
            control.session.reserved != 0 ||
            (control.session.session_id == 0 && !cold_resync) ||
            (control.session.command != DUAL_END_AUDIO_SESSION_ACK &&
             control.session.command != DUAL_END_AUDIO_SESSION_RESYNC)) {
            return true;
        }
    } else if (control.nack.version != DUAL_END_AUDIO_RECOVERY_VERSION ||
               control.nack.remaining_ms == 0 ||
               control.nack.remaining_ms > DUAL_END_AUDIO_PLAYOUT_DELAY_US / 1000 ||
               control.nack.session_id == 0 || control.nack.missing_mask == 0) {
        return true;
    }
    const audio_recovery_control_event_t event = {
        .control = control,
        .received_us = esp_timer_get_time(),
    };
    if (xQueueSend(s_recovery_queue, &event, 0) != pdTRUE) {
        s_recovery_rx_queue_drop++;
    }
    return true;
}

bool dual_end_audio_sender_handle_retry_request(const uint8_t *data, int len)
{
    return dual_end_audio_sender_queue_control(data, len);
}

void dual_end_audio_sender_note_link_state(bool connected)
{
    if (s_sender_mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
    sender_update_session_gate_locked(connected, s_ota_paused,
                                      dual_end_espnow_link_is_audio_paused(),
                                      esp_timer_get_time());
    xSemaphoreGive(s_sender_mutex);
}

esp_err_t dual_end_audio_sender_start(void)
{
    if (s_sender_mutex != NULL && s_recovery_queue != NULL &&
        s_recovery_cache != NULL) {
        return ESP_OK;
    }
    if (s_recovery_cache == NULL) {
        s_recovery_cache = heap_caps_calloc(1, sizeof(*s_recovery_cache),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        ESP_RETURN_ON_FALSE(s_recovery_cache != NULL, ESP_ERR_NO_MEM, TAG,
                            "allocate audio recovery cache in PSRAM");
    }
    s_tx_sequence = esp_random();
    s_tx_capture_sample_index = 0;
    s_last_tx_power_qdbm = INT8_MIN;
    s_sender_link_connected = false;
    memset(&s_last_owner_timing, 0, sizeof(s_last_owner_timing));
    s_owner_timing_produced_generation = 0;
    s_owner_timing_consumed_generation = 0;
    s_sender_mutex = xSemaphoreCreateMutex();
    s_recovery_queue = xQueueCreate(8, sizeof(audio_recovery_control_event_t));
    ESP_RETURN_ON_FALSE(s_sender_mutex != NULL && s_recovery_queue != NULL,
                        ESP_ERR_NO_MEM, TAG, "allocate audio recovery controls");
    audio_recovery_tx_reset(s_recovery_cache, 0);
    ESP_RETURN_ON_ERROR(ensure_g722_encoder(), TAG, "init G722 encoder");
    memset(&s_last_tx_log_stats, 0, sizeof(s_last_tx_log_stats));
    dual_end_espnow_link_get_stats(&s_last_tx_log_stats);
    s_last_tx_log_us = esp_timer_get_time();
    /* Internal RAM is exhausted by the PDM/LCD/ESP-NOW path; keep this light
     * task's stack in PSRAM. */
    TaskHandle_t recovery_handle = NULL;
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCoreWithCaps(sender_recovery_task, "audio_recover", 5120,
                                    NULL, 6, &recovery_handle, DUAL_END_COMM_TASK_CORE,
                                    MALLOC_CAP_SPIRAM) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create audio recovery task");
    ESP_LOGI(TAG,
             "audio sender ready: stereo G722 %d Hz %d-bit %d ms, recovery v=%d",
             DUAL_END_AUDIO_SAMPLE_RATE_HZ, DUAL_END_AUDIO_BITS_PER_SAMPLE,
             DUAL_END_AUDIO_FRAME_MS, DUAL_END_AUDIO_RECOVERY_VERSION);
    return ESP_OK;
}

void dual_end_audio_sender_set_ota_paused(bool paused)
{
    if (s_sender_mutex == NULL) {
        s_ota_paused = paused;
        return;
    }
    xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
    sender_update_session_gate_locked(s_sender_link_connected, paused,
                                      dual_end_espnow_link_is_audio_paused(),
                                      esp_timer_get_time());
    xSemaphoreGive(s_sender_mutex);
    ESP_LOGI(TAG, "audio sender %s for OTA", paused ? "paused" : "resumed");
}

/* Called only by the microphone owner at a 20 ms PCM boundary. Target
 * encoders are prepared before commit. A new recovery session clears queued
 * old codec frames, cached repairs and both encoder histories under one lock. */
esp_err_t dual_end_audio_sender_set_codec(uint8_t codec)
{
    if ((codec != DUAL_END_AUDIO_CODEC_G722 && !dual_end_audio_codec_is_opus(codec)) ||
        !s_sender_mutex) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
    if (codec == s_tx_codec) {
        xSemaphoreGive(s_sender_mutex);
        return ESP_OK;
    }
    esp_err_t ret = dual_end_audio_codec_is_opus(codec) ?
        dual_end_opus_encoder_prepare_for(codec) : ensure_g722_encoder();
    if (ret == ESP_OK) {
        const uint8_t previous = s_tx_codec;
        sender_new_session_locked();
        s_tx_codec = codec;
        ret = dual_end_audio_codec_is_opus(codec) ?
            dual_end_opus_encoder_prepare_for(codec) : ensure_g722_encoder();
        if (ret != ESP_OK) {
            s_tx_codec = previous;
            sender_new_session_locked();
        }
        ESP_LOGI(TAG, "codec applied=%u result=%s session=%08" PRIx32,
                 s_tx_codec, esp_err_to_name(ret), s_session_id);
    }
    xSemaphoreGive(s_sender_mutex);
    return ret;
}
esp_err_t dual_end_audio_sender_restart_processing(void)
{
    return dual_end_audio_sender_set_processing_layout(s_tx_layout);
}
esp_err_t dual_end_audio_sender_set_processing_layout(uint8_t layout)
{
    if (!s_sender_mutex) return ESP_ERR_INVALID_STATE;
    if (layout != DUAL_END_LAYOUT_STEREO && layout != DUAL_END_LAYOUT_DUAL_MONO)
        return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
    /* Both histories reset on every processing-mode transition. A right encoder
     * left idle in dual mono must never resume old stereo history. Reset failure
     * closes the handle; prepare must successfully rebuild before owner commit. */
    sender_new_session_locked();
    const esp_err_t ret = dual_end_audio_codec_is_opus(s_tx_codec) ?
        dual_end_opus_encoder_prepare_for(s_tx_codec) : ensure_g722_encoder();
    if (ret == ESP_OK) s_tx_layout = layout;
    xSemaphoreGive(s_sender_mutex);
    return ret;
}
void dual_end_audio_sender_get_encode_stats(dual_end_audio_encode_stats_t *stats)
{
    if (!stats) return;
    memset(stats, 0, sizeof(*stats));
    if (!s_sender_mutex) return;
    xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
    *stats = s_encoding_stats;
    stats->session_id = s_session_id;
    stats->layout = s_tx_layout;
    xSemaphoreGive(s_sender_mutex);
}

bool dual_end_audio_sender_copy_last_owner_timing(dual_end_audio_owner_timing_t *timing)
{
    if (!timing) return false;
    memset(timing, 0, sizeof(*timing));
    /* Owner-only API: recovery/control tasks never write this snapshot.
     * Consume exactly one completed sender admission, so a later callback
     * that did not invoke the PCM sender cannot attribute old stage timings. */
    if (!s_last_owner_timing.valid ||
        s_owner_timing_produced_generation == s_owner_timing_consumed_generation)
        return false;
    *timing = s_last_owner_timing;
    s_owner_timing_consumed_generation = s_owner_timing_produced_generation;
    s_last_owner_timing.valid = false;
    return true;
}

/* The outcome is maintained alongside the one-shot timing snapshot while the
 * sender mutex is held.  The PCM callback worker is its only consumer, after
 * the send function has released that mutex, so this handoff never blocks the
 * real-time microphone owner. */
static dual_end_owner_outcome_t sender_current_owner_outcome(void)
{
    if (s_encode_attempt_outcome != DUAL_END_OUTCOME_UNCLASSIFIED)
        return s_encode_attempt_outcome;
    if (s_encode_attempt_pending)
        return s_encode_attempt_success ? DUAL_END_OUTCOME_OK :
            DUAL_END_OUTCOME_OPERATION_FAILED;
    return DUAL_END_OUTCOME_UNCLASSIFIED;
}

bool dual_end_audio_sender_take_owner_timing(dual_end_audio_owner_timing_t *timing)
{
    if (!dual_end_audio_sender_copy_last_owner_timing(timing)) return false;
    timing->outcome=sender_current_owner_outcome();
    s_encode_attempt_pending=false;
    s_encode_attempt_outcome=DUAL_END_OUTCOME_UNCLASSIFIED;
    return true;
}
uint8_t dual_end_audio_sender_get_codec(void)
{
    if (!s_sender_mutex) return DUAL_END_AUDIO_CODEC_G722;
    xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
    const uint8_t codec = s_tx_codec;
    xSemaphoreGive(s_sender_mutex);
    return codec;
}

void dual_end_audio_sender_record_mic_work(int64_t elapsed_us)
{
    if (!s_sender_mutex) return;
    xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
    const uint8_t codec = s_encode_attempt_pending ? s_encode_attempt_codec : s_tx_codec;
    const uint32_t session = s_encode_attempt_pending ? s_encode_attempt_session : s_session_id;
    const dual_end_owner_outcome_t outcome=sender_current_owner_outcome();
    s_encode_attempt_pending = false;
    s_encode_attempt_outcome = DUAL_END_OUTCOME_UNCLASSIFIED;
    dual_end_runtime_record_outcome(DUAL_END_STAGE_MIC_PIPELINE, codec, session, elapsed_us, outcome);
    xSemaphoreGive(s_sender_mutex);
}

esp_err_t dual_end_audio_sender_send_pcm(const int16_t *pcm, size_t samples)
{
    return dual_end_audio_sender_send_pcm_with_hardware_and_analysis(
        pcm, samples, NULL, NULL);
}

esp_err_t dual_end_audio_sender_send_pcm_with_hardware(const int16_t *pcm,
                                                       size_t samples,
                                                       const dual_end_hid_input_payload_t *hardware_payload)
{
    return dual_end_audio_sender_send_pcm_with_hardware_and_analysis(
        pcm, samples, hardware_payload, NULL);
}

esp_err_t dual_end_audio_sender_send_pcm_with_hardware_and_analysis(
    const int16_t *pcm,
    size_t samples,
    const dual_end_hid_input_payload_t *hardware_payload,
    const dual_end_audio_analysis_metadata_t *analysis_metadata)
{
    if ((!pcm || samples != DUAL_END_AUDIO_FRAME_SAMPLE_VALUES) && s_sender_mutex) {
        xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
        s_encode_attempt_pending = false;
        s_encode_attempt_outcome = DUAL_END_OUTCOME_INVALID_REQUEST;
        memset(&s_last_owner_timing, 0, sizeof(s_last_owner_timing));
        xSemaphoreGive(s_sender_mutex);
    }
    ESP_RETURN_ON_FALSE(pcm, ESP_ERR_INVALID_ARG, TAG, "pcm is null");
    ESP_RETURN_ON_FALSE(samples == DUAL_END_AUDIO_FRAME_SAMPLE_VALUES,
                        ESP_ERR_INVALID_SIZE, TAG,
                        "pcm samples=%u expected=%u", (unsigned)samples,
                        DUAL_END_AUDIO_FRAME_SAMPLE_VALUES);
    ESP_RETURN_ON_FALSE(s_sender_mutex != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "audio sender is not started");

    const int64_t created_us = esp_timer_get_time();
    xSemaphoreTake(s_sender_mutex, portMAX_DELAY);
    const int64_t sender_locked_us = esp_timer_get_time();
    memset(&s_last_owner_timing, 0, sizeof(s_last_owner_timing));
    s_last_owner_timing.valid = true;
    ++s_owner_timing_produced_generation;
    s_last_owner_timing.mutex_wait_us = sender_locked_us <= created_us ? 0U :
        ((uint64_t)(sender_locked_us - created_us) > UINT32_MAX) ? UINT32_MAX :
        (uint32_t)(sender_locked_us - created_us);
    s_last_owner_timing.codec = s_tx_codec;
    s_last_owner_timing.session_id = s_session_id;
    /* Fresh admission evidence for this callback; no earlier frame's outcome
     * may leak into a callback that returns before the codec operation. */
    s_encode_attempt_pending = false;
    s_encode_attempt_outcome = DUAL_END_OUTCOME_UNCLASSIFIED;
    const bool connected = dual_end_espnow_link_is_connected();
    sender_update_session_gate_locked(connected, s_ota_paused,
                                      dual_end_espnow_link_is_audio_paused(),
                                      esp_timer_get_time());
    if (!sender_session_available_locked()) {
        s_encode_attempt_outcome = DUAL_END_OUTCOME_WAIT_SESSION;
        xSemaphoreGive(s_sender_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_session_id == 0) {
        sender_new_session_locked();
    }
    /* Attribute the attempt after admission has established its stream. */
    s_last_owner_timing.codec = s_tx_codec;
    s_last_owner_timing.session_id = s_session_id;
    /* One-shot input events may arrive while BEGIN/ACK is still in flight.
     * Keep them until a normal audio packet is admitted; repair packets never
     * contain or replay this state. */
    if (hardware_payload != NULL) {
        s_pending_input_flags |= hardware_payload->flags;
    }
    s_pending_input_flags |= dual_end_espnow_link_take_unsent_input_flags();
    sender_send_begin_locked(created_us);
    if (!s_session_ready) {
        s_encode_attempt_outcome = DUAL_END_OUTCOME_WAIT_ACK;
        s_recovery_stats.waiting_frames++;
        xSemaphoreGive(s_sender_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (!dual_end_audio_deadline_valid(esp_timer_get_time(), created_us)) {
        s_encode_attempt_outcome = DUAL_END_OUTCOME_DEADLINE;
        xSemaphoreGive(s_sender_mutex);
        return ESP_ERR_TIMEOUT;
    }

    const uint32_t sequence = s_tx_sequence;
    const int8_t tx_power_qdbm = SCEPTER_ESPNOW_TX_POWER_QDBM;
    if (tx_power_qdbm != s_last_tx_power_qdbm) {
        esp_err_t power_ret = dual_end_espnow_link_set_tx_power(tx_power_qdbm);
        if (power_ret != ESP_OK) {
            s_encode_attempt_outcome = DUAL_END_OUTCOME_POWER;
            xSemaphoreGive(s_sender_mutex);
            return power_ret;
        }
        s_last_tx_power_qdbm = tx_power_qdbm;
    }

    uint32_t encoded_bytes = 0;
    uint32_t process_calls = 0;
    const bool dual_mono = s_tx_layout == DUAL_END_LAYOUT_DUAL_MONO;
    memset(&s_tx_packet, 0, sizeof(s_tx_packet));
    const int64_t encode_started_us = esp_timer_get_time();
    s_last_owner_timing.encode_attempted = true;
#if DUAL_END_PCM_LEVEL_DIAG
    /* Diagnostic: level of the PCM handed to the encoder. */
    {
        static int64_t level_log_us;
        const int64_t level_now = esp_timer_get_time();
        if (level_now - level_log_us >= 2000000) {
            level_log_us = level_now;
            int32_t peak = 0;
            int64_t sum = 0;
            const size_t count = DUAL_END_AUDIO_FRAME_SAMPLES * DUAL_END_AUDIO_CHANNELS;
            for (size_t i = 0; i < count; ++i) {
                const int32_t v = pcm[i];
                const int32_t a = v < 0 ? -v : v;
                if (a > peak) peak = a;
                sum += (int64_t)v * v;
            }
            ESP_LOGI(TAG, "sender pcm peak=%d rms=%d", (int)peak,
                     (int)sqrt((double)sum / (double)count));
        }
    }
#endif
    esp_err_t encode_ret = dual_end_audio_codec_is_opus(s_tx_codec) ?
        dual_end_opus_encode_with_layout(pcm, s_tx_packet.payload.audio,
                            sizeof(s_tx_packet.payload.audio), &encoded_bytes,
                            dual_mono, &process_calls) :
        g722_encode_frame(pcm, s_tx_packet.payload.audio,
                          sizeof(s_tx_packet.payload.audio), &encoded_bytes,
                          dual_mono, &process_calls);
    s_encoding_stats.process_calls += process_calls;
    if (encode_ret == ESP_OK) {
        ++s_encoding_stats.frames;
        if (dual_mono) ++s_encoding_stats.reused_frames;
    } else {
        ++s_encoding_stats.failures;
        s_encoding_stats.failed_process_calls += process_calls;
        if (dual_mono && encode_ret == ESP_ERR_INVALID_ARG)
            ++s_encoding_stats.layout_mismatches;
    }
    s_last_encoded_bytes = encode_ret == ESP_OK ? encoded_bytes : 0;
    const int64_t encode_us = esp_timer_get_time() - encode_started_us;
    s_last_owner_timing.encode_us = encode_us <= 0 ? 0U :
        ((uint64_t)encode_us > UINT32_MAX ? UINT32_MAX : (uint32_t)encode_us);
    s_last_owner_timing.encode_ok = encode_ret == ESP_OK;
    if (encode_us > 0 && (uint64_t)encode_us > s_encode_max_us)
        s_encode_max_us = encode_us > UINT32_MAX ? UINT32_MAX : (uint32_t)encode_us;
    s_encode_attempt_pending = true;
    s_encode_attempt_success = encode_ret == ESP_OK;
    s_encode_attempt_codec = s_tx_codec;
    s_encode_attempt_session = s_session_id;
    dual_end_runtime_record(DUAL_END_STAGE_ENCODE, s_tx_codec, s_session_id,
                            encode_us, encode_ret == ESP_OK);
    if (encode_ret != ESP_OK) {
        sender_new_session_locked();
        xSemaphoreGive(s_sender_mutex);
        return encode_ret;
    }
    if (hardware_payload != NULL) {
        s_tx_packet.payload.hardware = *hardware_payload;
    } else {
        s_tx_packet.payload.hardware.version = DUAL_END_HID_INPUT_VERSION;
    }
    s_tx_packet.payload.hardware.flags |= s_pending_input_flags;
    s_tx_packet.payload.stream_id = s_session_id;
    s_tx_packet.payload.capture_sample_index = s_tx_capture_sample_index;
    if (analysis_metadata != NULL) {
        s_tx_packet.payload.analysis = *analysis_metadata;
    }
    s_tx_packet.packet = (dual_end_packet_header_t) {
        .protocol_version = SCEPTER_PROTOCOL_VERSION,
        .source_role = SCEPTER_ROLE_SCEPTER,
        .message_type = SCEPTER_MSG_AUDIO_FRAME,
        .flags = (uint8_t)tx_power_qdbm,
        .sequence = sequence,
        .payload_len = sizeof(dual_end_audio_frame_header_t) + sizeof(s_tx_packet.payload),
    };
    s_tx_packet.audio = (dual_end_audio_frame_header_t) {
        .codec = s_tx_codec,
        .channels = DUAL_END_AUDIO_CHANNELS,
        .sample_rate_hz = DUAL_END_AUDIO_SAMPLE_RATE_HZ,
        .samples_per_channel = DUAL_END_AUDIO_FRAME_SAMPLES,
        .pcm_bytes_after_decode = DUAL_END_AUDIO_FRAME_BYTES,
        .previous_encoded_bytes = 0,
        .current_encoded_bytes = (uint16_t)encoded_bytes,
    };
    s_tx_packet.audio.payload_crc32 = audio_payload_crc32(
        (const uint8_t *)&s_tx_packet.payload, sizeof(s_tx_packet.payload));

    /* Store synchronously copies into its cache; packed byte payload is already
     * the exact 320-byte recovery representation, without temporary repacking. */
    audio_recovery_tx_store(s_recovery_cache, sequence, created_us,
                            &s_tx_packet.audio, s_tx_packet.payload.audio);

    const size_t packet_len = sizeof(dual_end_packet_header_t) +
                              s_tx_packet.packet.payload_len;
    const int64_t radio_submit_started_us = esp_timer_get_time();
    const esp_err_t ret = dual_end_espnow_link_send_audio_at(
        &s_tx_packet, (int)packet_len, created_us);
    const int64_t radio_submit_us = esp_timer_get_time() - radio_submit_started_us;
    s_last_owner_timing.radio_submitted = true;
    s_last_owner_timing.radio_submit_us = radio_submit_us <= 0 ? 0U :
        ((uint64_t)radio_submit_us > UINT32_MAX ? UINT32_MAX : (uint32_t)radio_submit_us);
    s_last_owner_timing.radio_ok = ret == ESP_OK;
    /* Encode success alone is not a MIC_PIPELINE success.  Keep the pending
     * admission open for its owner-timing consumer, but classify a failed
     * radio-queue submission explicitly so it cannot be reported as OK. */
    if (ret != ESP_OK) {
        s_encode_attempt_success=false;
        s_encode_attempt_outcome=DUAL_END_OUTCOME_OPERATION_FAILED;
    }
    s_pending_input_flags = ret == ESP_OK ? 0 : s_tx_packet.payload.hardware.flags;
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "audio queue seq=%" PRIu32 " ret=%s",
                 sequence, esp_err_to_name(ret));
    }

    if (((sequence + 1U) % AUDIO_TX_LOG_INTERVAL_FRAMES) == 0) {
        dual_end_espnow_link_stats_t link = {0};
        dual_end_espnow_link_get_stats(&link);
        const int64_t now_us = esp_timer_get_time();
        const uint32_t submitted = link.tx_audio_submit -
                                   s_last_tx_log_stats.tx_audio_submit;
        const uint32_t fps_x10 = now_us > s_last_tx_log_us ?
            (uint32_t)(((uint64_t)submitted * 10000000ULL) /
                       (uint64_t)(now_us - s_last_tx_log_us)) : 0;
        ESP_LOGI(TAG,
                 "tx seq=%" PRIu32 " session=%08" PRIx32
                 " codec=%u profile=%s encode_max_us=%" PRIu32
                 " layout=%u encoded_frames=%" PRIu64
                 " process_calls=%" PRIu64 " reused_frames=%" PRIu64
                 " failures=%" PRIu64 " failed_process_calls=%" PRIu64
                 " encoded_bytes_last=%" PRIu32 " opus_fec_enabled=%s"
                 " rate=%" PRIu32 ".%" PRIu32 "fps link=%d rssi=%d"
                 " queue=%" PRIu32 "/%" PRIu32 " stale_drop=%" PRIu32
                 " original[ok/fail/deadline]=%" PRIu32 "/%" PRIu32 "/%" PRIu32
                 " repair[ok/fail/deadline]=%" PRIu32 "/%" PRIu32 "/%" PRIu32,
                 sequence, s_session_id, s_tx_codec, codec_profile_name(s_tx_codec),
                 s_encode_max_us, s_tx_layout,
                 s_encoding_stats.frames, s_encoding_stats.process_calls,
                 s_encoding_stats.reused_frames, s_encoding_stats.failures,
                 s_encoding_stats.failed_process_calls, s_last_encoded_bytes,
                 codec_fec_state(s_tx_codec), fps_x10 / 10U, fps_x10 % 10U,
                 link.connected ? 1 : 0, link.last_rssi,
                 link.tx_queue_depth, link.tx_queue_high_water,
                 link.tx_audio_stale_drop,
                 link.tx_audio_final_ok, link.tx_audio_final_fail,
                 link.tx_audio_deadline_drop, link.tx_repair_ok,
                 link.tx_repair_fail, link.tx_repair_deadline_drop);
        s_last_tx_log_stats = link;
        s_last_tx_log_us = now_us;
    }

    s_tx_sequence++;
    s_tx_capture_sample_index += DUAL_END_AUDIO_FRAME_SAMPLES;
    xSemaphoreGive(s_sender_mutex);
    return ret;
}

esp_err_t dual_end_audio_preencode_test_loop(void)
{
    return dual_end_audio_sender_start();
}

static void audio_recv_cb(const uint8_t *src_addr, const uint8_t *data, int len, int rssi, void *ctx)
{
    (void)src_addr;
    (void)ctx;

    if (!data || len < (int)sizeof(dual_end_packet_header_t)) {
        return;
    }

    const dual_end_packet_header_t *header = (const dual_end_packet_header_t *)data;
    if (header->protocol_version != SCEPTER_PROTOCOL_VERSION) {
        return;
    }

    if (header->message_type != SCEPTER_MSG_AUDIO_FRAME ||
        len < (int)(sizeof(dual_end_packet_header_t) + sizeof(dual_end_audio_frame_header_t))) {
        return;
    }

    const dual_end_audio_packet_t *pkt = (const dual_end_audio_packet_t *)data;
    if (pkt->audio.codec != DUAL_END_AUDIO_CODEC_G722 ||
        pkt->audio.sample_rate_hz != DUAL_END_AUDIO_SAMPLE_RATE_HZ ||
        pkt->audio.channels != DUAL_END_AUDIO_CHANNELS) {
        ESP_LOGW(TAG, "unsupported audio frame codec=%u rate=%u ch=%u",
                 pkt->audio.codec, pkt->audio.sample_rate_hz, pkt->audio.channels);
        return;
    }

    const int expected_len = sizeof(dual_end_packet_header_t) + pkt->packet.payload_len;
    if (len != expected_len ||
        pkt->packet.payload_len != sizeof(dual_end_audio_frame_header_t) + sizeof(pkt->payload) ||
        pkt->audio.current_encoded_bytes != DUAL_END_AUDIO_G722_FRAME_BYTES ||
        pkt->audio.pcm_bytes_after_decode != DUAL_END_AUDIO_FRAME_BYTES) {
        ESP_LOGW(TAG,
                 "bad g722 frame len=%d expected=%d payload_len=%u curr=%u pcm=%u",
                 len, expected_len, pkt->packet.payload_len,
                 pkt->audio.current_encoded_bytes, pkt->audio.pcm_bytes_after_decode);
        return;
    }

    bool crc_error = pkt->audio.payload_crc32 != audio_payload_crc32((const uint8_t *)&pkt->payload,
                                                                     sizeof(pkt->payload));
    if (crc_error) {
        ESP_LOGW(TAG, "combined payload crc error seq=%" PRIu32, pkt->packet.sequence);
        return;
    }

    if (g722_decode_frame(pkt->payload.audio,
                          pkt->audio.current_encoded_bytes,
                          s_decoded_interleaved_pcm,
                          DUAL_END_AUDIO_FRAME_BYTES) != ESP_OK) {
        ESP_LOGW(TAG, "audio decode seq=%" PRIu32 " failed", pkt->packet.sequence);
        return;
    }

    if (s_input_event_sink) {
        dual_end_hid_input_payload_t hardware_payload;
        memcpy(&hardware_payload, &pkt->payload.hardware, sizeof(hardware_payload));
        s_input_event_sink(&hardware_payload,
                           pkt->packet.sequence,
                           rssi,
                           s_input_event_sink_ctx);
    }

    int8_t tx_power_qdbm = (int8_t)pkt->packet.flags;
    if (s_current_segment_power_qdbm != tx_power_qdbm) {
        if (s_current_segment_power_qdbm != INT8_MIN) {
            log_stats("rx_segment_summary", s_segment_index, s_current_segment_power_qdbm, &s_segment_stats);
            s_segment_index++;
        }
        reset_stats(&s_segment_stats);
        s_current_segment_power_qdbm = tx_power_qdbm;
        ESP_LOGI(TAG,
                 "audio_rx_start segment=%" PRIu32 " tx_power_qdbm=%d seq=%" PRIu32 " rssi=%d",
                 s_segment_index, tx_power_qdbm, pkt->packet.sequence, rssi);
    }

    update_stats(&s_segment_stats, pkt->packet.sequence, false, rssi);
    update_stats(&s_total_stats, pkt->packet.sequence, false, rssi);
    if ((s_segment_stats.frames_rx % 250) == 0) {
        log_stats("audio_rx_progress", s_segment_index, tx_power_qdbm, &s_segment_stats);
    }
    if ((s_total_stats.frames_rx % 500) == 0) {
        log_stats("audio_rx_total", 0, tx_power_qdbm, &s_total_stats);
    }

    if (s_pcm_sink) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(s_pcm_sink(s_decoded_interleaved_pcm,
                                                 DUAL_END_AUDIO_FRAME_SAMPLE_VALUES,
                                                 pkt->packet.sequence,
                                                 rssi,
                                                 s_pcm_sink_ctx));
    }
}

esp_err_t dual_end_audio_receiver_set_input_event_sink(dual_end_input_event_sink_t sink, void *ctx)
{
    s_input_event_sink = sink;
    s_input_event_sink_ctx = ctx;
    return ESP_OK;
}

esp_err_t dual_end_audio_receiver_start(dual_end_audio_pcm_sink_t sink, void *ctx)
{
    s_pcm_sink = sink;
    s_pcm_sink_ctx = ctx;
    reset_stats(&s_segment_stats);
    reset_stats(&s_total_stats);

    ESP_RETURN_ON_ERROR(ensure_g722_decoder(), TAG, "init G722 decoder");
    ESP_RETURN_ON_ERROR(dual_end_espnow_link_register_recv_cb(audio_recv_cb, NULL),
                        TAG, "register espnow audio rx");
    ESP_LOGI(TAG, "audio receiver started: stereo G722 %d Hz %d bps/channel",
             DUAL_END_AUDIO_SAMPLE_RATE_HZ,
             DUAL_END_AUDIO_G722_BITRATE_BPS);
    return ESP_OK;
}
