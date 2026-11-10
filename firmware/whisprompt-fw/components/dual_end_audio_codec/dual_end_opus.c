#include "dual_end_opus.h"
#include "dual_end_pcm_layout.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_opus_enc.h"
#include "esp_opus_dec.h"

static const char *TAG = "dual_end_opus";
static void *s_encoder[DUAL_END_AUDIO_CHANNELS];
static uint8_t *s_encode_output[DUAL_END_AUDIO_CHANNELS];
static uint32_t s_encode_output_capacity[DUAL_END_AUDIO_CHANNELS];
static void *s_decoder[DUAL_END_AUDIO_CHANNELS];
/* Active encoder profile; 0 means "not prepared". Opus is CBR, so the frame
 * size follows from the bitrate: 32 kbps -> 80 B, 62 kbps -> 155 B. */
static uint8_t s_encoder_codec;
_Alignas(16) static int16_t s_encode_pcm[DUAL_END_AUDIO_CHANNELS][DUAL_END_AUDIO_FRAME_SAMPLES];
_Alignas(16) static int16_t s_decode_pcm[DUAL_END_AUDIO_CHANNELS][DUAL_END_AUDIO_FRAME_SAMPLES];

_Static_assert(DUAL_END_AUDIO_CHANNEL_FRAME_BYTES % 16 == 0, "Opus PCM row alignment");

static void close_encoders(void)
{
    for (size_t ch = 0; ch < DUAL_END_AUDIO_CHANNELS; ++ch) {
        if (s_encoder[ch]) esp_opus_enc_close(s_encoder[ch]);
        s_encoder[ch] = NULL;
        free(s_encode_output[ch]);
        s_encode_output[ch] = NULL;
        s_encode_output_capacity[ch] = 0;
    }
}
static bool opus_profile(uint8_t codec, uint32_t *bitrate, uint16_t *channel_bytes)
{
    if (codec == DUAL_END_AUDIO_CODEC_OPUS) {
        *bitrate = DUAL_END_AUDIO_OPUS_BITRATE_BPS;
        *channel_bytes = DUAL_END_AUDIO_OPUS_CHANNEL_FRAME_BYTES;
        return true;
    }
    if (codec == DUAL_END_AUDIO_CODEC_OPUS_62) {
        *bitrate = DUAL_END_AUDIO_OPUS62_BITRATE_BPS;
        *channel_bytes = DUAL_END_AUDIO_OPUS62_CHANNEL_FRAME_BYTES;
        return true;
    }
    return false;
}
static esp_err_t encoder_prepare_for(uint8_t codec)
{
    uint32_t bitrate = 0;
    uint16_t channel_bytes = 0;
    if (!opus_profile(codec, &bitrate, &channel_bytes)) return ESP_ERR_INVALID_ARG;
    /* A reset may have retired either channel. Reuse complete channels in
     * the same profile; only a profile change replaces both SDK histories. */
    if (s_encoder_codec != codec) close_encoders();
    for (size_t ch = 0; ch < DUAL_END_AUDIO_CHANNELS; ++ch) {
        if (s_encoder[ch] && s_encode_output[ch] &&
            s_encode_output_capacity[ch] >= channel_bytes &&
            s_encode_output_capacity[ch] <= DUAL_END_AUDIO_OPUS_MAX_FRAME_BYTES)
            continue;
        /* Handle/scratch readiness is one unit. Retire an incomplete channel
         * before reopening it, without leaking a surviving handle or buffer. */
        if (s_encoder[ch]) esp_opus_enc_close(s_encoder[ch]);
        s_encoder[ch] = NULL;
        free(s_encode_output[ch]);
        s_encode_output[ch] = NULL;
        s_encode_output_capacity[ch] = 0;
        esp_opus_enc_config_t cfg = ESP_OPUS_ENC_CONFIG_DEFAULT();
        cfg.sample_rate = DUAL_END_AUDIO_SAMPLE_RATE_HZ;
        cfg.channel = 1;
        cfg.bits_per_sample = 16;
        cfg.bitrate = bitrate;
        cfg.frame_duration = ESP_OPUS_ENC_FRAME_DURATION_20_MS;
        /* The producer must deliver two mono packets every 20 ms. Restricted
         * low delay uses CELT rather than the SILK speech-analysis path; it is
         * still standard Opus, decoded from the packet TOC by the same decoder.
         * Keep CBR/frame/channel parameters and radio/recovery timing unchanged.
         * On-device timing and speech/ASR quality must be measured separately. */
        cfg.application_mode = ESP_OPUS_ENC_APPLICATION_LOWDELAY;
        cfg.complexity = DUAL_END_AUDIO_OPUS_COMPLEXITY;
        /* FEC: the encoder API does support it (esp_opus_enc.h `enable_fec`),
         * but espressif/esp_audio_codec 2.6.x has no Opus FEC decode path:
         * `esp_opus_dec_decode` never reads in_raw.frame_recover and the
         * component README documents PLC only for SBC and LC3. Enabling it
         * would spend roughly a fifth of the bitrate on parity that no peer
         * can consume. Revisit only when both ends ship a decoder that calls
         * opus_decode(..., decode_fec = 1). */
        cfg.enable_fec = false;
        cfg.enable_dtx = cfg.enable_vbr = false;
        esp_audio_err_t err = esp_opus_enc_open(&cfg, sizeof(cfg), &s_encoder[ch]);
        int in_size = 0, out_size = 0;
        if (err == ESP_AUDIO_ERR_OK && s_encoder[ch])
            err = esp_opus_enc_get_frame_size(s_encoder[ch], &in_size, &out_size);
        if (err != ESP_AUDIO_ERR_OK || !s_encoder[ch] ||
            in_size != DUAL_END_AUDIO_CHANNEL_FRAME_BYTES ||
            out_size < channel_bytes ||
            out_size > DUAL_END_AUDIO_OPUS_MAX_FRAME_BYTES) {
            ESP_LOGE(TAG, "encoder prepare ch=%u err=%d input=%d output=%d",
                     (unsigned)ch, err, in_size, out_size);
            close_encoders();
            return ESP_FAIL;
        }
        /* The SDK requires output.len to match its returned frame size, which
         * can be larger than the actual 80-byte CBR packet. Keep that scratch
         * separate from the fixed radio reservation and copy only checked data. */
        s_encode_output[ch] = malloc((size_t)out_size);
        if (!s_encode_output[ch]) {
            close_encoders();
            return ESP_ERR_NO_MEM;
        }
        s_encode_output_capacity[ch] = (uint32_t)out_size;
    }
    s_encoder_codec = codec;
    return ESP_OK;
}
esp_err_t dual_end_opus_encoder_prepare(void)
{
    return encoder_prepare_for(DUAL_END_AUDIO_CODEC_OPUS);
}
esp_err_t dual_end_opus_encoder_prepare_for(uint8_t codec)
{
    return encoder_prepare_for(codec);
}
void dual_end_opus_encoder_reset(void)
{
    for (size_t ch = 0; ch < DUAL_END_AUDIO_CHANNELS; ++ch) {
        if (s_encoder[ch] && esp_opus_enc_reset(s_encoder[ch]) != ESP_AUDIO_ERR_OK) {
            esp_opus_enc_close(s_encoder[ch]);
            s_encoder[ch] = NULL;
            free(s_encode_output[ch]);
            s_encode_output[ch] = NULL;
            s_encode_output_capacity[ch] = 0;
        }
    }
}
esp_err_t dual_end_opus_encode(const int16_t *pcm, uint8_t *payload,
    size_t payload_len, uint32_t *encoded_bytes)
{
    return dual_end_opus_encode_with_layout(pcm, payload, payload_len,
                                            encoded_bytes, false, NULL);
}
esp_err_t dual_end_opus_encode_with_layout(const int16_t *pcm, uint8_t *payload,
    size_t payload_len, uint32_t *encoded_bytes, bool dual_mono,
    uint32_t *process_calls)
{
    if (process_calls) *process_calls = 0;
    if (encoded_bytes) *encoded_bytes = 0;
    if (!pcm || !payload || !encoded_bytes ||
        payload_len != DUAL_END_AUDIO_ENCODED_FRAME_BYTES) return ESP_ERR_INVALID_ARG;
    if (dual_mono) {
        if (!dual_end_pcm_extract_dual_mono(pcm, s_encode_pcm[0],
                                           DUAL_END_AUDIO_FRAME_SAMPLES))
            return ESP_ERR_INVALID_ARG;
    } else {
        dual_end_pcm_split(pcm, s_encode_pcm[0], s_encode_pcm[1],
                            DUAL_END_AUDIO_FRAME_SAMPLES);
    }
    /* Healthy channels take the allocation-free reuse path. A failed reset
     * repairs either side while retaining the explicitly selected profile. */
    esp_err_t result = encoder_prepare_for(s_encoder_codec ? s_encoder_codec :
        DUAL_END_AUDIO_CODEC_OPUS);
    if (result != ESP_OK) return result;
    uint32_t bitrate = 0;
    uint16_t channel_bytes = 0;
    if (!opus_profile(s_encoder_codec, &bitrate, &channel_bytes)) return ESP_ERR_INVALID_STATE;
    const uint16_t total_bytes = (uint16_t)(4U + 2U * channel_bytes);
    memset(payload, 0, payload_len);
    const size_t channels_to_encode = dual_mono ? 1 : DUAL_END_AUDIO_CHANNELS;
    for (size_t ch = 0; ch < channels_to_encode; ++ch) {
        esp_audio_enc_in_frame_t input = {
            .buffer = (uint8_t *)s_encode_pcm[ch], .len = DUAL_END_AUDIO_CHANNEL_FRAME_BYTES};
        esp_audio_enc_out_frame_t output = {
            .buffer = s_encode_output[ch],
            .len = s_encode_output_capacity[ch]};
        if (process_calls) ++*process_calls;
        esp_audio_err_t err = esp_opus_enc_process(s_encoder[ch], &input, &output);
        if (err != ESP_AUDIO_ERR_OK ||
            output.encoded_bytes != channel_bytes) {
            ESP_LOGE(TAG, "encode ch=%u err=%d bytes=%lu", (unsigned)ch,
                     err, (unsigned long)output.encoded_bytes);
            return ESP_FAIL;
        }
        memcpy(payload + 4 + ch * channel_bytes,
               s_encode_output[ch], output.encoded_bytes);
        payload[ch * 2] = (uint8_t)output.encoded_bytes;
        payload[ch * 2 + 1] = (uint8_t)(output.encoded_bytes >> 8);
    }
    if (dual_mono) {
        /* Both wire slots carry the same packet, sized by the active profile.
         * Each Dongle decoder retains its own history and decodes it twice. */
        memcpy(payload + 4 + channel_bytes,
               payload + 4, channel_bytes);
        payload[2] = payload[0];
        payload[3] = payload[1];
    }
    *encoded_bytes = total_bytes;
    return ESP_OK;
}
static void close_decoders(void)
{
    for (size_t ch = 0; ch < DUAL_END_AUDIO_CHANNELS; ++ch) {
        if (s_decoder[ch]) (void)esp_opus_dec_close(s_decoder[ch]);
        s_decoder[ch] = NULL;
    }
}
esp_err_t dual_end_opus_decoder_prepare(void)
{
    for (size_t ch = 0; ch < DUAL_END_AUDIO_CHANNELS; ++ch) {
        if (s_decoder[ch]) continue;
        esp_opus_dec_cfg_t cfg = ESP_OPUS_DEC_CONFIG_DEFAULT();
        cfg.sample_rate = DUAL_END_AUDIO_SAMPLE_RATE_HZ;
        cfg.channel = 1;
        cfg.frame_duration = ESP_OPUS_DEC_FRAME_DURATION_20_MS;
        cfg.self_delimited = false;
        esp_audio_err_t err = esp_opus_dec_open(&cfg, sizeof(cfg), &s_decoder[ch]);
        if (err != ESP_AUDIO_ERR_OK || !s_decoder[ch]) {
            ESP_LOGE(TAG, "decoder prepare ch=%u err=%d", (unsigned)ch, err);
            close_decoders();
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}
void dual_end_opus_decoder_reset(void)
{
    for (size_t ch = 0; ch < DUAL_END_AUDIO_CHANNELS; ++ch) {
        if (s_decoder[ch] && esp_opus_dec_reset(s_decoder[ch]) != ESP_AUDIO_ERR_OK) {
            (void)esp_opus_dec_close(s_decoder[ch]);
            s_decoder[ch] = NULL;
        }
    }
    memset(s_decode_pcm, 0, sizeof(s_decode_pcm));
}
esp_err_t dual_end_opus_decode(const uint8_t *payload, size_t payload_len,
    int16_t *pcm, size_t pcm_bytes)
{
    if (!payload || !pcm || pcm_bytes < DUAL_END_AUDIO_FRAME_BYTES ||
        payload_len != DUAL_END_AUDIO_ENCODED_FRAME_BYTES) return ESP_ERR_INVALID_ARG;
    /* Length-driven: the packet carries both mono lengths, so the same decoder
     * serves 32 kbps (80 B) and 62 kbps (155 B) without extra state. */
    const uint16_t length[2] = {dual_end_codec_u16(payload),
                                dual_end_codec_u16(payload + 2)};
    uint8_t codec;
    uint16_t encoded_bytes;
    if (length[0] == DUAL_END_AUDIO_OPUS_CHANNEL_FRAME_BYTES) {
        codec = DUAL_END_AUDIO_CODEC_OPUS;
        encoded_bytes = DUAL_END_AUDIO_OPUS_FRAME_BYTES;
    } else if (length[0] == DUAL_END_AUDIO_OPUS62_CHANNEL_FRAME_BYTES) {
        codec = DUAL_END_AUDIO_CODEC_OPUS_62;
        encoded_bytes = DUAL_END_AUDIO_OPUS62_FRAME_BYTES;
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    /* Validate both profile lengths and all reserved padding before opening
     * or invoking any SDK decoder; malformed packets must not mutate history. */
    if (!dual_end_codec_payload_valid(codec, encoded_bytes, payload, payload_len))
        return ESP_ERR_INVALID_ARG;
    esp_err_t result = dual_end_opus_decoder_prepare();
    if (result != ESP_OK) return result;
    size_t offset = 4U;
    for (size_t ch = 0; ch < DUAL_END_AUDIO_CHANNELS; ++ch) {
        esp_audio_dec_in_raw_t raw = {
            .buffer = (uint8_t *)payload + offset,
            .len = length[ch]};
        offset += length[ch];
        esp_audio_dec_out_frame_t output = {
            .buffer = (uint8_t *)s_decode_pcm[ch], .len = DUAL_END_AUDIO_CHANNEL_FRAME_BYTES};
        esp_audio_dec_info_t info = {0};
        esp_audio_err_t err = esp_opus_dec_decode(s_decoder[ch], &raw, &output, &info);
        if (err != ESP_AUDIO_ERR_OK || raw.consumed != length[ch] ||
            output.decoded_size != DUAL_END_AUDIO_CHANNEL_FRAME_BYTES) {
            ESP_LOGE(TAG, "decode ch=%u err=%d consumed=%lu samples=%lu",
                     (unsigned)ch, err, (unsigned long)raw.consumed,
                     (unsigned long)output.decoded_size);
            return ESP_FAIL;
        }
    }
    dual_end_pcm_interleave(s_decode_pcm[0], s_decode_pcm[1], pcm, DUAL_END_AUDIO_FRAME_SAMPLES);
    return ESP_OK;
}
