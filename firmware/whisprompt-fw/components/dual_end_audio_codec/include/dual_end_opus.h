#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "dual_end_codec_frame.h"

/* Calls are owned by the audio sender lock or receiver task. No internal lock.
 * Codec state is independent for each microphone; both channels share a frame
 * sequence, CRC, deadline and recovery slot. FEC, DTX and VBR are disabled. */
esp_err_t dual_end_opus_encoder_prepare(void);
/* Prepare the encoder for a specific Opus codec id (3 = 32 kbps, 5 = 62 kbps).
 * Switching profiles closes and reopens the SDK encoders. */
esp_err_t dual_end_opus_encoder_prepare_for(uint8_t codec);
void dual_end_opus_encoder_reset(void);
esp_err_t dual_end_opus_encode(const int16_t *pcm, uint8_t *payload,
    size_t payload_len, uint32_t *encoded_bytes);
/* Owner-declared dual mono reuses one compressed packet in both existing wire
 * slots; switching layout requires a sender session reset first. process_calls
 * counts actual SDK invocations, including failed ones, when non-NULL. */
esp_err_t dual_end_opus_encode_with_layout(const int16_t *pcm, uint8_t *payload,
    size_t payload_len, uint32_t *encoded_bytes, bool dual_mono,
    uint32_t *process_calls);
esp_err_t dual_end_opus_decoder_prepare(void);
void dual_end_opus_decoder_reset(void);
esp_err_t dual_end_opus_decode(const uint8_t *payload, size_t payload_len,
    int16_t *pcm, size_t pcm_bytes);
