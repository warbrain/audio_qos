#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "dual_end_link_config.h"

/* Fixed 320-byte recovery slot, preserving the existing 400-byte normal packet.
 * Opus stores two little-endian uint16 lengths followed by independent L/R
 * mono packets. The rest is zero. Length includes the four prefix bytes.
 * CBR is 32 kbps/channel at 20 ms: exactly 80 bytes per mono packet. */
static inline uint16_t dual_end_codec_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline bool dual_end_codec_payload_valid(uint8_t codec,
    uint16_t encoded_bytes, const uint8_t *payload, size_t payload_len)
{
    if (!payload || payload_len != DUAL_END_AUDIO_ENCODED_FRAME_BYTES) return false;
    if (codec == DUAL_END_AUDIO_CODEC_G722)
        return encoded_bytes == DUAL_END_AUDIO_G722_FRAME_BYTES;
    uint16_t total = 0, channel = 0;
    if (codec == DUAL_END_AUDIO_CODEC_OPUS) {
        total = DUAL_END_AUDIO_OPUS_FRAME_BYTES;
        channel = DUAL_END_AUDIO_OPUS_CHANNEL_FRAME_BYTES;
    } else if (codec == DUAL_END_AUDIO_CODEC_OPUS_62) {
        total = DUAL_END_AUDIO_OPUS62_FRAME_BYTES;
        channel = DUAL_END_AUDIO_OPUS62_CHANNEL_FRAME_BYTES;
    } else {
        return false;
    }
    if (encoded_bytes != total ||
        dual_end_codec_u16(payload) != channel ||
        dual_end_codec_u16(payload + 2) != channel)
        return false;
    for (size_t i = encoded_bytes; i < payload_len; ++i)
        if (payload[i] != 0) return false;
    return true;
}
static inline bool dual_end_dsp_version_item_valid(uint8_t version, uint8_t item)
{
    return (version == 1 && item <= 14) || (version == 2 && (item == 15 || item == 16)) ||
        (version == 3 && item >= 17 && item <= 22);
}
