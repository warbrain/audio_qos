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
    if (codec != DUAL_END_AUDIO_CODEC_G722) return false;
    return encoded_bytes == DUAL_END_AUDIO_G722_FRAME_BYTES;
}
static inline bool dual_end_dsp_version_item_valid(uint8_t version, uint8_t item)
{
    return (version == 1 && item <= 14) || (version == 2 && (item == 15 || item == 16)) ||
        (version == 3 && item >= 17 && item <= 22);
}
