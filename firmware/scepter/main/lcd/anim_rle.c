/*
 * RLE decompression implementation for compressed animation frames.
 */

#include "anim_rle.h"

size_t anim_rle_decompress(const uint8_t *src, size_t src_len,
                           uint8_t *dst, size_t dst_capacity)
{
    if (src == NULL || dst == NULL || src_len == 0 || dst_capacity == 0) {
        return 0;
    }

    size_t src_pos = 0;
    size_t dst_pos = 0;

    while (src_pos + 4 <= src_len) {
        uint16_t run_len = ((uint16_t)src[src_pos] << 8) | src[src_pos + 1];
        uint16_t pixel   = ((uint16_t)src[src_pos + 2] << 8) | src[src_pos + 3];
        src_pos += 4;

        for (uint16_t i = 0; i < run_len; i++) {
            if (dst_pos + 2 > dst_capacity) {
                return 0; /* Output buffer overflow */
            }
            dst[dst_pos++] = (uint8_t)(pixel >> 8);
            dst[dst_pos++] = (uint8_t)(pixel & 0xFF);
        }
    }

    return dst_pos;
}
