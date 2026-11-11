/*
 * RLE decompression helper for compressed animation frames.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Decompress a single RLE-compressed animation frame.
 *
 * Input format: [run_length BE 2 bytes][pixel BE 2 bytes]...
 * Output format: big-endian RGB565 pixel bytes.
 *
 * @param src           Pointer to compressed frame data.
 * @param src_len       Length of compressed data in bytes.
 * @param dst           Output buffer for decompressed RGB565 bytes.
 * @param dst_capacity  Maximum number of bytes to write to dst.
 *
 * @return Number of bytes written to dst, or 0 on error.
 */
size_t anim_rle_decompress(const uint8_t *src, size_t src_len,
                           uint8_t *dst, size_t dst_capacity);

#ifdef __cplusplus
}
#endif
