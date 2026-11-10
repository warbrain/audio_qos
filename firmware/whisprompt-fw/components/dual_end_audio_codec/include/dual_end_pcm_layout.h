#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* Private codec-owner helpers: source/destination ranges must not overlap.
 * No alignment assumption on public caller PCM; internal row strides are asserted. */
static inline void dual_end_pcm_split(const int16_t *__restrict input,
    int16_t *__restrict left, int16_t *__restrict right, size_t frames)
{
    for(size_t i=0;i<frames;++i) { left[i]=input[2*i]; right[i]=input[2*i+1]; }
}
static inline void dual_end_pcm_interleave(const int16_t *__restrict left,
    const int16_t *__restrict right, int16_t *__restrict output, size_t frames)
{
    for(size_t i=0;i<frames;++i) { output[2*i]=left[i]; output[2*i+1]=right[i]; }
}
/* The owner chooses the layout. This checks its dual-mono contract rather than
 * inferring a layout from occasional equal stereo samples. Never encode only
 * one side of a mismatched frame: that would silently discard microphone data. */
static inline bool dual_end_pcm_extract_dual_mono(const int16_t *__restrict input,
    int16_t *__restrict mono, size_t frames)
{
    for (size_t i = 0; i < frames; ++i) {
        if (input[2 * i] != input[2 * i + 1]) return false;
        mono[i] = input[2 * i];
    }
    return true;
}
