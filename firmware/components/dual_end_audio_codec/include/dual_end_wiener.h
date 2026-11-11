#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DUAL_END_WIENER_FFT_SIZE 512U
#define DUAL_END_WIENER_HOP_SIZE 256U
#define DUAL_END_WIENER_BINS (DUAL_END_WIENER_FFT_SIZE / 2U + 1U)
#define DUAL_END_WIENER_OUTPUT_CAPACITY 1024U

/* Fixed-memory, 16 kHz streaming Wiener post-filter. State belongs to the
 * existing single audio owner; processing allocates nothing and takes no lock. */
typedef struct {
    float frame[DUAL_END_WIENER_FFT_SIZE];
    float pending[DUAL_END_WIENER_HOP_SIZE];
    float fft[DUAL_END_WIENER_FFT_SIZE * 2U];
    float window[DUAL_END_WIENER_FFT_SIZE];
    float overlap[DUAL_END_WIENER_HOP_SIZE];
    float overlap_norm[DUAL_END_WIENER_HOP_SIZE];
    float noise_power[DUAL_END_WIENER_BINS];
    float gain[DUAL_END_WIENER_BINS];
    int16_t output[DUAL_END_WIENER_OUTPUT_CAPACITY];
    uint16_t pending_count;
    uint16_t output_head;
    uint16_t output_count;
    bool noise_valid;
    bool first_frame;
    /* Completed FFT/STFT hops, including the startup-padding frame. This is
     * processing-work telemetry and does not equal emitted audio hop count. */
    uint64_t processed_hops;
    uint64_t queue_underflows;
    uint64_t queue_overflows;
} dual_end_wiener_t;

void dual_end_wiener_reset(dual_end_wiener_t *state);
/* Processes one mono signal duplicated in interleaved stereo PCM in place.
 * The fixed STFT startup queue introduces 32 ms of algorithmic latency. */
bool dual_end_wiener_process(dual_end_wiener_t *state, int16_t *interleaved,
                             size_t frames);
