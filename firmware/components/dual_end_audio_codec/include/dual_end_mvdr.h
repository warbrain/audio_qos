#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DUAL_END_MVDR_FFT_SIZE 512U
#define DUAL_END_MVDR_HOP_SIZE 256U
#define DUAL_END_MVDR_BINS (DUAL_END_MVDR_FFT_SIZE / 2U + 1U)
#define DUAL_END_MVDR_OUTPUT_CAPACITY 1024U

/* Fixed-memory, owner-only, two-channel soft-mask MVDR beamformer.
 * It accepts 16 kHz interleaved stereo in 320-sample blocks and emits dual
 * mono. No heap, RTOS primitive, VAD model, or calibration file is required. */
typedef struct {
    float frame[2][DUAL_END_MVDR_FFT_SIZE];
    float pending[2][DUAL_END_MVDR_HOP_SIZE];
    float fft[2][DUAL_END_MVDR_FFT_SIZE * 2U];
    float window[DUAL_END_MVDR_FFT_SIZE];
    float synthesis[DUAL_END_MVDR_FFT_SIZE];
    float overlap[DUAL_END_MVDR_HOP_SIZE];
    float overlap_norm[DUAL_END_MVDR_HOP_SIZE];

    /* Per-frequency-bin soft-mask noise floor and normalized Hermitian
     * speech/noise covariance estimates (xx, yy and complex xy). */
    float noise_power[DUAL_END_MVDR_BINS];
    float speech_weight[DUAL_END_MVDR_BINS];
    float noise_weight[DUAL_END_MVDR_BINS];
    float speech_xx[DUAL_END_MVDR_BINS], speech_yy[DUAL_END_MVDR_BINS];
    float speech_xy_re[DUAL_END_MVDR_BINS], speech_xy_im[DUAL_END_MVDR_BINS];
    float noise_xx[DUAL_END_MVDR_BINS], noise_yy[DUAL_END_MVDR_BINS];
    float noise_xy_re[DUAL_END_MVDR_BINS], noise_xy_im[DUAL_END_MVDR_BINS];

    int16_t output[DUAL_END_MVDR_OUTPUT_CAPACITY];
    uint16_t pending_count, output_head, output_count, fallback;
    bool noise_valid, first_frame;

    /* Processing telemetry. valid_bins/fallback_bins count frequency-bin
     * decisions; fallback_hops counts hops that produced no MVDR bin. */
    uint64_t processed_hops, valid_bins, fallback_bins, fallback_hops;
    uint64_t queue_underflows, queue_overflows;
    uint16_t speech_mask_q15;
} dual_end_mvdr_t;

void dual_end_mvdr_reset(dual_end_mvdr_t *state);
/* Processes exactly one 20 ms interleaved stereo block, in place. The
 * windowed overlap-add startup queue adds 32 ms, matching the Wiener path. */
bool dual_end_mvdr_process(dual_end_mvdr_t *state, int16_t *interleaved,
                          size_t frames);
