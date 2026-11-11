#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "dual_end_wiener.h"

/* Portable 16 kHz/20 ms owner-only processing; no allocation, RTOS or DMA.
 * PCB spacing is fixed at 20 mm. Amplitude/phase responses are not calibrated.
 * Mode 0 is bit-exact stereo bypass; fused modes explicitly emit dual mono.
 * Mode 3 preserves aligned left and NLMS-matched right separately, without DAS.
 * Mode 4 adds a fixed-memory spectral Wiener post-filter to fused DAS.
 * Mode 5 applies independent fixed-memory Wiener filters to L and R. */
#define DUAL_END_SPATIAL_CAPABILITIES 31U /* bit 3 DAS+Wiener; bit 4 independent stereo Wiener */
#define DUAL_END_SPATIAL_FRAME_SAMPLES 320U
#define DUAL_END_SPATIAL_TAPS 64U
#define DUAL_END_SPATIAL_MAX_DELAY (0.020f * 16000.0f / 343.0f)
enum { DUAL_END_SPATIAL_BYPASS=0, DUAL_END_SPATIAL_DAS=1,
    DUAL_END_SPATIAL_NLMS=2, DUAL_END_SPATIAL_NLMS_STEREO=3,
    DUAL_END_SPATIAL_DAS_WIENER=4, DUAL_END_SPATIAL_WIENER_STEREO=5 };
enum { DUAL_END_LAYOUT_STEREO=0, DUAL_END_LAYOUT_DUAL_MONO=1 };
/* Numeric modes 0..2 and their capability requirements are a legacy contract.
 * Mode 3 requires its stereo-NLMS bit; mode 4 requires DAS + Wiener; mode 5
 * requires the separate independent-stereo-Wiener capability bit. */
static inline bool dual_end_spatial_mode_supported(uint16_t mode, uint16_t caps) {
    switch(mode) {
        case DUAL_END_SPATIAL_BYPASS:return true;
        case DUAL_END_SPATIAL_DAS:return (caps & 1U)==1U;
        case DUAL_END_SPATIAL_NLMS:return (caps & 3U)==3U;
        case DUAL_END_SPATIAL_NLMS_STEREO:return (caps & 4U)==4U;
        case DUAL_END_SPATIAL_DAS_WIENER:return (caps & 9U)==9U;
        case DUAL_END_SPATIAL_WIENER_STEREO:return (caps & 16U)==16U;
        default:return false;
    }
}
static inline uint16_t dual_end_spatial_mode_layout(uint16_t mode) {
    return mode==DUAL_END_SPATIAL_DAS || mode==DUAL_END_SPATIAL_NLMS ||
        mode==DUAL_END_SPATIAL_DAS_WIENER ?
        DUAL_END_LAYOUT_DUAL_MONO : DUAL_END_LAYOUT_STEREO;
}
enum { DUAL_END_SPATIAL_OK=0, DUAL_END_SPATIAL_LOW_ENERGY=1,
    DUAL_END_SPATIAL_LOW_CORRELATION=2, DUAL_END_SPATIAL_CLIPPED=3,
    DUAL_END_SPATIAL_CHANNEL_FAULT=4, DUAL_END_SPATIAL_ALIGNMENT=5,
    DUAL_END_SPATIAL_WARMUP=6, DUAL_END_SPATIAL_COEFFICIENT_FAULT=7 };
typedef struct {
    uint16_t mode, layout, fallback, generation;
    float delay_samples, correlation, matching_mix, coefficient_l1;
    uint64_t frames, updated_frames, frozen_frames, mixed_frames;
    /* fallback_frames counts NLMS samples without a coefficient update, not
     * all DAS alignment fallbacks. Current fallback enum applies to the frame. */
    uint64_t fallback_frames, matching_mix_q16_sum;
    uint32_t bounded_updates, resets;
    bool polarity_inverted;
} dual_end_spatial_stats_t;
typedef struct {
    dual_end_spatial_stats_t stats;
    dual_end_wiener_t wiener;
    dual_end_wiener_t wiener_right;
    int16_t wiener_channel_pcm[2U * DUAL_END_SPATIAL_FRAME_SAMPLES];
    float previous[2], coefficients[DUAL_END_SPATIAL_TAPS];
    float history[DUAL_END_SPATIAL_TAPS], history_power;
    uint32_t head, history_frames;
    /* Explicitly consume alignment before the 16-byte-aligned owner scratch;
     * this keeps strict MSVC host verification free of implicit padding. */
    uint32_t scratch_alignment_padding[5];
    /* Scratch lives with the owner state, never on the DMA or task stack. */
#if defined(_MSC_VER)
    __declspec(align(16)) float aligned[2][DUAL_END_SPATIAL_FRAME_SAMPLES];
#else
    _Alignas(16) float aligned[2][DUAL_END_SPATIAL_FRAME_SAMPLES];
#endif
} dual_end_spatial_t;
typedef struct {
    uint32_t us[2];
    uint8_t channels;
    bool valid;
} dual_end_spatial_wiener_timing_t;
typedef int64_t (*dual_end_spatial_clock_us_fn)(void);
_Static_assert(offsetof(dual_end_spatial_t, aligned) ==
    offsetof(dual_end_spatial_t, scratch_alignment_padding) +
        sizeof(((dual_end_spatial_t *)0)->scratch_alignment_padding),
    "explicit scratch alignment padding");
bool dual_end_spatial_reset(dual_end_spatial_t *state, uint16_t mode, uint16_t generation);
/* Accept exactly a complete interleaved stereo block. Return false without
 * mutation for an invalid argument. Mode 0 returns before touching PCM/state. */
bool dual_end_spatial_process(dual_end_spatial_t *state, int16_t *pcm, size_t frames);
/* Optional diagnostic variant. A non-null clock measures only each
 * dual_end_wiener_process() call; output samples and state transitions are
 * identical to dual_end_spatial_process(). Mode 4 reports one fused channel;
 * mode 5 reports independent left/right durations. */
bool dual_end_spatial_process_timed(dual_end_spatial_t *state, int16_t *pcm,
    size_t frames, dual_end_spatial_clock_us_fn clock_us,
    dual_end_spatial_wiener_timing_t *wiener_timing);
