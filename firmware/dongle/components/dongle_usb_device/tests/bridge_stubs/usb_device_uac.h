#pragma once

#include "uac_test_boundary.h"

typedef struct {
    uint32_t mic_generation;
} uac_device_stats_t;

static uint32_t uac_test_generation;

static inline void uac_device_get_stats(uac_device_stats_t *stats)
{
    stats->mic_generation = uac_test_generation;
}

static inline void uac_device_mark_first_audio_sample(size_t offset_samples,
                                                       int64_t target_us)
{
    (void)offset_samples;
    (void)target_us;
}

static inline bool uac_device_input_playout_time(int64_t *playout_us)
{
    (void)playout_us;
    return false;
}
