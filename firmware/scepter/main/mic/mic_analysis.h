#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dual_end_link_config.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MIC_ANALYSIS_INPUT_FRAMES 960

esp_err_t mic_analysis_init(void);
void mic_analysis_reset(void);
void mic_analysis_set_frame_gap(uint32_t frame_gap);
/* Startup failure cleanup only, before a diagnostic owner starts. */
void mic_analysis_release(void);

/* Processes one 20 ms, interleaved, raw 48 kHz stereo frame.  The return
 * value is elapsed DSP time in microseconds for real-time budget logging. */
uint32_t mic_analysis_process_frame(
    const int16_t *interleaved_raw,
    size_t frame_count,
    bool motor_active,
    dual_end_audio_analysis_metadata_t *metadata);

#ifdef __cplusplus
}
#endif
