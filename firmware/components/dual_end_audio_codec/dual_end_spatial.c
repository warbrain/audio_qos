#include "dual_end_spatial.h"
#include <math.h>
#include <string.h>

/* Release capability set: bypass (0) and stereo Wiener (5) only.
 *
 * DAS/NLMS (1..3), DAS+Wiener (4) and mask MVDR (6) were measured with no
 * positive gain (DAS +12.5 pp and NLMS +9.5 pp CER on the hard directional
 * condition, both significant), so their matcher/alignment state and code are
 * removed. The numeric mode ids stay in the header as a protocol contract:
 * dual_end_spatial_mode_supported() still rejects them for capabilities 16U. */

bool dual_end_spatial_reset(dual_end_spatial_t *s,uint16_t mode,uint16_t generation) {
    if (!s || !dual_end_spatial_mode_supported(mode,DUAL_END_SPATIAL_CAPABILITIES)) return false;
    memset(s,0,sizeof(*s));
    s->stats.mode=mode; s->stats.generation=generation;
    s->stats.layout=dual_end_spatial_mode_layout(mode);
    if (mode==DUAL_END_SPATIAL_WIENER_STEREO) {
        dual_end_wiener_reset(&s->wiener);
        dual_end_wiener_reset(&s->wiener_right);
    }
    return true;
}

bool dual_end_spatial_process(dual_end_spatial_t *s,int16_t *pcm,size_t frames) {
    return dual_end_spatial_process_timed(s, pcm, frames, NULL, NULL);
}

bool dual_end_spatial_process_timed(dual_end_spatial_t *s,int16_t *pcm,size_t frames,
    dual_end_spatial_clock_us_fn clock_us,
    dual_end_spatial_wiener_timing_t *wiener_timing) {
    if (wiener_timing) memset(wiener_timing, 0, sizeof(*wiener_timing));
    if (!s || !pcm || frames != DUAL_END_SPATIAL_FRAME_SAMPLES ||
        !dual_end_spatial_mode_supported(s->stats.mode,DUAL_END_SPATIAL_CAPABILITIES)) return false;
    if (s->stats.mode==DUAL_END_SPATIAL_BYPASS) return true;
    /* Capability check above admits stereo Wiener only. */
    if (s->stats.mode!=DUAL_END_SPATIAL_WIENER_STEREO) return false;

    const bool measure_wiener = clock_us != NULL && wiener_timing != NULL;
    /* Keep the physical L/R channels separate. Each channel has its own noise
     * estimate, gain history, overlap state and queue; no alignment,
     * averaging or shared Wiener state is used. The owner-owned scratch keeps
     * this off the task stack and heap. */
    for (size_t channel=0;channel<2U;++channel) {
        dual_end_wiener_t *filter=channel==0U?&s->wiener:&s->wiener_right;
        for (size_t i=0;i<frames;++i) {
            const int16_t sample=pcm[2U*i+channel];
            s->wiener_channel_pcm[2U*i]=sample;
            s->wiener_channel_pcm[2U*i+1U]=sample;
        }
        const int64_t started_us = measure_wiener ? clock_us() : 0;
        if (!dual_end_wiener_process(filter,s->wiener_channel_pcm,frames)) return false;
        const int64_t finished_us = measure_wiener ? clock_us() : 0;
        if (measure_wiener) {
            const int64_t elapsed_us = finished_us - started_us;
            wiener_timing->us[channel] = elapsed_us <= 0 ? 0U :
                ((uint64_t)elapsed_us > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed_us);
        }
        for (size_t i=0;i<frames;++i) pcm[2U*i+channel]=s->wiener_channel_pcm[2U*i];
    }
    if (measure_wiener) {
        wiener_timing->channels = 2U;
        wiener_timing->valid = true;
    }
    s->stats.frames+=frames;
    s->stats.fallback=DUAL_END_SPATIAL_OK;
    return true;
}
