"""Portable behavioral checks for the production fixed-memory mode-6 DSP.

These synthetic vectors verify state/stream safety only; they do not claim
real-room quality, ESP32 deadline margin, or Qwen3-ASR improvement.
"""
from pathlib import Path

from test_audio_recovery_042 import _build_host_executable, _run

ROOT = Path(__file__).resolve().parents[1]
CODEC = ROOT / "components/dual_end_audio_codec"


def test_mask_mvdr_stream_fallback_and_reset(tmp_path):
    harness = tmp_path / "mvdr.c"
    harness.write_text(r'''
#include "dual_end_spatial.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static dual_end_spatial_t state;
static unsigned phase;
static int16_t sample(float value) {
    value=fmaxf(-1.0f,fminf(1.0f,value));
    return (int16_t)lrintf(value*32767.0f);
}
int main(void) {
    assert(DUAL_END_SPATIAL_CAPABILITIES==63U);
    assert(!dual_end_spatial_mode_supported(6,31U));
    assert(dual_end_spatial_mode_supported(6,63U));
    assert(dual_end_spatial_mode_layout(6)==DUAL_END_LAYOUT_DUAL_MONO);
    assert(dual_end_spatial_reset(&state,DUAL_END_SPATIAL_MASK_MVDR,71U));
    assert(state.stats.layout==DUAL_END_LAYOUT_DUAL_MONO);

    int16_t pcm[640];
    for(unsigned block=0;block<256U;++block) {
        for(unsigned i=0;i<320U;++i) {
            const unsigned n=phase++;
            const float noise=0.035f*sinf(2.0f*3.14159265358979323846f*1300.0f*(float)n/16000.0f);
            const float target=block>=80U ?
                0.16f*sinf(2.0f*3.14159265358979323846f*440.0f*(float)n/16000.0f) : 0.0f;
            pcm[2U*i]=sample(target+noise);
            pcm[2U*i+1U]=sample(target-noise);
        }
        assert(dual_end_spatial_process(&state,pcm,320U));
        for(unsigned i=0;i<320U;++i) assert(pcm[2U*i]==pcm[2U*i+1U]);
    }
    assert(state.mvdr.processed_hops==320U);
    assert(state.mvdr.valid_bins>0U);
    assert(state.mvdr.fallback_bins>0U);
    assert(state.mvdr.queue_underflows==0U&&state.mvdr.queue_overflows==0U);
    assert(state.stats.frames==256U*320U);

    dual_end_spatial_t before=state;
    assert(!dual_end_spatial_process(&state,pcm,319U));
    assert(!memcmp(&before,&state,sizeof(state)));

    assert(dual_end_spatial_reset(&state,6U,72U));
    for(unsigned block=0;block<8U;++block) {
        for(unsigned i=0;i<320U;++i) {
            pcm[2U*i]=sample(0.1f*sinf((float)(phase++)*0.07f));
            pcm[2U*i+1U]=0;
        }
        assert(dual_end_spatial_process(&state,pcm,320U));
    }
    assert(state.mvdr.fallback==4U);
    assert(state.mvdr.processed_hops==10U);

    /* A quiet but non-silent Whisper-like frame (~-70 dBFS) must not be
     * classified as digital silence by the absolute energy guard. */
    assert(dual_end_spatial_reset(&state,6U,74U));
    for(unsigned block=0;block<8U;++block) {
        for(unsigned i=0;i<320U;++i) {
            const int16_t quiet=sample(0.0003f*sinf((float)(phase++)*0.07f));
            pcm[2U*i]=pcm[2U*i+1U]=quiet;
        }
        assert(dual_end_spatial_process(&state,pcm,320U));
    }
    assert(state.mvdr.fallback!=1U);

    assert(dual_end_spatial_reset(&state,6U,73U));
    for(unsigned block=0;block<4U;++block) {
        for(unsigned i=0;i<320U;++i) pcm[2U*i]=pcm[2U*i+1U]=32767;
        assert(dual_end_spatial_process(&state,pcm,320U));
    }
    assert(state.mvdr.fallback==3U);
    for(unsigned i=0;i<320U;++i) assert(pcm[2U*i]==pcm[2U*i+1U]);
    puts("mode 6 soft-mask MVDR: stereo input, mono layout, finite bins, fallback, queue and reset PASS");
    return 0;
}
''', encoding="utf-8")
    exe, env = _build_host_executable(
        [harness, CODEC / "dual_end_spatial.c", CODEC / "dual_end_wiener.c", CODEC / "dual_end_mvdr.c"],
        tmp_path, extra_includes=(CODEC / "include",))
    assert "mode 6 soft-mask MVDR" in _run([str(exe)], cwd=tmp_path, env=env).stdout
