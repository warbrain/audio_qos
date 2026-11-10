"""Host-level functional checks for the production fixed-memory Wiener path.

Synthetic speech/noise verifies bounds, stream continuity, delay and a modest
SNR gain only. It is not an ESP32 timing, acoustic-box or ASR-quality result.
"""
from pathlib import Path

from test_audio_recovery_042 import _build_host_executable, _run

ROOT = Path(__file__).resolve().parents[1]
CODEC = ROOT / "components/dual_end_audio_codec"


def test_wiener_stream_is_bounded_continuous_and_reduces_stationary_noise(tmp_path):
    harness = tmp_path / "wiener_stream.c"
    harness.write_text(r'''
#include "dual_end_wiener.h"
#include "dual_end_spatial.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TOTAL_SECONDS 8U
#define RATE 16000U
#define TOTAL (TOTAL_SECONDS * RATE)
static dual_end_wiener_t state;
static dual_end_spatial_t spatial;
static int16_t input[TOTAL], output[TOTAL];
static uint32_t random_state=0x7a91c35dU;
static float noise_sample(void) {
    random_state=random_state*1664525U+1013904223U;
    return ((float)(random_state>>8)/(float)0x00ffffffU*2.0f-1.0f)*5200.0f;
}
int main(void) {
    const unsigned speech_start=2U*RATE, speech_end=7U*RATE;
    for(unsigned i=0;i<TOTAL;++i) {
        float noise=noise_sample(), speech=0.0f;
        if(i>=speech_start && i<speech_end) {
            float t=(float)(i-speech_start)/(float)RATE;
            float f0=126.0f+31.0f*sinf(2.0f*3.14159265f*0.55f*t);
            float phase=2.0f*3.14159265f*f0*t;
            float envelope=0.65f+0.35f*sinf(2.0f*3.14159265f*2.2f*t);
            speech=envelope*(4800.0f*sinf(phase)+2100.0f*sinf(2.0f*phase)+
                             1050.0f*sinf(3.0f*phase)+500.0f*sinf(4.0f*phase));
        }
        float mixed=speech+noise;
        if(mixed>32767.0f)mixed=32767.0f;
        if(mixed< -32768.0f)mixed= -32768.0f;
        input[i]=(int16_t)mixed;
    }
    dual_end_wiener_reset(&state);
    for(unsigned base=0;base<TOTAL;base+=320U) {
        int16_t block[640];
        for(unsigned i=0;i<320U;++i) block[2U*i]=block[2U*i+1U]=input[base+i];
        assert(dual_end_wiener_process(&state,block,320U));
        for(unsigned i=0;i<320U;++i) {
            assert(block[2U*i]==block[2U*i+1U]);
            output[base+i]=block[2U*i];
        }
    }
    assert(state.queue_underflows==0 && state.queue_overflows==0);
    assert(state.processed_hops==500); /* 8 s × 16 kHz / 256, startup frame included */
    assert(state.pending_count==0);
    assert(state.output_count<=DUAL_END_WIENER_OUTPUT_CAPACITY);

    double speech_energy=0.0, noise_energy=0.0, input_error=0.0, output_error=0.0;
    const unsigned delay=512U;
    for(unsigned n=speech_start+RATE/2U;n<speech_end-RATE/2U;++n) {
        float t=(float)(n-speech_start)/(float)RATE;
        float f0=126.0f+31.0f*sinf(2.0f*3.14159265f*0.55f*t);
        float phase=2.0f*3.14159265f*f0*t;
        float envelope=0.65f+0.35f*sinf(2.0f*3.14159265f*2.2f*t);
        double speech=envelope*(4800.0f*sinf(phase)+2100.0f*sinf(2.0f*phase)+
                                1050.0f*sinf(3.0f*phase)+500.0f*sinf(4.0f*phase));
        double source=input[n];
        double y=output[n+delay];
        speech_energy+=speech*speech;
        noise_energy+=(source-speech)*(source-speech);
        input_error+=(source-speech)*(source-speech);
        output_error+=(y-speech)*(y-speech);
    }
    const double input_snr=10.0*log10(speech_energy/input_error);
    const double output_snr=10.0*log10(speech_energy/output_error);
    printf("synthetic input_snr_db=%.2f output_snr_db=%.2f improvement_db=%.2f noise_energy=%.0f\n",
           input_snr,output_snr,output_snr-input_snr,noise_energy);
    assert(output_snr-input_snr>0.5);

    assert(dual_end_spatial_reset(&spatial,4U,23U));
    assert(spatial.stats.layout==DUAL_END_LAYOUT_DUAL_MONO);
    for(unsigned base=0;base<2560U;base+=320U) {
        int16_t block[640];
        for(unsigned i=0;i<320U;++i) block[2U*i]=block[2U*i+1U]=input[base+i];
        assert(dual_end_spatial_process(&spatial,block,320U));
        for(unsigned i=0;i<320U;++i)assert(block[2U*i]==block[2U*i+1U]);
    }
    assert(spatial.stats.mode==4U && spatial.stats.frames==2560U);
    assert(spatial.wiener.processed_hops==10U); /* 2560 / 256, including startup */
    assert(spatial.wiener.queue_underflows==0 && spatial.wiener.queue_overflows==0);

    assert(dual_end_spatial_reset(&spatial,5U,24U));
    assert(spatial.stats.layout==DUAL_END_LAYOUT_STEREO);
    assert(spatial.wiener.processed_hops==0U && spatial.wiener_right.processed_hops==0U);
    unsigned differing_channels=0;
    for(unsigned base=0;base<TOTAL;base+=320U) {
        int16_t block[640];
        for(unsigned i=0;i<320U;++i) {
            const int32_t left=input[base+i];
            const int32_t right=(left/3)+((int32_t)((base+i)%97U)-48)*41;
            block[2U*i]=(int16_t)left;
            block[2U*i+1U]=(int16_t)right;
        }
        assert(dual_end_spatial_process(&spatial,block,320U));
        for(unsigned i=0;i<320U;++i) {
            if(block[2U*i]!=block[2U*i+1U])++differing_channels;
        }
    }
    assert(spatial.stats.mode==5U && spatial.stats.layout==DUAL_END_LAYOUT_STEREO);
    assert(spatial.stats.frames==TOTAL);
    assert(differing_channels>TOTAL/2U); /* No L/R collapse or duplicate fallback. */
    assert(spatial.wiener.processed_hops==500U && spatial.wiener_right.processed_hops==500U);
    assert(spatial.wiener.noise_valid && spatial.wiener_right.noise_valid);
    assert(spatial.wiener.queue_underflows==0 && spatial.wiener_right.queue_underflows==0);
    assert(spatial.wiener.queue_overflows==0 && spatial.wiener_right.queue_overflows==0);
    assert(spatial.wiener.pending_count==0U && spatial.wiener_right.pending_count==0U);
    assert(spatial.wiener.noise_power[1]!=spatial.wiener_right.noise_power[1]);

    dual_end_wiener_t before=state;
    int16_t invalid[640];memset(invalid,0x5a,sizeof(invalid));
    int16_t invalid_before[640];memcpy(invalid_before,invalid,sizeof(invalid));
    assert(!dual_end_wiener_process(&state,invalid,319U));
    assert(!memcmp(&state,&before,sizeof(state)));
    assert(!memcmp(invalid,invalid_before,sizeof(invalid)));
    puts("PASS: bounded Wiener stream; mode 4 dual mono; mode 5 keeps independent L/R Wiener states; 32 ms alignment and queue health");
    return 0;
}
''', encoding="utf-8")
    exe, env = _build_host_executable(
        [harness, CODEC / "dual_end_spatial.c", CODEC / "dual_end_wiener.c"], tmp_path,
        extra_includes=(CODEC / "include",),
    )
    assert "PASS" in _run([str(exe)], cwd=tmp_path, env=env).stdout
