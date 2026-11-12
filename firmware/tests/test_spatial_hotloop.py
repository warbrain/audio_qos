"""Compare actual DSP against the frozen 0.5.10 kernel, not ASR or timing.

The reference preserves the pre-optimization accumulation order. Synthetic
PCM covers bypass, polarity, clipping, low energy, mode resets and NaN guards.
"""
from pathlib import Path
import pytest
from test_audio_recovery_042 import _build_host_executable, _run

ROOT = Path(__file__).resolve().parents[1]
CODEC = ROOT / 'components/dual_end_audio_codec'

@pytest.mark.parametrize('mode', [0, 1, 2])
def test_exact_pcm_and_matcher_state_against_0510(tmp_path, mode):
    reference = tmp_path / 'reference.c'
    reference.write_text(
        '#define dual_end_spatial_reset reference_reset\n'
        '#define dual_end_spatial_process reference_process\n'
        '#include "' + (ROOT / 'tests/reference/dual_end_spatial_0510.c').as_posix() + '"\n', encoding='utf-8')
    harness = tmp_path / 'spatial_exact.c'
    harness.write_text(r'''
#include "dual_end_spatial.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
bool reference_reset(dual_end_spatial_t *, uint16_t, uint16_t);
bool reference_process(dual_end_spatial_t *, int16_t *, size_t);
static dual_end_spatial_t before, after;
static uint32_t random_state=0x187f5771U;
static int16_t sample(void) {
    random_state=random_state*1664525U+1013904223U;
    return (int16_t)((int32_t)(random_state >> 17)-16384);
}
int main(void) {
    int16_t original[640], actual[640];
    const uint16_t initial_mode=MODE_VALUE;
    assert(reference_reset(&before,initial_mode,1));
    assert(dual_end_spatial_reset(&after,initial_mode,1));
    for(unsigned block=0; block<720; ++block) {
        if(block==240 || block==480) {
            uint16_t mode=(uint16_t)((initial_mode+block/240)%3);
            assert(reference_reset(&before,mode,(uint16_t)block));
            assert(dual_end_spatial_reset(&after,mode,(uint16_t)block));
        }
        unsigned scenario=block%12;
        for(unsigned i=0;i<320;++i) {
            int16_t value=sample();
            original[2*i]=value;
            original[2*i+1]=value;
            if(scenario==1)original[2*i+1]=(int16_t)(-value);
            if(scenario==2)original[2*i+1]=(int16_t)(value/2);
            if(scenario==3)original[2*i+1]=sample();
            if(scenario==4)original[2*i]=original[2*i+1]=0;
            if(scenario==5)original[2*i]=0;
            if(scenario==6)original[2*i+1]=0;
            if(scenario==7)original[2*i]=original[2*i+1]=(i&1)?32767:-32768;
            if(scenario==8)original[2*i+1]=(int16_t)(value/4+30);
            if(scenario==9)original[2*i]=original[2*i+1]=1;
        }
        if(scenario==10)before.coefficients[17]=after.coefficients[17]=NAN;
        if(scenario==11)before.coefficients[0]=after.coefficients[0]=20.0f;
        memcpy(actual,original,sizeof(actual));
        assert(reference_process(&before,original,320));
        assert(dual_end_spatial_process(&after,actual,320));
        assert(memcmp(original,actual,sizeof(actual))==0);
        assert(memcmp(&before,&after,sizeof(before))==0);
        dual_end_spatial_t unchanged=after;
        assert(!dual_end_spatial_process(&after,actual,319));
        assert(memcmp(&unchanged,&after,sizeof(after))==0);
    }
    puts("PASS: frozen 0.5.10 PCM/state exact through 720 blocks, wrap/reset/fallback/nonfinite");
    return 0;
}
'''.replace('MODE_VALUE', str(mode)), encoding='utf-8')
    exe, env = _build_host_executable([harness, reference, CODEC / 'dual_end_spatial.c', CODEC / 'dual_end_wiener.c', CODEC / 'dual_end_mvdr.c'], tmp_path,
        extra_includes=(CODEC / 'include',))
    assert 'PASS' in _run([str(exe)], cwd=tmp_path, env=env).stdout


def test_independent_nlms_stereo_keeps_channels_and_legacy_matcher(tmp_path):
    """Real kernel: no DAS sum, same safe matcher, explicit stereo contract.

    Compare complete matcher state to legacy mode 2 on identical source PCM;
    this establishes that the new output choice does not feed back into NLMS.
    Synthetic behavior does not claim acoustic quality or ASR improvement.
    """
    harness = tmp_path / 'nlms_stereo.c'
    harness.write_text(r'''
#include "dual_end_spatial.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static dual_end_spatial_t stereo, fused;
static uint32_t rng=0x5e327710U;
static int16_t value(void) {
    rng=rng*1664525U+1013904223U;
    return (int16_t)((int32_t)(rng>>19)-4096);
}
static int16_t rounded(float v) {
    if(!isfinite(v))return 0;
    v=fminf(32767.0f,fmaxf(-32768.0f,v));
    return (int16_t)(v>=0.0f?v+0.5f:v-0.5f);
}
static void reset(uint16_t generation) {
    assert(dual_end_spatial_reset(&stereo,DUAL_END_SPATIAL_NLMS_STEREO,generation));
    assert(dual_end_spatial_reset(&fused,DUAL_END_SPATIAL_NLMS,generation));
    assert(stereo.stats.layout==DUAL_END_LAYOUT_STEREO);
    assert(fused.stats.layout==DUAL_END_LAYOUT_DUAL_MONO);
}
static void compare(int16_t *pcm) {
    int16_t mono[640];memcpy(mono,pcm,sizeof(mono));
    assert(dual_end_spatial_process(&stereo,pcm,320));
    assert(dual_end_spatial_process(&fused,mono,320));
    for(unsigned i=0;i<320;++i) {
        assert(pcm[2*i]==rounded(stereo.aligned[0][i]));
        assert(mono[2*i]==mono[2*i+1]);
        /* Separate rounding can differ by one PCM16 count from the old sum. */
        float average=0.5f*((float)pcm[2*i]+(float)pcm[2*i+1]);
        assert(fabsf(average-(float)mono[2*i])<=1.0f);
    }
    dual_end_spatial_t normalized=stereo;
    normalized.stats.mode=DUAL_END_SPATIAL_NLMS;
    normalized.stats.layout=DUAL_END_LAYOUT_DUAL_MONO;
    assert(!memcmp(&normalized,&fused,sizeof(fused)));
}
int main(void) {
    assert(DUAL_END_SPATIAL_CAPABILITIES==63U);
    for(uint16_t caps=0;caps<64;++caps) {
        assert(dual_end_spatial_mode_supported(0,caps));
        assert(dual_end_spatial_mode_supported(1,caps)==((caps&1U)==1U));
        assert(dual_end_spatial_mode_supported(2,caps)==((caps&3U)==3U));
        assert(dual_end_spatial_mode_supported(3,caps)==((caps&4U)==4U));
        assert(dual_end_spatial_mode_supported(4,caps)==((caps&9U)==9U));
        assert(dual_end_spatial_mode_supported(5,caps)==((caps&16U)==16U));
        assert(dual_end_spatial_mode_supported(6,caps)==((caps&32U)==32U));
    }
    assert(!dual_end_spatial_mode_supported(3,7U&3U));
    assert(!dual_end_spatial_mode_supported(3,3U&7U));
    assert(!dual_end_spatial_mode_supported(6,31U));
    int16_t pcm[640];reset(17);
    for(unsigned i=0;i<320;++i){pcm[2*i]=30;pcm[2*i+1]=12;}
    compare(pcm);
    for(unsigned i=0;i<320;++i)assert(pcm[2*i]==30&&pcm[2*i+1]==12);
    assert(!stereo.stats.updated_frames&&!stereo.stats.mixed_frames);
    assert(stereo.stats.fallback==DUAL_END_SPATIAL_LOW_ENERGY);
    reset(18);double early=0.0,late=0.0;
    for(unsigned block=0;block<300;++block) {
        for(unsigned i=0;i<320;++i){int16_t v=value();pcm[2*i]=v;pcm[2*i+1]=(int16_t)(v/2);}
        compare(pcm);
        for(unsigned i=0;i<320;++i){double e=(double)pcm[2*i]-(double)pcm[2*i+1];
            if(block<10)early+=e*e;if(block>=290)late+=e*e;}
    }
    assert(late<early*0.5 && stereo.stats.updated_frames>90000);
    uint64_t updated=stereo.stats.updated_frames;
    for(unsigned i=0;i<320;++i){pcm[2*i]=30;pcm[2*i+1]=12;}
    compare(pcm);
    assert(stereo.stats.updated_frames==updated&&stereo.stats.matching_mix==0.0f);
    assert(pcm[638]==30&&pcm[639]==12);
    /* Weak correlation freezes; opposite wiring polarity is corrected by the
     * same matcher reset/alignment policy while retaining separate outputs. */
    reset(19);
    for(unsigned i=0;i<320;++i){pcm[2*i]=value();pcm[2*i+1]=value();}
    compare(pcm);assert(stereo.stats.fallback==DUAL_END_SPATIAL_LOW_CORRELATION);
    assert(!stereo.stats.updated_frames&&!stereo.stats.matching_mix);
    for(unsigned i=0;i<320;++i){int16_t v=value();pcm[2*i]=v;pcm[2*i+1]=(int16_t)(-v/2);}
    compare(pcm);assert(stereo.stats.polarity_inverted&&stereo.stats.resets==1);
    /* A failed microphone remains visible in stereo, never replaced by the
     * healthy microphone; adaptation freezes and the right mix is bypassed. */
    for(unsigned dead=0;dead<2;++dead) {
        reset((uint16_t)(20+dead));
        for(unsigned i=0;i<320;++i){int16_t v=value();pcm[2*i]=dead? v:0;pcm[2*i+1]=dead?0:v;}
        assert(dual_end_spatial_process(&stereo,pcm,320));
        assert(stereo.stats.fallback==DUAL_END_SPATIAL_CHANNEL_FAULT);
        assert(!stereo.stats.updated_frames&&stereo.stats.matching_mix==0.0f);
        for(unsigned i=0;i<320;++i)assert(pcm[2*i+(dead?1U:0U)]==0);
    }
    reset(25);stereo.coefficients[9]=NAN;fused.coefficients[9]=NAN;
    for(unsigned i=0;i<320;++i){int16_t v=value();pcm[2*i]=v;pcm[2*i+1]=(int16_t)(v/2);}
    compare(pcm);
    assert(stereo.stats.resets==1&&stereo.stats.matching_mix==0.0f);
    assert(stereo.stats.fallback==DUAL_END_SPATIAL_COEFFICIENT_FAULT);
    for(unsigned i=0;i<320;++i){pcm[2*i]=32767;pcm[2*i+1]=30000;}
    compare(pcm);assert(stereo.stats.fallback==DUAL_END_SPATIAL_CLIPPED);
    assert(pcm[638]==32767&&pcm[639]==30000);
    assert(dual_end_spatial_reset(&stereo,2,30));
    assert(dual_end_spatial_reset(&stereo,3,31));
    assert(stereo.stats.generation==31&&stereo.stats.layout==0&&stereo.coefficients[0]==1.0f);
    assert(!stereo.history_frames&&!stereo.stats.frames&&!stereo.stats.matching_mix);
    dual_end_spatial_t untouched=stereo;int16_t original[640];memcpy(original,pcm,sizeof(pcm));
    assert(!dual_end_spatial_reset(&stereo,7,32)&&!memcmp(&untouched,&stereo,sizeof(stereo)));
    assert(!dual_end_spatial_process(&stereo,pcm,319)&&!memcmp(&untouched,&stereo,sizeof(stereo)));
    stereo.stats.mode=7;untouched=stereo;
    assert(!dual_end_spatial_process(&stereo,pcm,320));
    assert(!memcmp(&untouched,&stereo,sizeof(stereo))&&!memcmp(original,pcm,sizeof(pcm)));
    puts("PASS: NLMS stereo channel separation, convergence, legacy matcher parity, caps, faults, reset and bounds");
    return 0;
}
''', encoding='utf-8')
    exe, env = _build_host_executable([harness, CODEC / 'dual_end_spatial.c', CODEC / 'dual_end_wiener.c', CODEC / 'dual_end_mvdr.c'], tmp_path,
        extra_includes=(CODEC / 'include',))
    assert 'PASS' in _run([str(exe)], cwd=tmp_path, env=env).stdout


@pytest.mark.parametrize(('mode', 'channels', 'clock_calls'), [(4, 1, 2), (5, 2, 4)])
def test_wiener_timing_isolated_and_pcm_state_unchanged(tmp_path, mode, channels, clock_calls):
    harness = tmp_path / 'spatial_wiener_timing.c'
    harness.write_text(r'''
#include "dual_end_spatial.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>
static unsigned clock_calls;
static int64_t fake_clock_us(void) { return (int64_t)(++clock_calls * 13U); }
int main(void) {
    dual_end_spatial_t plain={0}, timed={0};
    assert(dual_end_spatial_reset(&plain,MODE_VALUE,7));
    assert(dual_end_spatial_reset(&timed,MODE_VALUE,7));
    int16_t input[640], actual[640], expected[640];
    for(unsigned i=0;i<640;++i) input[i]=(int16_t)((i*97U)%12000U-6000);
    memcpy(actual,input,sizeof(input)); memcpy(expected,input,sizeof(input));
    dual_end_spatial_wiener_timing_t timing={0};
    assert(dual_end_spatial_process(&plain,expected,320));
    assert(dual_end_spatial_process_timed(&timed,actual,320,fake_clock_us,&timing));
    assert(memcmp(actual,expected,sizeof(actual))==0);
    assert(memcmp(&plain,&timed,sizeof(plain))==0);
    assert(timing.valid && timing.channels==EXPECTED_CHANNELS);
    assert(clock_calls==EXPECTED_CLOCK_CALLS);
    for(unsigned channel=0;channel<EXPECTED_CHANNELS;++channel)
        assert(timing.us[channel]==13U);
    return 0;
}
'''.replace('MODE_VALUE', str(mode)).replace('EXPECTED_CHANNELS', str(channels))
       .replace('EXPECTED_CLOCK_CALLS', str(clock_calls)), encoding='utf-8')
    exe, env = _build_host_executable(
        [harness, CODEC / 'dual_end_spatial.c', CODEC / 'dual_end_wiener.c', CODEC / 'dual_end_mvdr.c'],
        tmp_path, extra_includes=(CODEC / 'include',))
    _run([str(exe)], cwd=tmp_path, env=env)
