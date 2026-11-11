#include "dual_end_spatial.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool reference_reset(dual_end_spatial_t *, uint16_t, uint16_t);
bool reference_process(dual_end_spatial_t *, int16_t *, size_t);
uint64_t actual_predictions_take(void);
uint64_t reference_predictions_take(void);

static dual_end_spatial_t before, after;
static int16_t original[2*DUAL_END_SPATIAL_FRAME_SAMPLES];
static int16_t expected[2*DUAL_END_SPATIAL_FRAME_SAMPLES];
static int16_t actual[2*DUAL_END_SPATIAL_FRAME_SAMPLES];
static unsigned blocks;
static uint64_t actual_work, reference_work;
static uint32_t random_state=0x5f345517U;

static void exact(const void *a,const void *b,size_t bytes,const char *kind) {
    if(memcmp(a,b,bytes)!=0) {
        const unsigned char *left=a,*right=b;
        size_t offset=0;
        while(offset<bytes && left[offset]==right[offset]) ++offset;
        fprintf(stderr,"block %u %s differs at byte %zu: %u != %u\n",
            blocks,kind,offset,(unsigned)left[offset],(unsigned)right[offset]);
        abort();
    }
}
static void reset(uint16_t mode,uint16_t generation) {
    assert(reference_reset(&before,mode,generation));
    assert(dual_end_spatial_reset(&after,mode,generation));
    exact(&before,&after,sizeof(before),"reset state");
}
static int16_t random_sample(void) {
    random_state=random_state*1664525U+1013904223U;
    return (int16_t)((int32_t)(random_state>>17)-16384);
}
/* Small random pairs are deliberately below MIN_RMS; large correlated pairs
 * exercise actual training and polarity; independent pairs freeze learning. */
static void fill(unsigned scenario) {
    for(size_t i=0;i<DUAL_END_SPATIAL_FRAME_SAMPLES;++i) {
        int16_t value=random_sample();
        int16_t left=value,right=value;
        switch(scenario) {
            case 0:left=right=0;break;
            case 1:left=(int16_t)(value/1000);right=(int16_t)(value/1200);break;
            case 2:right=(int16_t)(value/2);break;
            case 3:right=random_sample();break;
            case 4:left=0;break;
            case 5:right=0;break;
            case 6:left=right=(i&1U)?32767:-32768;break;
            case 7:right=(int16_t)(-value);break;
            case 8:left=right=3;break;
            case 9:left=right=63;break;
            case 10:left=right=64;break;
            default:break;
        }
        original[2*i]=left;original[2*i+1]=right;
    }
}
static uint64_t step(void) {
    uint16_t mode=before.stats.mode;
    memcpy(expected,original,sizeof(expected));
    memcpy(actual,original,sizeof(actual));
    assert(reference_process(&before,expected,DUAL_END_SPATIAL_FRAME_SAMPLES));
    assert(dual_end_spatial_process(&after,actual,DUAL_END_SPATIAL_FRAME_SAMPLES));
    ++blocks;
    exact(expected,actual,sizeof(expected),"PCM");
    exact(&before,&after,sizeof(before),"complete state");
    uint64_t reference_calls=reference_predictions_take();
    uint64_t actual_calls=actual_predictions_take();
    assert(reference_calls==(mode==DUAL_END_SPATIAL_NLMS?320U:0U));
    assert(actual_calls<=reference_calls);
    reference_work+=reference_calls;actual_work+=actual_calls;
    return actual_calls;
}
static void expect_calls(uint64_t count) { assert(step()==count); }
static void train(void) {
    for(unsigned i=0;i<24;++i) { fill(2);expect_calls(320); }
    assert(after.stats.matching_mix==1.0f);
    assert(after.history_frames==DUAL_END_SPATIAL_TAPS);
    assert(after.stats.updated_frames==24U*320U-63U);
    assert(after.stats.frozen_frames==63U);
    assert(after.stats.fallback_frames==63U);
    assert(after.stats.mixed_frames==24U*320U-63U);
}
static void quiet(void) {
    reset(DUAL_END_SPATIAL_NLMS,1);train();
    fill(0);expect_calls(256); /* Predict every sample until fade reaches zero. */
    uint64_t learned=after.stats.updated_frames;
    for(unsigned i=0;i<1500;++i) { fill(i%2U);expect_calls(0); }
    assert(after.stats.updated_frames==learned);
    assert(after.stats.frozen_frames==63U+1501U*320U);
    assert(after.stats.fallback_frames==after.stats.frozen_frames);
    assert(after.history_frames==0U && after.stats.matching_mix==0.0f);
    fill(2);expect_calls(320); /* Warmup and reliable frame must never skip. */
    assert(after.stats.updated_frames==learned+257U);
}
static void transitions(void) {
    reset(DUAL_END_SPATIAL_NLMS,2);
    for(unsigned round=0;round<160;++round) {
        for(unsigned scenario=0;scenario<11;++scenario) {
            fill(scenario);
            (void)step();
        }
    }
    assert(after.stats.resets>0U); /* Polarity transitions exercise reset. */
    assert(after.stats.updated_frames>0U && after.stats.frozen_frames>0U);
    assert(actual_work<reference_work);
}
static void fade(void) {
    reset(DUAL_END_SPATIAL_NLMS,3);train();
    fill(3);expect_calls(256);
    fill(1);expect_calls(0);
    fill(2);expect_calls(320); /* First 63 samples still compute warmup dot. */
    assert(after.stats.matching_mix==1.0f);
    fill(4);expect_calls(1); /* Fault forces mix to zero on the first sample. */
    assert(after.stats.fallback==DUAL_END_SPATIAL_CHANNEL_FAULT);
    fill(2);expect_calls(320);
    fill(6);expect_calls(1); /* Clipping also immediately zeroes the mix. */
    assert(after.stats.fallback==DUAL_END_SPATIAL_CLIPPED);
    fill(6);expect_calls(0);
    reset(DUAL_END_SPATIAL_NLMS,4);
    before.stats.matching_mix=after.stats.matching_mix=1.0f/256.0f;
    fill(1);expect_calls(1);
    before.stats.matching_mix=after.stats.matching_mix=-0.0f;
    fill(1);expect_calls(0);
    assert(!signbit(after.stats.matching_mix));
}
static void nonfinite(void) {
    const float poison[3]={NAN,INFINITY,-INFINITY};
    for(unsigned type=0;type<3;++type) {
        for(unsigned k=0;k<DUAL_END_SPATIAL_TAPS;++k) {
            reset(DUAL_END_SPATIAL_NLMS,(uint16_t)k);
            before.coefficients[k]=after.coefficients[k]=poison[type];
            fill(2);(void)step();
            assert(after.stats.resets==1U);
            assert(after.stats.fallback==DUAL_END_SPATIAL_COEFFICIENT_FAULT);
            fill(0);expect_calls(0);
        }
        reset(DUAL_END_SPATIAL_NLMS,10);
        before.history[17]=after.history[17]=poison[type];
        before.history_power=after.history_power=poison[type];
        fill(1);expect_calls(320); /* Corrupt history retains old dot handling. */
        fill(0);expect_calls(0);
        reset(DUAL_END_SPATIAL_NLMS,11);
        before.previous[1]=after.previous[1]=poison[type];
        fill(1);(void)step();
        reset(DUAL_END_SPATIAL_NLMS,12);
        before.stats.delay_samples=after.stats.delay_samples=poison[type];
        fill(1);(void)step();
    }
}
static void finite_overflow(void) {
    reset(DUAL_END_SPATIAL_NLMS,20);
    before.coefficients[0]=after.coefficients[0]=FLT_MAX/2.0f;
    fill(8);expect_calls(320);
    /* The old finite prediction overflows during PCM scaling even at mix=0.
     * A simple unconditional zero-mix bypass would incorrectly emit 3 here. */
    for(size_t i=0;i<640;++i) assert(actual[i]==0);
    assert(after.stats.resets==0U && after.stats.matching_mix==0.0f);
    reset(DUAL_END_SPATIAL_NLMS,21);
    before.coefficients[17]=after.coefficients[17]=0.1f;
    for(unsigned k=0;k<64;++k) before.history[k]=after.history[k]=FLT_MAX/2.0f;
    fill(8);expect_calls(320);
    fill(8);expect_calls(0); /* The abnormal history has now been fully drained. */
    reset(DUAL_END_SPATIAL_NLMS,22);
    before.previous[1]=after.previous[1]=FLT_MAX;
    before.stats.delay_samples=after.stats.delay_samples=-0.5f;
    fill(1);expect_calls(320);
    fill(1);expect_calls(0);
}
static void modes(void) {
    for(unsigned round=0;round<90;++round) {
        reset((uint16_t)(round%3U),(uint16_t)(65535U-round));
        for(unsigned scenario=0;scenario<11;++scenario) { fill(scenario);(void)step(); }
        dual_end_spatial_t unchanged=after;
        memcpy(actual,original,sizeof(actual));
        assert(!dual_end_spatial_process(&after,actual,319));
        assert(!dual_end_spatial_process(&after,NULL,320));
        /* Mode 4 is DAS + Wiener; mode 6 is outside the protocol. */
        assert(!dual_end_spatial_reset(&after,6,0));
        assert(!dual_end_spatial_process(NULL,actual,320));
        exact(&unchanged,&after,sizeof(after),"invalid unchanged state");
        exact(actual,original,sizeof(actual),"invalid unchanged PCM");
    }
}
int main(int argc,char **argv) {
    assert(argc==2);
    if(strcmp(argv[1],"quiet")==0)quiet();
    else if(strcmp(argv[1],"transitions")==0)transitions();
    else if(strcmp(argv[1],"fade")==0)fade();
    else if(strcmp(argv[1],"nonfinite")==0)nonfinite();
    else if(strcmp(argv[1],"finite_overflow")==0)finite_overflow();
    else if(strcmp(argv[1],"modes")==0)modes();
    else assert(0);
    printf("PASS: %s, %u blocks exact PCM/full state; predictions %llu vs %llu\n",
        argv[1],blocks,(unsigned long long)actual_work,(unsigned long long)reference_work);
    return 0;
}
