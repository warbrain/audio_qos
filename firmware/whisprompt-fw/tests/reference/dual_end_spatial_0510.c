#include "dual_end_spatial.h"
#include <math.h>
#include <string.h>
#include <limits.h>

#define MIN_RMS 64.0f
#define MIN_CORRELATION 0.80f
#define STEP 0.02f
#define EPSILON 1e-8f
#define FADE (1.0f / 256.0f)
_Static_assert(sizeof(((dual_end_spatial_t *)0)->aligned[0]) % 16 == 0, "aligned channel stride");

static float bounded(float value, float minimum, float maximum) {
    return value < minimum ? minimum : value > maximum ? maximum : value;
}
static int16_t pcm16(float value) {
    if (!isfinite(value)) return 0;
    value=bounded(value, (float)INT16_MIN, (float)INT16_MAX);
    return (int16_t)(value >= 0.0f ? value+0.5f : value-0.5f);
}
static void reset_matcher(dual_end_spatial_t *s) {
    memset(s->coefficients,0,sizeof(s->coefficients));
    memset(s->history,0,sizeof(s->history));
    s->coefficients[0]=1.0f; s->head=0; s->history_frames=0;
    s->history_power=0.0f; s->stats.matching_mix=0.0f;
    s->stats.coefficient_l1=1.0f;
}
bool dual_end_spatial_reset(dual_end_spatial_t *s,uint16_t mode,uint16_t generation) {
    if (!s || mode > DUAL_END_SPATIAL_NLMS) return false;
    memset(s,0,sizeof(*s));
    s->stats.mode=mode; s->stats.generation=generation;
    s->stats.layout=mode ? DUAL_END_LAYOUT_DUAL_MONO : DUAL_END_LAYOUT_STEREO;
    reset_matcher(s); return true;
}

/* Cross-correlation is bounded by the physical aperture: < 1 sample at 16k.
 * Three integer-lag points and parabolic interpolation avoid an FFT in the
 * realtime owner. This is a local fractional-delay estimator, not GCC-PHAT or
 * calibrated DOA. Weak/ambiguous frames hold the previous alignment. */
static float lag_correlation(const int16_t *pcm,int lag) {
    float cross=0.0f,left=0.0f,right=0.0f;
    for (size_t i=1;i+1<DUAL_END_SPATIAL_FRAME_SAMPLES;++i) {
        float l=(float)pcm[2*i] / 32768.0f;
        float r=(float)pcm[2*((int)i+lag)+1] / 32768.0f;
        cross+=l*r; left+=l*l; right+=r*r;
    }
    float denominator=sqrtf(left*right);
    return denominator > EPSILON ? bounded(cross/denominator,-1.0f,1.0f) : 0.0f;
}
static float delayed(const int16_t *pcm,size_t i,size_t channel,float delay,float previous) {
    float now=(float)pcm[2*i+channel];
    float before=i ? (float)pcm[2*(i-1)+channel] : previous;
    return now*(1.0f-delay)+before*delay;
}
bool dual_end_spatial_process(dual_end_spatial_t *s,int16_t *pcm,size_t frames) {
    if (!s || !pcm || frames != DUAL_END_SPATIAL_FRAME_SAMPLES || s->stats.mode > 2) return false;
    if (s->stats.mode==DUAL_END_SPATIAL_BYPASS) return true;
    float energy[2]={0,0}; int peak[2]={0,0}; unsigned clipped=0;
    for (size_t i=0;i<frames;++i) {
        for (size_t ch=0;ch<2;++ch) {
            int v=pcm[2*i+ch], magnitude=v < 0 ? -v : v;
            float normalized=(float)v/32768.0f;
            energy[ch]+=normalized*normalized;
            if (magnitude>peak[ch]) peak[ch]=magnitude;
            if (magnitude>=32700) ++clipped;
        }
    }
    float rms[2]={sqrtf(energy[0]/(float)frames)*32768.0f,sqrtf(energy[1]/(float)frames)*32768.0f};
    bool fault=(peak[0]<=2 && rms[1]>=MIN_RMS) || (peak[1]<=2 && rms[0]>=MIN_RMS);
    bool low=rms[0]<MIN_RMS || rms[1]<MIN_RMS;
    float correlations[3]={lag_correlation(pcm,-1),lag_correlation(pcm,0),lag_correlation(pcm,1)};
    unsigned best=1;
    for (unsigned i=0;i<3;++i) if (fabsf(correlations[i])>fabsf(correlations[best])) best=i;
    float correlation=fabsf(correlations[best]);
    s->stats.correlation=correlation;
    bool reliable=!fault && !low && !clipped && correlation>=MIN_CORRELATION;
    float initial_delay=s->stats.delay_samples,target_delay=initial_delay;
    if (reliable) {
        float values[3]={fabsf(correlations[0]),fabsf(correlations[1]),fabsf(correlations[2])};
        float curvature=values[0]-2.0f*values[1]+values[2];
        float estimate=(float)((int)best-1);
        if (best==1 && curvature < -EPSILON) estimate=0.5f*(values[0]-values[2])/curvature;
        estimate=bounded(estimate,-DUAL_END_SPATIAL_MAX_DELAY,DUAL_END_SPATIAL_MAX_DELAY);
        target_delay=initial_delay+0.2f*(estimate-initial_delay);
        bool inverted=correlations[best]<0.0f;
        if (inverted!=s->stats.polarity_inverted) { reset_matcher(s); ++s->stats.resets; }
        s->stats.polarity_inverted=inverted;
    }
    uint16_t reason=fault ? DUAL_END_SPATIAL_CHANNEL_FAULT : clipped ? DUAL_END_SPATIAL_CLIPPED :
        low ? DUAL_END_SPATIAL_LOW_ENERGY : correlation<MIN_CORRELATION ? DUAL_END_SPATIAL_LOW_CORRELATION : DUAL_END_SPATIAL_OK;
    if (s->stats.mode==DUAL_END_SPATIAL_NLMS) {
        bool finite=true;
        for(size_t k=0;k<DUAL_END_SPATIAL_TAPS;++k) finite=finite && isfinite(s->coefficients[k]);
        if(!finite) { reset_matcher(s); ++s->stats.resets; reliable=false; reason=DUAL_END_SPATIAL_COEFFICIENT_FAULT; }
    }
    /* Delay the earlier channel causally; never read future samples or add a
     * lookahead block. A linear ramp prevents a delay jump at block boundaries. */
    for (size_t i=0;i<frames;++i) {
        float delay=initial_delay+(target_delay-initial_delay)*(float)(i+1)/(float)frames;
        s->aligned[0][i]=delayed(pcm,i,0,delay>0.0f?delay:0.0f,s->previous[0]);
        s->aligned[1][i]=delayed(pcm,i,1,delay<0.0f?-delay:0.0f,s->previous[1])*(s->stats.polarity_inverted?-1.0f:1.0f);
    }
    s->previous[0]=(float)pcm[2*(frames-1)]; s->previous[1]=(float)pcm[2*(frames-1)+1];
    s->stats.delay_samples=target_delay;
    bool adapt=s->stats.mode==DUAL_END_SPATIAL_NLMS && reliable;
    if (!adapt) s->history_frames=0;
    uint64_t updated_before=s->stats.updated_frames;
    for (size_t i=0;i<frames;++i) {
        float left=s->aligned[0][i],right=s->aligned[1][i];
        float output_right=right;
        if (s->stats.mode==DUAL_END_SPATIAL_NLMS) {
            s->head=(s->head+DUAL_END_SPATIAL_TAPS-1)%DUAL_END_SPATIAL_TAPS;
            float r=right/32768.0f,old=s->history[s->head];
            s->history[s->head]=r;
            s->history_power=fmaxf(0.0f,s->history_power+r*r-old*old);
            if (adapt && s->history_frames<DUAL_END_SPATIAL_TAPS) ++s->history_frames;
            float matched=0.0f;
            for (size_t k=0;k<DUAL_END_SPATIAL_TAPS;++k) matched+=s->coefficients[k]*s->history[(s->head+k)%DUAL_END_SPATIAL_TAPS];
            bool updated=adapt && s->history_frames>=DUAL_END_SPATIAL_TAPS && s->history_power>EPSILON;
            if (updated) {
                float scale=STEP*(left/32768.0f-matched)/(s->history_power+EPSILON);
                float other=0.0f; bool finite=true;
                for (size_t k=0;k<DUAL_END_SPATIAL_TAPS;++k) {
                    s->coefficients[k]+=scale*s->history[(s->head+k)%DUAL_END_SPATIAL_TAPS];
                    finite=finite && isfinite(s->coefficients[k]);
                    if (k) other+=fabsf(s->coefficients[k]);
                }
                if (!finite) {
                    reset_matcher(s); ++s->stats.resets; adapt=false;
                    reason=DUAL_END_SPATIAL_COEFFICIENT_FAULT; updated=false;
                } else {
                    float direct=bounded(s->coefficients[0],0.25f,4.0f),room=4.0f-direct;
                    if (direct!=s->coefficients[0] || other>room) {
                        ++s->stats.bounded_updates;
                        s->coefficients[0]=direct;
                        float factor=other>room && other>0.0f ? room/other : 1.0f;
                        for (size_t k=1;k<DUAL_END_SPATIAL_TAPS;++k) s->coefficients[k]*=factor;
                    }
                    s->stats.coefficient_l1=direct+fminf(other,room);
                }
            }
            if (updated) ++s->stats.updated_frames; else ++s->stats.frozen_frames;
            float target=updated?1.0f:0.0f;
            s->stats.matching_mix+=bounded(target-s->stats.matching_mix,-FADE,FADE);
            if (fault || clipped || !isfinite(matched)) s->stats.matching_mix=0.0f;
            output_right=isfinite(matched) ? right+s->stats.matching_mix*(matched*32768.0f-right) : right;
            if (s->stats.matching_mix>0.0f) ++s->stats.mixed_frames;
            s->stats.matching_mix_q16_sum+=(uint64_t)(s->stats.matching_mix*65535.0f+0.5f);
            if (!updated) ++s->stats.fallback_frames;
        }
        float mono=fault ? (peak[0]<=2 ? right:left) : 0.5f*(left+output_right);
        int16_t sample=pcm16(mono);
        pcm[2*i]=sample; pcm[2*i+1]=sample;
    }
    s->stats.frames+=frames;
    if (adapt && s->stats.updated_frames==updated_before) reason=DUAL_END_SPATIAL_WARMUP;
    s->stats.fallback=reason;
    return true;
}
