/* Exercise the production timeline, including identity-rate copy, rather than
 * a second buffer implementation. No ESP audio or USB device is opened. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "uac_test_boundary.h"

TickType_t uac_test_tick;
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4267)
#endif
#include "../../dongle_uac_bridge/dongle_uac_bridge.c"
#ifdef _MSC_VER
#pragma warning(pop)
#endif

static int16_t pattern(int64_t sample, unsigned channel)
{
    int32_t value = (int32_t)((sample * 97 + 31) % 60001) - 30000;
    return (int16_t)(channel ? -value : value);
}

static void fixture(int64_t cursor)
{
    timeline_reset_unlocked();
    s_draining = false;
    s_timeline_enabled = s_timeline_started = true;
    s_timeline_cursor = cursor;
    s_timeline_next_output_us = 1000000;
    s_timeline_last_read_us = 999000;
    s_timeline_tail = cursor + AUDIO_MS_TO_SAMPLES(242);
    s_uac_samples_from_buffer = s_uac_silence_fills = 0;
    s_level_drop_samples = s_level_dup_samples = 0;
    for (int64_t frame = cursor / 320; frame <= (cursor + 1000) / 320; ++frame) {
        audio_timeline_slot_t *slot = &s_timeline_slots[(uint64_t)frame % AUDIO_TIMELINE_SLOTS];
        slot->frame_index = frame;
        slot->present = true;
        for (unsigned i=0;i<320;++i)
            for (unsigned ch=0;ch<AUDIO_CHANNELS;++ch)
                slot->pcm[2*i+ch]=pattern(frame*320+i,ch);
    }
}

static void normal_case(int64_t cursor, bool hole, bool drain)
{
    fixture(cursor);
    if (hole) {
        int64_t missing = cursor / 320 + 1;
        s_timeline_slots[(uint64_t)missing % AUDIO_TIMELINE_SLOTS].present = false;
    }
    if (drain) {s_draining=true;s_timeline_tail=cursor+347;}
    int16_t expected[700*AUDIO_CHANNELS], out[700*AUDIO_CHANNELS];
    unsigned real=0; bool filled=false;
    /* An independent per-sample reference specifies holes/drain boundaries. */
    for(unsigned i=0;i<700;++i) {
        bool tail=drain&&cursor+i>=s_timeline_tail;
        int16_t value[AUDIO_CHANNELS]={0};
        bool present=!tail&&timeline_sample_unlocked(cursor+i,value);
        if(present) ++real; else if(!tail) filled=true;
        memcpy(&expected[2*i],value,sizeof(value));
    }
    assert(timeline_read_pcm((uint8_t*)out,sizeof(out),1000000)==sizeof(out));
    assert(memcmp(out,expected,sizeof(out))==0);
    assert(s_timeline_cursor==cursor+700);
    assert(s_uac_samples_from_buffer==real&&s_uac_silence_fills==(unsigned)filled);
}

static void drift_case(int correction)
{
    fixture(319);
    const int64_t cursor=s_timeline_cursor;
    s_timeline_drift_window_samples=AUDIO_TIMELINE_DRIFT_WINDOW_SAMPLES-16;
    const int64_t lead=AUDIO_MS_TO_SAMPLES(correction>0?250:230);
    s_timeline_tail=cursor+lead;
    s_timeline_drift_lead_sum=lead*(int64_t)s_timeline_drift_window_samples;
    int16_t out[16*AUDIO_CHANNELS];
    timeline_read_pcm((uint8_t*)out,sizeof(out),1000000);
    assert(s_timeline_cursor==cursor+16+correction);
    assert(s_level_drop_samples==(unsigned)(correction>0));
    assert(s_level_dup_samples==(unsigned)(correction<0));
    for(int64_t i=0;i<16;++i) {
        int64_t num=i*(16+correction), offset=num/16, fraction=num%16;
        for(unsigned ch=0;ch<AUDIO_CHANNELS;++ch) {
            int64_t value=(int64_t)pattern(cursor+offset,ch)*(16-fraction)+
                (int64_t)pattern(cursor+offset+1,ch)*fraction;
            assert(out[2*i+ch]==(int16_t)(value/16));
        }
    }
}

int main(void)
{
    uac_test_tick=1000;
    assert(dongle_uac_bridge_start()==ESP_OK);
    /* Cross frame and physical slot-31/0 wrap with asymmetric sample values. */
    normal_case(319,false,false);
    normal_case(31*320+313,false,false);
    normal_case(31*320+313,true,false);
    normal_case(319,false,true);
    normal_case(319,true,true);
    drift_case(1);drift_case(-1);
    dongle_uac_bridge_reset_session();
    int16_t out[32];memset(out,1,sizeof(out));
    assert(dongle_uac_bridge_read_pcm((uint8_t*)out,sizeof(out))==sizeof(out));
    for(unsigned i=0;i<32;++i) assert(out[i]==0);
    assert(!s_timeline_enabled&&!s_timeline_started&&s_timeline_cursor==0);
    puts("timeline bitwise copy/holes/slot wrap/drain/+1/-1/reset PASS");
    return 0;
}
