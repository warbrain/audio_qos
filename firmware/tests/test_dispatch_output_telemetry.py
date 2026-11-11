"""Portable production diagnostics and real sender-event ownership checks."""
import json
from pathlib import Path

from test_audio_recovery_042 import _build_host_executable, _run
from test_runtime_codec import function

ROOT = Path(__file__).resolve().parents[1]
MIC = ROOT / "scepter/main/mic"


def run_c(tmp_path, code, sources=()):
    harness = tmp_path / "dispatch_output.c"
    harness.write_text(code, encoding="utf-8")
    (tmp_path / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n", encoding="utf-8")
    exe, env = _build_host_executable([harness, *sources], tmp_path,
        extra_includes=[MIC, ROOT / "components/dual_end_runtime/include",
                        ROOT / "components/dual_end_audio_codec/include",
                        ROOT / "scepter/components/dual_end_audio/include", tmp_path])
    return _run([str(exe)], cwd=tmp_path, env=env).stdout


def test_additive_events_bounds_unknown_and_codec5(tmp_path):
    code = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mic_pipeline_telemetry.h"
int main(void) {
    mic_pipeline_telemetry_t r = {.boot_id=UINT32_MAX,.session_id=UINT32_MAX,
        .sequence=UINT32_MAX,.sampled_us=UINT64_MAX,.codec=5,.codec_known=true,.mode=5};
    r.dispatch_queue_wait=r.dispatch_control_wait=r.dispatch_frame_age=
        (mic_pipeline_metric_t){UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT64_MAX};
    r.dispatch_queue_full=r.dispatch_invalidated=r.dispatch_unavailable=
        r.dispatch_event_lost=r.dispatch_queue_highwater=r.dispatch_late_events=UINT32_MAX;
    r.capture_health.output=(mic_output_health_t){.samples=UINT64_MAX,
        .sum_sq={UINT64_MAX,UINT64_MAX},.limited_frames=UINT64_MAX,
        .peak={UINT32_MAX,UINT32_MAX}};
    for (unsigned unknown=0;unknown<2;++unknown) {
        if (unknown) {
            r.codec_known=false;
            r.dispatch_queue_wait.n=r.dispatch_control_wait.n=r.dispatch_frame_age.n=0;
            r.capture_health.output=(mic_output_health_t){0};
        }
        for (unsigned kind=0;kind<2;++kind) {
            const unsigned parts=kind ? MIC_OUTPUT_HEALTH_TELEMETRY_PARTS : MIC_DISPATCH_TELEMETRY_PARTS;
            for (unsigned part=0;part<parts;++part) {
                unsigned char memory[183];memset(memory,0xa5,sizeof(memory));
                char *line=(char *)memory;
                size_t n=kind ? mic_output_health_telemetry_format(&r,part,line,181) :
                    mic_dispatch_telemetry_format(&r,part,line,181);
                assert(n>0&&n<=180&&n==strlen(line));
                assert(memory[181]==0xa5&&memory[182]==0xa5);puts(line);
                size_t rejected=kind ? mic_output_health_telemetry_format(&r,part,line,n) :
                    mic_dispatch_telemetry_format(&r,part,line,n);
                assert(!rejected&&line[0]=='\0'&&memory[181]==0xa5);
            }
        }
    }
    char line[181];r.codec_known=true;r.codec=99;
    assert(!mic_dispatch_telemetry_format(&r,0,line,sizeof(line)));
    assert(!mic_output_health_telemetry_format(&r,0,line,sizeof(line)));
    r.codec=5;assert(mic_pipeline_telemetry_format(&r,0,line,sizeof(line)));
    assert(mic_capture_health_telemetry_format(&r,0,line,sizeof(line)));
    return 0;
}
'''
    raw = run_c(tmp_path, code, [MIC / "mic_pipeline_telemetry.c", MIC / "mic_observability_telemetry.c"])
    rows = [json.loads(line) for line in raw.splitlines()]
    assert len(rows) == 48
    assert all(len(line.encode()) <= 180 for line in raw.splitlines())
    for start in (0, 24):
        for offset, count in ((0, 17), (17, 7)):
            group = rows[start + offset:start + offset + count]
            assert [row["part"] for row in group] == list(range(count))
            assert group[0]["part_count"] == count
    dispatch = {key: value for row in rows[:17] for key, value in row.items()}
    unknown = {key: value for row in rows[24:41] for key, value in row.items()}
    output = {key: value for row in rows[41:] for key, value in row.items()}
    assert dispatch["qw_us"] == str(2**64 - 1) and dispatch["queue_full"] == 2**32 - 1
    assert unknown["qw_lo"] is None and unknown["fa_hi"] is None
    assert output["l_rms"] is None and output["l_peak"] is None and output["limited_pct"] is None


def test_output_health_merges_actual_energy_and_limiter_counts(tmp_path):
    code = r'''
#include <assert.h>
#include <stdio.h>
#include "mic_pipeline_telemetry.h"
int main(void) {
    mic_capture_health_t window={0};
    mic_capture_health_t block={.output={.samples=320,.sum_sq={3200000,12800000},
        .limited_frames=32,.peak={120,240}}};
    for (unsigned i=0;i<960;++i) mic_capture_health_record(&block,10,-20);
    mic_capture_health_complete_block(&block);
    mic_capture_health_merge(&window,&block);mic_capture_health_merge(&window,&block);
    assert(window.samples[0]==1920&&window.sum_sq[0]==192000);
    assert(window.output.samples==640&&window.output.sum_sq[0]==6400000);
    assert(window.output.limited_frames==64&&window.output.peak[1]==240);
    mic_pipeline_telemetry_t r={.boot_id=1,.codec=4,.codec_known=true,.capture_health=window};
    char line[181];
    for(unsigned p=0;p<MIC_OUTPUT_HEALTH_TELEMETRY_PARTS;++p) {
        assert(mic_output_health_telemetry_format(&r,p,line,sizeof(line)));puts(line);
    }
    return 0;
}
'''
    rows = [json.loads(line) for line in run_c(tmp_path, code, [MIC / "mic_observability_telemetry.c"]).splitlines()]
    merged = {key: value for row in rows for key, value in row.items()}
    assert merged["samples"] == "640" and merged["limited_frames"] == "64"
    assert merged["l_rms"] == 100 and merged["r_rms"] == 200 and merged["limited_pct"] == 10


def test_real_completion_joins_only_matching_windows_and_monotonic_times(tmp_path):
    source = (MIC / "mic_audio.c").read_text(encoding="utf-8")
    event_start = source.index("typedef struct {\n    dual_end_audio_owner_timing_t timing;")
    event_end = source.index("} mic_owner_timing_event_t;", event_start) + len("} mic_owner_timing_event_t;")
    code = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "mic_pipeline_telemetry.h"
typedef struct { uint32_t session_id,mutex_wait_us,encode_us,radio_submit_us;
 uint8_t codec;bool valid,encode_attempted,encode_ok,radio_submitted,radio_ok;unsigned outcome;
} dual_end_audio_owner_timing_t;
''' + source[event_start:event_end] + r'''
static mic_pipeline_telemetry_t s_pipeline_window;
static bool s_pipeline_window_active=true;
static unsigned records,locked;
static int s_pipeline_lock;
static uint32_t s_frame_dispatch_late_events;
#define DUAL_END_STAGE_MIC_PIPELINE 1
#define portENTER_CRITICAL(p) ((void)(p),assert(!locked),++locked)
#define portEXIT_CRITICAL(p) ((void)(p),assert(locked),--locked)
static uint32_t mic_frame_dispatch_epoch_load(void){return 7;}
static void dual_end_runtime_record_outcome(unsigned stage,unsigned codec,unsigned session,unsigned us,unsigned outcome){
 (void)stage;(void)codec;(void)session;(void)us;(void)outcome;++records;}
''' + function(source, "mic_pipeline_record_owner_timing_event") + r'''
int main(void){
 s_pipeline_window=(mic_pipeline_telemetry_t){.boot_id=1,.session_id=2,.codec=4,
 .codec_known=true,.generation=3,.capture_generation=4,.start_us=150};
 mic_owner_timing_event_t e={.timing={.session_id=2,.codec=4,.valid=true},
 .boot_id=1,.spatial_generation=3,.capture_generation=4,.dispatch_epoch=7,
 .captured_us=100,.enqueued_us=200,.dequeued_us=1200,.control_ready_us=5200,.callback_started_us=5300};
 mic_pipeline_record_owner_timing_event(&e);
 assert(records==1&&s_pipeline_window.dispatch_queue_wait.us==1000);
 assert(s_pipeline_window.dispatch_control_wait.us==4000&&s_pipeline_window.dispatch_frame_age.us==5200);
 e.dispatch_epoch=8;mic_pipeline_record_owner_timing_event(&e);assert(records==1);
 e.dispatch_epoch=7;e.timing.session_id=99;mic_pipeline_record_owner_timing_event(&e);
 assert(s_frame_dispatch_late_events==2&&s_pipeline_window.dispatch_queue_wait.n==1);
 e.timing.session_id=2;e.enqueued_us=149;mic_pipeline_record_owner_timing_event(&e);
 assert(s_frame_dispatch_late_events==3&&s_pipeline_window.dispatch_queue_wait.n==1);
 e.timing.session_id=2;e.enqueued_us=1300;e.control_ready_us=1100;e.captured_us=6000;
 mic_pipeline_record_owner_timing_event(&e);
 assert(s_pipeline_window.dispatch_queue_wait.n==1&&s_pipeline_window.dispatch_control_wait.n==1);
 assert(s_pipeline_window.dispatch_frame_age.n==1&&!locked);puts("completion queue/control/frame ages PASS");return 0;
}
'''
    assert "PASS" in run_c(tmp_path, code)


def test_actual_capture_loop_output_matches_queued_pcm_across_rollover_and_gain(tmp_path):
    """Execute the production sample loop, interval reset and gain applicator.

    Only scheduling/transport and mode-zero spatial identity are stubbed. The
    PCM energy oracle reads the actual queue copy independently of the loop's
    statistics. No DSP/statistics implementation is copied into this test.
    """
    source = (MIC / "mic_audio.c").read_text(encoding="utf-8")
    def implementation(name):
        cursor = 0
        while True:
            name_at = source.index(name + "(", cursor)
            opening = source.index("{", name_at)
            semicolon = source.find(";", name_at, opening)
            if semicolon < 0:
                start = source.rfind("\n", 0, name_at) + 1
                return function(source[start:], name)
            cursor = semicolon + 1
    task = implementation("mic_audio_task")
    local_start = task.index("    size_t pcm_frames = 0;")
    local_end = task.index("    s_capture_blocks=xQueueCreate", local_start)
    locals_source = task[local_start:local_end]
    processing_start = task.index("        stats_input_frames +=")
    processing_end = task.rfind("\n    }\n}")
    assert processing_end > processing_start
    processing = task[processing_start:processing_end]
    assert "capture_health.output.sum_sq" in processing and "s_soft_limited_frames = 0;" in processing
    types = []
    for begin, end in (("typedef struct {\n    uint32_t sequence, generation;", "} mic_capture_block_t;"),
                       ("typedef struct {\n    mic_audio_frame_cb_t callback;", "} mic_frame_dispatch_t;")):
        start = source.index(begin)
        stop = source.index(end, start) + len(end)
        types.append(source[start:stop])
    coeff_start = source.index("static const float s_decimator_coeff[")
    coeff_end = source.index("};", coeff_start) + 2
    code = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "mic_audio.h"
#include "mic_pipeline_telemetry.h"
#include "dual_end_audio.h"
#include "dual_end_telemetry_format.h"
#define MIC_CHANNEL_COUNT 2
#define MIC_DECIMATION_FACTOR 3
#define MIC_CAPTURE_FRAMES_PER_PACKET 960
#define MIC_OUTPUT_FRAMES_PER_PACKET 320
#define MIC_OUTPUT_SAMPLE_VALUES_PER_PACKET 640
#define MIC_CAPTURE_BUFFER_BYTES 3840U
#define MIC_DECIMATOR_FIR_TAPS 95
#define MIC_DECIMATOR_FIR_STORAGE 96
#define MIC_HPF_ALPHA 0.9895826647f
#define MIC_PROCESSING_BUDGET_US 20000
#define MIC_PIPELINE_WINDOW_US UINT64_C(5000000)
#define pdTRUE 1
#define pdMS_TO_TICKS(ms) (ms)
typedef uint32_t TickType_t;
static unsigned locks,fixture_tick,fixture_group,cost_us,queued_packets;
static int s_config_lock,s_pipeline_lock;
#define portENTER_CRITICAL(p) ((void)(p),assert(!locks),++locks)
#define portEXIT_CRITICAL(p) ((void)(p),assert(locks==1),--locks)
static int64_t esp_timer_get_time(void){return (int64_t)fixture_tick*1000+(int64_t)++cost_us;}
static TickType_t xTaskGetTickCount(void){return fixture_tick;}
static uint16_t s_config[13],s_requested_config[13];
static uint32_t s_config_generation,s_applied_generation;
static uint8_t s_output_gain_x;
static float s_runtime_hpf=MIC_HPF_ALPHA,s_hpf_previous_input[2],s_hpf_previous_output[2];
static float s_fir_history[2][MIC_DECIMATOR_FIR_STORAGE],s_runtime_fir[MIC_DECIMATOR_FIR_TAPS];
static size_t s_fir_write_index;
static unsigned s_decimation_phase;
static uint32_t s_soft_limited_frames,s_vad_hold_frames;
static float s_soft_limit_min_gain=1.0f,s_vad_gain=1.0f;
static bool s_vad_open=true;
static dual_end_spatial_t spatial;
static dual_end_spatial_t *s_spatial=&spatial;
static dual_end_spatial_stats_t s_spatial_status;
static uint16_t s_spatial_requested_mode;
static uint64_t s_spatial_work_us;
static uint32_t s_spatial_work_frames,s_spatial_work_max_us;
static uint32_t s_diagnostic_frames,s_diagnostic_max_us;
static uint64_t s_diagnostic_total_us;
static dual_end_audio_analysis_metadata_t s_diagnostic_metadata;
static uint32_t s_frame_dispatch_invalidated,s_frame_dispatch_queue_full;
static uint32_t s_frame_dispatch_highwater,s_frame_dispatch_unavailable,s_pipeline_deadline_misses;
static uint64_t s_pipeline_last_overrun_us;
static int s_frame_dispatch_blocks=1;
static void *s_frame_user_ctx;
static void callback(const int16_t *pcm,size_t count,
    const dual_end_audio_analysis_metadata_t *analysis,void *ctx){
    (void)pcm;(void)count;(void)analysis;(void)ctx;assert(false);
}
static mic_audio_frame_cb_t s_frame_callback=callback;
static dual_end_spatial_telemetry_t s_spatial_telemetry;
static mic_pipeline_telemetry_t s_pipeline_window,published[3];
static bool s_pipeline_window_active;
static unsigned published_count;
static mic_wiener_timing_histogram_t s_pipeline_wiener_histogram[2];
typedef struct {mic_audio_stats_t stats;int64_t raw_sum_sq[2],pcm_sum_sq[2];uint32_t sequence;} mic_stats_snapshot_t;
static mic_stats_snapshot_t s_stats_snapshot;
static mic_output_health_t expected_groups[3],expected_interval;
static void mic_pipeline_close_window(uint64_t now){
    (void)now;
    if(!s_pipeline_window_active)return;
    assert(published_count<3U && !locks);
    published[published_count++]=s_pipeline_window;s_pipeline_window_active=false;
}
static void mic_spatial_reset_state_owner(uint16_t mode){
    assert(mode==0);s_spatial->stats.mode=mode;++s_spatial->stats.generation;
}
static void mic_owner_apply_spatial_commit(void){}
static bool mic_owner_spatial_transition_active(void){return false;}
static uint32_t mic_frame_dispatch_epoch_load(void){return 7;}
static void mic_drain_owner_timing_events(void){}
static void mic_spatial_cache_snapshot(void){
    s_spatial_telemetry=(dual_end_spatial_telemetry_t){.boot_id=7,.session_id=9,.codec=4,
        .codec_known=true,.generation=s_spatial->stats.generation,.applied=0};
}
static mic_nlms_block_t mic_nlms_observe(const dual_end_spatial_stats_t *stats){
    (void)stats;assert(false);return (mic_nlms_block_t){0};
}
bool dual_end_spatial_process_timed(dual_end_spatial_t *state,int16_t *pcm,size_t frames,
    dual_end_spatial_clock_us_fn clock,dual_end_spatial_wiener_timing_t *timing){
    (void)state;(void)pcm;(void)frames;(void)clock;(void)timing;assert(false);return false;
}
/* MSVC diagnoses the intentional trailing padding of the extracted queue
 * record. Preserve the production alignment and assert it rather than alter
 * the type layout to make a host-only warning disappear. */
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4324)
#endif
''' + "\n".join(types) + r'''
#ifdef _MSC_VER
#pragma warning(pop)
#endif
_Static_assert(offsetof(mic_capture_block_t,pcm)%16U==0U,"capture PCM alignment");
_Static_assert(sizeof(mic_capture_block_t)%16U==0U,"capture stride alignment");
_Static_assert(offsetof(mic_frame_dispatch_t,pcm)%16U==0U,"dispatch PCM alignment");
static void queue_oracle(mic_output_health_t *expected,const int16_t *pcm){
    expected->samples+=320U;
    expected->limited_frames+=fixture_group==1U?320U:0U;
    for(unsigned i=0;i<640U;++i){
        const int32_t sample=pcm[i];
        const uint32_t magnitude=(uint32_t)(sample<0?-sample:sample);
        expected->sum_sq[i%2U]+=(uint64_t)(sample*sample);
        if(magnitude>expected->peak[i%2U])expected->peak[i%2U]=magnitude;
    }
}
static int xQueueSend(int queue,const void *value,unsigned wait){
    assert(queue==1 && !wait && !locks);
    const mic_frame_dispatch_t *frame=value;
    assert(frame->callback==callback && frame->dispatch_epoch==7 && frame->capture_generation==1);
    queue_oracle(&expected_groups[fixture_group],frame->pcm);
    queue_oracle(&expected_interval,frame->pcm);
    ++queued_packets;return pdTRUE;
}
static unsigned uxQueueMessagesWaiting(int queue){assert(queue==1);return 1;}
'''
    if "s_frame_dispatch_drops" in processing:
        code += "\nstatic uint32_t s_frame_dispatch_drops;\n"
    code += source[coeff_start:coeff_end]
    for name in ("abs_i16_to_i32", "clamp_i16_from_float", "mic_shared_soft_limit_gain", "mic_filter_and_decimate",
                 "mic_vad_process", "mic_owner_apply_dsp_config", "mic_pipeline_record_block"):
        code += "\n" + implementation(name)
    code += r'''
int main(void){
    mic_capture_block_t storage={.generation=1};
    mic_capture_block_t *processing_block=&storage;
    int16_t *raw_buf=storage.pcm;
    int16_t pcm_storage[640],*pcm_buf=pcm_storage;
    const size_t input_frames=960,bytes_read=3840;
    unsigned stats_seen=0;
    const uint16_t fixture_config[13]={1,0,1,1,0,80,0,7000,1,20000,26000,48,300};
    memcpy(s_requested_config,fixture_config,sizeof(fixture_config));s_config_generation=1;
''' + locals_source + r'''
    for(unsigned iteration=0;iteration<110U;++iteration){
        fixture_tick=(iteration+1U)*20U;cost_us=0;
        fixture_group=iteration<25U?0U:(iteration<75U?1U:2U);
        if(iteration==25U || iteration==75U){
            s_requested_config[2]=iteration==25U?4U:1U;
            s_requested_config[3]=iteration==25U?2U:1U;
            ++s_config_generation;
        }
        const uint64_t dequeued_us=(uint64_t)fixture_tick*1000U;
        storage.captured_us=storage.enqueued_us=dequeued_us-100U;
        ++storage.sequence;
        for(unsigned frame=0;frame<960U;++frame){
            raw_buf[2U*frame]=(int16_t)(frame%2U?10000:-10000);
            raw_buf[2U*frame+1U]=(int16_t)(frame%2U?-4000:4000);
        }
''' + processing + r'''
        assert(queued_packets==iteration+1U && !locks && pcm_frames==0U);
        assert(!s_frame_dispatch_queue_full && !s_frame_dispatch_unavailable && !s_frame_dispatch_invalidated);
        assert(!s_pipeline_deadline_misses && s_frame_dispatch_highwater==1U);
        if(s_stats_snapshot.sequence!=stats_seen){
            assert(++stats_seen==s_stats_snapshot.sequence && (iteration==49U || iteration==99U));
            assert(s_stats_snapshot.stats.input_frames==48000U && s_stats_snapshot.stats.output_frames==16000U);
            assert(s_stats_snapshot.stats.packets_per_sec==50U && s_stats_snapshot.stats.sample_values_per_sec==32000U);
            assert(s_stats_snapshot.raw_sum_sq[0]==INT64_C(48000)*10000*10000);
            assert(s_stats_snapshot.raw_sum_sq[1]==INT64_C(48000)*4000*4000);
            assert(s_stats_snapshot.stats.left_raw_peak==10000 && s_stats_snapshot.stats.right_raw_peak==4000);
            for(unsigned channel=0;channel<2U;++channel)
                assert((uint64_t)s_stats_snapshot.pcm_sum_sq[channel]==expected_interval.sum_sq[channel]);
            assert((uint32_t)s_stats_snapshot.stats.left_pcm_peak==expected_interval.peak[0]);
            assert((uint32_t)s_stats_snapshot.stats.right_pcm_peak==expected_interval.peak[1]);
            assert(s_stats_snapshot.stats.soft_limited_frames==expected_interval.limited_frames);
            assert(expected_interval.samples==16000U && expected_interval.limited_frames==8000U);
            assert(s_stats_snapshot.stats.minimum_gain_permille<1000U);
            assert(!s_soft_limited_frames && s_soft_limit_min_gain==1.0f);
            assert(!stats_input_frames && !stats_output_frames && !raw_sum_sq[0] && !pcm_sum_sq[0]);
            expected_interval=(mic_output_health_t){0};
        }
    }
    mic_pipeline_close_window((uint64_t)fixture_tick*1000U);
    assert(stats_seen==2U && published_count==3U);
    const unsigned frames[]={25U,50U,35U};
    for(unsigned group=0;group<3U;++group){
        const mic_output_health_t *actual=&published[group].capture_health.output;
        const mic_output_health_t *expected=&expected_groups[group];
        assert(actual->samples==(uint64_t)frames[group]*320U && actual->samples==expected->samples);
        assert(actual->limited_frames==expected->limited_frames);
        for(unsigned channel=0;channel<2U;++channel){
            assert(actual->sum_sq[channel]==expected->sum_sq[channel]);
            assert(actual->peak[channel]==expected->peak[channel]);
        }
        assert(published[group].output_gain_l_x==(group==1U?4U:1U));
        assert(published[group].output_gain_r_x==(group==1U?2U:1U));
    }
    assert(published[0].generation!=published[1].generation && published[1].generation!=published[2].generation);
    assert(published[1].capture_health.output.limited_frames==16000U);
    puts("actual capture loop/queue PCM energies, peaks, limiter, 1s reset and gain windows PASS");return 0;
}
'''
    assert "gain windows PASS" in run_c(tmp_path, code)
