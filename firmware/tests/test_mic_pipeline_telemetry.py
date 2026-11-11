"""Real portable formatter/policy and extracted owner/background C behavior.

Synthetic clocks/SDK stubs only; no recording, playback, ESP timing or ASR claim.
"""
import json
from pathlib import Path
from test_runtime_codec import function
from test_audio_recovery_042 import _build_host_executable, _run

ROOT = Path(__file__).resolve().parents[1]
MIC = ROOT / "scepter/main/mic"


def build_run(tmp_path, code, sources=(), includes=()):
    harness = tmp_path / "pipeline_telemetry.c"
    harness.write_text(code, encoding="utf-8")
    executable, env = _build_host_executable([harness, *sources], tmp_path,
        extra_includes=[MIC, ROOT / "components/dual_end_runtime/include",
                        ROOT / "components/dual_end_audio_codec/include", *includes])
    return _run([str(executable)], cwd=tmp_path, env=env).stdout


def test_formatter_all_parts_identity_uint64_null_and_bounds(tmp_path):
    code = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mic_pipeline_telemetry.h"
int main(void){
 mic_pipeline_telemetry_t r={.boot_id=UINT32_MAX,.session_id=UINT32_MAX,.sequence=UINT32_MAX,
  .capture_generation=UINT32_MAX,.window=UINT32_MAX,.codec=3,.codec_known=true,.mode=2,.generation=UINT16_MAX,
  .sampled_us=UINT64_MAX,.start_us=UINT64_MAX-1,.diag_q=UINT32_MAX,.diag_b=UINT32_MAX,.diag_o=UINT32_MAX,
  .diag_s=UINT32_MAX,.fft_hi=UINT32_MAX,.win_lost=UINT32_MAX,.late=UINT32_MAX,.raw_seq=UINT32_MAX,
  .q_hi_all=UINT32_MAX,.fft_us=1,.fft_n=UINT64_MAX,.fft_u=UINT64_MAX};
 r.q=r.a=r.d=r.s=r.w=(mic_pipeline_metric_t){UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT64_MAX};
 for(unsigned unknown=0;unknown<2;++unknown){
  if(unknown){r.codec_known=false;r.q.n=r.a.n=r.d.n=r.s.n=r.w.n=0;r.fft_us=0;}
  for(unsigned p=0;p<MIC_PIPELINE_TELEMETRY_PARTS;++p){
   unsigned char memory[183];memset(memory,0xa5,sizeof(memory));char *line=(char*)memory;
   size_t n=mic_pipeline_telemetry_format(&r,p,line,181);assert(n>0&&n<=180&&n==strlen(line));
   assert(memory[181]==0xa5&&memory[182]==0xa5);puts(line);
   assert(!mic_pipeline_telemetry_format(&r,p,line,n)&&line[0]=='\0'&&memory[181]==0xa5);
  }
 }
 char line[181];assert(!mic_pipeline_telemetry_format(&r,32,line,sizeof(line)));
 r.boot_id=0;assert(!mic_pipeline_telemetry_format(&r,0,line,sizeof(line)));
 r.boot_id=1;r.mode=5;assert(mic_pipeline_telemetry_format(&r,0,line,sizeof(line))&&strstr(line,"\"m\":5"));
 r.mode=6;assert(!mic_pipeline_telemetry_format(&r,0,line,sizeof(line))&&line[0]=='\0');
 r.mode=0;r.codec_known=true;r.codec=99;assert(!mic_pipeline_telemetry_format(&r,0,line,sizeof(line)));
 return 0;
}
'''
    raw = build_run(tmp_path, code, [MIC / "mic_pipeline_telemetry.c"])
    rows = [json.loads(line) for line in raw.splitlines()]
    assert len(rows) == 64
    assert all(len(line.encode()) <= 180 for line in raw.splitlines())
    for group in (rows[:32], rows[32:]):
        assert [r["part"] for r in group] == list(range(32))
        assert all((r["boot_id"], r["role"], r["session_id"], r["seq"], r["m"], r["g"])
                   == ("ffffffff", 0, 4294967295, 4294967295, 2, 65535) for r in group)
        merged = {key: value for row in group for key, value in row.items()}
        assert merged["part_count"] == 32 and merged["q_us"] == str(2**64 - 1)
        assert merged["us"] == str(2**64 - 1) and merged["cg"] == 2**32 - 1
    assert rows[5]["q_lo"] == 2**32 - 1
    assert all(r["c"] is None for r in rows[32:])
    assert rows[37]["q_lo"] is None and rows[38]["q_hi"] is None
    assert rows[55]["fft_us"] is None and rows[56]["age"] is None


def test_budget_rate_charge_outlier_recovery_and_metrics(tmp_path):
    code = r'''
#include <assert.h>
#include <stdio.h>
#include "mic_pipeline_telemetry.h"
int main(void){
 mic_diagnostic_budget_t b={0};unsigned starts=0;
 for(uint64_t now=1000000;now<2000000;now+=20000){
  if(mic_diagnostic_budget_admit(&b,now)){++starts;mic_diagnostic_budget_charge(&b,8000);}
 }
 assert(starts==5&&b.used_us==40000);
 assert(mic_diagnostic_budget_admit(&b,2000000));mic_diagnostic_budget_charge(&b,55000);
 assert(b.used_us==55000&&!mic_diagnostic_budget_admit(&b,2200000));
 assert(!mic_diagnostic_budget_admit(&b,2999999));
 assert(mic_diagnostic_budget_admit(&b,3000000));mic_diagnostic_budget_charge(&b,7000);
 assert(!mic_diagnostic_budget_admit(&b,3199999)&&mic_diagnostic_budget_admit(&b,3200000));
 assert(!mic_diagnostic_budget_admit(&b,1)); /* no unsigned-wrap budget reset */
 mic_pipeline_metric_t m={0};mic_pipeline_metric_add(&m,123);mic_pipeline_metric_add(&m,0);mic_pipeline_metric_add(&m,456);
 assert(m.n==3&&m.lo==0&&m.hi==456&&m.us==579);
 m.us=UINT64_MAX-1;mic_pipeline_metric_add(&m,10);assert(m.us==UINT64_MAX);
 puts("budget 5 Hz/40 ms admission, outlier full-charge/recovery, extrema/totals PASS");return 0;
}
'''
    assert "PASS" in build_run(tmp_path, code)


def test_owner_windows_do_not_mix_sessions_modes_generations_or_nested_times(tmp_path):
    mic = (MIC / "mic_audio.c").read_text(encoding="utf-8")
    code = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mic_pipeline_telemetry.h"
#include "mic_diagnostics_snapshot.h"
#include "dual_end_telemetry_format.h"
#include "dual_end_spatial.h"
typedef struct{uint64_t captured_us,enqueued_us;uint32_t generation;}mic_capture_block_t;
typedef struct {uint32_t session_id,mutex_wait_us,encode_us,radio_submit_us;uint8_t codec;
 bool valid,encode_attempted,encode_ok,radio_submitted,radio_ok;}dual_end_audio_owner_timing_t;
static int s_config_lock,s_pipeline_lock;
    static uint16_t s_config[13];
static unsigned locked;
#define portENTER_CRITICAL(p) ((void)(p),assert(!locked),++locked)
#define portEXIT_CRITICAL(p) ((void)(p),assert(locked),--locked)
static mic_pipeline_telemetry_t s_pipeline_window,s_pipeline_report;
static bool s_pipeline_window_active,s_pipeline_report_ready;
static uint32_t s_pipeline_report_sequence,s_pipeline_windows_lost;
static uint32_t s_frame_dispatch_queue_full,s_frame_dispatch_invalidated,s_frame_dispatch_unavailable;
static uint32_t s_owner_timing_event_drops,s_frame_dispatch_highwater,s_frame_dispatch_late_events;
static mic_audio_diagnostics_snapshot_t s_pipeline_diagnostics_cache;
static bool s_pipeline_diagnostics_cache_ready;
static mic_wiener_timing_histogram_t s_pipeline_wiener_histogram[2];
static dual_end_spatial_t spatial_state;
static dual_end_spatial_t *s_spatial=&spatial_state;
static dual_end_spatial_telemetry_t s_spatial_telemetry;
static uint32_t s_diagnostic_drops=3,s_diagnostic_budget_skips=4,s_diagnostic_overload_skips=5,s_diagnostic_stale_skips=6;
static int64_t s_diagnostic_completed_us=900000,clock_us=1000000;
static uint64_t s_diagnostic_completed_frames=7,s_diagnostic_completed_total_us=800;
static uint32_t s_diagnostic_lifetime_max_us=90,s_pipeline_deadline_misses=10,s_capture_sequence=11,s_pipeline_highwater=2;
static int64_t esp_timer_get_time(void){return clock_us;}
''' + function(mic, "mic_pipeline_close_window") + '\n' + function(mic, "mic_pipeline_record_block") + r'''
static void record(uint64_t dequeue,unsigned d,unsigned s,unsigned w,
                   const dual_end_spatial_wiener_timing_t *wiener_timing){
 mic_capture_block_t b={dequeue-100,dequeue-60,9};clock_us=(int64_t)dequeue+100;
 mic_pipeline_record_block(&b,dequeue,d,s,w,NULL,NULL,NULL,wiener_timing);assert(!locked);
}
int main(void){
 s_spatial_telemetry=(dual_end_spatial_telemetry_t){.boot_id=7,.session_id=1,.codec=3,.codec_known=true,.applied=0,.generation=1};
     s_config[2]=16;s_config[3]=16;
 record(1000000,10,0,30,NULL);record(2000000,20,0,40,NULL);
 assert(!s_pipeline_report_ready&&s_pipeline_window.q.n==2&&s_pipeline_window.w.us==70&&s_pipeline_window.d.us==30);
 s_spatial_telemetry.session_id=2;s_spatial_telemetry.codec=4;s_spatial_telemetry.applied=1;++s_spatial_telemetry.generation;
 record(3000000,30,40,100,NULL);
 assert(s_pipeline_report_ready&&s_pipeline_report.session_id==1&&s_pipeline_report.mode==0&&s_pipeline_report.w.us==70);
 assert(s_pipeline_report.diag_q==3&&s_pipeline_report.diag_b==4&&s_pipeline_report.fft_us==900000);
 assert(s_pipeline_window.session_id==2&&s_pipeline_window.mode==1&&s_pipeline_window.w.us==100&&s_pipeline_window.s.us==40);
 record(8000000,40,50,120,NULL);assert(s_pipeline_report.session_id==2&&s_pipeline_report.w.us==220&&s_pipeline_report.s.us==90);
 assert(s_pipeline_report.win_lost==1&&s_pipeline_report.sequence==2&&!s_pipeline_window_active);
 assert(s_pipeline_diagnostics_cache_ready&&s_pipeline_diagnostics_cache.pipeline.sequence==2&&s_pipeline_diagnostics_cache.pipeline.w.us==220);
 assert(s_pipeline_diagnostics_cache.spatial_ready&&s_pipeline_diagnostics_cache.cache_sequence==2);
 record(9000000,1,2,6,NULL);s_spatial_telemetry.generation=3;record(10000000,3,4,10,NULL);
 assert(s_pipeline_report.generation==2&&s_pipeline_report.w.us==6&&s_pipeline_window.generation==3&&s_pipeline_window.w.us==10);
 mic_pipeline_close_window(10001000);assert(s_pipeline_report.generation==3&&s_pipeline_report.w.us==10&&!locked);
 s_spatial_telemetry.applied=4;s_spatial_telemetry.generation=4;
 dual_end_spatial_wiener_timing_t mode4={.us={123},.channels=1,.valid=true};
 record(10500000,1,2,6,&mode4);
 assert(s_pipeline_wiener_histogram[0].count==1&&s_pipeline_wiener_histogram[0].max_us==123);
 mic_pipeline_close_window(10501000);
 assert(s_pipeline_report.mode==4&&s_pipeline_report.wiener_timing[0].available&&
        s_pipeline_report.wiener_timing[0].p50_us==150&&s_pipeline_report.wiener_timing[0].max_us==123&&
        !s_pipeline_report.wiener_timing[1].available);
 s_spatial_telemetry.applied=5;s_spatial_telemetry.generation=5;
 dual_end_spatial_wiener_timing_t mode5={.us={201,349},.channels=2,.valid=true};
 record(11000000,1,2,6,&mode5);
 mic_pipeline_close_window(11001000);
 assert(s_pipeline_report.mode==5&&s_pipeline_report.wiener_timing[0].p50_us==250&&
        s_pipeline_report.wiener_timing[0].max_us==201&&s_pipeline_report.wiener_timing[1].p50_us==350&&
        s_pipeline_report.wiener_timing[1].max_us==349);
 mic_capture_block_t bad={UINT64_MAX,UINT64_MAX,10};clock_us=11000000;
 mic_pipeline_record_block(&bad,11000000,1,2,4,NULL,NULL,NULL,NULL);assert(!s_pipeline_window.q.n&&!s_pipeline_window.a.n&&s_pipeline_window.w.n==1);
 bad.captured_us=bad.enqueued_us=1;clock_us=(int64_t)UINT32_MAX+200;
 mic_pipeline_record_block(&bad,(uint64_t)clock_us,1,2,4,NULL,NULL,NULL,NULL);
 assert(!s_pipeline_window.q.n&&!s_pipeline_window.a.n&&s_pipeline_window.w.n==2); /* invalid durations omitted */
     s_pipeline_window_active=false;s_spatial_telemetry.applied=0;s_spatial_telemetry.generation=6;
     s_config[2]=16;s_config[3]=16;record(12000000,1,2,6,NULL);
     assert(s_pipeline_window.output_gain_l_x==16&&s_pipeline_window.output_gain_r_x==16);
     s_config[2]=4;record(12100000,1,2,6,NULL);
     assert(s_pipeline_report_ready&&s_pipeline_report.output_gain_l_x==16&&
            s_pipeline_report.output_gain_r_x==16&&s_pipeline_window_active&&
            s_pipeline_window.output_gain_l_x==4&&s_pipeline_window.output_gain_r_x==16);
     mic_pipeline_close_window(12101000);
     assert(s_pipeline_report.output_gain_l_x==4&&s_pipeline_report.output_gain_r_x==16);
 puts("owner nonoverlap/context switch, immutable boot counters, nested stage separation and cache overwrite PASS");return 0;
}
'''
    assert "PASS" in build_run(tmp_path, code)


def test_actual_background_outputs_spatial_and_pipeline_with_no_fft_task(tmp_path):
    mic = (MIC / "mic_audio.c").read_text(encoding="utf-8")
    (tmp_path / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n", encoding="utf-8")
    code = r'''
#include <assert.h>
#include <stdarg.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <setjmp.h>
#include "dual_end_link_config.h"
#include "mic_audio.h"
#include "mic_pipeline_telemetry.h"
#include "dual_end_telemetry_format.h"
typedef unsigned TickType_t;
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(x) (x)
#define MIC_ANALYSIS_RUNTIME_LOG_INTERVAL_MS 5000
static int s_config_lock,s_pipeline_lock;
static unsigned locked,clock_ticks,polls,pipeline_parts,spatial_parts,capture_parts,owner_parts,nlms_parts,nlms_presence_parts,dispatch_parts,output_parts,callbacks,yields;
static uint32_t s_capture_overloads,s_diagnostic_drops,s_pipeline_highwater,s_pipeline_deadline_misses,s_dma_overflows,s_diagnostic_sequence;
static int64_t s_diagnostic_completed_us;
static int64_t esp_timer_get_time(void){return clock_ticks*1000;}
static jmp_buf done;
#define portENTER_CRITICAL(p) ((void)(p),assert(!locked),++locked)
#define portEXIT_CRITICAL(p) ((void)(p),assert(locked),--locked)
static unsigned ulTaskNotifyTake(int clear,unsigned timeout){assert(clear&&timeout==UINT32_MAX);return 1;}
static TickType_t xTaskGetTickCount(void){return clock_ticks;}
static void vTaskDelay(unsigned ticks){assert(!locked);clock_ticks+=ticks;if(ticks==100){if(++polls>22)longjmp(done,1);}else{assert(ticks==1);++yields;}}
static mic_pipeline_telemetry_t s_pipeline_report;
static bool s_pipeline_report_ready=true;
static dual_end_spatial_telemetry_t s_spatial_telemetry;
typedef struct{mic_audio_stats_t stats;int64_t raw_sum_sq[2],pcm_sum_sq[2];uint32_t sequence;}mic_stats_snapshot_t;
static mic_stats_snapshot_t s_stats_snapshot;
static void stats_callback(const mic_audio_stats_t *s,void *ctx){assert(!ctx&&!locked);++callbacks;
 assert(s->left_raw_rms==100&&s->right_raw_rms==50&&s->left_pcm_rms==200&&s->right_pcm_rms==300);}
static mic_audio_stats_cb_t s_stats_callback=stats_callback;
static void *s_stats_user_ctx;
static void log_message(const char *fmt,...){assert(!locked);if(strcmp(fmt,"%s"))return;
 va_list args;va_start(args,fmt);const char *line=va_arg(args,const char*);va_end(args);
 if(strstr(line,"audio.pipeline"))++pipeline_parts;else if(strstr(line,"audio.spatial"))++spatial_parts;
 else if(strstr(line,"audio.capture_health"))++capture_parts;else if(strstr(line,"audio.owner_timing"))++owner_parts;
 else if(strstr(line,"audio.dispatch"))++dispatch_parts;else if(strstr(line,"audio.output"))++output_parts;
 else if(strstr(line,"audio.nlms_presence"))++nlms_presence_parts;
 else if(strstr(line,"audio.nlms"))++nlms_parts;else assert(false);}
#define ESP_LOGI(tag,...) log_message(__VA_ARGS__)
''' + function(mic, "mic_telemetry_task") + r'''
int main(void){
 s_pipeline_report=(mic_pipeline_telemetry_t){.boot_id=7,.session_id=9,.sequence=1,.codec=3,.codec_known=true,.sampled_us=1000000,.nlms={.blocks=1}};
 s_spatial_telemetry=(dual_end_spatial_telemetry_t){.boot_id=7,.session_id=9,.codec=3,.codec_known=true,.sampled_us=1000000};
 s_stats_snapshot.sequence=1;s_stats_snapshot.stats.input_frames=1000;s_stats_snapshot.stats.output_frames=320;
 s_stats_snapshot.raw_sum_sq[0]=10000000;s_stats_snapshot.raw_sum_sq[1]=2500000;
 s_stats_snapshot.pcm_sum_sq[0]=12800000;s_stats_snapshot.pcm_sum_sq[1]=28800000;
 if(!setjmp(done))mic_telemetry_task(NULL);
 assert(pipeline_parts==32&&capture_parts==18&&owner_parts==17&&nlms_parts==23&&nlms_presence_parts==MIC_NLMS_PRESENCE_TELEMETRY_PARTS&&spatial_parts==0&&callbacks==1&&!s_pipeline_report_ready&&!locked);
 assert(dispatch_parts==MIC_DISPATCH_TELEMETRY_PARTS&&output_parts==MIC_OUTPUT_HEALTH_TELEMETRY_PARTS);
 assert(yields==pipeline_parts+capture_parts+owner_parts+nlms_parts+nlms_presence_parts+dispatch_parts+output_parts);
 puts("independent background emits coherent bounded pipeline/health/owner windows without any FFT queue/task; sqrt/callback only here PASS");return 0;
}
'''
    assert "PASS" in build_run(tmp_path, code,
        [MIC / "mic_pipeline_telemetry.c", MIC / "mic_observability_telemetry.c", ROOT / "components/dual_end_runtime/dual_end_telemetry_format.c"],
        [tmp_path, ROOT / "components/dual_end_audio_codec/include"])


def test_realtime_boundary_has_no_fft_format_or_sqrt_and_separate_background(tmp_path):
    # Policy guard supplements actual lifecycle/window tests, not a substitute
    # for real ESP timing. It prevents FFT/logger coupling from returning.
    mic = (MIC / "mic_audio.c").read_text(encoding="utf-8")
    owner = function(mic, "mic_audio_task")
    processing = owner[owner.index("    while (true) {"):]
    assert "mic_analysis_process_frame" not in owner
    assert "telemetry_format" not in processing and "sqrt(" not in processing
    assert "s_stats_callback(" not in processing
    assert "s_frame_callback(" not in processing
    assert "dual_end_audio_sender_record_mic_work" not in processing
    assert "xQueueSend(s_frame_dispatch_blocks,&dispatch,0)" in processing
    dispatch = function(mic, "mic_frame_dispatch_task_fn")
    assert "xQueueReceive(s_frame_dispatch_blocks,&frame,portMAX_DELAY)" in dispatch
    assert "dual_end_audio_sender_take_owner_timing" in dispatch
    # The dispatcher is intentionally a separate worker.  The bounded config
    # worker also owns codec/layout transitions; the PCM owner only applies a
    # local commit at a frame boundary and never takes their mutex or sender.
    assert "xSemaphoreTake(s_frame_dispatch_control,portMAX_DELAY)" in dispatch
    assert "ESP_LOG" not in dispatch
    assert "s_frame_dispatch_control" not in processing
    assert "dual_end_audio_sender_" not in processing
    assert "s_frame_dispatch_control=xSemaphoreCreateMutex();" in owner
    assert "!s_frame_dispatch_control)" in owner
    assert "mic_codec_apply_control" not in processing
    assert "mic_spatial_apply_control" not in processing
    assert "mic_owner_apply_dsp_config" in processing
    assert "mic_owner_apply_spatial_commit" in processing
    assert "pcm_dispatch_epoch=mic_frame_dispatch_epoch_load()" in processing
    assert "!pcm_dispatch_allowed" in processing
    runtime = (ROOT / "components/dual_end_runtime/dual_end_runtime.c").read_text(
        encoding="utf-8"
    )
    publish_start = runtime.index("static void runtime_publish(")
    publish_end = runtime.index("void dual_end_runtime_record(", publish_start)
    runtime_publish = runtime[publish_start:publish_end]
    runtime_start = runtime.index("void dual_end_runtime_record_outcome(")
    runtime_end = runtime.index("uint32_t dual_end_runtime_boot_id(void)", runtime_start)
    runtime_record = runtime[runtime_start:runtime_end]
    # The owner completion path may publish best-effort profiling evidence,
    # but cannot inherit a mutex, delay, log, heap, or blocking queue send.
    assert "xQueueSend(s_snapshots, &snapshot, 0)" in runtime_publish
    assert all(token not in runtime_publish + runtime_record for token in (
        "xSemaphore", "vTaskDelay", "ESP_LOG", "malloc", "portMAX_DELAY"
    ))
    codec_apply = function(mic, "mic_codec_apply_control")
    spatial_apply = function(mic, "mic_spatial_apply_control")
    config_worker = function(mic, "mic_config_transition_task_fn")
    assert "mic_frame_dispatch_control_lock" in codec_apply
    assert "mic_frame_dispatch_control_lock" in spatial_apply
    # The dispatch gate serializes the whole callback with a sender mutation:
    # a config worker cannot reset/destroy the encoder while that callback owns
    # it.  A failed setter can still reset the sender session, so every attempt
    # invalidates prior queued PCM before the gate is released.
    dispatch_take = dispatch.index("xSemaphoreTake(s_frame_dispatch_control,portMAX_DELAY)")
    dispatch_callback = dispatch.index("frame.callback(frame.pcm")
    dispatch_give = dispatch.index("xSemaphoreGive(s_frame_dispatch_control);", dispatch_callback)
    assert dispatch_take < dispatch_callback < dispatch_give
    for apply, setter, result in (
        (codec_apply, "dual_end_audio_sender_set_codec", "ret"),
        (spatial_apply, "dual_end_audio_sender_set_processing_layout", "result"),
    ):
        setter_at = apply.index(setter)
        invalidate_at = apply.index("mic_frame_dispatch_invalidate();")
        unlock_at = apply.index("mic_frame_dispatch_control_unlock();")
        assert setter_at < invalidate_at < unlock_at
        assert f"if ({result} == ESP_OK) mic_frame_dispatch_invalidate" not in apply
        assert f"if({result}==ESP_OK) mic_frame_dispatch_invalidate" not in apply
    assert "mic_codec_apply_control" in config_worker
    assert "mic_spatial_apply_control" in config_worker
    assert "ulTaskNotifyTake(pdTRUE,portMAX_DELAY)" in config_worker
    assert "s_codec_apply_result" in codec_apply
    assert "s_spatial_apply_result" in spatial_apply
    # There are no alternate foreground/lifecycle routes that can mutate the
    # sender alongside an active callback; both transport setters live only in
    # these control-worker helpers.
    assert mic.count("dual_end_audio_sender_set_codec(") == 1
    assert mic.count("dual_end_audio_sender_set_processing_layout(") == 1
    codec_parameter = function(mic, "mic_codec_parameter")
    spatial_parameter = function(mic, "mic_spatial_parameter")
    # The synchronous parameter reply reports the transition outcome.  It does
    # not claim success merely because the low-priority worker accepted a
    # request or started a sender mutation.
    assert "s_codec_applied_generation == generation" in codec_parameter
    assert "s_codec_apply_result" in codec_parameter
    assert "if (!applied || ret != ESP_OK) return ret;" in codec_parameter
    assert "s_spatial_applied_request==generation" in spatial_parameter
    assert "s_spatial_apply_result" in spatial_parameter
    assert "if(!applied || result!=ESP_OK)return result;" in spatial_parameter
    assert "xTaskCreatePinnedToCore(\n        mic_config_transition_task_fn" in owner
    # A spatial layout transition remains armed until the same DSP owner has
    # locally reset its matcher, so a queued frame cannot cross configurations.
    owner_commit = function(mic, "mic_owner_apply_spatial_commit")
    assert "s_spatial_transition_active" in owner_commit
    assert "mic_spatial_reset_state_owner" in owner_commit
    assert "mic_config_transition_notify" in owner_commit
    diagnostic = function(mic, "mic_diagnostic_task")
    assert "ESP_LOG" not in diagnostic and "telemetry_format" not in diagnostic
    assert "mic_diagnostic_budget_admit" in diagnostic
    background = function(mic, "mic_telemetry_task")
    assert "dual_end_spatial_telemetry_format" in background
    assert "mic_pipeline_telemetry_format" in background and "vTaskDelay(1)" in background
    assert "mic_analysis_process_frame" not in background
    assert "xQueueCreate(1,sizeof(mic_capture_block_t))" in owner


def test_sender_radio_submit_failure_is_not_reported_as_pipeline_success(tmp_path):
    # This is a source-level regression guard for the ownership handoff.  The
    # broader codec test mocks the actual admission path when a host compiler
    # is available; this guard remains runnable without firmware tooling.
    sender = (ROOT / "scepter/components/dual_end_audio/dual_end_audio.c").read_text(
        encoding="utf-8"
    )
    definition = sender.index(
        "esp_err_t dual_end_audio_sender_send_pcm_with_hardware_and_analysis("
    )
    send = function(
        sender[definition:], "dual_end_audio_sender_send_pcm_with_hardware_and_analysis"
    )
    take = function(sender, "dual_end_audio_sender_take_owner_timing")
    assert "dual_end_espnow_link_send_audio_at" in send
    assert "if (ret != ESP_OK)" in send
    assert "s_encode_attempt_success=false;" in send
    assert "s_encode_attempt_outcome=DUAL_END_OUTCOME_OPERATION_FAILED;" in send
    assert "timing->outcome=sender_current_owner_outcome();" in take
    assert "s_encode_attempt_pending=false;" in take
