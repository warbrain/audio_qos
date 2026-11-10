"""Compile real format/owner/logger code; no ESP device or audio operations.

Standalone standard-library tests keep these checks runnable without installing
pytest into ESP-IDF's Python environment. The compiler is native host MSVC/cc,
not ESP timing evidence. Pass TEST_MSVC_TOOLS/TEST_WINDOWS_SDK when relocated.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "components/dual_end_runtime"


def function(source, name):
    start = source.index(name + "(")
    start = source.rfind("\n", 0, start) + 1
    opening = source.index("{", start)
    end, depth = opening + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def compile_run(code, sources=(), includes=()):
    base = Path(os.environ.get("FIRMWARE_TEST_TMP", ROOT / "build/host-tests"))
    base.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="telemetry-host-", dir=base) as directory:
        directory = Path(directory)
        harness, executable = directory / "check.c", directory / "check.exe"
        harness.write_text(code, encoding="utf-8")
        # Only ESP error typedef is needed for the portable RX snapshot header.
        (directory / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n", encoding="utf-8")
        env = os.environ.copy()
        for key in ("TMP", "TEMP", "TMPDIR"):
            env[key] = str(directory)
        include_paths = [directory, RUNTIME / "include", *includes]
        if os.name == "nt":
            tools = Path(env.get("TEST_MSVC_TOOLS", "D:/Program Files/Microsoft Visual Studio/2022/Professional/VC/Tools/MSVC/14.44.35207"))
            sdk = Path(env.get("TEST_WINDOWS_SDK", "C:/Program Files (x86)/Windows Kits/10"))
            sdk_version = "10.0.26100.0"
            env["PATH"] = str(tools / "bin/Hostx64/x64") + os.pathsep + env.get("PATH", "")
            env["INCLUDE"] = os.pathsep.join(map(str, [tools / "include", *(sdk / "Include" / sdk_version / n for n in ("ucrt", "shared", "um"))]))
            env["LIB"] = os.pathsep.join(map(str, [tools / "lib/x64", *(sdk / "Lib" / sdk_version / n / "x64" for n in ("ucrt", "um"))]))
            extra=[]
            if '"audio_rx_telemetry.h"' in code or '"dual_end_link_config.h"' in code:
                adapter=directory / "wire_adapter.h"
                adapter.write_text('#include <stdbool.h>\n#include <stdint.h>\n#include <stddef.h>\n#pragma pack(push,1)\n#define __attribute__(attributes)\n#include "dual_end_audio_recovery_protocol.h"\n#undef __attribute__\n#pragma pack(pop)\n',encoding="utf-8")
                extra=["/FI"+str(adapter)]
            command = [str(tools / "bin/Hostx64/x64/cl.exe"), "/nologo", "/std:c11", "/utf-8", "/W4", "/WX", "/D_CRT_SECURE_NO_WARNINGS", *extra, *["/I" + str(p) for p in include_paths], str(harness), *map(str, sources), "/Fe:" + str(executable)]
        else:
            command = [shutil.which("cc") or "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", *["-I" + str(p) for p in include_paths], str(harness), *map(str, sources), "-lm", "-o", str(executable)]
        build = subprocess.run(command, cwd=directory, env=env, text=True, capture_output=True)
        if build.returncode:
            raise AssertionError(build.stdout + build.stderr)
        run = subprocess.run([str(executable)], cwd=directory, env=env, text=True, capture_output=True)
        if run.returncode:
            raise AssertionError(run.stdout + run.stderr)
        return run.stdout


class TelemetryCompletion(unittest.TestCase):
    def test_existing_diagnostic_gap_and_generation_harness_still_passes(self):
        fixture=(ROOT / "tests/test_mic_pipeline.py").read_text(encoding="utf-8").split("def test_actual_diagnostic_task_gap_reset_and_stale_generation",1)[1]
        prefix=fixture.split("code=r'''",1)[1].split("'''+task+r'''",1)[0]
        suffix=fixture.split("'''+task+r'''",1)[1].split("'''",1)[0]
        mic=(ROOT / "scepter/main/mic/mic_audio.c").read_text(encoding="utf-8")
        self.assertIn("PASS",compile_run(prefix+function(mic,"mic_diagnostic_task")+suffix,
            sources=[RUNTIME / "dual_end_telemetry_format.c"],
            includes=[ROOT / "components/dual_end_audio_recovery/include",ROOT / "scepter/components/dual_end_common/include",ROOT / "scepter/main/mic"]))

    def test_resource_cpu_first_sample_and_unsupported_configuration_stay_unknown(self):
        runtime = (RUNTIME / "dual_end_runtime.c").read_text(encoding="utf-8")
        for generated, timer, smp in [(1,1,0),(0,1,0),(1,0,0),(1,1,1)]:
            with self.subTest(generated=generated,timer=timer,smp=smp):
                source = f"#define CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS {generated}\n#define CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER {timer}\n#define CONFIG_FREERTOS_SMP {smp}\n" + r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "dual_end_resource_format.h"
#define configNUMBER_OF_CORES 2
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
#define MALLOC_CAP_SPIRAM 4
#define memory_order_relaxed 0
#define atomic_load_explicit(p,m) ((void)(m),*(p))
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
typedef int BaseType_t;
static unsigned s_role=1,s_boot_id=7,s_telemetry_drop=5;
static int s_resource_lock;
static bool s_resource_ready;
static dual_end_resource_snapshot_t s_resource_cache;
static bool (*s_io_probe)(uint32_t[8]);
static uint64_t fixture_clock_us=1000000;
static uint64_t esp_timer_get_time(void){return fixture_clock_us;}
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS && CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER && !CONFIG_FREERTOS_SMP
static uint32_t ulTaskGetIdleRunTimeCounterForCore(BaseType_t core){return (uint32_t)((fixture_clock_us-1000000)*(uint64_t)(core+1)/10+100);}
#endif
static uint32_t heap_caps_get_free_size(unsigned caps){(void)caps;return 4000;}
static uint32_t heap_caps_get_minimum_free_size(unsigned caps){(void)caps;return 2000;}
static uint32_t heap_caps_get_largest_free_block(unsigned caps){(void)caps;return 1024;}
static uint32_t uxTaskGetStackHighWaterMark(void*p){assert(p==NULL);return 3000;}
''' + function(runtime,"runtime_sample_resources") + r'''
int main(void){runtime_sample_resources();assert(s_resource_ready&&s_resource_cache.cpu_busy_permille[0]==-1&&s_resource_cache.cpu_busy_permille[1]==-1&&s_resource_cache.telemetry_drop==5&&!s_resource_cache.io_available);
 fixture_clock_us+=5000000;runtime_sample_resources();
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS && CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER && !CONFIG_FREERTOS_SMP
 assert(s_resource_cache.cpu_busy_permille[0]==900&&s_resource_cache.cpu_busy_permille[1]==800);
#else
 assert(s_resource_cache.cpu_busy_permille[0]==-1&&s_resource_cache.cpu_busy_permille[1]==-1);
#endif
 puts("actual resource CPU first observation/unsupported guard PASS");return 0;}
'''
                self.assertIn("PASS",compile_run(source,sources=[RUNTIME / "dual_end_resource_format.c"]))

    def test_existing_session_pressure_harness_with_added_resource_batches(self):
        # Reuse its independent session-churn fixture, not a rewritten logger.
        test=Path(__file__).with_name("test_runtime_log_pressure.py").read_text(encoding="utf-8")
        prefix=test.split("code = r'''",1)[1].split("''' + function(source, 'runtime_logger') + r'''",1)[0]
        suffix=test.split("''' + function(source, 'runtime_logger') + r'''",1)[1].split("'''",1)[0]
        runtime=(RUNTIME / "dual_end_runtime.c").read_text(encoding="utf-8")
        self.assertIn("PASS",compile_run(prefix+function(runtime,"runtime_logger")+suffix))

    def test_rx_owner_copy_nonblocking_drops_and_saturation(self):
        rx = (ROOT / "dongle/components/dual_end_audio/audio_rx_telemetry.c").read_text(encoding="utf-8")
        source = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "audio_rx_telemetry.h"
#define pdTRUE 1
typedef struct {audio_rx_telemetry_snapshot_t snapshot;uint32_t snapshots_dropped,events_dropped,sequence;} telemetry_record_t;
typedef struct {audio_rx_telemetry_event_kind_t kind;uint32_t a,b,c,d,e;} telemetry_event_t;
static int s_snapshots=1,s_events=2,s_telemetry_task=3;
static telemetry_record_t s_publish_record,last_record;
static uint32_t s_snapshots_dropped,s_events_dropped,s_snapshot_sequence;
static unsigned notifications;
static bool queue_available=true;
static int xQueueSend(int queue,const void*data,unsigned wait){assert(wait==0);if(queue==1)last_record=*(const telemetry_record_t*)data;else assert(queue==2);return queue_available;}
static void xTaskNotifyGive(int task){assert(task==3);++notifications;}
''' + function(rx, "audio_rx_telemetry_publish") + "\n" + function(rx, "audio_rx_telemetry_event") + r'''
int main(void){audio_rx_telemetry_snapshot_t snapshot={.session_id=7,.codec=3,.active=true,.sampled_us=100};
 assert(audio_rx_telemetry_publish(&snapshot));snapshot.session_id=8;snapshot.codec=4;assert(last_record.snapshot.session_id==7&&last_record.snapshot.codec==3&&last_record.sequence==1);
 queue_available=false;assert(!audio_rx_telemetry_publish(&snapshot));assert(s_snapshots_dropped==1&&notifications==1);
 audio_rx_telemetry_event(AUDIO_RX_LOG_SESSION_BEGIN,1,2,3,4,5);assert(s_events_dropped==1&&notifications==1);
 queue_available=true;assert(audio_rx_telemetry_publish(&snapshot));assert(last_record.sequence==3&&last_record.snapshots_dropped==1&&last_record.events_dropped==1);
 s_snapshots_dropped=s_events_dropped=UINT32_MAX;queue_available=false;assert(!audio_rx_telemetry_publish(&snapshot));audio_rx_telemetry_event(AUDIO_RX_LOG_SESSION_BEGIN,0,0,0,0,0);assert(s_snapshots_dropped==UINT32_MAX&&s_events_dropped==UINT32_MAX);
 puts("RX snapshot identity copying; nonblocking failure/drop and saturation PASS");return 0;}
'''
        self.assertIn("PASS", compile_run(source, includes=[ROOT / "dongle/components/dual_end_audio/include", ROOT / "components/dual_end_audio_recovery/include", ROOT / "dongle/components/dual_end_common/include"]))

    def test_real_format_bounds_uint64_identity_and_unknown(self):
        source = r'''
#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "dual_end_telemetry_format.h"
#include "dual_end_resource_format.h"
int main(void){
 char line[181],small[12];
 dual_end_spatial_telemetry_t s={.boot_id=UINT32_MAX,.session_id=UINT32_MAX,
  .sequence=UINT32_MAX,.role=0,.codec=4,.codec_known=true,.sampled_us=UINT64_MAX,
  .generation=UINT16_MAX,.requested=UINT16_MAX,.applied=UINT16_MAX,.layout=UINT16_MAX,
  .fallback=UINT16_MAX,.polarity_inverted=true,.delay_samples=-1,.correlation=-1,
  .current_mix=1,.frames=UINT64_MAX,.learned_frames=UINT64_MAX,.frozen_frames=UINT64_MAX,
  .mixed_frames=UINT64_MAX,.nlms_fallback_frames=UINT64_MAX,.mix_q16_sum=UINT64_MAX,
  .processing_us=UINT64_MAX,.processing_frames=UINT32_MAX,.processing_max_us=UINT32_MAX,
  .bounded_updates=UINT32_MAX,.resets=UINT32_MAX};
 for(unsigned p=0;p<15;++p){size_t n=dual_end_spatial_telemetry_format(&s,p,line,sizeof(line));assert(n>0&&n<=180&&n==strlen(line));puts(line);
  assert(!dual_end_spatial_telemetry_format(&s,p,small,sizeof(small))&&small[0]=='\0');}
 assert(!dual_end_spatial_telemetry_format(&s,15,line,sizeof(line)));
 s.frames=0;s.codec_known=false;assert(dual_end_spatial_telemetry_format(&s,1,line,sizeof(line)));puts(line);
 assert(dual_end_spatial_telemetry_format(&s,5,line,sizeof(line)));puts(line);
 s.delay_samples=NAN;assert(!dual_end_spatial_telemetry_format(&s,0,line,sizeof(line)));
 s.delay_samples=0;s.boot_id=0;assert(!dual_end_spatial_telemetry_format(&s,0,line,sizeof(line)));
 dual_end_rx_telemetry_t r={.boot_id=UINT32_MAX,.session_id=UINT32_MAX,.sequence=UINT32_MAX,
  .role=1,.codec=3,.codec_known=true,.active=true,.sampled_us=UINT64_MAX,
  .snapshot_age_us=UINT64_MAX,.snapshot_dropped=UINT32_MAX,.event_dropped=UINT32_MAX,
  .rx_stack_min_bytes=UINT32_MAX,.telemetry_stack_min_bytes=UINT32_MAX};
 for(unsigned p=0;p<6;++p){size_t n=dual_end_rx_telemetry_format(&r,p,line,sizeof(line));assert(n>0&&n<=180&&n==strlen(line));puts(line);
  assert(!dual_end_rx_telemetry_format(&r,p,small,sizeof(small))&&small[0]=='\0');}
 r.codec_known=false;assert(dual_end_rx_telemetry_format(&r,1,line,sizeof(line)));puts(line);
 assert(!dual_end_rx_telemetry_format(&r,6,line,sizeof(line)));
 r.role=0;assert(!dual_end_rx_telemetry_format(&r,0,line,sizeof(line)));
 dual_end_resource_snapshot_t d={.boot_id=UINT32_MAX,.sequence=UINT32_MAX,
  .sampled_us=UINT64_MAX,.interval_us=UINT64_MAX,.role=1,.cpu_busy_permille={-1,1000},
  .internal_free=UINT32_MAX,.internal_min=UINT32_MAX,.internal_largest=UINT32_MAX,
  .psram_free=UINT32_MAX,.psram_min=UINT32_MAX,.psram_largest=UINT32_MAX,
  .logger_stack=UINT32_MAX,.telemetry_drop=UINT32_MAX,.io_available=true,
  .io={UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX}};
 for(unsigned p=0;p<10;++p){size_t n=dual_end_resource_format(&d,p,line,sizeof(line));assert(n>0&&n<=180);puts(line);}
 fprintf(stderr,"snapshot bytes spatial=%zu rx=%zu\n",sizeof(s),sizeof(r));return 0;
}
'''
        rows = [json.loads(line) for line in compile_run(source, sources=[RUNTIME / "dual_end_telemetry_format.c", RUNTIME / "dual_end_resource_format.c"]).splitlines()]
        spatial = rows[:15]
        self.assertEqual([r["part"] for r in spatial], list(range(15)))
        self.assertTrue(all(r["boot_id"] == "ffffffff" and r["role"] == 0 and r["session_id"] == 4294967295 for r in spatial))
        merged = dict(item for row in spatial for item in row.items())
        self.assertEqual(merged["mix_q16_sum"], "18446744073709551615")
        self.assertEqual(merged["frames"], "18446744073709551615")
        self.assertEqual(rows[15]["codec"], None)
        self.assertEqual(rows[16]["mean_mix"], None)
        self.assertEqual([r["part"] for r in rows[17:23]], list(range(6)))
        self.assertEqual(rows[23]["codec"], None)
        self.assertEqual(rows[29]["cpu0_permille"], None)

    def test_real_idle_logger_emits_resources_without_codec_activity(self):
        runtime = (RUNTIME / "dual_end_runtime.c").read_text(encoding="utf-8")
        source = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <setjmp.h>
#include "dual_end_perf_format.h"
#define DUAL_END_STAGE_COUNT 4
#define DUAL_END_STAGE_MIC_PIPELINE 1
#define RUNTIME_WINDOW_US INT64_C(5000000)
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(x) (x)
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
#define MALLOC_CAP_SPIRAM 4
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define ESP_LOGI(...) (++lines)
typedef struct {dual_end_perf_record_t record;void *owner;} runtime_snapshot_t;
static int s_snapshots,s_resource_lock;
static dual_end_perf_record_t s_perf_cache[4];
static bool s_perf_ready[4];
static unsigned polls,lines,identities,resource_samples,resource_batches,health_batches,build_batches;
static int64_t now=1000000;
static jmp_buf done;
static int64_t esp_timer_get_time(void){return now;}
static void runtime_sample_resources(void){++resource_samples;}
static void runtime_log_resources(void){++resource_batches;}
static void runtime_log_transport_health(void){++health_batches;}
static void vTaskDelay(unsigned ticks){(void)ticks;assert(false);}
static unsigned heap_caps_get_free_size(unsigned c){(void)c;return 4000;}
static unsigned heap_caps_get_minimum_free_size(unsigned c){(void)c;return 300;}
static unsigned uxTaskGetStackHighWaterMark(void*p){(void)p;return 1000;}
static void runtime_log_identity(int c){assert(c==0);++identities;}
static size_t runtime_format_build_info(char *out,size_t capacity){assert(out&&capacity>1);++build_batches;out[0]='x';out[1]=0;return 1;}
static int xPortGetCoreID(void){return 0;}
static int ulTaskNotifyTake(int c,unsigned t){assert(c&&t==UINT32_MAX);return 1;}
static int xQueueReceive(int q,runtime_snapshot_t*s,unsigned t){(void)q;(void)s;assert(t==250);if(++polls==160)longjmp(done,1);now+=250000;return 0;}
size_t dual_end_perf_format_part(const dual_end_perf_record_t*r,unsigned p,char*b,size_t c){(void)r;(void)p;(void)b;(void)c;assert(false);return 0;}
size_t dual_end_owner_outcome_format_part(const dual_end_perf_record_t*r,unsigned p,char*b,size_t c){(void)r;(void)p;(void)b;(void)c;assert(false);return 0;}
''' + function(runtime, "runtime_logger") + r'''
int main(void){if(!setjmp(done))runtime_logger(NULL);assert(resource_samples==8&&resource_batches==8&&identities==2&&build_batches==2&&lines==2);assert(health_batches==8);puts("idle resource and identity cadence PASS");return 0;}
'''
        self.assertIn("PASS", compile_run(source))

    def test_owner_spatial_cache_does_not_read_sender_or_mutable_logger_state(self):
        mic = (ROOT / "scepter/main/mic/mic_audio.c").read_text(encoding="utf-8")
        source = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "dual_end_telemetry_format.h"
#include "dual_end_spatial.h"
#define DUAL_END_STAGE_MIC_PIPELINE 1
static unsigned locked;
#define portENTER_CRITICAL(x) ((void)(x),assert(!locked),++locked)
#define portEXIT_CRITICAL(x) ((void)(x),assert(locked),--locked)
static int s_config_lock;
static dual_end_spatial_t spatial_storage;
static dual_end_spatial_t *s_spatial = &spatial_storage;
static dual_end_spatial_telemetry_t s_spatial_telemetry;
static uint16_t s_spatial_requested_mode;
static uint64_t s_spatial_work_us;
static uint32_t s_spatial_work_frames,s_spatial_work_max_us;
static bool context_available=true;
static uint32_t dual_end_runtime_boot_id(void){return 7;}
static int64_t esp_timer_get_time(void){assert(!locked);return 3000;}
static bool dual_end_runtime_owner_context(unsigned stage,uint32_t*session,uint8_t*codec,uint64_t*sample){assert(stage==1&&!locked);if(!context_available)return false;*session=9;*codec=3;*sample=2000;return true;}
''' + function(mic, "mic_spatial_cache_snapshot") + r'''
int main(void){s_spatial->stats.generation=5;s_spatial->stats.mode=2;s_spatial->stats.frames=UINT64_C(9007199254740993);s_spatial->stats.matching_mix_q16_sum=UINT64_C(9007199254740995);s_spatial_requested_mode=2;s_spatial_work_us=555;s_spatial_work_frames=11;
 mic_spatial_cache_snapshot();assert(!locked&&s_spatial_telemetry.codec_known&&s_spatial_telemetry.session_id==9&&s_spatial_telemetry.sampled_us==2000&&s_spatial_telemetry.generation==5&&s_spatial_telemetry.processing_us==555);
 s_spatial->stats.generation=6;s_spatial->stats.frames=0;assert(s_spatial_telemetry.generation==5&&s_spatial_telemetry.frames==UINT64_C(9007199254740993));
 /* New NLMS stereo must remain distinguishable from fused dual mono in the
  * owner-coherent telemetry; no schema or field meaning changes are needed. */
 s_spatial->stats.mode=3;s_spatial->stats.layout=0;s_spatial_requested_mode=3;
 mic_spatial_cache_snapshot();assert(s_spatial_telemetry.requested==3&&s_spatial_telemetry.applied==3&&s_spatial_telemetry.layout==0);
 context_available=false;mic_spatial_cache_snapshot();assert(!s_spatial_telemetry.codec_known&&!s_spatial_telemetry.session_id&&s_spatial_telemetry.sampled_us==3000);puts("owner coherent snapshot and explicit unknown PASS");return 0;}
'''
        self.assertIn("PASS", compile_run(source, includes=[ROOT / "components/dual_end_audio_codec/include"]))


if __name__ == "__main__":
    unittest.main()
