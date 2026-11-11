"""Actual IO task with deterministic DMA/queue stubs; no hardware timing claim."""
from pathlib import Path
from test_runtime_codec import function
from test_audio_recovery_042 import _build_host_executable, _run

ROOT=Path(__file__).resolve().parents[1]

import pytest

@pytest.mark.parametrize("external", [0, 1])
def test_pipeline_startup(tmp_path, external):
    source=(ROOT/'scepter/main/mic/mic_audio.c').read_text(encoding='utf-8')
    task=function(source,'mic_audio_task')
    prefix=task[:task.index('    while (true) {')]+'\n}\n'
    block_type=source[source.index('typedef struct {',source.index('#define MIC_PIPELINE_BLOCKS')):source.index('static QueueHandle_t s_capture_blocks')]
    sizes=source[source.index('#define MIC_CAPTURE_SAMPLE_RATE_HZ'):source.index('#define MIC_I2S_NUM')]
    code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <setjmp.h>
#include <stdio.h>
#include "dual_end_link_config.h"
/* The extracted startup prefix intentionally has no processing loop; its
 * processing locals are unused here. Production is compiled independently. */
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-function"
#else
#pragma warning(disable:4101 4189 4324 4505)
#endif
#define pdTRUE 1
#define pdPASS 1
#define pdFAIL 0
#define portMAX_DELAY UINT32_MAX
#define MIC_START_READY_BIT 4
#define MIC_START_FAILED_BIT 8
#define MIC_PIPELINE_BLOCKS 4
#define MIC_FRAME_DISPATCH_BLOCKS 2
#define MIC_CAPTURE_TASK_STACK 4096
#define MIC_DIAGNOSTIC_TASK_STACK 8192
#define MIC_TELEMETRY_TASK_STACK 6144
#define MIC_CONFIG_TASK_STACK 4096
#define MIC_FRAME_DISPATCH_TASK_STACK (40 * 1024)
#define MIC_FRAME_DISPATCH_TASK_PRIORITY 4
#define MIC_BACKGROUND_TASK_PRIORITY 1
#define MIC_BACKGROUND_TASK_CORE 0
#define DUAL_END_AUDIO_TASK_CORE 1
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
#define MALLOC_CAP_SPIRAM 4
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
typedef unsigned TickType_t;
typedef int BaseType_t;
typedef int dual_end_owner_outcome_t;
typedef struct {
    uint32_t session_id;
    uint32_t mutex_wait_us;
    uint32_t encode_us;
    uint32_t radio_submit_us;
    uint8_t codec;
    bool valid;
    bool encode_attempted;
    bool encode_ok;
    bool radio_submitted;
    bool radio_ok;
    dual_end_owner_outcome_t outcome;
} dual_end_audio_owner_timing_t;
typedef void (*mic_audio_frame_cb_t)(const int16_t *, size_t,
    const dual_end_audio_analysis_metadata_t *, void *);
PLACEHOLDER_BLOCK_TYPE
static void *s_capture_blocks,*s_diagnostic_blocks,*s_capture_io_lock;
static void *s_frame_dispatch_blocks,*s_owner_timing_events,*s_frame_dispatch_control;
static void *s_capture_task,*s_diagnostic_task,*s_telemetry_task,*s_frame_dispatch_task;
static void *s_config_task_handle;
static bool s_capture_io_running,s_analysis_available=true;
static void mic_analysis_release(void){}
static void *s_mic_state_events=(void*)(uintptr_t)1;
static unsigned allocations,active,frees,fail1,fail2,taskfail,creates,taskactive,notifications,ready,failed;
static unsigned char storage[12][20000];static bool live[12];
static jmp_buf done;
static unsigned ulTaskNotifyTake(int clear,unsigned timeout){
 assert(clear==1 && timeout==UINT32_MAX);
 if(!notifications++){assert(!allocations);return 1;}
 assert(failed==1 && !ready && !active && !taskactive);longjmp(done,2);
}
static void *fake_malloc(size_t bytes){
 unsigned call=++allocations;assert(call<=12 && bytes<=20000);
 if(call==fail1 || call==fail2)return NULL;
 assert(!live[call-1]);live[call-1]=true;++active;return storage[call-1];
}
static void fake_free(void *ptr){
 if(!ptr)return;
 for(unsigned i=0;i<12;++i)if(ptr==storage[i]){assert(live[i]);live[i]=false;--active;++frees;return;}
 assert(false);
}
static void *heap_caps_aligned_alloc(size_t alignment,size_t bytes,unsigned caps){assert(alignment==16 && (caps==3 || caps==6));return fake_malloc(bytes);}
#define malloc fake_malloc
#define free fake_free
static void *xQueueCreate(unsigned depth,size_t bytes){assert(depth==4 || depth==2 || depth==1);return fake_malloc(depth*bytes);}
static void *xSemaphoreCreateMutex(void){return fake_malloc(16);}
static void vQueueDelete(void *ptr){fake_free(ptr);}
static void vSemaphoreDelete(void *ptr){fake_free(ptr);}
static void mic_capture_io_task(void *ptr){(void)ptr;}
static void mic_diagnostic_task(void *ptr){(void)ptr;}
static void mic_telemetry_task(void *ptr){assert(!ptr);}
static void mic_frame_dispatch_task_fn(void *ptr){(void)ptr;}
static void mic_config_transition_task_fn(void *ptr){(void)ptr;}
static BaseType_t xTaskCreatePinnedToCore(void(*entry)(void*),const char *name,unsigned stack,
 void *arg,unsigned priority,void **handle,unsigned core){
 (void)name;assert(!s_capture_io_running);
 if(entry==mic_capture_io_task)assert(arg && core==1 && stack==4096 && priority==6);
 else if(entry==mic_frame_dispatch_task_fn)assert(!arg && core==1 && stack==40*1024 && priority==4);
 else if(entry==mic_config_transition_task_fn)assert(!arg && core==0 && stack==4096 && priority==1);
 else if(entry==mic_diagnostic_task)assert(arg && core==0 && stack==8192 && priority==1);
 else assert(entry==mic_telemetry_task && !arg && core==0 && stack==6144 && priority==1);
 ++creates;if(creates==taskfail)return 0;
 *handle=(void*)(uintptr_t)(100U+creates);++taskactive;return 1;
}
static BaseType_t xTaskCreatePinnedToCoreWithCaps(void(*entry)(void*),const char *name,
 unsigned stack,void *arg,unsigned priority,void **handle,unsigned core,unsigned caps){
 assert(caps==6 && (entry==mic_frame_dispatch_task_fn || entry==mic_diagnostic_task || entry==mic_telemetry_task));
 return xTaskCreatePinnedToCore(entry,name,stack,arg,priority,handle,core);
}
static void vTaskDelete(void *handle){assert(handle && taskactive);--taskactive;}
static void vTaskDeleteWithCaps(void *handle){vTaskDelete(handle);}
static void xTaskNotifyGive(void *handle){assert(handle && taskactive>=3 && taskactive<=5 && s_capture_io_running);++notifications;}
static void xEventGroupSetBits(void *events,unsigned bits){
 assert(events==s_mic_state_events);
 if(bits==8){assert(!active && !taskactive && !ready);++failed;return;}
 assert(bits==4 && active>=8 && active<=10 && taskactive>=3 && taskactive<=5 && notifications==taskactive-1 && !failed);
 ++ready;longjmp(done,1);
}
'''.replace('PLACEHOLDER_BLOCK_TYPE',block_type.replace('MIC_CAPTURE_FRAMES_PER_PACKET * MIC_CHANNEL_COUNT','1920').replace('MIC_OUTPUT_SAMPLE_VALUES_PER_PACKET','640'))+f'\n#define MIC_TASK_PSRAM_STACK {external}\n'+sizes+prefix+r'''
static void reset(unsigned first,unsigned second,unsigned task){
 allocations=active=frees=creates=taskactive=notifications=ready=failed=0;
 fail1=first;fail2=second;taskfail=task;
 s_capture_blocks=s_diagnostic_blocks=s_capture_io_lock=s_capture_task=s_diagnostic_task=s_telemetry_task=NULL;
 s_frame_dispatch_blocks=s_owner_timing_events=s_frame_dispatch_control=s_frame_dispatch_task=s_config_task_handle=NULL;
 s_capture_io_running=false;s_analysis_available=true;for(unsigned i=0;i<12;++i)live[i]=false;
}
static void run(unsigned first,unsigned second,unsigned task,bool succeeds){
 reset(first,second,task);int outcome=setjmp(done);if(!outcome)mic_audio_task(NULL);
 if(succeeds)assert(outcome==1 && ready==1 && !failed && active>=8 && active<=10 && taskactive>=3 && taskactive<=5);
 else assert(outcome==2 && !ready && failed==1 && !active && !taskactive);
}
int main(void){
 run(0,0,0,true);
 for(unsigned failure=1;failure<=10;++failure)run(failure,0,0,failure==3 || failure==6 || failure==8);
 run(3,4,0,false); /* Optional diagnostic failure plus mandatory PCM failure. */
 run(0,0,1,false);run(0,0,2,false);run(0,0,3,false);
 run(0,0,4,true);run(0,0,5,true);
 reset(0,0,0);s_analysis_available=false;
 int outcome=setjmp(done);if(!outcome)mic_audio_task(NULL);
 assert(outcome==1 && ready && active==8 && taskactive==4 && s_diagnostic_blocks==NULL && s_telemetry_task);
 puts("actual pipeline startup: mandatory IO plus independent optional FFT/telemetry allocation/task-create scenarios PASS");return 0;
}
'''
    harness=tmp_path/'pipeline_startup.c';harness.write_text(code,encoding='utf-8')
    executable,env=_build_host_executable([harness],tmp_path)
    assert 'PASS' in _run([str(executable)],cwd=tmp_path,env=env).stdout

def test_actual_capture_task_partial_timeout_generation_and_overload(tmp_path):
    source=(ROOT/'scepter/main/mic/mic_audio.c').read_text(encoding='utf-8')
    task=function(source,'mic_capture_io_task')
    code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <setjmp.h>
#include <stdio.h>
#define MIC_CAPTURE_BUFFER_BYTES 3840
#define MIC_CHANNEL_COUNT 2
#define MIC_READ_TIMEOUT_MS 50
#define pdTRUE 1
#define portMAX_DELAY 0xffffffffU
#define pdMS_TO_TICKS(x) (x)
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define ESP_LOGE(...) ((void)0)
#define ESP_OK 0
#define ESP_ERR_TIMEOUT 1
typedef int esp_err_t;
typedef struct {uint32_t sequence,generation;bool motor_active;uint64_t captured_us,enqueued_us;int16_t pcm[1920];} mic_capture_block_t;
static bool s_capture_enabled,s_capture_io_running,s_motor_active;
static int s_capture_io_lock,s_mic_rx_chan,s_pipeline_lock,s_capture_blocks;
static uint32_t s_pipeline_generation,s_capture_sequence,s_capture_overloads,s_pipeline_highwater;
static bool s_dma_overflow_pending;
static int s_mic_state_events;
#define MIC_STATE_FAULT_BIT 16
static void xEventGroupSetBits(int events,unsigned bits){(void)events;assert(bits==16);}
static int scenario,reads,sends,locked;
static jmp_buf done;
static mic_capture_block_t received;
static int64_t esp_timer_get_time(void){return 1234000+reads;}
static unsigned ulTaskNotifyTake(int a,unsigned b){(void)a;(void)b;return 1;}
static void vTaskDelay(unsigned ticks){(void)ticks;assert(!locked);longjmp(done,1);}
static void xSemaphoreTake(int lock,unsigned ticks){(void)lock;(void)ticks;assert(!locked);locked=1;}
static void xSemaphoreGive(int lock){(void)lock;assert(locked);locked=0;}
static unsigned uxQueueMessagesWaiting(int q){(void)q;return 1;}
static int xQueueSend(int q,const mic_capture_block_t *block,unsigned ticks){
    (void)q;assert(ticks==0 && locked);received=*block;++sends;
    if(scenario==1)return 0;
    s_capture_enabled=false;return 1;
}
static esp_err_t i2s_channel_read(int channel,void *out,size_t requested,size_t *bytes,unsigned timeout){
    (void)channel;(void)timeout;assert(locked);++reads;
    if(scenario==3){*bytes=0;return 2;}
    if(scenario==5){*bytes=0;return ESP_ERR_TIMEOUT;}
    if((scenario==0 || scenario==2) && reads==1){
        assert(requested==3840);*bytes=1920;memset(out,0x11,*bytes);
        if(scenario==2)++s_pipeline_generation;
        return ESP_ERR_TIMEOUT;
    }
    if(scenario==0)assert(requested==1920);
    else assert(requested==3840);
    *bytes=requested;memset(out,0x22,*bytes);return ESP_OK;
}
'''+task+r'''
int main(void){
 for(scenario=0;scenario<6;++scenario){
    mic_capture_block_t block={0};memset(&received,0,sizeof(received));
    s_capture_enabled=s_capture_io_running=true; s_motor_active=true;
    s_dma_overflow_pending=scenario==4;
    s_pipeline_generation=7;s_capture_sequence=s_capture_overloads=s_pipeline_highwater=0;
    reads=sends=locked=0;
    if(!setjmp(done))mic_capture_io_task(&block);
    assert(!s_capture_enabled && !locked);
    if(scenario==3){assert(!sends && reads==1);continue;}
    if(scenario==4){assert(!sends && !reads);continue;}
    if(scenario==5){assert(!sends && reads==3);continue;}
    assert(sends==1 && received.sequence==1 && received.motor_active);
    assert(received.captured_us>0 && received.enqueued_us>=received.captured_us);
    assert(received.generation==(scenario==2 ? 8U:7U));
    assert(s_capture_overloads==(scenario==1 ? 1U:0U));
    const uint8_t *pcm=(const uint8_t*)received.pcm;
    for(unsigned i=0;i<3840;++i)assert(pcm[i]==(scenario==0 && i<1920 ? 0x11:0x22));
 }
 puts("actual capture: partial-timeout samples preserved; reset generation; queue-full stop; IO failure stop PASS");
 return 0;
}
'''
    harness=tmp_path/'capture_task.c';harness.write_text(code,encoding='utf-8')
    executable,env=_build_host_executable([harness],tmp_path)
    assert 'PASS' in _run([str(executable)],cwd=tmp_path,env=env).stdout

def test_actual_diagnostic_task_gap_reset_and_stale_generation(tmp_path):
    source=(ROOT/'scepter/main/mic/mic_audio.c').read_text(encoding='utf-8')
    task=function(source,'mic_diagnostic_task')
    code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <setjmp.h>
#include <stdio.h>
#define MIC_CAPTURE_FRAMES_PER_PACKET 960
#define MIC_DIAGNOSTIC_MAX_GAP_FRAMES 50U
#define pdTRUE 1
#define portMAX_DELAY 0xffffffffU
#define pdMS_TO_TICKS(x) (x)
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#include <inttypes.h>
#define ESP_LOGI(tag,...) ((void)printf(__VA_ARGS__))
typedef unsigned TickType_t;
#include "dual_end_link_config.h"
#include "mic_pipeline_telemetry.h"
typedef struct {uint32_t sequence,generation;bool motor_active;uint64_t captured_us,enqueued_us;int16_t pcm[1920];} mic_capture_block_t;
static int s_pipeline_lock,s_diagnostic_blocks,calls,resets;
static uint32_t gaps[9];
static int64_t s_diagnostic_completed_us;
static int64_t fixture_clock=1000000;
static int64_t esp_timer_get_time(void){return fixture_clock;}
static void mic_analysis_set_frame_gap(uint32_t gap){gaps[calls-1]=gap;}
static uint32_t s_pipeline_generation=1,s_diagnostic_sequence,s_diagnostic_frames;
static uint32_t s_diagnostic_max_us;
static uint64_t s_diagnostic_total_us;
static uint64_t s_diagnostic_completed_frames,s_diagnostic_completed_total_us,s_pipeline_last_overrun_us;
static uint32_t s_diagnostic_lifetime_max_us,s_diagnostic_budget_skips,s_diagnostic_overload_skips,s_diagnostic_stale_skips;
static dual_end_audio_analysis_metadata_t s_diagnostic_metadata;
#include "dual_end_telemetry_format.h"
static jmp_buf done;
static unsigned ulTaskNotifyTake(int a,unsigned b){(void)a;(void)b;return 1;}
static int xQueueReceive(int q,mic_capture_block_t *block,unsigned ticks){
    (void)q;(void)ticks;
    if(calls==9)longjmp(done,1);
    memset(block,0,sizeof(*block));
    static const unsigned sequences[9]={10,20,30,40,100,110,120,130,140};
    block->sequence=sequences[calls];
    block->generation=calls==3?1U:(calls<2?1U:2U);
    if(calls==2)s_pipeline_generation=2;
    fixture_clock=1000000+calls*200000;block->captured_us=(uint64_t)fixture_clock;
    if(calls==5)block->captured_us-=500000; /* stale age, not queue failure */
    s_pipeline_last_overrun_us=calls==6 ? (uint64_t)fixture_clock-100000 : 0;
    block->pcm[0]=(int16_t)block->sequence;++calls;return 1;
}
static void mic_analysis_reset(void){++resets;}
static uint32_t mic_analysis_process_frame(const int16_t *pcm,size_t frames,bool motor,
    dual_end_audio_analysis_metadata_t *metadata){
    (void)motor;assert(frames==960);metadata->confidence_q15=(uint16_t)pcm[0];fixture_clock+=23;
    if(calls==9)++s_pipeline_generation; /* change while FFT executes: pay cost, reject metadata */
    return 23;
}
'''+task+r'''
int main(void){
 mic_capture_block_t block;
 if(!setjmp(done))mic_diagnostic_task(&block);
 assert(resets==3 && s_diagnostic_frames==5 && s_diagnostic_total_us==115);
 assert(gaps[1]==10 && gaps[4]==70 && gaps[7]==30);
 assert(s_diagnostic_max_us==23 && s_diagnostic_sequence==130 && s_diagnostic_metadata.confidence_q15==130 && s_diagnostic_completed_us==2400023);
 assert(s_diagnostic_completed_frames==6 && s_diagnostic_completed_total_us==138 && s_diagnostic_lifetime_max_us==23);
 assert(s_diagnostic_stale_skips==3 && s_diagnostic_overload_skips==1 && !s_diagnostic_budget_skips);
 puts("actual diagnostic: scheduled gap retains state; long gap resets; stale generation/age and owner overload are skipped separately PASS");return 0;
}
'''
    harness=tmp_path/'diagnostic_task.c';harness.write_text(code,encoding='utf-8')
    runtime=ROOT/'components/dual_end_runtime'
    executable,env=_build_host_executable([harness,runtime/'dual_end_telemetry_format.c'],tmp_path,
        extra_includes=[runtime/'include',ROOT/'scepter/main/mic'])
    assert 'PASS' in _run([str(executable)],cwd=tmp_path,env=env).stdout


def check_pipeline_startup(tmp_path):
    test_pipeline_startup(tmp_path, 0)
