"""Actual resume/warmup helpers with fault injection; no device IO."""
from pathlib import Path
from test_runtime_codec import function
from test_audio_recovery_042 import _build_host_executable,_run
ROOT=Path(__file__).resolve().parents[1]

def test_resume_warmup_failure_and_stop_never_publish_active(tmp_path):
    s=(ROOT/'scepter/main/mic/mic_audio.c').read_text(encoding='utf-8')
    code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_TIMEOUT 1
#define ESP_ERR_INVALID_STATE 2
#define MIC_STATE_SUSPENDED_BIT 2U
#define MIC_STATE_TIMEOUT_MS 1500
#define pdFALSE 0
#define pdTRUE 1
#define ESP_RETURN_ON_FALSE(condition,err,...) do {if(!(condition))return (err);}while(0)
#define MIC_STATE_ACTIVE_BIT 1U
#define MIC_STATE_FAULT_BIT 16U
#define MIC_RESUME_SETTLE_MS 10
#define pdMS_TO_TICKS(x) (x)
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define ESP_LOGI(...) ((void)0)
#define ESP_RETURN_ON_ERROR(expr,...) do {int rc=(expr);if(rc)return rc;}while(0)
typedef int esp_err_t;
typedef unsigned EventBits_t;
static void *s_mic_task_handle=(void*)(uintptr_t)1;
static int s_state_change_result;
static unsigned wait_reply,notifications;
static bool s_capture_enabled,s_capture_io_running,s_dma_overflow_pending;
static int s_mic_rx_chan,s_pipeline_lock;
static int s_mic_state_events=1;
static int16_t s_warmup_discard_buffer[32];
static unsigned fixture_bits,reads,failread,resets,shortread;
/* 0.5.9 added matcher reset to a completed physical warmup. Keep this real
 * resume helper under test instead of silently omitting its new dependency. */
static struct { struct { uint16_t mode; } stats; } spatial_storage, *s_spatial=&spatial_storage;
static unsigned spatial_resets;
static void mic_spatial_reset_state_owner(uint16_t mode){assert(mode==2);++spatial_resets;}
static int enable_result;
static void xEventGroupSetBits(int e,unsigned value){(void)e;fixture_bits|=value;}
static void xEventGroupClearBits(int e,unsigned value){(void)e;fixture_bits&=~value;}
static unsigned xEventGroupGetBits(int e){(void)e;return fixture_bits;}
static void xTaskNotifyGive(void *task){assert(task);++notifications;}
static unsigned xEventGroupWaitBits(int e,unsigned mask,int clear,int all,unsigned timeout){
 (void)e;assert(mask==17 || mask==18);assert(!clear && !all && timeout==1500 && fixture_bits==0);return wait_reply;}
static int i2s_channel_enable(int channel){(void)channel;return enable_result;}
static void vTaskDelay(unsigned ticks){assert(ticks==10);}
static void mic_filter_reset(void){++resets;}
static int i2s_channel_read(int channel,void *out,size_t size,size_t *bytes,unsigned ticks){
 (void)channel;(void)out;assert(size==64 && ticks==100);++reads;
 *bytes=shortread?32:64;return reads==failread?ESP_ERR_TIMEOUT:ESP_OK;
}
'''+function(s,'mic_warmup_discard')+'\n'+function(s,'mic_hardware_resume')+'\n'+function(s,'mic_publish_resume_result')+'\n'+function(s,'mic_audio_set_enabled')+r'''
static void reset(void){fixture_bits=1;reads=failread=resets=shortread=spatial_resets=0;enable_result=0;s_spatial->stats.mode=2;
 s_capture_enabled=s_capture_io_running=s_dma_overflow_pending=true;}
int main(void){
 reset();assert(mic_hardware_resume()==0 && reads==8 && resets==1 && spatial_resets==1 && !s_dma_overflow_pending);
 mic_publish_resume_result(0);assert(fixture_bits==1 && s_capture_enabled);
 reset();enable_result=-1;int result=mic_hardware_resume();assert(result==-1 && !reads && !resets && !spatial_resets);
 mic_publish_resume_result(result);assert(fixture_bits==16 && !s_capture_enabled && !s_capture_io_running);
 reset();failread=3;result=mic_hardware_resume();assert(result==1 && reads==3 && !resets && !spatial_resets);
 mic_publish_resume_result(result);assert(fixture_bits==16 && !s_capture_enabled);
 reset();shortread=1;assert(mic_hardware_resume()==-1 && reads==1 && !resets && !spatial_resets);
 reset();s_capture_enabled=false;mic_publish_resume_result(0);assert(fixture_bits==0);
 reset();notifications=0;assert(mic_audio_set_enabled(true)==0 && !notifications);
 reset();fixture_bits=2;s_capture_enabled=false;assert(mic_audio_set_enabled(false)==0 && !notifications);
 reset();fixture_bits=2;s_capture_enabled=s_capture_io_running=false;wait_reply=16;assert(mic_audio_set_enabled(true)==-1 && notifications==1);
 reset();fixture_bits=2;s_capture_enabled=s_capture_io_running=false;wait_reply=0;assert(mic_audio_set_enabled(true)==1);
 reset();fixture_bits=0;s_capture_io_running=false;wait_reply=2;assert(mic_audio_set_enabled(false)==0);
 puts("physical resume, timeout, incomplete warmup, stop-during-warmup and ACK state PASS");return 0;
}
'''
    p=tmp_path/'mic_state.c';p.write_text(code,encoding='utf-8')
    exe,env=_build_host_executable([p],tmp_path)
    assert 'PASS' in _run([str(exe)],cwd=tmp_path,env=env).stdout
