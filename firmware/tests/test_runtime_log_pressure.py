"""Run the actual logger loop against rapid session churn, without hardware."""
from pathlib import Path
from test_runtime_codec import function
from test_audio_task_affinity import compile_run

ROOT = Path(__file__).resolve().parents[1]


def test_session_churn_keeps_cache_fresh_without_serial_flood(tmp_path):
    source = (ROOT / 'components/dual_end_runtime/dual_end_runtime.c').read_text(encoding='utf-8')
    code = r'''
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
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
static unsigned received,lines,identities,resource_samples,health_batches,delays;
static int64_t now=1000000;
static jmp_buf finished;
static int64_t esp_timer_get_time(void){return now;}
static void runtime_sample_resources(void){++resource_samples;}
static void runtime_log_resources(void){lines+=10;}
static void runtime_log_transport_health(void){++health_batches;}
static void vTaskDelay(unsigned ticks){assert(ticks==1);++delays;}
static unsigned heap_caps_get_free_size(unsigned c){(void)c;return 4000;}
static unsigned heap_caps_get_minimum_free_size(unsigned c){(void)c;return 300;}
static unsigned uxTaskGetStackHighWaterMark(void*p){(void)p;return 1000;}
static void runtime_log_identity(int c){assert(c==0);++identities;}
/* Build identity serialization has a separate portable formatter regression;
 * this fixture isolates PERF/session flood volume and cache freshness. */
static size_t runtime_format_build_info(char *out,size_t capacity){assert(out&&capacity);return 0;}
static int xPortGetCoreID(void){return 0;}
static int ulTaskNotifyTake(int c,unsigned t){assert(c && t==UINT32_MAX);return 1;}
static int xQueueReceive(int q,runtime_snapshot_t*s,unsigned t){
 (void)q;assert(t==250);
 if(received==50)longjmp(finished,1);
 ++received;now+=300000; s->record.stage=0;s->record.window_seq=received;
 s->record.stats.session_id=received;s->owner=NULL;return 1;
}
size_t dual_end_perf_format_part(const dual_end_perf_record_t*r,unsigned p,char*b,size_t c){
 (void)b;(void)c;assert(r->window_seq>0 && p<13);return 1;
}
size_t dual_end_owner_outcome_format_part(const dual_end_perf_record_t*r,unsigned p,char*b,size_t c){
 (void)r;(void)p;(void)b;(void)c;assert(false);return 0;
}
''' + function(source, 'runtime_logger') + r'''
int main(void){
 if(!setjmp(finished))runtime_logger(NULL);
 assert(received==50 && s_perf_ready[0] && s_perf_cache[0].window_seq==50);
 assert(lines==69 && identities==1 && resource_samples==3);
 assert(health_batches==3 && delays==39);
 puts("50 session windows cached, only 3 serial batches: PASS");return 0;
}
'''
    assert 'PASS' in compile_run(tmp_path, code)
