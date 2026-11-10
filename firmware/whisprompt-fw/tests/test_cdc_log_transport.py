"""Actual CDC mirror admission/TX helpers with a fault-injected host backend.

No USB, audio, flashing or device operation. Uses the shared portable compiler
discovery and tests production C, not a second implementation of its policy.
"""
from pathlib import Path
import re

import pytest

from test_audio_recovery_042 import _build_host_executable, _run
from test_runtime_codec import function


ROOT = Path(__file__).resolve().parents[1]
USB = ROOT / "dongle/components/dongle_usb_device"


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include "dongle_usb_device.h"

#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NOT_FINISHED 0x104
#define pdTRUE 1
#define pdMS_TO_TICKS(ms) ((unsigned)(ms))
#define TINYUSB_CDC_ACM_0 0
#define CDC_OTA_IDLE 0
#define portENTER_CRITICAL(lock) do {assert((lock)==&s_cdc_rx_lock && !locks);++locks;} while(0)
#define portEXIT_CRITICAL(lock) do {assert((lock)==&s_cdc_rx_lock && locks==1);--locks;} while(0)

typedef void *QueueHandle_t;
typedef int (*vprintf_like_t)(const char *, va_list);
'''


GLOBALS = r'''
static unsigned locks;
static int s_cdc_rx_lock, queue_token, worker_token, current_token;
static QueueHandle_t s_cdc_responses=&queue_token;
static void *s_cdc_worker=&worker_token;
static uint32_t s_cdc_epoch=4, s_cdc_processing_epoch=4;
static uint32_t s_cdc_response_drops, s_cdc_response_tx_incomplete;
static uint64_t s_cdc_response_tx_queued_bytes;
static bool s_cdc_tx_needs_boundary;
static bool s_cdc_log_mirror_runtime_enabled=true, s_cdc_dtr=true;
static int s_ota_mode=CDC_OTA_IDLE;
static atomic_uint s_cdc_log_attempted, s_cdc_log_enqueued, s_cdc_log_dropped;
static atomic_uint s_cdc_log_truncated, s_cdc_log_write_failed;
static cdc_response_t queue[16];
static unsigned queue_used, queue_sends, primary_calls, write_calls;
static bool queue_fail, mounted=true, always_zero, negative_format, change_epoch;
static int64_t now_us;
static int flush_result=ESP_OK;
static size_t write_plan[16];
static unsigned write_plan_count, write_plan_cursor;
static char output[4096];
static size_t output_bytes;

static void *xTaskGetCurrentTaskHandle(void) {return &current_token;}
static bool tud_mounted(void) {return mounted;}
static int64_t esp_timer_get_time(void) {return now_us;}
static void vTaskDelay(unsigned ticks) {assert(ticks>0);now_us+=(int64_t)ticks*1000;}
static unsigned uxQueueMessagesWaiting(QueueHandle_t handle) {
    assert(handle==s_cdc_responses);return queue_used;
}
static int xQueueSend(QueueHandle_t handle,const void *item,unsigned wait) {
    assert(handle==s_cdc_responses && wait==0);++queue_sends;
    if(queue_fail || queue_used==16)return 0;
    queue[queue_used++]=*(const cdc_response_t *)item;return pdTRUE;
}
static size_t tinyusb_cdcacm_write_queue(unsigned itf,const uint8_t *data,size_t bytes) {
    assert(itf==0 && bytes>0 && !locks);++write_calls;
    size_t result=always_zero?0:bytes;
    if(write_plan_cursor<write_plan_count)result=write_plan[write_plan_cursor++];
    if(result<=bytes && result>0) {
        assert(output_bytes+result<sizeof(output));
        memcpy(output+output_bytes,data,result);output_bytes+=result;output[output_bytes]=0;
    }
    if(change_epoch) {++s_cdc_epoch;change_epoch=false;}
    return result;
}
static int tinyusb_cdcacm_write_flush(unsigned itf,unsigned wait) {
    assert(itf==0 && wait==0 && !locks);return flush_result;
}
static int primary_log(const char *fmt,va_list args) {
    ++primary_calls;
    char buffer[1024];return fmt?vsnprintf(buffer,sizeof(buffer),fmt,args):-1;
}
static vprintf_like_t s_prev_log_vprintf=primary_log;
static int test_vsnprintf(char *out,size_t cap,const char *fmt,va_list args) {
    return negative_format?-1:vsnprintf(out,cap,fmt,args);
}
#define vsnprintf test_vsnprintf
'''


SUFFIX = r'''
#undef vsnprintf
static int emit(const char *fmt,...) {
    va_list args;va_start(args,fmt);int n=cdc_log_vprintf(fmt,args);va_end(args);return n;
}
static dongle_cdc_log_stats_t snapshot(void) {
    dongle_cdc_log_stats_t s;unsigned writes=write_calls;
    dongle_usb_cdc_get_log_stats(&s);dongle_usb_cdc_get_log_stats(NULL);
    assert(write_calls==writes && !locks);return s;
}
static cdc_response_t log_record(const char *text) {
    cdc_response_t r={.epoch=s_cdc_epoch,.itf=0,.log_line=true};
    r.length=(uint16_t)strlen(text);assert(r.length<sizeof(r.text));
    memcpy(r.text,text,r.length+1U);return r;
}
static void admission(void) {
    assert(emit("one")==3 && emit("two\n")==4);
    assert(queue_used==2 && queue[0].log_line && strcmp(queue[0].text,"one\n")==0);
    assert(queue[0].length==4 && queue[1].length==4);
    emit("backpressure\n");
    dongle_cdc_log_stats_t s=snapshot();
    assert(s.attempted==3 && s.enqueued==2 && s.dropped==1 && !s.truncated);
    queue_used=0;queue_fail=true;emit("queue failure\n");
    queue_fail=false;s_cdc_responses=NULL;emit("queue unavailable\n");
    s=snapshot();assert(s.attempted==5 && s.enqueued==2 && s.dropped==3);
    assert(s.attempted==s.enqueued+s.dropped && !s.write_failed && !s_cdc_response_drops);
    s_cdc_responses=&queue_token;queue_used=2;
    assert(cdc_enqueue(0,"OK reply\n",9,false,false));
    assert(queue_used==3 && !queue[2].log_line);
    s=snapshot();assert(s.attempted==5 && s.enqueued==2 && s.dropped==3);
}
static void formats(void) {
    char maximum[CDC_RESPONSE_TEXT_BYTES];
    memset(maximum,'x',sizeof(maximum)-2U);maximum[sizeof(maximum)-2U]='\n';
    maximum[sizeof(maximum)-1U]=0;
    emit("%s",maximum);assert(queue_used==1 && queue[0].length==sizeof(maximum)-1U);
    queue_used=0;maximum[sizeof(maximum)-2U]='x';emit("%s",maximum);
    char too_long[CDC_RESPONSE_TEXT_BYTES+1U];
    memset(too_long,'x',sizeof(too_long)-1U);too_long[sizeof(too_long)-1U]=0;emit("%s",too_long);
    emit("fragment\nsecond\n");emit("%c",0);emit("");emit(NULL);
    negative_format=true;emit("encoding failure");negative_format=false;
    dongle_cdc_log_stats_t s=snapshot();
    assert(s.attempted==8 && s.enqueued==1 && s.dropped==7 && s.truncated==2);
    assert(queue_used==0 && !s.write_failed && !write_calls);
}
static void disabled(void) {
    s_cdc_log_mirror_runtime_enabled=false;emit("disabled\n");
    s_cdc_log_mirror_runtime_enabled=true;s_cdc_dtr=false;emit("no DTR\n");
    s_cdc_dtr=true;s_ota_mode=1;emit("OTA in progress\n");
    s_ota_mode=0;mounted=false;emit("not mounted\n");
    dongle_cdc_log_stats_t s=snapshot();
    assert(primary_calls==4 && !s.attempted && !s.enqueued && !s.dropped && !queue_sends);
}
static void writes(void) {
    cdc_response_t r=log_record("complete\n");
    flush_result=ESP_ERR_NOT_FINISHED;
    assert(cdc_response_send(&r));
    assert(strcmp(output,"complete\n")==0 && s_cdc_response_tx_queued_bytes==9);
    assert(!snapshot().write_failed && !s_cdc_response_tx_incomplete);
    s_cdc_response_tx_queued_bytes=UINT64_MAX;
    assert(cdc_response_send(&r) && s_cdc_response_tx_queued_bytes==UINT64_MAX);
}
static void failures(void) {
    cdc_response_t r=log_record("complete\n");r.epoch=0;
    assert(!cdc_response_send(&r) && !write_calls);
    r.epoch=s_cdc_epoch;mounted=false;assert(!cdc_response_send(&r));
    mounted=true;always_zero=true;flush_result=ESP_FAIL;assert(!cdc_response_send(&r));
    always_zero=false;flush_result=ESP_OK;write_plan[0]=1000;write_plan_count=1;
    assert(!cdc_response_send(&r));
    assert(snapshot().write_failed==4 && s_cdc_response_tx_incomplete==4 && !output_bytes);
    write_plan_count=0;write_plan_cursor=0;change_epoch=true;
    assert(!cdc_response_send(&r) && snapshot().write_failed==5);
}
static void partial(void) {
    cdc_response_t r=log_record("abcdefgh\n");
    write_plan[0]=3;write_plan_count=1;always_zero=true;
    assert(!cdc_response_send(&r));
    assert(strcmp(output,"abc")==0 && s_cdc_tx_needs_boundary && now_us==100000);
    assert(snapshot().write_failed==1);
    r=log_record("must not concatenate\n");
    assert(!cdc_response_send(&r) && strcmp(output,"abc")==0 && s_cdc_tx_needs_boundary);
    assert(snapshot().write_failed==2);
    always_zero=false;
    r=log_record("OK TELEMETRY token=1 {}\n");r.log_line=false;
    assert(cdc_response_send(&r));
    assert(strcmp(output,"abc\nOK TELEMETRY token=1 {}\n")==0 && !s_cdc_tx_needs_boundary);
    assert(snapshot().write_failed==2 && s_cdc_response_tx_incomplete==2);
}
static void saturation(void) {
    atomic_store(&s_cdc_log_attempted,UINT32_MAX);
    atomic_store(&s_cdc_log_enqueued,UINT32_MAX);
    atomic_store(&s_cdc_log_dropped,UINT32_MAX);
    atomic_store(&s_cdc_log_truncated,UINT32_MAX);
    atomic_store(&s_cdc_log_write_failed,UINT32_MAX);
    emit("%0256u",1U);emit("valid\n");queue_used=0;queue_fail=true;emit("full\n");
    cdc_response_t r=log_record("old\n");r.epoch=0;s_cdc_response_tx_incomplete=UINT32_MAX;
    assert(!cdc_response_send(&r));
    dongle_cdc_log_stats_t s=snapshot();
    assert(s.attempted==UINT32_MAX && s.enqueued==UINT32_MAX && s.dropped==UINT32_MAX &&
        s.truncated==UINT32_MAX && s.write_failed==UINT32_MAX && s_cdc_response_tx_incomplete==UINT32_MAX);
}
int main(int argc,char **argv) {
    assert(argc==2);
    if(strcmp(argv[1],"admission")==0)admission();
    else if(strcmp(argv[1],"formats")==0)formats();
    else if(strcmp(argv[1],"disabled")==0)disabled();
    else if(strcmp(argv[1],"writes")==0)writes();
    else if(strcmp(argv[1],"failures")==0)failures();
    else if(strcmp(argv[1],"partial")==0)partial();
    else if(strcmp(argv[1],"saturation")==0)saturation();
    else assert(0);
    printf("PASS CDC mirror: %s\n",argv[1]);return 0;
}
'''


@pytest.fixture(scope="module")
def cdc_transport(tmp_path_factory):
    build_dir = tmp_path_factory.mktemp("cdc_log_transport")
    stubs = build_dir / "stubs"
    stubs.mkdir()
    (stubs / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n", encoding="utf-8")
    source = (USB / "dongle_usb_device.c").read_text(encoding="utf-8")
    response_type = re.search(r"typedef struct \{\n    uint32_t epoch;\n    uint16_t length;\n    uint8_t itf;[\s\S]*?\} cdc_response_t;", source)
    assert response_type, "Production response type was not found"
    names = ("cdc_log_counter_increment", "dongle_usb_cdc_get_log_stats", "cdc_enqueue",
             "cdc_write_len", "cdc_response_send", "cdc_log_vprintf")
    helpers = "\n\n".join(function(source, name) for name in names)
    harness = build_dir / "cdc_log_transport.c"
    text_capacity = re.search(r"#define CDC_RESPONSE_TEXT_BYTES (\d+)u", source)
    assert text_capacity, "Production CDC response bound was not found"
    harness.write_text(PREFIX + f"#define CDC_RESPONSE_TEXT_BYTES {text_capacity.group(1)}u\n" +
                       response_type.group(0) + GLOBALS + helpers + SUFFIX, encoding="utf-8")
    return (*_build_host_executable([harness], build_dir, extra_includes=(stubs, USB / "include")), build_dir)


@pytest.mark.parametrize("scenario", ["admission", "formats", "disabled", "writes", "failures", "partial", "saturation"])
def test_actual_cdc_mirror_admission_and_bounded_framing(cdc_transport, scenario):
    executable, env, build_dir = cdc_transport
    assert "PASS CDC mirror" in _run([str(executable), scenario], cwd=build_dir, env=env).stdout
