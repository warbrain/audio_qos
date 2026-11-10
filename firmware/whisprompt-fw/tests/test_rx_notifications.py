"""Run the real RX worker/merge/producer with deterministic RTOS scheduling.

This checks the application's notification protocol under documented counting
notification semantics. It does not emulate FreeRTOS internals, SMP memory
ordering or ESP interrupt timing, and makes no hardware latency claim.
"""
import os
from pathlib import Path
import re
import subprocess
import sys

import pytest


ROOT = Path(os.environ.get("FIRMWARE_RX_SOURCE_ROOT", Path(__file__).resolve().parents[1]))
DEPS = Path(os.environ.get("FIRMWARE_RX_DEPENDENCY_ROOT", ROOT))
sys.path.insert(0, str(DEPS / "tests"))
from test_audio_recovery_042 import _build_host_executable
from test_dongle_transport import function


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "dual_end_link_config.h"
#include "dual_end_channel_guard.h"
#define ESP_NOW_ETH_ALEN 6
#define pdTRUE 1
typedef uint32_t TickType_t;
typedef int BaseType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)((ms)/TEST_TICK_MS))
#define ESP_LOGD(tag,...) ((void)(tag))
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
typedef struct {uint8_t channel;int rssi;} rx_ctrl_t;
typedef struct {const uint8_t *src_addr;rx_ctrl_t *rx_ctrl;} esp_now_recv_info_t;
'''


GLOBALS = r'''
typedef struct {dual_end_rx_packet_t packets[16];unsigned read,used;} test_queue_t;
typedef test_queue_t *QueueHandle_t;
typedef void *TaskHandle_t;
typedef void (*recv_cb_t)(const uint8_t *,const uint8_t *,int,int,void *);
static test_queue_t regular_queue,input_queue;
static QueueHandle_t s_rx_queue=&regular_queue,s_rx_input_queue=&input_queue;
static int worker_token,poll_token,recv_token,s_peer_state_lock;
static TaskHandle_t s_rx_task=&worker_token;
static uint32_t s_rx_invalid_len_count,s_rx_queue_drop_count;
static uint8_t s_current_channel=6,s_channel_desired=6;
static uint32_t s_channel_generation,s_channel_accept_generation,s_channel_confirm_generation;
static uint32_t s_channel_rescan_request_count;
static bool s_channel_peer_seen_after_switch,s_idle_hello_pending;
static int64_t s_channel_switched_us,s_rx_received_us,clock_us;
static const char *TAG="rx-worker-test";
static recv_cb_t s_control_response_cb;
static void *s_control_cb_ctx;
static jmp_buf finished;
enum {BEFORE_WAIT,DURING_WAIT,COALESCED,STALE,TIMEOUTS,WHILE_DRAINING};
static unsigned scenario,notifications,notify_gives,take_calls,immediate_returns;
static unsigned waited_notifications,timeout_returns,polls,recovery_advances,received,peer_seen;
static uint32_t largest_take;
static bool injected,scheduled;
static int64_t first_received_us=-1;
static uint8_t delivered[16];
static const int64_t recovery_deadline_us=3000;
static int64_t esp_timer_get_time(void){return clock_us;}
static bool radio_ready(void){return true;}
static bool is_peer_addr(const uint8_t *peer){return peer && peer[0]==42;}
static bool is_valid_framework_packet(const uint8_t *data,int len){
    return data && len>=(int)sizeof(dual_end_packet_header_t);
}
static void mark_peer_seen(int rssi){assert(rssi==-50);++peer_seen;}
static int dual_end_espnow_link_request_channel(uint8_t channel){(void)channel;return 0;}
static int dual_end_espnow_link_request_channel_rescan(bool avoid){(void)avoid;return 0;}

static BaseType_t xQueueSend(QueueHandle_t queue,const void *packet,TickType_t wait){
    assert((queue==s_rx_queue || queue==s_rx_input_queue) && wait==0);
    if(queue->used==16)return 0;
    queue->packets[(queue->read+queue->used)%16U]=*(const dual_end_rx_packet_t *)packet;
    ++queue->used;return pdTRUE;
}
static BaseType_t xQueuePeek(QueueHandle_t queue,void *packet,TickType_t wait){
    assert((queue==s_rx_queue || queue==s_rx_input_queue) && wait==0);
    if(!queue->used)return 0;
    *(dual_end_rx_packet_t *)packet=queue->packets[queue->read];return pdTRUE;
}
static BaseType_t xQueueReceive(QueueHandle_t queue,void *packet,TickType_t wait){
    if(xQueuePeek(queue,packet,wait)!=pdTRUE)return 0;
    queue->read=(queue->read+1U)%16U;--queue->used;return pdTRUE;
}
static void xTaskNotifyGive(TaskHandle_t task){
    assert(task==s_rx_task);++notifications;++notify_gives;
}
'''


SCHEDULER = r'''
static void arrive(uint8_t identifier){
    uint8_t bytes[DUAL_END_AUDIO_PACKET_BYTES]={0};
    dual_end_packet_header_t header={0};header.message_type=SCEPTER_MSG_AUDIO_FRAME;
    memcpy(bytes,&header,sizeof(header));bytes[sizeof(header)]=identifier;
    uint8_t peer[6]={42};rx_ctrl_t ctrl={6,-50};esp_now_recv_info_t info={peer,&ctrl};
    on_espnow_recv(&info,bytes,(int)sizeof(bytes));
    assert(!s_rx_invalid_len_count && !s_rx_queue_drop_count);
}
static void recovery_poll(void *ctx){
    assert(ctx==&poll_token && !regular_queue.used && !input_queue.used);
    assert(++polls<=10); /* A stale notification must not sustain a busy loop. */
    if(clock_us>=recovery_deadline_us && !recovery_advances)++recovery_advances;
    if(scenario==BEFORE_WAIT && !injected){
        /* The real merge has already returned empty; the real worker has not
         * called ulTaskNotifyTake yet. Admit through the real producer. */
        injected=true;arrive(1);
    }
}
static void receive_packet(const uint8_t *peer,const uint8_t *data,int length,int rssi,void *ctx){
    assert(is_peer_addr(peer) && length==DUAL_END_AUDIO_PACKET_BYTES && rssi==-50);
    assert(ctx==&recv_token && received<sizeof(delivered));
    delivered[received++]=data[sizeof(dual_end_packet_header_t)];
    assert(s_rx_received_us<=clock_us);
    if(first_received_us<0)first_received_us=clock_us;
    if(scenario==WHILE_DRAINING && !injected){
        injected=true;arrive(2);arrive(3);
    }
}
static recv_cb_t s_recv_cb=receive_packet;
static void *s_recv_ctx=&recv_token;
static void (*s_poll_cb)(void *)=recovery_poll;
static void *s_poll_ctx=&poll_token;

static uint32_t take_notification(BaseType_t clear_on_exit,bool waited){
    const uint32_t result=notifications;assert(result>0);
    if(clear_on_exit)notifications=0;else --notifications;
    if(result>largest_take)largest_take=result;
    if(waited)++waited_notifications;else ++immediate_returns;
    return result;
}
static uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit,TickType_t wait){
    assert(++take_calls<=12);
    const TickType_t poll_ticks=pdMS_TO_TICKS(2);
    assert(wait==(poll_ticks?poll_ticks:1U) && wait>0);
    /* FreeRTOS counting notifications remain pending across this check-to-wait
     * gap. A notification already present cannot consume a timeout. */
    if(notifications)return take_notification(clear_on_exit,false);
    const int64_t deadline=clock_us+(int64_t)wait*TEST_TICK_MS*1000;
    if(scenario==DURING_WAIT && !scheduled){
        scheduled=true;
        const int64_t arrival_us=clock_us+1000;
        assert(arrival_us<deadline);clock_us=arrival_us;arrive(1);
        return take_notification(clear_on_exit,true);
    }
    clock_us=deadline;++timeout_returns;
    if(timeout_returns==3)longjmp(finished,1);
    return 0;
}
'''


SUFFIX = r'''
int main(int argc,char **argv){
    assert(argc==2);
    if(strcmp(argv[1],"before_wait")==0)scenario=BEFORE_WAIT;
    else if(strcmp(argv[1],"during_wait")==0)scenario=DURING_WAIT;
    else if(strcmp(argv[1],"coalesced")==0)scenario=COALESCED;
    else if(strcmp(argv[1],"stale")==0)scenario=STALE;
    else if(strcmp(argv[1],"timeouts")==0)scenario=TIMEOUTS;
    else if(strcmp(argv[1],"while_draining")==0)scenario=WHILE_DRAINING;
    else assert(0);
    if(scenario==COALESCED){arrive(1);arrive(2);arrive(3);notifications+=61;}
    if(scenario==STALE)notifications=64;
    if(scenario==WHILE_DRAINING)arrive(1);
    if(!setjmp(finished))rx_worker_task(NULL);
    assert(!regular_queue.used && !input_queue.used && timeout_returns==3);
    assert(recovery_advances==1 && polls>=3 && polls<=4 && !notifications);
    assert(immediate_returns<=1 && peer_seen==received);
    const unsigned expected=scenario==COALESCED || scenario==WHILE_DRAINING?3U:
        scenario==BEFORE_WAIT || scenario==DURING_WAIT?1U:0U;
    assert(received==expected && notify_gives==expected);
    for(unsigned i=0;i<expected;++i)assert(delivered[i]==i+1U);
    if(scenario==BEFORE_WAIT){assert(first_received_us==0 && immediate_returns==1 && !waited_notifications);}
    if(scenario==DURING_WAIT){assert(first_received_us==1000 && waited_notifications==1 && !immediate_returns);}
    if(scenario==COALESCED || scenario==STALE){assert(largest_take==64 && immediate_returns==1);}
    if(scenario==WHILE_DRAINING){assert(largest_take==3 && immediate_returns==1 && first_received_us==0);}
    if(scenario==TIMEOUTS){assert(take_calls==3 && immediate_returns==0 && received==0);}
    printf("PASS real RX loop: %s tick_ms=%u receives=%u polls=%u timeouts=%u\n",
        argv[1],(unsigned)TEST_TICK_MS,received,polls,timeout_returns);
    return 0;
}
'''


@pytest.fixture(scope="module", params=[1, 10])
def rx_worker(tmp_path_factory, request):
    tick_ms = request.param
    build_dir = tmp_path_factory.mktemp(f"rx_worker_tick_{tick_ms}")
    source = (ROOT / "dongle/components/dual_end_espnow_link/dual_end_espnow_link.c").read_text(encoding="utf-8")
    packet_type = re.search(r"typedef struct \{\n    uint8_t src_addr\[ESP_NOW_ETH_ALEN\];[\s\S]*?\} dual_end_rx_packet_t;", source)
    assert packet_type, "Production RX packet type was not found"
    harness = build_dir / "rx_worker_notifications.c"
    harness.write_text(
        f"#define TEST_TICK_MS {tick_ms}\n" + PREFIX + packet_type.group(0) + GLOBALS +
        function(source, "on_espnow_recv") + SCHEDULER +
        function(source, "rx_take_next_packet") + function(source, "rx_worker_task") + SUFFIX,
        encoding="utf-8",
    )
    return (*_build_host_executable([harness], build_dir,
        extra_includes=(DEPS / "dongle/components/dual_end_espnow_link/include",)), build_dir)


@pytest.mark.parametrize("scenario", ["before_wait", "during_wait", "coalesced", "stale", "timeouts", "while_draining"])
def test_real_rx_worker_has_no_check_wait_loss_or_persistent_spin(rx_worker, scenario):
    executable, env, build_dir = rx_worker
    result = subprocess.run([str(executable), scenario], cwd=build_dir, env=env,
                            text=True, capture_output=True, check=True, timeout=10)
    assert "PASS real RX loop" in result.stdout
