"""Exercise real Dongle transport functions without ESP hardware.

Optional source/dependency roots let a project-local draft be checked before
applying it to the firmware checkout. All generated C and binaries use tmp_path.
"""
import os
from pathlib import Path
import sys
import pytest


ROOT = Path(os.environ.get("FIRMWARE_TRANSPORT_SOURCE_ROOT", Path(__file__).resolve().parents[1]))
DEPS = Path(os.environ.get("FIRMWARE_TRANSPORT_DEPENDENCY_ROOT", ROOT))
sys.path.insert(0, str(DEPS / "dongle/components/dongle_usb_device/tests"))
from host_build import _build_host_executable, _run


def function(source, name):
    cursor = 0
    while True:
        start = source.index(name + "(", cursor)
        opening = source.index("{", start)
        prototype_end = source.find(";", start, opening)
        if prototype_end < 0:
            break
        cursor = prototype_end + 1
    start = source.rfind("\n", 0, start) + 1
    end, depth = opening + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def compile_run(code, tmp_path, includes=(), defines=()):
    source = tmp_path / "transport_test.c"
    source.write_text(code, encoding="utf-8")
    executable, env = _build_host_executable(
        [source], tmp_path, extra_includes=includes, extra_defines=defines
    )
    return _run([str(executable)], cwd=tmp_path, env=env).stdout


def test_plc_burst_fades_once_and_session_reset_clears_history(tmp_path):
    receiver = (ROOT / "dongle/components/dual_end_audio/dual_end_audio.c").read_text(encoding="utf-8")
    code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define DUAL_END_AUDIO_CHANNELS 2U
#define DUAL_END_AUDIO_FRAME_SAMPLES 320U
#define DUAL_END_AUDIO_FRAME_SAMPLE_VALUES 640U
static int16_t s_last_frame_pcm[640], s_decoded_interleaved_pcm[640];
static int16_t s_decode_channel_pcm[2][320];
static void *s_g722_decoder[2];
static bool s_last_frame_valid;
static uint32_t s_concealed_repeat_frames, closed;
static int esp_g722_dec_close(void *p) { assert(p); ++closed; return 0; }
''' + function(receiver, "reset_g722_decoder") + "\n" + function(receiver, "conceal_missing_frame") + r'''
static void real_frame(void) {
    for (unsigned i=0;i<320;++i) {
        s_last_frame_pcm[2*i]=INT16_MAX;
        s_last_frame_pcm[2*i+1]=INT16_MIN;
    }
    s_last_frame_valid=true;
}
static void silence(void) { for(unsigned i=0;i<640;++i) assert(s_decoded_interleaved_pcm[i]==0); }
int main(void) {
    const unsigned bursts[]={1,2,5,10};
    conceal_missing_frame(); silence(); assert(s_concealed_repeat_frames==0);
    for(unsigned b=0;b<4;++b) {
        real_frame(); uint32_t before=s_concealed_repeat_frames;
        for(unsigned lost=0;lost<bursts[b];++lost) {
            conceal_missing_frame();
            if(lost==0) {
                assert(s_decoded_interleaved_pcm[0]==INT16_MAX);
                assert(s_decoded_interleaved_pcm[1]==INT16_MIN);
                assert(s_decoded_interleaved_pcm[638]==0&&s_decoded_interleaved_pcm[639]==0);
                for(unsigned i=1;i<320;++i) {
                    assert(s_decoded_interleaved_pcm[2*i]<=s_decoded_interleaved_pcm[2*i-2]);
                    assert(s_decoded_interleaved_pcm[2*i+1]>=s_decoded_interleaved_pcm[2*i-1]);
                }
            } else silence();
            assert(!s_last_frame_valid&&s_concealed_repeat_frames==before+1);
        }
    }
    real_frame(); s_g722_decoder[0]=&closed; s_g722_decoder[1]=&closed;
    memset(s_decode_channel_pcm,1,sizeof(s_decode_channel_pcm));
    reset_g722_decoder(); assert(closed==2&&!s_last_frame_valid);
    assert(!s_g722_decoder[0]&&!s_g722_decoder[1]);
    for(unsigned i=0;i<640;++i) assert(s_last_frame_pcm[i]==0);
    conceal_missing_frame(); silence();
    puts("PLC burst/session reset PASS"); return 0;
}
'''
    assert "PASS" in compile_run(code, tmp_path)


def test_rx_cache_one_hz_state_changes_and_low_frequency_stack_scan(tmp_path):
    receiver = (ROOT / "dongle/components/dual_end_audio/dual_end_audio.c").read_text(encoding="utf-8")
    (tmp_path / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n", encoding="utf-8")
    code = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#ifdef _MSC_VER
#pragma pack(push,1)
#define __attribute__(x)
#include "dual_end_audio_recovery_protocol.h"
#undef __attribute__
#pragma pack(pop)
#endif
#include "audio_rx_telemetry.h"
static audio_recovery_rx_t s_rx_recovery;
static int64_t s_last_rx_snapshot_us,s_last_rx_stack_scan_us,s_hid_wait_peak_us;
static uint32_t s_rx_stack_min_bytes,s_rx_session_generation;
static uint32_t s_delivered_frames,s_timed_sink_drop_frames,s_masked_frames,s_concealed_repeat_frames;
static uint32_t s_crc_error_frames,s_decode_error_frames,s_invalid_frames,s_feedback_queue_failures;
static uint32_t s_nack_queued,s_recovered_decoded_frames,s_hid_dispatches,s_hid_wait_max_us;
static uint32_t s_hid_wait_peak_sequence,s_decode_frame_max_us,s_poll_max_us,s_poll_audio_max;
static uint8_t s_rx_codec=4;
static bool s_rx_snapshot_ready, publish_success=true;
static unsigned copies,scans;
static audio_rx_telemetry_snapshot_t latest;
bool audio_rx_telemetry_enabled(void) { return true; }
uint32_t audio_rx_telemetry_stack_min(void) { ++scans; return 1234; }
uint32_t dongle_uac_bridge_session_generation(void) { return 4; }
bool audio_rx_telemetry_publish(const audio_rx_telemetry_snapshot_t *p) { latest=*p; ++copies; return publish_success; }
int64_t audio_recovery_rx_playout_us(const audio_recovery_rx_t *rx,uint32_t seq) {(void)rx;(void)seq;return 0;}
''' + function(receiver, "publish_rx_telemetry") + r'''
int main(void) {
    s_rx_session_generation=2;s_rx_recovery.active=true;
    for(int64_t t=0;t<=10000000;t+=10000) publish_rx_telemetry(t);
    assert(copies==11&&scans==2&&latest.sampled_us==10000000&&latest.rx_stack_min_bytes==1234);
    assert(latest.session_generation==2&&latest.uac_session_generation==4&&latest.codec==4);
    s_rx_recovery.active=false;publish_rx_telemetry(10000100);assert(copies==12&&!latest.active);
    ++s_rx_session_generation;publish_rx_telemetry(10000200);assert(copies==13);
    s_rx_codec=5;publish_rx_telemetry(10000300);assert(copies==14&&latest.codec==5&&scans==2);
    publish_success=false;publish_rx_telemetry(11000300);assert(copies==15);
    for(int64_t t=11000301;t<12000300;t+=10000) publish_rx_telemetry(t);
    assert(copies==15);publish_rx_telemetry(12000300);assert(copies==16&&scans==2);
    s_rx_snapshot_ready=false;publish_rx_telemetry(12000301);assert(copies==17&&scans==3);
    puts("RX 1 Hz/cache lifecycle/stack cadence PASS");return 0;
}
'''
    includes = (
        tmp_path,
        DEPS / "dongle/components/dual_end_audio/include",
        DEPS / "components/dual_end_audio_recovery/include",
        DEPS / "dongle/components/dual_end_common/include",
    )
    assert "PASS" in compile_run(code, tmp_path, includes)


def test_rx_text_throttles_without_delaying_lifecycle(tmp_path):
    worker = (ROOT / "dongle/components/dual_end_audio/audio_rx_telemetry.c").read_text(encoding="utf-8")
    code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef struct {int64_t sampled_us;uint32_t session_generation;uint8_t codec;bool active;} audio_rx_telemetry_snapshot_t;
typedef struct {int64_t sampled_us;uint32_t session_generation;uint8_t codec;bool active,valid;} telemetry_log_state_t;
''' + function(worker, "telemetry_log_due") + r'''
int main(void) {
    telemetry_log_state_t state={0};
    audio_rx_telemetry_snapshot_t snap={.sampled_us=100,.session_generation=1,.codec=4,.active=true};
    assert(telemetry_log_due(&state,&snap));
    for(unsigned i=1;i<10;++i) {snap.sampled_us=100+(int64_t)i*1000000;assert(!telemetry_log_due(&state,&snap));}
    snap.sampled_us=10000100;assert(telemetry_log_due(&state,&snap));
    ++snap.session_generation;assert(telemetry_log_due(&state,&snap));
    snap.active=false;assert(telemetry_log_due(&state,&snap));
    snap.codec=5;assert(telemetry_log_due(&state,&snap));
    assert(!telemetry_log_due(&state,&snap));
    puts("RX text 10 s/lifecycle PASS");return 0;
}
'''
    assert "PASS" in compile_run(code, tmp_path)


def test_actual_timeline_fast_copy_and_drift(tmp_path):
    tests = DEPS / "dongle/components/dongle_usb_device/tests"
    code = (ROOT / "dongle/components/dongle_usb_device/tests/transport_timeline_test.c").read_text(encoding="utf-8")
    bridge = ROOT / "dongle/components/dongle_uac_bridge/dongle_uac_bridge.c"
    code = code.replace('"../../dongle_uac_bridge/dongle_uac_bridge.c"', '"' + bridge.as_posix() + '"')
    includes = (
        tests / "bridge_stubs", tests / "uac_stubs",
        DEPS / "dongle/components/dongle_uac_bridge/include",
        DEPS / "components/dual_end_audio_recovery/include",
        DEPS / "dongle/components/dual_end_common/include",
    )
    assert "PASS" in compile_run(code, tmp_path, includes, ("DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING=0",))


@pytest.mark.parametrize("harness,defines,message", (
    ("bridge_test.c", (), "25 scenarios"),
    ("bridge_timeline_only_test.c", ("DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING=0",), "legacy ring=0"),
))
def test_existing_bridge_regressions_use_current_implementation(tmp_path, harness, defines, message):
    tests = DEPS / "dongle/components/dongle_usb_device/tests"
    code = (tests / harness).read_text(encoding="utf-8")
    bridge = ROOT / "dongle/components/dongle_uac_bridge/dongle_uac_bridge.c"
    code = code.replace('"../../dongle_uac_bridge/dongle_uac_bridge.c"', '"' + bridge.as_posix() + '"')
    includes = (
        tests / "bridge_stubs", tests / "uac_stubs",
        DEPS / "dongle/components/dongle_uac_bridge/include",
        DEPS / "components/dual_end_audio_recovery/include",
        DEPS / "dongle/components/dual_end_common/include",
    )
    assert message in compile_run(code, tmp_path, includes, defines)


def test_rx_notifies_only_admitted_audio_and_control_packets(tmp_path):
    link = (ROOT / "dongle/components/dual_end_espnow_link/dual_end_espnow_link.c").read_text(encoding="utf-8")
    idle = function(link, "rx_worker_task")
    assert "ulTaskNotifyTake(pdTRUE, poll_ticks ? poll_ticks : 1)" in idle
    assert "pdMS_TO_TICKS(2)" in idle
    assert "vTaskDelay(" not in idle
    code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define ESP_NOW_ETH_ALEN 6
#define DUAL_END_ESPNOW_MAX_DATA_BYTES 400
#define DUAL_END_AUDIO_PACKET_BYTES 400
#define SCEPTER_MSG_AUDIO_FRAME 16
#define pdTRUE 1
#undef NULL
#define NULL 0
typedef int BaseType_t;
typedef struct {uint8_t message_type;} dual_end_packet_header_t;
typedef struct {uint8_t channel;int rssi;} rx_ctrl_t;
typedef struct {const uint8_t *src_addr;rx_ctrl_t *rx_ctrl;} esp_now_recv_info_t;
typedef struct {uint16_t len;int8_t rssi;int64_t received_us;uint8_t src_addr[6],data[400];} dual_end_rx_packet_t;
static int s_rx_queue=1,s_rx_input_queue=2,s_rx_task=3,s_current_channel=6;
static unsigned s_rx_invalid_len_count,s_rx_queue_drop_count,notifications,sends;
static bool ready=true,full=false,replaceable=true,replacement_fails=false;
static bool radio_ready(void) {return ready;}
static bool is_peer_addr(const uint8_t *addr) {return addr[0]==42;}
static int64_t esp_timer_get_time(void) {return 12345;}
static void xTaskNotifyGive(int task) {assert(task==3);++notifications;}
static int xQueueSend(int queue,const void *p,unsigned wait) {
    const dual_end_rx_packet_t *packet=(const dual_end_rx_packet_t *)p;
    assert((queue==1||queue==2)&&wait==0&&packet->received_us==12345);
    ++sends;return !full&&!replacement_fails;
}
static int xQueueReceive(int queue,void *p,unsigned wait) {
    (void)p;assert(queue==1&&wait==0);if(!replaceable)return 0;full=false;return 1;
}
''' + function(link, "on_espnow_recv") + r'''
int main(void) {
    uint8_t peer[6]={42},data[400]={SCEPTER_MSG_AUDIO_FRAME};
    rx_ctrl_t ctrl={6,-50};esp_now_recv_info_t info={peer,&ctrl};
    on_espnow_recv(&info,data,400);assert(notifications==1&&sends==1);
    full=true;on_espnow_recv(&info,data,400);assert(notifications==1&&s_rx_queue_drop_count==1);
    full=false;data[0]=17;on_espnow_recv(&info,data,24);assert(notifications==2);
    full=true;on_espnow_recv(&info,data,24);assert(notifications==3&&s_rx_queue_drop_count==2);
    full=true;replaceable=false;on_espnow_recv(&info,data,24);assert(notifications==3&&s_rx_queue_drop_count==3);
    replaceable=true;replacement_fails=true;on_espnow_recv(&info,data,24);assert(notifications==3&&s_rx_queue_drop_count==4);
    data[0]=SCEPTER_MSG_AUDIO_FRAME;on_espnow_recv(&info,data,399);assert(s_rx_invalid_len_count==1&&notifications==3);
    ready=false;unsigned before=sends;on_espnow_recv(&info,data,400);assert(sends==before);
    ready=true;replacement_fails=false;full=false;s_rx_task=0;on_espnow_recv(&info,data,400);assert(notifications==3);
    puts("RX admission notifications/drop/replacement/startup PASS");return 0;
}
'''
    assert "PASS" in compile_run(code, tmp_path)
