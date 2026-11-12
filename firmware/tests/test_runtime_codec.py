"""Host behavior tests for real packet validators and extracted transaction gates.

These tests never call ESP codec DSP or a device. ESP APIs are stubs only for
failure injection. Real Opus audio behavior requires two-end hardware testing.
"""
from pathlib import Path
from test_audio_recovery_042 import _build_host_executable, _run

ROOT = Path(__file__).resolve().parents[1]
CODEC = ROOT / "components/dual_end_audio_codec"


def function(source: str, name: str) -> str:
    cursor = 0
    while True:
        start = source.index(name + "(", cursor)
        opening = source.index("{", start)
        prototype_end = source.find(";", start, opening)
        if prototype_end < 0:
            break
        cursor = prototype_end + 1
    start = source.rfind("\n", 0, start) + 1
    depth = 1
    end = opening + 1
    while depth:
        if source[end] == "{": depth += 1
        elif source[end] == "}": depth -= 1
        end += 1
    return source[start:end]


def test_runtime_codec_contract(tmp_path: Path):
    sender = (ROOT / "scepter/components/dual_end_audio/dual_end_audio.c").read_text(encoding="utf-8")
    receiver = (ROOT / "dongle/components/dual_end_audio/dual_end_audio.c").read_text(encoding="utf-8")
    harness = tmp_path / "codec_transaction.c"
    harness.write_text(r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "dual_end_codec_frame.h"
#include "audio_recovery_rx.h"
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG -2
#define portMAX_DELAY 0
#define xSemaphoreTake(a,b) ((void)(a), (void)(b))
#define xSemaphoreGive(a) ((void)(a))
#define ESP_LOGI(...) ((void)0)
#define PRIx32 "x"
static int s_sender_mutex = 1;
static uint8_t s_tx_codec = DUAL_END_AUDIO_CODEC_G722;
static uint32_t s_session_id = 77;
static uint32_t s_encode_max_us;
static unsigned prepare_calls, session_changes;
static unsigned fail_prepare_call;
static esp_err_t ensure_g722_encoder(void) {
    ++prepare_calls; return prepare_calls == fail_prepare_call ? ESP_FAIL : ESP_OK;
}
static esp_err_t dual_end_opus_encoder_prepare_for(uint8_t codec) { assert(dual_end_audio_codec_is_opus(codec)); return ensure_g722_encoder(); }
static void sender_new_session_locked(void) { ++session_changes; ++s_session_id; }
static uint8_t s_rx_codec = UINT8_MAX;
static audio_recovery_rx_t s_rx_recovery;
static esp_err_t ensure_g722_decoder(void) { return ensure_g722_encoder(); }
static esp_err_t dual_end_opus_decoder_prepare(void) { return ensure_g722_encoder(); }
''' + function(sender, "dual_end_audio_sender_set_codec") + "\n" +
        function(receiver, "receiver_bind_codec") + r'''
int main(void) {
    uint8_t payload[320] = {0};
    assert(DUAL_END_AUDIO_CODEC_CAP_G722 == 1 && DUAL_END_AUDIO_CODEC_CAP_OPUS == 2);
    assert(DUAL_END_AUDIO_CODEC_CAP_OPUS_62 == 4 && DUAL_END_AUDIO_CODEC_CAPABILITIES == 7);
    assert(dual_end_codec_payload_valid(4, 320, payload, 320));
    assert(!dual_end_codec_payload_valid(4, 164, payload, 320));
    payload[0] = payload[2] = 80;
    memset(payload + 4, 0xA7, 160);
    assert(dual_end_codec_payload_valid(3, 164, payload, 320));
    assert(!dual_end_codec_payload_valid(3, 320, payload, 320));
    assert(!dual_end_codec_payload_valid(3, 164, payload, 319));
    assert(!dual_end_codec_payload_valid(8, 164, payload, 320));
    payload[1] = 1; assert(!dual_end_codec_payload_valid(3, 164, payload, 320)); payload[1] = 0;
    payload[2] = 79; assert(!dual_end_codec_payload_valid(3, 164, payload, 320)); payload[2] = 80;
    payload[319] = 1; assert(!dual_end_codec_payload_valid(3, 164, payload, 320)); payload[319] = 0;
    assert(!dual_end_codec_payload_valid(3, 164, NULL, 320));
    for (uint8_t i = 0; i <= 16; ++i) {
        assert(dual_end_dsp_version_item_valid(1, i) == (i <= 14));
        assert(dual_end_dsp_version_item_valid(2, i) == (i == 15 || i == 16));
        assert(!dual_end_dsp_version_item_valid(3, i));
    }
    assert(dual_end_audio_sender_set_codec(0) == ESP_ERR_INVALID_ARG);
    assert(prepare_calls == 0 && session_changes == 0 && s_tx_codec == 4);
    assert(dual_end_audio_sender_set_codec(4) == ESP_OK && prepare_calls == 0);
    fail_prepare_call = 1;
    assert(dual_end_audio_sender_set_codec(3) == ESP_FAIL);
    assert(s_tx_codec == 4 && session_changes == 0);
    prepare_calls = 0; fail_prepare_call = 0;
    assert(dual_end_audio_sender_set_codec(3) == ESP_OK);
    assert(s_tx_codec == 3 && session_changes == 1 && prepare_calls == 2);
    prepare_calls = 0; fail_prepare_call = 2;
    assert(dual_end_audio_sender_set_codec(4) == ESP_FAIL);
    assert(s_tx_codec == 3 && session_changes == 3);
    s_sender_mutex = 0;
    assert(dual_end_audio_sender_set_codec(4) == ESP_ERR_INVALID_ARG);

    prepare_calls = 0; fail_prepare_call = 0;
    audio_recovery_rx_init(&s_rx_recovery);
    assert(!receiver_bind_codec(3, 10) && prepare_calls == 0);
    assert(audio_recovery_rx_begin(&s_rx_recovery, 10, 100, 1000) == AUDIO_RX_SESSION_NEW);
    assert(!receiver_bind_codec(3, 9) && prepare_calls == 0);
    fail_prepare_call = 1;
    assert(!receiver_bind_codec(3, 10) && s_rx_codec == UINT8_MAX);
    fail_prepare_call = 0;
    assert(receiver_bind_codec(3, 10) && s_rx_codec == 3);
    assert(!receiver_bind_codec(4, 10));
    assert(receiver_bind_codec(3, 10) && prepare_calls == 2);
    assert(audio_recovery_rx_push(&s_rx_recovery, 10, 100, payload, 0, false, 2000) == AUDIO_RX_ACCEPTED);
    assert(audio_recovery_rx_begin(&s_rx_recovery, 11, 200, 3000) == AUDIO_RX_SESSION_NEW);
    s_rx_codec = UINT8_MAX;
    assert(!receiver_bind_codec(3, 10));
    assert(audio_recovery_rx_push(&s_rx_recovery, 10, 100, payload, 0, true, 4000) == AUDIO_RX_REJECTED);
    assert(receiver_bind_codec(4, 11));
    assert(audio_recovery_rx_begin(&s_rx_recovery, 10, 100, 5000) == AUDIO_RX_SESSION_REJECTED);
    puts("codec frame / version gates / apply rollback / session isolation PASS (ESP DSP stubbed)");
    return 0;
}
''', encoding="utf-8")
    executable, env = _build_host_executable(
        [harness, ROOT / "components/dual_end_audio_recovery/audio_recovery_rx.c"], tmp_path,
        extra_includes=(CODEC / "include",))
    result = _run([str(executable)], cwd=tmp_path, env=env)
    assert "session isolation PASS (ESP DSP stubbed)" in result.stdout


def test_same_wire_constants_on_both_targets():
    relative = "components/dual_end_common/include/dual_end_link_config.h"
    scepter = (ROOT / "scepter" / relative).read_text(encoding="utf-8")
    dongle = (ROOT / "dongle" / relative).read_text(encoding="utf-8")
    assert scepter == dongle


def test_spatial_stereo_request_owner_and_capability_intersection(tmp_path: Path):
    """Run actual HID forward/reply and bounded control/owner handoff gates.

    No radio, RTOS scheduling or audio device is involved. The real processing
    kernel supplies the applied mode/layout; stubs model the async control
    notification and inject layout-prepare failure or timeout.
    """
    mic = (ROOT / 'scepter/main/mic/mic_audio.c').read_text(encoding='utf-8')
    dongle = (ROOT / 'dongle/main/app_main.c').read_text(encoding='utf-8')
    harness = tmp_path / 'spatial_transaction.c'
    harness.write_text(r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "dual_end_spatial.h"
#include "dual_end_codec_frame.h"
#include "dual_end_link_config.h"
typedef int esp_err_t;
typedef void *TaskHandle_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG -2
#define ESP_ERR_TIMEOUT -3
#define ESP_ERR_INVALID_STATE -4
#define ESP_ERR_NO_MEM -5
static const char *TAG="fixture";
static const char *esp_err_to_name(esp_err_t error) {(void)error;return "fixture";}
static void fixture_log(const char *tag,const char *format,...) {(void)tag;(void)format;}
#define ESP_LOGW(...) fixture_log(__VA_ARGS__)
#define portENTER_CRITICAL(lock) do { (void)(lock); ++locked; } while(0)
#define portEXIT_CRITICAL(lock) do { (void)(lock); assert(locked); --locked; } while(0)
#define pdMS_TO_TICKS(v) (v)
static int locked,s_config_lock,s_dsp_lock;
static dual_end_spatial_t spatial_storage;
static dual_end_spatial_t *s_spatial = &spatial_storage;
static dual_end_spatial_stats_t s_spatial_status;
static uint16_t s_spatial_requested_mode,s_spatial_generation,s_spatial_commit_mode;
static uint32_t s_spatial_request_generation,s_spatial_applied_request,
    s_spatial_transition_generation,s_spatial_commit_generation;
static esp_err_t s_spatial_apply_result=ESP_OK;
static bool s_spatial_transition_active,s_spatial_commit_pending,s_spatial_dispatch_paused;
static void *s_mic_task_handle=(void*)1,*s_config_task_handle=(void*)2;
static uint8_t layout,s_dsp_reply[16],usb_reply[16],fixture_peer[6]={1,2,3,4,5,6};
static uint32_t s_control_sequence;
static unsigned notices,layout_prepares,queued,invalidated,notifying;
static bool postpone;
static esp_err_t layout_result=ESP_OK;
static dual_end_control_packet_t forwarded;
static bool mic_spatial_apply_control(void);
static void mic_owner_apply_spatial_commit(void);
static void mic_config_transition_notify(void);
static void mic_spatial_reset_state_owner(uint16_t mode) {
    assert(dual_end_spatial_reset(s_spatial,mode,++s_spatial_generation));
    s_spatial_status=s_spatial->stats;
    ++invalidated;
}
static void mic_frame_dispatch_control_lock(void) { ++locked; }
static void mic_frame_dispatch_control_unlock(void) { assert(locked); --locked; }
static void mic_frame_dispatch_invalidate(void) { ++invalidated; }
static void mic_frame_dispatch_reset_stale_queue(void) { ++invalidated; }
static void xTaskNotifyGive(void *task) {
    assert(task); ++notices;
    if (!postpone && !notifying) {
        ++notifying;
        (void)mic_spatial_apply_control();
        mic_owner_apply_spatial_commit();
        --notifying;
    }
}
static void vTaskDelay(unsigned ticks) { assert(ticks==5); }
static esp_err_t dual_end_audio_sender_set_processing_layout(uint8_t value) {
    ++layout_prepares;assert(value<=1);if(layout_result==ESP_OK)layout=value;return layout_result;
}
static esp_err_t dual_end_espnow_link_get_peer(uint8_t *out,void *unused) {
    assert(!unused);memcpy(out,fixture_peer,6);return ESP_OK;
}
static void dongle_usb_device_set_config_reply(const uint8_t *data) { memcpy(usb_reply,data,16); }
static uint16_t dual_end_audio_receiver_codec_capabilities(void) { return 3; }
static bool dual_end_espnow_link_is_connected(void) { return true; }
static esp_err_t dual_end_espnow_link_queue_control(const dual_end_control_packet_t *p,int bytes) {
    assert(bytes==DUAL_END_CONTROL_PACKET_WIRE_BYTES(13));forwarded=*p;++queued;return ESP_OK;
}
''' + function(mic, 'mic_config_transition_notify') + '\n' +
        function(mic, 'mic_spatial_apply_control') + '\n' +
        function(mic, 'mic_owner_apply_spatial_commit') + '\n' +
        function(mic, 'mic_spatial_parameter') + '\n' +
        function(dongle, 'dongle_dsp_response') + '\n' +
        function(dongle, 'dongle_dsp_request') + r'''
static void request(uint8_t op,uint8_t item,uint16_t value) {
    uint8_t data[16]={0x11,3,op,item,1,0,0,0,(uint8_t)value,(uint8_t)(value>>8)};
    assert(dongle_dsp_request(data)==ESP_OK);
    assert(forwarded.data[0]==3&&forwarded.data[1]==op&&forwarded.data[2]==item);
}
int main(void) {
    uint16_t actual=999;mic_spatial_reset_state_owner(0);s_spatial_status=s_spatial->stats;
    assert(mic_spatial_parameter(false,17,0,&actual)==ESP_OK&&actual==63);
    const uint16_t sequence[]={0,1,2,3,2,3,4,5,6,0};
    for(unsigned i=0;i<sizeof(sequence)/sizeof(sequence[0]);++i) {
        uint16_t mode=sequence[i],generation=s_spatial_generation;
        assert(mic_spatial_parameter(true,18,mode,&actual)==ESP_OK&&actual==mode);
        assert(s_spatial_generation==(uint16_t)(generation+1));
        assert(layout==dual_end_spatial_mode_layout(mode));
        assert(mic_spatial_parameter(false,19,0,&actual)==ESP_OK&&actual==mode);
        assert(mic_spatial_parameter(false,20,0,&actual)==ESP_OK&&actual==layout);
        assert(mic_spatial_parameter(false,22,0,&actual)==ESP_OK&&actual==s_spatial_generation);
        request(2,18,mode);
    }
    unsigned before=notices,prepares=layout_prepares;uint16_t generation=s_spatial_generation;
    assert(mic_spatial_parameter(true,18,7,&actual)==ESP_ERR_INVALID_ARG);
    assert(mic_spatial_parameter(true,19,3,&actual)==ESP_ERR_INVALID_ARG);
    assert(notices==before&&layout_prepares==prepares&&s_spatial_generation==generation);
    uint8_t invalid[16]={0x11,3,2,18,1,0,0,0,7,0};before=queued;
    assert(dongle_dsp_request(invalid)==ESP_ERR_INVALID_ARG&&queued==before);
    layout_result=ESP_FAIL;
    assert(mic_spatial_parameter(true,18,3,&actual)==ESP_FAIL);
    assert(s_spatial->stats.mode==0&&layout==0&&s_spatial_generation==generation);
    layout_result=ESP_OK;
    assert(mic_spatial_parameter(true,18,3,&actual)==ESP_OK&&
           s_spatial->stats.layout==dual_end_spatial_mode_layout(3));
    generation=s_spatial_generation;postpone=true;
    assert(mic_spatial_parameter(true,18,2,&actual)==ESP_ERR_TIMEOUT);
    assert(s_spatial->stats.mode==3&&s_spatial_generation==generation&&layout==0);
    postpone=false;(void)mic_spatial_apply_control();mic_owner_apply_spatial_commit();
    assert(s_spatial->stats.mode==2&&layout==1&&!s_spatial_transition_active);
    /* Actual reply intersection gates every spatial mode on both device caps. */
    for(uint16_t caps=0;caps<64;++caps) {
        request(1,17,0);dual_end_control_packet_t response=forwarded;
        response.packet.source_role=SCEPTER_ROLE_SCEPTER;
        response.packet.message_type=SCEPTER_MSG_CONTROL_RESPONSE;
        response.data[7]=(uint8_t)caps;response.data[8]=0;response.data[9]=0;
        dongle_dsp_response(fixture_peer,(const uint8_t*)&response,DUAL_END_CONTROL_PACKET_WIRE_BYTES(13),0,NULL);
        uint16_t reported=dual_end_codec_u16(usb_reply+8);
        assert(reported==(caps&63U));
        assert(dual_end_spatial_mode_supported(3,reported)==((caps&4U)!=0));
        assert(dual_end_spatial_mode_supported(4,reported)==((caps&9U)==9U));
        assert(dual_end_spatial_mode_supported(5,reported)==((caps&16U)!=0));
        assert(dual_end_spatial_mode_supported(6,reported)==((caps&32U)!=0));
        assert(usb_reply[10]==0);
    }
    assert(!locked);puts("spatial HID modes 0..6 / owner apply-reset-rollback / capability intersection PASS");
    return 0;
}
''', encoding='utf-8')
    executable, env = _build_host_executable([harness, CODEC / 'dual_end_spatial.c', CODEC / 'dual_end_wiener.c', CODEC / 'dual_end_mvdr.c'], tmp_path,
        extra_includes=(CODEC / 'include',))
    assert 'PASS' in _run([str(executable)], cwd=tmp_path, env=env).stdout


def test_audio_codec_dispatch_preserves_dsp_and_recovery():
    tx = (ROOT / "scepter/components/dual_end_audio/dual_end_audio.c").read_text(encoding="utf-8")
    rx = (ROOT / "dongle/components/dual_end_audio/dual_end_audio.c").read_text(encoding="utf-8")
    mic = (ROOT / "scepter/main/mic/mic_audio.c").read_text(encoding="utf-8")
    assert "static uint8_t s_tx_codec = DUAL_END_AUDIO_CODEC_G722" in tx
    assert "dual_end_espnow_link_discard_audio();" in tx and "audio_recovery_tx_session_reset" in tx
    setter = function(tx, "dual_end_audio_sender_set_codec")
    assert setter.index("dual_end_opus_encoder_prepare_for(codec)") < setter.index("s_tx_codec = codec;")
    assert "if (pcm_frames == 0)" in mic
    assert "mic_owner_apply_dsp_config();" in mic
    assert "mic_config_transition_task_fn" in mic
    assert "s_config[2 + channel]" in mic and "mic_vad_process(pcm_buf)" in mic
    assert "dual_end_codec_payload_valid(repair.audio.codec" in rx
    assert "dual_end_codec_payload_valid(packet.audio.codec" in rx
    assert "dual_end_opus_decoder_reset();" in rx



def test_only_admitted_frames_bind_codec():
    source = (ROOT / "dongle/components/dual_end_audio/dual_end_audio.c").read_text(encoding="utf-8")
    for marker in ("receiver_bind_codec(repair.audio.codec", "receiver_bind_codec(packet.audio.codec"):
        offset = source.index(marker)
        assert "pushed == AUDIO_RX_ACCEPTED &&" in source[offset - 80:offset]
        assert source.rfind("audio_recovery_rx_push(", 0, offset) >= 0



def test_opus_sdk_recommended_output_capacity(tmp_path: Path):
    """Actual wrapper C with DSP stubs: capacity, separation and cleanup contracts."""
    stubs = tmp_path / "stubs"
    stubs.mkdir()
    (stubs / "esp_err.h").write_text("""#pragma once
    typedef int esp_err_t;
    #define ESP_OK 0
    #define ESP_FAIL -1
    #define ESP_ERR_INVALID_ARG -2
    #define ESP_ERR_NO_MEM -3
    #define ESP_ERR_INVALID_STATE -4
    """, encoding="utf-8")
    (stubs / "esp_log.h").write_text("""#pragma once
    #define ESP_LOGE(tag, ...) ((void)(tag))
    """, encoding="utf-8")
    (stubs / "esp_opus_enc.h").write_text("""#pragma once
    #include <stdbool.h>
    #include <stdint.h>
    typedef int esp_audio_err_t;
    #define ESP_AUDIO_ERR_OK 0
    #define ESP_OPUS_ENC_FRAME_DURATION_20_MS 3
    #define ESP_OPUS_ENC_APPLICATION_LOWDELAY 2
    typedef struct { int sample_rate, channel, bits_per_sample, bitrate;
      int frame_duration, application_mode, complexity;
      bool enable_fec, enable_dtx, enable_vbr; } esp_opus_enc_config_t;
    #define ESP_OPUS_ENC_CONFIG_DEFAULT() {0}
    typedef struct { uint8_t *buffer; uint32_t len; } esp_audio_enc_in_frame_t;
    typedef struct { uint8_t *buffer; uint32_t len, encoded_bytes; } esp_audio_enc_out_frame_t;
    esp_audio_err_t esp_opus_enc_open(void *, uint32_t, void **);
    esp_audio_err_t esp_opus_enc_get_frame_size(void *, int *, int *);
    esp_audio_err_t esp_opus_enc_process(void *, esp_audio_enc_in_frame_t *, esp_audio_enc_out_frame_t *);
    esp_audio_err_t esp_opus_enc_reset(void *);
    void esp_opus_enc_close(void *);
    """, encoding="utf-8")
    (stubs / "esp_opus_dec.h").write_text("""#pragma once
    #include "esp_opus_enc.h"
    #define ESP_OPUS_DEC_FRAME_DURATION_20_MS 3
    typedef struct { uint32_t sample_rate; uint8_t channel; int frame_duration;
      bool self_delimited; } esp_opus_dec_cfg_t;
    #define ESP_OPUS_DEC_CONFIG_DEFAULT() {0}
    typedef struct { uint8_t *buffer; uint32_t len, consumed; } esp_audio_dec_in_raw_t;
    typedef struct { uint8_t *buffer; uint32_t len, decoded_size, needed_size; } esp_audio_dec_out_frame_t;
    typedef struct { int unused; } esp_audio_dec_info_t;
    esp_audio_err_t esp_opus_dec_open(void *, uint32_t, void **);
    esp_audio_err_t esp_opus_dec_decode(void *, esp_audio_dec_in_raw_t *, esp_audio_dec_out_frame_t *, esp_audio_dec_info_t *);
    esp_audio_err_t esp_opus_dec_reset(void *);
    esp_audio_err_t esp_opus_dec_close(void *);
    """, encoding="utf-8")
    harness = tmp_path / "codec_sdk_capacity.c"
    harness.write_text(r"""
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_opus_enc.h"
#include "esp_opus_dec.h"
static unsigned allocations, frees, close_calls;
static int recommend_size = 160, encoded_size = 80, failed_open_index = -1, failed_reset_index = -1;
static int profile_bitrate = 32000;
static struct { int index; bool open; } handles[2] = {{0, false}, {1, false}};
static void *codec_test_malloc(size_t size) { ++allocations; return malloc(size); }
static void codec_test_free(void *ptr) { if (ptr) ++frees; free(ptr); }
esp_audio_err_t esp_opus_enc_open(void *config, uint32_t size, void **out) {
    esp_opus_enc_config_t *cfg = config;
    assert(size == sizeof(*cfg) && cfg->sample_rate == 16000 && cfg->channel == 1);
    assert(cfg->bits_per_sample == 16 && cfg->bitrate == profile_bitrate && cfg->complexity == 0);
    assert(cfg->frame_duration == 3 && cfg->application_mode == 2);
    assert(!cfg->enable_fec && !cfg->enable_dtx && !cfg->enable_vbr);
    for (int i = 0; i < 2; ++i) {
        if (handles[i].open) continue;
        if (i == failed_open_index) { *out = NULL; return -1; }
        handles[i].open = true; *out = &handles[i]; return 0;
    }
    assert(false); return -1;
}
esp_audio_err_t esp_opus_enc_get_frame_size(void *h, int *in_size, int *out_size) {
    assert(h); *in_size = 640; *out_size = recommend_size; return 0;
}
esp_audio_err_t esp_opus_enc_process(void *h, esp_audio_enc_in_frame_t *in, esp_audio_enc_out_frame_t *out) {
    const int index = h == &handles[0] ? 0 : 1;
    assert(handles[index].open && in->len == 640 && out->len == (uint32_t)recommend_size);
    const int16_t *pcm = (const int16_t *)in->buffer;
    for (int i = 0; i < 320; ++i) assert(pcm[i] == (index ? 2222 : 1111));
    assert((uint32_t)encoded_size <= out->len);
    memset(out->buffer, index ? 0x42 : 0x41, (size_t)encoded_size);
    out->encoded_bytes = (uint32_t)encoded_size; return 0;
}
esp_audio_err_t esp_opus_enc_reset(void *h) {
    const int index = h == &handles[0] ? 0 : 1;
    return index == failed_reset_index || failed_reset_index == -2 ? -1 : 0;
}
void esp_opus_enc_close(void *h) {
    const int index = h == &handles[0] ? 0 : 1;
    assert(handles[index].open); handles[index].open = false; ++close_calls;
}
esp_audio_err_t esp_opus_dec_open(void *cfg, uint32_t size, void **out) { (void)cfg; (void)size; *out = (void *)1; return 0; }
esp_audio_err_t esp_opus_dec_decode(void *h, esp_audio_dec_in_raw_t *in, esp_audio_dec_out_frame_t *out, esp_audio_dec_info_t *info) { (void)h; (void)in; (void)out; (void)info; return -1; }
esp_audio_err_t esp_opus_dec_reset(void *h) { (void)h; return 0; }
esp_audio_err_t esp_opus_dec_close(void *h) { (void)h; return 0; }
#define malloc codec_test_malloc
#define free codec_test_free
""" + '#include "' + (CODEC / "dual_end_opus.c").as_posix() + '"\n' + r"""
#undef malloc
#undef free
int main(void) {
    failed_open_index = 1;
    assert(dual_end_opus_encoder_prepare() == ESP_FAIL);
    assert(allocations == 1 && frees == 1 && close_calls == 1);
    assert(!handles[0].open && !handles[1].open);
    failed_open_index = -1;
    assert(dual_end_opus_encoder_prepare() == ESP_OK);
    assert(allocations == 3 && frees == 1);
    uint8_t payload[320]; uint32_t encoded = 0;
    int16_t pcm[640];
    for (unsigned i = 0; i < 320; ++i) { pcm[i * 2] = 1111; pcm[i * 2 + 1] = 2222; }
    assert(dual_end_opus_encode(pcm, payload, sizeof(payload), &encoded) == ESP_OK);
    assert(encoded == 164 && payload[0] == 80 && payload[2] == 80);
    assert(dual_end_codec_payload_valid(3, (uint16_t)encoded, payload, sizeof(payload)));
    for (unsigned i = 4; i < 84; ++i) assert(payload[i] == 0x41);
    for (unsigned i = 84; i < 164; ++i) assert(payload[i] == 0x42);
    for (unsigned i = 164; i < 320; ++i) assert(payload[i] == 0);
    assert(allocations == 3 && frees == 1); /* Cached handles and scratch reused. */
    encoded_size = 81;
    assert(dual_end_opus_encode(pcm, payload, sizeof(payload), &encoded) == ESP_FAIL);
    assert(payload[0] == 0 && payload[2] == 0); /* Oversized SDK packet never copied. */
    encoded_size = 80;
    failed_reset_index = 0;
    dual_end_opus_encoder_reset();
    assert(frees == 2 && !handles[0].open && handles[1].open);
    failed_reset_index = -1;
    assert(dual_end_opus_encoder_prepare() == ESP_OK);
    assert(allocations == 4);
    failed_reset_index = 1;
    dual_end_opus_encoder_reset();
    assert(frees == 3 && handles[0].open && !handles[1].open);
    failed_reset_index = -1;
    assert(dual_end_opus_encoder_prepare() == ESP_OK);
    assert(allocations == 5 && frees == 3 && handles[0].open && handles[1].open);
    assert(dual_end_opus_encode(pcm, payload, sizeof(payload), &encoded) == ESP_OK);
    failed_reset_index = -2;
    dual_end_opus_encoder_reset();
    assert(frees == allocations && !handles[0].open && !handles[1].open);
    recommend_size = 1276;
    failed_reset_index = -1;
    assert(dual_end_opus_encoder_prepare() == ESP_FAIL);
    assert(frees == allocations && !handles[0].open && !handles[1].open);
    /* Direct encode must repair either missing encoder without falling back
     * from the selected 62 kbps profile to the default 32 kbps profile. */
    recommend_size = 160; encoded_size = 155; profile_bitrate = 62000;
    assert(dual_end_opus_encoder_prepare_for(5) == ESP_OK);
    for (int side = 0; side < 2; ++side) {
        failed_reset_index = side;
        dual_end_opus_encoder_reset();
        assert(!handles[side].open && handles[1-side].open);
        const unsigned before = allocations, released = frees;
        failed_reset_index = -1;
        assert(dual_end_opus_encode(pcm, payload, sizeof(payload), &encoded) == ESP_OK);
        assert(allocations == before + 1 && frees == released);
        assert(handles[0].open && handles[1].open && encoded == 314);
        assert(dual_end_codec_payload_valid(5, (uint16_t)encoded, payload, sizeof(payload)));
    }
    failed_reset_index = -2; dual_end_opus_encoder_reset();
    assert(frees == allocations && !handles[0].open && !handles[1].open);
    puts("Opus wrapper SDK recommended 160 / actual 80, oversize, cleanup PASS (DSP stubbed)");
    return 0;
}
""", encoding="utf-8")
    executable, env = _build_host_executable([harness], tmp_path,
        extra_includes=(stubs, CODEC / "include"))
    result = _run([str(executable)], cwd=tmp_path, env=env)
    assert "cleanup PASS (DSP stubbed)" in result.stdout
