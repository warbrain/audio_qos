"""Execute real sender/codec C with stateful ESP SDK mocks, never hardware DSP.

Wire bytes, histories, resets, per-session counters and failure admission are
tested. Bit equality of the actual Xtensa SDK codec remains a device check.
"""
from pathlib import Path
import re

from test_audio_recovery_042 import _build_host_executable, _run

ROOT = Path(__file__).resolve().parents[1]
CODEC = ROOT / "components/dual_end_audio_codec"
SENDER = ROOT / "scepter/components/dual_end_audio"


def definition(source: str, name: str) -> str:
    """Select a definition, never an earlier wrapper call of the same name."""
    match = re.search(r"(?m)^(?:static )?(?:void|esp_err_t|bool|dual_end_owner_outcome_t) " + re.escape(name) + r"\(", source)
    assert match is not None, name
    opening = source.index("{", match.start())
    depth, end = 1, opening + 1
    while depth:
        if source[end] == "{": depth += 1
        elif source[end] == "}": depth -= 1
        end += 1
    return source[match.start():end]


def test_owner_dual_mono_sender_and_codec_state(tmp_path: Path):
    stubs = tmp_path / "stubs"
    stubs.mkdir()
    (stubs / "esp_err.h").write_text("""#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG -2
#define ESP_ERR_NO_MEM -3
#define ESP_ERR_INVALID_SIZE -4
#define ESP_ERR_INVALID_STATE -5
#define ESP_ERR_TIMEOUT -6
""", encoding="utf-8")
    (stubs / "esp_log.h").write_text("""#pragma once
#define ESP_LOGE(tag, ...) ((void)(tag))
#define ESP_LOGI(tag, ...) do { (void)(tag); printf(__VA_ARGS__); putchar('\\n'); } while(0)
#define ESP_LOGD(tag, ...) ((void)(tag))
""", encoding="utf-8")
    (stubs / "esp_opus_enc.h").write_text("""#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef int esp_audio_err_t;
#define ESP_AUDIO_ERR_OK 0
#define ESP_OPUS_ENC_FRAME_DURATION_20_MS 3
#define ESP_OPUS_ENC_APPLICATION_LOWDELAY 2
typedef struct { int sample_rate,channel,bits_per_sample,bitrate;
 int frame_duration,application_mode,complexity; bool enable_fec,enable_dtx,enable_vbr;
} esp_opus_enc_config_t;
#define ESP_OPUS_ENC_CONFIG_DEFAULT() {0}
typedef struct { uint8_t *buffer; uint32_t len; } esp_audio_enc_in_frame_t;
typedef struct { uint8_t *buffer; uint32_t len,encoded_bytes; } esp_audio_enc_out_frame_t;
esp_audio_err_t esp_opus_enc_open(void *, uint32_t, void **);
esp_audio_err_t esp_opus_enc_get_frame_size(void *, int *, int *);
esp_audio_err_t esp_opus_enc_process(void *, esp_audio_enc_in_frame_t *, esp_audio_enc_out_frame_t *);
esp_audio_err_t esp_opus_enc_reset(void *);
void esp_opus_enc_close(void *);
""", encoding="utf-8")
    (stubs / "esp_opus_dec.h").write_text("""#pragma once
#include "esp_opus_enc.h"
#define ESP_OPUS_DEC_FRAME_DURATION_20_MS 3
typedef struct { uint32_t sample_rate; uint8_t channel; int frame_duration; bool self_delimited; } esp_opus_dec_cfg_t;
#define ESP_OPUS_DEC_CONFIG_DEFAULT() {0}
typedef struct { uint8_t *buffer; uint32_t len,consumed; } esp_audio_dec_in_raw_t;
typedef struct { uint8_t *buffer; uint32_t len,decoded_size,needed_size; } esp_audio_dec_out_frame_t;
typedef struct { int unused; } esp_audio_dec_info_t;
esp_audio_err_t esp_opus_dec_open(void *, uint32_t, void **);
esp_audio_err_t esp_opus_dec_decode(void *, esp_audio_dec_in_raw_t *, esp_audio_dec_out_frame_t *, esp_audio_dec_info_t *);
esp_audio_err_t esp_opus_dec_reset(void *);
esp_audio_err_t esp_opus_dec_close(void *);
""", encoding="utf-8")
    (stubs / "esp_g722_enc.h").write_text("""#pragma once
#include "esp_opus_enc.h"
typedef struct { int sample_rate,channel,bits_per_sample,bitrate,frame_duration; bool packed; } esp_g722_enc_config_t;
#define ESP_G722_ENC_CONFIG_DEFAULT() {0}
esp_audio_err_t esp_g722_enc_open(void *,uint32_t,void **);
esp_audio_err_t esp_g722_enc_get_frame_size(void *,int *,int *);
esp_audio_err_t esp_g722_enc_process(void *,esp_audio_enc_in_frame_t *,esp_audio_enc_out_frame_t *);
esp_audio_err_t esp_g722_enc_reset(void *);
void esp_g722_enc_close(void *);
""", encoding="utf-8")
    source = (SENDER / "dual_end_audio.c").read_text(encoding="utf-8")
    extracted = "\n".join(definition(source, name) for name in (
        "ensure_g722_encoder", "g722_encode_frame", "sender_reset_encoders_locked",
        "sender_invalidate_session_locked", "sender_new_session_locked",
        "dual_end_audio_sender_set_processing_layout",
        "dual_end_audio_sender_restart_processing", "dual_end_audio_sender_set_codec",
        "dual_end_audio_sender_get_encode_stats",
        "dual_end_audio_sender_copy_last_owner_timing",
        "sender_current_owner_outcome",
        "dual_end_audio_sender_record_mic_work",
        "dual_end_audio_sender_send_pcm_with_hardware_and_analysis",
    ))
    harness = tmp_path / "sender_dual_mono_actual.c"
    harness.write_text(r'''
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dual_end_audio.h"
#include "dual_end_opus.h"
#include "dual_end_pcm_layout.h"
#include "dual_end_spatial.h"
#include "dual_end_perf_stats.h"
#include "audio_recovery_tx.h"
#include "esp_opus_enc.h"
#include "esp_opus_dec.h"
#include "esp_g722_enc.h"
#include "esp_log.h"
typedef struct { bool open; unsigned history,index,kind,calls,resets; } handle_t;
static handle_t enc[2][2],dec[2];
static int fail_open_kind=-1,fail_reset_index=-1,fail_process_index=-1;
static bool bad_g722_size;
static unsigned sdk_calls[2],open_calls[2],close_calls[2];
static esp_audio_err_t mock_open(unsigned kind,void **out) {
    ++open_calls[kind];
    for(unsigned i=0;i<2;++i) if(!enc[kind][i].open) {
        enc[kind][i]=(handle_t){.open=true,.index=i,.kind=kind};
        *out=&enc[kind][i]; return (int)kind==fail_open_kind ? -1 : 0;
    }
    assert(false);return -1;
}
static esp_audio_err_t mock_process(void *h,esp_audio_enc_in_frame_t *in,esp_audio_enc_out_frame_t *out) {
    handle_t *state=h; assert(state && state->open && in->len==640 && out->len>=160);
    ++sdk_calls[state->kind];++state->calls;
    unsigned value=state->history ^ 0x9E3779B9u;
    for(unsigned n=0;n<640;++n) value=(value^in->buffer[n])*16777619u;
    state->history=value;
    unsigned bytes=state->kind ? 80 : 160;
    for(unsigned n=0;n<bytes;++n) out->buffer[n]=(uint8_t)((value>>((n%4)*8))+n);
    out->encoded_bytes=bytes;
    return (int)state->index==fail_process_index ? -1 : 0;
}
static esp_audio_err_t mock_reset(void *h) {
    handle_t *state=h;assert(state && state->open);++state->resets;
    if((int)state->index==fail_reset_index || fail_reset_index==-2)return -1;
    state->history=0;return 0;
}
static void mock_close(void *h) {
    handle_t *state=h;assert(state && state->open);++close_calls[state->kind];state->open=false;
}
esp_audio_err_t esp_opus_enc_open(void *cfg,uint32_t size,void **out) {
    esp_opus_enc_config_t *c=cfg;assert(size==sizeof(*c));
    assert(c->sample_rate==16000 && c->channel==1 && c->bitrate==32000 && c->complexity==0);
    assert(c->frame_duration==3 && c->application_mode==2);
    assert(!c->enable_fec && !c->enable_dtx && !c->enable_vbr);return mock_open(1,out);
}
esp_audio_err_t esp_opus_enc_get_frame_size(void *h,int *in,int *out) {(void)h;*in=640;*out=160;return 0;}
esp_audio_err_t esp_opus_enc_process(void *h,esp_audio_enc_in_frame_t *in,esp_audio_enc_out_frame_t *out){return mock_process(h,in,out);}
esp_audio_err_t esp_opus_enc_reset(void *h){return mock_reset(h);}
void esp_opus_enc_close(void *h){mock_close(h);}
esp_audio_err_t esp_g722_enc_open(void *cfg,uint32_t size,void **out) {
    esp_g722_enc_config_t *c=cfg;assert(size==sizeof(*c));
    assert(c->sample_rate==16000 && c->channel==1 && c->bitrate==64000);
    assert(c->bits_per_sample==16 && c->packed && c->frame_duration==20);return mock_open(0,out);
}
esp_audio_err_t esp_g722_enc_get_frame_size(void *h,int *in,int *out) {(void)h;*in=bad_g722_size ? 639 : 640;*out=160;return 0;}
esp_audio_err_t esp_g722_enc_process(void *h,esp_audio_enc_in_frame_t *in,esp_audio_enc_out_frame_t *out){return mock_process(h,in,out);}
esp_audio_err_t esp_g722_enc_reset(void *h){return mock_reset(h);}
void esp_g722_enc_close(void *h){mock_close(h);}
esp_audio_err_t esp_opus_dec_open(void *cfg,uint32_t size,void **out) {
    (void)cfg;(void)size;for(unsigned i=0;i<2;++i)if(!dec[i].open){dec[i]=(handle_t){.open=true,.index=i};*out=&dec[i];return 0;}
    assert(false);return -1;
}
esp_audio_err_t esp_opus_dec_decode(void *h,esp_audio_dec_in_raw_t *in,esp_audio_dec_out_frame_t *out,esp_audio_dec_info_t *info) {
    (void)info;handle_t *state=h;assert(in->len==80 && out->len==640);++state->calls;
    state->history=state->history*33+in->buffer[0];
    for(unsigned n=0;n<320;++n)((int16_t*)out->buffer)[n]=(int16_t)(state->history+n);
    in->consumed=80;out->decoded_size=640;return 0;
}
esp_audio_err_t esp_opus_dec_reset(void *h){handle_t *state=h;state->history=0;return 0;}
esp_audio_err_t esp_opus_dec_close(void *h){((handle_t*)h)->open=false;return 0;}
''' + '#include "' + (CODEC / "dual_end_opus.c").as_posix() + '"\n' + r'''
#define ESP_RETURN_ON_ERROR(expr,tag,...) do { esp_err_t r=(expr);if(r!=ESP_OK)return r; } while(0)
#define ESP_RETURN_ON_FALSE(expr,error,tag,...) do { if(!(expr))return error; } while(0)
#define portMAX_DELAY 0
#define xSemaphoreTake(a,b) ((void)(a),(void)(b))
#define xSemaphoreGive(a) ((void)(a))
#define AUDIO_TX_LOG_INTERVAL_FRAMES 50
#define DUAL_END_STAGE_ENCODE 0
#define DUAL_END_STAGE_MIC_PIPELINE 1
static unsigned discards,admitted;
static unsigned outcome_records;
static dual_end_owner_outcome_t last_outcome;
static uint8_t last_outcome_codec;
static uint32_t last_outcome_session;
static int64_t last_outcome_elapsed;
static bool link_connected=true,radio_paused;
  static esp_err_t power_result=ESP_OK,radio_result=ESP_OK;
static int64_t clock_us=10000;
static int64_t clock_step=1;
static int64_t esp_timer_get_time(void){clock_us+=clock_step;return clock_us;}
static uint32_t esp_random(void){static uint32_t seed=100;return ++seed;}
static const char *esp_err_to_name(esp_err_t error){(void)error;return "mock";}
static uint32_t audio_payload_crc32(const uint8_t *p,size_t n) {
    uint32_t c=UINT32_MAX;for(size_t i=0;i<n;++i){c^=p[i];for(int b=0;b<8;++b)c=(c>>1)^((0u-(c&1))&0xEDB88320u);}return c^UINT32_MAX;
}
static void *s_g722_encoder[2];
_Alignas(16) static int16_t s_encode_channel_pcm[2][320];
static void *s_sender_mutex=(void*)1;
static uint8_t s_tx_codec=DUAL_END_AUDIO_CODEC_G722,s_tx_layout=DUAL_END_LAYOUT_STEREO;
static dual_end_audio_encode_stats_t s_encoding_stats;
static bool s_session_ready=true,s_sender_link_connected=true,s_ota_paused,s_channel_paused;
static uint32_t s_session_id=99,s_session_first_sequence,s_tx_sequence=1001,s_tx_capture_sample_index;
static int64_t s_session_last_begin_us;
static audio_recovery_tx_t cache;
static audio_recovery_tx_t *s_recovery_cache=&cache;
static struct { unsigned sessions,waiting_frames; } s_recovery_stats;
static uint32_t s_encode_max_us;
static bool s_encode_attempt_pending,s_encode_attempt_success;
static dual_end_owner_outcome_t s_encode_attempt_outcome=DUAL_END_OUTCOME_UNCLASSIFIED;
static uint8_t s_encode_attempt_codec;
static uint32_t s_encode_attempt_session;
static dual_end_audio_owner_timing_t s_last_owner_timing;
static uint32_t s_owner_timing_produced_generation,s_owner_timing_consumed_generation;
static int8_t s_last_tx_power_qdbm=80;
static uint8_t s_pending_input_flags;
typedef struct { uint32_t tx_audio_submit,tx_queue_depth,tx_queue_high_water,tx_audio_stale_drop,
 tx_audio_final_ok,tx_audio_final_fail,tx_audio_deadline_drop,tx_repair_ok,tx_repair_fail,tx_repair_deadline_drop;
 bool connected;int last_rssi; } dual_end_espnow_link_stats_t;
static dual_end_espnow_link_stats_t s_last_tx_log_stats;
static int64_t s_last_tx_log_us;
#pragma pack(push,1)
typedef struct {dual_end_packet_header_t packet;dual_end_audio_frame_header_t audio;
 dual_end_audio_hardware_payload_t payload;} test_packet_t;
#pragma pack(pop)
static test_packet_t s_tx_packet,last_packet;
static void dual_end_espnow_link_discard_audio(void){++discards;}
static bool dual_end_espnow_link_is_connected(void){return link_connected;}
static bool dual_end_espnow_link_is_audio_paused(void){return radio_paused;}
static uint8_t dual_end_espnow_link_take_unsent_input_flags(void){return 0;}
static esp_err_t dual_end_espnow_link_set_tx_power(int8_t p){(void)p;return power_result;}
static void dual_end_espnow_link_get_stats(dual_end_espnow_link_stats_t *s){memset(s,0,sizeof(*s));}
 static esp_err_t dual_end_espnow_link_send_audio_at(const void *p,int size,int64_t created) {
     (void)created;assert(size==400);last_packet=*(const test_packet_t*)p;++admitted;return radio_result;
}
static void dual_end_runtime_record(int stage,uint8_t codec,uint32_t session,int64_t elapsed,bool ok){
 (void)stage;(void)codec;(void)session;(void)elapsed;(void)ok;}
static void dual_end_runtime_record_outcome(int stage,uint8_t codec,uint32_t session,int64_t elapsed,dual_end_owner_outcome_t outcome){
 assert(stage==DUAL_END_STAGE_MIC_PIPELINE);++outcome_records;last_outcome=outcome;
 last_outcome_codec=codec;last_outcome_session=session;last_outcome_elapsed=elapsed;}
/* Transport gate is independently tested by test_audio_recovery_042. Here it
 * stays connected; real new-session/cache/reset and actual send admission run. */
static void sender_update_session_gate_locked(bool connected,bool ota,bool channel,int64_t now) {
 (void)now;s_sender_link_connected=connected;s_ota_paused=ota;s_channel_paused=channel;}
static bool sender_session_available_locked(void){return s_sender_link_connected && !s_ota_paused && !s_channel_paused;}
static void sender_send_begin_locked(int64_t now){(void)now;}
''' + extracted + r'''
static void ready(void){s_session_ready=true;}
static void mic_outcome(dual_end_owner_outcome_t expected){
 unsigned before=outcome_records;dual_end_audio_sender_record_mic_work(1234);
 assert(outcome_records==before+1U && last_outcome==expected && last_outcome_elapsed==1234);
 assert(!s_encode_attempt_pending && s_encode_attempt_outcome==DUAL_END_OUTCOME_UNCLASSIFIED);
}
static void fill(int16_t *pcm,unsigned frame,bool stereo) {
    for(unsigned n=0;n<320;++n){pcm[2*n]=(int16_t)(n*97+frame*333-15000);
        pcm[2*n+1]=stereo ? (int16_t)(7000-n*29-frame*39) : pcm[2*n];}
}
static void check_packet(uint8_t codec,bool dual) {
    assert(last_packet.audio.codec==codec && last_packet.audio.channels==2);
    assert(last_packet.audio.sample_rate_hz==16000 && last_packet.audio.samples_per_channel==320);
    assert(last_packet.audio.pcm_bytes_after_decode==1280);
    assert(last_packet.audio.previous_encoded_bytes==0);
    assert(last_packet.audio.payload_crc32==audio_payload_crc32((uint8_t*)&last_packet.payload,sizeof(last_packet.payload)));
    assert(dual_end_codec_payload_valid(codec,last_packet.audio.current_encoded_bytes,last_packet.payload.audio,320));
    if(dual){size_t off=codec==3 ? 4 : 0,len=codec==3 ? 80 : 160;
        assert(!memcmp(last_packet.payload.audio+off,last_packet.payload.audio+off+len,len));}
    const audio_recovery_tx_entry_t *entry=NULL;
    assert(audio_recovery_tx_get(&cache,s_session_id,last_packet.packet.sequence,clock_us+100000,&entry)==AUDIO_RECOVERY_TX_HIT);
    assert(entry && !memcmp(entry->encoded,last_packet.payload.audio,320));
}
int main(void) {
    assert(sizeof(test_packet_t)==400);
    audio_recovery_tx_reset(&cache,s_session_id);
    uint8_t expected[12][320];int16_t pcm[640],original[640],decoded[640];
    for(unsigned codec_index=0;codec_index<2;++codec_index){
        uint8_t codec=codec_index ? 3 : 4;
        assert(dual_end_audio_sender_set_codec(codec)==ESP_OK);
        assert(dual_end_audio_sender_set_processing_layout(0)==ESP_OK);ready();
        uint32_t stereo_session=s_session_id;
        /* Owner-declared stereo with equal L/R still does TWO SDK calls. */
        for(unsigned frame=0;frame<12;++frame){
            fill(pcm,frame,false);memcpy(original,pcm,sizeof(pcm));
            unsigned before=sdk_calls[codec_index];uint32_t sequence=s_tx_sequence;
            assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_OK);
            assert(sdk_calls[codec_index]==before+2 && s_tx_sequence==sequence+1);
            assert(!memcmp(pcm,original,sizeof(pcm)));check_packet(codec,true);
            memcpy(expected[frame],last_packet.payload.audio,320);
        }
        dual_end_audio_encode_stats_t stats;
        dual_end_audio_sender_get_encode_stats(&stats);
        assert(stats.frames==12 && stats.process_calls==24 && !stats.reused_frames && !stats.failures);
        uint32_t old_sequence=last_packet.packet.sequence;
        assert(dual_end_audio_sender_set_processing_layout(1)==ESP_OK);ready();
        assert(s_session_id!=stereo_session && !cache.entries[old_sequence%DUAL_END_AUDIO_TX_CACHE_WINDOW].valid);
        unsigned idle_calls=enc[codec_index][1].calls;
        for(unsigned frame=0;frame<12;++frame){
            fill(pcm,frame,false);unsigned before=sdk_calls[codec_index];
            assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_OK);
            assert(sdk_calls[codec_index]==before+1 && enc[codec_index][1].calls==idle_calls);
            check_packet(codec,true);assert(!memcmp(expected[frame],last_packet.payload.audio,320));
            if(codec==3){assert(dual_end_opus_decode(last_packet.payload.audio,320,decoded,sizeof(decoded))==ESP_OK);
                for(unsigned n=0;n<320;++n)assert(decoded[2*n]==decoded[2*n+1]);}
        }
        dual_end_audio_sender_get_encode_stats(&stats);
        assert(stats.layout==1 && stats.frames==12 && stats.process_calls==12 && stats.reused_frames==12);
        uint32_t old_session=s_session_id;
        assert(dual_end_audio_sender_set_processing_layout(1)==ESP_OK && s_session_id!=old_session);ready();
        /* DAS -> NLMS stays dual mono, but resets the session and histories. */
        fill(pcm,0,false);assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_OK);
        assert(!memcmp(expected[0],last_packet.payload.audio,320));
        old_session=s_session_id;unsigned old_admitted=admitted,old_calls=sdk_calls[codec_index];
        pcm[639]^=1;
        assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_ERR_INVALID_ARG);
        assert(sdk_calls[codec_index]==old_calls && admitted==old_admitted && s_session_id!=old_session);
        ready();fill(pcm,0,false);
        assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_OK);
        assert(!memcmp(expected[0],last_packet.payload.audio,320));
        /* Back to true stereo must not reuse the idle right encoder history. */
        assert(dual_end_audio_sender_set_processing_layout(0)==ESP_OK);ready();fill(pcm,0,true);
        assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_OK);
        assert(enc[codec_index][0].calls>=1 && enc[codec_index][1].history!=0);
        assert(memcmp(last_packet.payload.audio+(codec==3 ? 4 : 0),
                      last_packet.payload.audio+(codec==3 ? 84 : 160),codec==3 ? 80 : 160));
        /* New NLMS-only mode retains separate microphone outputs. The real
         * kernel's layout drives two SDK calls even while channels converge;
         * it must never take the fused-mode one-encoder shortcut. */
        dual_end_spatial_t spatial;
        assert(dual_end_spatial_reset(&spatial,DUAL_END_SPATIAL_NLMS_STEREO,9));
        assert(dual_end_audio_sender_set_processing_layout((uint8_t)spatial.stats.layout)==ESP_OK);ready();
        unsigned stereo_calls=sdk_calls[codec_index];
        fill(pcm,0,true);assert(dual_end_spatial_process(&spatial,pcm,320));
        memcpy(original,pcm,sizeof(pcm));
        assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_OK);
        assert(sdk_calls[codec_index]==stereo_calls+2&&!memcmp(original,pcm,sizeof(pcm)));
        dual_end_audio_sender_get_encode_stats(&stats);
        assert(stats.layout==0&&stats.frames==1&&stats.process_calls==2&&!stats.reused_frames);
        assert(memcmp(last_packet.payload.audio+(codec==3 ? 4 : 0),
                      last_packet.payload.audio+(codec==3 ? 84 : 160),codec==3 ? 80 : 160));
        /* A reset error forces handle rebuild. A failed rebuild returns error,
         * leaves stereo applied and admits no packet, rather than faking ACK. */
        fail_reset_index=-2;fail_open_kind=(int)codec_index;old_session=s_session_id;old_admitted=admitted;
        assert(dual_end_audio_sender_set_processing_layout(1)==ESP_FAIL);
        assert(s_tx_layout==0 && s_session_id!=old_session && admitted==old_admitted);
        fail_reset_index=-1;fail_open_kind=-1;
        assert(dual_end_audio_sender_set_processing_layout(1)==ESP_OK);ready();
        fill(pcm,0,false);assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_OK);
        assert(!memcmp(expected[0],last_packet.payload.audio,320));
        /* Failed SDK process mutates history: sender must reset/new session. */
        fail_process_index=0;old_session=s_session_id;old_admitted=admitted;
        assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_FAIL);
        assert(admitted==old_admitted && s_session_id!=old_session);
        mic_outcome(DUAL_END_OUTCOME_OPERATION_FAILED);
        assert(last_outcome_codec==codec && last_outcome_session==old_session);
        fail_process_index=-1;ready();
        assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_OK);
        assert(!memcmp(expected[0],last_packet.payload.audio,320));
    }
    uint32_t session=s_session_id;
    assert(dual_end_audio_sender_set_processing_layout(2)==ESP_ERR_INVALID_ARG && s_session_id==session);
    s_sender_mutex=NULL;assert(dual_end_audio_sender_set_processing_layout(0)==ESP_ERR_INVALID_STATE);s_sender_mutex=(void*)1;
    assert(dual_end_audio_sender_set_codec(4)==ESP_OK);
    fail_reset_index=-2;bad_g722_size=true;
    assert(dual_end_audio_sender_set_processing_layout(0)==ESP_FAIL);
    bad_g722_size=false;fail_reset_index=-1;
    assert(dual_end_audio_sender_set_processing_layout(0)==ESP_OK);
    assert(discards>0 && open_calls[0]>2 && open_calls[1]>2 && close_calls[0] && close_calls[1]);
    /* Real admission branches must classify the current callback before codec
     * work; consuming a reason clears it, and attempted work retains its own
     * codec/session even if a failure moved the sender into a new session. */
    fill(pcm,0,true);ready();s_last_tx_power_qdbm=80;
    unsigned sdk_before=sdk_calls[0]+sdk_calls[1],admitted_before=admitted;
    link_connected=false;
    assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_ERR_INVALID_STATE);
    mic_outcome(DUAL_END_OUTCOME_WAIT_SESSION);link_connected=true;
    s_session_ready=false;
    assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_ERR_INVALID_STATE);
    mic_outcome(DUAL_END_OUTCOME_WAIT_ACK);ready();
    clock_step=200001;
    assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_ERR_TIMEOUT);
    mic_outcome(DUAL_END_OUTCOME_DEADLINE);clock_step=1;
    s_last_tx_power_qdbm=INT8_MIN;power_result=ESP_FAIL;
    assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_FAIL);
    mic_outcome(DUAL_END_OUTCOME_POWER);power_result=ESP_OK;s_last_tx_power_qdbm=80;
    assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(NULL,640,NULL,NULL)==ESP_ERR_INVALID_ARG);
    mic_outcome(DUAL_END_OUTCOME_INVALID_REQUEST);
    assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,639,NULL,NULL)==ESP_ERR_INVALID_SIZE);
    mic_outcome(DUAL_END_OUTCOME_INVALID_REQUEST);
    assert(sdk_calls[0]+sdk_calls[1]==sdk_before && admitted==admitted_before);
    uint32_t current_session=s_session_id;
    assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_OK);
    dual_end_audio_owner_timing_t timing = {0};
    assert(dual_end_audio_sender_copy_last_owner_timing(&timing));
    assert(timing.valid && timing.codec==s_tx_codec && timing.session_id==current_session &&
           timing.encode_attempted && timing.radio_submitted);
    assert(!dual_end_audio_sender_copy_last_owner_timing(&timing) && !timing.valid);
    mic_outcome(DUAL_END_OUTCOME_OK);
    assert(last_outcome_codec==s_tx_codec && last_outcome_session==current_session);
    /* A successful encoder is not a successful MIC_PIPELINE admission when
     * the radio queue rejects the packet. */
    radio_result=ESP_FAIL;
    assert(dual_end_audio_sender_send_pcm_with_hardware_and_analysis(pcm,640,NULL,NULL)==ESP_FAIL);
    assert(dual_end_audio_sender_copy_last_owner_timing(&timing));
    assert(timing.encode_ok && timing.radio_submitted && !timing.radio_ok);
    mic_outcome(DUAL_END_OUTCOME_OPERATION_FAILED);radio_result=ESP_OK;
    mic_outcome(DUAL_END_OUTCOME_UNCLASSIFIED); /* No stale successful attempt. */
    unsigned records_before=outcome_records;s_sender_mutex=NULL;
    dual_end_audio_sender_record_mic_work(1234);assert(outcome_records==records_before);
    s_sender_mutex=(void*)1;
    puts("actual sender: dual-mono one call / 400-byte wire / CRC / cache / state switch / rebuild / no failed admission PASS (SDK mocked)");
    return 0;
}
''', encoding="utf-8")
    exe, env = _build_host_executable(
        [harness, ROOT / "components/dual_end_audio_recovery/audio_recovery_tx.c",
         CODEC / "dual_end_spatial.c", CODEC / "dual_end_wiener.c"],
        tmp_path, extra_includes=(stubs, CODEC / "include", SENDER / "include",
                                 ROOT / "components/dual_end_runtime/include"))
    result = _run([str(exe)], cwd=tmp_path, env=env)
    assert "no failed admission PASS (SDK mocked)" in result.stdout
    # Failure counts survive a session reset in its closing summary; failed SDK
    # calls are separate from completed-frame calls and from a PCM contract fault.
    summaries = [line for line in result.stdout.splitlines()
                 if line.startswith("codec_session_summary ")]
    assert sum("failures=1 failed_process_calls=0 layout_mismatches=1" in line
               for line in summaries) == 2
    assert sum("failures=1 failed_process_calls=1 layout_mismatches=0" in line
               for line in summaries) == 2
