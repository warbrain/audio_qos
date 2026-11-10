"""Actual Opus wrapper contracts with DSP stubs, never an ESP/audio benchmark."""
from pathlib import Path
from test_audio_recovery_042 import _build_host_executable, _run

ROOT = Path(__file__).resolve().parents[1]
CODEC = ROOT / "components/dual_end_audio_codec"


def test_lowdelay_profile_preparation_and_packet_compatibility(tmp_path: Path):
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
    #define ESP_OPUS_ENC_APPLICATION_VOIP 0
    #define ESP_OPUS_ENC_APPLICATION_AUDIO 1
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
    harness = tmp_path / "opus_profile_actual_wrapper.c"
    harness.write_text(r"""
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_opus_enc.h"
#include "esp_opus_dec.h"
static struct { int index; bool open; } enc[2]={{0,false},{1,false}}, dec[2]={{0,false},{1,false}};
static unsigned live_allocations, allocations, frees, open_calls, encode_calls, decode_calls;
static int fail_open=-1, fail_malloc=-1, fail_size=-1, fail_process=-1, fail_decode=-1;
static int input_size=640, recommended=160, emitted=80;
static unsigned expected_decode_bytes=80;
static void *test_malloc(size_t bytes) {
    unsigned attempt=allocations++;
    if ((int)attempt==fail_malloc) return NULL;
    void *ptr=malloc(bytes); if(ptr) ++live_allocations; return ptr;
}
static void test_free(void *ptr) { if(ptr){ assert(live_allocations); --live_allocations; ++frees; } free(ptr); }
esp_audio_err_t esp_opus_enc_open(void *config,uint32_t size,void **out) {
    esp_opus_enc_config_t *cfg=config;
    assert(size==sizeof(*cfg));
    assert(cfg->sample_rate==16000 && cfg->channel==1 && cfg->bits_per_sample==16);
    assert(cfg->bitrate==32000 && cfg->frame_duration==3 && cfg->complexity==0);
    assert(cfg->application_mode==ESP_OPUS_ENC_APPLICATION_LOWDELAY);
    assert(!cfg->enable_fec && !cfg->enable_dtx && !cfg->enable_vbr);
    ++open_calls;
    for(int i=0;i<2;++i) if(!enc[i].open){
        /* Include a non-null handle on error: wrapper must still close it. */
        enc[i].open=true; *out=&enc[i]; return i==fail_open ? -1 : 0;
    }
    assert(false); return -1;
}
esp_audio_err_t esp_opus_enc_get_frame_size(void *h,int *in,int *out){
    *in=input_size; *out=recommended; return h==&enc[fail_size==1 ? 1 : 0] && fail_size>=0 ? -1 : 0;
}
esp_audio_err_t esp_opus_enc_process(void *h,esp_audio_enc_in_frame_t *in,esp_audio_enc_out_frame_t *out){
    int i=h==&enc[0] ? 0 : 1; ++encode_calls;
    assert(enc[i].open && in->len==640 && out->len==(uint32_t)recommended);
    for(unsigned n=0;n<320;++n) assert(((int16_t*)in->buffer)[n]==(i ? -2345 : 1234));
    assert(emitted<=recommended);
    memset(out->buffer, i ? 0x82 : 0x81,(size_t)emitted);
    out->encoded_bytes=(uint32_t)emitted; return i==fail_process ? -1 : 0;
}
esp_audio_err_t esp_opus_enc_reset(void *h){(void)h;return 0;}
void esp_opus_enc_close(void *h){int i=h==&enc[0] ? 0 : 1;assert(enc[i].open);enc[i].open=false;}
esp_audio_err_t esp_opus_dec_open(void *config,uint32_t size,void **out){
    esp_opus_dec_cfg_t *cfg=config;
    assert(size==sizeof(*cfg) && cfg->sample_rate==16000 && cfg->channel==1);
    assert(cfg->frame_duration==3 && !cfg->self_delimited);
    for(int i=0;i<2;++i) if(!dec[i].open){dec[i].open=true;*out=&dec[i];return 0;}
    assert(false);return -1;
}
esp_audio_err_t esp_opus_dec_decode(void *h,esp_audio_dec_in_raw_t *in,esp_audio_dec_out_frame_t *out,esp_audio_dec_info_t *info){
    int i=h==&dec[0] ? 0 : 1; (void)info; ++decode_calls;
    assert(in->len==expected_decode_bytes && out->len==640 && in->buffer[0]==(i ? 0x82 : 0x81));
    for(unsigned n=0;n<320;++n) ((int16_t*)out->buffer)[n]=(int16_t)(i ? -2345 : 1234);
    in->consumed=expected_decode_bytes;out->decoded_size=640;return i==fail_decode ? -1 : 0;
}
esp_audio_err_t esp_opus_dec_reset(void *h){(void)h;return 0;}
esp_audio_err_t esp_opus_dec_close(void *h){int i=h==&dec[0] ? 0 : 1;assert(dec[i].open);dec[i].open=false;return 0;}
#define malloc test_malloc
#define free test_free
""" + '#include "' + (CODEC / "dual_end_opus.c").as_posix() + '"\n' + r"""
#undef malloc
#undef free
static void clean(void){close_encoders();close_decoders();assert(!enc[0].open && !enc[1].open && live_allocations==0);}
static void invalid_packet(const uint8_t *packet) {
    int16_t output[640];memset(output,0x55,sizeof(output));
    unsigned before=decode_calls;
    assert(dual_end_opus_decode(packet,320,output,sizeof(output))==ESP_ERR_INVALID_ARG);
    assert(decode_calls==before);
    for(unsigned i=0;i<sizeof(output);++i) assert(((uint8_t*)output)[i]==0x55);
}
int main(void){
    for(int channel=0;channel<2;++channel){
        fail_open=channel;assert(dual_end_opus_encoder_prepare()==ESP_FAIL);clean();
    }
    fail_open=-1;
    for(int channel=0;channel<2;++channel){
        fail_size=channel;assert(dual_end_opus_encoder_prepare()==ESP_FAIL);clean();
    }
    fail_size=-1;
    for(int channel=0;channel<2;++channel){
        fail_malloc=(int)allocations+channel;assert(dual_end_opus_encoder_prepare()==ESP_ERR_NO_MEM);clean();
    }
    fail_malloc=-1;
    input_size=639;assert(dual_end_opus_encoder_prepare()==ESP_FAIL);clean();input_size=640;
    recommended=79;assert(dual_end_opus_encoder_prepare()==ESP_FAIL);clean();
    recommended=1276;assert(dual_end_opus_encoder_prepare()==ESP_FAIL);clean();recommended=160;
    int16_t pcm[640],decoded[640];uint8_t packet[320];uint32_t size=0;
    for(unsigned n=0;n<320;++n){pcm[n*2]=1234;pcm[n*2+1]=-2345;}
    unsigned before=open_calls;
    assert(dual_end_opus_encoder_prepare()==ESP_OK && open_calls==before+2);
    assert(dual_end_opus_encode(pcm,packet,320,&size)==ESP_OK && size==164);
    assert(open_calls==before+2 && live_allocations==2);
    assert(dual_end_codec_payload_valid(3,(uint16_t)size,packet,320));
    for(unsigned n=164;n<320;++n) assert(packet[n]==0);
    assert(dual_end_opus_decode(packet,320,decoded,sizeof(decoded))==ESP_OK);
    assert(!memcmp(pcm,decoded,sizeof(pcm)));
    /* Both length prefixes and padding must pass before any channel mutates. */
    unsigned previous_decodes=decode_calls;
    packet[2]=79;invalid_packet(packet);
    packet[2]=80;packet[319]=1;invalid_packet(packet);
    packet[319]=0;assert(decode_calls==previous_decodes);
    uint8_t valid32[320],valid62[320]={0},invalid[320];memcpy(valid32,packet,320);
    valid62[0]=valid62[2]=155;
    memset(valid62+4,0x81,155);memset(valid62+159,0x82,155);
    expected_decode_bytes=155;
    assert(dual_end_opus_decode(valid62,320,decoded,sizeof(decoded))==ESP_OK);
    assert(!memcmp(pcm,decoded,sizeof(pcm)));
    const uint16_t bad_lengths[]={0,79,81,154,156,UINT16_MAX};
    for(unsigned profile=0;profile<2;++profile) {
        const uint8_t *valid=profile?valid62:valid32;
        for(unsigned side=0;side<2;++side) {
            for(unsigned n=0;n<sizeof(bad_lengths)/sizeof(bad_lengths[0]);++n) {
                memcpy(invalid,valid,320);
                invalid[2*side]=(uint8_t)bad_lengths[n];
                invalid[2*side+1]=(uint8_t)(bad_lengths[n]>>8);
                invalid_packet(invalid);
            }
        }
        memcpy(invalid,valid,320);invalid[319]=1;invalid_packet(invalid);
    }
    expected_decode_bytes=80;memcpy(packet,valid32,320);
    fail_decode=1;memset(decoded,0x55,sizeof(decoded));
    assert(dual_end_opus_decode(packet,320,decoded,sizeof(decoded))==ESP_FAIL);
    for(unsigned n=0;n<sizeof(decoded);++n) assert(((uint8_t*)decoded)[n]==0x55);
    fail_decode=-1;
    for(int channel=0;channel<2;++channel){
        fail_process=channel;assert(dual_end_opus_encode(pcm,packet,320,&size)==ESP_FAIL);
    }
    fail_process=-1;emitted=79;assert(dual_end_opus_encode(pcm,packet,320,&size)==ESP_FAIL);
    emitted=81;assert(dual_end_opus_encode(pcm,packet,320,&size)==ESP_FAIL);
    emitted=80;assert(dual_end_opus_encode(pcm,packet,320,&size)==ESP_OK);
    assert(dual_end_opus_encode(NULL,packet,320,&size)==ESP_ERR_INVALID_ARG);
    assert(dual_end_opus_encode(pcm,packet,319,&size)==ESP_ERR_INVALID_ARG);
    clean();assert(encode_calls>0 && frees>0);
    puts("actual LOWDELAY wrapper config / guards / cleanup / standard decoder routing PASS (DSP mocked)");
    return 0;
}
""", encoding="utf-8")
    exe, env = _build_host_executable([harness], tmp_path, extra_includes=(stubs, CODEC / "include"))
    result = _run([str(exe)], cwd=tmp_path, env=env)
    assert "standard decoder routing PASS (DSP mocked)" in result.stdout
