"""Real Dongle BEGIN/recovery/NACK integration with deterministic host boundaries."""
import os
from pathlib import Path

from test_runtime_codec import function
from test_audio_recovery_042 import _build_host_executable, _run

ROOT=Path(os.environ.get('FIRMWARE_SESSION_ROOT',Path(__file__).resolve().parents[1]))
SOURCE=Path(os.environ.get('FIRMWARE_SESSION_SOURCE',ROOT/'dongle/components/dual_end_audio/dual_end_audio.c'))


def test_rejected_begin_preserves_current_session_and_nack_owner(tmp_path):
    receiver=SOURCE.read_text(encoding='utf-8')
    (tmp_path/'esp_err.h').write_text('#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n',encoding='utf-8')
    code=r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "audio_rx_telemetry.h"
#include "audio_feedback_mailbox.h"
static audio_recovery_rx_t s_rx_recovery;
static audio_rx_event_t s_nack_event;
static dual_end_audio_nack_packet_t s_nack_packet;
static dual_end_nack_mailbox_t mailbox;
static bool s_nack_pending,s_hid_sequence_valid,s_last_frame_valid;
static uint8_t s_rx_codec=UINT8_MAX;
static uint32_t s_rx_session_generation,s_invalid_frames,s_feedback_queue_failures,s_feedback_sequence;
static unsigned g722_closes,opus_resets,callbacks,begins,acks,cancellations,publications;
static uint32_t expected_session;
static int16_t s_decode_channel_pcm[DUAL_END_AUDIO_CHANNELS][DUAL_END_AUDIO_FRAME_SAMPLES];
static int16_t s_decoded_interleaved_pcm[DUAL_END_AUDIO_FRAME_SAMPLE_VALUES];
static int16_t s_last_frame_pcm[DUAL_END_AUDIO_FRAME_SAMPLE_VALUES];
static void *s_g722_decoder[DUAL_END_AUDIO_CHANNELS];
static dual_end_audio_session_packet_t last_ack;
static int esp_g722_dec_close(void *p){assert(p);++g722_closes;return 0;}
static void dual_end_opus_decoder_reset(void){assert(!s_nack_pending&&!mailbox.pending);++opus_resets;}
static void session_reset(void *ctx){
    assert(ctx==&mailbox&&!s_nack_pending&&!mailbox.pending);
    assert(s_rx_recovery.active&&s_rx_recovery.session_id==expected_session);
    assert(s_rx_codec==UINT8_MAX&&!s_hid_sequence_valid&&!s_last_frame_valid);
    for(unsigned i=0;i<DUAL_END_AUDIO_FRAME_SAMPLE_VALUES;++i)assert(!s_last_frame_pcm[i]);
    ++callbacks;
}
static void (*s_session_reset_cb)(void*)=session_reset;
static void *s_session_reset_ctx=&mailbox;
static bool dual_end_espnow_link_cancel_audio_nack(uint32_t token){
    bool cancelled=dual_end_nack_mailbox_cancel(&mailbox,token);
    if(cancelled)++cancellations;
    return cancelled;
}
static esp_err_t dual_end_espnow_link_publish_audio_nack(const dual_end_nack_snapshot_t *snapshot){
    assert(dual_end_nack_mailbox_publish(&mailbox,snapshot));++publications;return ESP_OK;
}
static esp_err_t dual_end_espnow_link_queue_audio_feedback(const void *data,int len){
    assert(len==(int)sizeof(last_ack));memcpy(&last_ack,data,sizeof(last_ack));
    assert(last_ack.command==DUAL_END_AUDIO_SESSION_ACK);
    assert(dual_end_audio_recovery_header_valid(&last_ack.packet,sizeof(last_ack),
        SCEPTER_MSG_AUDIO_SESSION,SCEPTER_ROLE_DONGLE));
    ++acks;return ESP_OK;
}
void audio_rx_telemetry_event(audio_rx_telemetry_event_kind_t kind,
    uint32_t a,uint32_t b,uint32_t c,uint32_t d,uint32_t e){
    assert(kind==AUDIO_RX_LOG_SESSION_BEGIN&&a==expected_session&&b==s_rx_recovery.first_sequence);
    assert(!c&&!d&&!e);++begins;
}
''' + '\n'.join(function(receiver,name) for name in (
        'increment_nonzero','reset_g722_decoder','feedback_header',
        'send_session_feedback','update_pending_nack','begin_audio_session')) + r'''
static void begin(uint32_t id,uint32_t sequence,int64_t now){
    const dual_end_audio_session_packet_t packet={.session_id=id,.first_sequence=sequence};
    expected_session=id;begin_audio_session(&packet,now);
}
static void history(void){
    s_rx_codec=DUAL_END_AUDIO_CODEC_G722;s_hid_sequence_valid=s_last_frame_valid=true;
    memset(s_last_frame_pcm,0x12,sizeof(s_last_frame_pcm));
    s_g722_decoder[0]=&g722_closes;s_g722_decoder[1]=&g722_closes;
}
static void pending_gap(uint32_t first,int64_t now){
    uint8_t encoded[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES]={0};audio_rx_event_t event;
    assert(audio_recovery_rx_push(&s_rx_recovery,s_rx_recovery.session_id,first,encoded,-50,false,now)==AUDIO_RX_ACCEPTED);
    assert(audio_recovery_rx_next(&s_rx_recovery,now+1,&event)&&event.kind==AUDIO_RX_FRAME);
    assert(audio_recovery_rx_push(&s_rx_recovery,s_rx_recovery.session_id,first+2,encoded,-50,false,now+100)==AUDIO_RX_ACCEPTED);
    assert(audio_recovery_rx_next(&s_rx_recovery,now+5000,&event)&&event.kind==AUDIO_RX_NACK);
    s_nack_event=event;s_nack_pending=true;
    s_nack_packet=(dual_end_audio_nack_packet_t){.version=DUAL_END_AUDIO_RECOVERY_VERSION,
        .session_id=event.session_id,.base_sequence=event.sequence,.missing_mask=event.missing_mask};
    update_pending_nack(now+5000);
    assert(mailbox.pending&&s_rx_recovery.nack_pending);
}
static void reject_unchanged(uint32_t id,uint32_t sequence,int64_t now){
    audio_recovery_rx_t recovery_before;dual_end_nack_mailbox_t mailbox_before;
    memcpy(&recovery_before,&s_rx_recovery,sizeof(recovery_before));
    memcpy(&mailbox_before,&mailbox,sizeof(mailbox_before));
    const unsigned cb=callbacks,op=opus_resets,cl=g722_closes,ack=acks,cancel=cancellations,pub=publications,logs=begins;
    const uint32_t generation=s_rx_session_generation,invalid=s_invalid_frames;
    begin(id,sequence,now);
    assert(s_rx_recovery.active&&s_rx_recovery.session_id==recovery_before.session_id);
    assert(!memcmp(&recovery_before,&s_rx_recovery,sizeof(recovery_before)));
    assert(!memcmp(&mailbox_before,&mailbox,sizeof(mailbox_before)));
    assert(s_invalid_frames==invalid+1&&s_rx_session_generation==generation);
    assert(callbacks==cb&&opus_resets==op&&g722_closes==cl&&acks==ack&&cancellations==cancel&&publications==pub&&begins==logs);
    assert(s_last_frame_valid&&s_hid_sequence_valid&&s_rx_codec==DUAL_END_AUDIO_CODEC_G722);
    assert(s_nack_pending&&s_nack_event.nack_token==s_rx_recovery.nack_token);
}
int main(void){
    audio_recovery_rx_init(&s_rx_recovery);
    begin(111,100,1000);assert(callbacks==1&&acks==1&&s_rx_session_generation==1);
    history();begin(222,200,2000);
    assert(callbacks==2&&acks==2&&s_rx_session_generation==2&&g722_closes==2);
    history();pending_gap(200,2100);
    reject_unchanged(111,100,7101); /* A -> B -> delayed A. */
    reject_unchanged(222,201,7102); /* Same ID with a conflicting BEGIN. */
    reject_unchanged(0,100,7103);   /* Invalid identity is likewise side-effect free. */
    begin(333,300,8000);
    assert(callbacks==3&&begins==3&&acks==3&&cancellations==1&&s_rx_session_generation==3);
    assert(s_rx_recovery.next_nack_admission_us==8000&&!s_rx_recovery.nack_pending);
    assert(!s_nack_pending&&!mailbox.pending&&g722_closes==4&&opus_resets==3);
    assert(last_ack.session_id==333&&last_ack.first_sequence==300);
    history();pending_gap(300,8100);
    dual_end_nack_snapshot_t claimed;
    assert(dual_end_nack_mailbox_claim(&mailbox,&claimed,1));
    const uint32_t old_token=claimed.token;
    begin(444,400,14000);
    assert(callbacks==4&&acks==4&&cancellations==1&&!s_nack_pending);
    assert(s_rx_recovery.next_nack_admission_us==14000);
    pending_gap(400,14100);
    assert(s_rx_recovery.nack_token!=old_token);
    const uint32_t new_token=s_rx_recovery.nack_token;
    const int64_t admission=s_rx_recovery.next_nack_admission_us;
    audio_recovery_rx_finish_nack(&s_rx_recovery,old_token,AUDIO_RX_NACK_ACCEPTED,19200);
    assert(s_rx_recovery.nack_pending&&s_rx_recovery.nack_token==new_token);
    assert(s_rx_recovery.next_nack_admission_us==admission&&mailbox.pending);
    begin(444,400,19201); /* An exact retransmitted BEGIN still gets its ACK. */
    assert(callbacks==4&&begins==4&&acks==5&&s_rx_session_generation==4);
    assert(s_rx_recovery.active&&s_rx_recovery.nack_token==new_token&&mailbox.pending);
    puts("real BEGIN A/B/late-A isolation; queued/claimed NACK retirement; decoder/UAC reset PASS");
    return 0;
}
'''
    harness=tmp_path/'session_lifecycle.c'
    harness.write_text(code,encoding='utf-8')
    executable,env=_build_host_executable(
        [harness,ROOT/'components/dual_end_audio_recovery/audio_recovery_rx.c'],tmp_path,
        extra_includes=(tmp_path,ROOT/'dongle/components/dual_end_audio/include',
                        ROOT/'dongle/components/dual_end_espnow_link/include'))
    assert 'PASS' in _run([str(executable)],cwd=tmp_path,env=env).stdout
