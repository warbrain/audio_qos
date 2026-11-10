"""Execute actual CDC byte framing; no USB or device commands are performed."""
import os
from pathlib import Path
import subprocess

import pytest
from test_audio_recovery_042 import _build_host_executable
from test_runtime_codec import function

ROOT=Path(__file__).resolve().parents[1]
SOURCE=Path(os.environ.get("FIRMWARE_CDC_SOURCE",ROOT/"dongle/components/dongle_usb_device/dongle_usb_device.c"))


@pytest.fixture(scope="module")
def cdc_framing(tmp_path_factory):
    folder=tmp_path_factory.mktemp("cdc_text_frames")
    source=SOURCE.read_text(encoding="utf-8")
    prefix=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#define CDC_OTA_IDLE 0
#define CDC_OTA_DONGLE 1
#define CDC_OTA_SCEPTER 2
static char s_cmd_line[128];
static size_t s_cmd_line_len;
static bool s_cmd_line_overflow,s_cmd_line_invalid;
static int s_ota_mode,s_cdc_rx_lock;
static bool s_ota2_receiving_chunk,s_cdc_disconnect_pending,s_cdc_rx_overflow_pending;
static uint32_t s_cdc_epoch=1,s_cdc_processing_epoch=1,s_cdc_rx_commands,s_ota_session=3;
static int64_t s_ota_last_rx_us;
static unsigned locks,commands,errors,aborts;
static char last_command[128],last_error[128];
static uint8_t binary[32];
static size_t binary_used,binary_remaining;
#define portENTER_CRITICAL(lock) ((void)(lock),assert(!locks),++locks)
#define portEXIT_CRITICAL(lock) ((void)(lock),assert(locks==1),--locks)
static int64_t esp_timer_get_time(void){return 1000;}
static bool cdc_ota_epoch_valid(void){return !s_cdc_disconnect_pending && s_cdc_processing_epoch==s_cdc_epoch;}
static void cdc_write(int itf,const char *text){assert(itf==0);++errors;snprintf(last_error,sizeof(last_error),"%s",text);}
static void cdc_process_command(int itf,char *line){assert(itf==0);++commands;snprintf(last_command,sizeof(last_command),"%s",line);}
static size_t cdc_ota2_chunk_process(int itf,const uint8_t *data,size_t n){
 assert(itf==0&&binary_remaining);size_t take=n<binary_remaining?n:binary_remaining;
 assert(binary_used+take<=sizeof(binary));memcpy(binary+binary_used,data,take);
 binary_used+=take;binary_remaining-=take;if(!binary_remaining)s_ota2_receiving_chunk=false;return take;}
static int dual_end_ota_send_scepter_abort(uint32_t session){assert(session==3);return 0;}
static void dual_end_ota_self_abort(void){++aborts;}
static void cdc_reset_ota_state(void){s_ota_mode=CDC_OTA_IDLE;s_ota2_receiving_chunk=false;}
'''
    reset=function(source,"cdc_reset_command_line") if "static void cdc_reset_command_line(" in source else r'''
static void cdc_reset_command_line(void){s_cmd_line_len=0;s_cmd_line_overflow=false;s_cmd_line_invalid=false;}
'''
    code=prefix+reset+function(source,"cdc_abort_ota_stream")+function(source,"cdc_process_rx")+function(source,"cdc_handle_rx_events")+r'''
static void send_bytes(const uint8_t *data,size_t size){cdc_process_rx(0,data,size);assert(!locks);}
static void send_text(const char *text){send_bytes((const uint8_t *)text,strlen(text));}
static void setup(void){cdc_reset_command_line();commands=errors=aborts=0;s_cdc_rx_commands=0;
 s_ota_mode=CDC_OTA_IDLE;s_ota2_receiving_chunk=false;s_cdc_disconnect_pending=false;s_cdc_rx_overflow_pending=false;
 s_cdc_processing_epoch=s_cdc_epoch=1;last_command[0]=last_error[0]=0;binary_used=binary_remaining=0;}
int main(int argc,char **argv){
 assert(argc==2);setup();
 if(!strcmp(argv[1],"invalid")){
  const uint8_t bad[]="DIAG_CONTEXT 1 10 20\0junk\nPING\r\n";
  send_bytes(bad,sizeof(bad)-1);
  assert(commands==1&&!strcmp(last_command,"PING")&&errors==1);
  assert(strstr(last_error,"invalid_character")&&s_cdc_rx_commands==2);
 }else if(!strcmp(argv[1],"fragmented")){
  const uint8_t bad[]={0,'j','u','n','k'};send_text("UI DONE");send_bytes(bad,sizeof(bad));
  send_text("ERASE DONGLE 1");assert(commands==0&&errors==0);send_text("\nPING\n");
  assert(errors==1&&commands==1&&!strcmp(last_command,"PING"));
 }else if(!strcmp(argv[1],"overflow")){
  uint8_t bytes[160];memset(bytes,'a',sizeof(bytes));send_bytes(bytes,sizeof(bytes));
  send_text("\nPING\n");assert(errors==1&&strstr(last_error,"line_too_long")&&commands==1);
  setup();send_bytes(bytes,sizeof(bytes));const uint8_t zero=0;send_bytes(&zero,1);send_text("\nPING\n");
  assert(errors==1&&strstr(last_error,"invalid_character")&&commands==1);
 }else if(!strcmp(argv[1],"binary")){
  const uint8_t raw[]={0,'\n','\r',27,255,'A'};s_ota_mode=CDC_OTA_DONGLE;s_ota2_receiving_chunk=true;binary_remaining=sizeof(raw);
  send_bytes(raw,2);send_bytes(raw+2,sizeof(raw)-2);assert(binary_used==sizeof(raw)&&!memcmp(binary,raw,sizeof(raw)));
  assert(!commands&&!errors&&!s_cdc_rx_commands);send_text("PING\n");assert(commands==1&&!errors);
 }else if(!strcmp(argv[1],"reset")){
  const uint8_t bad[]="PING\0junk";send_bytes(bad,sizeof(bad)-1);
  s_cdc_disconnect_pending=true;cdc_handle_rx_events(0);send_text("PING\n");assert(commands==1&&!errors&&aborts==1);
  setup();send_bytes(bad,sizeof(bad)-1);cdc_abort_ota_stream();send_text("PING\n");assert(commands==1&&!errors);
  setup();send_bytes(bad,sizeof(bad)-1);s_cdc_processing_epoch=0;cdc_abort_ota_stream();
  s_cdc_processing_epoch=1;send_text("\n");assert(!commands&&errors==1);
 }else assert(false);
 puts("PASS CDC text framing");return 0;
}
'''
    harness=folder/"cdc_text_frames.c"
    harness.write_text(code,encoding="utf-8")
    executable,env=_build_host_executable([harness],folder)
    return executable,env,folder


@pytest.mark.parametrize("scenario",["invalid","fragmented","overflow","binary","reset"])
def test_actual_text_frames_reject_nul_and_preserve_binary_and_reset(cdc_framing,scenario):
    executable,env,folder=cdc_framing
    result=subprocess.run([str(executable),scenario],cwd=folder,env=env,text=True,capture_output=True,timeout=10)
    assert result.returncode==0,result.stdout+result.stderr
    assert "PASS CDC text framing" in result.stdout
