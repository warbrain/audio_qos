"""Run the production OTA state machine against a deterministic host platform.

The radio, flash and RTOS boundaries are stubbed; production packet validation,
transfer ordering, duplicate handling and finalize/error code are compiled as C.
Run with: python -m unittest discover -s scepter/components/dual_end_ota/tests -v
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

COMPONENT = Path(__file__).resolve().parents[1]


def production_function(text, name, return_type="void"):
    start = text.index("static " + return_type + " " + name + "(")
    pos = text.index("{", start)
    depth = 1
    end = pos + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]

PLATFORM = r'''
#pragma once
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
static void mbedtls_platform_zeroize(void *buffer,size_t size){
 volatile unsigned char *p=(volatile unsigned char*)buffer;while(size--)*p++=0;
}
#ifdef _MSC_VER
#define __attribute__(x)
#endif
typedef int esp_err_t;
enum { ESP_OK=0, ESP_FAIL=-1, ESP_ERR_INVALID_ARG=1, ESP_ERR_INVALID_STATE,
 ESP_ERR_INVALID_SIZE, ESP_ERR_INVALID_CRC, ESP_ERR_INVALID_RESPONSE, ESP_ERR_NOT_FOUND,
 ESP_ERR_NO_MEM, ESP_ERR_TIMEOUT, ESP_ERR_ESPNOW_NO_MEM, ESP_ERR_NVS_NOT_FOUND,
 ESP_ERR_OTA_ROLLBACK_INVALID_STATE };
#define ESP_RETURN_ON_FALSE(a,b,...) do { if(!(a)) return (b); } while(0)
#define ESP_RETURN_ON_ERROR(a,...) do { int r_=(a); if(r_!=0) return r_; } while(0)
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_ERROR_CHECK(x) assert((x)==0)
static const char *esp_err_to_name(int x) { (void)x; return "error"; }
typedef enum {SCEPTER_ROLE_SCEPTER=1, SCEPTER_ROLE_DONGLE=2} scepter_role_t;
static const char *scepter_role_name(scepter_role_t r) { return r==1?"scepter":"dongle"; }
#define DUAL_END_OTA_IMAGE_KEY_PREFIX_APP "app-"
#define DUAL_END_OTA_IMAGE_KEY_PREFIX_FACTORY "factory-"
#define DUAL_END_OTA_APP_KEY "app"
#define DUAL_END_OTA_FACTORY_KEY "factory"
typedef int BaseType_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
#define pdTRUE 1
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
#define portMAX_DELAY UINT32_MAX
typedef struct {int count;} test_sem;
typedef test_sem *SemaphoreHandle_t;
typedef void *QueueHandle_t;
static void (*wait_hook)(void);
static int xSemaphoreTake(SemaphoreHandle_t s, unsigned ticks) {
 if(!s)return 0; if(!s->count && ticks && wait_hook) wait_hook();
 if(!s->count)return 0; --s->count; return 1;
}
static int xSemaphoreGive(SemaphoreHandle_t s) { s->count=1; return 1; }
static SemaphoreHandle_t xSemaphoreCreateBinary(void) { return calloc(1,sizeof(test_sem)); }
static QueueHandle_t xQueueCreate(unsigned n,unsigned size) {(void)n;(void)size;return (void*)1;}
static int xQueueSend(QueueHandle_t q,const void *v,unsigned ticks){(void)q;(void)v;(void)ticks;return 1;}
static int xQueueReceive(QueueHandle_t q,void *v,unsigned ticks){(void)q;(void)v;(void)ticks;return 0;}
static void vQueueDelete(QueueHandle_t q){(void)q;}
static int xTaskCreate(void(*fn)(void*),const char*n,unsigned st,void*a,unsigned p,void*t){(void)fn;(void)n;(void)st;(void)a;(void)p;(void)t;return 1;}
static unsigned event_order,last_write_order,last_delay_order,last_send_order;
static unsigned delay_count,last_delay_ticks;
static void vTaskDelay(unsigned x){++delay_count;last_delay_ticks=x;last_delay_order=++event_order;}
static int64_t esp_timer_get_time(void){return 1000000;}
static int restarts;
static void esp_restart(void){++restarts;}
static int dual_end_espnow_link_is_connected(void){return 1;}
static int test_send(const void *data,int len);
#define dual_end_espnow_link_send_ota test_send
static uint32_t esp_crc32_le(uint32_t crc,const uint8_t*p,size_t n){crc=~crc;while(n--){crc^=*p++;for(int i=0;i<8;++i)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}return ~crc;}
static uint16_t esp_crc16_le(uint16_t crc,const uint8_t*p,size_t n){while(n--){crc^=*p++;for(int i=0;i<8;++i)crc=(crc>>1)^((crc&1)?0xa001:0);}return crc;}
typedef struct {int type,subtype;uint32_t address,size;char label[16];} esp_partition_t;
#define ESP_PARTITION_TYPE_APP 0
#define ESP_PARTITION_SUBTYPE_ANY 255
#define ESP_PARTITION_SUBTYPE_APP_FACTORY 0
#define OTA_WITH_SEQUENTIAL_WRITES 0xfffffffe
static esp_partition_t running={0,1,0x200000,0x200000,"ota_0"};
static esp_partition_t staging={0,2,0x400000,0x200000,"ota_1"};
static const esp_partition_t *boot=&running;
static const esp_partition_t *esp_ota_get_running_partition(void){return &running;}
static const esp_partition_t *esp_ota_get_boot_partition(void){return boot;}
static const esp_partition_t *esp_ota_get_next_update_partition(const esp_partition_t*p){(void)p;return &staging;}
static const esp_partition_t *esp_partition_find_first(int t,int sub,const char*label){(void)t;(void)sub;return strcmp(label,staging.label)==0?&staging:&running;}
typedef struct {char project_name[32];char version[32];} esp_app_desc_t;
static esp_app_desc_t app={"cytoscepter_scepter","app-test"};
static const esp_app_desc_t *esp_app_get_description(void){return &app;}
static int invalid_project;
static int esp_ota_get_partition_description(const esp_partition_t*p,esp_app_desc_t*d){(void)p;*d=app;if(invalid_project)strcpy(d->project_name,"wrong");return 0;}
typedef unsigned esp_ota_handle_t;
static int writes,aborts,ends,begin_calls,ota_write_error,ota_end_error,boot_error;
static int esp_ota_begin(const esp_partition_t*p,uint32_t sz,esp_ota_handle_t*h){(void)p;assert(sz==OTA_WITH_SEQUENTIAL_WRITES);++begin_calls;*h=1;return 0;}
static int esp_ota_write(esp_ota_handle_t h,const void*p,size_t n){assert(h==1&&p&&n);++writes;last_write_order=++event_order;return ota_write_error;}
static int esp_ota_end(esp_ota_handle_t h){assert(h==1);++ends;return ota_end_error;}
static int esp_ota_abort(esp_ota_handle_t h){assert(h==1);++aborts;return 0;}
static int esp_ota_set_boot_partition(const esp_partition_t*p){if(boot_error)return boot_error;boot=p;return 0;}
static int esp_partition_erase_range(const esp_partition_t*p,unsigned o,unsigned n){(void)p;(void)o;(void)n;return 0;}
#define CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE 0
#define ESP_MAC_WIFI_STA 0
static int esp_read_mac(uint8_t*p,int t){(void)t;memset(p,2,6);return 0;}
typedef struct{int baud_rate,data_bits,parity,stop_bits,flow_ctrl,source_clk;}uart_config_t;
#define UART_NUM_0 0
#define UART_DATA_8_BITS 0
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 0
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0
static int uart_param_config(int n,const uart_config_t*c){(void)n;(void)c;return 0;}
static int uart_driver_install(int a,int b,int c,int d,void*e,int f){(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;return 0;}
static int uart_write_bytes(int n,const char*s,size_t l){(void)n;(void)s;return (int)l;}
static void esp_log_set_vprintf(int(*f)(const char*,va_list)){(void)f;}
typedef int nvs_handle_t;
#define NVS_READWRITE 1
#define NVS_READONLY 0
static unsigned char stored_result[256];static size_t stored_len;static int nvs_error;
static int nvs_open(const char*n,int mode,nvs_handle_t*h){(void)n;(void)mode;*h=1;return nvs_error;}
static int nvs_set_blob(nvs_handle_t h,const char*k,const void*p,size_t n){(void)h;(void)k;assert(n<=sizeof(stored_result));memcpy(stored_result,p,n);stored_len=n;return 0;}
static int nvs_get_blob(nvs_handle_t h,const char*k,void*p,size_t*n){(void)h;(void)k;if(!stored_len)return ESP_ERR_NVS_NOT_FOUND;memcpy(p,stored_result,stored_len);*n=stored_len;return 0;}
static int nvs_erase_key(nvs_handle_t h,const char*k){(void)h;(void)k;stored_len=0;return 0;}
static int nvs_commit(nvs_handle_t h){(void)h;return 0;}
static void nvs_close(nvs_handle_t h){(void)h;}
'''

CASES = r'''
static dual_end_ota_packet_t last_sent;
static int sends,send_error;
static bool activity;
static void note_activity(bool active,void*ctx){(void)ctx;activity=active;}
static int test_send(const void *p,int len){assert(len>0);last_sent=*(const dual_end_ota_packet_t*)p;++sends;last_send_order=++event_order;return send_error;}
static void reset(void){
 dual_end_ota_self_abort(); boot=&running;writes=aborts=ends=begin_calls=restarts=0;
 ota_write_error=ota_end_error=boot_error=invalid_project=nvs_error=send_error=0;stored_len=0;
 event_order=last_write_order=last_delay_order=last_send_order=delay_count=last_delay_ticks=0;
 s_relay_active=false;s_boot_confirmed=true;s_service_role=SCEPTER_ROLE_SCEPTER;
 s_wireless_session=false;activity=false;wait_hook=NULL;sends=0;
}
static dual_end_ota_packet_t begin_packet(void){
 dual_end_ota_packet_t p={0};p.magic=DUAL_END_OTA_MAGIC;p.version=DUAL_END_OTA_VERSION;
 p.msg_type=DUAL_END_OTA_MSG_BEGIN;p.target=DUAL_END_OTA_TARGET_SCEPTER;p.session_id=71;
 p.image_size=3;p.image_crc32=dual_end_ota_crc32_update(0,(const uint8_t*)"abc",3);return p;
}
static dual_end_ota_packet_t data_packet(void){dual_end_ota_packet_t p=begin_packet();p.msg_type=DUAL_END_OTA_MSG_DATA;p.payload_len=3;memcpy(p.payload,"abc",3);return p;}
static dual_end_ota_ack_payload_t ack(void){dual_end_ota_ack_payload_t a;memcpy(&a,last_sent.payload,sizeof(a));return a;}
static void boot_result_on_wait(void){
 dual_end_ota_packet_t p={0};p.magic=DUAL_END_OTA_MAGIC;p.version=DUAL_END_OTA_VERSION;
 p.msg_type=DUAL_END_OTA_MSG_RESULT;p.target=DUAL_END_OTA_TARGET_DONGLE;p.session_id=71;
 p.image_size=3;p.image_crc32=s_relay_image_crc;p.payload_len=sizeof(dual_end_ota_result_payload_t);
 dual_end_ota_result_payload_t r={0};r.version=DUAL_END_OTA_RESULT_VERSION;
 r.status=DUAL_END_OTA_RESULT_INSTALL_CONFIRMED;r.stage=DUAL_END_OTA_RESULT_STAGE_BOOT_CONFIRM;
 strcpy(r.app_version,"app-test");strcpy(r.partition,"ota_1");memcpy(p.payload,&r,sizeof(r));
 dual_end_ota_packet_set_crc(&p);assert(dual_end_ota_handle_scepter_espnow_packet((void*)&p,32+p.payload_len));
 wait_hook=NULL;
}
static void rebooted_end_rejection(void){
 dual_end_ota_packet_t p=s_expected_ack;p.msg_type=DUAL_END_OTA_MSG_ACK;
 p.target=DUAL_END_OTA_TARGET_DONGLE;p.reserved=DUAL_END_OTA_TARGET_SCEPTER;
 dual_end_ota_ack_payload_t a={0};a.version=DUAL_END_OTA_ACK_VERSION;
 a.acked_msg_type=DUAL_END_OTA_MSG_END;a.status=DUAL_END_OTA_ACK_REJECTED;a.error_code=ESP_ERR_INVALID_STATE;
 p.payload_len=sizeof(a);memcpy(p.payload,&a,sizeof(a));dual_end_ota_packet_set_crc(&p);
 assert(dual_end_ota_handle_scepter_espnow_packet((void*)&p,32+p.payload_len));
 wait_hook=boot_result_on_wait;
}
int main(void){
 dual_end_ota_register_activity_cb(note_activity,NULL);
 reset();dual_end_ota_packet_t b=begin_packet(),d=data_packet();
 process_wireless_packet(&b);assert(begin_calls==1&&s_status.active&&activity&&ack().status==1);
 process_wireless_packet(&b);assert(begin_calls==1&&ack().duplicate);
 assert(delay_count==0);
 process_wireless_packet(&d);assert(writes==1&&s_status.bytes_written==3&&ack().received==3);
 assert(delay_count==1&&last_delay_ticks>=2&&last_write_order<last_delay_order&&last_delay_order<last_send_order);
 process_wireless_packet(&d);assert(writes==1&&ack().duplicate);
 assert(delay_count==2&&last_delay_ticks>=2&&last_delay_order<last_send_order);
 d.image_crc32++;process_wireless_packet(&d);assert(ack().status==3&&writes==1&&s_status.active);
 d=data_packet();d.offset=1;process_wireless_packet(&d);assert(ack().status==3&&writes==1);
 assert(delay_count==2);
 dual_end_ota_packet_t e=b;e.msg_type=DUAL_END_OTA_MSG_END;e.sequence=1;e.offset=3;
 process_wireless_packet(&e);assert(restarts==1&&ends==1&&boot==&staging&&stored_len>0&&activity);
 puts("duplicate BEGIN/DATA, wrong metadata/offset, validated commit: passed");
 reset();b=begin_packet();process_wireless_packet(&b);d=data_packet();ota_write_error=ESP_FAIL;
 process_wireless_packet(&d);assert(ack().status==3&&writes==1&&!s_status.active&&delay_count==0);
 puts("accepted DATA yields after write and before ACK; rejected DATA does not: passed");
 reset();b=begin_packet();process_wireless_packet(&b);e=b;e.msg_type=DUAL_END_OTA_MSG_END;
 process_wireless_packet(&e);assert(!s_status.active&&!activity&&boot==&running&&aborts==1);
 reset();process_wireless_packet(&b);d=data_packet();process_wireless_packet(&d);ota_end_error=ESP_FAIL;
 e=b;e.msg_type=DUAL_END_OTA_MSG_END;e.sequence=1;e.offset=3;process_wireless_packet(&e);
 assert(!s_status.active&&s_ota_handle==0&&!activity&&boot==&running&&aborts==0);
 reset();process_wireless_packet(&b);process_wireless_packet(&d);invalid_project=1;process_wireless_packet(&e);
 assert(!s_status.active&&s_ota_handle==0&&boot==&running&&stored_len==0);
 reset();process_wireless_packet(&b);process_wireless_packet(&d);boot_error=ESP_FAIL;process_wireless_packet(&e);
 assert(!s_status.active&&boot==&running&&stored_len==0);
 puts("short transfer, esp_ota_end, image validation and boot commit failure cleanup: passed");
 reset();boot=&staging;assert(dual_end_ota_self_begin(DUAL_END_OTA_TARGET_SCEPTER,71,3,0)==ESP_ERR_INVALID_STATE);
 assert(dual_end_ota_erase_partition("ota_1")==ESP_ERR_INVALID_STATE);
 assert(begin_calls==0);boot=&running;assert(dual_end_ota_self_begin(DUAL_END_OTA_TARGET_SCEPTER,71,0,0)==ESP_ERR_INVALID_ARG);
 d=data_packet();dual_end_ota_packet_set_crc(&d);assert(dual_end_ota_packet_valid(&d,35));
 assert(!dual_end_ota_packet_valid(&d,34)&&!dual_end_ota_packet_valid(&d,36));
 assert(!dual_end_ota_handle_scepter_espnow_packet((void*)&d,-1));
 puts("safe boot partition and exact wire size: passed");
 reset();s_service_role=SCEPTER_ROLE_DONGLE;s_ack_sem=xSemaphoreCreateBinary();s_result_sem=xSemaphoreCreateBinary();
 s_relay_active=true;s_relay_image_size=3;s_relay_image_crc=b.image_crc32;
 assert(dual_end_ota_prepare_scepter_result(71)==ESP_OK);wait_hook=boot_result_on_wait;
 assert(send_packet_with_ack(&e,5000)==ESP_OK);assert(sends==1);
 dual_end_ota_result_t result;assert(dual_end_ota_wait_scepter_result(1000,&result)==ESP_OK);
 assert(!s_relay_active&&!activity&&result.session_id==71);
 puts("lost END ACK with matching boot RESULT: passed");
 reset();s_service_role=SCEPTER_ROLE_DONGLE;s_relay_active=true;s_relay_image_size=3;s_relay_image_crc=b.image_crc32;
 assert(dual_end_ota_prepare_scepter_result(71)==ESP_OK);wait_hook=rebooted_end_rejection;
 assert(send_packet_with_ack(&e,5000)==ESP_ERR_INVALID_STATE);
 assert(dual_end_ota_wait_scepter_result(1000,&result)==ESP_OK);
 puts("rebooted receiver rejects retried END before later boot RESULT: passed");
 reset();ota_pending_result_t pending={0};send_error=ESP_FAIL;
 assert(!send_boot_result(&pending));send_error=ESP_OK;assert(send_boot_result(&pending));
 puts("failed boot result remains retryable: passed");
 return 0;
}
'''

PAIRING_PLATFORM = r'''
#include <setjmp.h>
#define ESP_PLATFORM 1
#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#endif
#define CONFIG_SECURE_FLASH_ENC_ENABLED 0
#define DUAL_END_PAIRING_KEY_BYTES 16u
#define DUAL_END_PAIRING_KEY_ID_BYTES 8u
typedef enum {
 DUAL_END_PAIRING_SECURITY_NONE=0,
 DUAL_END_PAIRING_SECURITY_LEGACY_PLAIN=1,
 DUAL_END_PAIRING_SECURITY_ENCRYPTED=2,
} dual_end_pairing_security_t;
typedef enum {
 DUAL_END_PAIRING_PENDING_NONE=0,
 DUAL_END_PAIRING_PENDING_PREPARED=1,
 DUAL_END_PAIRING_PENDING_COMMITTED=2,
} dual_end_pairing_pending_state_t;
typedef struct {
 bool paired,from_nvs;dual_end_pairing_security_t security;uint8_t peer_mac[6];
 uint32_t generation;uint8_t key_id[DUAL_END_PAIRING_KEY_ID_BYTES];
 dual_end_pairing_pending_state_t pending_state;uint32_t pending_transaction_id;
} dual_end_pairing_status_t;
typedef struct {unsigned tx_buffer_size,rx_buffer_size;} usb_serial_jtag_driver_config_t;
typedef int esp_log_level_t;
#define ESP_LOG_NONE 0
static int esp_log_get_default_level(void){return 3;}
static void esp_log_level_set(const char*tag,esp_log_level_t level){(void)tag;(void)level;}
static uint8_t usb_in[16384];static char usb_out[16384];
static size_t usb_in_len,usb_read_pos;static jmp_buf usb_done;
static bool native_driver_installed,native_vfs_uses_driver,usb_write_fail;
static int stdout_lock_depth;static size_t usb_write_limit;
static void test_flockfile(FILE *stream){assert(stream==stdout);assert(stdout_lock_depth==0);++stdout_lock_depth;}
static void test_funlockfile(FILE *stream){assert(stream==stdout);assert(stdout_lock_depth==1);--stdout_lock_depth;}
#define flockfile test_flockfile
#define funlockfile test_funlockfile
static int usb_serial_jtag_read_bytes(void*p,size_t n,unsigned timeout){
 (void)timeout;if(usb_read_pos>=usb_in_len)longjmp(usb_done,1);
 size_t available=usb_in_len-usb_read_pos;if(available>n)available=n;
 memcpy(p,usb_in+usb_read_pos,available);usb_read_pos+=available;return (int)available;
}
static int usb_serial_jtag_write_bytes(const void*p,size_t n,unsigned timeout){
 (void)timeout;assert(native_driver_installed);
 if(usb_write_fail)return 0;if(usb_write_limit&&n>usb_write_limit)n=usb_write_limit;
 size_t have=strlen(usb_out);assert(have+n<sizeof(usb_out));memcpy(usb_out+have,p,n);usb_out[have+n]=0;return (int)n;
}
static int usb_serial_jtag_driver_install(const usb_serial_jtag_driver_config_t*c){(void)c;native_driver_installed=true;return 0;}
static void usb_serial_jtag_vfs_use_driver(void){assert(native_driver_installed);native_vfs_uses_driver=true;}
static void usb_serial_jtag_vfs_use_nonblocking(void){native_vfs_uses_driver=false;}
static int usb_serial_jtag_driver_uninstall(void){native_driver_installed=false;return ESP_OK;}
static int usb_serial_jtag_wait_tx_done(unsigned ticks){(void)ticks;return ESP_OK;}
static int set_calls,clear_calls,prepare_calls,commit_calls,confirm_calls,rollback_calls;
static int clear_error,prepare_error,commit_error,confirm_error,rollback_error;
static bool ota_active,relay_active,candidate_valid;
static uint8_t ota_received[8192];static size_t ota_received_len;
static int ota_begin_calls,ota_finish_calls,ota_abort_calls;
static uint32_t ota_session,ota_image_size,ota_image_crc;
static dual_end_pairing_status_t pairing_status,previous_status;
static uint8_t candidate_peer[6],candidate_lmk[DUAL_END_PAIRING_KEY_BYTES];
static uint8_t candidate_pmk[DUAL_END_PAIRING_KEY_BYTES],candidate_key_id[DUAL_END_PAIRING_KEY_ID_BYTES];
static uint32_t candidate_generation,candidate_transaction;
static int dual_end_espnow_link_get_peer(uint8_t*p,bool*from){(void)p;*from=false;return ESP_ERR_NOT_FOUND;}
static int dual_end_espnow_link_set_peer(const uint8_t*p,bool persist){assert(p&&persist);++set_calls;return ESP_FAIL;}
static int dual_end_espnow_link_clear_peer(bool persist){
 assert(persist);++clear_calls;if(clear_error)return clear_error;
 memset(&pairing_status,0,sizeof(pairing_status));candidate_valid=false;return ESP_OK;
}
static int dual_end_espnow_link_get_pairing_status(dual_end_pairing_status_t*out){assert(out);*out=pairing_status;return ESP_OK;}
static int dual_end_espnow_link_pair_prepare(const uint8_t*peer,const uint8_t*lmk,const uint8_t*pmk,
 uint32_t generation,uint32_t transaction,const uint8_t*key_id){
 assert(peer&&lmk&&pmk&&key_id);++prepare_calls;if(prepare_error)return prepare_error;
 previous_status=pairing_status;memcpy(candidate_peer,peer,6);memcpy(candidate_lmk,lmk,sizeof(candidate_lmk));
 memcpy(candidate_pmk,pmk,sizeof(candidate_pmk));memcpy(candidate_key_id,key_id,sizeof(candidate_key_id));
 candidate_generation=generation;candidate_transaction=transaction;candidate_valid=true;
 pairing_status.pending_state=DUAL_END_PAIRING_PENDING_PREPARED;
 pairing_status.pending_transaction_id=transaction;return ESP_OK;
}
static int dual_end_espnow_link_pair_commit(uint32_t transaction){
 ++commit_calls;if(commit_error)return commit_error;
 if(!candidate_valid||transaction!=candidate_transaction)return ESP_ERR_INVALID_STATE;
 pairing_status.paired=true;pairing_status.from_nvs=true;
 pairing_status.security=DUAL_END_PAIRING_SECURITY_ENCRYPTED;
 memcpy(pairing_status.peer_mac,candidate_peer,6);pairing_status.generation=candidate_generation;
 memcpy(pairing_status.key_id,candidate_key_id,sizeof(pairing_status.key_id));
 pairing_status.pending_state=DUAL_END_PAIRING_PENDING_COMMITTED;
 pairing_status.pending_transaction_id=transaction;return ESP_OK;
}
static int dual_end_espnow_link_pair_confirm(uint32_t transaction){
 ++confirm_calls;if(confirm_error)return confirm_error;
 if(!candidate_valid||transaction!=candidate_transaction||pairing_status.pending_state!=DUAL_END_PAIRING_PENDING_COMMITTED)return ESP_ERR_INVALID_STATE;
 pairing_status.pending_state=DUAL_END_PAIRING_PENDING_NONE;pairing_status.pending_transaction_id=0;
 candidate_valid=false;return ESP_OK;
}
static int dual_end_espnow_link_pair_rollback(uint32_t transaction){
 ++rollback_calls;if(rollback_error)return rollback_error;
 if(!candidate_valid||transaction!=candidate_transaction||pairing_status.pending_state==DUAL_END_PAIRING_PENDING_NONE)return ESP_ERR_INVALID_STATE;
 pairing_status=previous_status;pairing_status.pending_state=DUAL_END_PAIRING_PENDING_NONE;
 pairing_status.pending_transaction_id=0;candidate_valid=false;return ESP_OK;
}
'''

PAIRING_CASES = r'''
dual_end_ota_status_t dual_end_ota_get_status(void){dual_end_ota_status_t s={0};s.active=ota_active;s.target=DUAL_END_OTA_TARGET_SCEPTER;s.session_id=ota_session;s.image_size=ota_image_size;s.image_crc32=ota_image_crc;s.bytes_written=(uint32_t)ota_received_len;return s;}
bool dual_end_ota_relay_is_active(void){return relay_active;}
esp_err_t dual_end_ota_boot_confirmation_error(void){return boot_error;}
esp_err_t dual_end_ota_self_begin_partition(dual_end_ota_target_t target,uint32_t session,uint32_t size,uint32_t crc,const char*partition){
 assert(target==DUAL_END_OTA_TARGET_SCEPTER&&session&&size&&partition);++ota_begin_calls;ota_active=true;ota_received_len=0;ota_session=session;ota_image_size=size;ota_image_crc=crc;return ESP_OK;
}
esp_err_t dual_end_ota_self_write(const uint8_t*p,size_t n){assert(ota_active&&p&&n&&ota_received_len+n<=sizeof(ota_received));memcpy(ota_received+ota_received_len,p,n);ota_received_len+=n;return ESP_OK;}
esp_err_t dual_end_ota_self_finish(bool reboot_after){assert(ota_active&&!reboot_after);++ota_finish_calls;ota_active=false;return ESP_OK;}
void dual_end_ota_self_abort(void){++ota_abort_calls;ota_active=false;}
uint32_t dual_end_ota_crc32_update(uint32_t crc,const uint8_t*p,size_t n){return esp_crc32_le(crc,p,n);}
static int test_send(const void*p,int len){(void)p;(void)len;return 0;}
int main(void){
 const char*prepare42="PAIR PREPARE 04:12:34:56:78:9a 7 42 000102030405060708090a0b0c0d0e0f 101112131415161718191a1b1c1d1e1f a1a2a3a4a5a6a7a8";
 const char*prepare43="PAIR PREPARE 04:12:34:56:78:9b 8 43 202122232425262728292a2b2c2d2e2f 303132333435363738393a3b3c3d3e3f b1b2b3b4b5b6b7b8";
 char response[512];assert(dual_end_pairing_service_init(SCEPTER_ROLE_SCEPTER,true)==ESP_OK);
 assert(native_driver_installed);
 usb_write_limit=7;native_write_all("OK PAIR test_short_write\n");
 assert(strcmp(usb_out,"OK PAIR test_short_write\n")==0&&stdout_lock_depth==0);
 usb_write_fail=true;native_write_all("must_not_be_written\n");assert(stdout_lock_depth==0);
 usb_write_fail=false;usb_write_limit=0;usb_out[0]=0;
 assert(dual_end_pairing_handle_command("pair set 04:12:34:56:78:9a",response,sizeof(response)));
 assert(set_calls==0&&strstr(response,"secure_pairing_protocol_required"));
 dual_end_pairing_handle_command("PAIR INFO",response,sizeof(response));
 assert(strstr(response,"pair_protocol=2")&&strstr(response,"paired=0")&&strstr(response,"encryption=none"));
 assert(strstr(response,"key_storage=plaintext_flash")&&strstr(response,"pending=none")&&strstr(response,"pair_ready=1"));
 const char*invalid[]={
  "PAIR PREPARE 4:12:34:56:78:9a 7 42 000102030405060708090a0b0c0d0e0f 101112131415161718191a1b1c1d1e1f a1a2a3a4a5a6a7a8",
  "PAIR PREPARE 04:12:34:56:78:9z 7 42 000102030405060708090a0b0c0d0e0f 101112131415161718191a1b1c1d1e1f a1a2a3a4a5a6a7a8",
  "PAIR PREPARE 04:12:34:56:78:9a 0 42 000102030405060708090a0b0c0d0e0f 101112131415161718191a1b1c1d1e1f a1a2a3a4a5a6a7a8",
  "PAIR PREPARE 04:12:34:56:78:9a 7 0 000102030405060708090a0b0c0d0e0f 101112131415161718191a1b1c1d1e1f a1a2a3a4a5a6a7a8",
  "PAIR PREPARE 04:12:34:56:78:9a 7 42 000102030405060708090a0b0c0d0e0g 101112131415161718191a1b1c1d1e1f a1a2a3a4a5a6a7a8",
  "PAIR PREPARE 04:12:34:56:78:9a 7 42 000102030405060708090a0b0c0d0e0f 101112131415161718191a1b1c1d1e1f a1a2a3a4a5a6a7a8 extra"};
 for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);++i){
  assert(dual_end_pairing_handle_command(invalid[i],response,sizeof(response)));
  assert(strstr(response,"ERR PAIR PREPARE reason=invalid_fields"));
 }
 assert(prepare_calls==0);
 assert(dual_end_pairing_handle_command(prepare42,response,sizeof(response)));
 assert(prepare_calls==1&&strstr(response,"OK PAIR PREPARE")&&strstr(response,"txn=42"));
 assert(strstr(response,"generation=7")&&strstr(response,"key_id=a1a2a3a4a5a6a7a8"));
 assert(memcmp(candidate_peer,(uint8_t[]){4,0x12,0x34,0x56,0x78,0x9a},6)==0);
 assert(candidate_lmk[0]==0&&candidate_lmk[15]==0x0f&&candidate_pmk[0]==0x10&&candidate_pmk[15]==0x1f);
 assert(!dual_end_pairing_take_link_confirmation_pending());
 dual_end_pairing_handle_command("PAIR INFO",response,sizeof(response));
 assert(strstr(response,"paired=0")&&strstr(response,"pending=prepared")&&strstr(response,"txn=42"));
 ota_active=true;dual_end_pairing_handle_command(prepare43,response,sizeof(response));
 assert(prepare_calls==1&&strstr(response,"reason=ota_active"));
 dual_end_pairing_handle_command("PAIR COMMIT 42",response,sizeof(response));
 assert(commit_calls==0&&strstr(response,"reason=ota_active"));
 ota_active=false;commit_error=ESP_FAIL;dual_end_pairing_handle_command("PAIR COMMIT 42",response,sizeof(response));
 assert(commit_calls==1&&strstr(response,"ERR PAIR COMMIT")&&!dual_end_pairing_take_link_confirmation_pending());
 commit_error=0;dual_end_pairing_handle_command("PAIR COMMIT 42",response,sizeof(response));
 assert(commit_calls==2&&strstr(response,"OK PAIR COMMIT")&&dual_end_pairing_take_link_confirmation_pending());
 assert(!dual_end_pairing_take_link_confirmation_pending());
 dual_end_pairing_handle_command("PAIR INFO",response,sizeof(response));
 assert(strstr(response,"paired=1")&&strstr(response,"peer=04:12:34:56:78:9a"));
 assert(strstr(response,"source=nvs")&&strstr(response,"encryption=ccmp")&&strstr(response,"generation=7"));
 assert(strstr(response,"key_id=a1a2a3a4a5a6a7a8")&&strstr(response,"pending=committed"));
 dual_end_pairing_handle_command("PAIR CONFIRM 42",response,sizeof(response));
 assert(confirm_calls==1&&strstr(response,"OK PAIR CONFIRM")&&!dual_end_pairing_take_link_confirmation_pending());
 dual_end_pairing_handle_command("PAIR INFO",response,sizeof(response));assert(strstr(response,"pending=none")&&strstr(response,"txn=0"));
 dual_end_pairing_handle_command(prepare43,response,sizeof(response));assert(prepare_calls==2);
 dual_end_pairing_handle_command("PAIR COMMIT 43",response,sizeof(response));
 assert(commit_calls==3&&dual_end_pairing_take_link_confirmation_pending());
 dual_end_pairing_handle_command("PAIR COMMIT 0",response,sizeof(response));assert(commit_calls==3&&strstr(response,"reason=usage"));
 dual_end_pairing_handle_command("PAIR ROLLBACK 43",response,sizeof(response));
 assert(rollback_calls==1&&strstr(response,"OK PAIR ROLLBACK")&&!dual_end_pairing_take_link_confirmation_pending());
 dual_end_pairing_handle_command("PAIR INFO",response,sizeof(response));
 assert(strstr(response,"peer=04:12:34:56:78:9a")&&strstr(response,"generation=7")&&strstr(response,"pending=none"));
 relay_active=true;dual_end_pairing_handle_command("PAIR CLEAR",response,sizeof(response));
 assert(clear_calls==0&&strstr(response,"ota_active"));relay_active=false;
 clear_error=ESP_FAIL;dual_end_pairing_handle_command("PAIR CLEAR",response,sizeof(response));
 assert(clear_calls==1&&strstr(response,"ERR PAIR CLEAR"));clear_error=0;
 dual_end_pairing_handle_command("PAIR CLEAR",response,sizeof(response));
 assert(clear_calls==2&&strstr(response,"OK PAIR CLEAR")&&!dual_end_pairing_take_link_confirmation_pending());
 prepare_error=ESP_FAIL;dual_end_pairing_handle_command(prepare42,response,sizeof(response));
 assert(prepare_calls==3&&strstr(response,"ERR PAIR PREPARE")&&!dual_end_pairing_take_link_confirmation_pending());prepare_error=0;
 usb_out[0]=0;usb_read_pos=0;memset(usb_in,'X',330);memcpy(usb_in+330,"\nPAIR INFO\n",11);usb_in_len=341;
 if(setjmp(usb_done)==0)pairing_usb_task(NULL);
 assert(clear_calls==2&&strstr(usb_out,"line_too_long")&&strstr(usb_out,"OK PAIR INFO")&&stdout_lock_depth==0);

 uint8_t image[48];for(unsigned i=0;i<sizeof(image);++i)image[i]=(uint8_t)(i*23u+5u);
 uint32_t image_crc=dual_end_ota_crc32_update(0,image,sizeof(image));uint32_t first_crc=dual_end_ota_crc32_update(0,image,32);
 uint32_t second_crc=dual_end_ota_crc32_update(0,image+32,16);usb_out[0]=0;usb_read_pos=usb_in_len=0;
 int count=sprintf((char*)usb_in,"OTA2 SCEPTER 48 %08x 77 ota_1 32\nCHUNK 77 0 0 32 %08x\n",image_crc,first_crc);
 memcpy(usb_in+count,image,32);count+=32;
 count+=sprintf((char*)usb_in+count,"CHUNK 77 1 32 16 %08x\n",second_crc);
 memcpy(usb_in+count,image+32,16);count+=16;count+=sprintf((char*)usb_in+count,"END 77\n");usb_in_len=(size_t)count;
 if(setjmp(usb_done)==0)pairing_usb_task(NULL);
 assert(ota_begin_calls==1&&ota_finish_calls==1&&ota_abort_calls==0&&restarts==1);
 assert(ota_received_len==sizeof(image)&&memcmp(ota_received,image,sizeof(image))==0);
 assert(strstr(usb_out,"OK BEGIN SCEPTER session=77")&&strstr(usb_out,"OK CHUNK session=77 sequence=0"));
 assert(strstr(usb_out,"received=48 rolling_crc=")&&strstr(usb_out,"OK END SCEPTER session=77"));
 puts("direct Scepter OTA2 mixed command/binary stream commits exact bytes: passed");
 puts("secure pairing PREPARE/COMMIT/CONFIRM/ROLLBACK/INFO/CLEAR state transitions: passed");
 puts("pairing field validation, OTA exclusion, failed operations and overlong-line recovery: passed");
 puts("native USB driver ownership and complete short-write responses: passed");
 return 0;
}
'''

CDC_PLATFORM = r'''
#include <inttypes.h>
#include <ctype.h>
#ifdef _MSC_VER
#define strcasecmp _stricmp
#endif
#define DUAL_END_OTA_MAX_PAYLOAD 180u
#define DUAL_END_OTA_TARGET_DONGLE 1
#define DUAL_END_OTA_RESULT_INSTALL_CONFIRMED 1
#define DUAL_END_OTA_RESULT_STAGE_BOOT_CONFIRM 4
#define DONGLE_HID_CONFIG_REPORT_SIZE 64
#define DONGLE_HID_CONFIG_CMD_FORWARD_TO_SCEPTER 1
#define DONGLE_HID_REPORT_ID_CONFIG 4
#define DONGLE_HID_REPORT_TYPE_OUTPUT 2
#define DUAL_END_CONTROL_CMD_UI_THINKING 1
#define DUAL_END_CONTROL_CMD_UI_APPROVE 2
#define DUAL_END_CONTROL_CMD_UI_DONE 3
#define TINYUSB_CDC_ACM_0 0
#define CDC_RX_BLOCK_BYTES 64u
#define CDC_RX_QUEUE_BLOCKS 96u
#define CDC_OTA2_MAX_CHUNK_BYTES 4096u
typedef enum {CDC_OTA_IDLE,CDC_OTA_DONGLE,CDC_OTA_SCEPTER} cdc_ota_mode_t;
typedef struct {uint32_t epoch;uint16_t length;uint8_t data[CDC_RX_BLOCK_BYTES];} cdc_rx_block_t;
typedef struct {uint8_t status,stage;int32_t error_code;char app_version[32],partition[16];} test_result_payload_t;
typedef struct {uint32_t session_id;test_result_payload_t payload;} dual_end_ota_result_t;
typedef void *TaskHandle_t;
typedef int cdcacm_event_t;
static TaskHandle_t s_cdc_worker=(void*)1;
static QueueHandle_t s_cdc_rx_queue=(void*)2;
static portMUX_TYPE s_cdc_rx_lock;
static volatile bool s_cdc_connected,s_cdc_dtr,s_cdc_disconnect_pending,s_cdc_rx_overflow_pending;
static volatile bool s_cdc_discard_until_disconnect;
static volatile uint32_t s_cdc_epoch;
static uint32_t s_cdc_processing_epoch;
static char s_cmd_line[128];static size_t s_cmd_line_len;static bool s_cmd_line_overflow;
static cdc_ota_mode_t s_ota_mode;
static uint32_t s_ota_size,s_ota_crc32,s_ota_session,s_ota_remaining,s_ota_offset,s_ota_sequence;
static char s_ota_partition[16];static int64_t s_ota_last_rx_us;
static bool s_ota2_active,s_ota2_receiving_chunk,s_ota2_duplicate_chunk,s_ota2_have_last_chunk;
static uint32_t s_ota2_max_chunk_size,s_ota2_next_sequence,s_ota2_received,s_ota2_running_crc;
static uint32_t s_ota2_chunk_session,s_ota2_chunk_sequence,s_ota2_chunk_offset,s_ota2_chunk_size,s_ota2_chunk_crc;
static uint32_t s_ota2_last_sequence,s_ota2_last_offset,s_ota2_last_size,s_ota2_last_crc;
static size_t s_ota2_chunk_received;static uint8_t s_ota2_chunk[CDC_OTA2_MAX_CHUNK_BYTES];
static uint8_t received[16384];static size_t received_len;
static char responses[32768];static size_t responses_len;
static bool mounted=true,usb_context,completion_delivery=true;
static unsigned completion_calls,scepter_abort_calls;
static uint8_t usb_fifo[8192];static size_t usb_size,usb_position;static unsigned usb_flushes;
static cdc_rx_block_t queue[CDC_RX_QUEUE_BLOCKS];static unsigned queue_head,queue_count;
static esp_err_t (*s_hid_config_report_cb)(int,int,const uint8_t*,size_t,void*);
static void *s_hid_config_report_ctx;
static void append_response(const char*p,size_t n){assert(responses_len+n<sizeof(responses));memcpy(responses+responses_len,p,n);responses_len+=n;responses[responses_len]=0;}
static void cdc_write(int itf,const char*line){assert(!usb_context&&itf==0);append_response(line,strlen(line));}
static void cdc_printf(int itf,const char*fmt,...){char line[512];va_list args;va_start(args,fmt);int n=vsnprintf(line,sizeof(line),fmt,args);va_end(args);assert(n>=0&&(size_t)n<sizeof(line));cdc_write(itf,line);}
static bool cdc_write_completion(int itf,const char*line){++completion_calls;cdc_write(itf,line);return completion_delivery;}
static bool tud_mounted(void){return mounted;}
static void dual_end_ota_cancel_scepter_result_wait(void){}
static int dual_end_ota_self_begin_partition(int target,uint32_t session,uint32_t size,uint32_t crc,const char*partition){(void)target;(void)session;(void)size;(void)crc;(void)partition;++begin_calls;return ESP_OK;}
static int dual_end_ota_self_write(const uint8_t*p,size_t n){assert(!usb_context&&received_len+n<=sizeof(received));memcpy(received+received_len,p,n);received_len+=n;++writes;return ota_write_error;}
static int dual_end_ota_self_finish(bool reboot){assert(!reboot);++ends;return ota_end_error;}
static void dual_end_ota_self_abort(void){++aborts;}
static uint32_t dual_end_ota_crc32_update(uint32_t crc,const uint8_t*p,size_t n){return esp_crc32_le(crc,p,n);}
static int dual_end_ota_send_scepter_begin(uint32_t size,uint32_t crc,uint32_t session,const char*partition){(void)size;(void)crc;(void)session;(void)partition;return ESP_OK;}
static int dual_end_ota_send_scepter_data(const uint8_t*p,size_t n,uint32_t off,uint32_t seq,uint32_t size,uint32_t crc,uint32_t session){(void)p;(void)n;(void)off;(void)seq;(void)size;(void)crc;(void)session;return ESP_OK;}
static int dual_end_ota_send_scepter_end(uint32_t size,uint32_t crc,uint32_t session,uint32_t sequence){(void)size;(void)crc;(void)session;(void)sequence;return ESP_OK;}
static int dual_end_ota_send_scepter_abort(uint32_t session){(void)session;++scepter_abort_calls;return ESP_OK;}
static int dual_end_ota_wait_scepter_result(unsigned timeout,dual_end_ota_result_t*out){(void)timeout;memset(out,0,sizeof(*out));return ESP_ERR_TIMEOUT;}
static bool dual_end_pairing_handle_command(const char*line,char*out,size_t n){(void)line;(void)out;(void)n;return false;}
static void cdc_process_erase(int itf,const char*a,const char*b){(void)itf;(void)a;(void)b;}
static void cdc_process_boot(int itf,const char*a,const char*b){(void)itf;(void)a;(void)b;}
static void cdc_process_log_command(int itf,const char*a){(void)itf;(void)a;}
static void cdc_process_reboot(int itf,const char*a){(void)itf;(void)a;}
static void cdc_print_status(int itf){(void)itf;}
static int xQueueSend(QueueHandle_t q,const void*p,unsigned ticks){assert(q==s_cdc_rx_queue&&ticks==0&&usb_context);if(queue_count==CDC_RX_QUEUE_BLOCKS)return 0;queue[(queue_head+queue_count)%CDC_RX_QUEUE_BLOCKS]=*(const cdc_rx_block_t*)p;++queue_count;return pdTRUE;}
static int xQueueReceive(QueueHandle_t q,void*p,unsigned ticks){assert(q==s_cdc_rx_queue&&ticks==0&&!usb_context);if(!queue_count)return 0;*(cdc_rx_block_t*)p=queue[queue_head];queue_head=(queue_head+1)%CDC_RX_QUEUE_BLOCKS;--queue_count;return pdTRUE;}
static void xTaskNotifyGive(TaskHandle_t task){assert(task==s_cdc_worker);}
static int tinyusb_cdcacm_read(int itf,uint8_t*p,size_t n,size_t*done){assert(usb_context&&itf==0&&n==64);size_t take=usb_size-usb_position;if(take>n)take=n;memcpy(p,usb_fifo+usb_position,take);usb_position+=take;*done=take;return ESP_OK;}
static void tud_cdc_n_read_flush(int itf){assert(usb_context&&itf==0);usb_position=usb_size;++usb_flushes;}
static int test_send(const void*p,int len){(void)p;(void)len;return ESP_OK;}
static void cdc_reset_ota_state(void);
static void cdc_process_rx(int itf,const uint8_t*data,size_t len);
static void reset_cdc(void){
 cdc_reset_ota_state();queue_head=queue_count=0;usb_size=usb_position=usb_flushes=0;
 s_cdc_epoch=s_cdc_processing_epoch=7;s_cdc_connected=true;s_cdc_dtr=false;
 s_cdc_disconnect_pending=s_cdc_rx_overflow_pending=s_cdc_discard_until_disconnect=false;
 s_cmd_line_len=0;s_cmd_line_overflow=false;received_len=responses_len=0;responses[0]=0;
 mounted=true;usb_context=false;completion_delivery=true;completion_calls=scepter_abort_calls=0;
 writes=aborts=ends=begin_calls=restarts=0;ota_write_error=ota_end_error=0;
}
static void feed(const void*p,size_t n){cdc_process_rx(0,(const uint8_t*)p,n);}
static void command(const char*text){feed(text,strlen(text));}
static size_t chunk_frame(uint8_t*out,uint32_t session,uint32_t sequence,uint32_t offset,const uint8_t*data,size_t n,uint32_t crc){
 int h=sprintf((char*)out,"CHUNK %u %u %u %u %08x\n",session,sequence,offset,(unsigned)n,crc);assert(h>0);memcpy(out+h,data,n);return (size_t)h+n;
}
'''

CDC_CASES = r'''
int main(void){
 uint8_t image[128],frame[256];for(unsigned i=0;i<sizeof(image);++i)image[i]=(uint8_t)(i*29u+7u);
 uint32_t image_crc=dual_end_ota_crc32_update(0,image,sizeof(image));char begin[128];
 reset_cdc();sprintf(begin,"OTA2 DONGLE 128 %08x 71 ota_1 64\n",image_crc);command(begin);
 assert(begin_calls==1&&s_ota2_active&&strstr(responses,"OK BEGIN DONGLE"));responses_len=0;responses[0]=0;
 uint32_t crc0=dual_end_ota_crc32_update(0,image,20);size_t n=chunk_frame(frame,71,0,0,image,20,crc0);assert(n<64);feed(frame,n);
 char expected[96];sprintf(expected,"received=20 rolling_crc=%08x duplicate=0",crc0);
 assert(writes==1&&received_len==20&&!memcmp(received,image,20)&&strstr(responses,expected));
 responses_len=0;responses[0]=0;uint32_t crc1=dual_end_ota_crc32_update(0,image+20,64);
 n=chunk_frame(frame,71,1,20,image+20,64,crc1^1u);feed(frame,n);
 assert(writes==1&&s_ota2_received==20&&!s_ota2_receiving_chunk&&strstr(responses,"ERR CHUNK_CRC")&&strstr(responses,"retryable=1"));
 responses_len=0;responses[0]=0;n=chunk_frame(frame,71,1,20,image+20,64,crc1);feed(frame,n);
 uint32_t rolling=dual_end_ota_crc32_update(crc0,image+20,64);sprintf(expected,"received=84 rolling_crc=%08x duplicate=0",rolling);
 assert(writes==2&&received_len==84&&strstr(responses,expected));
 responses_len=0;responses[0]=0;feed(frame,n);
 sprintf(expected,"received=84 rolling_crc=%08x duplicate=1",rolling);
 assert(writes==2&&received_len==84&&s_ota2_next_sequence==2&&strstr(responses,expected));
 responses_len=0;responses[0]=0;command("CHUNK 71 4 84 44 00000000\n");
 assert(writes==2&&!s_ota2_receiving_chunk&&strstr(responses,"ERR CHUNK_ORDER")&&strstr(responses,"expected_sequence=2")&&strstr(responses,"expected_offset=84"));
 responses_len=0;responses[0]=0;uint32_t crc2=dual_end_ota_crc32_update(0,image+84,44);n=chunk_frame(frame,71,2,84,image+84,44,crc2);feed(frame,n);
 assert(writes==3&&received_len==sizeof(image)&&!memcmp(received,image,sizeof(image))&&s_ota2_running_crc==image_crc);
 responses_len=0;responses[0]=0;command("END 72\n");assert(ends==0&&s_ota2_active&&strstr(responses,"stage=session"));
 responses_len=0;responses[0]=0;command("END 71\n");
 assert(ends==1&&completion_calls==1&&restarts==1&&strstr(responses,"OK END DONGLE")&&strstr(responses,"calculated_crc="));
 puts("OTA2 CRC retry, duplicate ACK idempotency, ordering, rolling CRC and END: passed");

 reset_cdc();uint32_t partial_crc=dual_end_ota_crc32_update(0,image,32);sprintf(begin,"OTA2 DONGLE 32 %08x 91 ota_1 32\n",partial_crc);command(begin);
 responses_len=0;responses[0]=0;n=chunk_frame(frame,91,0,0,image,16,dual_end_ota_crc32_update(0,image,16));feed(frame,n);command("ABORT 91\n");
 assert(aborts==1&&s_ota_mode==CDC_OTA_IDLE&&!s_ota2_active&&strstr(responses,"OK ABORT DONGLE session=91 received=16"));
 puts("OTA2 ABORT reports committed bytes and clears the transfer: passed");
 return 0;
}
'''

RX_CASES = r'''
static void reset_rx(void){reset_cdc();}
static void receive_packet(const uint8_t*p,size_t n){
 assert(n<=sizeof(usb_fifo));memcpy(usb_fifo,p,n);usb_size=n;usb_position=0;
 usb_context=true;cdc_rx_cb(0,NULL);usb_context=false;
 /* Endpoint storage may be reused as soon as the callback returns. */
 memset(usb_fifo,0xa5,sizeof(usb_fifo));
}
static void drain_worker(void){
 cdc_handle_rx_events(0);cdc_rx_block_t block;
 while(xQueueReceive(s_cdc_rx_queue,&block,0)==pdTRUE){cdc_handle_rx_events(0);cdc_consume_rx_block(0,&block);}
}
static unsigned occurrences(const char*text,const char*needle){unsigned count=0;while((text=strstr(text,needle))!=NULL){++count;text+=strlen(needle);}return count;}
int main(void){
 uint8_t image[116],frame[256];for(unsigned i=0;i<sizeof(image);++i)image[i]=(uint8_t)(i*17u+3u);
 uint32_t image_crc=dual_end_ota_crc32_update(0,image,sizeof(image));char begin[128];
 reset_rx();sprintf(begin,"OTA2 DONGLE 116 %08x 77 ota_1 100\n",image_crc);receive_packet((const uint8_t*)begin,strlen(begin));drain_worker();
 responses_len=0;responses[0]=0;size_t n=chunk_frame(frame,77,0,0,image,16,dual_end_ota_crc32_update(0,image,16));assert(n<64);receive_packet(frame,n);
 assert(queue_count==1&&received_len==0);drain_worker();assert(received_len==16&&writes==1&&strstr(responses,"received=16"));
 responses_len=0;responses[0]=0;n=chunk_frame(frame,77,1,16,image+16,100,dual_end_ota_crc32_update(0,image+16,100));assert(n>64);receive_packet(frame,n);
 assert(queue_count==(n+63)/64&&queue_count>=2&&received_len==16);drain_worker();
 assert(received_len==sizeof(image)&&writes==2&&!memcmp(received,image,sizeof(image))&&strstr(responses,"received=116")&&strstr(responses,"rolling_crc="));
 puts("CHUNK header plus binary survives same and crossed USB 64-byte blocks: passed");

 reset_rx();sprintf(begin,"OTA2 DONGLE 8192 %08x 88 ota_1 4096\n",0u);command(begin);responses_len=0;responses[0]=0;
 uint8_t noise[64];memset(noise,0x5a,sizeof(noise));for(unsigned i=0;i<=CDC_RX_QUEUE_BLOCKS;++i)receive_packet(noise,sizeof(noise));
 assert(s_cdc_epoch==8&&s_cdc_rx_overflow_pending&&s_cdc_discard_until_disconnect);drain_worker();
 assert(aborts==1&&strstr(responses,"ERR CDC_RX_OVERFLOW")&&received_len==0&&s_ota_mode==CDC_OTA_IDLE);
 unsigned queued=queue_count;receive_packet((const uint8_t*)"PING\n",5);assert(queue_count==queued);
 puts("RX queue overflow invalidates queued binary and requires reconnect: passed");

 reset_rx();receive_packet((const uint8_t*)"OLD\n",4);assert(queue_count==1);
 usb_context=true;cdc_invalidate_connection();usb_context=false;assert(s_cdc_epoch==8&&s_cdc_disconnect_pending&&usb_flushes==1);
 receive_packet((const uint8_t*)"PING\n",5);assert(queue_count==2);drain_worker();
 assert(occurrences(responses,"OK PONG role=dongle")==1&&!strstr(responses,"ERR COMMAND unknown")&&!s_cdc_discard_until_disconnect);
 puts("disconnect epoch drops stale copies without flushing the new connection: passed");
 return 0;
}
'''


def compiler_env():
    env = os.environ.copy()
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler:
        return compiler, env, False
    msvc = sorted(Path(r"C:\Program Files\Microsoft Visual Studio").glob("*/*/VC/Tools/MSVC/*"))
    msvc += sorted(Path(r"C:\Program Files (x86)\Microsoft Visual Studio").glob("*/*/VC/Tools/MSVC/*"))
    for tools in reversed(msvc):
        cl = tools / "bin/Hostx64/x64/cl.exe"
        if not cl.exists():
            continue
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\Microsoft\Windows Kits\Installed Roots") as key:
            sdk = Path(os.environ.get("WindowsSdkDir") or winreg.QueryValueEx(key, "KitsRoot10")[0])
        version = sorted((sdk / "Include").glob("10.*"))[-1].name
        env["INCLUDE"] = ";".join(str(p) for p in [tools / "include", sdk / "Include" / version / "ucrt", sdk / "Include" / version / "shared", sdk / "Include" / version / "um"])
        env["LIB"] = ";".join(str(p) for p in [tools / "lib/x64", sdk / "Lib" / version / "ucrt/x64", sdk / "Lib" / version / "um/x64"])
        env["PATH"] = str(cl.parent) + os.pathsep + env["PATH"]
        return str(cl), env, True
    raise RuntimeError("A host C compiler is required (GCC, Clang or MSVC).")


def cdc_ota2_implementation(production):
    functions = [
        ("cdc_reset_ota_state", "void"),
        ("cdc_start_ota", "void"),
        ("cdc_ota_epoch_valid", "bool"),
        ("cdc_abort_ota_stream", "void"),
        ("cdc_ota2_chunk_begin", "void"),
        ("cdc_ota2_chunk_process", "size_t"),
        ("cdc_ota2_finish", "void"),
        ("trim_line", "void"),
        ("split_command_tokens", "size_t"),
        ("parse_u32_token", "bool"),
        ("cdc_process_command", "void"),
        ("cdc_process_rx", "void"),
    ]
    return "\n".join(production_function(production, name, return_type)
                     for name, return_type in functions)


class OtaReliability(unittest.TestCase):
    def test_cdc_rx_callback_ownership(self):
        compiler, env, msvc = compiler_env()
        usb = COMPONENT.parents[2] / "dongle/components/dongle_usb_device/dongle_usb_device.c"
        production = usb.read_text()
        implementation = cdc_ota2_implementation(production)
        implementation += "\n" + "\n".join(
            production_function(production, name) for name in [
                "cdc_invalidate_connection", "cdc_rx_cb",
                "cdc_handle_rx_events", "cdc_consume_rx_block",
            ])
        # Replace the generic platform queue stubs with a copying bounded queue.
        platform = "#define xQueueSend unused_queue_send\n#define xQueueReceive unused_queue_receive\n" + PLATFORM
        platform += "\n#undef xQueueSend\n#undef xQueueReceive\n"
        with tempfile.TemporaryDirectory(prefix="dual_mic_rx_") as temp:
            work = Path(temp)
            source = work / "rx_test.c"
            source.write_text(platform + CDC_PLATFORM + implementation + RX_CASES)
            exe = work / "rx_test.exe"
            command = ([compiler, "/nologo", "/std:c11", "/TC", "/W3", str(source), "/Fe:" + str(exe)] if msvc else
                       [compiler, "-std=c11", str(source), "-o", str(exe)])
            build = subprocess.run(command, cwd=work, env=env, capture_output=True, text=True, errors="replace")
            self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
            run = subprocess.run([str(exe)], cwd=work, env=env, capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            print(run.stdout)

    def test_cdc_ota2_protocol(self):
        compiler, env, msvc = compiler_env()
        usb = COMPONENT.parents[2] / "dongle/components/dongle_usb_device/dongle_usb_device.c"
        implementation = cdc_ota2_implementation(usb.read_text())
        platform = "#define xQueueSend unused_queue_send\n#define xQueueReceive unused_queue_receive\n" + PLATFORM
        platform += "\n#undef xQueueSend\n#undef xQueueReceive\n"
        with tempfile.TemporaryDirectory(prefix="dual_mic_cdc_") as temp:
            work = Path(temp)
            source = work / "cdc_test.c"
            source.write_text(platform + CDC_PLATFORM + implementation + CDC_CASES)
            exe = work / "cdc_test.exe"
            command = ([compiler, "/nologo", "/std:c11", "/TC", "/W3", str(source), "/Fe:" + str(exe)] if msvc else
                       [compiler, "-std=c11", str(source), "-o", str(exe)])
            build = subprocess.run(command, cwd=work, env=env, capture_output=True, text=True, errors="replace")
            self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
            run = subprocess.run([str(exe)], cwd=work, env=env, capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            print(run.stdout)

    def test_pairing_commands(self):
        compiler, env, msvc = compiler_env()
        pairing = COMPONENT.parent / "dual_end_pairing"
        with tempfile.TemporaryDirectory(prefix="dual_mic_pairing_") as temp:
            work = Path(temp)
            (work / "test_platform.h").write_text(PLATFORM + PAIRING_PLATFORM)
            for header in ["driver/usb_serial_jtag.h", "driver/usb_serial_jtag_vfs.h", "dual_end_espnow_link.h", "dual_end_link_config.h", "esp_err.h", "esp_check.h", "esp_log.h", "esp_mac.h", "esp_app_desc.h", "esp_system.h", "esp_timer.h", "freertos/FreeRTOS.h", "freertos/task.h", "mbedtls/platform_util.h", "sdkconfig.h", "strings.h"]:
                target = work / header
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text('#include "test_platform.h"\n')
            source = work / "pairing_test.c"
            source.write_text('#include "test_platform.h"\n#include "' + str(pairing / "dual_end_pairing.c").replace("\\", "/") + '"\n' + PAIRING_CASES)
            exe = work / "pairing_test.exe"
            includes = [work, COMPONENT / "include", pairing / "include"]
            if msvc:
                command = [compiler, "/nologo", "/std:c11", "/TC", "/W3"] + ["/I" + str(p) for p in includes] + [str(source), "/Fe:" + str(exe)]
            else:
                command = [compiler, "-std=c11"] + ["-I" + str(p) for p in includes] + [str(source), "-o", str(exe)]
            build = subprocess.run(command, cwd=work, env=env, capture_output=True, text=True, errors="replace")
            self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
            run = subprocess.run([str(exe)], cwd=work, env=env, capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            print(run.stdout)

    def test_real_state_machine(self):
        compiler, env, msvc = compiler_env()
        with tempfile.TemporaryDirectory(prefix="dual_mic_ota_") as temp:
            work = Path(temp)
            (work / "test_platform.h").write_text(PLATFORM)
            headers = ["driver/uart.h", "esp_app_desc.h", "esp_check.h", "esp_crc.h", "esp_log.h", "esp_mac.h", "esp_now.h", "esp_ota_ops.h", "esp_partition.h", "esp_system.h", "esp_timer.h", "dual_end_espnow_link.h", "dual_end_link_config.h", "freertos/FreeRTOS.h", "freertos/task.h", "freertos/queue.h", "freertos/semphr.h", "nvs.h", "sdkconfig.h", "esp_err.h"]
            for header in headers:
                target = work / header
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text('#include "test_platform.h"\n')
            source = work / "ota_test.c"
            source.write_text('#include "test_platform.h"\n#include "' + str(COMPONENT / "dual_end_ota.c").replace("\\", "/") + '"\n' + CASES)
            exe = work / "ota_test.exe"
            if msvc:
                command = [compiler, "/nologo", "/std:c11", "/TC", "/W3", "/I" + str(work), "/I" + str(COMPONENT / "include"), str(source), "/Fe:" + str(exe)]
            else:
                command = [compiler, "-std=c11", "-I" + str(work), "-I" + str(COMPONENT / "include"), str(source), "-o", str(exe)]
            build = subprocess.run(command, cwd=work, env=env, capture_output=True, text=True, errors="replace")
            self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
            run = subprocess.run([str(exe)], cwd=work, env=env, capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            self.assertIn("lost END ACK", run.stdout)
            print(run.stdout)


if __name__ == "__main__":
    unittest.main()
