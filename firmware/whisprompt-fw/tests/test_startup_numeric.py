"""Compile actual startup helpers on the host; ESP APIs are mocks, no devices.

Scheduling/cache safety remains a source audit and hardware validation item.
"""
from pathlib import Path
import re
import pytest
from test_audio_recovery_042 import _build_host_executable, _run
from test_runtime_codec import function

ROOT = Path(__file__).resolve().parents[1]
MIC = ROOT / 'scepter/main/mic/mic_audio.c'
RUNTIME = ROOT / 'components/dual_end_runtime/dual_end_runtime.c'
POLICY = ROOT / 'components/dual_end_runtime/include'


def compile_run(tmp_path, code):
    harness = tmp_path / 'startup_contract.c'
    harness.write_text(code, encoding='utf-8')
    executable, env = _build_host_executable([harness], tmp_path, extra_includes=(POLICY,))
    return _run([str(executable)], cwd=tmp_path, env=env).stdout


def mic_defines(source):
    start = source.index('#define MIC_START_READY_BIT')
    end = source.index('#define MIC_ANALYSIS_RUNTIME_LOG_INTERVAL_MS', start)
    return source[start:end]


@pytest.mark.parametrize('psram,allowed,cores,unicore', [(1,1,2,0),(0,0,2,0),(1,0,2,0),(1,1,1,1)])
def test_actual_mic_starter_faults_lifecycle_and_core_fallback(tmp_path, psram, allowed, cores, unicore):
    source = MIC.read_text(encoding='utf-8')
    code = f'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#define CONFIG_SPIRAM {psram}
#define CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM {allowed}
#define SOC_CPU_CORES_NUM {cores}
#define CONFIG_FREERTOS_UNICORE {unicore}
#include "dual_end_task_policy.h"
''' + r'''
#define BIT2 (1U<<2)
#define BIT3 (1U<<3)
#define ESP_OK 0
#define ESP_ERR_NO_MEM -1
#define ESP_ERR_INVALID_STATE -2
#define ESP_ERR_TIMEOUT -3
#define ESP_RETURN_ON_FALSE(c,err,...) do {if(!(c))return(err);}while(0)
#define ESP_LOGI(...) ((void)0)
#define pdPASS 1
#define pdFALSE 0
#define MALLOC_CAP_SPIRAM 1U
#define MALLOC_CAP_8BIT 2U
#define pdMS_TO_TICKS(ms) (ms)
typedef int esp_err_t,BaseType_t;
typedef void *TaskHandle_t;
typedef unsigned EventBits_t;
static TaskHandle_t s_mic_task_handle;
static void *s_mic_state_events=(void *)(uintptr_t)1;
''' + mic_defines(source) + r'''
static unsigned event_bits,wait_reply,creates,notifications,deletes,ordinary_deletes,caps_deletes,order;
static unsigned creation_result=pdPASS;
static void mic_audio_task(void *p){(void)p;}
static void xEventGroupClearBits(void *e,unsigned b){assert(e==s_mic_state_events && order==0);event_bits&=~b;order=1;}
static int create_task(void(*fn)(void*),const char *n,unsigned stack,void *arg,unsigned pri,TaskHandle_t *h,unsigned core,unsigned caps){
 (void)n;assert(fn==mic_audio_task && !arg && stack==40U*1024U && pri==5);
 assert(core==DUAL_END_AUDIO_TASK_CORE && core<SOC_CPU_CORES_NUM && order==1);
 assert(caps==(MIC_TASK_PSRAM_STACK?MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT:0));
 assert(!(event_bits&(MIC_START_READY_BIT|MIC_START_FAILED_BIT)));creates++;order=2;
 if(creation_result==pdPASS)*h=(void *)(uintptr_t)2;return creation_result;
}
int xTaskCreatePinnedToCoreWithCaps(void(*f)(void*),const char*n,unsigned s,void*p,unsigned pri,TaskHandle_t*h,unsigned c,unsigned caps){return create_task(f,n,s,p,pri,h,c,caps);}
int xTaskCreatePinnedToCore(void(*f)(void*),const char*n,unsigned s,void*p,unsigned pri,TaskHandle_t*h,unsigned c){return create_task(f,n,s,p,pri,h,c,0);}
static void xTaskNotifyGive(TaskHandle_t h){assert(order==2 && h==s_mic_task_handle && h);notifications++;order=3;}
static unsigned xEventGroupWaitBits(void *e,unsigned b,int clear,int all,unsigned t){
 assert(e==s_mic_state_events && order==3 && !clear && !all && t==3000);
 assert(b==(MIC_START_READY_BIT|MIC_START_FAILED_BIT));order=4;return wait_reply;
}
static void delete_task(TaskHandle_t h,bool caps){assert(order==4 && h && !s_mic_task_handle);deletes++;if(caps)caps_deletes++;else ordinary_deletes++;}
void vTaskDeleteWithCaps(TaskHandle_t h){delete_task(h,true);}
void vTaskDelete(TaskHandle_t h){delete_task(h,false);}
''' + function(source,'mic_audio_start') + r'''
static void reset_case(void){
 s_mic_task_handle=NULL;s_mic_state_events=(void *)(uintptr_t)1;event_bits=MIC_START_READY_BIT|MIC_START_FAILED_BIT;
 wait_reply=order=creates=notifications=deletes=ordinary_deletes=caps_deletes=0;creation_result=pdPASS;
}
int main(void){
 reset_case();s_mic_state_events=NULL;assert(mic_audio_start()==ESP_ERR_INVALID_STATE && !creates);
 reset_case();s_mic_task_handle=(void *)(uintptr_t)7;assert(mic_audio_start()==ESP_ERR_INVALID_STATE && !creates);
 reset_case();creation_result=0;assert(mic_audio_start()==ESP_ERR_NO_MEM && creates==1 && !notifications && !deletes && !s_mic_task_handle);
 reset_case();wait_reply=MIC_START_READY_BIT;assert(mic_audio_start()==ESP_OK && notifications==1 && !deletes && s_mic_task_handle);
 unsigned before=creates;assert(mic_audio_start()==ESP_ERR_INVALID_STATE && creates==before);
 reset_case();wait_reply=MIC_START_FAILED_BIT;assert(mic_audio_start()==ESP_ERR_NO_MEM && deletes==1 && !s_mic_task_handle);
 assert(caps_deletes==(MIC_TASK_PSRAM_STACK?1U:0U) && ordinary_deletes==(MIC_TASK_PSRAM_STACK?0U:1U));
 reset_case();wait_reply=MIC_START_READY_BIT|MIC_START_FAILED_BIT;assert(mic_audio_start()==ESP_ERR_NO_MEM && deletes==1 && !s_mic_task_handle);
 reset_case();assert(mic_audio_start()==ESP_ERR_TIMEOUT && !deletes && s_mic_task_handle);
 before=creates;assert(mic_audio_start()==ESP_ERR_INVALID_STATE && creates==before);
 puts("actual mic starter lifecycle PASS (ESP mocks)");return 0;
}
'''
    assert 'actual mic starter' in compile_run(tmp_path, code)


def test_actual_mic_task_initialization_never_reports_ready_after_alloc_failure(tmp_path):
    # Retain the original failure/ready contract and extend it to every new
    # buffer, queue, mutex, fallback and worker creation boundary.
    from test_mic_pipeline import check_pipeline_startup
    check_pipeline_startup(tmp_path)


@pytest.mark.parametrize("external", [0, 1])
def test_actual_runtime_start_publication_and_failure_cleanup(tmp_path, external):
    source = RUNTIME.read_text(encoding='utf-8')
    code = f'#define CONFIG_SPIRAM {external}\n#define CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM {external}\n'+r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include <inttypes.h>
#include "dual_end_perf_format.h"
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE -2
#define ESP_ERR_NO_MEM -1
#define ESP_MAC_WIFI_STA 0
#define SCEPTER_ROLE_SCEPTER 1
#define RUNTIME_QUEUE_DEPTH 4
#define RUNTIME_LOGGER_STACK_BYTES 6144
#define RUNTIME_LOGGER_PRIORITY 2
#define DUAL_END_COMM_TASK_CORE 0
#define pdPASS 1
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
typedef int scepter_role_t,esp_err_t,BaseType_t;
#define MALLOC_CAP_SPIRAM 4
#define MALLOC_CAP_8BIT 2
typedef void *TaskHandle_t,*QueueHandle_t;
typedef struct{unsigned cores;}esp_chip_info_t;
typedef struct{unsigned value;}runtime_snapshot_t;
typedef struct{char version[32];uint8_t app_elf_sha256[32];}esp_app_desc_t;
static bool s_attempted;static atomic_bool s_enabled,s_identity_ready;static QueueHandle_t s_snapshots;
static uint32_t s_boot_id;static scepter_role_t s_role;static esp_chip_info_t s_chip;
static char s_fw[32],s_device_id[24],s_elf_sha256[65];static esp_app_desc_t desc={"app-0.5.3",{0}};
static unsigned queues,creations,deletes,notifications,random_calls;static bool queue_ok=true,task_ok=true;static unsigned transport_starts;static esp_err_t transport_result=ESP_OK;static esp_err_t dual_end_log_transport_start(scepter_role_t role){assert(role==1 && !s_snapshots && !atomic_load_explicit(&s_enabled,memory_order_acquire));++transport_starts;return transport_result;}
static uint32_t esp_random(void){return random_calls++?7:0;}
static void esp_chip_info(esp_chip_info_t*p){p->cores=2;}
static const esp_app_desc_t *esp_app_get_description(void){return &desc;}
static int esp_read_mac(uint8_t*mac,int k){(void)k;memset(mac,1,6);return ESP_OK;}
static void runtime_logger(void*p){(void)p;}
static void runtime_log_identity(int c){assert(c==-1 && !atomic_load_explicit(&s_enabled,memory_order_acquire));}
static void *xQueueCreate(unsigned depth,size_t size){assert(depth==4 && size==sizeof(runtime_snapshot_t) && !atomic_load_explicit(&s_enabled,memory_order_acquire));queues++;return queue_ok?(void *)(uintptr_t)3:NULL;}
int xTaskCreatePinnedToCore(void(*f)(void*),const char*n,unsigned stack,void*p,unsigned pri,TaskHandle_t*h,unsigned c){
 (void)n;assert(f==runtime_logger && !p && stack==6144 && pri==2 && c==0 && s_snapshots);
 assert(!atomic_load_explicit(&s_enabled,memory_order_acquire));creations++;if(task_ok)*h=(void *)(uintptr_t)4;return task_ok?pdPASS:0;}
int xTaskCreatePinnedToCoreWithCaps(void(*f)(void*),const char*n,unsigned stack,
 void*p,unsigned pri,TaskHandle_t*h,unsigned c,unsigned caps){
 assert(caps==6);return xTaskCreatePinnedToCore(f,n,stack,p,pri,h,c);
}
static void vQueueDelete(void*q){assert(q==s_snapshots && !atomic_load_explicit(&s_enabled,memory_order_acquire));deletes++;}
static void xTaskNotifyGive(TaskHandle_t h){assert(h && s_snapshots && atomic_load_explicit(&s_enabled,memory_order_acquire));notifications++;}
''' + function(source,'runtime_format_build_info') + '\n' + function(source,'dual_end_runtime_start') + r'''
static void reset(void){s_attempted=false;atomic_store_explicit(&s_enabled,false,memory_order_release);s_snapshots=NULL;
 atomic_store_explicit(&s_identity_ready,false,memory_order_release);s_boot_id=0;
 desc.app_elf_sha256[0]=0xab;desc.app_elf_sha256[31]=0xcd;memset(s_elf_sha256,0,sizeof(s_elf_sha256));
 queues=creations=deletes=notifications=random_calls=transport_starts=0;queue_ok=task_ok=true;transport_result=ESP_OK;}
int main(void){
 reset();queue_ok=false;assert(dual_end_runtime_start(1)==ESP_ERR_NO_MEM && queues==1 && !creations && !deletes && !notifications && !s_snapshots);
 assert(dual_end_runtime_start(1)==ESP_ERR_INVALID_STATE && queues==1);
 reset();task_ok=false;assert(dual_end_runtime_start(1)==ESP_ERR_NO_MEM && creations==1 && deletes==1 && !notifications && !s_snapshots);
 assert(!atomic_load_explicit(&s_enabled,memory_order_acquire));
 reset();assert(dual_end_runtime_start(1)==ESP_OK && notifications==1 && !deletes && queues==1 && creations==1 && s_boot_id==7);
 assert(atomic_load_explicit(&s_enabled,memory_order_acquire));assert(dual_end_runtime_start(1)==ESP_OK && queues==1 && creations==1); assert(transport_starts==1);
 assert(atomic_load_explicit(&s_identity_ready,memory_order_acquire));
 assert(strlen(s_elf_sha256)==64 && !memcmp(s_elf_sha256,"ab",2) && !memcmp(s_elf_sha256+62,"cd",2));
 reset();s_boot_id=42;assert(dual_end_runtime_start(1)==ESP_OK&&s_boot_id==42&&random_calls==0); reset();transport_result=ESP_ERR_NO_MEM;assert(dual_end_runtime_start(1)==ESP_OK && notifications==1 && transport_starts==1); assert(atomic_load_explicit(&s_enabled,memory_order_acquire));
 puts("actual runtime publication/failure cleanup PASS (ESP mocks)");return 0;}
'''
    assert 'actual runtime publication' in compile_run(tmp_path, code)
    logger = function(source,'runtime_logger')
    assert logger.index('ulTaskNotifyTake') < logger.index('runtime_log_identity')


def test_shared_numeric_version_and_critical_confirmation_precedes_profiler(tmp_path):
    versions = []
    for role in ('scepter','dongle'):
        cmake = (ROOT/role/'CMakeLists.txt').read_text(encoding='utf-8')
        version = re.search(r'set\(DUAL_END_PUBLIC_VERSION "([^"]+)"\)',cmake).group(1)
        assert re.fullmatch(r'\d+\.\d+\.\d+',version)
        assert 'set(PROJECT_VER "${DUAL_END_IMAGE_KEY}-${DUAL_END_PUBLIC_VERSION}")' in cmake
        header = (ROOT/role/'components/dual_end_common/include/dual_end_link_config.h').read_text(encoding='utf-8')
        assert re.search(r'^#define SCEPTER_FW_VERSION "([^"]+)"',header,re.M).group(1)==version
        versions.append(version)
        app = function((ROOT/role/'main/app_main.c').read_text(encoding='utf-8'),'app_main')
        confirm = app.index('dual_end_ota_confirm_running_image(')
        assert confirm < app.index('dual_end_runtime_start')
        if role=='scepter':
            assert 'STARTUP_DIAG_CHECK("confirm_image", dual_end_ota_confirm_running_image());' in app
            full=(ROOT/role/'main/app_main.c').read_text(encoding='utf-8')
            assert 'ESP_ERROR_CHECK(startup_result)' in full
            required='STARTUP_DIAG_CHECK("audio_sender", dual_end_audio_sender_start());'
        else:
            assert 'ESP_ERROR_CHECK(dual_end_ota_confirm_running_image())' in app
            required='ESP_ERROR_CHECK(dual_end_audio_receiver_start_timed('            start = app.index(required)            assert app.index('dongle_uac_bridge_write_timed_pcm', start) < confirm            assert app.index('dongle_uac_bridge_playout_clock', start) < confirm
        assert app.index(required) < confirm
    assert versions[0]==versions[1]
    hardware = (ROOT/'scepter/main/scepter_hardware.c').read_text(encoding='utf-8')
    assert 'ESP_RETURN_ON_ERROR(mic_audio_start()' in hardware
    task = function(MIC.read_text(encoding='utf-8'),'mic_audio_task')
    assert not re.search(r'\b(?:nvs_commit|nvs_set_\w+|esp_flash_\w+|esp_ota_\w+)\s*\(',task)


def test_actual_bootstatus_formatter_image_identity_and_bounds(tmp_path):
    source=(ROOT/'scepter/components/dual_end_pairing/dual_end_pairing.c').read_text(encoding='utf-8')
    public=(ROOT/'scepter/components/dual_end_common/include/dual_end_link_config.h').read_text(encoding='utf-8')
    version_define=re.search(r'^#define SCEPTER_FW_VERSION .*$',public,re.M).group(0)
    code=r'''
#include <assert.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#define PAIRING_RESPONSE_MAX_LEN 512
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 259
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
#define MALLOC_CAP_DMA 4
#define MALLOC_CAP_SPIRAM 8
typedef int esp_err_t;
typedef enum{ESP_OTA_IMG_NEW=0,ESP_OTA_IMG_PENDING_VERIFY=1,ESP_OTA_IMG_VALID=2,ESP_OTA_IMG_INVALID=3,ESP_OTA_IMG_ABORTED=4,ESP_OTA_IMG_UNDEFINED=-1}esp_ota_img_states_t;
typedef struct{char version[32];uint8_t app_elf_sha256[32];}esp_app_desc_t;
typedef struct{char label[17];}esp_partition_t;
static esp_app_desc_t fixture_desc;static esp_partition_t fixture_running,fixture_boot;
static bool have_running=true,have_boot=true;static int fixture_state_result=ESP_OK;
static esp_ota_img_states_t fixture_state=ESP_OTA_IMG_VALID;static char capture[1024];
static const esp_app_desc_t *esp_app_get_description(void){return &fixture_desc;}
static const esp_partition_t *esp_ota_get_running_partition(void){return have_running?&fixture_running:NULL;}
static const esp_partition_t *esp_ota_get_boot_partition(void){return have_boot?&fixture_boot:NULL;}
static esp_err_t esp_ota_get_state_partition(const esp_partition_t*p,esp_ota_img_states_t*s){(void)p;*s=fixture_state;return fixture_state_result;}
static const char *esp_err_to_name(int err){return err==ESP_OK?"ESP_OK":"ESP_ERR_INVALID_STATE";}
static int esp_reset_reason(void){return INT32_MAX;}
static size_t heap_caps_get_free_size(int c){(void)c;return UINT32_MAX;}
static size_t heap_caps_get_largest_free_block(int c){(void)c;return UINT32_MAX;}
static void native_write_all(const char *s){assert(strlen(s)<sizeof(capture));memcpy(capture,s,strlen(s)+1);}
''' + version_define + '\n' + function(source,'native_printf') + '\n' + function(source,'boot_state_name') + '\n' + function(source,'print_boot_status') + r'''
int main(void){
 const char *names[]={"new","pending_verify","valid","invalid","aborted"};
 for(int i=0;i<5;i++)assert(strcmp(boot_state_name((esp_ota_img_states_t)i),names[i])==0);
 assert(strcmp(boot_state_name(ESP_OTA_IMG_UNDEFINED),"undefined")==0);
 assert(strcmp(boot_state_name((esp_ota_img_states_t)INT32_MAX),"unknown")==0);
 memset(fixture_desc.version,'v',31);fixture_desc.version[31]=0;memset(fixture_running.label,'r',16);fixture_running.label[16]=0;memset(fixture_boot.label,'b',16);fixture_boot.label[16]=0;
 for(unsigned i=0;i<32;i++)fixture_desc.app_elf_sha256[i]=(uint8_t)i;
 fixture_state=ESP_OTA_IMG_PENDING_VERIFY;print_boot_status();
 assert(strstr(capture,"running_state=pending_verify state_code=1 state_err=ESP_OK"));
 assert(strstr(capture,"image_id=000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f"));
 assert(strstr(capture,"public_version=" SCEPTER_FW_VERSION "\n"));
 assert(strstr(capture,"internal_free=4294967295 internal_largest=4294967295 dma_largest=4294967295 psram_free=4294967295"));
 size_t maximum=strlen(capture);assert(maximum<512);
 fixture_state_result=ESP_ERR_INVALID_STATE;print_boot_status();assert(strstr(capture,"running_state=unknown state_code=-1 state_err=ESP_ERR_INVALID_STATE"));
 if(strlen(capture)>maximum)maximum=strlen(capture);
 fixture_state_result=ESP_OK;fixture_state=(esp_ota_img_states_t)INT32_MAX;print_boot_status();assert(strstr(capture,"running_state=unknown state_code=2147483647"));
 if(strlen(capture)>maximum)maximum=strlen(capture);
 have_running=have_boot=false;print_boot_status();assert(strstr(capture,"running=unknown boot=unknown running_state=unknown state_code=-1"));
 assert(maximum<512);printf("actual BOOTSTATUS ELF identity/state/bounds PASS; maximum=%zu (ESP mocks)\n",maximum);
 return 0;
}
'''
    assert 'actual BOOTSTATUS' in compile_run(tmp_path,code)
