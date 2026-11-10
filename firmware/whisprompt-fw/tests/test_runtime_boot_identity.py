"""Actual immutable identity publication, independent of optional profiling."""
import os
from pathlib import Path
import subprocess

import pytest

from test_audio_recovery_042 import _build_host_executable
from test_dongle_transport import function


ROOT = Path(os.environ.get("FIRMWARE_LOG_BOUNDS_SOURCE_ROOT", Path(__file__).resolve().parents[1]))
DEPS = Path(os.environ.get("FIRMWARE_LOG_BOUNDS_DEPENDENCY_ROOT", ROOT))


@pytest.mark.parametrize("external", [0, 1])
def test_real_identity_survives_optional_profiler_failure_and_remains_immutable(tmp_path, external):
    source = (ROOT / "components/dual_end_runtime/dual_end_runtime.c").read_text(encoding="utf-8")
    code = f"#define CONFIG_SPIRAM {external}\n#define CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM {external}\n" + r'''
#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdatomic.h>
#include <string.h>
#include "dual_end_perf_format.h"
#define ESP_OK 0
#define ESP_ERR_NO_MEM -1
#define ESP_ERR_INVALID_STATE -2
#define ESP_ERR_INVALID_ARG -3
#define ESP_MAC_WIFI_STA 0
#define SCEPTER_ROLE_SCEPTER 1
#define RUNTIME_QUEUE_DEPTH 4
#define RUNTIME_LOGGER_STACK_BYTES 6144
#define RUNTIME_LOGGER_PRIORITY 2
#define DUAL_END_COMM_TASK_CORE 0
#define pdPASS 1
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define MALLOC_CAP_SPIRAM 4
#define MALLOC_CAP_8BIT 2
typedef int scepter_role_t,esp_err_t,BaseType_t;
typedef void *TaskHandle_t,*QueueHandle_t;
typedef struct{unsigned cores;}esp_chip_info_t;
typedef struct{unsigned value;}runtime_snapshot_t;
typedef struct{char version[32];uint8_t app_elf_sha256[32];}esp_app_desc_t;
static bool s_attempted;
static atomic_bool s_enabled,s_identity_ready;
static QueueHandle_t s_snapshots;
static uint32_t s_boot_id;
static scepter_role_t s_role;
static esp_chip_info_t s_chip;
static char s_fw[32],s_device_id[24],s_elf_sha256[65];
static esp_app_desc_t app_description={"app-0.5.20",{0xab}};
static bool queue_ok=true,task_ok=true;
static unsigned random_calls,queues,creates,deletes,notifications;
''' + function(source, "dual_end_runtime_set_boot_id") + "\n" + function(source, "dual_end_runtime_identity_boot_id") + "\n" + function(source, "dual_end_runtime_boot_id") + r'''
static esp_err_t dual_end_log_transport_start(scepter_role_t role){assert(role==1);return ESP_OK;}
static uint32_t esp_random(void){return random_calls++?UINT32_MAX:0;}
static const esp_app_desc_t *esp_app_get_description(void){return &app_description;}
static void esp_chip_info(esp_chip_info_t *out){out->cores=2;}
static int esp_read_mac(uint8_t *mac,int type){assert(type==0);memset(mac,1,6);return ESP_OK;}
static void runtime_logger(void *arg){(void)arg;}
static void runtime_log_identity(int core){assert(core==-1 && !dual_end_runtime_boot_id() && dual_end_runtime_identity_boot_id());}
static void *xQueueCreate(unsigned count,size_t bytes){
    assert(count==4 && bytes==sizeof(runtime_snapshot_t));
    assert(dual_end_runtime_identity_boot_id() && !dual_end_runtime_boot_id());++queues;
    return queue_ok?(void *)(uintptr_t)3:NULL;
}
int xTaskCreatePinnedToCore(void(*function)(void *),const char *name,unsigned stack,
    void *arg,unsigned priority,TaskHandle_t *handle,unsigned core){
    (void)name;assert(function==runtime_logger && stack==6144 && !arg && priority==2 && core==0);
    assert(dual_end_runtime_identity_boot_id() && !dual_end_runtime_boot_id());++creates;
    if(task_ok)*handle=(void *)(uintptr_t)4;return task_ok?pdPASS:0;
}
int xTaskCreatePinnedToCoreWithCaps(void(*function)(void *),const char *name,unsigned stack,
    void *arg,unsigned priority,TaskHandle_t *handle,unsigned core,unsigned caps){
    assert(caps==6);return xTaskCreatePinnedToCore(function,name,stack,arg,priority,handle,core);
}
static void vQueueDelete(void *queue){assert(queue==s_snapshots && !dual_end_runtime_boot_id());++deletes;}
static void xTaskNotifyGive(TaskHandle_t task){assert(task && dual_end_runtime_boot_id()==dual_end_runtime_identity_boot_id());++notifications;}
''' + function(source, "runtime_format_build_info") + "\n" + function(source, "dual_end_runtime_start") + r'''
static void reset_case(void){
    s_attempted=false;s_snapshots=NULL;s_boot_id=0;queue_ok=task_ok=true;
    atomic_store(&s_enabled,false);atomic_store(&s_identity_ready,false);
    random_calls=queues=creates=deletes=notifications=0;
}
static void verify_identity(uint32_t expected,bool profiling){
    assert(dual_end_runtime_identity_boot_id()==expected);
    assert(dual_end_runtime_boot_id()==(profiling?expected:0));
    assert(dual_end_runtime_set_boot_id(7)==ESP_ERR_INVALID_STATE);
    assert(dual_end_runtime_identity_boot_id()==expected);
    char output[181],identity[40];
    assert(runtime_format_build_info(output,sizeof(output)));
    snprintf(identity,sizeof(identity),"\"boot_id\":\"%08" PRIx32 "\"",expected);
    assert(strstr(output,identity));
}
int main(void){
    reset_case();assert(!dual_end_runtime_identity_boot_id() && !dual_end_runtime_boot_id());
    assert(dual_end_runtime_set_boot_id(0)==ESP_ERR_INVALID_ARG);
    assert(dual_end_runtime_set_boot_id(UINT32_C(0x00abcdef))==ESP_OK);
    assert(!dual_end_runtime_identity_boot_id());queue_ok=false;
    assert(dual_end_runtime_start(1)==ESP_ERR_NO_MEM && queues==1 && !creates);
    verify_identity(UINT32_C(0x00abcdef),false);
    assert(dual_end_runtime_start(1)==ESP_ERR_INVALID_STATE);
    verify_identity(UINT32_C(0x00abcdef),false);
    reset_case();assert(dual_end_runtime_set_boot_id(UINT32_C(0x00abcdef))==ESP_OK);task_ok=false;
    assert(dual_end_runtime_start(1)==ESP_ERR_NO_MEM && creates==1 && deletes==1 && !s_snapshots);
    verify_identity(UINT32_C(0x00abcdef),false);
    reset_case();assert(dual_end_runtime_set_boot_id(UINT32_C(0x00abcdef))==ESP_OK);
    assert(dual_end_runtime_start(1)==ESP_OK && !random_calls && notifications==1);
    verify_identity(UINT32_C(0x00abcdef),true);
    assert(dual_end_runtime_start(1)==ESP_OK && notifications==1);
    reset_case();assert(dual_end_runtime_start(1)==ESP_OK && random_calls==2);
    verify_identity(UINT32_MAX,true);
    puts("PASS shared runtime identity survives optional queue/task failure and preserves legacy readiness");return 0;
}
'''
    harness = tmp_path / "runtime_boot_identity.c"
    harness.write_text(code, encoding="utf-8")
    executable, env = _build_host_executable([harness], tmp_path,
        extra_includes=(DEPS / "components/dual_end_runtime/include",))
    result = subprocess.run([str(executable)], cwd=tmp_path, env=env, text=True,
                            capture_output=True, check=True, timeout=10)
    assert "PASS shared runtime identity" in result.stdout
