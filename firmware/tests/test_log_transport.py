"""Compile the actual optional USB log sink with RTOS/USB fault stubs.

No real USB, audio, OTA, console read, DTR, system setting or firmware operation.
Temporary headers replace only external ESP APIs; the queue hook, writer,
initialization/cleanup and the existing native command writer are actual C.
"""
from pathlib import Path

import pytest

from test_audio_recovery_042 import _build_host_executable, _run
from test_runtime_codec import function

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'components/dual_end_runtime'

STUBS = r'''
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_STATE 0x103
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
typedef struct {uint8_t *storage;unsigned length,item_bytes,used,read,write;} StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define MALLOC_CAP_8BIT 1U
#define MALLOC_CAP_SPIRAM 2U
#define MALLOC_CAP_INTERNAL 4U
void *heap_caps_malloc(size_t,uint32_t);
void heap_caps_free(void *);
QueueHandle_t xQueueCreateStatic(UBaseType_t,UBaseType_t,uint8_t *,StaticQueue_t *);
BaseType_t xQueueSend(QueueHandle_t,const void *,TickType_t);
BaseType_t xQueueReceive(QueueHandle_t,void *,TickType_t);
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t);
void vQueueDelete(QueueHandle_t);
BaseType_t xTaskCreatePinnedToCoreWithCaps(TaskFunction_t,const char *,uint32_t,void *,UBaseType_t,TaskHandle_t *,BaseType_t,UBaseType_t);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t,const char *,uint32_t,void *,UBaseType_t,TaskHandle_t *,BaseType_t);
void vTaskDeleteWithCaps(TaskHandle_t);
void vTaskDelete(TaskHandle_t);
uint32_t ulTaskNotifyTake(BaseType_t,TickType_t);
void xTaskNotifyGive(TaskHandle_t);
typedef int (*vprintf_like_t)(const char *,va_list);
vprintf_like_t esp_log_set_vprintf(vprintf_like_t);
typedef struct {uint32_t tx_buffer_size,rx_buffer_size;} usb_serial_jtag_driver_config_t;
bool usb_serial_jtag_is_driver_installed(void);
esp_err_t usb_serial_jtag_driver_install(usb_serial_jtag_driver_config_t *);
void usb_serial_jtag_vfs_use_driver(void);
int usb_serial_jtag_write_bytes(const void *,size_t,TickType_t);
void transport_test_flockfile(FILE *);
void transport_test_funlockfile(FILE *);
#define flockfile transport_test_flockfile
#define funlockfile transport_test_funlockfile
'''


@pytest.fixture(scope='module')
def log_transport_build(tmp_path_factory, request):
    usb, version, psram = request.param
    build_dir = tmp_path_factory.mktemp(f'log_transport_usb{usb}_v{version}_psram{psram}')
    stubs = build_dir / 'stubs'
    stubs.mkdir()
    (stubs / 'transport_test_stubs.h').write_text(STUBS, encoding='utf-8')
    for name in ['esp_err.h', 'esp_heap_caps.h', 'esp_log.h',
                 'driver/usb_serial_jtag.h', 'driver/usb_serial_jtag_vfs.h',
                 'freertos/FreeRTOS.h', 'freertos/idf_additions.h',
                 'freertos/queue.h', 'freertos/task.h']:
        path = stubs / name
        path.parent.mkdir(exist_ok=True, parents=True)
        path.write_text('#pragma once\n#include "transport_test_stubs.h"\n', encoding='utf-8')
    (stubs / 'sdkconfig.h').write_text(
        f'#define CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG {usb}\n'
        f'#define CONFIG_LOG_VERSION {version}\n'
        f'#define CONFIG_SPIRAM {psram}\n'
        f'#define CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM {psram}\n', encoding='utf-8')
    pairing = (ROOT / 'scepter/components/dual_end_pairing/dual_end_pairing.c').read_text(encoding='utf-8')
    harness = build_dir / 'test_log_transport.c'
    transport_source = (ROOT / 'tests/test_log_transport.c').read_text(encoding='utf-8')
    native_helper = '\n' + function(pairing, 'native_write_all') + '\n' if usb and version == 1 else ''
    harness.write_text(transport_source + native_helper, encoding='utf-8')
    executable, env = _build_host_executable([harness], build_dir,
        extra_includes=(stubs, RUNTIME / 'include', RUNTIME))
    return executable, env, build_dir, bool(usb and version == 1)


SCENARIOS = [
    'whole', 'format', 'partial', 'boundary', 'backend', 'saturate',
    'alloc_fail', 'queue_fail', 'task_fail', 'driver_fail', 'new_driver', 'noop',
]


@pytest.mark.parametrize('log_transport_build,scenario',
    [(config, scenario) for config in ((1, 1, 1), (1, 1, 0)) for scenario in SCENARIOS] +
    [((0, 1, 1), 'noop'), ((1, 2, 1), 'noop')], indirect=['log_transport_build'])
def test_actual_optional_usb_log_transport(log_transport_build, scenario):
    executable, env, build_dir, active = log_transport_build
    assert active or scenario == 'noop'
    result = _run([str(executable), scenario], cwd=build_dir, env=env)
    assert 'PASS:' in result.stdout

