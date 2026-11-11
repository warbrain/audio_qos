"""Actual conditional UART route; strict host compilation, no serial IO."""
from pathlib import Path
import pytest
from test_audio_recovery_042 import _build_host_executable,_run
ROOT=Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('role',['scepter','dongle'])
@pytest.mark.parametrize('enabled',[0,1])
def test_ota_uart_route_optional_and_idempotent(tmp_path,role,enabled):
    source=(ROOT/role/'components/dual_end_ota/dual_end_ota.c').read_text(encoding='utf-8')
    begin=source.index('#if DUAL_END_OTA_ROUTE_LOGS_TO_UART0\n/* Compile UART')
    end=source.index('#endif /* DUAL_END_OTA_ROUTE_LOGS_TO_UART0 */',begin)+len('#endif /* DUAL_END_OTA_ROUTE_LOGS_TO_UART0 */')
    block=source[begin:end]
    code=f'#define DUAL_END_OTA_ROUTE_LOGS_TO_UART0 {enabled}\n'+r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#if DUAL_END_OTA_ROUTE_LOGS_TO_UART0
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 2
#define UART_NUM_0 0
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0
#define ESP_RETURN_ON_ERROR(expr,...) do{int result=(expr);if(result)return result;}while(0)
typedef int esp_err_t;
typedef struct{int baud_rate,data_bits,parity,stop_bits,flow_ctrl,source_clk;}uart_config_t;
static unsigned installs,configs,log_sets,writes;
static size_t written;
static int config_result,install_result;
static int (*log_callback)(const char*,va_list);
static int uart_param_config(int port,const uart_config_t *cfg){
 assert(port==0 && cfg->baud_rate==115200 && cfg->data_bits==8);++configs;return config_result;
}
static int uart_driver_install(int port,int rx,int tx,int queue,void *handle,int flags){
 assert(!port && rx==2048 && !tx && !queue && !handle && !flags);++installs;return install_result;
}
static void esp_log_set_vprintf(int(*callback)(const char*,va_list)){assert(callback);log_callback=callback;++log_sets;}
static int uart_write_bytes(int port,const char *bytes,size_t count){
 assert(!port && bytes && count<=255);++writes;written=count;return (int)count;
}
static int invoke_log(const char *fmt,...){va_list args;va_start(args,fmt);int result=log_callback(fmt,args);va_end(args);return result;}
#endif
'''+block+r'''
int main(void){
#if DUAL_END_OTA_ROUTE_LOGS_TO_UART0
 config_result=7;assert(route_logs_to_uart0()==7 && !installs && !s_uart0_log_route_enabled);
 config_result=0;install_result=9;assert(route_logs_to_uart0()==9 && installs==1 && !log_sets && !s_uart0_log_route_enabled);
 install_result=2;assert(route_logs_to_uart0()==0 && installs==2 && log_sets==1 && s_uart0_log_route_enabled);
 unsigned before=configs;assert(route_logs_to_uart0()==0 && configs==before && installs==2);
 assert(invoke_log("%s","hello")==5 && writes==1 && written==5);
 char long_line[401];memset(long_line,'x',400);long_line[400]=0;
 assert(invoke_log("%s",long_line)==400 && writes==2 && written==255);
#endif
 puts("optional UART route compile/runtime contract PASS (UART stubbed)");return 0;
}
'''
    p=tmp_path/'ota_route.c';p.write_text(code,encoding='utf-8')
    exe,env=_build_host_executable([p],tmp_path)
    assert 'PASS' in _run([str(exe)],cwd=tmp_path,env=env).stdout
