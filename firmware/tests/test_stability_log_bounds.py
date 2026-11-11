"""Saturated production counter logs must fit the actual bounded transport."""
from pathlib import Path
import ast
import json
import os
import re
import subprocess
import sys

import pytest

from test_audio_recovery_042 import _build_host_executable, _run
from test_dongle_transport import function


ROOT = Path(os.environ.get("FIRMWARE_LOG_BOUNDS_SOURCE_ROOT", Path(__file__).resolve().parents[1]))
DEPS = Path(os.environ.get("FIRMWARE_LOG_BOUNDS_DEPENDENCY_ROOT", ROOT))
RUNTIME = DEPS / "components/dual_end_runtime"


def test_actual_stability_logs_fit_with_saturated_counters_and_prefix(tmp_path):
    stubs = tmp_path / "stubs"
    stubs.mkdir()
    (stubs / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n", encoding="utf-8")
    source = (ROOT / "scepter/components/dual_end_espnow_link/dual_end_espnow_link.c").read_text(encoding="utf-8")
    code = r'''
#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "dual_end_log_transport.h"
typedef struct {
    uint64_t calls,submitted,mac_ok,mac_fail,immediate_fail,unresolved;
} dual_end_tx_attempt_stats_t;
static dual_end_tx_attempt_stats_t s_stability_first,s_stability_retry,s_stability_repair,s_audio_control_attempts;
static esp_err_t s_audio_last_immediate_error,s_audio_control_last_submit_error;
static uint32_t s_audio_no_mem_errors,s_audio_control_submit_failures,s_audio_control_no_mem_retries;
static int s_tx_stats_lock;
static unsigned locks,records,first_records,repair_records,control_records;
static const char *TAG="scepter_espnow";
static uint64_t seen;
static const char *fields[]={
    " original_first_calls=", " original_first_submitted=", " original_first_mac_ok=",
    " original_first_mac_fail=", " original_first_immediate_fail=", " original_first_unresolved=",
    " fast_retry_calls=", " fast_retry_submitted=", " fast_retry_mac_ok=", " fast_retry_mac_fail=",
    " fast_retry_immediate_fail=", " fast_retry_unresolved=", " repair_calls=", " repair_submitted=",
    " repair_mac_ok=", " repair_mac_fail=", " repair_immediate_fail=", " repair_unresolved=",
    " calls=", " submitted=", " mac_ok=", " mac_fail=", " immediate_fail=", " unresolved=",
    " no_mem_retries=", " submit_failures=", " last_error=", " last_immediate=", " no_mem_total="
};
#define portENTER_CRITICAL(lock) do{assert((lock)==&s_tx_stats_lock && !locks);++locks;}while(0)
#define portEXIT_CRITICAL(lock) do{assert((lock)==&s_tx_stats_lock && locks==1);--locks;}while(0)
static int64_t esp_timer_get_time(void){return INT64_MAX;}
static const char *esp_err_to_name(esp_err_t error){(void)error;return "ESP_ERR_INVALID_STATE";}
static void inspect_log(const char *tag,const char *fmt,...) {
    assert(strcmp(tag,"scepter_espnow")==0 && !locks);
    char line[1024];memset(line,'p',64);
    va_list args;va_start(args,fmt);
    int length=vsnprintf(line+64,sizeof(line)-64,fmt,args);va_end(args);
    assert(length>0 && 64U+(unsigned)length+1U<DUAL_END_LOG_TRANSPORT_LINE_BYTES);
    const char *payload=line+64;
    if(strstr(payload,"STABILITY_TX_A ")==payload){++first_records;assert(strstr(payload,"parts=6"));}
    if(strstr(payload,"STABILITY_TX_B ")==payload){++repair_records;assert(strstr(payload,"parts=3"));}
    if(strstr(payload,"AUDIO_CONTROL_TX ")==payload){++control_records;assert(strstr(payload,"parts=5"));}
    if(strstr(payload,"uptime_ms="))assert(strstr(payload,"uptime_ms=9223372036854775"));
    for(unsigned i=0;i<sizeof(fields)/sizeof(fields[0]);++i) {
        if(strstr(payload,fields[i])){assert(!(seen&(UINT64_C(1)<<i)));seen|=UINT64_C(1)<<i;}
    }
    ++records;
}
#define ESP_LOGI(tag,...) inspect_log(tag,__VA_ARGS__)
''' + function(source, "stability_log_tx") + r'''
int main(void){
    s_stability_first=(dual_end_tx_attempt_stats_t){UINT64_MAX,UINT64_MAX,UINT64_MAX,UINT64_MAX,UINT64_MAX,UINT64_MAX};
    s_stability_retry=s_stability_first;s_stability_repair=s_stability_first;s_audio_control_attempts=s_stability_first;
    s_audio_no_mem_errors=UINT32_MAX;s_audio_control_submit_failures=UINT32_MAX;s_audio_control_no_mem_retries=UINT32_MAX;
    stability_log_tx();
    assert(records==15 && first_records==6 && repair_records==3 && control_records==5 && !locks);
    assert(seen==((UINT64_C(1)<<(sizeof(fields)/sizeof(fields[0])))-1U));
    puts("PASS saturated production logs, legacy field coverage and common sample time");return 0;
}
'''
    harness = tmp_path / "stability_log_bounds.c"
    harness.write_text(code, encoding="utf-8")
    executable, env = _build_host_executable([harness], tmp_path, extra_includes=(stubs, RUNTIME / "include"))
    assert "PASS saturated" in _run([str(executable)], cwd=tmp_path, env=env).stdout


def _source(role):
    return (ROOT / role / "components/dual_end_espnow_link/dual_end_espnow_link.c").read_text(encoding="utf-8")


def _arguments(text):
    result, start, depth = [], 0, 0
    quoted = escaped = False
    for index, char in enumerate(text):
        if quoted:
            if escaped: escaped = False
            elif char == "\\": escaped = True
            elif char == '"': quoted = False
        elif char == '"': quoted = True
        elif char in "([{": depth += 1
        elif char in ")]}": depth -= 1
        elif char == "," and depth == 0:
            result.append(text[start:index].strip()); start = index + 1
    return [*result, text[start:].strip()]


def _calls(text):
    for match in re.finditer(r"ESP_LOG[EWIDV]\(", text):
        opening = cursor = match.end() - 1
        depth, quoted, escaped = 1, False, False
        while depth:
            cursor += 1
            char = text[cursor]
            if quoted:
                if escaped: escaped = False
                elif char == "\\": escaped = True
                elif char == '"': quoted = False
            elif char == '"': quoted = True
            elif char == "(": depth += 1
            elif char == ")": depth -= 1
        yield text.count("\n", 0, match.start()) + 1, _arguments(text[opening + 1:cursor])


_TOKENS = re.compile(r'"(?:[^"\\]|\\.)*"|PRI[duxXi]\d+|MACSTR')
_SPECS = re.compile(r'%(?:[-+ #0]*)(?:\d+)?(?:\.\d+)?(hh|ll|l|z|j)?(?:\{(PRI[duxXi]\d+)\}|([duxXiscfp]))')


def _logical(expression):
    assert not _TOKENS.sub("", expression).strip(), expression
    return "".join(ast.literal_eval(value) if value.startswith('"') else
                   "%02x:%02x:%02x:%02x:%02x:%02x" if value == "MACSTR" else "{" + value + "}"
                   for value in _TOKENS.findall(expression))


def _maximum(spec):
    length, macro, kind = spec.groups()
    if macro:
        return ("INT" if macro[3] in "di" else "UINT") + macro[4:] + ("_MIN" if macro[3] in "di" else "_MAX")
    if kind == "s": return json.dumps("x" * 64)
    if kind == "c": return "'x'"
    if kind == "p": return "(void *)(uintptr_t)UINTPTR_MAX"
    if kind == "f": return "1.7976931348623157e308"
    if length == "z": return "SIZE_MAX"
    if length == "ll": return "LLONG_MIN" if kind in "di" else "ULLONG_MAX"
    if length == "l": return "LONG_MIN" if kind in "di" else "ULONG_MAX"
    return "INT32_MIN" if kind in "di" else "UINT32_MAX"


@pytest.mark.parametrize("role", ["scepter", "dongle"])
def test_every_real_link_log_format_fits_the_complete_record_at_counter_limits(tmp_path, role):
    """Format production literals in C; do not reproduce the split algorithm."""
    source = _source(role)
    statements = []
    for line, args in _calls(source):
        logical = _logical(args[1])
        values = [_maximum(spec) for spec in _SPECS.finditer(logical)]
        assert len(values) == len(args) - 2, (role, line, args)
        statements.append(f'    inspect("{role}:{line}", {args[1]}' +
                          (", " + ", ".join(values) if values else "") + ");")
    code = r'''
#include <assert.h>
#include <inttypes.h>
#include <limits.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
static unsigned records,maximum;
static void inspect(const char *site,const char *fmt,...){
    char record[1024];va_list args;va_start(args,fmt);
    int bytes=vsnprintf(record,sizeof(record),fmt,args);va_end(args);
    if(bytes<=0 || bytes+64+1>=256){fprintf(stderr,"%s body=%d exceeds bounded record\n",site,bytes);assert(0);}
    if((unsigned)bytes>maximum)maximum=(unsigned)bytes;
    ++records;
}
int main(void){
''' + "\n".join(statements) + r'''
    assert(records>20);printf("PASS all real format literals: records=%u max_body=%u\n",records,maximum);return 0;
}
'''
    harness = tmp_path / "all_link_log_formats.c"
    harness.write_text(code, encoding="utf-8")
    executable, env = _build_host_executable([harness], tmp_path)
    result = subprocess.run([str(executable)], cwd=tmp_path, env=env, text=True,
                            capture_output=True, check=True, timeout=10)
    assert "PASS all real format literals" in result.stdout


_EXPECTED = {
    "STABILITY_LINK": "boot_ready reset_reason rate_mbps connected rssi channel suspended disconnects disconnected_ms max_disconnect_ms hop fallback heap_free heap_min",
    "RADIO_RECOVERY": "state reason generation early hard forced success fail reset_last_us reset_max_us first_cb_last_us first_cb_max_us first_cb_pending first_rx_last_us first_rx_max_us first_rx_pending",
    "tx_diag": "accepted cb debt soft late recover orphan immediate latency_us hist ch rate",
    "TX_PIPE": "queue_count queue_max_us peer_count peer_max_us prepare_count prepare_max_us api_count api_max_us callback_count callback_max_us wake_count wake_max_us",
    "tx_policy": "cb_ok_over_20_60_100ms cb_fail_over_20_60_100ms queue_wait_max_us submit_age_max_us mac_ok_after_100ms fast_retry_suppressed reserve_ms original_limit",
    "ESP-NOW": "peer source ch rendezvous auto_channel rate tx_power_qdbm hello link_timeout cb_soft/hard",
}


@pytest.mark.parametrize("role", ["scepter", "dongle"])
def test_bounded_parts_preserve_legacy_fields_and_one_sample_identity(role):
    groups = {}
    for line, args in _calls(_source(role)):
        fmt = _logical(args[1])
        event = fmt.split(" ")[0]
        if event not in _EXPECTED or " part=" not in fmt: continue
        groups.setdefault(event, []).append((fmt, args))
    expected_events = set(_EXPECTED) if role == "scepter" else set(_EXPECTED) - {"TX_PIPE", "tx_policy"}
    assert set(groups) == expected_events
    for event, parts in groups.items():
        assert [int(re.search(r" part=(\d+)", fmt).group(1)) for fmt, _ in parts] == list(range(len(parts)))
        assert all(f" parts={len(parts)} " in fmt for fmt, _ in parts)
        assert all(args[2:].count("log_uptime_ms") == 1 for _, args in parts)
        keys = [key for fmt, _ in parts for key in re.findall(r"(?:^| )([\w/]+)=", fmt)]
        fields = [key for key in keys if key not in {"role", "schema", "part", "parts", "uptime_ms", "boot_id"}]
        expected = set(_EXPECTED[event].split())
        if event == "STABILITY_LINK" and role == "scepter": expected |= {"dma_free", "dma_largest", "dma_min"}
        if event == "RADIO_RECOVERY": expected |= {"error", "congestion_late", "congestion_expired"} if role == "scepter" else {"last_error"}
        if event == "tx_diag":
            expected |= {"queue", "slots", "audio_pending", "collapse_ms", "hop", "fallback", "rescan", "paused"} if role == "scepter" else {"scan", "hop", "fallback", "rescan_req", "cb_ok_gt20_60_100", "cb_fail_gt20_60_100"}
        if event == "ESP-NOW": expected |= {"slots", "audio_pending"} if role == "scepter" else {"scan_best"}
        assert set(fields) == expected and len(fields) == len(expected), (role, event, fields, expected)


@pytest.mark.parametrize("role,expected_parts", [("scepter", 5), ("dongle", 4)])
@pytest.mark.parametrize("identity_available", [False, True])
def test_real_link_stability_snapshots_before_background_pacing(tmp_path, role, expected_parts, identity_available):
    source = _source(role)
    code = r'''
#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_DMA 2
static const char *TAG="link";
static uint32_t s_channel_switch_count=UINT32_MAX,s_channel_fallback_count=UINT32_MAX;
static uint8_t s_current_channel=255;
static uint32_t heap_free=UINT32_MAX;
static int64_t clock_us=INT64_MAX-1000000000;
static bool identity_available;
static uint32_t expected_heap,expected_hop;
static uint64_t expected_uptime;
static unsigned records,delays;
static int64_t esp_timer_get_time(void){return clock_us;}
static uint32_t dual_end_runtime_identity_boot_id(void){return identity_available?UINT32_C(0x00abcdef):0;}
static int esp_reset_reason(void){return INT32_MAX;}
static uint32_t esp_get_free_heap_size(void){return heap_free;}
static uint32_t esp_get_minimum_free_heap_size(void){return heap_free;}
''' + (r'''
static size_t heap_caps_get_free_size(unsigned caps){assert(caps==3);return heap_free;}
static size_t heap_caps_get_largest_free_block(unsigned caps){assert(caps==3);return heap_free;}
static size_t heap_caps_get_minimum_free_size(unsigned caps){assert(caps==3);return heap_free;}
''' if role == "scepter" else "") + r'''
static void vTaskDelay(unsigned ticks){assert(ticks==1);++delays;--heap_free;--s_channel_switch_count;clock_us+=1000;}
static void inspect(const char *tag,const char *fmt,...){
    assert(strcmp(tag,"link")==0);char line[1024];va_list args;va_start(args,fmt);
    int n=vsnprintf(line,sizeof(line),fmt,args);va_end(args);assert(n>0 && n+64+1<256);
    char time[80];snprintf(time,sizeof(time)," uptime_ms=%" PRIu64,expected_uptime);assert(strstr(line,time));
    char expected[80];snprintf(expected,sizeof(expected)," boot_id=%08" PRIx32,dual_end_runtime_identity_boot_id());assert(strstr(line,expected));
    if(strstr(line," boot_ready="))assert(strstr(line,identity_available?" boot_ready=1":" boot_ready=0"));
    if(strstr(line," heap_free=")){snprintf(expected,sizeof(expected)," heap_free=%" PRIu32,expected_heap);assert(strstr(line,expected));}
    if(strstr(line," heap_min=")){snprintf(expected,sizeof(expected)," heap_min=%" PRIu32,expected_heap);assert(strstr(line,expected));}
    if(strstr(line," hop=")){snprintf(expected,sizeof(expected)," hop=%" PRIu32,expected_hop);assert(strstr(line,expected));}
    ++records;
}
#define ESP_LOGI(tag,...) inspect(tag,__VA_ARGS__)
''' + function(source, "stability_log_link") + f'''
int main(void){{identity_available={str(identity_available).lower()};
expected_heap=heap_free;expected_hop=s_channel_switch_count;expected_uptime=(uint64_t)(clock_us/1000LL);
stability_log_link(true,INT32_MIN,true);
assert(records=={expected_parts} && delays=={expected_parts-1});
identity_available=true;clock_us+=INT64_C(30000000);
expected_heap=heap_free;expected_hop=s_channel_switch_count;expected_uptime=(uint64_t)(clock_us/1000LL);
stability_log_link(true,INT32_MIN,true);
assert(records==2*{expected_parts} && delays==2*{expected_parts-1});
puts("PASS real immutable link diagnostics before pacing");return 0;}}
'''
    harness = tmp_path / "real_link_stability.c"
    harness.write_text(code, encoding="utf-8")
    executable, env = _build_host_executable([harness], tmp_path)
    result = subprocess.run([str(executable)], cwd=tmp_path, env=env, text=True,
                            capture_output=True, check=True, timeout=10)
    assert "PASS real immutable link diagnostics" in result.stdout
