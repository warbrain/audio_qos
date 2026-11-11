"""Run production cache parsers/formatters on the host without a device.

Compile against the existing host compiler discovery helper, including on
Windows. Fixed-clock fixtures prove immutable multipart leases and expiry.
"""
from pathlib import Path
import json
from unittest.mock import patch

from test_audio_recovery_042 import ROOT as REFERENCE_ROOT, _build_host_executable, _run
from test_runtime_codec import function


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "components/dual_end_runtime"
MAIN = ROOT / "dongle/main"


def run_host(tmp_path, code, sources=(), includes=()):
    harness = tmp_path / "snapshot_consistency.c"
    harness.write_text(code, encoding="utf-8")
    with patch.dict("os.environ", {name: str(tmp_path) for name in ("TMP", "TEMP", "TMPDIR")}):
        executable, env = _build_host_executable(
            [harness, *sources], tmp_path,
            extra_includes=(*includes, REFERENCE_ROOT / "components/dual_end_runtime/include"),
        )
    return _run([str(executable)], cwd=tmp_path, env=env).stdout


def runtime_prefix(source):
    lease_start = source.index("typedef struct {\n    bool valid;\n    uint32_t token;\n    uint8_t stage;")
    lease_end = source.index("static dual_end_runtime_io_probe_t s_io_probe;", lease_start)
    return r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#define DUAL_END_STAGE_COUNT 4U
#define DUAL_END_RESOURCE_PARTS 10U
#define DUAL_END_PERF_PART_COUNT 13U
#define DUAL_END_PERF_JSON_MAX_BYTES 180U
#define DUAL_END_PERF_JSON_BUFFER_BYTES 181U
#define RUNTIME_SNAPSHOT_LEASE_US UINT64_C(10000000)
#define atomic_load_explicit(p,m) ((void)(m),*(p))
#define memory_order_acquire 0
typedef struct { uint32_t window_seq; } dual_end_perf_record_t;
typedef struct { uint32_t sequence; } dual_end_resource_snapshot_t;
typedef int portMUX_TYPE;
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
static portMUX_TYPE s_resource_lock;
static dual_end_resource_snapshot_t s_resource_cache;
static bool s_resource_ready, s_perf_ready[DUAL_END_STAGE_COUNT], s_identity_ready=true;
static dual_end_perf_record_t s_perf_cache[DUAL_END_STAGE_COUNT];
static uint64_t fixture_clock_us=100;
static int64_t esp_timer_get_time(void){return (int64_t)fixture_clock_us;}
static uint32_t s_boot_id=UINT32_MAX,s_role=1;
static char s_elf_sha256[65]="0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
static size_t dual_end_resource_format(const dual_end_resource_snapshot_t *r,
    unsigned part, char *out, size_t capacity) {
    return (size_t)snprintf(out, capacity, "{\"resource\":%u,\"part\":%u}", r->sequence, part);
}
static size_t dual_end_perf_format_part(const dual_end_perf_record_t *r,
    unsigned part, char *out, size_t capacity) {
    return (size_t)snprintf(out, capacity, "{\"window\":%u,\"part\":%u}", r->window_seq, part);
}
''' + source[lease_start:lease_end] + "\n" + function(source, "runtime_format_build_info") + "\n" + function(source, "runtime_parse_numbers") + "\n" + function(source, "dual_end_runtime_command")


def test_resource_perf_snapshot_fixed_lifetimes_and_build_identity(tmp_path):
    source = (RUNTIME / "dual_end_runtime.c").read_text(encoding="utf-8")
    output = run_host(tmp_path, runtime_prefix(source) + r'''
int main(void){
 char reply[384];
 s_resource_ready=true;s_resource_cache.sequence=41;
 assert(dual_end_runtime_command("TELEMETRY 7 1",reply,sizeof(reply)));
 assert(strstr(reply,"snapshot_lease_mismatch"));
 assert(dual_end_runtime_command("TELEMETRY 7 0",reply,sizeof(reply)));
 assert(strstr(reply,"\"resource\":41"));
 s_resource_cache.sequence=42;
 for(unsigned part=0;part<DUAL_END_RESOURCE_PARTS;++part){
  char command[64];snprintf(command,sizeof(command),"TELEMETRY 7 %u",part);
  assert(dual_end_runtime_command(command,reply,sizeof(reply)));
  assert(strstr(reply,"OK TELEMETRY token=7") && strstr(reply,"\"resource\":41"));
 }
 fixture_clock_us=9999999;
 assert(dual_end_runtime_command("TELEMETRY 7 0",reply,sizeof(reply)));
 assert(strstr(reply,"\"resource\":41")); /* retry does not prolong lease */
 fixture_clock_us=10000100;
 assert(dual_end_runtime_command("TELEMETRY 7 9",reply,sizeof(reply)));
 assert(strstr(reply,"snapshot_lease_mismatch"));
 assert(dual_end_runtime_command("TELEMETRY 7 0",reply,sizeof(reply)));
 assert(strstr(reply,"\"resource\":42"));
 assert(dual_end_runtime_command("TELEMETRY 8 1",reply,sizeof(reply)));
 assert(strstr(reply,"snapshot_lease_mismatch"));
 fixture_clock_us=1; /* backwards clock invalidates, never wraps to young */
 assert(dual_end_runtime_command("TELEMETRY 7 1",reply,sizeof(reply)));
 assert(strstr(reply,"snapshot_lease_mismatch"));
 s_perf_ready[1]=true;s_perf_cache[1].window_seq=101;
 assert(dual_end_runtime_command("PERF_SNAPSHOT 77 1 0",reply,sizeof(reply)));
 s_perf_cache[1].window_seq=202;
 assert(dual_end_runtime_command("PERF_SNAPSHOT 77 1 12",reply,sizeof(reply)));
 assert(strstr(reply,"\"window\":101"));
 assert(dual_end_runtime_command("PERF 88 1 12",reply,sizeof(reply)));
 assert(strstr(reply,"\"window\":202"));
 fixture_clock_us+=RUNTIME_SNAPSHOT_LEASE_US;
 assert(dual_end_runtime_command("PERF_SNAPSHOT 77 1 12",reply,sizeof(reply)));
 assert(strstr(reply,"snapshot_lease_mismatch"));
 assert(dual_end_runtime_command("PERF_SNAPSHOT 77 1 0",reply,sizeof(reply)));
 assert(strstr(reply,"\"window\":202"));
 assert(dual_end_runtime_command("BUILD_INFO 4294967295 0",reply,sizeof(reply)));
 assert(strstr(reply,"OK BUILD_INFO token=4294967295 "));
 assert(strstr(reply,s_elf_sha256) && strlen(reply)<sizeof(reply));
 puts(strchr(reply,'{'));
 assert(dual_end_runtime_command("BUILD_INFO 1 1",reply,sizeof(reply)));
 assert(strcmp(reply,"ERR BUILD_INFO arguments\n")==0);
 s_identity_ready=false;
 assert(dual_end_runtime_command("BUILD_INFO 1 0",reply,sizeof(reply)));
 assert(strstr(reply,"not_ready"));
 puts("resource/perf lease TTL and build SHA PASS");return 0;
}
''')
    build = json.loads(output.splitlines()[0])
    assert build["elf_sha256"] == "0123456789abcdef" * 4
    assert build["boot_id"] == "ffffffff" and build["part_count"] == 1
    assert len(output.splitlines()[0]) <= 180
    assert "resource/perf lease TTL and build SHA PASS" in output


def test_startup_boot_registration_rejects_zero_and_late_mutation(tmp_path):
    source = (RUNTIME / "dual_end_runtime.c").read_text(encoding="utf-8")
    output = run_host(tmp_path, r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 1
#define ESP_ERR_INVALID_ARG 2
static bool s_attempted;
static uint32_t s_boot_id;
''' + function(source, "dual_end_runtime_set_boot_id") + r'''
int main(void){
 assert(dual_end_runtime_set_boot_id(0)==ESP_ERR_INVALID_ARG && !s_boot_id);
 assert(dual_end_runtime_set_boot_id(123)==ESP_OK && s_boot_id==123);
 s_attempted=true;
 assert(dual_end_runtime_set_boot_id(456)==ESP_ERR_INVALID_STATE && s_boot_id==123);
 puts("startup boot registration PASS");return 0;
}
''')
    assert "startup boot registration PASS" in output
    app = (ROOT / "scepter/main/app_main.c").read_text(encoding="utf-8")
    assert "dual_end_runtime_set_boot_id(s_startup_diag_boot)" in app


def test_dongle_v1_v2_leases_remain_immutable_and_context_invalidates(tmp_path):
    source = (MAIN / "dongle_diagnostics.c").read_text(encoding="utf-8")
    types_start = source.index("typedef struct {\n    uint32_t run_id;")
    types_end = source.index("static portMUX_TYPE s_diagnostics_lock", types_start)
    lease_start = source.index("typedef struct {\n    bool valid;\n    uint32_t token;")
    lease_end = source.index("static uint64_t positive_us", lease_start)
    code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "dongle_diagnostics_format.h"
#define DONGLE_DIAGNOSTICS_LEASE_US UINT64_C(10000000)
typedef int portMUX_TYPE;
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
static portMUX_TYPE s_diagnostics_lock;
static uint64_t fixture_clock_us=100;
static int64_t esp_timer_get_time(void){return (int64_t)fixture_clock_us;}
typedef struct { uint32_t sequence; } audio_rx_telemetry_cache_t;
static bool audio_rx_telemetry_get_latest(audio_rx_telemetry_cache_t *out){out->sequence=8;return true;}
''' + source[types_start:types_end] + r'''
static diagnostics_context_t s_context;
static diagnostics_v2_context_t s_v2_context;
static dongle_diagnostics_snapshot_t s_cache;
static bool s_cache_ready;
''' + source[lease_start:lease_end]
    for name in ("positive_us", "increment_nonzero", "response_error_for", "response_error", "set_context", "send_cached_part"):
        code += "\n" + function(source, name)
    output = run_host(tmp_path, code + r'''
int main(void){
 char reply[384];
 s_cache_ready=true;s_cache.boot_id=7;s_cache.role=1;
 s_cache.cache_sequence=11;s_cache.v2_identity_ready=true;
 assert(send_cached_part(7,1,false,reply,sizeof(reply)) && strstr(reply,"snapshot_lease_mismatch"));
 assert(send_cached_part(7,0,false,reply,sizeof(reply)) && strstr(reply,"\"q\":11"));
 s_cache.cache_sequence=22;
 assert(send_cached_part(8,0,true,reply,sizeof(reply)) && strstr(reply,"\"q\":22"));
 s_cache.cache_sequence=33;
 for(unsigned part=0;part<DONGLE_DIAGNOSTICS_PARTS;++part){
  assert(send_cached_part(7,part,false,reply,sizeof(reply)));
  assert(strstr(reply,"OK DONGLE_TELEMETRY token=7") && strstr(reply,"\"q\":11"));
 }
 for(unsigned part=0;part<DONGLE_DIAGNOSTICS_V2_PARTS;++part){
  assert(send_cached_part(8,part,true,reply,sizeof(reply)));
  assert(strstr(reply,"OK DONGLE_TELEMETRY_V2 token=8") && strstr(reply,"\"q\":22"));
 }
 assert(send_cached_part(9,1,true,reply,sizeof(reply)) && strstr(reply,"snapshot_lease_mismatch"));
 fixture_clock_us+=DONGLE_DIAGNOSTICS_LEASE_US;
 assert(send_cached_part(7,1,false,reply,sizeof(reply)) && strstr(reply,"snapshot_lease_mismatch"));
 assert(send_cached_part(8,1,true,reply,sizeof(reply)) && strstr(reply,"snapshot_lease_mismatch"));
 s_cache.v2_identity_mixed=true;
 assert(send_cached_part(8,0,true,reply,sizeof(reply)) && strstr(reply,"mixed_identity"));
 assert(!s_snapshot_leases[1].valid); /* pending/error zero must not freeze */
 s_cache.v2_identity_mixed=false;
 assert(send_cached_part(8,0,true,reply,sizeof(reply)) && strstr(reply,"\"q\":33"));
 assert(send_cached_part(7,0,false,reply,sizeof(reply)) && strstr(reply,"\"q\":33"));
 assert(set_context(19,2,3,reply,sizeof(reply)) && strstr(reply,"OK DIAG_CONTEXT"));
 assert(!s_snapshot_leases[0].valid && !s_snapshot_leases[1].valid);
 assert(send_cached_part(8,1,true,reply,sizeof(reply)) && strstr(reply,"snapshot_lease_mismatch"));
 assert(send_cached_part(8,0,true,reply,sizeof(reply)) && strstr(reply,"context_pending"));
 assert(!s_snapshot_leases[1].valid);
 s_cache.v2_run_id=2;s_cache.v2_arm_id=3;
 s_cache.v2_context_sequence=s_v2_context.sequence;
 s_cache.v2_context_ready=true;s_cache.v2_rx_cache_sequence=9;
 s_v2_context.identity_bound=true;
 s_cache.v2_rx_session_id=s_v2_context.bound_session_id=4;
 s_cache.v2_rx_session_generation=s_v2_context.bound_rx_generation=5;
 s_cache.v2_uac_session_generation=s_v2_context.bound_uac_generation=6;
 assert(send_cached_part(8,0,true,reply,sizeof(reply)) && strstr(reply,"OK DONGLE_TELEMETRY_V2"));
 fixture_clock_us=1;
 assert(send_cached_part(8,1,true,reply,sizeof(reply)) && strstr(reply,"snapshot_lease_mismatch"));
 puts("DIAG frozen v1/v2, TTL, pending retry and context invalidation PASS");return 0;
}
''', sources=(MAIN / "dongle_diagnostics_format.c",), includes=(MAIN,))
    assert "pending retry and context invalidation PASS" in output


def test_cdc_log_health_maximum_counters_are_complete_and_bounded(tmp_path):
    output = run_host(tmp_path, r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "dongle_diagnostics_format.h"
int main(void){
 dongle_diagnostics_snapshot_t s={.boot_id=UINT32_MAX,.role=1,.cache_sequence=UINT32_MAX,
  .sampled_us=UINT64_MAX,.cdc_log_attempted=UINT32_MAX,.cdc_log_enqueued=UINT32_MAX,
  .cdc_log_dropped=UINT32_MAX,.cdc_log_truncated=UINT32_MAX,.cdc_log_write_failed=UINT32_MAX};
 char line[181],small[10];
 for(unsigned part=0;part<DONGLE_CDC_LOG_HEALTH_PARTS;++part){
  size_t n=dongle_diagnostics_cdc_log_format_part(&s,part,line,sizeof(line));
  assert(n>0 && n<=180U && n==strlen(line));puts(line);
  assert(!dongle_diagnostics_cdc_log_format_part(&s,part,small,sizeof(small)) && !small[0]);
 }
 assert(!dongle_diagnostics_cdc_log_format_part(&s,DONGLE_CDC_LOG_HEALTH_PARTS,line,sizeof(line)) && !line[0]);
 s.boot_id=0;
 assert(!dongle_diagnostics_cdc_log_format_part(&s,0,line,sizeof(line)) && !line[0]);
 return 0;
}
''', sources=(MAIN / "dongle_diagnostics_format.c",), includes=(MAIN,))
    rows = [json.loads(line) for line in output.splitlines()]
    assert [r["part"] for r in rows] == list(range(6))
    assert rows[0]["sampled_us"] == "18446744073709551615"
    for part, key in enumerate(("attempted", "enqueued", "dropped", "truncated", "write_failed"), 1):
        assert rows[part][key] == 4294967295


def test_codec5_shared_spatial_and_rx_formatters_preserve_optional_codec(tmp_path):
    output = run_host(tmp_path, r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "dual_end_telemetry_format.h"
int main(void){
 char line[181];
 dual_end_spatial_telemetry_t spatial={.boot_id=1,.role=0,.codec=5,.codec_known=true};
 dual_end_rx_telemetry_t rx={.boot_id=1,.role=1,.codec=5,.codec_known=true};
 for(unsigned p=0;p<DUAL_END_SPATIAL_TELEMETRY_PARTS;++p)
  assert(dual_end_spatial_telemetry_format(&spatial,p,line,sizeof(line))>0);
 assert(dual_end_spatial_telemetry_format(&spatial,1,line,sizeof(line))>0 && strstr(line,"\"codec\":5"));
 for(unsigned p=0;p<DUAL_END_RX_TELEMETRY_PARTS;++p)
  assert(dual_end_rx_telemetry_format(&rx,p,line,sizeof(line))>0);
 assert(dual_end_rx_telemetry_format(&rx,1,line,sizeof(line))>0 && strstr(line,"\"codec\":5"));
 spatial.codec=6;rx.codec=6;
 assert(!dual_end_spatial_telemetry_format(&spatial,1,line,sizeof(line)) && !line[0]);
 assert(!dual_end_rx_telemetry_format(&rx,1,line,sizeof(line)) && !line[0]);
 spatial.codec_known=false;rx.codec_known=false;
 assert(dual_end_spatial_telemetry_format(&spatial,1,line,sizeof(line))>0 && strstr(line,"\"codec\":null"));
 assert(dual_end_rx_telemetry_format(&rx,1,line,sizeof(line))>0 && strstr(line,"\"codec\":null"));
 puts("codec5 optional and unknown compatibility PASS");return 0;
}
''', sources=(RUNTIME / "dual_end_telemetry_format.c",))
    assert "codec5 optional and unknown compatibility PASS" in output
