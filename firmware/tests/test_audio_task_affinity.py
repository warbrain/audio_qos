"""Compile real audio task policy and timing helpers on the host.

This validates configured core choices and helper arithmetic. It does not
prove where a task executes on an ESP32 or establish hardware CPU load.
"""
from pathlib import Path
import json

import pytest
from test_audio_recovery_042 import _build_host_executable, _run

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "components/dual_end_runtime"


def compile_run(tmp_path: Path, code: str, *, sources=(), includes=()):
    harness = tmp_path / "audio_runtime_contract.c"
    harness.write_text(code, encoding="utf-8")
    executable, env = _build_host_executable(
        [harness, *sources], tmp_path,
        extra_includes=(*includes, RUNTIME / "include"),
    )
    return _run([str(executable)], cwd=tmp_path, env=env).stdout


@pytest.mark.parametrize("cores,unicore", [(2, False), (2, True), (1, False)])
def test_real_task_policy_core_choices_and_safe_unicore(tmp_path: Path, cores: int, unicore: bool):
    stubs = tmp_path / "sdk_stubs"
    (stubs / "freertos").mkdir(parents=True)
    settings = f"#pragma once\n#define CONFIG_FREERTOS_NUMBER_OF_CORES {cores}\n#define configNUMBER_OF_CORES {cores}\n#define SOC_CPU_CORES_NUM {cores}\n"
    if unicore:
        settings += "#define CONFIG_FREERTOS_UNICORE 1\n"
    (stubs / "sdkconfig.h").write_text(settings, encoding="utf-8")
    (stubs / "freertos/FreeRTOS.h").write_text(settings, encoding="utf-8")
    code = f'''
#include <assert.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "dual_end_task_policy.h"
int main(void) {{
    assert(DUAL_END_AUDIO_TASK_CORE == {1 if cores == 2 and not unicore else 0});
    assert(dual_end_policy_audio_core({cores}, {str(unicore).lower()}) == DUAL_END_AUDIO_TASK_CORE);
    assert(dual_end_policy_audio_core(0, false) == 0);
    assert(dual_end_policy_audio_core(2, true) == 0);
    assert(dual_end_policy_comm_core() == 0);
    assert(DUAL_END_COMM_TASK_CORE == 0);
    assert(DUAL_END_AUDIO_TASK_CORE < {cores});
    assert(DUAL_END_COMM_TASK_CORE < {cores});
    puts("configured task cores PASS; hardware scheduling not exercised");
    return 0;
}}
'''
    assert "configured task cores PASS" in compile_run(tmp_path, code, includes=(stubs,))


PERF_PREFIX = r"""
#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include "dual_end_perf_stats.h"
void unknown(const dual_end_perf_stats_t *s) {
    uint32_t mean = 77, p95 = 88;
    assert(!dual_end_perf_summary(s, &mean, &p95));
    assert(mean == 0 && p95 == 0);
}
"""


def run_perf(tmp_path: Path, body: str):
    code = PERF_PREFIX + "\nint main(void) {\n" + body + "\nreturn 0;\n}\n"
    return compile_run(tmp_path, code, sources=(RUNTIME / "dual_end_perf_stats.c",))


def test_real_perf_session_codec_windows_reset_and_unknown(tmp_path: Path):
    out = run_perf(tmp_path, r"""
    dual_end_perf_stats_t s;
    dual_end_perf_reset(&s, 4, 100, 1);
    assert(s.codec == 4 && s.session_id == 100 && s.expected_core == 1);
    assert(s.samples == 0 && s.failures == 0 && s.observed_core == -1);
    unknown(&s); unknown(NULL);
    uint32_t mean = 99, p95 = 88;
    assert(!dual_end_perf_summary(&s, NULL, &p95) && p95 == 0);
    assert(!dual_end_perf_summary(&s, &mean, NULL) && mean == 0);
    assert(!dual_end_perf_record(NULL, 100, 1, true));
    dual_end_perf_reset(NULL, 3, 99, 0);
    assert(dual_end_perf_record(&s, 999, 1, true));
    assert(dual_end_perf_summary(&s, &mean, &p95));
    assert(mean == 999 && p95 == 1000 && s.samples == 1);
    dual_end_perf_reset(&s, 3, 101, 1);
    assert(s.codec == 3 && s.session_id == 101 && s.samples == 0);
    assert(s.sum_us == 0 && s.max_us == 0 && s.min_us == UINT32_MAX);
    assert(s.failures == 0 && s.overflow == 0 && s.observed_core_mask == 0);
    for (unsigned i = 0; i < DUAL_END_PERF_HISTOGRAM_BUCKETS; ++i) assert(s.histogram[i] == 0);
    unknown(&s);
    assert(dual_end_perf_record(&s, 200, 0, true));
    assert(dual_end_perf_summary(&s, &mean, &p95));
    assert(mean == 200 && p95 == 250 && s.core_wrong == 1);
    dual_end_perf_reset(&s, 3, 102, 1); /* Same codec, new session clears history. */
    unknown(&s); assert(s.core_wrong == 0 && s.session_id == 102);
    assert(dual_end_perf_record(&s, 123, 1, false));
    assert(dual_end_perf_record(&s, 321, 1, false));
    assert(s.samples == 2 && s.failures == 2);
    unknown(&s); /* Failed attempts cannot establish valid codec measurements. */
    puts("session/codec/reset/unknown PASS");
    """)
    assert "session/codec/reset/unknown PASS" in out


def test_real_perf_sum_min_max_budget_histogram_and_p95(tmp_path: Path):
    out = run_perf(tmp_path, r"""
    dual_end_perf_stats_t s;
    dual_end_perf_reset(&s, 4, 1, 1);
    const uint32_t elapsed[] = {0,100,101,250,251,500,501,1000,1001,2500,2501,5000,5001,10000,10001,20000,20001};
    uint64_t total = 0;
    for (unsigned i = 0; i < sizeof(elapsed)/sizeof(elapsed[0]); ++i) {
        total += elapsed[i];
        assert(dual_end_perf_record(&s, elapsed[i], 1, i != 5));
    }
    assert(s.samples == 17 && s.failures == 1 && s.over_budget == 1);
    assert(s.sum_us == total && s.min_us == 0 && s.max_us == 20001);
    assert(s.observed_core == 1 && s.observed_core_mask == 2);
    for (unsigned i = 0; i < 8; ++i) assert(s.histogram[i] == 2);
    assert(s.histogram[8] == 1);
    uint32_t mean, p95;
    assert(dual_end_perf_summary(&s, &mean, &p95));
    assert(mean == total / 17 && p95 == UINT32_MAX); /* Last bucket means unbounded upper. */
    dual_end_perf_reset(&s, 3, 2, 1);
    for (unsigned i = 0; i < 18; ++i) assert(dual_end_perf_record(&s, 100, 1, true));
    assert(dual_end_perf_record(&s, 1000, 1, true));
    assert(dual_end_perf_record(&s, 20001, 1, true));
    assert(dual_end_perf_summary(&s, &mean, &p95));
    assert(mean == 22801 / 20 && p95 == 1000); /* ceil(95% * 20) is rank 19. */
    dual_end_perf_reset(&s, 3, 3, 1);
    for (unsigned i = 0; i < 19; ++i) assert(dual_end_perf_record(&s, 101, 1, true));
    assert(dual_end_perf_record(&s, 20000, 1, true));
    assert(dual_end_perf_summary(&s, &mean, &p95));
    assert(p95 == 250 && p95 > 101); /* Histogram upper bound, not exact sample P95. */
    assert(s.over_budget == 0); /* Exactly 20ms is not over budget. */
    puts("sum/min/max/budget/histogram/P95 upper-bound PASS");
    """)
    assert "sum/min/max/budget/histogram/P95 upper-bound PASS" in out


def test_real_perf_invalid_cores_and_durations_do_not_wrap_or_mix(tmp_path: Path):
    out = run_perf(tmp_path, r"""
    dual_end_perf_stats_t s;
    dual_end_perf_reset(&s, 3, 77, 1);
    assert(dual_end_perf_record(&s, 1, 1, true));
    assert(dual_end_perf_record(&s, 2, 0, true));
    assert(s.observed_core_mask == 3 && s.core_wrong == 1 && s.invalid_core == 0);
    for (int i = 0; i < 3; ++i) {
        int invalid[] = {-1, 2, INT_MAX};
        assert(dual_end_perf_record(&s, 3, invalid[i], true));
        assert(s.observed_core_mask == 3 && s.core_wrong == 1);
    }
    assert(s.invalid_core == 3 && s.samples == 5 && s.sum_us == 12);
    assert(!dual_end_perf_record(&s, -1, 1, false));
    assert(!dual_end_perf_record(&s, INT64_MIN, 1, false));
    assert(s.invalid_duration == 2 && s.samples == 5 && s.failures == 0 && s.sum_us == 12);
    dual_end_perf_reset(&s, 4, 78, 0);
    assert(dual_end_perf_record(&s, 0, 0, true));
    assert(s.invalid_core == 0 && s.invalid_duration == 0 && s.core_wrong == 0);
    assert(s.observed_core_mask == 1);
    uint32_t mean, p95;
    assert(dual_end_perf_summary(&s, &mean, &p95) && mean == 0 && p95 == 100);
    puts("core/duration rejection and core-mask isolation PASS");
    """)
    assert "core/duration rejection and core-mask isolation PASS" in out


def test_real_perf_counters_and_large_percentile_are_overflow_safe(tmp_path: Path):
    out = run_perf(tmp_path, r"""
    dual_end_perf_stats_t s;
    dual_end_perf_reset(&s, 3, 1, 1);
    assert(dual_end_perf_record(&s, INT64_MAX, 1, true));
    assert(s.overflow == 1 && s.max_us == UINT32_MAX && s.sum_us == UINT32_MAX);
    unknown(&s);
    dual_end_perf_reset(&s, 3, 2, 1);
    s.sum_us = UINT64_MAX - 50;
    assert(dual_end_perf_record(&s, 100, 1, true));
    assert(s.sum_us == UINT64_MAX && s.overflow == 1); unknown(&s);
    dual_end_perf_reset(&s, 3, 3, 1);
    s.samples = UINT32_MAX;
    assert(!dual_end_perf_record(&s, 100, 1, true));
    assert(s.samples == UINT32_MAX && s.sum_us == 0 && s.overflow == 1); unknown(&s);
    dual_end_perf_reset(&s, 3, 4, 1);
    s.histogram[0] = UINT32_MAX;
    assert(dual_end_perf_record(&s, 100, 1, true));
    assert(s.histogram[0] == UINT32_MAX && s.overflow == 1); unknown(&s);
    dual_end_perf_reset(&s, 3, 5, 1);
    s.failures = UINT32_MAX; s.over_budget = UINT32_MAX; s.core_wrong = UINT32_MAX;
    assert(dual_end_perf_record(&s, 20001, 0, false));
    assert(s.failures == UINT32_MAX && s.over_budget == UINT32_MAX && s.core_wrong == UINT32_MAX);
    assert(s.overflow == 3); unknown(&s);
    dual_end_perf_reset(&s, 3, 6, 1); s.invalid_core = UINT32_MAX;
    assert(dual_end_perf_record(&s, 100, -1, true));
    assert(s.invalid_core == UINT32_MAX && s.overflow == 1); unknown(&s);
    dual_end_perf_reset(&s, 3, 7, 1); s.invalid_duration = UINT32_MAX;
    assert(!dual_end_perf_record(&s, -1, 1, true));
    assert(s.invalid_duration == UINT32_MAX && s.overflow == 1); unknown(&s);
    dual_end_perf_reset(&s, 3, 8, 1); s.overflow = UINT32_MAX;
    assert(dual_end_perf_record(&s, INT64_MAX, 1, true));
    assert(s.overflow == UINT32_MAX); unknown(&s);
    dual_end_perf_reset(&s, 3, 9, 1);
    s.samples = UINT32_MAX - 1; s.histogram[0] = s.samples;
    s.sum_us = (uint64_t)s.samples * 100; s.min_us = s.max_us = 100;
    uint32_t mean, p95;
    assert(dual_end_perf_summary(&s, &mean, &p95) && mean == 100 && p95 == 100);
    assert(dual_end_perf_record(&s, 100, 1, true));
    assert(s.samples == UINT32_MAX);
    assert(dual_end_perf_summary(&s, &mean, &p95) && mean == 100 && p95 == 100);
    assert(!dual_end_perf_record(&s, 100, 1, true)); unknown(&s);
    dual_end_perf_reset(&s, 3, 10, 1);
    s.samples = 100; s.sum_us = 1000; /* Missing histogram entries must not invent P95. */
    unknown(&s);
    puts("saturating counters/sum/large-rank/unknown overflow PASS");
    """)
    assert "saturating counters/sum/large-rank/unknown overflow PASS" in out



def test_real_formatter_all_parts_worst_case_and_truncation(tmp_path: Path):
    code = r"""
#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "dual_end_perf_format.h"
int main(void) {
    dual_end_perf_record_t r;
    memset(&r, 0, sizeof(r));
    r.role = r.stage = UINT8_MAX;
    r.boot_id = r.window_seq = UINT32_MAX;
    r.start_us = UINT64_MAX - 1; r.end_us = r.resource_at_us = UINT64_MAX;
    r.heap_internal = r.heap_min_internal = r.heap_psram = r.stack_min = r.telemetry_drop = UINT32_MAX;
    r.stats.codec = UINT8_MAX; r.stats.session_id = UINT32_MAX;
    r.stats.expected_core = r.stats.observed_core = INT_MIN;
    r.stats.observed_core_mask = UINT32_MAX;
    r.stats.samples = UINT32_MAX; r.stats.failures = 0;
    r.stats.over_budget = r.stats.core_wrong = r.stats.invalid_core = r.stats.invalid_duration = UINT32_MAX;
    r.stats.overflow = UINT32_MAX; r.stats.sum_us = UINT64_MAX;
    r.stats.min_us = r.stats.max_us = UINT32_MAX;
    for (unsigned i = 0; i < 9; ++i) r.stats.histogram[i] = UINT32_MAX;
    for (unsigned mode = 0; mode < 2; ++mode) {
        if (mode) {
            --r.window_seq;
            dual_end_perf_reset(&r.stats, 3, UINT32_MAX, INT_MIN);
            assert(dual_end_perf_record(&r.stats, UINT32_MAX, INT_MIN, true));
        }
        for (unsigned i = 0; i < DUAL_END_PERF_PART_COUNT; ++i) {
            unsigned char storage[DUAL_END_PERF_JSON_BUFFER_BYTES + 2];
            memset(storage, 0xA5, sizeof(storage));
            char *buf = (char *)storage;
            size_t n = dual_end_perf_format_part(&r, i, buf, DUAL_END_PERF_JSON_BUFFER_BYTES);
            assert(n > 0 && n <= DUAL_END_PERF_JSON_MAX_BYTES);
            assert(strlen(buf) == n && buf[n] == 0 && buf[0] == '{' && buf[n-1] == '}');
            assert(storage[DUAL_END_PERF_JSON_BUFFER_BYTES] == 0xA5);
            assert(storage[DUAL_END_PERF_JSON_BUFFER_BYTES+1] == 0xA5);
            puts(buf);
            assert(dual_end_perf_format_part(&r, i, buf, n) == 0 && buf[0] == 0);
            assert(storage[DUAL_END_PERF_JSON_BUFFER_BYTES] == 0xA5);
            char shortbuf[4] = {'A','B','C','D'};
            assert(dual_end_perf_format_part(&r, i, shortbuf, 1) == 0 && shortbuf[0] == 0);
            assert(shortbuf[1] == 'B' && shortbuf[2] == 'C' && shortbuf[3] == 'D');
        }
    }
    char buf[DUAL_END_PERF_JSON_BUFFER_BYTES];
    assert(dual_end_perf_format_part(NULL, 0, buf, sizeof(buf)) == 0);
    assert(dual_end_perf_format_part(&r, 0, NULL, sizeof(buf)) == 0);
    assert(dual_end_perf_format_part(&r, 0, buf, 0) == 0);
    assert(dual_end_perf_format_part(&r, DUAL_END_PERF_PART_COUNT, buf, sizeof(buf)) == 0);
    return 0;
}
"""
    raw = compile_run(tmp_path, code, sources=(RUNTIME / "dual_end_perf_stats.c", RUNTIME / "dual_end_perf_format.c"))
    lines = raw.splitlines()
    parts = [json.loads(line) for line in lines]
    assert len(parts) == 26 and all(len(line.encode('utf-8')) <= 180 for line in lines)
    for group in (parts[:13], parts[13:]):
        assert {part['part'] for part in group} == set(range(13))
        assert len({(part['boot_id'], part['window_seq'], part['role']) for part in group}) == 1
        assert group[0]['part_count'] == 13
        assert all(part['event'] == 'audio.codec_perf' and part['schema'] == 1 for part in group)
    assert parts[5]['known'] is False and parts[5]['p95_upper_us'] == 0
    assert parts[18]['known'] is True and parts[18]['p95_upper_us'] == 2**32 - 1
