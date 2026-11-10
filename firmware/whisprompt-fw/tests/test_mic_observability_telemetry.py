"""Bounded Scepter microphone observability format and hot-owner contracts.

Host-only checks compile the real formatters. They do not build firmware, open
audio devices, or invoke any ESP transport.
"""
import json
from pathlib import Path

from test_audio_recovery_042 import _build_host_executable, _run

ROOT = Path(__file__).resolve().parents[1]
MIC = ROOT / "scepter/main/mic"
SENDER = ROOT / "scepter/components/dual_end_audio/dual_end_audio.c"
MIC_AUDIO = ROOT / "scepter/main/mic/mic_audio.c"


def _run_formatter(tmp_path: Path, code: str) -> str:
    harness = tmp_path / "observability.c"
    harness.write_text(code, encoding="utf-8")
    executable, env = _build_host_executable(
        [harness, MIC / "mic_pipeline_telemetry.c", MIC / "mic_observability_telemetry.c"],
        tmp_path, extra_includes=[MIC],
    )
    return _run([str(executable)], cwd=tmp_path, env=env).stdout


def test_capture_health_owner_timing_and_nlms_parts_are_bounded(tmp_path: Path):
    code = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mic_pipeline_telemetry.h"
int main(void) {
    mic_capture_health_t one = {0};
    mic_capture_health_record(&one, INT16_MIN, 0);
    mic_capture_health_record(&one, 127, -127);
    mic_capture_health_complete_block(&one);
    assert(one.samples[0] == 2 && one.sum_sq[0] == UINT64_C(1073741824) + 16129);
    assert(one.clip_samples[0] == 1 && one.zero_samples[1] == 1);
    assert(one.underlevel_blocks[0] == 0 && one.underlevel_blocks[1] == 1);
    mic_nlms_telemetry_t window = {0};
    mic_nlms_block_t delta = {.valid=true, .updated_samples=4, .bounded_updates=2, .resets=1};
    mic_nlms_telemetry_add(&window, &delta);
    delta.bounded_updates = 3; delta.resets = 2;
    mic_nlms_telemetry_add(&window, &delta);
    assert(window.blocks == 2 && window.bounded_updates == 5 && window.resets == 3);
    mic_nlms_telemetry_t convergence = {0};
    mic_nlms_block_t missing_index = {.valid=true, .convergence_event=true};
    mic_nlms_telemetry_add(&convergence, &missing_index);
    assert(convergence.convergence_events == 1 && !convergence.has_first_converged_frame);
    mic_nlms_block_t first = {.valid=true, .convergence_event=true,
        .has_convergence_frame=true, .convergence_frame=320};
    mic_nlms_telemetry_add(&convergence, &first);
    assert(convergence.convergence_events == 2 && convergence.has_first_converged_frame &&
           convergence.first_converged_frame == 320);

    mic_wiener_timing_histogram_t timing_histogram = {0};
    for (uint32_t us = 100; us <= 1000; us += 100)
        mic_wiener_timing_histogram_add(&timing_histogram, us);
    mic_wiener_timing_summary_t timing_summary =
        mic_wiener_timing_histogram_summary(&timing_histogram);
    assert(timing_summary.available && timing_summary.count == 10 &&
           timing_summary.p50_us == 550 && timing_summary.p95_us == 1050 &&
           timing_summary.max_us == 1000 && !timing_summary.p50_overflow &&
           !timing_summary.p95_overflow);
    mic_wiener_timing_histogram_t threshold = {0};
    mic_wiener_timing_histogram_add(&threshold, 9999);
    timing_summary = mic_wiener_timing_histogram_summary(&threshold);
    assert(timing_summary.p95_us == 10000 && !timing_summary.p95_overflow);
    mic_wiener_timing_histogram_add(&threshold, 10000);
    timing_summary = mic_wiener_timing_histogram_summary(&threshold);
    assert(timing_summary.p95_us == 10000 && timing_summary.p95_overflow);
    mic_wiener_timing_histogram_add(&timing_histogram, 12000);
    timing_summary = mic_wiener_timing_histogram_summary(&timing_histogram);
    assert(timing_summary.p95_overflow && timing_summary.p95_us == MIC_WIENER_TIMING_MAX_US &&
           timing_summary.max_us == 12000);

    mic_pipeline_telemetry_t r = {
        .boot_id = UINT32_MAX, .session_id = UINT32_MAX, .sequence = UINT32_MAX,
        .codec = 4, .codec_known = true, .mode = 3, .generation = UINT16_MAX,
        .output_gain_l_x = 16, .output_gain_r_x = 16,
        .sampled_us = UINT64_MAX,
    };
    r.capture_health = (mic_capture_health_t){
        .samples = {UINT64_MAX, UINT64_MAX}, .sum_sq = {UINT64_MAX, UINT64_MAX},
        .zero_samples = {UINT64_MAX, UINT64_MAX}, .quiet_samples = {UINT64_MAX, UINT64_MAX},
        .clip_samples = {UINT64_MAX, UINT64_MAX}, .peak = {UINT32_MAX, UINT32_MAX},
        .underlevel_blocks = {UINT32_MAX, UINT32_MAX}, .blocks = UINT32_MAX,
    };
    r.owner_mutex_wait = r.owner_encode = r.owner_radio_submit =
        (mic_pipeline_metric_t){UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT64_MAX};
    r.nlms = (mic_nlms_telemetry_t){.blocks=UINT64_MAX,.updated_blocks=UINT64_MAX,
        .frozen_blocks=UINT64_MAX,.updated_samples=UINT64_MAX,.frozen_samples=UINT64_MAX,
        .converged_blocks=UINT64_MAX,.first_converged_frame=UINT64_MAX,
        .convergence_events=UINT32_MAX,.update_streak=UINT32_MAX,.max_update_streak=UINT32_MAX,
        .bounded_updates=UINT32_MAX,.resets=UINT32_MAX,.last_reason=7};
    for (unsigned i = 0; i < MIC_NLMS_REASON_COUNT; ++i) r.nlms.reasons[i] = UINT64_MAX;
    char line[MIC_OBSERVABILITY_TELEMETRY_BYTES];
    for (unsigned p = 0; p < MIC_CAPTURE_HEALTH_TELEMETRY_PARTS; ++p) {
        size_t n = mic_capture_health_telemetry_format(&r, p, line, sizeof(line));
            if (!(n && n == strlen(line) && n <= 180)) {
                fprintf(stderr, "invalid capture-health part=%u bytes=%zu line=%s\\n", p, n, line);
                return 2;
            }
            puts(line);
    }
    r.capture_health = (mic_capture_health_t){
        .samples={2,2}, .sum_sq={UINT64_C(2)*8192U*8192U,UINT64_C(2)*16384U*16384U},
        .clip_samples={1,2}, .peak={16384,32768}, .blocks=1,
    };
    assert(mic_capture_health_telemetry_format(&r, 14, line, sizeof(line)) &&
           strstr(line, "\"rms_dbfs_l\":-12.0") &&
           strstr(line, "\"rms_dbfs_r\":-6.0"));
    assert(mic_capture_health_telemetry_format(&r, 15, line, sizeof(line)) &&
           strstr(line, "\"peak_dbfs_l\":-6.0") &&
           strstr(line, "\"peak_dbfs_r\":0.0"));
    assert(mic_capture_health_telemetry_format(&r, 16, line, sizeof(line)) &&
           strstr(line, "\"clip_pct\":75.00"));
    assert(mic_capture_health_telemetry_format(&r, 17, line, sizeof(line)) &&
           strstr(line, "\"gain_l_x\":16") && strstr(line, "\"gain_r_x\":16"));
    r.output_gain_l_x = 0;
    assert(mic_capture_health_telemetry_format(&r, 17, line, sizeof(line)) &&
           strstr(line, "\"gain_l_x\":null") && strstr(line, "\"gain_r_x\":16"));
    r.output_gain_l_x = 4; r.output_gain_r_x = 1;
    assert(mic_capture_health_telemetry_format(&r, 17, line, sizeof(line)) &&
           strstr(line, "\"gain_l_x\":4") && strstr(line, "\"gain_r_x\":1"));
    for (unsigned p = 0; p < MIC_OWNER_TIMING_TELEMETRY_PARTS; ++p) {
        size_t n = mic_owner_timing_telemetry_format(&r, p, line, sizeof(line));
        assert(n && n == strlen(line) && n <= 180); puts(line);
    }
    assert(mic_owner_timing_telemetry_format(&r, 11, line, sizeof(line)) &&
           strstr(line, "\"wiener_timing_active\":false"));
    r.mode = 5;
    r.wiener_timing[0] = (mic_wiener_timing_summary_t){
        .count=250, .p50_us=5000, .p95_us=MIC_WIENER_TIMING_MAX_US,
        .max_us=12000, .available=true, .p95_overflow=true};
    r.wiener_timing[1] = (mic_wiener_timing_summary_t){
        .count=250, .p50_us=5100, .p95_us=7800, .max_us=8300, .available=true};
    size_t wiener_n = mic_owner_timing_telemetry_format(&r, 11, line, sizeof(line));
    assert(wiener_n && wiener_n <= 180 && strstr(line, "\"wiener_l_us_p50\":5000"));
    wiener_n = mic_owner_timing_telemetry_format(&r, 12, line, sizeof(line));
    assert(wiener_n && wiener_n <= 180 && strstr(line, "\"wiener_l_us_p95\":\">=10000\""));
    wiener_n = mic_owner_timing_telemetry_format(&r, 13, line, sizeof(line));
    assert(wiener_n && wiener_n <= 180 && strstr(line, "\"wiener_l_us_max\":12000"));
    wiener_n = mic_owner_timing_telemetry_format(&r, 14, line, sizeof(line));
    assert(wiener_n && wiener_n <= 180 && strstr(line, "\"wiener_r_us_p50\":5100"));
    wiener_n = mic_owner_timing_telemetry_format(&r, 15, line, sizeof(line));
    assert(wiener_n && wiener_n <= 180 && strstr(line, "\"wiener_r_us_p95\":7800"));
    wiener_n = mic_owner_timing_telemetry_format(&r, 16, line, sizeof(line));
    assert(wiener_n && wiener_n <= 180 && strstr(line, "\"wiener_r_us_max\":8300"));
    r.mode = 4;
    wiener_n = mic_owner_timing_telemetry_format(&r, 11, line, sizeof(line));
    assert(wiener_n && strstr(line, "\"wiener_us_p50\":5000"));
    wiener_n = mic_owner_timing_telemetry_format(&r, 12, line, sizeof(line));
    assert(wiener_n && strstr(line, "\"wiener_us_p95\":\">=10000\""));
    wiener_n = mic_owner_timing_telemetry_format(&r, 13, line, sizeof(line));
    assert(wiener_n && strstr(line, "\"wiener_us_max\":12000"));
    for (unsigned p = 0; p < MIC_NLMS_TELEMETRY_PARTS; ++p) {
        size_t n = mic_nlms_telemetry_format(&r, p, line, sizeof(line));
        assert(n && n == strlen(line) && n <= 180); puts(line);
    }
    char presence[MIC_NLMS_PRESENCE_TELEMETRY_BYTES];
    r.nlms.has_first_converged_frame = false;
    r.nlms.first_converged_frame = 0;
    size_t presence_n = mic_nlms_presence_telemetry_format(&r, 0, presence, sizeof(presence));
    assert(presence_n && presence_n == strlen(presence) && presence_n <= 190);
    assert(strstr(presence, "\"has_first_converged_frame\":false"));
    assert(mic_nlms_presence_telemetry_format(&r, 1, presence, sizeof(presence)) &&
           strstr(presence, "\"first_converged_frame\":null"));
    r.nlms.has_first_converged_frame = true;
    for (unsigned part = 0; part < MIC_NLMS_PRESENCE_TELEMETRY_PARTS; ++part) {
        presence_n = mic_nlms_presence_telemetry_format(&r, part, presence, sizeof(presence));
        assert(presence_n && presence_n == strlen(presence) && presence_n <= 190);
        puts(presence);
    }
    r.nlms.first_converged_frame = UINT64_MAX;
    assert(mic_nlms_presence_telemetry_format(&r, 1, presence, sizeof(presence)) &&
           strstr(presence, "18446744073709551615"));
    r.mode = 4;
    assert(mic_capture_health_telemetry_format(&r, 0, line, sizeof(line)) && strstr(line, "\"m\":4"));
    r.mode = 5;
    assert(mic_capture_health_telemetry_format(&r, 0, line, sizeof(line)) && strstr(line, "\"m\":5"));
    r.mode = 3; r.nlms.blocks = 0;
    assert(!mic_nlms_telemetry_format(&r, 0, line, sizeof(line)) && !line[0]);
    assert(!mic_nlms_presence_telemetry_format(&r, 0, presence, sizeof(presence)) && !presence[0]);
    return 0;
}
'''
    rows = [json.loads(line) for line in _run_formatter(tmp_path, code).splitlines()]
    expected = {
        "audio.capture_health": 18,
        "audio.owner_timing": 17,
        "audio.nlms": 23,
        "audio.nlms_presence": 2,
    }
    assert len(rows) == sum(expected.values())
    for event, count in expected.items():
        group = [row for row in rows if row["event"] == event]
        assert [row["part"] for row in group] == list(range(count))
        if event != "audio.nlms_presence":
            assert all(row["schema"] == 1 and row["role"] == 0 and row["c"] == 4 and row["m"] in (3, 4, 5)
                       for row in group)
        merged = {key: value for row in group for key, value in row.items()}
        assert merged["part_count"] == count
        if event != "audio.nlms_presence":
            assert merged["sampled_us"] == str(2**64 - 1)
    capture = {key: value for row in rows if row["event"] == "audio.capture_health" for key, value in row.items()}
    assert capture["l_zero"] == str(2**64 - 1) and capture["balance_q15"] == 32767
    owner = {key: value for row in rows if row["event"] == "audio.owner_timing" for key, value in row.items()}
    assert owner["enc_us"] == str(2**64 - 1) and owner["rs_hi"] == 2**32 - 1
    nlms = {key: value for row in rows if row["event"] == "audio.nlms" for key, value in row.items()}
    assert nlms["first_converged_frame"] == str(2**64 - 1) and nlms["reason_7"] == str(2**64 - 1)
    presence = {key: value for row in rows if row["event"] == "audio.nlms_presence" for key, value in row.items()}
    assert presence["part_count"] == 2
    assert presence["has_first_converged_frame"] is True
    assert presence["first_converged_frame"] == "0"


def test_owner_timing_snapshot_does_not_add_a_second_sender_lock():
    source = SENDER.read_text(encoding="utf-8")
    start = source.index("bool dual_end_audio_sender_copy_last_owner_timing")
    end = source.index("uint8_t dual_end_audio_sender_get_codec", start)
    copy = source[start:end]
    assert "xSemaphoreTake" not in copy and "*timing = s_last_owner_timing" in copy
    send_start = source.index("esp_err_t dual_end_audio_sender_send_pcm_with_hardware_and_analysis")
    send = source[send_start:source.index("esp_err_t dual_end_audio_preencode_test_loop", send_start)]
    for token in ("sender_locked_us", "encode_started_us", "radio_submit_started_us",
                  "mutex_wait_us", "encode_us", "radio_submit_us"):
        assert token in send


def test_nlms_observer_uses_a_positive_completed_frame_and_presence_bit():
    source = MIC_AUDIO.read_text(encoding="utf-8")
    start = source.index("static mic_nlms_block_t mic_nlms_observe")
    end = source.index("static void mic_spatial_cache_snapshot", start)
    observer = source[start:end]
    assert "block.has_convergence_frame = true" in observer
    assert "block.convergence_frame = stats->frames ? stats->frames :" in observer
    assert "DUAL_END_SPATIAL_FRAME_SAMPLES" in observer


def test_capture_health_window_is_split_by_applied_gain():
    source = MIC_AUDIO.read_text(encoding="utf-8")
    record = source[source.index("static void mic_pipeline_record_block"):]
    assert "const uint8_t output_gain_l_x=(uint8_t)s_config[2];" in record
    assert "const uint8_t output_gain_r_x=(uint8_t)s_config[3];" in record
    assert "s_pipeline_window.output_gain_l_x!=output_gain_l_x" in record
    assert "s_pipeline_window.output_gain_r_x!=output_gain_r_x" in record
    assert "s_pipeline_window.output_gain_l_x=output_gain_l_x;" in record
    assert "s_pipeline_window.output_gain_r_x=output_gain_r_x;" in record


def test_presence_full_log_prefix_and_lf_fit_real_transport(tmp_path):
    (tmp_path / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n", encoding="utf-8")
    code = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mic_pipeline_telemetry.h"
#include "dual_end_log_transport.h"
int main(void) {
    mic_pipeline_telemetry_t r = {.boot_id=UINT32_MAX,.session_id=UINT32_MAX,
        .sequence=UINT32_MAX,.codec=5,.codec_known=true,.mode=5,.nlms={.blocks=UINT64_MAX}};
    const uint8_t codecs[]={3,4,5,UINT8_MAX};
    for(unsigned codec=0;codec<4U;++codec) {
        r.codec=codecs[codec];r.codec_known=codec<3U;
        for(unsigned state=0;state<3U;++state) {
            r.nlms.has_first_converged_frame=state!=0U;
            r.nlms.first_converged_frame=state==2U?UINT64_MAX:0;
            for(unsigned part=0;part<MIC_NLMS_PRESENCE_TELEMETRY_PARTS;++part) {
                unsigned char memory[MIC_NLMS_PRESENCE_TELEMETRY_BYTES+2U];
                memset(memory,0xa5,sizeof(memory));
                char *json=(char *)memory;
                const size_t n=mic_nlms_presence_telemetry_format(&r,part,json,
                    MIC_NLMS_PRESENCE_TELEMETRY_BYTES);
                assert(n>0U && n<=190U && n==strlen(json));
                assert(memory[MIC_NLMS_PRESENCE_TELEMETRY_BYTES]==0xa5);
                assert(memory[MIC_NLMS_PRESENCE_TELEMETRY_BYTES+1U]==0xa5);
                char record[DUAL_END_LOG_TRANSPORT_LINE_BYTES];
                char prefix[65];memset(prefix,'p',64);prefix[64]='\0';
                const int whole=snprintf(record,sizeof(record),"%s%s\n",prefix,json);
                assert(whole>0 && (size_t)whole<sizeof(record));
                assert((size_t)whole==64U+n+1U && record[whole-1]=='\n');
                puts(json);
                assert(!mic_nlms_presence_telemetry_format(&r,part,json,n) && !json[0]);
            }
        }
    }
    char line[MIC_NLMS_PRESENCE_TELEMETRY_BYTES];
    assert(!mic_nlms_presence_telemetry_format(&r,MIC_NLMS_PRESENCE_TELEMETRY_PARTS,
                                             line,sizeof(line)) && !line[0]);
    r.nlms.blocks=0;
    assert(!mic_nlms_presence_telemetry_format(&r,0,line,sizeof(line)) && !line[0]);
    assert(MIC_NLMS_TELEMETRY_PARTS==23U);
    return 0;
}
'''
    harness = tmp_path / "presence_full_log.c"
    harness.write_text(code, encoding="utf-8")
    executable, env = _build_host_executable(
        [harness, MIC / "mic_pipeline_telemetry.c", MIC / "mic_observability_telemetry.c"],
        tmp_path, extra_includes=[MIC, ROOT / "components/dual_end_runtime/include", tmp_path],
    )
    rows = [json.loads(line) for line in _run([str(executable)], cwd=tmp_path, env=env).stdout.splitlines()]
    assert len(rows) == 24
    for index in range(0, len(rows), 2):
        first, second = rows[index:index+2]
        assert first["part"] == 0 and second["part"] == 1 and first["part_count"] == 2
        assert first["has_first_converged_frame"] is (second["first_converged_frame"] is not None)
        for key in ("event", "schema", "boot_id", "role", "session_id", "seq", "c", "m"):
            assert first[key] == second[key]
        assert second["first_converged_frame"] in (None, "0", str(2**64-1))
        assert len(json.dumps(first, separators=(",", ":"))) <= 190
