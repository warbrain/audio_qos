"""Host-only checks for the Scepter cache/query telemetry extension.

These tests compile portable formatters and inspect source contracts only. They
do not build firmware, access a serial device, open audio, or change hardware.
"""
import json
from pathlib import Path

from test_audio_recovery_042 import _build_host_executable, _run


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "scepter/main"
MIC = MAIN / "mic"
RUNTIME = ROOT / "components/dual_end_runtime"
PAIRING = ROOT / "scepter/components/dual_end_pairing"
CODEC = ROOT / "components/dual_end_audio_codec"


def _run_formatter(tmp_path: Path, code: str) -> str:
    harness = tmp_path / "scepter_diagnostics.c"
    harness.write_text(code, encoding="utf-8")
    executable, env = _build_host_executable(
        [
            harness,
            MAIN / "scepter_diagnostics_format.c",
            MIC / "mic_pipeline_telemetry.c",
            MIC / "mic_observability_telemetry.c",
            RUNTIME / "dual_end_telemetry_format.c",
        ],
        tmp_path,
        extra_includes=[MAIN, MIC, RUNTIME / "include", CODEC / "include"],
    )
    return _run([str(executable)], cwd=tmp_path, env=env).stdout


def test_scepter_diagnostics_formats_all_completed_kinds_with_stable_identity(tmp_path: Path):
    code = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "dual_end_spatial.h"
#include "scepter_diagnostics_format.h"
int main(void) {
    mic_audio_diagnostics_snapshot_t s = {0};
    s.ready = true; s.spatial_ready = true; s.cache_sequence = UINT32_MAX;
    s.pipeline = (mic_pipeline_telemetry_t){
        .boot_id=UINT32_MAX,.session_id=UINT32_MAX,.sequence=UINT32_MAX,
        .capture_generation=UINT32_MAX,.window=UINT32_MAX,.codec=4,.codec_known=true,
        .mode=3,.generation=UINT16_MAX,.sampled_us=UINT64_MAX,.start_us=UINT64_MAX,
        .diag_q=UINT32_MAX,.diag_b=UINT32_MAX,.diag_o=UINT32_MAX,.diag_s=UINT32_MAX,
        .fft_hi=UINT32_MAX,.win_lost=UINT32_MAX,.late=UINT32_MAX,.raw_seq=UINT32_MAX,
        .q_hi_all=UINT32_MAX,.fft_us=UINT64_MAX,.fft_n=UINT64_MAX,.fft_u=UINT64_MAX,
        .capture_health={.samples={UINT64_MAX,UINT64_MAX},.sum_sq={UINT64_MAX,UINT64_MAX},
          .zero_samples={UINT64_MAX,UINT64_MAX},.quiet_samples={UINT64_MAX,UINT64_MAX},
          .clip_samples={UINT64_MAX,UINT64_MAX},.peak={UINT32_MAX,UINT32_MAX},
          .underlevel_blocks={UINT32_MAX,UINT32_MAX},.blocks=UINT32_MAX},
        .owner_mutex_wait={UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT64_MAX},
        .owner_encode={UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT64_MAX},
        .owner_radio_submit={UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT64_MAX},
        .nlms={.blocks=UINT64_MAX,.updated_blocks=UINT64_MAX,.frozen_blocks=UINT64_MAX,
          .updated_samples=UINT64_MAX,.frozen_samples=UINT64_MAX,.converged_blocks=UINT64_MAX,
          .first_converged_frame=UINT64_MAX,.convergence_events=UINT32_MAX,
          .update_streak=UINT32_MAX,.max_update_streak=UINT32_MAX,
          .bounded_updates=UINT32_MAX,.resets=UINT32_MAX,.last_reason=7},
    };
    s.pipeline.q=s.pipeline.a=s.pipeline.d=s.pipeline.s=s.pipeline.w=
        (mic_pipeline_metric_t){UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT64_MAX};
    for (unsigned i=0; i<MIC_NLMS_REASON_COUNT; ++i) s.pipeline.nlms.reasons[i]=UINT64_MAX;
    s.wiener=(mic_wiener_diagnostics_t){.available=true,.channels=1};
    s.spatial=(dual_end_spatial_telemetry_t){.boot_id=UINT32_MAX,.role=0,
        .session_id=UINT32_MAX,.sequence=UINT32_MAX,.codec=4,.codec_known=true,
        .sampled_us=UINT64_MAX,.generation=UINT16_MAX,.requested=3,.applied=3,
        .layout=UINT16_MAX,.fallback=UINT16_MAX,.delay_samples=0,.correlation=0,.current_mix=0,
        .frames=UINT64_MAX,.learned_frames=UINT64_MAX,.frozen_frames=UINT64_MAX,
        .mixed_frames=UINT64_MAX,.nlms_fallback_frames=UINT64_MAX,.mix_q16_sum=UINT64_MAX,
        .processing_us=UINT64_MAX,.processing_frames=UINT32_MAX,
        .processing_max_us=UINT32_MAX,.bounded_updates=UINT32_MAX,.resets=UINT32_MAX};
    s.spatial.applied=6;
    s.mvdr=(mic_mvdr_diagnostics_t){.available=true,.noise_valid=true,.fallback=7,
        .speech_mask_q15=32767,.processed_hops=UINT64_MAX,.valid_bins=UINT64_MAX,
        .fallback_bins=UINT64_MAX,.fallback_hops=UINT64_MAX,
        .queue_underflows=UINT64_MAX,.queue_overflows=UINT64_MAX};
    char json[SCEPTER_DIAGNOSTICS_JSON_BUFFER_BYTES];
    for (unsigned kind=0; kind<SCEPTER_DIAGNOSTICS_KIND_COUNT; ++kind) {
        unsigned expected_schema=kind==SCEPTER_DIAGNOSTICS_KIND_SPATIAL ? 2U : 1U;
        unsigned part_count=kind==SCEPTER_DIAGNOSTICS_KIND_WIENER ? 5U : scepter_diagnostics_part_count(kind);
        for (unsigned part=0; part<part_count; ++part) {
            const char *event=NULL; unsigned schema=0; bool active=false;
            size_t n=scepter_diagnostics_format_part(&s,kind,part,json,sizeof(json),&event,&schema,&active);
            if (!(n && n==strlen(json) && n<=180 && event && schema==expected_schema && active)) {
                fprintf(stderr,"invalid diagnostics part kind=%u part=%u bytes=%zu event=%s schema=%u active=%u\n",
                    kind,part,n,event?event:"null",schema,active?1U:0U);
                return 2;
            }
            puts(json);
        }
    }
    s.pipeline.nlms.blocks=0;
    for (unsigned part=0; part<MIC_NLMS_TELEMETRY_PARTS; ++part) {
        const char *event=NULL; unsigned schema=0; bool active=true;
        size_t n=scepter_diagnostics_format_part(&s,SCEPTER_DIAGNOSTICS_KIND_NLMS,part,json,sizeof(json),&event,&schema,&active);
        assert(n && !active && event && schema==1); puts(json);
    }
    s.spatial.applied=4; s.spatial.layout=DUAL_END_LAYOUT_DUAL_MONO;
    s.wiener=(mic_wiener_diagnostics_t){.available=true,.channels=1,
        .noise_valid={true,false},.processed_hops={123,0},
        .queue_underflows={4,0},.queue_overflows={5,0}};
    for (unsigned part=0; part<5U; ++part) {
        const char *event=NULL; unsigned schema=0; bool active=false;
        size_t n=scepter_diagnostics_format_part(&s,SCEPTER_DIAGNOSTICS_KIND_WIENER,part,json,sizeof(json),&event,&schema,&active);
        assert(n && n==strlen(json) && n<=180 && event && schema==1 && active);
        puts(json);
    }
    s.spatial.applied=5; s.spatial.layout=DUAL_END_LAYOUT_STEREO;
    s.wiener=(mic_wiener_diagnostics_t){.available=true,.channels=2,
        .noise_valid={true,false},.processed_hops={UINT64_MAX,UINT64_MAX},
        .queue_underflows={11,12},.queue_overflows={13,14}};
    for (unsigned part=0; part<8U; ++part) {
        const char *event=NULL; unsigned schema=0; bool active=false;
        size_t n=scepter_diagnostics_format_part(&s,SCEPTER_DIAGNOSTICS_KIND_WIENER,part,json,sizeof(json),&event,&schema,&active);
        assert(n && n==strlen(json) && n<=180 && event && schema==1 && active);
        puts(json);
    }
    s.spatial_ready=false;
    assert(!scepter_diagnostics_format_part(&s,SCEPTER_DIAGNOSTICS_KIND_SPATIAL,0,json,sizeof(json),NULL,NULL,NULL) && !json[0]);
    assert(!scepter_diagnostics_format_part(&s,SCEPTER_DIAGNOSTICS_KIND_COUNT,0,json,sizeof(json),NULL,NULL,NULL));
    return 0;
}
'''
    rows = [json.loads(line) for line in _run_formatter(tmp_path, code).splitlines()]
    expected = {
        "audio.spatial": (2, 15),
        "audio.pipeline": (1, 32),
        "audio.capture_health": (1, 18),
        "audio.owner_timing": (1, 17),
        "audio.nlms": (1, 46),
        "audio.wiener": (1, 18),
        "audio.mvdr": (1, 8),
        "audio.dispatch": (1, 17),
    }
    assert len(rows) == sum(count for _, count in expected.values())
    for event, (schema, count) in expected.items():
        group = [row for row in rows if row["event"] == event]
        assert len(group) == count
        assert all(row["schema"] == schema and row["boot_id"] == "ffffffff" and
                   row["session_id"] == row["seq"] == 2**32 - 1 for row in group)
    for event, (_, count) in expected.items():
        group = [row for row in rows if row["event"] == event]
        if event == "audio.wiener":
            assert [row["part"] for row in group] == list(range(5)) + list(range(5)) + list(range(8))
        else:
            first = group[:count if event != "audio.nlms" else 23]
            assert [row["part"] for row in first] == list(range(len(first)))
    inactive = [row for row in rows if row["event"] == "audio.nlms"][23:]
    assert [row["part"] for row in inactive] == list(range(23))
    assert {key: value for row in inactive for key, value in row.items()}["blocks"] == "0"
    wiener = [row for row in rows if row["event"] == "audio.wiener"]
    assert "active" not in wiener[0]
    assert wiener[1]["active"] is False and wiener[1]["available"] is False
    assert wiener[2]["processed_hops"] is None
    assert wiener[3]["queue_underflows"] is None
    assert wiener[4]["queue_overflows"] is None
    assert [row["part"] for row in wiener[5:10]] == list(range(5))
    assert wiener[7]["processed_hops"] == "123" and wiener[8]["queue_underflows"] == "4"
    assert wiener[9]["queue_overflows"] == "5"
    assert [row["part"] for row in wiener[10:]] == list(range(8))
    assert wiener[11]["lv"] is True and wiener[11]["rv"] is False
    assert wiener[12]["lh"] == str(2**64 - 1)
    assert wiener[13]["rh"] == str(2**64 - 1)
    assert (wiener[14]["lu"],wiener[15]["ru"]) == ("11","12")
    assert (wiener[16]["lo"],wiener[17]["ro"]) == ("13","14")


def test_scepter_diagnostics_keeps_realtime_and_legacy_command_boundaries():
    mic = (MIC / "mic_audio.c").read_text(encoding="utf-8")
    diagnostics = (MAIN / "scepter_diagnostics.c").read_text(encoding="utf-8")
    pairing = (PAIRING / "dual_end_pairing.c").read_text(encoding="utf-8")
    app = (MAIN / "app_main.c").read_text(encoding="utf-8")

    close_start = mic.index("static void mic_pipeline_close_window")
    close_end = mic.index("static void mic_pipeline_record_block", close_start)
    close = mic[close_start:close_end]
    assert "spatial_at_close=s_spatial_telemetry" in close
    assert "s_pipeline_diagnostics_cache=diagnostics" in close
    assert "spatial.sequence=" not in close
    assert "s_pipeline_report_ready=true" in close
    assert all(token not in close for token in ("ESP_LOG", "telemetry_format", "xQueue", "malloc", "vTaskDelay"))

    copy_start = mic.index("bool mic_audio_copy_diagnostics_snapshot")
    copy = mic[copy_start:]
    assert "s_pipeline_report_ready=false" not in copy
    assert "s_pipeline_diagnostics_cache" in copy and "s_spatial_telemetry" not in copy
    assert "s_config_lock" not in copy
    assert "complete persistent bundle" in copy

    worker_start = diagnostics.index("static void diagnostics_sample")
    worker_end = diagnostics.index("static void diagnostics_task", worker_start)
    worker = diagnostics[worker_start:worker_end]
    assert "mic_audio_copy_diagnostics_snapshot" in worker
    assert all(token not in worker for token in ("ESP_LOG", "telemetry_format", "xQueue", "malloc", "esp_now"))
    assert '#include "dual_end_task_policy.h"' in diagnostics
    assert "DUAL_END_COMM_TASK_CORE" in diagnostics
    assert "DIAG_CONTEXT" in diagnostics and "SCEPTER_TELEMETRY" in diagnostics
    assert "s_cache_ready" in diagnostics and "next.cache_sequence != s_cache.cache_sequence" in diagnostics
    assert "context_pending" in diagnostics and "snapshot->pipeline.sampled_us >= s_context.set_us" in diagnostics
    assert "if (++s_pipeline_report_sequence == 0U) ++s_pipeline_report_sequence;" in mic

    runtime_offset = pairing.index("dual_end_runtime_command(line")
    callback_offset = pairing.index("s_diagnostics_command_cb(line")
    legacy_info_offset = pairing.index("strcasecmp(line, \"INFO\")")
    assert runtime_offset < callback_offset < legacy_info_offset
    assert "dual_end_pairing_set_diagnostics_command_cb" in pairing
    register_offset = app.index("dual_end_pairing_set_diagnostics_command_cb")
    pairing_start_offset = app.index('STARTUP_DIAG_CHECK("pairing_usb"')
    assert register_offset < pairing_start_offset


def test_scepter_context_requires_a_fresh_immutable_cache_and_bound_identity():
    """Keep DIAG_CONTEXT strict without running firmware or a USB device."""
    diagnostics = (MAIN / "scepter_diagnostics.c").read_text(encoding="utf-8")

    assert "cache_sequence_floor" in diagnostics
    assert "context_identity_matches" in diagnostics
    assert "context_bind_identity" in diagnostics
    assert "snapshot->cache_sequence != s_context.cache_sequence_floor" in diagnostics
    assert "snapshot->pipeline.sampled_us >= s_context.set_us" in diagnostics
    assert "s_context.set_us = UINT64_MAX;" in diagnostics
    assert "mic_audio_diagnostics_snapshot_t latest = {0};" in diagnostics
    assert "mic_audio_copy_diagnostics_snapshot(&latest)" in diagnostics
    assert "latest.cache_sequence : 0" in diagnostics
    assert "s_context.identity_bound = false;" in diagnostics
    assert "context_bind_identity(&s_context, snapshot);" in diagnostics
    assert "context.identity_bound && context_identity_matches(&context, &snapshot)" in diagnostics

    set_context = diagnostics[diagnostics.index("static bool set_context"):]
    first_unlock = set_context.index("portEXIT_CRITICAL(&s_diagnostics_lock);")
    source_copy = set_context.index("mic_audio_copy_diagnostics_snapshot(&latest)")
    second_lock = set_context.index("portENTER_CRITICAL(&s_diagnostics_lock);", first_unlock + 1)
    assert first_unlock < source_copy < second_lock


def test_scepter_context_rechecks_cache_observed_during_final_publish():
    """Model the cache-N interleaving without compiling or running firmware.

    The worker may mirror an eligible immutable owner window while a new
    DIAG_CONTEXT deliberately exposes its provisional UINT64_MAX boundary.
    That first observation must remain pending, then be evaluated once after
    the final source-cache floor and timestamp commit.  Stale and mixed caches
    must remain rejected by the same recheck.
    """
    diagnostics = (MAIN / "scepter_diagnostics.c").read_text(encoding="utf-8")
    worker_start = diagnostics.index("static void diagnostics_sample")
    worker_end = diagnostics.index("static void diagnostics_task", worker_start)
    worker = diagnostics[worker_start:worker_end]
    helper_start = diagnostics.index("static void context_evaluate_cache_locked")
    helper_end = diagnostics.index("static void diagnostics_sample", helper_start)
    helper = diagnostics[helper_start:helper_end]
    set_context = diagnostics[diagnostics.index("static bool set_context"):]

    assert "s_context.set_us != UINT64_MAX" in helper
    assert "snapshot->cache_sequence != s_context.cache_sequence_floor" in helper
    assert "snapshot->pipeline.sampled_us >= s_context.set_us" in helper
    assert "context_identity_matches(&s_context, snapshot)" in helper
    assert "context_evaluate_cache_locked(&next);" in worker
    final_timestamp = set_context.index("s_context.set_us = set_us;")
    final_floor = set_context.index("s_context.cache_sequence_floor = cache_sequence_floor;")
    recheck = set_context.index("context_evaluate_cache_locked(&s_cache);")
    assert final_timestamp < final_floor < recheck

    def evaluate(context, cache):
        context["ready"] = False
        context["cache_context_sequence"] = 0
        if not cache["ready"] or not cache["sequence"]:
            return
        if not context["required"]:
            context["cache_context_sequence"] = context["sequence"]
            context["ready"] = True
            return
        fresh = (context["set_us"] != 2**64 - 1 and
                 cache["sequence"] != context["floor"] and
                 cache["sampled_us"] >= context["set_us"])
        if not fresh:
            return
        if context["identity"] is None:
            context["identity"] = cache["identity"]
        if context["identity"] == cache["identity"]:
            context["cache_context_sequence"] = context["sequence"]
            context["ready"] = True

    context = {
        "sequence": 9, "required": True, "set_us": 2**64 - 1,
        "floor": 0, "identity": None, "ready": False,
        "cache_context_sequence": 0,
    }
    # Cache N arrives after the clock boundary but before the final context
    # commit, so its first mirrored observation is intentionally pending.
    cache_n = {"ready": True, "sequence": 42, "sampled_us": 1001,
               "identity": (7, 11, 13, 4, True, 2, 5)}
    evaluate(context, cache_n)
    assert not context["ready"] and context["identity"] is None

    # The source floor captured cache N-1.  Rechecking the already mirrored N
    # after the final commit accepts it without waiting for N+1.
    context["set_us"] = 1000
    context["floor"] = 41
    evaluate(context, cache_n)
    assert context["ready"] and context["cache_context_sequence"] == 9
    assert context["identity"] == cache_n["identity"]

    # The recheck remains fail-closed for the floor window, a newer cache
    # sampled before the final context timestamp, and an identity that changes
    # after the first eligible cache bound this context.
    stale = {**cache_n, "sequence": 41}
    evaluate(context, stale)
    assert not context["ready"]
    too_early = {**cache_n, "sequence": 43, "sampled_us": 999}
    evaluate(context, too_early)
    assert not context["ready"]
    mixed = {**cache_n, "sequence": 44,
             "identity": (7, 12, 13, 4, True, 2, 5)}
    evaluate(context, mixed)
    assert not context["ready"]


def test_response_budget_fits_the_existing_native_usb_response_buffer():
    maximum = 2**32 - 1
    prefix = (
        f"OK SCEPTER_TELEMETRY token={maximum} kind=4 event=audio.capture_health "
        f"schema=2 run={maximum} arm={maximum} ctx={maximum} cache={maximum} "
        f"boot=ffffffff session={maximum} codec=none mode=255 generation=65535 "
        f"capture_generation={maximum} active=1 "
    )
    assert len(prefix.encode()) + 180 + 1 <= 512
