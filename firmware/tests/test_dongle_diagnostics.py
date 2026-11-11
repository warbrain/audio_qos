"""Host checks for the bounded Dongle diagnostics response schema.

These compile only the portable formatter. They do not touch a serial device,
audio route, firmware image, or an active benchmark.
"""
import json
import os
from pathlib import Path
import tempfile
from unittest.mock import patch

from test_audio_recovery_042 import _build_host_executable, _run


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "dongle" / "main"


def compile_and_run(source: str) -> str:
    build_root = Path(os.environ.get("FIRMWARE_TEST_TMP", ROOT / "build/host-tests"))
    build_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="dongle-diag-host-", dir=build_root) as directory:
        directory = Path(directory)
        harness = directory / "check.c"
        harness.write_text(source, encoding="utf-8")
        with patch.dict(os.environ, {name: str(directory) for name in ("TMP", "TEMP", "TMPDIR")}):
            executable, env = _build_host_executable(
                [harness, MAIN / "dongle_diagnostics_format.c"], directory,
                extra_includes=(MAIN,),
            )
        return _run([str(executable)], cwd=directory, env=env).stdout


def test_dongle_diagnostics_parts_are_complete_bounded_and_joinable():
    output = compile_and_run(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "dongle_diagnostics_format.h"
int main(void) {
    dongle_diagnostics_snapshot_t s = {
        .runtime_ready = true, .resource_cache_ready = true, .rx_cache_ready = true,
        .context_set = true, .role = 1, .codec = UINT8_MAX,
        .boot_id = UINT32_MAX, .cache_sequence = UINT32_MAX,
        .run_id = UINT32_MAX, .arm_id = UINT32_MAX, .context_sequence = UINT32_MAX,
        .session_id = UINT32_MAX, .sampled_us = UINT64_MAX,
        .resource_sequence = UINT32_MAX, .resource_sampled_us = UINT64_MAX,
        .perf_ready_mask = 15,
        .perf_window_sequence = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX},
        .rx_active = true, .rx_accepted = UINT32_MAX, .rx_output_frames = UINT32_MAX,
        .rx_permanent_loss = UINT32_MAX, .rx_gap_events = UINT32_MAX,
        .rx_recovered = UINT32_MAX, .rx_duplicates = UINT32_MAX, .rx_late = UINT32_MAX,
        .rx_rejected = UINT32_MAX, .rx_abort_count = UINT32_MAX,
        .rx_nack_requests = UINT32_MAX, .rx_nack_accepted = UINT32_MAX,
        .rx_nack_cancelled = UINT32_MAX, .rx_nack_expired = UINT32_MAX,
        .rx_nack_rejected = UINT32_MAX, .rx_resync_requests = UINT32_MAX,
        .rx_delivered_decoded = UINT32_MAX, .rx_masked_frames = UINT32_MAX,
        .rx_recovered_decoded = UINT32_MAX, .rx_crc_failures = UINT32_MAX,
        .rx_decode_failures = UINT32_MAX, .rx_invalid_frames = UINT32_MAX,
        .rx_feedback_queue_failures = UINT32_MAX, .rx_nack_queued = UINT32_MAX,
        .rx_poll_max_us = UINT32_MAX, .rx_decode_max_us = UINT32_MAX,
        .rx_poll_audio_max = UINT32_MAX, .uac_rx_active = true, .uac_start_pending = true,
        .uac_frames_in = UINT32_MAX, .uac_sequence_lost = UINT32_MAX,
        .uac_sequence_reordered = UINT32_MAX, .uac_timeouts = UINT32_MAX,
        .uac_silence_fills = UINT32_MAX, .uac_buffer_samples = UINT32_MAX,
        .uac_overflow_drops = UINT32_MAX, .uac_level_drop_samples = UINT32_MAX,
        .uac_level_dup_samples = UINT32_MAX, .uac_timeline_enabled = true,
        .uac_timeline_started = true, .uac_timeline_cursor_samples = UINT64_MAX,
        .uac_timeline_late_samples = UINT64_MAX,
        .uac_timeline_local_retired_samples = UINT64_MAX,
        .uac_timeline_resume_skipped_samples = UINT64_MAX,
        .uac_timeline_pause_count = UINT32_MAX, .uac_timeline_max_gap_us = UINT32_MAX,
        .uac_timeline_start_us = UINT64_MAX, .uac_timeline_first_output_us = UINT64_MAX,
        .link_connected = true, .channel_pending = true, .channel = UINT8_MAX,
        .channel_state = UINT8_MAX, .rx_queue_drops = UINT32_MAX,
        .rx_invalid_lengths = UINT32_MAX, .tx_accepted = UINT32_MAX,
        .tx_callbacks = UINT32_MAX, .tx_immediate_failures = UINT32_MAX,
        .tx_soft_timeouts = UINT32_MAX, .tx_late_callbacks = UINT32_MAX,
        .tx_hard_recoveries = UINT32_MAX, .tx_recovery_failures = UINT32_MAX,
        .rx_stack_min_bytes = UINT32_MAX, .tx_stack_min_bytes = UINT32_MAX,
        .link_stack_min_bytes = UINT32_MAX, .rx_cache_sequence = UINT32_MAX,
        .rx_snapshot_drops = UINT32_MAX, .rx_event_drops = UINT32_MAX,
    };
    char buffer[DONGLE_DIAGNOSTICS_JSON_BUFFER_BYTES];
    char small[12];
    for (unsigned p = 0; p < DONGLE_DIAGNOSTICS_PARTS; ++p) {
        size_t n = dongle_diagnostics_format_part(&s, p, buffer, sizeof(buffer));
        assert(n > 0 && n <= DONGLE_DIAGNOSTICS_JSON_MAX_BYTES && n == strlen(buffer));
        assert((sizeof("OK DONGLE_TELEMETRY token=4294967295 ") - 1U) +
               n + sizeof("\n") <= 384U);
        assert(!dongle_diagnostics_format_part(&s, p, small, sizeof(small)) && small[0] == '\0');
        puts(buffer);
    }
    assert(!dongle_diagnostics_format_part(&s, DONGLE_DIAGNOSTICS_PARTS, buffer, sizeof(buffer)));
    s.boot_id = 0;
    assert(!dongle_diagnostics_format_part(&s, 0, buffer, sizeof(buffer)) && buffer[0] == '\0');
    return 0;
}
''')
    rows = [json.loads(line) for line in output.splitlines()]
    assert [row["p"] for row in rows] == list(range(13))
    for row in rows:
        assert row["e"] == "dongle.diag"
        assert row["v"] == 1 and row["n"] == 13
        assert row["b"] == "ffffffff" and row["r"] == 1
        assert row["q"] == row["run"] == row["arm"] == row["cx"] == row["s"] == 4294967295
        assert row["t"] == "18446744073709551615"
    assert rows[0]["co"] is None
    assert rows[1]["pe"] == rows[1]["pd"] == rows[1]["pp"] == 4294967295
    assert rows[4]["nr"] == rows[4]["sy"] == 4294967295
    assert rows[12]["xs"] == rows[12]["xl"] == rows[12]["xh"] == rows[12]["xf"] == 4294967295
    assert rows[9]["tc"] == rows[9]["tl"] == rows[9]["tr"] == "18446744073709551615"
    assert rows[10]["tk"] == rows[10]["tb"] == rows[10]["to"] == "18446744073709551615"


def test_dongle_diagnostics_is_cache_only_and_uses_dtr_independent_command_path():
    diagnostics = (MAIN / "dongle_diagnostics.c").read_text(encoding="utf-8")
    usb = (ROOT / "dongle/components/dongle_usb_device/dongle_usb_device.c").read_text(encoding="utf-8")
    runtime = (ROOT / "components/dual_end_runtime/dual_end_runtime.c").read_text(encoding="utf-8")
    link = (ROOT / "dongle/components/dual_end_espnow_link/dual_end_espnow_link.c").read_text(encoding="utf-8")
    assert "DIAG_CONTEXT" in diagnostics and "DONGLE_TELEMETRY" in diagnostics
    assert "audio_rx_telemetry_get_latest" in diagnostics
    assert "dual_end_runtime_cache_identity" in diagnostics
    assert "dual_end_espnow_link_get_diagnostics" in diagnostics
    command_offset = usb.index("s_diagnostics_command_cb(line")
    assert "tud_mounted()" in usb[:command_offset]
    assert "s_cdc_dtr" not in usb[command_offset:command_offset + 500]
    write_start = usb.index("static void cdc_write(")
    write_end = usb.index("static void cdc_write_len(", write_start)
    assert "s_cdc_dtr" not in usb[write_start:write_end]
    runtime_start = runtime.index("bool dual_end_runtime_cache_identity(")
    runtime_end = runtime.index("bool dual_end_runtime_owner_context(", runtime_start)
    assert all(token not in runtime[runtime_start:runtime_end]
               for token in ("heap_caps_", "uxTaskGetStack", "s_windows["))
    link_start = link.index("esp_err_t dual_end_espnow_link_get_diagnostics(")
    link_end = link.index("/* Request storage", link_start)
    assert "esp_now_" not in link[link_start:link_end]


def test_dongle_diagnostics_v2_keeps_v1_and_marks_strict_rx_cache_timing():
    output = compile_and_run(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "dongle_diagnostics_format.h"
int main(void) {
    dongle_diagnostics_snapshot_t s = {
        .runtime_ready = true, .resource_cache_ready = true, .rx_cache_ready = true,
        .context_set = true, .role = 1, .codec = 4,
        .boot_id = UINT32_MAX, .cache_sequence = UINT32_MAX,
        .run_id = UINT32_MAX, .arm_id = UINT32_MAX, .context_sequence = UINT32_MAX,
        .v2_run_id = 123, .v2_arm_id = 456, .v2_context_sequence = 789,
        .v2_context_set = true, .v2_context_ready = true,
        .v2_identity_ready = true, .v2_rx_session_id = UINT32_MAX,
        .v2_rx_session_generation = UINT32_MAX,
        .v2_uac_session_generation = UINT32_MAX,
        .v2_rx_cache_sequence = UINT32_MAX,
        .session_id = UINT32_MAX, .sampled_us = UINT64_MAX,
        .resource_sequence = UINT32_MAX, .resource_sampled_us = UINT64_MAX,
        .perf_ready_mask = UINT8_MAX,
        .perf_window_sequence = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX},
        .rx_active = true, .rx_accepted = UINT32_MAX, .rx_output_frames = UINT32_MAX,
        .rx_permanent_loss = UINT32_MAX, .rx_gap_events = UINT32_MAX,
        .rx_recovered = UINT32_MAX, .rx_duplicates = UINT32_MAX, .rx_late = UINT32_MAX,
        .rx_rejected = UINT32_MAX, .rx_abort_count = UINT32_MAX,
        .rx_nack_requests = UINT32_MAX, .rx_nack_accepted = UINT32_MAX,
        .rx_nack_cancelled = UINT32_MAX, .rx_nack_expired = UINT32_MAX,
        .rx_nack_rejected = UINT32_MAX, .rx_resync_requests = UINT32_MAX,
        .rx_delivered_decoded = UINT32_MAX, .rx_masked_frames = UINT32_MAX,
        .rx_recovered_decoded = UINT32_MAX, .rx_crc_failures = UINT32_MAX,
        .rx_decode_failures = UINT32_MAX, .rx_invalid_frames = UINT32_MAX,
        .rx_feedback_queue_failures = UINT32_MAX, .rx_nack_queued = UINT32_MAX,
        .rx_poll_max_us = UINT32_MAX, .rx_decode_max_us = UINT32_MAX,
        .rx_poll_audio_max = UINT32_MAX, .uac_rx_active = true, .uac_start_pending = true,
        .uac_frames_in = UINT32_MAX, .uac_sequence_lost = UINT32_MAX,
        .uac_sequence_reordered = UINT32_MAX, .uac_timeouts = UINT32_MAX,
        .uac_silence_fills = UINT32_MAX, .uac_buffer_samples = UINT32_MAX,
        .uac_overflow_drops = UINT32_MAX, .uac_level_drop_samples = UINT32_MAX,
        .uac_level_dup_samples = UINT32_MAX, .uac_timeline_enabled = true,
        .uac_timeline_started = true, .uac_timeline_cursor_samples = UINT64_MAX,
        .uac_timeline_late_samples = UINT64_MAX,
        .uac_timeline_local_retired_samples = UINT64_MAX,
        .uac_timeline_resume_skipped_samples = UINT64_MAX,
        .uac_timeline_pause_count = UINT32_MAX, .uac_timeline_max_gap_us = UINT32_MAX,
        .uac_timeline_start_us = UINT64_MAX, .uac_timeline_first_output_us = UINT64_MAX,
        .link_connected = true, .channel_pending = true, .channel = UINT8_MAX,
        .channel_state = UINT8_MAX, .rx_queue_drops = UINT32_MAX,
        .rx_invalid_lengths = UINT32_MAX, .tx_accepted = UINT32_MAX,
        .tx_callbacks = UINT32_MAX, .tx_immediate_failures = UINT32_MAX,
        .tx_soft_timeouts = UINT32_MAX, .tx_late_callbacks = UINT32_MAX,
        .tx_hard_recoveries = UINT32_MAX, .tx_recovery_failures = UINT32_MAX,
        .rx_stack_min_bytes = UINT32_MAX, .tx_stack_min_bytes = UINT32_MAX,
        .link_stack_min_bytes = UINT32_MAX, .rx_cache_sequence = UINT32_MAX,
        .rx_snapshot_drops = UINT32_MAX, .rx_event_drops = UINT32_MAX,
        .rx_owner_timing_ready = true, .rx_owner_sampled_us = UINT64_MAX,
        .rx_owner_age_us = UINT64_MAX, .rx_cache_published_us = UINT64_MAX,
        .rx_owner_stack_min_bytes = UINT32_MAX,
        .rx_telemetry_stack_min_bytes = UINT32_MAX,
        .diagnostics_stack_min_bytes = UINT32_MAX,
        .cdc_connected = true, .cdc_dtr = true,
        .cdc_rx_commands = UINT32_MAX, .cdc_rx_queue_overflows = UINT32_MAX,
        .cdc_response_drops = UINT32_MAX,
        .cdc_response_tx_queued_bytes = UINT64_MAX,
        .cdc_response_tx_incomplete = UINT32_MAX,
    };
    char buffer[DONGLE_DIAGNOSTICS_JSON_BUFFER_BYTES];
    for (unsigned p = 0; p < DONGLE_DIAGNOSTICS_V2_PARTS; ++p) {
        size_t n = dongle_diagnostics_format_v2_part(&s, p, buffer, sizeof(buffer));
        assert(n > 0 && n <= DONGLE_DIAGNOSTICS_JSON_MAX_BYTES && n == strlen(buffer));
        assert((sizeof("OK DONGLE_TELEMETRY_V2 token=4294967295 ") - 1U) +
               n + sizeof("\n") <= 384U);
        puts(buffer);
    }
    assert(!dongle_diagnostics_format_v2_part(&s, DONGLE_DIAGNOSTICS_V2_PARTS,
                                               buffer, sizeof(buffer)));
    assert(dongle_diagnostics_format_part(&s, 0, buffer, sizeof(buffer)) > 0);
    puts(buffer);
    assert(!dongle_diagnostics_format_part(&s, DONGLE_DIAGNOSTICS_PARTS,
                                            buffer, sizeof(buffer)));
    s.rx_owner_timing_ready = false;
    assert(dongle_diagnostics_format_v2_part(&s, 13, buffer, sizeof(buffer)) > 0);
    puts(buffer);
    s.rx_owner_timing_ready = true;
    s.rx_owner_sampled_us = 100;
    s.rx_owner_age_us = 23;
    s.rx_cache_published_us = 109;
    s.rx_owner_stack_min_bytes = 456;
    s.rx_telemetry_stack_min_bytes = 789;
    s.diagnostics_stack_min_bytes = 1234;
    s.cdc_connected = false;
    s.cdc_dtr = false;
    s.cdc_rx_commands = 12;
    s.cdc_rx_queue_overflows = 3;
    s.cdc_response_drops = 4;
    s.cdc_response_tx_queued_bytes = 9876543210;
    s.cdc_response_tx_incomplete = 2;
    assert(dongle_diagnostics_format_v2_part(&s, 13, buffer, sizeof(buffer)) > 0);
    puts(buffer);
    assert(dongle_diagnostics_format_v2_part(&s, 1, buffer, sizeof(buffer)) > 0);
    puts(buffer);
    return 0;
}
''')
    rows = [json.loads(line) for line in output.splitlines()]
    assert [row["p"] for row in rows[:14]] == list(range(14))
    assert all(row["v"] == 2 and row["n"] == 14 for row in rows[:14])
    assert all(row["run"] == 123 and row["arm"] == 456 and row["cx"] == 789
               for row in rows[:14])
    rx = rows[13]
    assert rx["xr"] == 1 and rx["xq"] == 4294967295
    assert rx["rxt"] == rx["rxa"] == rx["rxc"] == "18446744073709551615"
    assert rx["rxs"] == rx["rxw"] == rx["dgw"] == 4294967295
    cdc = rows[1]
    assert cdc["cc"] == cdc["cd"] == 1
    assert cdc["crx"] == cdc["cqo"] == cdc["crd"] == cdc["cti"] == 4294967295
    assert cdc["ctq"] == "18446744073709551615"
    assert rows[14]["v"] == 1 and rows[14]["n"] == 13 and rows[14]["p"] == 0
    assert rows[14]["run"] == rows[14]["arm"] == rows[14]["cx"] == 4294967295
    assert rows[15]["rxt"] is None and rows[15]["rxa"] is None and rows[15]["rxc"] is None
    assert rows[15]["rxs"] is None and rows[15]["rxw"] is None
    assert rows[16]["rxt"] == "100" and rows[16]["rxa"] == "23" and rows[16]["rxc"] == "109"
    assert rows[16]["rxs"] == 456 and rows[16]["rxw"] == 789 and rows[16]["dgw"] == 1234
    assert rows[17]["cc"] == rows[17]["cd"] == 0
    assert rows[17]["crx"] == 12 and rows[17]["cqo"] == 3 and rows[17]["crd"] == 4
    assert rows[17]["ctq"] == "9876543210" and rows[17]["cti"] == 2

    diagnostics = (MAIN / "dongle_diagnostics.c").read_text(encoding="utf-8")
    rx = (ROOT / "dongle/components/dual_end_audio/audio_rx_telemetry.c").read_text(encoding="utf-8")
    v2_dispatch = diagnostics.index("if (strncmp(line, telemetry_v2_prefix")
    v1_dispatch = diagnostics.index("if (strncmp(line, telemetry_prefix", v2_dispatch)
    assert v2_dispatch < v1_dispatch
    assert "audio_rx_telemetry_get_latest" in diagnostics
    assert "s_rx_recovery" not in diagnostics and "s_rx_codec" not in diagnostics
    assert "diagnostics_v2_context_t" in diagnostics
    assert "s_v2_context.set_us = UINT64_MAX;" in diagnostics
    assert "s_v2_context.required = true;" in diagnostics
    assert "s_v2_context.rx_cache_sequence_floor" in diagnostics
    assert "audio_rx_telemetry_cache_t latest_rx_cache" in diagnostics
    assert "latest_rx_ready ?\n        latest_rx_cache.sequence : 0" in diagnostics
    assert "next.rx_owner_sampled_us >= s_v2_context.set_us" in diagnostics
    assert "next.rx_cache_published_us >= s_v2_context.set_us" in diagnostics
    assert "snapshot.v2_context_sequence != v2_context.sequence" in diagnostics
    assert "runtime_identity_matches_rx" in diagnostics
    assert "snapshot.v2_identity_mixed" in diagnostics
    assert "mixed_identity" in diagnostics and "identity_pending" in diagnostics
    pending = diagnostics.index("ERR DONGLE_TELEMETRY_V2 token=%")
    format_start = diagnostics.index("char json[DONGLE_DIAGNOSTICS_JSON_BUFFER_BYTES]", pending)
    assert pending < format_start
    publish_start = rx.index("bool audio_rx_telemetry_publish")
    publish_end = rx.index("void audio_rx_telemetry_event", publish_start)
    assert "cache_published_us" not in rx[publish_start:publish_end]


def test_dongle_diagnostics_v2_identity_context_and_optional_start_guards():
    """Source-level guard for the nonportable RX/FreeRTOS paths.

    The portable formatter test above exercises the wire limit. These checks
    keep the v2 gate tied to immutable caches without running firmware or
    touching a device.
    """
    diagnostics = (MAIN / "dongle_diagnostics.c").read_text(encoding="utf-8")
    telemetry = (ROOT / "dongle/components/dual_end_audio/audio_rx_telemetry.c").read_text(
        encoding="utf-8"
    )
    receiver = (ROOT / "dongle/components/dual_end_audio/dual_end_audio.c").read_text(
        encoding="utf-8"
    )
    bridge = (ROOT / "dongle/components/dongle_uac_bridge/dongle_uac_bridge.c").read_text(
        encoding="utf-8"
    )
    format_header = (MAIN / "dongle_diagnostics_format.h").read_text(encoding="utf-8")

    assert "rx_identity_present" in diagnostics
    assert "runtime_identity_matches_rx" in diagnostics
    assert "uac_identity_mixed" in diagnostics
    assert "v2_identity_mixed" in diagnostics
    assert "rx_cache_sequence_floor" in diagnostics
    assert "latest_rx_cache.sequence" in diagnostics
    assert "bound_rx_generation" in diagnostics
    assert "bound_uac_generation" in diagnostics
    assert "snapshot.v2_rx_session_generation != v2_context.bound_rx_generation" in diagnostics
    assert "snapshot.v2_uac_session_generation != v2_context.bound_uac_generation" in diagnostics
    assert "ERR DONGLE_TELEMETRY_V2 token=%" in diagnostics
    assert "mixed_identity" in diagnostics
    assert "identity_pending" in diagnostics
    # The HMI-facing formatter contract makes a clear context unambiguous:
    # v1 clears its label, while v2 keeps the strict fresh-RX gate armed.
    assert "`co=1` means this strict gate is" in format_header
    assert "DIAG_CONTEXT <t> 0 0" in format_header
    assert "clears v1's legacy label" in format_header

    receiver_start = receiver.index("static esp_err_t receiver_start_common(void)")
    receiver_end = receiver.index("esp_err_t dual_end_audio_receiver_start(", receiver_start)
    receiver_start_body = receiver[receiver_start:receiver_end]
    assert "(void)audio_rx_telemetry_start();" in receiver_start_body
    assert "ESP_RETURN_ON_ERROR(audio_rx_telemetry_start" not in receiver_start_body
    assert "audio_rx_telemetry_enabled()" in receiver
    assert "snapshot.session_generation = s_rx_session_generation;" in receiver
    assert "snapshot.uac_session_generation = dongle_uac_bridge_session_generation();" in receiver
    assert "if (++s_snapshot_sequence == 0U) ++s_snapshot_sequence;" in telemetry

    start = telemetry.index("esp_err_t audio_rx_telemetry_start(void)")
    start_body = telemetry[start:]
    assert "s_telemetry_unavailable = true;" in start_body
    assert "RX telemetry disabled: allocation/task creation failed; audio continues" in start_body
    assert "return ESP_ERR_NO_MEM;" not in start_body
    assert "uint32_t dongle_uac_bridge_session_generation(void)" in bridge
