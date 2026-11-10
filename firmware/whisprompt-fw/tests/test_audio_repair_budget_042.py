"""Repair reservations and real driver-submission quota boundary behavior."""
from pathlib import Path
from test_audio_recovery_042 import ROOT, RECOVERY, _build_host_executable, _run


def test_repair_budget_executable(tmp_path: Path) -> None:
    executable, env = _build_host_executable(
        [ROOT / "tests/audio_repair_budget_test.c", RECOVERY / "audio_recovery_tx.c"],
        tmp_path,
    )
    result = _run([str(executable)], cwd=tmp_path, env=env)
    assert "audio repair budget tests passed" in result.stdout, result.stdout + result.stderr


def _function_body(source: str, name: str) -> str:
    start = source.index(name + "(")
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace : index + 1]
    raise AssertionError(f"unterminated C function: {name}")


def test_all_tx_producers_serialize_slot_admission() -> None:
    source = (
        ROOT / "scepter/components/dual_end_espnow_link/dual_end_espnow_link.c"
    ).read_text(encoding="utf-8")
    lock = "xSemaphoreTake(s_tx_submit_mutex, portMAX_DELAY)"
    acquire = "xQueueReceive(s_tx_free_queue"
    unlock = "xSemaphoreGive(s_tx_submit_mutex)"
    for name in (
        "tx_submit_and_wait",
        "tx_submit_priority_async",
        "tx_submit_audio_async",
    ):
        body = _function_body(source, name)
        assert lock in body, f"{name} bypasses the shared slot-admission mutex"
        assert acquire in body, f"{name} no longer acquires from the shared slot pool"
        assert body.index(lock) < body.index(acquire), (
            f"{name} observes/acquires the free slot before taking the submit mutex"
        )
        assert unlock in body, f"{name} never releases the slot-admission mutex"

    priority = _function_body(source, "tx_submit_priority_async")
    assert "audio_tx_slot_may_acquire" in priority


def test_042_priority_repair_runs_before_paced_original() -> None:
    source = (
        ROOT / "scepter/components/dual_end_espnow_link/dual_end_espnow_link.c"
    ).read_text(encoding="utf-8")
    body = _function_body(source, "tx_take_next_request")
    priority_receive = "xQueueReceive(s_tx_priority_queue"
    pacing_check = "audio_tx_pacing_wait_us"
    audio_receive = "xQueueReceive(s_tx_audio_queue"
    assert priority_receive in body
    assert pacing_check in body
    assert audio_receive in body
    assert body.index(priority_receive) < body.index(pacing_check) < body.index(audio_receive)
    assert "tx_repair_gap_open" not in body


def test_channel_pause_edges_gate_recovery_session() -> None:
    audio = (
        ROOT / "scepter/components/dual_end_audio/dual_end_audio.c"
    ).read_text(encoding="utf-8")
    link = (
        ROOT / "scepter/components/dual_end_espnow_link/dual_end_espnow_link.c"
    ).read_text(encoding="utf-8")

    pause = _function_body(link, "channel_pause_audio")
    assert pause.index("s_channel_audio_paused = true") < pause.index(
        "dual_end_espnow_link_discard_audio"
    )

    update = _function_body(audio, "sender_update_session_gate_locked")
    assert "audio_session_gate_transition" in update
    assert update.count("sender_invalidate_session_locked()") == 1
    assert update.count("sender_new_session_locked()") == 1

    worker = _function_body(audio, "sender_recovery_task")
    assert "dual_end_espnow_link_is_audio_paused()" in worker
    assert "event.received_us >= s_session_gate_open_us" in worker
    assert worker.index("sender_update_session_gate_locked") < worker.index(
        "sender_process_control_locked"
    )


def test_dongle_session_feedback_precedes_synchronous_control_and_nack() -> None:
    link = (
        ROOT / "dongle/components/dual_end_espnow_link/dual_end_espnow_link.c"
    ).read_text(encoding="utf-8")
    worker = _function_body(link, "audio_feedback_worker")
    session_definition = link[link.rindex(
        "static bool audio_feedback_worker_send_session("
    ):]
    session_send = _function_body(session_definition, "audio_feedback_worker_send_session")

    urgent = "audio_feedback_worker_send_session()"
    assert worker.index("radio_service_forced()") < worker.index(urgent)
    assert worker.index(urgent) < worker.index("audio_feedback_send_nack()")
    assert worker.index(urgent) < worker.index("tx_worker_service_sync()")
    assert "DUAL_END_AUDIO_SESSION_ACK" in session_send
    assert "DUAL_END_AUDIO_SESSION_RESYNC" in session_send
    assert "feedback.deadline_us, true" in session_send
    assert "if (result == ESP_ERR_NO_MEM)" in session_send
    assert "if (nack_pending && !allow_pending_nack)" in link


def test_scepter_logs_session_ack_diagnostics_separately() -> None:
    audio = (
        ROOT / "scepter/components/dual_end_audio/dual_end_audio.c"
    ).read_text(encoding="utf-8")
    link = (
        ROOT / "scepter/components/dual_end_espnow_link/dual_end_espnow_link.c"
    ).read_text(encoding="utf-8")
    recovery = _function_body(audio, "sender_recovery_task")
    assert "wrong_session=%" in recovery
    assert "AUDIO_CONTROL_TX schema=1" in link
    for field in (
        "submitted=%",
        "mac_ok=%",
        "mac_fail=%",
        "immediate_fail=%",
        "no_mem_retries=%",
    ):
        assert field in link
