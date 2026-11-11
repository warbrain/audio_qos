"""Host-only source guards for coherent dongle link status diagnostics.

These checks inspect source only. They do not build firmware or access serial,
audio, radio, or attached hardware.
"""

import re
from pathlib import Path


DONGLE = Path(__file__).resolve().parents[1]
LINK = DONGLE / "components/dual_end_espnow_link/dual_end_espnow_link.c"
APP = DONGLE / "main/app_main.c"


def _function_span(source: str, signature: str) -> tuple[int, int]:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for offset, character in enumerate(source[opening:], start=opening):
        if character == "{":
            depth += 1
        elif character == "}":
            depth -= 1
            if depth == 0:
                return start, offset + 1
    raise AssertionError(f"unterminated function: {signature}")


def _function_body(source: str, signature: str) -> str:
    start, end = _function_span(source, signature)
    return source[start:end]


def _assert_writes_are_confined(
    source: str, variable: str, signatures: tuple[str, ...]
) -> None:
    allowed = [_function_span(source, signature) for signature in signatures]
    offsets = [
        match.start()
        for match in re.finditer(rf"\b{re.escape(variable)}\s*=", source)
    ]
    assert offsets
    assert all(any(start <= offset < end for start, end in allowed) for offset in offsets)


def test_link_diagnostics_take_connected_and_rssi_as_one_locked_snapshot() -> None:
    source = LINK.read_text(encoding="utf-8")
    snapshot = _function_body(
        source, "static link_state_snapshot_t link_state_snapshot(void)"
    )
    assert snapshot.index("portENTER_CRITICAL(&s_peer_state_lock);") < snapshot.index(
        "snapshot.connected = s_connected;"
    ) < snapshot.index("snapshot.rssi = s_last_rssi;") < snapshot.index(
        "portEXIT_CRITICAL(&s_peer_state_lock);"
    )

    diagnostics = _function_body(
        source,
        "esp_err_t dual_end_espnow_link_get_diagnostics(dual_end_espnow_link_diagnostics_t *out)",
    )
    assert "const link_state_snapshot_t link_state = link_state_snapshot();" in diagnostics
    assert "out->connected = link_state.connected;" in diagnostics
    assert "int rssi = link_state.rssi;" in diagnostics


def test_link_state_writers_keep_connected_and_rssi_under_the_same_guard() -> None:
    source = LINK.read_text(encoding="utf-8")
    for signature, connected_update in (
        ("static link_state_snapshot_t link_state_invalidate(void)", "s_connected = false;"),
        (
            "static link_state_snapshot_t link_state_refresh_from_last_seen(int64_t now_us)",
            "s_connected = s_last_seen_us > 0 &&",
        ),
        ("static void mark_peer_seen(int rssi)", "s_connected = true;"),
    ):
        body = _function_body(source, signature)
        assert body.index("portENTER_CRITICAL(&s_peer_state_lock);") < body.index(
            connected_update
        ) < body.index("portEXIT_CRITICAL(&s_peer_state_lock);")

    seen = _function_body(source, "static void mark_peer_seen(int rssi)")
    assert seen.index("s_last_rssi = rssi;") < seen.index("s_connected = true;")
    _assert_writes_are_confined(
        source,
        "s_connected",
        (
            "static link_state_snapshot_t link_state_invalidate(void)",
            "static link_state_snapshot_t link_state_refresh_from_last_seen(int64_t now_us)",
            "static void mark_peer_seen(int rssi)",
            "esp_err_t dual_end_espnow_link_init(scepter_role_t role)",
        ),
    )
    _assert_writes_are_confined(
        source,
        "s_last_rssi",
        (
            "static void mark_peer_seen(int rssi)",
            "esp_err_t dual_end_espnow_link_init(scepter_role_t role)",
        ),
    )


def test_health_log_reads_the_callback_cache_as_one_locked_snapshot() -> None:
    source = APP.read_text(encoding="utf-8")
    update = _function_body(
        source, "static bool dongle_link_status_update(bool connected, int rssi)"
    )
    assert update.index("portENTER_CRITICAL(&s_link_status_lock);") < update.index(
        "s_link_connected = connected;"
    ) < update.index("s_link_rssi = rssi;") < update.index(
        "portEXIT_CRITICAL(&s_link_status_lock);"
    )

    snapshot = _function_body(
        source,
        "static dongle_link_status_snapshot_t dongle_link_status_snapshot(void)",
    )
    assert snapshot.index("portENTER_CRITICAL(&s_link_status_lock);") < snapshot.index(
        "snapshot.connected = s_link_connected;"
    ) < snapshot.index("snapshot.rssi = s_link_rssi;") < snapshot.index(
        "portEXIT_CRITICAL(&s_link_status_lock);"
    )
    assert "const dongle_link_status_snapshot_t link_status = dongle_link_status_snapshot();" in source
    assert "link_status.connected ? 1 : 0," in source
    assert "link_status.rssi," in source
    _assert_writes_are_confined(
        source,
        "s_link_connected",
        ("static bool dongle_link_status_update(bool connected, int rssi)",),
    )
    _assert_writes_are_confined(
        source,
        "s_link_rssi",
        ("static bool dongle_link_status_update(bool connected, int rssi)",),
    )

def test_dsp_status_four_logs_the_specific_local_forward_failure() -> None:
    source = APP.read_text(encoding="utf-8")
    request = _function_body(source, "static esp_err_t dongle_dsp_request(const uint8_t *data)")
    assert "const bool link_connected = dual_end_espnow_link_is_connected();" in request
    assert "!link_connected ? \"link_disconnected\"" in request
    assert "ret == ESP_ERR_NO_MEM ? \"control_queue_full\"" in request
    assert "ret == ESP_ERR_INVALID_STATE ? \"radio_not_ready\"" in request
    assert "DSP_FORWARD_REJECT token=" in request
    assert "esp_err_to_name(ret)" in request
    assert "hid_status=4" in request
    assert request.index("DSP_FORWARD_REJECT token=") < request.index("s_dsp_reply[10]=4")