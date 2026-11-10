from pathlib import Path
from host_build import ROOT, _build_host_executable, _run


def _bridge_includes(tests: Path) -> tuple[Path, ...]:
    return (
        tests / "bridge_stubs",
        tests / "uac_stubs",
        ROOT / "dongle/components/dongle_uac_bridge/include",
        ROOT / "components/dual_end_audio_recovery/include",
        ROOT / "dongle/components/dual_end_common/include",
    )


def test_bridge_pcm_behavior(tmp_path: Path) -> None:
    tests = Path(__file__).parent
    executable, env = _build_host_executable(
        [tests / "bridge_test.c"], tmp_path,
        extra_includes=_bridge_includes(tests),
    )
    result = _run([str(executable)], cwd=tmp_path, env=env)
    assert "audio UAC buffer tests passed (25 scenarios)" in result.stdout


def test_bridge_timeline_only_production_configuration(tmp_path: Path) -> None:
    tests = Path(__file__).parent
    executable, env = _build_host_executable(
        [tests / "bridge_timeline_only_test.c"], tmp_path,
        extra_includes=_bridge_includes(tests),
        extra_defines=("DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING=0",),
    )
    result = _run([str(executable)], cwd=tmp_path, env=env)
    assert "audio UAC timeline-only production test passed (legacy ring=0)" in result.stdout


def test_dongle_component_wires_timeline_and_compiles_out_legacy_ring() -> None:
    cmake = (ROOT / "dongle/components/dongle_uac_bridge/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    bridge = (ROOT / "dongle/components/dongle_uac_bridge/dongle_uac_bridge.c").read_text(
        encoding="utf-8"
    )
    app_main = (ROOT / "dongle/main/app_main.c").read_text(encoding="utf-8")

    assert "DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING=0" in cmake
    assert "#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING" in bridge
    assert "#define AUDIO_RING_BUFFER_BYTES (32 * 1024)" in bridge
    assert "#define DONGLE_UAC_BRIDGE_LEGACY_RING_STATIC_BYTES AUDIO_RING_BUFFER_BYTES" in bridge
    assert "#define DONGLE_UAC_BRIDGE_LEGACY_RING_STATIC_BYTES 0U" in bridge
    assert "dual_end_audio_receiver_start_timed(" in app_main
    assert "dongle_uac_bridge_write_timed_pcm" in app_main
    assert "dongle_uac_bridge_playout_clock" in app_main


def test_timeline_collision_is_narrow_soft_backpressure() -> None:
    source = (ROOT / "dongle/components/dual_end_audio/dual_end_audio.c").read_text(
        encoding="utf-8"
    )
    start = source.index("if (sink_result != ESP_OK) {")
    end = source.index("        break;", start)
    block = source[start:end]

    # Only the production timed timeline sink may treat a slot collision as a
    # dropped decoded frame. Every other sink/error still aborts RX recovery.
    assert "sink_result == ESP_ERR_NO_MEM" in block
    assert "s_timed_pcm_sink == dongle_uac_bridge_write_timed_pcm" in block
    assert "++s_timed_sink_drop_frames;" in block
    assert block.count("audio_recovery_rx_abort(&s_rx_recovery, now_us);") == 1
    assert "} else {" in block
