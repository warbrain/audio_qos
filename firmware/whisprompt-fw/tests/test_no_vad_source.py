"""Default VAD-off and optional runtime gate contract; PCM behavior is tested in test_mic_realtime."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_legacy_voice_gate_is_removed_and_runtime_vad_defaults_off() -> None:
    mic_dir = ROOT / "scepter/main/mic"
    mic_audio = (mic_dir / "mic_audio.c").read_text(encoding="utf-8-sig")
    mic_header = (mic_dir / "mic_audio.h").read_text(encoding="utf-8-sig")
    main_cmake = (ROOT / "scepter/main/CMakeLists.txt").read_text(
        encoding="utf-8-sig"
    )
    hardware = (ROOT / "scepter/main/scepter_hardware.c").read_text(
        encoding="utf-8-sig"
    )

    assert not (mic_dir / "mic_voice_gate.c").exists()
    assert not (mic_dir / "mic_voice_gate.h").exists()
    assert "mic_voice_gate" not in mic_audio
    assert "mic_voice_gate" not in main_cmake
    assert "gate_attenuated_frames" not in mic_header
    assert "s_config[13] = {1,0," in mic_audio
    assert "s_requested_config[13] = {1,0," in mic_audio
    assert "mic_vad_process(pcm_buf);" in mic_audio
    assert "s_vad_gain=1.0f;" in mic_audio
    assert "stats->vad_enabled" in hardware and "stats->vad_open" in hardware


def test_audio_callback_remains_unconditional_after_analysis() -> None:
    mic_audio = (ROOT / "scepter/main/mic/mic_audio.c").read_text(
        encoding="utf-8-sig"
    )

    analysis = mic_audio.index("mic_analysis_process_frame")
    callback = mic_audio.index("xQueueSend(s_frame_dispatch_blocks,&dispatch,0)")
    assert analysis < callback
    assert ".analysis=analysis" in mic_audio[analysis:callback]
