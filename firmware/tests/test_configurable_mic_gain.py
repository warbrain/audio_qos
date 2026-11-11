"""Firmware regression guards for selectable gain, persistence and wire symmetry."""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def test_selected_build_gain_is_in_range_and_dsp_uses_runtime_snapshot() -> None:
    kconfig = (ROOT / "scepter/main/Kconfig.projbuild").read_text(encoding="utf-8-sig")
    defaults = (ROOT / "scepter/sdkconfig.defaults").read_text(encoding="utf-8-sig")
    mic_audio = (ROOT / "scepter/main/mic/mic_audio.c").read_text(encoding="utf-8-sig")

    assert "config MIC_OUTPUT_GAIN_X" in kconfig
    assert "range 1 32" in kconfig
    option = re.search(r"(?ms)^[ \t]*config MIC_OUTPUT_GAIN_X\b(.*?)(?=^[ \t]*config |\Z)", kconfig)
    assert option is not None
    configured_default = int(re.search(r"\bdefault\s+(\d+)", option.group(1)).group(1))
    selected = int(re.search(r"(?m)^CONFIG_MIC_OUTPUT_GAIN_X=(\d+)$", defaults).group(1))
    assert 1 <= configured_default <= 32 and 1 <= selected <= 32
    assert "CONFIG_MIC_OUTPUT_GAIN_X,CONFIG_MIC_OUTPUT_GAIN_X" in mic_audio
    assert "s_output_gain_x = CONFIG_MIC_OUTPUT_GAIN_X" in mic_audio
    assert "accumulator * (float)s_config[2 + channel]" in mic_audio
    assert "MIC_OUTPUT_GAIN_NUM" not in mic_audio
    assert mic_audio.index("accumulator * (float)s_config[2 + channel]") < mic_audio.index(
        "mic_shared_soft_limit_gain(stereo_peak)"
    )
    assert "if (pcm_frames == 0)" in mic_audio
    assert "memcpy(s_config,next,sizeof(next));" in mic_audio
    assert "s_output_gain_x=(uint8_t)next[2];" in mic_audio



def test_gain_is_persisted_and_protocol_is_symmetric() -> None:
    mic_audio = (ROOT / "scepter/main/mic/mic_audio.c").read_text(encoding="utf-8-sig")
    scepter_app = (ROOT / "scepter/main/app_main.c").read_text(encoding="utf-8-sig")
    dongle_app = (ROOT / "dongle/main/app_main.c").read_text(encoding="utf-8-sig")

    assert 'MIC_CONFIG_NVS_NAMESPACE "mic_cfg"' in mic_audio
    assert 'MIC_CONFIG_NVS_GAIN_KEY "fixed_gain_x"' in mic_audio
    assert "nvs_set_u8" in mic_audio and "nvs_commit" in mic_audio
    assert "DUAL_END_CONTROL_CMD_MIC_FIXED_GAIN" in scepter_app
    assert "DUAL_END_CONTROL_CMD_MIC_FIXED_GAIN" in dongle_app

    headers = [
        ROOT / "scepter/components/dual_end_common/include/dual_end_link_config.h",
        ROOT / "dongle/components/dual_end_common/include/dual_end_link_config.h",
    ]
    texts = [path.read_text(encoding="utf-8-sig") for path in headers]
    assert texts[0] == texts[1]
    assert "DUAL_END_CONTROL_CMD_MIC_FIXED_GAIN = 0x0A" in texts[0]
    assert "DUAL_END_MIC_GAIN_MIN_X 1" in texts[0]
    assert "DUAL_END_MIC_GAIN_MAX_X 32" in texts[0]

