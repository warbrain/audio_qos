"""Firmware VAD retirement and DSP v1 compatibility."""

from pathlib import Path
from test_audio_recovery_042 import _build_host_executable, _run
from test_runtime_codec import function


ROOT = Path(__file__).resolve().parents[1]
MIC = ROOT / "scepter/main/mic"


def test_scepter_mic_has_no_preencode_gate() -> None:
    mic_audio = (MIC / "mic_audio.c").read_text(encoding="utf-8-sig")
    mic_header = (MIC / "mic_audio.h").read_text(encoding="utf-8-sig")
    main_cmake = (ROOT / "scepter/main/CMakeLists.txt").read_text(
        encoding="utf-8-sig"
    )
    hardware = (ROOT / "scepter/main/scepter_hardware.c").read_text(
        encoding="utf-8-sig"
    )

    assert not (MIC / "mic_voice_gate.c").exists()
    assert not (MIC / "mic_voice_gate.h").exists()
    assert "mic_voice_gate" not in mic_audio
    assert "mic_voice_gate" not in main_cmake
    assert "gate_attenuated_frames" not in mic_header
    assert "mic_vad_process" not in mic_audio and "s_vad_" not in mic_audio
    assert "vad_enabled" not in mic_header and "vad_open" not in mic_header
    assert "gate_open=" not in mic_audio and "gate_open=" not in hardware


def test_retired_vad_items_do_not_apply_dsp_settings(tmp_path: Path) -> None:
    mic_audio = (MIC / "mic_audio.c").read_text(encoding="utf-8-sig")
    harness = tmp_path / "retired_vad.c"
    harness.write_text(r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef int esp_err_t;
enum { ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_NOT_SUPPORTED, ESP_ERR_TIMEOUT };
#define DUAL_END_AUDIO_CODEC_CAPABILITIES 3
#define portENTER_CRITICAL(lock) ((void)(lock), ++config_locks)
#define portEXIT_CRITICAL(lock) ((void)(lock))
#define xTaskNotifyGive(task) ((void)(task))
#define vTaskDelay(ticks) ((void)(ticks))
#define pdMS_TO_TICKS(ms) (ms)

static unsigned config_locks, owner_applies;
static int s_config_lock, s_mic_task_handle;
static bool s_capture_enabled = true, s_capture_io_running = true;
static uint32_t s_config_generation, s_applied_generation;
static uint16_t s_config[11] = {1, 0, 16, 16, 1, 80, 1, 7000, 1, 20000, 26000};
static uint16_t s_requested_config[11] = {1, 0, 16, 16, 1, 80, 1, 7000, 1, 20000, 26000};

static void mic_owner_apply_dsp_config(void)
{
    ++owner_applies;
    memcpy(s_config, s_requested_config, sizeof(s_config));
    s_applied_generation = s_config_generation;
}

static esp_err_t mic_codec_parameter(bool write, uint16_t value, uint16_t *actual)
{
    (void)write; (void)value; (void)actual;
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t mic_spatial_parameter(bool write, uint8_t item, uint16_t value, uint16_t *actual)
{
    (void)item;
    return mic_codec_parameter(write, value, actual);
}
''' + "\n".join(function(mic_audio, name) for name in ("mic_config_valid", "mic_audio_parameter")) + r'''

static void check_parameter(bool write, uint8_t item, uint16_t value,
                            esp_err_t status, uint16_t readback)
{
    uint16_t actual = UINT16_MAX;
    assert(mic_audio_parameter(write, item, value, &actual) == status);
    assert(actual == readback);
}

int main(void)
{
    check_parameter(false, 1, 0, ESP_OK, 0);
    check_parameter(false, 11, 0, ESP_OK, 48);
    check_parameter(false, 12, 0, ESP_OK, 300);
    check_parameter(false, 13, 0, ESP_OK, 1);
    check_parameter(true, 1, 0, ESP_OK, 0);
    check_parameter(true, 11, 48, ESP_OK, 48);
    check_parameter(true, 12, 300, ESP_OK, 300);
    check_parameter(true, 1, 1, ESP_ERR_NOT_SUPPORTED, 0);
    check_parameter(true, 11, 50, ESP_ERR_NOT_SUPPORTED, 48);
    check_parameter(true, 12, 320, ESP_ERR_NOT_SUPPORTED, 300);
    check_parameter(true, 1, 2, ESP_ERR_INVALID_ARG, 0);
    check_parameter(true, 11, 81, ESP_ERR_INVALID_ARG, 48);
    check_parameter(true, 12, 2001, ESP_ERR_INVALID_ARG, 300);
    check_parameter(true, 13, 1, ESP_ERR_INVALID_ARG, 1);
    assert(!config_locks && !owner_applies && !s_config_generation);

    check_parameter(false, 0, 0, ESP_OK, 1);
    check_parameter(false, 14, 0, ESP_OK, 1);
    check_parameter(true, 2, 8, ESP_OK, 8);
    check_parameter(false, 3, 0, ESP_OK, 16);
    assert(owner_applies == 1 && s_config_generation == 1);
    puts("Retired VAD DSP v1 compatibility verified");
    return 0;
}
''', encoding="utf-8")
    executable, env = _build_host_executable([harness], tmp_path)
    assert "compatibility verified" in _run([str(executable)], cwd=tmp_path, env=env).stdout
