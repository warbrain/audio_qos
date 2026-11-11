"""Execute production UAC producer/ISR and TinyUSB IN scheduling with a USB FIFO."""
from pathlib import Path
import re

from host_build import ROOT, _build_host_executable, _run


def _production_function(source: str, name: str) -> str:
    match = re.search(r"^[^\n;]+\b" + re.escape(name) + r"\([^\n]*\)\s*\{", source, re.M)
    assert match, name
    brace = source.index("{", match.start())
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end] + "\n"


def test_uac_mic_stream(tmp_path: Path) -> None:
    component = ROOT / "dongle/components/espressif__usb_device_uac"
    source = (component / "usb_device_uac.c").read_text(encoding="utf-8")
    # Keep the real state layout and every function under test verbatim.
    state = re.search(r"typedef struct \{\s*usb_phy_handle_t phy_hdl;.*?\} uac_device_t;", source, re.S)
    assert state
    names = ("mic_prefill_locked", "uac_device_reset_mic_session", "uac_device_stop_mic_stream",
             "uac_device_mic_session_drained", "uac_device_input_playout_time",
             "uac_device_mark_first_audio_sample", "tud_audio_tx_pre_load_cb",
             "uac_device_on_usb_mount", "uac_device_on_usb_unmount",
             "tud_suspend_cb", "tud_resume_cb",
             "tud_audio_set_itf_close_EP_cb", "tud_audio_set_itf_cb",
             "tud_audio_tx_done_isr", "usb_mic_task", "uac_device_get_stats")
    generated = state.group() + "\nstatic uac_device_t *s_uac_device;\n"
    generated += "\n".join(_production_function(source, name) for name in names)
    tinyusb = (ROOT / "dongle/components/dongle_usb_device/tinyusb/audio_device.c").read_text(encoding="utf-8")
    generated += "\n" + _production_function(tinyusb, "audiod_tx_packet_size")
    generated += "\n" + _production_function(tinyusb, "audiod_tx_xfer_isr")
    (tmp_path / "uac_mic_production.h").write_text(generated, encoding="utf-8")
    executable, env = _build_host_executable(
        [Path(__file__).parent / "uac_mic_stream_test.c"], tmp_path,
        extra_includes=(tmp_path, Path(__file__).parent / "uac_stubs", component / "include"),
    )
    result = _run([str(executable)], cwd=tmp_path, env=env)
    assert "UAC mic stream tests passed" in result.stdout


def test_uac_ready_before_enumeration() -> None:
    source = (ROOT / "dongle/components/dongle_usb_device/dongle_usb_device.c").read_text(encoding="utf-8")
    start = _production_function(source, "dongle_usb_device_start")
    assert start.index("uac_device_init(") < start.index("tinyusb_driver_install(")
    assert start.index("s_mouse_snapshot_queue = xQueueCreate") < start.index("tinyusb_driver_install(")


def test_mic_lifecycle_uses_tinyusb_event_dispatch_for_audio_part() -> None:
    component = ROOT / "dongle/components/espressif__usb_device_uac"
    source = (component / "usb_device_uac.c").read_text(encoding="utf-8")
    defaults = (ROOT / "dongle/sdkconfig.defaults").read_text(encoding="utf-8")
    cmake = (component / "CMakeLists.txt").read_text(encoding="utf-8")

    # ESP-IDF TinyUSB owns the global mount/unmount callbacks. The UAC component
    # supplies state handlers consumed by the registered composite event hook.
    assert "void tud_mount_cb(void)" not in source
    assert "void tud_umount_cb(void)" not in source
    assert "void uac_device_on_usb_mount(void)" in source
    assert "void uac_device_on_usb_unmount(void)" in source
    assert "CONFIG_USB_DEVICE_UAC_AS_PART=y" in defaults
    assert "SRCS usb_device_uac.c" in cmake
    composite = (ROOT / "dongle/components/dongle_usb_device/dongle_usb_device.c").read_text(encoding="utf-8")
    event_cb = _production_function(composite, "usb_event_cb")
    assert "uac_device_on_usb_mount()" in event_cb
    assert "uac_device_on_usb_unmount()" in event_cb
