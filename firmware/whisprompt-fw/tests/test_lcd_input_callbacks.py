"""Source guards for nonblocking LCD input callbacks.

These checks preserve the scheduling boundary; they do not measure SPI or
audio deadlines on a physical device.
"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def _function(source: str, name: str) -> str:
    match = re.search(r"^[^\n;]+\b" + re.escape(name) + r"\([^\n]*\)\s*\{", source, re.M)
    assert match, name
    brace = source.index("{", match.start())
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def test_lcd_input_callbacks_only_enqueue_and_worker_owns_rendering() -> None:
    ui = (ROOT / "scepter/main/lcd/lcd_ui.c").read_text(encoding="utf-8")
    display = (ROOT / "scepter/main/lcd/lcd_display.c").read_text(encoding="utf-8")

    for name in ("lcd_ui_set_button_pressed", "lcd_ui_on_wheel_event", "lcd_ui_on_touchpad_event"):
        body = _function(ui, name)
        assert "ui_enqueue_input(" in body
        assert "ui_state_lock(" not in body
        assert "lcd_ui_stop_animation(" not in body
        assert "animation_draw" not in body
        assert "lcd_write_frame(" not in body

    enqueue = _function(ui, "ui_enqueue_input")
    assert "xQueueSend(ui_input_event_queue, &event, 0)" in enqueue
    assert "portMAX_DELAY" not in enqueue
    assert "vTaskDelay(" not in enqueue

    worker = _function(ui, "ui_input_task")
    assert "ui_handle_button_pressed(event.pressed)" in worker
    assert "ui_handle_wheel_event()" in worker
    assert "ui_handle_touchpad_event()" in worker
    assert "lcd_ui_cancel_sleep_intro()" in worker
    assert "lcd_ui_stop_animation()" not in worker

    stop_animation = _function(ui, "lcd_ui_stop_animation")
    assert "xEventGroupWaitBits(" in stop_animation
    assert "vTaskDelay(" not in stop_animation
    assert "while (animation_running)" not in stop_animation
    animation_task = _function(ui, "animation_task")
    assert "xEventGroupSetBits(animation_event_group, ANIMATION_STOPPED_BIT)" in animation_task

    for name in (
        "lcd_display_set_keypad_pressed",
        "lcd_display_on_wheel_event",
        "lcd_display_on_touchpad_event",
    ):
        body = _function(display, name)
        assert "lcd_ui_cancel_sleep_intro()" not in body
        assert "lcd_ui_set_button_pressed(" in body or "lcd_ui_on_" in body
        assert "lcd_write_frame(" not in body

