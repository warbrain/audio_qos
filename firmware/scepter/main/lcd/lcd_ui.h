/*
 * LCD UI Module — Screen layout and status display
 * For the current ST7789 172x320 display
 *
 * Layout:
 *   y=0  ~ y=36   : Status bar (connection + battery)
 *   y=36 ~ y=155  : Logo area
 *   y=155~ y=320  : Reserved for animation / messages
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "lcd_driver.h"

/* Set to 0 to compile out every log emitted by lcd_ui.c only. */
#ifndef LCD_UI_LOG_ENABLE
#define LCD_UI_LOG_ENABLE 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LCD_OFFLINE_ORIENTATION_FLAT = 0,
    LCD_OFFLINE_ORIENTATION_UPRIGHT,
    LCD_OFFLINE_ORIENTATION_UPSIDE_DOWN,
    LCD_OFFLINE_ORIENTATION_LEFT,
    LCD_OFFLINE_ORIENTATION_RIGHT,
} lcd_offline_orientation_t;

/* ── Color theme — dark background ─────────────────────────────────── */
#define UI_COLOR_BG          LCD_COLOR_BLACK               /* Pure black bg */
#define UI_COLOR_STATUS_ON   LCD_COLOR_GREEN
#define UI_COLOR_STATUS_OFF  LCD_COLOR_RED               /* Bright red */
#define UI_COLOR_TEXT        LCD_COLOR_WHITE               /* Pure white */
#define UI_COLOR_TEXT_DIM    LCD_RGB(0x80, 0x80, 0x80)   /* Gray */
#define UI_COLOR_ACCENT      LCD_RGB(0xFF, 0x66, 0x00)   /* Orange accent */
#define UI_COLOR_DIVIDER     LCD_RGB(0x33, 0x33, 0x38)   /* Subtle divider */

/* ── Layout constants ──────────────────────────────────────────────── */
/* Top margin: shift entire UI down by N pixels */
#define UI_TOP_MARGIN        16

/* Status bar: two rows
 *   Row 1 (y=margin~margin+28)  : Connection status
 *   Row 2 (y=margin+28~margin+68): Battery icon + percentage
 */
#define UI_STATUSBAR_H       72
#define UI_STATUS_ROW1_H     32
#define UI_STATUS_ROW2_Y     UI_STATUS_ROW1_H
#define UI_STATUS_ROW2_H     (UI_STATUSBAR_H - UI_STATUS_ROW2_Y)

#define UI_LOGO_AREA_TOP     (UI_STATUSBAR_H + UI_TOP_MARGIN)
#define UI_LOGO_AREA_BOTTOM  (170 + UI_TOP_MARGIN)
#define UI_LOGO_AREA_H       (UI_LOGO_AREA_BOTTOM - UI_LOGO_AREA_TOP)
#define UI_DIVIDER_Y         UI_LOGO_AREA_BOTTOM
#define UI_CONTENT_TOP       (UI_DIVIDER_Y + 1)

/* ── API ───────────────────────────────────────────────────────────── */

/**
 * @brief Show the v1.0 power-on logo for 1 second.
 *        This is the first screen shown after LCD init.
 */
void lcd_ui_show_power_on_logo(void);

#if 0 /* Legacy boot splash (Frame 89).  Use lcd_ui_show_power_on_logo() for v1.0 UI. */
/**
 * @brief Show boot splash image (legacy Frame 89).
 * @deprecated Use lcd_ui_show_power_on_logo() for v1.0 UI.
 */
void lcd_ui_show_boot(void);
#endif

/**
 * @brief Show the v1.0 main screen (normal/0).
 */
void lcd_ui_show_main(void);

/**
 * @brief Set global button-pressed state and update the main screen.
 *        When pressed, the full-screen button_press image is shown.
 *        When released, the screen reverts to W_button.
 * @param pressed  true = any key is held down, false = all keys released
 */
void lcd_ui_set_button_pressed(bool pressed);

/**
 * @brief Notify the UI that the wheel has been scrolled.
 *        Starts the scroll animation or marks a pending re-trigger
 *        so the current sequence can branch/extend.
 */
void lcd_ui_on_wheel_event(void);

/**
 * @brief Show the low-battery full-screen image and stop animation.
 *        Call this when the battery is too low to continue normal operation.
 */
void lcd_ui_show_low_power(void);

/**
 * @brief Leave the low-battery screen and resume the mode matching the
 *        latest connection state.
 */
void lcd_ui_exit_low_power(void);

#if 0 /* Legacy SPIFFS picture viewer.  Commented out for v1.0 UI. */
/**
 * @brief Show a full-screen picture stored in SPIFFS
 * @param index  0 ~ 15, maps to /spiffs/pic01.bin ~ pic16.bin
 */
void lcd_ui_show_spiffs_picture(int index);
#endif

/**
 * @brief Start or stop continuous playback of the animation
 *        stored in the animation partition.
 * @param enable  true = start playback task, false = stop at end of current loop
 */
void lcd_ui_play_animation(bool enable);

/**
 * @brief Play the shared black-in boot/wake animation once at 12 fps.
 */
void lcd_ui_play_jump_animation(void);

/**
 * @brief Stop animation playback immediately.
 */
void lcd_ui_stop_animation(void);

/**
 * @brief Set animation playback frame rate.
 * @param fps  frames per second (1 ~ 30, default 10)
 */
void lcd_ui_set_animation_fps(int fps);

/* ── Software-controlled animation modes ───────────────────────────── */

/**
 * @brief Enter thinking mode.
 *        Replaces the normal loop: plays thinking/0 once, then randomly loops
 *        thinking/a/b/c. Wheel/scroll and button-press animations temporarily
 *        interrupt it and then resume the thinking loop.
 */
void lcd_ui_start_thinking(void);

/**
 * @brief Exit thinking mode and return to normal mode.
 */
void lcd_ui_stop_thinking(void);

/**
 * @brief Start the approve animation.
 *        Plays approve/0 once, then loops approve/a/b until the software stops
 *        it or it is interrupted by wheel/scroll/button. After interruption it
 *        returns to the mode that was active before approve (normal/thinking).
 */
void lcd_ui_start_approve(void);

/**
 * @brief Force stop the approve animation and return to the previous mode.
 */
void lcd_ui_stop_approve(void);

/**
 * @brief Start the done animation.
 *        Plays the done sequence once. Can be interrupted by wheel/scroll/button
 *        or software stop; afterwards it returns to the previous mode.
 */
void lcd_ui_start_done(void);

/**
 * @brief Force stop the done animation and return to the previous mode.
 */
void lcd_ui_stop_done(void);

/** One-shot category sequences. Thinking and approve first play their intro,
 * then select one branch using the category's local weight table. Done has a
 * single clip. Playback then returns to Normal or Offline according to the
 * latest link state. Ignored while offline, booting, sleeping, updating, or
 * showing the low-power screen. */
void lcd_ui_play_thinking_once(void);
void lcd_ui_play_approve_once(void);
void lcd_ui_play_done_once(void);

/* ── Sleep animation mode ───────────────────────────────────────────── */

/**
 * @brief Enter sleep mode.
 *        Plays sleep/0 once and then holds its last frame while the application
 *        shuts the LCD panel down. When a wake signal is received after panel
 *        resume, it plays black-in-12FPS-2 once and then selects Normal or
 *        Offline from the latest connection state.
 *
 *        The application is responsible for detecting keys/wheel/touchpad
 *        events and calling lcd_ui_sleep_interrupt() to wake the UI.
 */
void lcd_ui_start_sleep(void);

/**
 * @brief Wait until the sleep intro clip has finished.
 * @param timeout_ms Maximum time to wait.
 * @return true when sleep/0 finished and the UI is ready for panel shutdown;
 *         false when sleep was interrupted, cancelled, or timed out.
 */
bool lcd_ui_wait_sleep_intro_complete(uint32_t timeout_ms);

/**
 * @brief Force exit sleep mode immediately and return to the previous mode.
 */
void lcd_ui_stop_sleep(void);

/**
 * @brief Handle activity during the five-second pre-sleep screen by playing
 *        ANIM_CLIP_SLEEP_B once, then returning to the active UI.
 */
void lcd_ui_cancel_sleep_intro(void);

/**
 * @brief Start the post-light-sleep black-in sequence. The clip plays once;
 *        afterward the latest connection state selects Normal or Offline.
 */
void lcd_ui_start_wake_animation(void);

/**
 * @brief Freeze UI rendering and establish the physical-wake state before the
 *        ST7789 is reinitialized. Must be paired with
 *        lcd_ui_end_panel_resume() from the same task.
 * @return true when the resume barrier was acquired.
 */
bool lcd_ui_begin_panel_resume(void);

/**
 * @brief Draw the first physical-wake frame while the resume barrier is held.
 * @return true when a complete frame was written.
 */
bool lcd_ui_draw_panel_resume_frame(void);

/**
 * @brief Release the physical-wake render barrier and let the mode task play
 *        the complete one-shot wake clip.
 */
void lcd_ui_end_panel_resume(void);

/**
 * @brief Compatibility alias for lcd_ui_start_wake_animation().
 *        New code should cancel pre-sleep activity with
 *        lcd_ui_cancel_sleep_intro() and start a real post-sleep wake with
 *        lcd_ui_start_wake_animation().
 */
void lcd_ui_sleep_interrupt(void);

/**
 * @brief Returns true if the UI is currently in sleep mode.
 */
bool lcd_ui_is_sleeping(void);

/* ── Cursor (touchpad) animation ────────────────────────────────────── */

/**
 * @brief Notify the UI that the touchpad has generated a motion event.
 *        Plays cursor/0 once; if new touchpad events arrive during the
 *        intro, cursor/a is looped until the touchpad is idle for a short
 *        timeout.  Afterwards the UI returns to the mode active before the
 *        touchpad event.
 */
void lcd_ui_on_touchpad_event(void);

/**
 * @brief Notify the Offline UI of an IMU shake. While Offline, the IP
 *        foreground wobbles briefly while the background keeps its normal
 *        animation cadence. Ignored in all other modes.
 */
void lcd_ui_on_imu_shake(void);

/**
 * @brief Update the physical orientation used by the Offline IP layer.
 *        The render loop moves smoothly to the corresponding screen edge.
 *        The latest value is retained even while another UI mode is active.
 */
void lcd_ui_set_offline_orientation(lcd_offline_orientation_t orientation);

/* ── OTA updating animation mode ────────────────────────────────────── */

/**
 * @brief Enter OTA updating mode.
 *        Loops Updating-background as the background, plays Updating-begin
 *        once on top of it, then holds the last frame of Updating-begin.
 *        User input (keys/wheel/touchpad) is ignored while in this mode.
 */
void lcd_ui_start_ota(void);

/**
 * @brief Signal that the OTA process has finished.
 *        Plays Updating-end once; the system is expected to reboot afterwards.
 */
void lcd_ui_ota_finished(void);

#if 0 /* Legacy SPIFFS picture viewer.  Commented out for v1.0 UI. */
/**
 * @brief Cycle to the next full-screen picture on button press.
 *        Order: pic01 → pic02 → pic03 → pic01 → ...
 */
void lcd_ui_next_picture(void);

/**
 * @brief Cycle to the previous full-screen picture on button press.
 *        Order: pic01 → pic03 → pic02 → pic01 → ...
 */
void lcd_ui_prev_picture(void);
#endif

/**
 * @brief Initialize UI resources and show boot sequence:
 *        v1.0 power-on logo for 1 second, then play black-in-12FPS-2 once.
 *        Offline is not allowed to appear before the one-shot completes.
 */
void lcd_ui_init(void);

/**
 * @brief Select the connection animation mode.
 *
 *        When disconnected, the Offline background and character layers loop
 *        independently at 5 fps. When connected, the UI returns to the normal
 *        main-screen animation loop. Sleep, OTA, and low-power screens retain
 *        their higher priority.
 * @param connected true to enter normal mode; false to enter Offline mode
 */
void lcd_ui_set_connected(bool connected);

/**
 * @brief Update the value and charging state used by the Normal-mode battery animation.
 * @param percent    0~100. A negative value means unknown/invalid.
 * @param millivolts Battery voltage in mV, retained for future UI use.
 * @param charging   true while external charging power is present.
 */
void lcd_ui_set_battery_status(int percent, int millivolts, bool charging);

#if 0 /* Legacy status-bar / keypad / logo public APIs.  Commented out for v1.0 UI. */
/**
 * @brief Set connection status and redraw status bar
 * @param connected  true = online (green), false = offline (red)
 */
void lcd_ui_set_connected(bool connected);

/**
 * @brief Notify the UI that the wheel has been scrolled.
 *        Starts the scroll animation or marks a pending re-trigger
 *        so the current sequence can branch/extend.
 */
/**
 * @brief Set signal strength level and redraw status bar
 * @param level  0 ~ 4 (0 = no signal, 4 = full)
 */
void lcd_ui_set_signal_level(int level);

/**
 * @brief Keypad state for A/B/C/D buttons
 */
typedef enum {
    LCD_KEYPAD_WAITING = 0,   /**< No key pressed — shows "Waiting" */
    LCD_KEYPAD_PRESSING,      /**< A/B/C key pressed — shows "Pressing" + animated dots */
    LCD_KEYPAD_LISTENING,     /**< D key pressed — shows "Listening" + animated dots (prepared) */
} lcd_ui_keypad_state_t;

/**
 * @brief Set keypad state (unified API for A/B/C/D buttons)
 * @param state  LCD_KEYPAD_WAITING / PRESSING / LISTENING
 *
 * Usage:
 *   lcd_ui_set_keypad_state(LCD_KEYPAD_PRESSING);   // A/B/C pressed
 *   lcd_ui_set_keypad_state(LCD_KEYPAD_LISTENING);  // D pressed
 *   lcd_ui_set_keypad_state(LCD_KEYPAD_WAITING);    // released
 */
void lcd_ui_set_keypad_state(lcd_ui_keypad_state_t state);

/**
 * @brief Set keypad pressed state and redraw logo area
 * @param pressed  true = show pressed bitmap, false = show normal bitmap
 * @deprecated Use lcd_ui_set_keypad_state() instead
 */
void lcd_ui_set_keypad_pressed(bool pressed);

/**
 * @brief Set keypad talking/listening state (displays "Listening..." below keypad)
 * @param talking  true = show "Listening...", false = revert to normal state
 * @deprecated Use lcd_ui_set_keypad_state() instead
 */
void lcd_ui_set_keypad_talking(bool talking);

/**
 * @brief Draw the logo in the logo area
 * Call this after init or when you want to restore the logo.
 */
void lcd_ui_draw_logo(void);

/**
 * @brief Clear the content area (below divider) to background color
 */
void lcd_ui_clear_content(void);

/**
 * @brief Full screen refresh — redraw everything
 */
void lcd_ui_refresh(void);
#endif

#ifdef __cplusplus
}
#endif
