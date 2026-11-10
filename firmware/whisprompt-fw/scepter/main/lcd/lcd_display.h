#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "lcd_ui.h"

void lcd_display_start(void);
esp_err_t lcd_display_prepare_sleep(void);
void lcd_display_start_sleep_intro(void);
void lcd_display_cancel_sleep_intro(void);
void lcd_display_resume_after_sleep(void);
void lcd_display_set_mic(bool on);
void lcd_display_set_keypad_pressed(bool pressed);
void lcd_display_set_paired(bool paired);
void lcd_display_set_battery(int percent, int millivolts, bool charging);
void lcd_display_set_text(const char *text);
void lcd_display_set_ota_active(bool active);
void lcd_display_on_wheel_event(void);
void lcd_display_on_touchpad_event(void);
void lcd_display_on_imu_shake(void);
void lcd_display_set_offline_orientation(lcd_offline_orientation_t orientation);
void lcd_display_play_thinking_once(void);
void lcd_display_play_approve_once(void);
void lcd_display_play_done_once(void);
