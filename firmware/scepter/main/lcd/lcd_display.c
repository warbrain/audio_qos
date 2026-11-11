#include "lcd_display.h"

#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_driver.h"
#include "lcd_ui.h"

static const char *TAG = "lcd_display";

#define LCD_LOW_POWER_PERCENT 10
#define LCD_SLEEP_INTRO_TIMEOUT_MS 2500
#define LCD_FIRST_FRAME_SETTLE_MS 34

static TaskHandle_t s_lcd_task;
static volatile bool s_started;
static volatile bool s_ui_ready;
static bool s_keypad_pressed;
static bool s_ota_active;
static bool s_low_power_shown;
static volatile bool s_panel_sleeping;
static volatile bool s_paired;
static bool s_pair_state_known;
static int s_battery_percent = -1;
static int s_battery_mv;
static bool s_battery_charging;

static void lcd_display_apply_state(void)
{
    if (!s_ui_ready) {
        return;
    }

    if (s_low_power_shown) {
        lcd_ui_show_low_power();
        return;
    }

    if (s_ota_active) {
        lcd_ui_start_ota();
        return;
    }

    lcd_ui_set_battery_status(s_battery_percent, s_battery_mv, s_battery_charging);
    lcd_ui_set_connected(s_paired);
    lcd_ui_set_button_pressed(s_keypad_pressed);
}

static void lcd_display_task(void *arg)
{
    (void)arg;

    esp_err_t ret = lcd_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LCD disabled: %s", esp_err_to_name(ret));
        s_lcd_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    /* Publish cached status before the boot sequence so its global overlays
     * are correct from the first visible frame. */
    lcd_ui_set_battery_status(s_battery_percent, s_battery_mv,
                              s_battery_charging);
    lcd_ui_set_connected(s_paired);
    lcd_ui_init();
    s_ui_ready = true;
    lcd_display_apply_state();

    ESP_LOGI(TAG, "LCD v1.0 animation UI started");
    s_lcd_task = NULL;
    vTaskDelete(NULL);
}

void lcd_display_start(void)
{
    if (s_started) {
        return;
    }
    s_started = true;

    BaseType_t ok = xTaskCreate(lcd_display_task, "lcd_display", 6144, NULL, 2, &s_lcd_task);
    if (ok != pdPASS) {
        ESP_LOGW(TAG, "create LCD task failed");
        s_started = false;
        s_lcd_task = NULL;
    }
}

esp_err_t lcd_display_prepare_sleep(void)
{
    if (!s_ui_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!lcd_ui_is_sleeping()) {
        lcd_ui_start_sleep();
    }
    if (!lcd_ui_wait_sleep_intro_complete(LCD_SLEEP_INTRO_TIMEOUT_MS)) {
        lcd_ui_stop_sleep();
        ESP_LOGW(TAG, "sleep intro interrupted or timed out; cancel panel sleep");
        return ESP_ERR_INVALID_STATE;
    }

    s_panel_sleeping = true;
    lcd_prepare_sleep();
    return ESP_OK;
}

void lcd_display_start_sleep_intro(void)
{
    if (!s_ui_ready || s_panel_sleeping || s_ota_active || s_low_power_shown) {
        return;
    }
    lcd_ui_start_sleep();
}

void lcd_display_cancel_sleep_intro(void)
{
    if (!s_ui_ready || s_panel_sleeping) {
        return;
    }
    lcd_ui_cancel_sleep_intro();
}

void lcd_display_resume_after_sleep(void)
{
    if (!s_ui_ready) {
        return;
    }

    /*
     * Keep s_panel_sleeping asserted for the whole transaction so key, wheel,
     * touch and remote-animation callbacks cannot replace the physical wake
     * state with sleep/b or another clip.
     */
    bool resume_locked = lcd_ui_begin_panel_resume();
    if (!resume_locked) {
        ESP_LOGW(TAG, "failed to acquire UI wake render barrier");
    }

    lcd_resume_after_sleep();

    bool first_frame_ready = resume_locked &&
                             lcd_ui_draw_panel_resume_frame();
    if (!first_frame_ready) {
        /* Never expose retained/undefined GRAM if the animation partition was
         * unavailable. A black frame is a safe fallback. */
        ESP_LOGW(TAG, "wake first frame unavailable; clearing panel");
        lcd_fill_screen(LCD_COLOR_BLACK);
    }

    lcd_display_on();
    /* Wait for two 60 Hz panel refresh periods while the backlight remains
     * off. This prevents liquid-crystal persistence from exposing the sleep
     * frame that was scanned before the GRAM replacement. */
    vTaskDelay(pdMS_TO_TICKS(LCD_FIRST_FRAME_SETTLE_MS));
    lcd_set_backlight(100);

    if (resume_locked) {
        lcd_ui_end_panel_resume();
    }
    s_panel_sleeping = false;

    /* A key edge can be recorded while panel rendering is blocked. Synchronize
     * the persistent physical state now; the UI will show it after the
     * one-shot wake animation if the key is still held. */
    lcd_ui_set_button_pressed(s_keypad_pressed);

    /* Apply high-priority state changes that may have been recorded while UI
     * rendering was blocked. */
    if (s_low_power_shown) {
        lcd_ui_show_low_power();
    } else if (s_ota_active) {
        lcd_ui_start_ota();
    }
}

void lcd_display_set_mic(bool on)
{
    (void)on;
}

void lcd_display_set_keypad_pressed(bool pressed)
{
    s_keypad_pressed = pressed;
    if (!s_ui_ready) {
        return;
    }

    /* The LCD worker owns sleep transitions and frame writes. Keep this
     * input callback bounded: it must not stop an animation or write SPI. */
    if (lcd_ui_is_sleeping() && s_panel_sleeping) return;
    lcd_ui_set_button_pressed(pressed);
}

void lcd_display_set_paired(bool paired)
{
    if (s_pair_state_known && s_paired == paired) {
        return;
    }

    s_pair_state_known = true;
    s_paired = paired;
    /* lcd_ui_set_connected() is safe before lcd_ui_init(): during boot it only
     * records the state, which is then used to choose Boot/Offline behavior. */
    lcd_ui_set_connected(paired);
}

void lcd_display_set_battery(int percent, int millivolts, bool charging)
{
    if (percent < 0) {
        percent = -1;
    } else if (percent > 100) {
        percent = 100;
    }

    bool visual_status_changed = s_battery_percent != percent ||
                                 s_battery_charging != charging;
    s_battery_percent = percent;
    s_battery_mv = millivolts;
    s_battery_charging = charging;
    if (s_ui_ready) {
        lcd_ui_set_battery_status(s_battery_percent, s_battery_mv, s_battery_charging);
    }

    bool low_power_required = !s_battery_charging && s_battery_percent >= 0 &&
                              s_battery_percent <= LCD_LOW_POWER_PERCENT;
    if (low_power_required) {
        bool entering_low_power = !s_low_power_shown;
        s_low_power_shown = true;
        if (s_ui_ready && !s_panel_sleeping &&
            (entering_low_power || visual_status_changed)) {
            lcd_ui_show_low_power();
        }
        return;
    }

    bool can_leave_low_power = s_battery_charging ||
                               (s_battery_percent > LCD_LOW_POWER_PERCENT);
    if (s_low_power_shown && can_leave_low_power) {
        s_low_power_shown = false;
        if (s_ui_ready && !s_panel_sleeping) {
            lcd_ui_exit_low_power();
            if (s_ota_active) {
                lcd_ui_start_ota();
            }
        }
    }
}

void lcd_display_set_text(const char *text)
{
    (void)text;
}

void lcd_display_set_ota_active(bool active)
{
    if (s_ota_active == active) {
        return;
    }

    s_ota_active = active;
    if (!s_ui_ready || s_panel_sleeping || s_low_power_shown) {
        return;
    }

    if (active) {
        lcd_ui_start_ota();
    } else {
        lcd_ui_ota_finished();
    }
}

void lcd_display_on_wheel_event(void)
{
    if (!s_ui_ready) {
        return;
    }

    if (lcd_ui_is_sleeping() && s_panel_sleeping) return;
    lcd_ui_on_wheel_event();
}

void lcd_display_on_touchpad_event(void)
{
    if (!s_ui_ready) {
        return;
    }

    if (lcd_ui_is_sleeping() && s_panel_sleeping) return;
    lcd_ui_on_touchpad_event();
}

void lcd_display_on_imu_shake(void)
{
    if (!s_ui_ready || s_panel_sleeping || s_ota_active || s_low_power_shown) {
        return;
    }
    lcd_ui_on_imu_shake();
}

void lcd_display_set_offline_orientation(lcd_offline_orientation_t orientation)
{
    /* Retain orientation even before UI startup or while another mode owns the
     * panel, so Offline can transition toward the correct position on entry. */
    lcd_ui_set_offline_orientation(orientation);
}

void lcd_display_play_thinking_once(void)
{
    if (!s_ui_ready || s_panel_sleeping || s_ota_active || s_low_power_shown) {
        return;
    }
    lcd_ui_play_thinking_once();
}

void lcd_display_play_approve_once(void)
{
    if (!s_ui_ready || s_panel_sleeping || s_ota_active || s_low_power_shown) {
        return;
    }
    lcd_ui_play_approve_once();
}

void lcd_display_play_done_once(void)
{
    if (!s_ui_ready || s_panel_sleeping || s_ota_active || s_low_power_shown) {
        return;
    }
    lcd_ui_play_done_once();
}
