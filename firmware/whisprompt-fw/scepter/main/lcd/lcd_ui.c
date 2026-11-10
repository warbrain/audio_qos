/*
 * LCD UI Module Implementation
 * Screen layout manager for the current ST7789 172x320 display
 */

#include "lcd_ui.h"
#include "anim_rle.h"
#include "lcd_font.h"
#include "u8g2_font_big22.h"
#include "u8g2_font_small.h"
#include "anim_clips.h"
#include "lcd_battery_anim_assets.h"

#if LCD_UI_LOG_ENABLE
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#else
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#endif
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_partition.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

static const char *TAG = "lcd_ui";

/* ── Animation constants ───────────────────────────────────────────── */
#define ANIMATION_PARTITION_LABEL "animation"
#define ANIMATION_MAGIC         "ANMR"
#define ANIMATION_HEADER_SIZE   16
#define ANIMATION_DEFAULT_FPS   7

/* ── Compressed animation context ──────────────────────────────────── */
typedef struct {
    uint16_t frame_count;
    uint16_t frame_width;
    uint16_t frame_height;
    uint32_t uncompressed_frame_size;
    uint32_t *compressed_sizes;
    uint32_t *compressed_offsets;
    uint8_t  *compressed_buf;
    uint32_t  compressed_buf_size;
    uint8_t  *frame_buf;
    bool      valid;
} anim_ctx_t;

static anim_ctx_t anim_ctx = {0};
static const esp_partition_t *animation_partition;

#if 0 /* ── Current state ───────────────────────────────────────────────── */
/* Legacy status-bar / keypad state variables.  Commented out for v1.0 UI. */
static bool  ui_connected = false;
static int   ui_battery_percent = 0;
static int   ui_battery_mv = 0;
static int   ui_signal_level = 4;   /* 0 ~ 4, 4 = full signal */
static bool  ui_keypad_pressed = false;
static bool  ui_keypad_talking = false;
static int   ui_dot_phase = 0;            /* 0~3: no dot → 1 → 2 → 3 dots */

static TaskHandle_t keypad_dot_task_handle __attribute__((unused)) = NULL;

#endif
/* ── Animation playback state ──────────────────────────────────────── */
static bool           animation_running = false;
static bool           animation_enabled = false;
static bool           animation_play_once = false;
static bool           animation_finished = false;
static int            animation_fps = ANIMATION_DEFAULT_FPS;
static int            animation_delay_ms = 1000 / ANIMATION_DEFAULT_FPS;
static TaskHandle_t   animation_task_handle = NULL;
static bool            anim_is_compressed = false;
static int             anim_clip_id = -1;
static int             anim_clip_start = 0;
static int             anim_clip_count = 0;
static const uint8_t   *anim_clip_frame_order = NULL;
static volatile bool   ui_connected = false;
static volatile bool   ui_connection_known = false;

/* ── UI mode/state machine ─────────────────────────────────────────── */
typedef enum {
    UI_MODE_NORMAL = 0,     /* Normal: main0 + a-f loop */
    UI_MODE_OFFLINE,        /* Offline: background + character loop */
    UI_MODE_THINKING,       /* Thinking: thinking0 once, then a-c loop */
    UI_MODE_APPROVE,        /* Approve: approve0 once, then a-b loop (transient) */
    UI_MODE_DONE,           /* Done: play done once (transient) */
    UI_MODE_SLEEP,          /* Sleep: play 0 once, hold last frame, wake with b */
    UI_MODE_UPDATING,       /* OTA updating: background loop + begin/end overlay */
    UI_MODE_TEST_ONCE,      /* Weighted one-shot category, then Normal/Offline */
} ui_mode_t;

typedef enum {
    UI_STATE_IDLE = 0,      /* Mode loop can pick next clip */
    UI_STATE_CLIP_PLAYING,  /* Playing an animation clip */
    UI_STATE_SCROLL_ANIM,   /* Wheel scroll animation */
    UI_STATE_CURSOR_ANIM,   /* Touchpad cursor animation */
    UI_STATE_BUTTON_PRESSED,/* Any-key full-screen overlay */
    UI_STATE_LOW_POWER,     /* Low-battery screen (highest priority) */
} ui_state_t;

static ui_mode_t  ui_mode = UI_MODE_NORMAL;
static ui_state_t ui_state = UI_STATE_IDLE;
static bool       boot_complete = false;
/* Keep the physical state independently from the currently visible UI state.
 * A held key must continue to own the screen even if another UI task was
 * already preparing the next Normal clip when the press edge arrived. */
static volatile bool button_held = false;
static volatile bool button_suppress_until_release = false;
static volatile int  battery_anim_percent = -1;
static volatile int  battery_anim_mv = 0;
static volatile bool battery_anim_charging = false;

#define BATTERY_STATUS_X            10
#define BATTERY_STATUS_Y            22
#define BATTERY_STATUS_BODY_W       30
#define BATTERY_STATUS_BODY_H       16
#define BATTERY_STATUS_TERMINAL_W   4
#define BATTERY_STATUS_TERMINAL_H   6
#define BATTERY_STATUS_INNER_W      24
#define BATTERY_STATUS_INNER_H      10
#define BATTERY_STATUS_ICON_W       38
#define BATTERY_STATUS_ICON_GAP     4
#define BATTERY_STATUS_DIGIT_GAP    1
#define BATTERY_STATUS_PERCENT_GAP  1

/* For transient modes (approve/done/sleep), remember where to return. */
static ui_mode_t  ui_previous_mode = UI_MODE_NORMAL;

/* Flags for first-entry intro clips. */
static bool       thinking_0_played = false;
static bool       approve_0_played = false;

typedef enum {
    UI_TEST_NONE = 0,
    UI_TEST_THINKING,
    UI_TEST_APPROVE,
    UI_TEST_DONE,
} ui_test_once_t;

static volatile ui_test_once_t ui_test_once;
static int ui_test_clip_index;
static int ui_test_clip_count;
static uint8_t ui_test_clips[2];

/* Sleep animation phase and external wake signal. */
typedef enum {
    SLEEP_PHASE_INTRO = 0,  /* Play sleep/0 once */
    SLEEP_PHASE_READY,      /* Intro done; panel can now be turned off */
    SLEEP_PHASE_WAKE,       /* Play the selected wake clip once */
} sleep_phase_t;

typedef enum {
    SLEEP_WAKE_BLACK_IN = 0, /* Boot/physical light-sleep wake */
    SLEEP_WAKE_SLEEP_B,      /* 5-second pre-sleep screen interruption */
} sleep_wake_animation_t;

static volatile sleep_phase_t sleep_phase = SLEEP_PHASE_INTRO;
static volatile sleep_wake_animation_t sleep_wake_animation = SLEEP_WAKE_BLACK_IN;
static volatile bool sleep_intro_started = false;
static volatile bool sleep_wake_clip_started = false;
static volatile bool panel_resume_active = false;
static TaskHandle_t panel_resume_owner = NULL;

/* The source directory is naturally sorted as sleep_1..sleep_7,W_sleep,
 * while the intended entrance is W_sleep,sleep_1..sleep_7. Keep the binary
 * unchanged and explicitly play the semantic order here. */
static const uint8_t sleep_intro_frame_order[] = {7, 0, 1, 2, 3, 4, 5, 6};

/* black-in-12FPS-2 is clip 0 in animation.bin and is used by boot and a real
 * post-light-sleep wake. A pre-sleep-screen interruption uses sleep/b. */
#define BOOT_WAKE_CLIP          ANIM_CLIP_BOOT
#define BOOT_WAKE_FPS           12
#define SCREEN_SAVER_WAKE_CLIP  ANIM_CLIP_SLEEP_B
#define SCREEN_SAVER_WAKE_FPS   7

/* Boot and wake suppress Offline transitions while the ESP-NOW peer has at
 * least two seconds to re-establish its link. The full black-in clip owns the
 * screen first; because it lasts longer than two seconds, the latest link
 * state is applied immediately after that one-shot finishes. */
#define LINK_DETECTION_WAIT_MS 2000
static volatile bool link_wait_active;
static TickType_t link_wait_deadline;

/* Offline is a two-layer 5 fps animation: a 141-frame full-screen
 * background and an 8-frame transparent character overlay. */
#define OFFLINE_FPS               5
#define OFFLINE_FRAME_PERIOD_MS   (1000 / OFFLINE_FPS)
#define OFFLINE_SHAKE_STEP_COUNT  12
#define OFFLINE_POSITION_STEP_PX  12
#define OFFLINE_POSITION_TOP_Y    (-96)
#define OFFLINE_POSITION_BOTTOM_Y 84
#define OFFLINE_POSITION_LEFT_X   (-26)
#define OFFLINE_POSITION_RIGHT_X  26
static int      offline_bg_frame_index = 0;
static int      offline_ip_frame_index = 0;
static uint8_t *offline_bg_buf = NULL;
static volatile lcd_offline_orientation_t offline_orientation =
    LCD_OFFLINE_ORIENTATION_FLAT;
static int offline_position_x;
static int offline_position_y;
static volatile uint32_t offline_shake_generation;
static uint32_t offline_shake_seen_generation;
static int offline_shake_step = OFFLINE_SHAKE_STEP_COUNT;

/* IP-only large wobble, based on the reference video's broad diagonal motion.
 * At 5 fps this lasts 2.4 seconds and returns exactly to the original
 * position. The background is never offset. */
static const int8_t offline_shake_x[OFFLINE_SHAKE_STEP_COUNT] = {
      0, -28,  30, -32,  28, -24,  22, -18,  14, -10,  6, 0,
};
static const int8_t offline_shake_y[OFFLINE_SHAKE_STEP_COUNT] = {
      0,  18, -24, -38,  22,  36, -30,  24, -18,  12, -6, 0,
};

/* ── Random clip weights ───────────────────────────────────────────── */
/* Normal mode: a 82%, b 10%, c 3%, d 2%, e 2%, f 1% */
static const int normal_clip_weights[ANIM_CLIP_COUNT] = {
    [ANIM_CLIP_A] = 82,
    [ANIM_CLIP_B] = 10,
    [ANIM_CLIP_C] = 3,
    [ANIM_CLIP_D] = 2,
    [ANIM_CLIP_E] = 2,
    [ANIM_CLIP_F] = 1,
};

/* Thinking mode: a 50%, b 30%, c 20% */
static const int thinking_clip_weights[ANIM_CLIP_COUNT] = {
    [ANIM_CLIP_THINKING_A] = 50,
    [ANIM_CLIP_THINKING_B] = 30,
    [ANIM_CLIP_THINKING_C] = 20,
};

/* Approve mode: a 70%, b 30% */
static const int approve_clip_weights[ANIM_CLIP_COUNT] = {
    [ANIM_CLIP_APPROVE_A] = 70,
    [ANIM_CLIP_APPROVE_B] = 30,
};

/* Time to keep the idle main screen visible before starting the next clip.
 * Set to 0 to start the next random clip immediately after returning to 0. */
#define NORMAL_MAIN0_HOLD_MS   0

/* ── Scroll animation state ────────────────────────────────────────── */
#define SCROLL_FPS               7

typedef enum {
    SCROLL_STATE_IDLE = 0,
    SCROLL_STATE_BASE,
    SCROLL_STATE_BRANCH,
    SCROLL_STATE_END,
} scroll_state_t;

static scroll_state_t scroll_state = SCROLL_STATE_IDLE;
static bool           scroll_branch_is_a = true;
static QueueHandle_t  scroll_event_queue = NULL;
static TaskHandle_t   scroll_task_handle = NULL;

/* ── Cursor (touchpad) animation state ──────────────────────────────── */
#define CURSOR_FPS                 15
#define CURSOR_CLASSIFY_MS         200
#define CURSOR_IDLE_TIMEOUT_MS     200

typedef enum {
    CURSOR_STATE_IDLE = 0,
    CURSOR_STATE_CLASSIFY,
    CURSOR_STATE_SINGLE,
    CURSOR_STATE_MULTI,
} cursor_state_t;

static volatile cursor_state_t cursor_state = CURSOR_STATE_IDLE;
static volatile uint32_t       cursor_generation;
static QueueHandle_t           cursor_event_queue = NULL;
static TaskHandle_t            cursor_task_handle = NULL;

/* ── OTA updating animation state ───────────────────────────────────── */
#define UPDATING_FPS               7
#define UPDATING_FRAME_PERIOD_MS   (1000 / UPDATING_FPS)

typedef enum {
    UPDATING_PHASE_BEGIN = 0,
    UPDATING_PHASE_WAIT_END,
    UPDATING_PHASE_END,
    UPDATING_PHASE_DONE,
} updating_phase_t;

static updating_phase_t updating_phase = UPDATING_PHASE_BEGIN;
static int              updating_bg_frame_index = 0;
static int              updating_fg_frame_index = 0;
static uint8_t         *updating_bg_buf = NULL;
static TaskHandle_t     updating_task_handle = NULL;

#if 0 /* ── Row-level framebuffers for flicker-free rendering ─────────────── */
/* Legacy status-bar / keypad-status framebuffers.  Commented out for v1.0 UI. */
static uint16_t *row1_buf = NULL;  /* 172 x 32 pixels, RGB565 */
static uint16_t *row2_buf = NULL;  /* 172 x 40 pixels, RGB565 */

/* ── Keypad status text framebuffer ─────────────────────────────────── */
static uint16_t *keypad_status_buf = NULL;  /* 172 x 32 pixels, RGB565 */
#define KEYPAD_STATUS_H  32

#endif
/* ── UI mutexes ────────────────────────────────────────────────────── */
static SemaphoreHandle_t ui_draw_mutex = NULL;
/* Serializes button-state transitions with synchronous main-frame drawing
 * and animation startup. ui_draw_mutex alone is not sufficient because an
 * animation can be armed without drawing its first frame immediately. */
static SemaphoreHandle_t ui_state_mutex = NULL;
static EventGroupHandle_t animation_event_group = NULL;
#define ANIMATION_STOPPED_BIT BIT0

typedef enum {
    UI_INPUT_BUTTON = 0,
    UI_INPUT_WHEEL,
    UI_INPUT_TOUCH,
} ui_input_kind_t;

typedef struct {
    ui_input_kind_t kind;
    bool pressed;
} ui_input_event_t;

#define UI_INPUT_EVENT_QUEUE_LENGTH 8U
static QueueHandle_t ui_input_event_queue = NULL;
static TaskHandle_t ui_input_task_handle = NULL;

#if 0 /* ── Logo cursor blink ───────────────────────────────────────────────
 * Exact orange pixels in the regenerated logo_bitmap.h:
 *   bitmap y=23: x=155~159  (width 5)
 *   bitmap y=24: x=154~160  (width 7)
 *   bitmap y=25: x=154~160  (width 7)
 * We draw three separate 1px-tall strips to avoid touching non-cursor pixels.
 * Commented out for v1.0 UI. */
#define LOGO_CURSOR_COLOR   LCD_RGB(0xFF, 0x79, 0x1B)

static bool logo_cursor_visible = true;
static TaskHandle_t cursor_task_handle __attribute__((unused)) = NULL;

static void __attribute__((unused)) draw_logo_cursor(int logo_x, int logo_y, lcd_color_t color)
{
    /* Row y=23 (relative): x=155~159, width=5 */
    lcd_fill_rect(logo_x + 155, logo_y + 23, 5, 1, color);
    /* Row y=24: x=154~160, width=7 */
    lcd_fill_rect(logo_x + 154, logo_y + 24, 7, 1, color);
    /* Row y=25: x=154~160, width=7 */
    lcd_fill_rect(logo_x + 154, logo_y + 25, 7, 1, color);
}

#endif
/* ── Forward declarations ──────────────────────────────────────────── */
#if 0 /* Legacy UI helpers (status bar / keypad / logo), commented out for v1.0 UI. */
static void draw_status_bar(void);
static void draw_row1_battery(void);
static void draw_row2_connection(void);
static void draw_signal_icon(int x, int y, int level, lcd_color_t color);
static void draw_battery_icon(int x, int y, int percent, lcd_color_t color);
static void draw_logo_internal(void);
static void draw_keypad_only(void);
static void draw_keypad_status(void);
static void logo_cursor_task(void *pvParameters);
static void keypad_dot_task(void *pvParameters);
#endif
static void animation_task(void *pvParameters);
static void scroll_control_task(void *pvParameters);
static void ui_input_task(void *pvParameters);
static bool ui_enqueue_input(ui_input_kind_t kind, bool pressed);
static void ui_handle_button_pressed(bool pressed);
static void ui_handle_wheel_event(void);
static void ui_handle_touchpad_event(void);
static bool animation_draw_frame(int frame_index);
static bool animation_draw_frame_locked(int frame_index);
static bool animation_draw_clip_frame(int clip_id, int frame_offset);
static bool animation_decompress_frame(int frame_index, uint8_t *dst);
static void battery_blit_status_to_frame_buf(uint8_t *dst, size_t dst_len);
static void cursor_animation_task(void *pvParameters);
static void cursor_cancel_sequence(bool return_to_normal);
static void updating_task(void *pvParameters);
static bool offline_draw_composite(int bg_frame, int ip_frame, int ip_dx, int ip_dy);
static void offline_reset_animation_state(void);
static void lcd_ui_show_main0(void);
static void lcd_ui_play_clip(int clip_id, int fps, bool once);
static void lcd_ui_play_clip_ordered(int clip_id,
                                     const uint8_t *frame_order,
                                     int frame_count,
                                     int fps,
                                     bool once);
static bool lcd_ui_play_scroll_clip(int clip_id);
static int  pick_random_clip(const int *weights);
static void ui_abort_current(void);
static void ui_start_test_once(ui_test_once_t test);
static void link_wait_begin(TickType_t start_tick);
static void link_wait_finish(void);
static bool link_wait_expired(void);
static void mode_loop_task(void *pvParameters);

static void ui_state_lock(void)
{
    if (ui_state_mutex != NULL) {
        xSemaphoreTake(ui_state_mutex, portMAX_DELAY);
    }
}

static void ui_state_unlock(void)
{
    if (ui_state_mutex != NULL) {
        xSemaphoreGive(ui_state_mutex);
    }
}

static bool ui_button_overlay_requested(void)
{
    return button_held &&
           !button_suppress_until_release &&
           ui_mode != UI_MODE_OFFLINE &&
           ui_mode != UI_MODE_SLEEP &&
           ui_mode != UI_MODE_UPDATING &&
           ui_state != UI_STATE_LOW_POWER;
}

/* ── UI init / refresh ─────────────────────────────────────────────── */

void lcd_ui_show_power_on_logo(void)
{
    ESP_LOGI(TAG, "Show v1.0 power-on logo");
    /* power/ contains low-power first and power-on logo second. */
    animation_draw_clip_frame(ANIM_CLIP_POWER, 1);
}

#if 0 /* Legacy boot splash (Frame 89).  Use lcd_ui_show_power_on_logo() for v1.0 UI. */
void lcd_ui_show_boot(void)
{
    ESP_LOGI(TAG, "Show boot splash (Frame 89)");
    lcd_draw_bitmap(0, 0, BOOT_WIDTH, BOOT_HEIGHT, boot_bitmap);
}
#endif

void lcd_ui_show_main(void)
{
    lcd_ui_show_main0();
}

static void lcd_ui_show_main0(void)
{
    /*
     * Serialize the complete synchronous draw with a button transition.
     * Without this lock, mode_loop_task could pass its IDLE test, wait for the
     * LCD draw mutex, and overwrite the button frame after the key callback.
     */
    ui_state_lock();
    if (ui_state != UI_STATE_IDLE || ui_button_overlay_requested()) {
        ui_state_unlock();
        return;
    }

    ESP_LOGI(TAG, "Show v1.0 main screen (normal/0)");
    animation_draw_clip_frame(ANIM_CLIP_MAIN0, 0);
    ui_state_unlock();
}

/* Caller must hold ui_state_mutex. */
static void ui_apply_button_overlay_locked(void)
{
    /* Keys have priority over the touchpad. Once interrupted, the cursor
     * sequence is discarded and key release returns to Normal. */
    if (cursor_state != CURSOR_STATE_IDLE) {
        cursor_cancel_sequence(true);
    }

    /* Interrupt any playing clip or scroll animation. */
    if (ui_state == UI_STATE_CLIP_PLAYING) {
        lcd_ui_stop_animation();
    }
    if (ui_state == UI_STATE_SCROLL_ANIM) {
        /* Scroll clips run on animation_task, so stop the shared player before
         * the static button frame takes ownership of the screen. */
        lcd_ui_stop_animation();
        scroll_state = SCROLL_STATE_IDLE;
        if (scroll_event_queue != NULL) {
            xQueueReset(scroll_event_queue);
        }
    }

    /* Approve / Done are transient: a button press cancels them and
     * returns to the previous mode. Normal / Thinking just pause. */
    if (ui_mode == UI_MODE_APPROVE || ui_mode == UI_MODE_DONE) {
        ui_mode = ui_previous_mode;
    }

    ui_state = UI_STATE_BUTTON_PRESSED;
    /* Button clip frame 0 is button_press.png. */
    animation_draw_clip_frame(ANIM_CLIP_BUTTON, 0);
}

static void ui_handle_button_pressed(bool pressed)
{
    ui_state_lock();

    bool changed = button_held != pressed;
    button_held = pressed;

    /* A release always clears wheel suppression, including releases received
     * while Offline, Sleeping, Updating, or Low Power owns the screen. */
    if (!pressed) {
        button_suppress_until_release = false;
    }

    if (!changed) {
        ui_state_unlock();
        return;
    }

    ESP_LOGI(TAG, "Button pressed state: %s (mode=%d state=%d)",
             pressed ? "true" : "false", (int)ui_mode, (int)ui_state);

    if (ui_mode == UI_MODE_OFFLINE || ui_mode == UI_MODE_SLEEP ||
        ui_mode == UI_MODE_UPDATING || ui_state == UI_STATE_LOW_POWER) {
        ui_state_unlock();
        return;
    }

    if (pressed) {
        /* If a wheel event interrupted a held button, ignore the overlay until
         * the physical key is released. */
        if (!button_suppress_until_release) {
            ui_apply_button_overlay_locked();
        }
    } else {
        if (ui_state == UI_STATE_BUTTON_PRESSED) {
            ui_state = UI_STATE_IDLE;
            /* The mode loop will resume the current mode. */
        }
    }

    ui_state_unlock();
}

static bool ui_enqueue_input(ui_input_kind_t kind, bool pressed)
{
    if (ui_input_event_queue == NULL) return false;
    const ui_input_event_t event = {.kind = kind, .pressed = pressed};
    if (xQueueSend(ui_input_event_queue, &event, 0) == pdTRUE) return true;

    /* A full queue must never leave a held key stuck. Replace stale UI input
     * with the latest key state; wheel/touch retriggers already coalesce in
     * their own sequence queues and can safely be dropped here under overload. */
    if (kind == UI_INPUT_BUTTON) {
        xQueueReset(ui_input_event_queue);
        return xQueueSend(ui_input_event_queue, &event, 0) == pdTRUE;
    }
    return false;
}

void lcd_ui_set_button_pressed(bool pressed)
{
    /* Initialization caches the physical key before the worker queue exists. */
    if (ui_input_event_queue == NULL) {
        button_held = pressed;
        return;
    }
    if (!ui_enqueue_input(UI_INPUT_BUTTON, pressed)) {
        ESP_LOGW(TAG, "UI input queue unavailable; button event not rendered");
    }
}

void lcd_ui_show_low_power(void)
{
    ui_state_lock();
    if (ui_state != UI_STATE_LOW_POWER) {
        ESP_LOGI(TAG, "Show low-battery screen");
        link_wait_active = false;

        /* Stop any running clip/scroll/cursor before replacing the whole screen. */
        lcd_ui_stop_animation();
        scroll_state = SCROLL_STATE_IDLE;
        if (scroll_event_queue != NULL) {
            xQueueReset(scroll_event_queue);
        }
        cursor_state = CURSOR_STATE_IDLE;
        if (cursor_event_queue != NULL) {
            xQueueReset(cursor_event_queue);
        }

        ui_mode = UI_MODE_NORMAL;  /* no longer meaningful */
        ui_state = UI_STATE_LOW_POWER;
    }

    /* Redraw even when already in Low Power so a changed percentage is shown. */
    /* power/ contains low-power first and power-on logo second. */
    animation_draw_clip_frame(ANIM_CLIP_POWER, 0);
    ui_state_unlock();
}

void lcd_ui_exit_low_power(void)
{
    ui_state_lock();
    if (ui_state != UI_STATE_LOW_POWER) {
        ui_state_unlock();
        return;
    }

    ui_mode = ui_connected ? UI_MODE_NORMAL : UI_MODE_OFFLINE;
    offline_reset_animation_state();
    thinking_0_played = false;
    approve_0_played = false;
    ui_state = UI_STATE_IDLE;
    ESP_LOGI(TAG, "Leave low-battery screen: connection=%d mode=%d",
             ui_connected ? 1 : 0, (int)ui_mode);
    ui_state_unlock();
}

/* ── Scroll animation ───────────────────────────────────────────────── */

static void ui_handle_wheel_event(void)
{
    if (scroll_event_queue == NULL) return;

    uint8_t dummy = 1;

    /* Wheel has priority over the touchpad. Do not resume a partially played
     * cursor animation after the scroll sequence completes. */
    if (cursor_state != CURSOR_STATE_IDLE) {
        cursor_cancel_sequence(true);
    }

    bool stop_current_clip = false;

    ui_state_lock();
    if (ui_state == UI_STATE_LOW_POWER ||
        ui_mode == UI_MODE_OFFLINE || ui_mode == UI_MODE_SLEEP ||
        ui_mode == UI_MODE_UPDATING) {
        ui_state_unlock();
        return;
    }

    ESP_LOGI(TAG, "Wheel event (mode=%d state=%d)", (int)ui_mode, (int)ui_state);

    if (ui_state == UI_STATE_BUTTON_PRESSED) {
        /* Wheel interrupts the held-button screen. Suppress the held key
         * until it is released, so the scroll animation can play. */
        button_suppress_until_release = true;
        ui_state = UI_STATE_SCROLL_ANIM;
        xQueueSend(scroll_event_queue, &dummy, 0);
        ui_state_unlock();
        return;
    }

    if (ui_state == UI_STATE_SCROLL_ANIM) {
        /* Already scrolling: feed another re-trigger event. */
        xQueueSend(scroll_event_queue, &dummy, 0);
        ui_state_unlock();
        return;
    }

    if (ui_state == UI_STATE_CLIP_PLAYING) {
        /* Claim the UI state before stopping the previous clip. This closes
         * the mode-loop race where Normal could otherwise arm another clip. */
        stop_current_clip = true;
    }

    /* Approve / Done are transient: a wheel event cancels them and
     * returns to the previous mode. Normal / Thinking just pause. */
    if (ui_mode == UI_MODE_APPROVE || ui_mode == UI_MODE_DONE) {
        ui_mode = ui_previous_mode;
    }

    ui_state = UI_STATE_SCROLL_ANIM;
    ui_state_unlock();

    if (stop_current_clip) {
        lcd_ui_stop_animation();
    }

    /* A button or mode transition may have taken ownership while the old
     * clip was stopping. Only queue the start event if Scroll still owns UI. */
    ui_state_lock();
    if (ui_state != UI_STATE_SCROLL_ANIM) {
        ui_state_unlock();
        return;
    }
    xQueueSend(scroll_event_queue, &dummy, 0);
    ui_state_unlock();
}

void lcd_ui_on_wheel_event(void)
{
    (void)ui_enqueue_input(UI_INPUT_WHEEL, false);
}

/* ── Cursor (touchpad) animation ────────────────────────────────────── */

static void ui_handle_touchpad_event(void)
{
    if (ui_state == UI_STATE_LOW_POWER) return;
    /*
     * Sleep/wake transitions are owned by lcd_display. A touch received while
     * the one-shot wake clip is playing must not call the physical-wake entry
     * again, otherwise every touch report resets the clip to frame zero.
     */
    if (ui_mode == UI_MODE_OFFLINE || ui_mode == UI_MODE_SLEEP ||
        ui_mode == UI_MODE_UPDATING) return;
    if (cursor_event_queue == NULL) return;

    ESP_LOGI(TAG, "Touchpad event (mode=%d state=%d)", (int)ui_mode, (int)ui_state);

    uint8_t dummy = 1;

    /* Button and wheel animations have priority; touchpad input during them
     * is used by HID only and does not replace their UI. */
    if (ui_state == UI_STATE_BUTTON_PRESSED || ui_state == UI_STATE_SCROLL_ANIM) {
        return;
    }

    /* Already classifying/playing: this is another trigger for that sequence. */
    if (cursor_state != CURSOR_STATE_IDLE) {
        xQueueSend(cursor_event_queue, &dummy, 0);
        return;
    }

    /* Interrupt a normal mode clip, then classify the trigger count before
     * selecting cursor/0 (single) or cursor/a (multiple). */
    if (ui_state == UI_STATE_CLIP_PLAYING) {
        lcd_ui_stop_animation();
    }

    /* Approve / Done are transient: a touchpad event cancels them and
     * returns to the previous mode. Normal / Thinking / Sleep just pause. */
    if (ui_mode == UI_MODE_APPROVE || ui_mode == UI_MODE_DONE) {
        ui_mode = ui_previous_mode;
    }

    xQueueReset(cursor_event_queue);
    cursor_generation++;
    cursor_state = CURSOR_STATE_CLASSIFY;
    ui_state = UI_STATE_CURSOR_ANIM;
    xQueueSend(cursor_event_queue, &dummy, 0);
}

void lcd_ui_on_touchpad_event(void)
{
    (void)ui_enqueue_input(UI_INPUT_TOUCH, false);
}

void lcd_ui_on_imu_shake(void)
{
    if (!boot_complete || ui_mode != UI_MODE_OFFLINE ||
        ui_state == UI_STATE_LOW_POWER) {
        return;
    }

    /* The Offline render loop owns the motion step. A generation counter
     * makes a new IMU shake restart the wobble without cross-task locking. */
    offline_shake_generation++;
    ESP_LOGI(TAG, "Offline IP shake triggered generation=%" PRIu32,
             offline_shake_generation);
}

void lcd_ui_set_offline_orientation(lcd_offline_orientation_t orientation)
{
    if ((unsigned)orientation > LCD_OFFLINE_ORIENTATION_RIGHT) {
        return;
    }
    offline_orientation = orientation;
}

static void cursor_cancel_sequence(bool return_to_normal)
{
    if (cursor_state == CURSOR_STATE_IDLE && ui_state != UI_STATE_CURSOR_ANIM) {
        return;
    }

    cursor_generation++;
    cursor_generation++;
    cursor_state = CURSOR_STATE_IDLE;
    if (cursor_event_queue != NULL) {
        xQueueReset(cursor_event_queue);
    }
    if (ui_state == UI_STATE_CLIP_PLAYING) {
        lcd_ui_stop_animation();
    }
    if (return_to_normal) {
        ui_mode = ui_connected ? UI_MODE_NORMAL : UI_MODE_OFFLINE;
    }
    if (ui_state == UI_STATE_CURSOR_ANIM || ui_state == UI_STATE_CLIP_PLAYING) {
        ui_state = UI_STATE_IDLE;
    }
}

static void cursor_animation_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        uint8_t dummy;
        if (xQueueReceive(cursor_event_queue, &dummy, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint32_t generation = cursor_generation;
        if (cursor_state != CURSOR_STATE_CLASSIFY) {
            continue;
        }

        /* Wait briefly before drawing anything so the first decision is based
         * on the actual number of touchpad reports, not merely the first one. */
        int trigger_count = 1;
        TickType_t classify_end = xTaskGetTickCount() + pdMS_TO_TICKS(CURSOR_CLASSIFY_MS);
        while (cursor_generation == generation &&
               cursor_state == CURSOR_STATE_CLASSIFY &&
               xTaskGetTickCount() < classify_end) {
            TickType_t remain = classify_end - xTaskGetTickCount();
            if (xQueueReceive(cursor_event_queue, &dummy, remain) == pdTRUE) {
                trigger_count++;
            }
        }

        if (cursor_generation != generation || cursor_state != CURSOR_STATE_CLASSIFY) {
            continue;
        }

        if (trigger_count == 1) {
            cursor_state = CURSOR_STATE_SINGLE;
            ESP_LOGI(TAG, "Cursor: single trigger, play cursor/0");
            lcd_ui_play_clip(ANIM_CLIP_CURSOR_0, CURSOR_FPS, true);

            bool became_multi = false;
            while (cursor_generation == generation &&
                   cursor_state == CURSOR_STATE_SINGLE &&
                   ui_state == UI_STATE_CLIP_PLAYING) {
                if (xQueueReceive(cursor_event_queue, &dummy, pdMS_TO_TICKS(10)) == pdTRUE) {
                    /* A later second report upgrades the sequence immediately
                     * instead of finishing the long single-trigger clip first. */
                    lcd_ui_stop_animation();
                    cursor_state = CURSOR_STATE_MULTI;
                    ui_state = UI_STATE_CURSOR_ANIM;
                    became_multi = true;
                    break;
                }
            }

            if (cursor_generation != generation || cursor_state == CURSOR_STATE_IDLE) {
                continue;
            }
            if (!became_multi) {
                cursor_state = CURSOR_STATE_IDLE;
                if (ui_state == UI_STATE_CURSOR_ANIM || ui_state == UI_STATE_IDLE) {
                    ui_state = UI_STATE_IDLE;
                }
                ESP_LOGI(TAG, "Cursor: single animation done, back to mode loop");
                continue;
            }
        } else {
            cursor_state = CURSOR_STATE_MULTI;
            ESP_LOGI(TAG, "Cursor: %d triggers, play cursor/a", trigger_count);
        }

        /* Multiple-trigger animation loops only while reports continue. */
        while (cursor_generation == generation && cursor_state == CURSOR_STATE_MULTI) {
            xQueueReset(cursor_event_queue);
            lcd_ui_play_clip(ANIM_CLIP_CURSOR_A, CURSOR_FPS, true);

            bool retriggered = false;
            while (cursor_generation == generation &&
                   cursor_state == CURSOR_STATE_MULTI &&
                   ui_state == UI_STATE_CLIP_PLAYING) {
                if (xQueueReceive(cursor_event_queue, &dummy, pdMS_TO_TICKS(10)) == pdTRUE) {
                    retriggered = true;
                }
            }

            if (cursor_generation != generation || cursor_state != CURSOR_STATE_MULTI) {
                break;
            }
            if (!retriggered &&
                xQueueReceive(cursor_event_queue, &dummy,
                              pdMS_TO_TICKS(CURSOR_IDLE_TIMEOUT_MS)) != pdTRUE) {
                break;
            }
        }

        if (cursor_generation == generation && cursor_state == CURSOR_STATE_MULTI) {
            cursor_state = CURSOR_STATE_IDLE;
            if (ui_state == UI_STATE_CURSOR_ANIM || ui_state == UI_STATE_IDLE) {
                ui_state = UI_STATE_IDLE;
            }
            ESP_LOGI(TAG, "Cursor: multiple animation done, back to mode loop");
        }
    }
}

static bool scroll_drain_events(void)
{
    uint8_t dummy;
    bool triggered = false;
    while (xQueueReceive(scroll_event_queue, &dummy, 0) == pdTRUE) {
        triggered = true;
    }
    return triggered;
}

static bool scroll_play_clip_and_collect(int clip_id, bool *triggered)
{
    *triggered = false;
    if (!lcd_ui_play_scroll_clip(clip_id)) {
        return false;
    }

    uint8_t dummy;
    while (!animation_finished) {
        if (ui_state != UI_STATE_SCROLL_ANIM) {
            return false;
        }
        if (xQueueReceive(scroll_event_queue, &dummy,
                          pdMS_TO_TICKS(10)) == pdTRUE) {
            *triggered = true;
        }
    }

    *triggered |= scroll_drain_events();
    return ui_state == UI_STATE_SCROLL_ANIM;
}

static void ui_input_task(void *pvParameters)
{
    (void)pvParameters;
    ui_input_event_t event;
    for (;;) {
        if (xQueueReceive(ui_input_event_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* Input callbacks only enqueue. Sleep transitions, animation stop
         * waits, and any full-screen SPI writes are owned by this worker. */
        if (ui_mode == UI_MODE_SLEEP) {
            if (sleep_phase != SLEEP_PHASE_WAKE) {
                lcd_ui_cancel_sleep_intro();
            }
            if (event.kind == UI_INPUT_TOUCH) continue;
        }

        switch (event.kind) {
        case UI_INPUT_BUTTON:
            ui_handle_button_pressed(event.pressed);
            break;
        case UI_INPUT_WHEEL:
            ui_handle_wheel_event();
            break;
        case UI_INPUT_TOUCH:
            ui_handle_touchpad_event();
            break;
        default:
            break;
        }
    }
}

/*
 * This task only controls the semantic scroll sequence. It never decompresses
 * or writes a frame; every BASE/A/B/END clip is played by animation_task.
 */
static void scroll_control_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        uint8_t dummy;
        if (xQueueReceive(scroll_event_queue, &dummy, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        ui_state_lock();
        if (ui_state != UI_STATE_SCROLL_ANIM) {
            ui_state_unlock();
            continue;
        }
        scroll_state = SCROLL_STATE_BASE;
        ui_state_unlock();

        while (ui_state == UI_STATE_SCROLL_ANIM) {
            bool triggered = false;
            int clip_id;

            switch (scroll_state) {
            case SCROLL_STATE_BASE:
                clip_id = ANIM_CLIP_SCROLL;
                break;
            case SCROLL_STATE_BRANCH:
                clip_id = scroll_branch_is_a ?
                          ANIM_CLIP_SCROLL_A : ANIM_CLIP_SCROLL_B;
                break;
            case SCROLL_STATE_END:
                clip_id = ANIM_CLIP_SCROLL_END;
                break;
            case SCROLL_STATE_IDLE:
            default:
                clip_id = -1;
                break;
            }

            if (clip_id < 0 ||
                !scroll_play_clip_and_collect(clip_id, &triggered)) {
                break;
            }

            switch (scroll_state) {
            case SCROLL_STATE_BASE:
                if (triggered) {
                    scroll_state = SCROLL_STATE_BRANCH;
                    scroll_branch_is_a = (esp_random() & 0x01) != 0;
                    ESP_LOGI(TAG, "Scroll: BASE retriggered, branch=%c",
                             scroll_branch_is_a ? 'a' : 'b');
                } else {
                    scroll_state = SCROLL_STATE_END;
                    ESP_LOGI(TAG, "Scroll: BASE done, going to END");
                }
                break;

            case SCROLL_STATE_BRANCH:
                if (triggered) {
                    scroll_branch_is_a = (esp_random() & 0x01) != 0;
                    ESP_LOGI(TAG, "Scroll: BRANCH retriggered, branch=%c",
                             scroll_branch_is_a ? 'a' : 'b');
                } else {
                    scroll_state = SCROLL_STATE_END;
                    ESP_LOGI(TAG, "Scroll: BRANCH done, going to END");
                }
                break;

            case SCROLL_STATE_END:
                /*
                 * Serialize the final queue check with lcd_ui_on_wheel_event().
                 * An event is either consumed here and restarts BASE, or sees
                 * IDLE in the callback and starts a new Scroll sequence.
                 */
                ui_state_lock();
                triggered |= scroll_drain_events();
                if (ui_state != UI_STATE_SCROLL_ANIM) {
                    ui_state_unlock();
                    break;
                }
                if (triggered) {
                    scroll_state = SCROLL_STATE_BASE;
                    ESP_LOGI(TAG, "Scroll: END retriggered, restart BASE");
                } else {
                    scroll_state = SCROLL_STATE_IDLE;
                    ui_state = UI_STATE_IDLE;
                    ESP_LOGI(TAG, "Scroll: END done, back to mode loop");
                }
                ui_state_unlock();
                break;

            case SCROLL_STATE_IDLE:
            default:
                break;
            }
        }
    }
}

/* ── OTA updating animation ─────────────────────────────────────────── */

static void updating_draw_buffer(const uint8_t *buf)
{
    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);

    lcd_write_frame(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1,
                    buf, anim_ctx.uncompressed_frame_size);

    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
}

static esp_err_t animation_partition_read(size_t offset, void *dst, size_t size)
{
    if (animation_partition == NULL || dst == NULL ||
        offset > animation_partition->size || size > animation_partition->size - offset) {
        return ESP_ERR_INVALID_SIZE;
    }
    return esp_partition_read(animation_partition, offset, dst, size);
}

static bool animation_decompress_frame(int frame_index, uint8_t *dst)
{
    if (!anim_ctx.valid || dst == NULL || frame_index < 0 ||
        frame_index >= anim_ctx.frame_count) {
        return false;
    }

    uint32_t compressed_offset = anim_ctx.compressed_offsets[frame_index];
    uint32_t compressed_size = anim_ctx.compressed_sizes[frame_index];

    esp_err_t ret = animation_partition_read(compressed_offset,
                                             anim_ctx.compressed_buf,
                                             compressed_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read animation frame %d: %s",
                 frame_index, esp_err_to_name(ret));
        return false;
    }

    size_t decoded = anim_rle_decompress(anim_ctx.compressed_buf, compressed_size,
                                          dst, anim_ctx.uncompressed_frame_size);
    if (decoded != anim_ctx.uncompressed_frame_size) {
        ESP_LOGE(TAG, "Animation frame %d decode failed: got %d, expected %u",
                 frame_index, (int)decoded, anim_ctx.uncompressed_frame_size);
        return false;
    }

    return true;
}

static bool animation_draw_frame(int frame_index)
{
    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
    bool ok = animation_draw_frame_locked(frame_index);
    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);

    return ok;
}

static bool animation_draw_frame_locked(int frame_index)
{
    bool ok = false;

    if (animation_decompress_frame(frame_index, anim_ctx.frame_buf)) {
        /* Battery status is a global UI overlay on every screen, including
         * the full-screen Low Power image. */
        battery_blit_status_to_frame_buf(anim_ctx.frame_buf,
                                         anim_ctx.uncompressed_frame_size);
        lcd_write_frame(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1,
                        anim_ctx.frame_buf, anim_ctx.uncompressed_frame_size);
        ok = true;
    }

    return ok;
}

static bool animation_draw_clip_frame(int clip_id, int frame_offset)
{
    if (clip_id < 0 || clip_id >= ANIM_CLIP_COUNT ||
        frame_offset < 0 || frame_offset >= ANIM_CLIPS[clip_id].count) {
        ESP_LOGW(TAG, "Invalid animation frame clip=%d offset=%d", clip_id, frame_offset);
        return false;
    }
    return animation_draw_frame(ANIM_CLIPS[clip_id].start + frame_offset);
}

static int battery_rle_blit_to_frame_buf(const lcd_battery_rle_image_t *image,
                                         uint8_t *dst,
                                         size_t dst_len,
                                         int dst_width,
                                         int dst_height,
                                         int x,
                                         int y)
{
    if (image == NULL || dst == NULL || dst_width <= 0 || dst_height <= 0) {
        return 0;
    }
    if (dst_len < (size_t)dst_width * dst_height * 2) {
        return 0;
    }

    int written = 0;
    int src_index = 0;
    for (uint16_t run = 0; run < image->run_count; run++) {
        uint16_t color = image->runs[run].color;
        uint16_t count = image->runs[run].count;
        for (uint16_t i = 0; i < count; i++) {
            int sx = src_index % image->width;
            int sy = src_index / image->width;
            int dx = x + sx;
            int dy = y + sy;
            if (color != LCD_BATTERY_TRANSPARENT_COLOR &&
                dx >= 0 && dx < dst_width && dy >= 0 && dy < dst_height) {
                size_t off = ((size_t)dy * dst_width + dx) * 2;
                dst[off] = (uint8_t)(color >> 8);
                dst[off + 1] = (uint8_t)(color & 0xFF);
                written++;
            }
            src_index++;
        }
    }

    return written;
}

static int battery_rle_blit_color_to_frame_buf(const lcd_battery_rle_image_t *image,
                                                uint16_t selected_color,
                                                uint8_t *dst,
                                                size_t dst_len,
                                                int dst_width,
                                                int dst_height,
                                                int x,
                                                int y)
{
    if (image == NULL || dst == NULL || dst_width <= 0 || dst_height <= 0) {
        return 0;
    }
    if (dst_len < (size_t)dst_width * dst_height * 2) {
        return 0;
    }

    int written = 0;
    int src_index = 0;
    for (uint16_t run = 0; run < image->run_count; run++) {
        uint16_t color = image->runs[run].color;
        uint16_t count = image->runs[run].count;
        for (uint16_t i = 0; i < count; i++) {
            int sx = src_index % image->width;
            int sy = src_index / image->width;
            int dx = x + sx;
            int dy = y + sy;
            if (color == selected_color &&
                dx >= 0 && dx < dst_width && dy >= 0 && dy < dst_height) {
                size_t off = ((size_t)dy * dst_width + dx) * 2;
                dst[off] = (uint8_t)(color >> 8);
                dst[off + 1] = (uint8_t)(color & 0xFF);
                written++;
            }
            src_index++;
        }
    }

    return written;
}

static void battery_frame_fill_rect(uint8_t *dst,
                                    size_t dst_len,
                                    int x,
                                    int y,
                                    int width,
                                    int height,
                                    uint16_t color)
{
    if (dst == NULL || width <= 0 || height <= 0 ||
        dst_len < (size_t)LCD_WIDTH * LCD_HEIGHT * 2) {
        return;
    }

    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = x + width > LCD_WIDTH ? LCD_WIDTH : x + width;
    int y1 = y + height > LCD_HEIGHT ? LCD_HEIGHT : y + height;
    for (int py = y0; py < y1; py++) {
        for (int px = x0; px < x1; px++) {
            size_t off = ((size_t)py * LCD_WIDTH + px) * 2;
            dst[off] = (uint8_t)(color >> 8);
            dst[off + 1] = (uint8_t)(color & 0xFF);
        }
    }
}

static void battery_blit_dynamic_icon(uint8_t *dst, size_t dst_len, int percent)
{
    bool valid = percent >= 0;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    uint16_t outline = valid ? (percent <= 20 ? 0xF73C : 0xEF7D) : LCD_COLOR_GRAY;
    uint16_t fill = battery_anim_charging ? 0xAFF5 :
                    (percent <= 20 ? 0xFB67 : LCD_COLOR_WHITE);
    int x = BATTERY_STATUS_X;
    int y = BATTERY_STATUS_Y + 2;

    /* Two-pixel body outline and a right-side positive terminal. */
    battery_frame_fill_rect(dst, dst_len, x, y,
                            BATTERY_STATUS_BODY_W, 2, outline);
    battery_frame_fill_rect(dst, dst_len, x, y + BATTERY_STATUS_BODY_H - 2,
                            BATTERY_STATUS_BODY_W, 2, outline);
    battery_frame_fill_rect(dst, dst_len, x, y,
                            2, BATTERY_STATUS_BODY_H, outline);
    battery_frame_fill_rect(dst, dst_len, x + BATTERY_STATUS_BODY_W - 2, y,
                            2, BATTERY_STATUS_BODY_H, outline);
    battery_frame_fill_rect(dst, dst_len,
                            x + BATTERY_STATUS_BODY_W,
                            y + (BATTERY_STATUS_BODY_H - BATTERY_STATUS_TERMINAL_H) / 2,
                            BATTERY_STATUS_TERMINAL_W,
                            BATTERY_STATUS_TERMINAL_H,
                            outline);

    /* Fill width follows the actual 0~100% value, not a 10/20/full bucket. */
    battery_frame_fill_rect(dst, dst_len,
                            x + 3,
                            y + 3,
                            BATTERY_STATUS_INNER_W,
                            BATTERY_STATUS_INNER_H,
                            LCD_COLOR_BLACK);
    int fill_w = valid ?
        (percent * BATTERY_STATUS_INNER_W + 99) / 100 : 0;
    if (fill_w > 0) {
        battery_frame_fill_rect(dst, dst_len,
                                x + 3,
                                y + 3,
                                fill_w,
                                BATTERY_STATUS_INNER_H,
                                fill);
    }
}

static void battery_blit_status_to_frame_buf(uint8_t *dst, size_t dst_len)
{
    int percent = battery_anim_percent;
    if (percent > 100) {
        percent = 100;
    }
    if (percent < 0) {
        percent = 100;
    }

    battery_blit_dynamic_icon(dst, dst_len, battery_anim_percent);
    if (battery_anim_charging) {
        /* Keep the proportional fill, then overlay only the white lightning
         * pixels from the vendor charging icon. Its outline/background must
         * remain transparent so it cannot replace the dynamic battery body. */
        battery_rle_blit_color_to_frame_buf(&lcd_battery_status_charge_icon_only,
                                            LCD_COLOR_WHITE,
                                            dst,
                                            dst_len,
                                            LCD_WIDTH,
                                            LCD_HEIGHT,
                                            BATTERY_STATUS_X,
                                            BATTERY_STATUS_Y);
    }
    int x = BATTERY_STATUS_X + BATTERY_STATUS_ICON_W + BATTERY_STATUS_ICON_GAP;
    int y = BATTERY_STATUS_Y;

    if (battery_anim_percent < 0) {
        return;
    }

    const lcd_battery_rle_image_t *digits = (percent <= 20) ?
        lcd_battery_status_digits_low : lcd_battery_status_digits;
    const lcd_battery_rle_image_t *percent_img = (percent <= 20) ?
        &lcd_battery_status_percent_low : &lcd_battery_status_percent;

    if (percent >= 100) {
        battery_rle_blit_to_frame_buf(&digits[1], dst, dst_len, LCD_WIDTH, LCD_HEIGHT, x, y);
        x += digits[1].width + BATTERY_STATUS_DIGIT_GAP;
        battery_rle_blit_to_frame_buf(&digits[0], dst, dst_len, LCD_WIDTH, LCD_HEIGHT, x, y);
        x += digits[0].width + BATTERY_STATUS_DIGIT_GAP;
        battery_rle_blit_to_frame_buf(&digits[0], dst, dst_len, LCD_WIDTH, LCD_HEIGHT, x, y);
        x += digits[0].width + BATTERY_STATUS_PERCENT_GAP;
    } else {
        if (percent >= 10) {
            battery_rle_blit_to_frame_buf(&digits[percent / 10], dst, dst_len,
                                          LCD_WIDTH, LCD_HEIGHT, x, y);
            x += digits[percent / 10].width + BATTERY_STATUS_DIGIT_GAP;
        }
        battery_rle_blit_to_frame_buf(&digits[percent % 10], dst, dst_len,
                                      LCD_WIDTH, LCD_HEIGHT, x, y);
        x += digits[percent % 10].width + BATTERY_STATUS_PERCENT_GAP;
    }

    battery_rle_blit_to_frame_buf(percent_img,
                                  dst,
                                  dst_len,
                                  LCD_WIDTH,
                                  LCD_HEIGHT,
                                  x,
                                  y + 3);
}

static void updating_draw_composite(int bg_frame, int fg_frame, bool black_is_transparent)
{
    if (!anim_ctx.valid || updating_bg_buf == NULL || anim_ctx.frame_buf == NULL) return;

    if (!animation_decompress_frame(bg_frame, updating_bg_buf)) return;
    if (!animation_decompress_frame(fg_frame, anim_ctx.frame_buf)) return;

    /* Composite foreground over background, skipping transparent key pixels.
     * Updating-end frames have an opaque black background instead of alpha, so
     * we optionally treat pure black as transparent for that clip. */
    for (uint32_t i = 0; i < anim_ctx.uncompressed_frame_size; i += 2) {
        uint16_t fg = (anim_ctx.frame_buf[i] << 8) | anim_ctx.frame_buf[i + 1];
        if (fg == LCD_COLOR_MAGENTA || (black_is_transparent && fg == LCD_COLOR_BLACK)) {
            anim_ctx.frame_buf[i]     = updating_bg_buf[i];
            anim_ctx.frame_buf[i + 1] = updating_bg_buf[i + 1];
        }
    }

    battery_blit_status_to_frame_buf(anim_ctx.frame_buf,
                                     anim_ctx.uncompressed_frame_size);
    updating_draw_buffer(anim_ctx.frame_buf);
}

static bool offline_draw_composite(int bg_frame, int ip_frame, int ip_dx, int ip_dy)
{
    if (!anim_ctx.valid || anim_ctx.frame_buf == NULL) {
        return false;
    }

    if (offline_bg_buf == NULL) {
        offline_bg_buf = (uint8_t *)heap_caps_malloc(anim_ctx.uncompressed_frame_size,
                                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        if (offline_bg_buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate offline background buffer");
            return false;
        }
    }

    bool ok = false;
    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);

    if (animation_decompress_frame(bg_frame, offline_bg_buf) &&
        animation_decompress_frame(ip_frame, anim_ctx.frame_buf)) {
        /* The IP clip is centered on a transparent full-screen canvas by the
         * asset tool. Keep the decoded background stationary, then copy only
         * non-transparent IP pixels to an optionally shifted destination. */
        for (int sy = 0; sy < LCD_HEIGHT; sy++) {
            int dy = sy + ip_dy;
            if (dy < 0 || dy >= LCD_HEIGHT) {
                continue;
            }
            for (int sx = 0; sx < LCD_WIDTH; sx++) {
                int dx = sx + ip_dx;
                if (dx < 0 || dx >= LCD_WIDTH) {
                    continue;
                }

                size_t src_off = ((size_t)sy * LCD_WIDTH + sx) * 2;
                uint16_t ip_pixel = ((uint16_t)anim_ctx.frame_buf[src_off] << 8) |
                                    anim_ctx.frame_buf[src_off + 1];
                if (ip_pixel != LCD_COLOR_MAGENTA) {
                    size_t dst_off = ((size_t)dy * LCD_WIDTH + dx) * 2;
                    offline_bg_buf[dst_off] = anim_ctx.frame_buf[src_off];
                    offline_bg_buf[dst_off + 1] = anim_ctx.frame_buf[src_off + 1];
                }
            }
        }

        battery_blit_status_to_frame_buf(offline_bg_buf,
                                         anim_ctx.uncompressed_frame_size);
        lcd_write_frame(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1,
                        offline_bg_buf, anim_ctx.uncompressed_frame_size);
        ok = true;
    }

    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
    return ok;
}

static void updating_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        if (ui_mode != UI_MODE_UPDATING) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int bg_clip = ANIM_CLIP_UPDATING_BG;
        int bg_frame = ANIM_CLIPS[bg_clip].start + updating_bg_frame_index;
        updating_bg_frame_index = (updating_bg_frame_index + 1) % ANIM_CLIPS[bg_clip].count;

        switch (updating_phase) {
        case UPDATING_PHASE_BEGIN: {
            int fg_clip = ANIM_CLIP_UPDATING_BEGIN;
            int fg_frame = ANIM_CLIPS[fg_clip].start + updating_fg_frame_index;
            updating_draw_composite(bg_frame, fg_frame, false);

            updating_fg_frame_index++;
            if (updating_fg_frame_index >= ANIM_CLIPS[fg_clip].count) {
                updating_fg_frame_index = ANIM_CLIPS[fg_clip].count - 1;
                updating_phase = UPDATING_PHASE_WAIT_END;
                ESP_LOGI(TAG, "OTA: begin finished, hold last frame");
            }
            break;
        }

        case UPDATING_PHASE_WAIT_END: {
            int fg_clip = ANIM_CLIP_UPDATING_BEGIN;
            int fg_frame = ANIM_CLIPS[fg_clip].start + updating_fg_frame_index;
            updating_draw_composite(bg_frame, fg_frame, false);
            break;
        }

        case UPDATING_PHASE_END: {
            int fg_clip = ANIM_CLIP_UPDATING_END;
            int fg_frame = ANIM_CLIPS[fg_clip].start + updating_fg_frame_index;
            /* Keep the background looping while end plays on top.
             * Updating-end frames have an opaque black background, so treat
             * pure black as transparent so the background remains visible. */
            updating_draw_composite(bg_frame, fg_frame, true);

            updating_fg_frame_index++;
            if (updating_fg_frame_index >= ANIM_CLIPS[fg_clip].count) {
                updating_phase = UPDATING_PHASE_DONE;
                updating_fg_frame_index = 0;
                ESP_LOGI(TAG, "OTA: end finished, wait for reboot");
            }
            break;
        }

        case UPDATING_PHASE_DONE:
        default:
            /* End has played; the system is expected to reboot. */
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(UPDATING_FRAME_PERIOD_MS));
    }
}

#if 0 /* ── Full-screen picture cycle from SPIFFS ──────────────────────────── */
/* Legacy SPIFFS picture viewer.  Commented out for v1.0 UI. */

#define SPIFFS_PICTURE_COUNT 16

static int current_picture = 0;
static bool spiffs_mounted = false;

static void __attribute__((unused)) lcd_ui_mount_spiffs(void)
{
    if (spiffs_mounted) return;

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 2,
        .format_if_mount_failed = false,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info("storage", &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted: total=%d KB, used=%d KB", total / 1024, used / 1024);
    }
    spiffs_mounted = true;
}

void lcd_ui_show_spiffs_picture(int index)
{
    if (!spiffs_mounted) {
        ESP_LOGE(TAG, "SPIFFS not mounted");
        return;
    }
    if (index < 0) index = 0;
    if (index >= SPIFFS_PICTURE_COUNT) index = SPIFFS_PICTURE_COUNT - 1;

    char path[32];
    snprintf(path, sizeof(path), "/spiffs/pic%02d.bin", index + 1);

    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return;
    }

    ESP_LOGI(TAG, "Show SPIFFS picture: %s", path);

    /* Set full-screen address window and stream file data directly to LCD.
     * Use a static buffer to avoid large stack usage inside the caller task. */
    lcd_set_address_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    static uint8_t read_buf[1024];
    size_t n;
    while ((n = fread(read_buf, 1, sizeof(read_buf), f)) > 0) {
        lcd_write_pixels(read_buf, n);
    }

    fclose(f);
}

void lcd_ui_next_picture(void)
{
    current_picture = (current_picture + 1) % SPIFFS_PICTURE_COUNT;
    ESP_LOGI(TAG, "Picture cycle: show index %d/%d", current_picture, SPIFFS_PICTURE_COUNT - 1);
    lcd_ui_show_spiffs_picture(current_picture);
}

void lcd_ui_prev_picture(void)
{
    current_picture = (current_picture - 1 + SPIFFS_PICTURE_COUNT) % SPIFFS_PICTURE_COUNT;
    ESP_LOGI(TAG, "Picture cycle: show index %d/%d", current_picture, SPIFFS_PICTURE_COUNT - 1);
    lcd_ui_show_spiffs_picture(current_picture);
}

#endif

/* Preview image from raw Flash removed: its offset (0x210000) conflicts with
 * the animation partition.  Use the animation partition for full-screen frames. */

/* ── PSRAM pre-load ────────────────────────────────────────────────── */

/* ── Compressed animation loading ──────────────────────────────────── */

static void animation_free_context(void)
{
    if (anim_ctx.compressed_sizes) {
        heap_caps_free(anim_ctx.compressed_sizes);
        anim_ctx.compressed_sizes = NULL;
    }
    if (anim_ctx.compressed_offsets) {
        heap_caps_free(anim_ctx.compressed_offsets);
        anim_ctx.compressed_offsets = NULL;
    }
    if (anim_ctx.compressed_buf) {
        heap_caps_free(anim_ctx.compressed_buf);
        anim_ctx.compressed_buf = NULL;
    }
    if (anim_ctx.frame_buf) {
        heap_caps_free(anim_ctx.frame_buf);
        anim_ctx.frame_buf = NULL;
    }
    if (offline_bg_buf) {
        heap_caps_free(offline_bg_buf);
        offline_bg_buf = NULL;
    }
    anim_ctx.compressed_buf_size = 0;
    anim_ctx.valid = false;
    anim_is_compressed = false;
}

static bool load_compressed_animation(void)
{
    animation_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                   ESP_PARTITION_SUBTYPE_ANY,
                                                   ANIMATION_PARTITION_LABEL);
    if (animation_partition == NULL) {
        ESP_LOGE(TAG, "Animation partition '%s' not found", ANIMATION_PARTITION_LABEL);
        return false;
    }

    ESP_LOGI(TAG, "Animation partition: offset=0x%" PRIx32 " size=0x%" PRIx32,
             animation_partition->address, animation_partition->size);

    uint8_t header[ANIMATION_HEADER_SIZE];
    esp_err_t ret = animation_partition_read(0, header, sizeof(header));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read animation header: %s", esp_err_to_name(ret));
        return false;
    }

    if (memcmp(header, ANIMATION_MAGIC, 4) != 0) {
        ESP_LOGI(TAG, "Animation binary is not compressed (no ANMR magic)");
        return false;
    }

    anim_ctx.frame_count = (header[6] << 8) | header[7];
    anim_ctx.frame_width = (header[8] << 8) | header[9];
    anim_ctx.frame_height = (header[10] << 8) | header[11];
    anim_ctx.uncompressed_frame_size = ((uint32_t)header[12] << 24) |
                                       ((uint32_t)header[13] << 16) |
                                       ((uint32_t)header[14] << 8) |
                                       header[15];

    const uint32_t required_frames = ANIM_CLIPS[ANIM_CLIP_COUNT - 1].start +
                                     ANIM_CLIPS[ANIM_CLIP_COUNT - 1].count;
    if (anim_ctx.frame_width != LCD_WIDTH || anim_ctx.frame_height != LCD_HEIGHT ||
        anim_ctx.uncompressed_frame_size != LCD_WIDTH * LCD_HEIGHT * 2 ||
        anim_ctx.frame_count < required_frames) {
        ESP_LOGE(TAG,
                 "Animation metadata mismatch: frames=%u/%" PRIu32 " size=%ux%u frame_bytes=%" PRIu32,
                 anim_ctx.frame_count, required_frames,
                 anim_ctx.frame_width, anim_ctx.frame_height,
                 anim_ctx.uncompressed_frame_size);
        return false;
    }

    ESP_LOGI(TAG, "Compressed animation: %d frames, %dx%d, frame size %u",
             anim_ctx.frame_count, anim_ctx.frame_width, anim_ctx.frame_height,
             anim_ctx.uncompressed_frame_size);

    size_t table_size = anim_ctx.frame_count * sizeof(uint32_t);
    anim_ctx.compressed_sizes = (uint32_t *)heap_caps_malloc(table_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    anim_ctx.compressed_offsets = (uint32_t *)heap_caps_malloc(table_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (anim_ctx.compressed_sizes == NULL || anim_ctx.compressed_offsets == NULL) {
        ESP_LOGE(TAG, "Failed to allocate animation tables");
        animation_free_context();
        return false;
    }

    uint32_t table_offset = ANIMATION_HEADER_SIZE;
    ret = animation_partition_read(table_offset,
                                   (uint8_t *)anim_ctx.compressed_sizes,
                                   table_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read compressed sizes: %s", esp_err_to_name(ret));
        animation_free_context();
        return false;
    }

    ret = animation_partition_read(table_offset + table_size,
                                   (uint8_t *)anim_ctx.compressed_offsets,
                                   table_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read compressed offsets: %s", esp_err_to_name(ret));
        animation_free_context();
        return false;
    }

    uint32_t max_compressed_size = 0;
    for (uint16_t i = 0; i < anim_ctx.frame_count; i++) {
        anim_ctx.compressed_sizes[i] = __builtin_bswap32(anim_ctx.compressed_sizes[i]);
        anim_ctx.compressed_offsets[i] = __builtin_bswap32(anim_ctx.compressed_offsets[i]);
        if (anim_ctx.compressed_offsets[i] > animation_partition->size ||
            anim_ctx.compressed_sizes[i] >
                animation_partition->size - anim_ctx.compressed_offsets[i]) {
            ESP_LOGE(TAG, "Animation frame %u exceeds partition", i);
            animation_free_context();
            return false;
        }
        if (anim_ctx.compressed_sizes[i] > max_compressed_size) {
            max_compressed_size = anim_ctx.compressed_sizes[i];
        }
    }

    anim_ctx.frame_buf = (uint8_t *)heap_caps_malloc(anim_ctx.uncompressed_frame_size,
                                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (anim_ctx.frame_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer (%u bytes)", anim_ctx.uncompressed_frame_size);
        animation_free_context();
        return false;
    }

    anim_ctx.compressed_buf = (uint8_t *)heap_caps_malloc(max_compressed_size,
                                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (anim_ctx.compressed_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate compressed buffer (%u bytes)", max_compressed_size);
        animation_free_context();
        return false;
    }
    anim_ctx.compressed_buf_size = max_compressed_size;

    ESP_LOGI(TAG, "Animation context ready, max compressed frame %u bytes", max_compressed_size);
    anim_ctx.valid = true;
    anim_is_compressed = true;
    return true;
}

/* ── Animation playback from decompressed frame buffer ─────────────── */

static void animation_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Animation task started, %d frames @ %d fps",
             anim_ctx.frame_count, animation_fps);

    for (;;) {
        if (!animation_enabled) {
            animation_running = false;
            if (animation_event_group != NULL) {
                xEventGroupSetBits(animation_event_group, ANIMATION_STOPPED_BIT);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        animation_running = true;

        for (int frame = 0; frame < anim_clip_count; frame++) {
            if (!animation_enabled) {
                break;
            }

            if (!anim_ctx.valid) {
                ESP_LOGE(TAG, "Animation context not valid");
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
            }

            int frame_offset = anim_clip_frame_order != NULL ?
                               anim_clip_frame_order[frame] : frame;
            int actual_frame = anim_clip_start + frame_offset;
            TickType_t frame_start = xTaskGetTickCount();

            if (!animation_draw_frame(actual_frame)) {
                ESP_LOGE(TAG, "Failed to draw animation frame %d", actual_frame);
                continue;
            }

            /* Accurate frame-rate timing */
            TickType_t elapsed = xTaskGetTickCount() - frame_start;
            TickType_t target = pdMS_TO_TICKS(animation_delay_ms);
            if (elapsed < target) {
                vTaskDelay(target - elapsed);
            }
        }

        /* One-shot playback: after finishing the whole sequence, signal
         * completion and stop. Do not mark finished if we were stopped
         * externally (animation_enabled already cleared). */
        if (animation_enabled && animation_play_once) {
            animation_enabled = false;
            animation_finished = true;
            animation_play_once = false;

            /* Hand control back to the mode loop so it can pick the next clip
             * or return to the previous mode (for transient approve/done). */
            if (ui_state == UI_STATE_CLIP_PLAYING) {
                ui_state = UI_STATE_IDLE;
            }
        }
    }
}

void lcd_ui_play_animation(bool enable)
{
    if (animation_enabled == enable) return;

    if (enable) {
        if (animation_event_group == NULL) {
            ESP_LOGE(TAG, "Animation event group unavailable; refusing asynchronous playback");
            return;
        }
        xEventGroupClearBits(animation_event_group, ANIMATION_STOPPED_BIT);
        if (animation_task_handle == NULL) {
            if (xTaskCreate(animation_task, "lcd_anim", 4096, NULL, 5,
                            &animation_task_handle) != pdPASS) {
                animation_task_handle = NULL;
                xEventGroupSetBits(animation_event_group, ANIMATION_STOPPED_BIT);
                ESP_LOGE(TAG, "Failed to create animation task");
                return;
            }
        }
        animation_enabled = true;
        ESP_LOGI(TAG, "Animation playback enabled");
    } else {
        animation_enabled = false;
        ESP_LOGI(TAG, "Animation playback disabled, will stop after current loop");
    }
}

void lcd_ui_stop_animation(void)
{
    animation_enabled = false;
    if (!animation_running) {
        return;
    }
    if (animation_event_group == NULL) {
        ESP_LOGE(TAG, "Animation stop event unavailable while animation is running");
        return;
    }
    (void)xEventGroupWaitBits(animation_event_group, ANIMATION_STOPPED_BIT,
                              pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Animation stopped");
}

void lcd_ui_play_jump_animation(void)
{
    lcd_ui_play_clip(BOOT_WAKE_CLIP, BOOT_WAKE_FPS, true);
    ESP_LOGI(TAG, "black-in boot/wake animation started (one-shot, %d fps)",
             BOOT_WAKE_FPS);
}

static void lcd_ui_play_clip(int clip_id, int fps, bool once)
{
    lcd_ui_play_clip_ordered(clip_id, NULL, 0, fps, once);
}

/* Caller must hold ui_state_mutex. */
static bool lcd_ui_arm_clip_locked(int clip_id,
                                   const uint8_t *frame_order,
                                   int frame_count,
                                   int fps,
                                   bool once,
                                   ui_state_t playback_state)
{
    if (clip_id < 0 || clip_id >= ANIM_CLIP_COUNT) {
        ESP_LOGW(TAG, "Invalid clip id %d", clip_id);
        return false;
    }
    if (!anim_ctx.valid) {
        ESP_LOGW(TAG, "Clip %d not available, animation context invalid", clip_id);
        return false;
    }

    anim_clip_id = clip_id;
    anim_clip_start = ANIM_CLIPS[clip_id].start;
    anim_clip_count = frame_order != NULL ? frame_count : ANIM_CLIPS[clip_id].count;
    if (anim_clip_count <= 0) {
        ESP_LOGW(TAG, "Clip %d has no frames", clip_id);
        return false;
    }
    if (frame_order != NULL) {
        for (int i = 0; i < anim_clip_count; i++) {
            if (frame_order[i] >= ANIM_CLIPS[clip_id].count) {
                ESP_LOGW(TAG, "Invalid ordered frame clip=%d index=%d offset=%u",
                         clip_id, i, frame_order[i]);
                return false;
            }
        }
    }
    anim_clip_frame_order = frame_order;

    animation_finished = false;
    animation_play_once = once;
    lcd_ui_set_animation_fps(fps);
    ui_state = playback_state;
    lcd_ui_play_animation(true);

    if (frame_order != NULL) {
        ESP_LOGI(TAG, "Play clip %d: %d ordered frames @ %d fps",
                 clip_id, anim_clip_count, fps);
    } else {
        ESP_LOGI(TAG, "Play clip %d: frames %d..%d @ %d fps", clip_id,
                 anim_clip_start, anim_clip_start + anim_clip_count - 1, fps);
    }
    return true;
}

static void lcd_ui_play_clip_ordered(int clip_id,
                                     const uint8_t *frame_order,
                                     int frame_count,
                                     int fps,
                                     bool once)
{
    /*
     * Serialize the final start decision with overlays and Scroll ownership.
     * Normal may have passed its IDLE test just before a wheel event; it must
     * not overwrite a newly claimed UI_STATE_SCROLL_ANIM.
     */
    ui_state_lock();
    if (ui_button_overlay_requested() || ui_state == UI_STATE_SCROLL_ANIM) {
        ui_state_unlock();
        return;
    }

    lcd_ui_arm_clip_locked(clip_id, frame_order, frame_count, fps, once,
                           UI_STATE_CLIP_PLAYING);
    ui_state_unlock();
}

static bool lcd_ui_play_scroll_clip(int clip_id)
{
    bool started = false;

    ui_state_lock();
    if (ui_state == UI_STATE_SCROLL_ANIM &&
        !ui_button_overlay_requested()) {
        started = lcd_ui_arm_clip_locked(clip_id, NULL, 0, SCROLL_FPS, true,
                                         UI_STATE_SCROLL_ANIM);
    }
    ui_state_unlock();
    return started;
}

static int pick_random_clip(const int *weights)
{
    int total = 0;
    for (int i = 0; i < ANIM_CLIP_COUNT; i++) {
        total += weights[i];
    }
    if (total <= 0) {
        return -1;
    }

    uint32_t r = esp_random() % total;
    int acc = 0;
    for (int i = 0; i < ANIM_CLIP_COUNT; i++) {
        acc += weights[i];
        if ((int)r < acc) {
            return i;
        }
    }
    return -1;
}

static void ui_abort_current(void)
{
    if (ui_state == UI_STATE_CLIP_PLAYING) {
        lcd_ui_stop_animation();
    }
    if (ui_state == UI_STATE_SCROLL_ANIM) {
        /* The scroll controller never draws; its current clip is owned by
         * animation_task and must be stopped before another mode is armed. */
        lcd_ui_stop_animation();
        scroll_state = SCROLL_STATE_IDLE;
        if (scroll_event_queue != NULL) {
            xQueueReset(scroll_event_queue);
        }
    }
    if (cursor_state != CURSOR_STATE_IDLE || ui_state == UI_STATE_CURSOR_ANIM) {
        cursor_generation++;
        cursor_state = CURSOR_STATE_IDLE;
        if (cursor_event_queue != NULL) {
            xQueueReset(cursor_event_queue);
        }
    }
    if (ui_state == UI_STATE_BUTTON_PRESSED) {
        button_suppress_until_release = false;
    }
}

static void offline_reset_animation_state(void)
{
    offline_bg_frame_index = 0;
    offline_ip_frame_index = 0;
    /* Enter Offline at the center, then visibly transition toward the latest
     * retained physical orientation instead of jumping to an edge. */
    offline_position_x = 0;
    offline_position_y = 0;
    offline_shake_seen_generation = offline_shake_generation;
    offline_shake_step = OFFLINE_SHAKE_STEP_COUNT;
}

static int offline_approach_position(int current, int target)
{
    int delta = target - current;
    if (delta > OFFLINE_POSITION_STEP_PX) {
        return current + OFFLINE_POSITION_STEP_PX;
    }
    if (delta < -OFFLINE_POSITION_STEP_PX) {
        return current - OFFLINE_POSITION_STEP_PX;
    }
    return target;
}

static void offline_orientation_target(int *target_x, int *target_y)
{
    *target_x = 0;
    *target_y = 0;

    switch (offline_orientation) {
    case LCD_OFFLINE_ORIENTATION_UPRIGHT:
        *target_y = OFFLINE_POSITION_BOTTOM_Y;
        break;
    case LCD_OFFLINE_ORIENTATION_UPSIDE_DOWN:
        *target_y = OFFLINE_POSITION_TOP_Y;
        break;
    case LCD_OFFLINE_ORIENTATION_LEFT:
        *target_x = OFFLINE_POSITION_LEFT_X;
        break;
    case LCD_OFFLINE_ORIENTATION_RIGHT:
        *target_x = OFFLINE_POSITION_RIGHT_X;
        break;
    case LCD_OFFLINE_ORIENTATION_FLAT:
    default:
        break;
    }
}

static void link_wait_begin(TickType_t start_tick)
{
    link_wait_deadline = start_tick + pdMS_TO_TICKS(LINK_DETECTION_WAIT_MS);
    link_wait_active = true;
}

static bool link_wait_expired(void)
{
    return link_wait_active &&
           (int32_t)(xTaskGetTickCount() - link_wait_deadline) >= 0;
}

static void link_wait_finish(void)
{
    if (!link_wait_active) {
        return;
    }

    link_wait_active = false;
    ui_abort_current();
    ui_mode = ui_connected ? UI_MODE_NORMAL : UI_MODE_OFFLINE;
    offline_reset_animation_state();
    thinking_0_played = false;
    approve_0_played = false;
    ui_state = UI_STATE_IDLE;
    ESP_LOGI(TAG, "Link detection finished, connection=%d mode=%d",
             ui_connected ? 1 : 0, (int)ui_mode);
}

/* ── Software-controlled mode APIs ─────────────────────────────────── */

void lcd_ui_set_connected(bool connected)
{
    bool changed = !ui_connection_known || ui_connected != connected;
    if (changed) {
        ui_connection_known = true;
        ui_connected = connected;
        ESP_LOGI(TAG, "Connection UI: %s", connected ? "online" : "offline");
    }

    /* The boot animation owns the screen until lcd_ui_init() completes. */
    if (!boot_complete || ui_state == UI_STATE_LOW_POWER) {
        return;
    }

    /* OTA remains the highest-priority animated mode. It normally finishes
     * with a reboot, so only remember the latest link state while it runs. */
    if (ui_mode == UI_MODE_UPDATING) {
        return;
    }

    /* Boot and physical wake deliberately show Normal while the peer gets a
     * fixed two-second detection window. Record link changes above, but do
     * not expose transient Offline frames before the deadline. */
    if (link_wait_active) {
        return;
    }

    /* Preserve the sleep intro/wake sequence, but make it return to the mode
     * matching the latest connection state. */
    if (ui_mode == UI_MODE_SLEEP) {
        ui_previous_mode = connected ? UI_MODE_NORMAL : UI_MODE_OFFLINE;
        return;
    }

    /* A repeated value normally needs no work. The exception is the first
     * post-init apply: it closes the tiny race where the link changes while
     * lcd_ui_init() is handing control to the mode loop. */
    if (!changed && ((connected && ui_mode != UI_MODE_OFFLINE) ||
                     (!connected && ui_mode == UI_MODE_OFFLINE))) {
        return;
    }

    ui_abort_current();
    ui_mode = connected ? UI_MODE_NORMAL : UI_MODE_OFFLINE;
    offline_reset_animation_state();
    thinking_0_played = false;
    approve_0_played = false;
    ui_state = UI_STATE_IDLE;
}

void lcd_ui_start_thinking(void)
{
    if (ui_state == UI_STATE_LOW_POWER) return;
    if (!ui_connected) return;
    if (ui_mode == UI_MODE_THINKING) return;

    ESP_LOGI(TAG, "Start thinking mode");
    ui_abort_current();
    ui_mode = UI_MODE_THINKING;
    thinking_0_played = false;
    ui_state = UI_STATE_IDLE;
}

void lcd_ui_stop_thinking(void)
{
    if (ui_mode != UI_MODE_THINKING) return;

    ESP_LOGI(TAG, "Stop thinking mode, return to normal");
    ui_abort_current();
    ui_mode = UI_MODE_NORMAL;
    ui_state = UI_STATE_IDLE;
}

void lcd_ui_start_approve(void)
{
    if (ui_state == UI_STATE_LOW_POWER) return;
    if (!ui_connected) return;
    if (ui_mode == UI_MODE_APPROVE) return;

    ESP_LOGI(TAG, "Start approve mode (previous=%d)", (int)ui_mode);
    ui_abort_current();
    ui_previous_mode = ui_mode;
    ui_mode = UI_MODE_APPROVE;
    approve_0_played = false;
    ui_state = UI_STATE_IDLE;
}

void lcd_ui_stop_approve(void)
{
    if (ui_mode != UI_MODE_APPROVE) return;

    ESP_LOGI(TAG, "Stop approve mode, return to %d", (int)ui_previous_mode);
    ui_abort_current();
    ui_mode = ui_previous_mode;
    ui_state = UI_STATE_IDLE;
}

void lcd_ui_start_done(void)
{
    if (ui_state == UI_STATE_LOW_POWER) return;
    if (!ui_connected) return;
    if (ui_mode == UI_MODE_DONE) return;

    ESP_LOGI(TAG, "Start done animation (previous=%d)", (int)ui_mode);
    ui_abort_current();
    ui_previous_mode = ui_mode;
    ui_mode = UI_MODE_DONE;
    ui_state = UI_STATE_IDLE;
}

void lcd_ui_stop_done(void)
{
    if (ui_mode != UI_MODE_DONE) return;

    ESP_LOGI(TAG, "Stop done animation, return to %d", (int)ui_previous_mode);
    ui_abort_current();
    ui_mode = ui_previous_mode;
    ui_state = UI_STATE_IDLE;
}

/* ── Sleep animation mode ───────────────────────────────────────────── */

static void ui_start_test_once(ui_test_once_t test)
{
    if (!boot_complete || ui_state == UI_STATE_LOW_POWER || !ui_connected ||
        link_wait_active || ui_mode == UI_MODE_SLEEP ||
        ui_mode == UI_MODE_UPDATING) {
        return;
    }

    ui_abort_current();
    int weighted_clip = -1;
    switch (test) {
    case UI_TEST_THINKING:
        weighted_clip = pick_random_clip(thinking_clip_weights);
        ui_test_clips[0] = ANIM_CLIP_THINKING0;
        ui_test_clips[1] = (uint8_t)weighted_clip;
        ui_test_clip_count = weighted_clip >= 0 ? 2 : 1;
        break;
    case UI_TEST_APPROVE:
        weighted_clip = pick_random_clip(approve_clip_weights);
        ui_test_clips[0] = ANIM_CLIP_APPROVE0;
        ui_test_clips[1] = (uint8_t)weighted_clip;
        ui_test_clip_count = weighted_clip >= 0 ? 2 : 1;
        break;
    case UI_TEST_DONE:
        ui_test_clips[0] = ANIM_CLIP_DONE;
        ui_test_clip_count = 1;
        break;
    case UI_TEST_NONE:
    default:
        return;
    }

    ui_test_once = test;
    ui_test_clip_index = 0;
    ui_mode = UI_MODE_TEST_ONCE;
    ui_state = UI_STATE_IDLE;
    ESP_LOGI(TAG,
             "Start weighted one-shot UI type=%d branch=%d",
             (int)test,
             weighted_clip);
}

void lcd_ui_play_thinking_once(void)
{
    ui_start_test_once(UI_TEST_THINKING);
}

void lcd_ui_play_approve_once(void)
{
    ui_start_test_once(UI_TEST_APPROVE);
}

void lcd_ui_play_done_once(void)
{
    ui_start_test_once(UI_TEST_DONE);
}

void lcd_ui_start_sleep(void)
{
    if (ui_state == UI_STATE_LOW_POWER) return;
    if (ui_mode == UI_MODE_SLEEP) return;

    ESP_LOGI(TAG, "Start sleep mode (previous=%d)", (int)ui_mode);
    ui_abort_current();
    link_wait_active = false;
    ui_previous_mode = ui_mode;
    ui_mode = UI_MODE_SLEEP;
    sleep_phase = SLEEP_PHASE_INTRO;
    sleep_wake_animation = SLEEP_WAKE_BLACK_IN;
    sleep_intro_started = false;
    sleep_wake_clip_started = false;
    ui_state = UI_STATE_IDLE;
}

bool lcd_ui_wait_sleep_intro_complete(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while (ui_mode == UI_MODE_SLEEP && sleep_phase == SLEEP_PHASE_INTRO) {
        if ((xTaskGetTickCount() - start) >= timeout_ticks) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ui_mode == UI_MODE_SLEEP &&
           sleep_phase == SLEEP_PHASE_READY;
}

void lcd_ui_stop_sleep(void)
{
    if (ui_mode != UI_MODE_SLEEP) return;

    ESP_LOGI(TAG, "Stop sleep mode, return to %d", (int)ui_previous_mode);
    ui_abort_current();
    ui_mode = ui_previous_mode;
    sleep_phase = SLEEP_PHASE_INTRO;
    sleep_wake_animation = SLEEP_WAKE_BLACK_IN;
    sleep_intro_started = false;
    sleep_wake_clip_started = false;
    ui_state = UI_STATE_IDLE;
}

void lcd_ui_cancel_sleep_intro(void)
{
    if (ui_mode != UI_MODE_SLEEP) return;
    /* Activity during the 10~15 second pre-sleep screen wakes with sleep/b. */
    if (sleep_phase == SLEEP_PHASE_WAKE) return;

    ui_abort_current();
    sleep_wake_animation = SLEEP_WAKE_SLEEP_B;
    sleep_phase = SLEEP_PHASE_WAKE;
    sleep_wake_clip_started = false;
    animation_finished = false;
    link_wait_begin(xTaskGetTickCount());
    ui_state = UI_STATE_IDLE;
    ESP_LOGI(TAG, "Pre-sleep screen interrupted: play sleep/b once");
}

void lcd_ui_start_wake_animation(void)
{
    if (!lcd_ui_begin_panel_resume()) {
        return;
    }
    (void)lcd_ui_draw_panel_resume_frame();
    lcd_ui_end_panel_resume();
}

bool lcd_ui_begin_panel_resume(void)
{
    if (ui_state == UI_STATE_LOW_POWER || ui_mode != UI_MODE_SLEEP ||
        panel_resume_active) {
        return false;
    }

    /*
     * Establish WAKE before making the panel writable. Input callbacks can no
     * longer reinterpret this physical wake as a pre-sleep interruption.
     * panel_resume_active prevents the mode task from starting the asynchronous
     * clip while ST7789 initialization and the synchronous first frame run.
     */
    ui_abort_current();
    panel_resume_active = true;
    panel_resume_owner = xTaskGetCurrentTaskHandle();
    sleep_wake_animation = SLEEP_WAKE_BLACK_IN;
    sleep_phase = SLEEP_PHASE_WAKE;
    sleep_wake_clip_started = false;
    animation_finished = false;
    link_wait_begin(xTaskGetTickCount());
    ui_state = UI_STATE_IDLE;

    if (ui_draw_mutex) {
        xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
    }
    ESP_LOGI(TAG, "Physical wake render barrier acquired");
    return true;
}

bool lcd_ui_draw_panel_resume_frame(void)
{
    if (!panel_resume_active ||
        panel_resume_owner != xTaskGetCurrentTaskHandle()) {
        return false;
    }
    return animation_draw_frame_locked(ANIM_CLIPS[BOOT_WAKE_CLIP].start);
}

void lcd_ui_end_panel_resume(void)
{
    if (!panel_resume_active ||
        panel_resume_owner != xTaskGetCurrentTaskHandle()) {
        return;
    }

    /* Leave the asynchronous clip unstarted so the mode task plays the full
     * physical-wake animation once after the panel is visible. */
    sleep_wake_clip_started = false;
    animation_finished = false;
    panel_resume_active = false;
    panel_resume_owner = NULL;
    if (ui_draw_mutex) {
        xSemaphoreGive(ui_draw_mutex);
    }
    ESP_LOGI(TAG, "Physical wake render barrier released");
}

void lcd_ui_sleep_interrupt(void)
{
    /* Compatibility entry point: physical resume now explicitly starts the
     * wake animation, while pre-sleep activity uses cancel_sleep_intro(). */
    lcd_ui_start_wake_animation();
}

bool lcd_ui_is_sleeping(void)
{
    return ui_mode == UI_MODE_SLEEP;
}

/* ── OTA updating animation mode ────────────────────────────────────── */

void lcd_ui_start_ota(void)
{
    if (ui_state == UI_STATE_LOW_POWER) return;
    if (ui_mode == UI_MODE_UPDATING) return;

    ESP_LOGI(TAG, "Start OTA updating mode");
    ui_abort_current();
    link_wait_active = false;
    ui_mode = UI_MODE_UPDATING;
    updating_phase = UPDATING_PHASE_BEGIN;
    updating_bg_frame_index = 0;
    updating_fg_frame_index = 0;
    ui_state = UI_STATE_IDLE;

    if (updating_bg_buf == NULL && anim_ctx.valid) {
        updating_bg_buf = (uint8_t *)heap_caps_malloc(anim_ctx.uncompressed_frame_size,
                                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        if (updating_bg_buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate OTA background buffer");
        }
    }
}

void lcd_ui_ota_finished(void)
{
    if (ui_mode != UI_MODE_UPDATING) return;
    if (updating_phase == UPDATING_PHASE_END || updating_phase == UPDATING_PHASE_DONE) return;

    ESP_LOGI(TAG, "OTA finished, play end animation");
    updating_phase = UPDATING_PHASE_END;
    updating_fg_frame_index = 0;
}

static void mode_loop_task(void *pvParameters)
{
    (void)pvParameters;

    /* Wait for boot animation to finish before starting the mode loop. */
    while (!boot_complete) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    for (;;) {
        if (ui_state == UI_STATE_LOW_POWER) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /*
         * button_held is the persistent source of truth. Reassert its overlay
         * after boot/wake/mode transitions instead of relying on another key
         * edge, which does not occur during a long press.
         */
        bool button_owns_screen = false;
        ui_state_lock();
        if (ui_button_overlay_requested()) {
            if (ui_state != UI_STATE_BUTTON_PRESSED) {
                ui_apply_button_overlay_locked();
            }
            button_owns_screen = true;
        }
        ui_state_unlock();
        if (button_owns_screen) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Do not interrupt the one-shot wake clip. Once that clip hands the
         * screen to Normal, enforce the original boot/wake detection deadline
         * even if a long Normal clip is in progress. */
        if (link_wait_expired() &&
            !(ui_mode == UI_MODE_SLEEP && sleep_phase == SLEEP_PHASE_WAKE)) {
            link_wait_finish();
            continue;
        }

        bool just_played_done = false;

        if (ui_state == UI_STATE_IDLE) {
            switch (ui_mode) {
            case UI_MODE_NORMAL:
                /* Normal always uses the original main/a-f animation loop.
                 * The battery indicator is composited onto every frame. */
                lcd_ui_show_main0();
                if (NORMAL_MAIN0_HOLD_MS > 0) {
                    vTaskDelay(pdMS_TO_TICKS(NORMAL_MAIN0_HOLD_MS));
                }
                {
                    int clip = pick_random_clip(normal_clip_weights);
                    if (clip >= 0) {
                        ESP_LOGI(TAG, "Normal loop: pick clip %d", clip);
                        lcd_ui_play_clip(clip, 7, true);
                    }
                }
                break;

            case UI_MODE_OFFLINE: {
                TickType_t frame_start = xTaskGetTickCount();
                int bg_frame = ANIM_CLIPS[ANIM_CLIP_OFFLINE_BG].start +
                               offline_bg_frame_index;
                int ip_frame = ANIM_CLIPS[ANIM_CLIP_OFFLINE_IP].start +
                               offline_ip_frame_index;

                uint32_t shake_generation = offline_shake_generation;
                if (offline_shake_seen_generation != shake_generation) {
                    offline_shake_seen_generation = shake_generation;
                    offline_shake_step = 0;
                }

                int shake_step = offline_shake_step;
                int target_x;
                int target_y;
                offline_orientation_target(&target_x, &target_y);
                int next_position_x =
                    offline_approach_position(offline_position_x, target_x);
                int next_position_y =
                    offline_approach_position(offline_position_y, target_y);
                int ip_dx = next_position_x;
                int ip_dy = next_position_y;
                if (shake_step >= 0 && shake_step < OFFLINE_SHAKE_STEP_COUNT) {
                    ip_dx += offline_shake_x[shake_step];
                    ip_dy += offline_shake_y[shake_step];
                }

                if (offline_draw_composite(bg_frame, ip_frame, ip_dx, ip_dy)) {
                    offline_position_x = next_position_x;
                    offline_position_y = next_position_y;
                    offline_bg_frame_index =
                        (offline_bg_frame_index + 1) % ANIM_CLIPS[ANIM_CLIP_OFFLINE_BG].count;
                    offline_ip_frame_index =
                        (offline_ip_frame_index + 1) % ANIM_CLIPS[ANIM_CLIP_OFFLINE_IP].count;
                    /* If another IMU event arrived while this frame was being
                     * drawn, leave the step untouched so the next frame can
                     * restart the wobble at step zero. */
                    if (offline_shake_generation == shake_generation &&
                        shake_step >= 0 && shake_step < OFFLINE_SHAKE_STEP_COUNT) {
                        offline_shake_step = shake_step + 1;
                    }
                }

                TickType_t elapsed = xTaskGetTickCount() - frame_start;
                TickType_t period = pdMS_TO_TICKS(OFFLINE_FRAME_PERIOD_MS);
                if (elapsed < period) {
                    vTaskDelay(period - elapsed);
                }
                break;
            }

            case UI_MODE_THINKING:
                if (!thinking_0_played) {
                    thinking_0_played = true;
                    ESP_LOGI(TAG, "Thinking loop: play intro 0");
                    lcd_ui_play_clip(ANIM_CLIP_THINKING0, 7, true);
                } else {
                    int clip = pick_random_clip(thinking_clip_weights);
                    if (clip >= 0) {
                        ESP_LOGI(TAG, "Thinking loop: pick clip %d", clip);
                        lcd_ui_play_clip(clip, 7, true);
                    }
                }
                break;

            case UI_MODE_APPROVE:
                if (!approve_0_played) {
                    approve_0_played = true;
                    ESP_LOGI(TAG, "Approve loop: play intro 0");
                    lcd_ui_play_clip(ANIM_CLIP_APPROVE0, 7, true);
                } else {
                    int clip = pick_random_clip(approve_clip_weights);
                    if (clip >= 0) {
                        ESP_LOGI(TAG, "Approve loop: pick clip %d", clip);
                        lcd_ui_play_clip(clip, 7, true);
                    }
                }
                break;

            case UI_MODE_DONE:
                just_played_done = true;
                ESP_LOGI(TAG, "Done: play once");
                lcd_ui_play_clip(ANIM_CLIP_DONE, 7, true);
                break;

            case UI_MODE_TEST_ONCE: {
                if (ui_test_once != UI_TEST_NONE &&
                    ui_test_clip_index < ui_test_clip_count) {
                    int clip = ui_test_clips[ui_test_clip_index++];
                    ESP_LOGI(TAG, "Weighted one-shot UI: clip=%d step=%d/%d",
                             clip, ui_test_clip_index, ui_test_clip_count);
                    lcd_ui_play_clip(clip, 7, true);
                } else {
                    ESP_LOGI(TAG, "Weighted one-shot UI finished");
                    ui_test_once = UI_TEST_NONE;
                    ui_test_clip_index = 0;
                    ui_test_clip_count = 0;
                    ui_mode = ui_connected ? UI_MODE_NORMAL : UI_MODE_OFFLINE;
                    offline_reset_animation_state();
                    ui_state = UI_STATE_IDLE;
                }
                break;
            }

            case UI_MODE_SLEEP:
                switch (sleep_phase) {
                case SLEEP_PHASE_INTRO:
                    ESP_LOGI(TAG, "Sleep: play intro 0");
                    if (anim_ctx.valid && ANIM_CLIPS[ANIM_CLIP_SLEEP_0].count > 0) {
                        /* Mark the real sleep clip as started. A normal-mode
                         * clip can race with the mode switch and finish while
                         * sleep_phase is INTRO; it must not satisfy the sleep
                         * completion wait. */
                        sleep_intro_started = true;
                        lcd_ui_play_clip_ordered(
                            ANIM_CLIP_SLEEP_0,
                            sleep_intro_frame_order,
                            sizeof(sleep_intro_frame_order) /
                                sizeof(sleep_intro_frame_order[0]),
                            7,
                            true);
                    }
                    break;
                case SLEEP_PHASE_READY:
                    /* Keep the last intro frame visible until the caller
                     * shuts the panel down and enters light sleep. */
                    vTaskDelay(pdMS_TO_TICKS(20));
                    break;
                case SLEEP_PHASE_WAKE:
                    if (panel_resume_active) {
                        /* LCD reset/init and the first wake frame are owned by
                         * the physical-wake task. Do not start another writer. */
                        break;
                    }
                    if (!sleep_wake_clip_started) {
                        sleep_wake_clip_started = true;
                        if (sleep_wake_animation == SLEEP_WAKE_SLEEP_B) {
                            ESP_LOGI(TAG, "Wake: play sleep/b once at %d fps",
                                     SCREEN_SAVER_WAKE_FPS);
                            lcd_ui_play_clip(SCREEN_SAVER_WAKE_CLIP,
                                             SCREEN_SAVER_WAKE_FPS,
                                             true);
                        } else {
                            ESP_LOGI(TAG, "Wake: play black-in once at %d fps",
                                     BOOT_WAKE_FPS);
                            lcd_ui_play_clip(BOOT_WAKE_CLIP,
                                             BOOT_WAKE_FPS,
                                             true);
                        }
                    }
                    break;
                }
                break;

            default:
                break;
            }
        }

        /* Wait until the current clip finishes or another UI mode cancels it. */
        while (ui_state == UI_STATE_CLIP_PLAYING) {
            if (link_wait_expired() &&
                !(ui_mode == UI_MODE_SLEEP && sleep_phase == SLEEP_PHASE_WAKE)) {
                link_wait_finish();
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        /* When the done clip finishes naturally, return to the previous mode.
         * Use just_played_done to avoid treating a start_done() transition as
         * an already-finished done animation. */
        if (just_played_done && ui_mode == UI_MODE_DONE && ui_state == UI_STATE_IDLE) {
            ESP_LOGI(TAG, "Done finished, return to %d", (int)ui_previous_mode);
            ui_mode = ui_previous_mode;
            continue;
        }

        /* Sleep mode state transitions after a clip finishes or is aborted. */
        if (ui_mode == UI_MODE_SLEEP && ui_state == UI_STATE_IDLE) {
            if (sleep_phase == SLEEP_PHASE_WAKE) {
                /* A wake request changes the phase while the mode task may
                 * still be leaving the interrupted sleep-intro iteration.
                 * IDLE alone therefore does not mean that the wake clip has
                 * finished: first let the next iteration start it, and only
                 * leave Sleep after the animation task reports completion. */
                if (!sleep_wake_clip_started || !animation_finished) {
                    continue;
                }

                /* The selected wake animation is strictly one-shot. Hand the
                 * screen to Normal, then apply the latest connection state. */
                bool used_sleep_b =
                    sleep_wake_animation == SLEEP_WAKE_SLEEP_B;
                ui_mode = UI_MODE_NORMAL;
                ESP_LOGI(TAG, "%s wake animation finished, show Normal",
                         used_sleep_b ? "sleep/b" : "black-in");
                sleep_phase = SLEEP_PHASE_INTRO;
                sleep_wake_animation = SLEEP_WAKE_BLACK_IN;
                sleep_intro_started = false;
                sleep_wake_clip_started = false;
                offline_reset_animation_state();
                continue;
            }

            if (sleep_phase == SLEEP_PHASE_INTRO && sleep_intro_started) {
                sleep_phase = SLEEP_PHASE_READY;
                /* Re-assert the exact last frame after the animation task has
                 * stopped. This makes READY a stable held frame even if a
                 * stale draw was queued during the phase transition. */
                int order_count = sizeof(sleep_intro_frame_order) /
                                  sizeof(sleep_intro_frame_order[0]);
                int final_offset = sleep_intro_frame_order[order_count - 1];
                animation_draw_clip_frame(ANIM_CLIP_SLEEP_0, final_offset);
                ESP_LOGI(TAG,
                         "Sleep intro finished, holding sleep_7 frame offset=%d",
                         final_offset);
            }
            /* SLEEP_PHASE_READY holds the last intro frame until panel off. */
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void lcd_ui_set_animation_fps(int fps)
{
    if (fps < 1) fps = 1;
    if (fps > 30) fps = 30;
    animation_fps = fps;
    animation_delay_ms = 1000 / fps;
    ESP_LOGI(TAG, "Animation FPS set to %d (delay %d ms)", fps, animation_delay_ms);
}

void lcd_ui_init(void)
{
    ESP_LOGI(TAG, "UI init: screen %dx%d", LCD_WIDTH, LCD_HEIGHT);

    /* Create UI rendering/state mutexes before starting any worker task. */
    if (ui_draw_mutex == NULL) {
        ui_draw_mutex = xSemaphoreCreateMutex();
    }
    if (ui_state_mutex == NULL) {
        ui_state_mutex = xSemaphoreCreateMutex();
    }
    if (animation_event_group == NULL) {
        animation_event_group = xEventGroupCreate();
        if (animation_event_group == NULL) {
            ESP_LOGE(TAG, "Failed to create animation stop event group");
        } else {
            xEventGroupSetBits(animation_event_group, ANIMATION_STOPPED_BIT);
        }
    }

    if (ui_input_event_queue == NULL) {
        ui_input_event_queue = xQueueCreate(UI_INPUT_EVENT_QUEUE_LENGTH,
                                             sizeof(ui_input_event_t));
    }
    if (ui_input_event_queue != NULL && ui_input_task_handle == NULL) {
        if (xTaskCreate(ui_input_task, "ui_input", 4096, NULL, 3,
                        &ui_input_task_handle) != pdPASS) {
            ui_input_task_handle = NULL;
            ESP_LOGE(TAG, "Failed to create nonblocking UI input worker");
        }
    }

    /* Allocate row-level framebuffers for flicker-free rendering
     * (legacy status-bar / keypad-status buffers; disabled for v1.0 UI). */
#if 0
    if (row1_buf == NULL) {
        row1_buf = (uint16_t *)heap_caps_malloc(LCD_WIDTH * UI_STATUS_ROW1_H * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    }
    if (row2_buf == NULL) {
        row2_buf = (uint16_t *)heap_caps_malloc(LCD_WIDTH * UI_STATUS_ROW2_H * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    }
    if (keypad_status_buf != NULL) {
        heap_caps_free(keypad_status_buf);
    }
    keypad_status_buf = (uint16_t *)heap_caps_malloc(LCD_WIDTH * KEYPAD_STATUS_H * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
#endif

    /* Create scroll animation event queue */
    if (scroll_event_queue == NULL) {
        scroll_event_queue = xQueueCreate(4, sizeof(uint8_t));
    }

    /* Start the scroll sequence controller. Frame playback stays exclusively
     * in animation_task. */
    if (scroll_task_handle == NULL) {
        xTaskCreate(scroll_control_task, "scroll_ctrl", 4096, NULL, 2,
                    &scroll_task_handle);
    }

    /* Create cursor (touchpad) animation event queue */
    if (cursor_event_queue == NULL) {
        cursor_event_queue = xQueueCreate(4, sizeof(uint8_t));
    }

    /* Start cursor animation task */
    if (cursor_task_handle == NULL) {
        xTaskCreate(cursor_animation_task, "cursor_anim", 4096, NULL, 2, &cursor_task_handle);
    }

    /* Start OTA updating animation task */
    if (updating_task_handle == NULL) {
        xTaskCreate(updating_task, "updating", 4096, NULL, 2, &updating_task_handle);
    }

    /* Start the mode-loop task (it waits until boot is complete). */
    static TaskHandle_t mode_loop_handle = NULL;
    if (mode_loop_handle == NULL) {
        xTaskCreate(mode_loop_task, "mode_loop", 4096, NULL, 2, &mode_loop_handle);
    }

    /* Boot sequence:
     *   1. Load the shared animation partition.
     *   2. Show v1.0 power-on logo for 1 second.
     *   3. Start link detection and play black-in-12FPS-2 exactly once.
     *   4. Show Normal, then select Offline if the latest link state is down.
     */
    load_compressed_animation();

    if (anim_ctx.valid) {
        lcd_ui_show_power_on_logo();
        /* lcd_init() deliberately leaves the backlight off so undefined
         * power-on GRAM is never visible. Keep display output disabled until
         * this first complete UI frame has been written as well. */
        lcd_display_on();
        /* Allow two nominal 60 Hz refresh periods before enabling the light,
         * so no portion of a pre-frame panel scan becomes visible. */
        vTaskDelay(pdMS_TO_TICKS(34));
        lcd_set_backlight(100);
        vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
        /* Keep the failure path deterministic as well: show black instead of
         * exposing uninitialized ST7789 GRAM. */
        lcd_fill_screen(LCD_COLOR_BLACK);
        lcd_display_on();
        vTaskDelay(pdMS_TO_TICKS(34));
        lcd_set_backlight(100);
    }

    boot_complete = false;

    if (anim_ctx.valid) {
        /* Boot follows the same visible sequence as a physical wake and a
         * pre-sleep-screen interruption. */
        link_wait_begin(xTaskGetTickCount());
        lcd_ui_play_clip(BOOT_WAKE_CLIP, BOOT_WAKE_FPS, true);
        while (!animation_finished) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        ESP_LOGI(TAG, "Boot black-in animation finished, show Normal");
    } else {
        link_wait_begin(xTaskGetTickCount());
    }

    /* Never expose Offline during initial link establishment. This is the
     * same two-second decision window used after physical wake. */
    ui_mode = UI_MODE_NORMAL;
    ui_state = UI_STATE_IDLE;
    offline_reset_animation_state();
    boot_complete = true;

    /* Discard any wheel/touchpad events that may have accumulated during boot
     * so the scroll/cursor animation does not start immediately on the main screen. */
    if (scroll_event_queue != NULL) {
        xQueueReset(scroll_event_queue);
    }
    if (cursor_event_queue != NULL) {
        xQueueReset(cursor_event_queue);
    }

    /* The mode loop now owns the Offline/Normal screen. */
}

void lcd_ui_set_battery_status(int percent, int millivolts, bool charging)
{
    if (percent < 0) {
        percent = -1;
    } else if (percent > 100) {
        percent = 100;
    }

    battery_anim_percent = percent;
    battery_anim_mv = millivolts;
    battery_anim_charging = charging;
}

#if 0 /* ── UI refresh / clear helpers (legacy) ─────────────────────────── */
/* Commented out for v1.0 UI. */
void lcd_ui_refresh(void)
{
    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
    lcd_fill_screen(UI_COLOR_BG);
    draw_status_bar();
    draw_logo_internal();
    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
}

void lcd_ui_clear_content(void)
{
    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
    lcd_fill_rect(0, UI_CONTENT_TOP, LCD_WIDTH, LCD_HEIGHT - UI_CONTENT_TOP, UI_COLOR_BG);
    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
}
#endif

#if 0 /* ── Status bar (two rows) ─────────────────────────────────────────── */
/* Legacy status-bar code.  Commented out for v1.0 UI. */

static void draw_row1_battery(void)
{
    if (row1_buf == NULL) return;

    int row1_base_y = UI_TOP_MARGIN;

    /* Activate framebuffer for Row 1 */
    lcd_set_framebuffer(row1_buf, 0, row1_base_y, LCD_WIDTH, UI_STATUS_ROW1_H);

    /* Clear Row 1 buffer to background */
    lcd_fill_rect(0, row1_base_y, LCD_WIDTH, UI_STATUS_ROW1_H, UI_COLOR_BG);

    char batt_str[8];
    snprintf(batt_str, sizeof(batt_str), "%d%%", ui_battery_percent);

    int icon_w = 28;
    int icon_h = 16;
    int gap = 6;
    int left_margin = 10;
    int start_x = left_margin;

    int row1_center_y = row1_base_y + UI_STATUS_ROW1_H / 2;
    int icon_y = row1_center_y - icon_h / 2;
    draw_battery_icon(start_x, icon_y, ui_battery_percent, UI_COLOR_TEXT);

    int big_y = row1_base_y + (UI_STATUS_ROW1_H - FONT_BIG22_HEIGHT) / 2;
    int text_x2 = start_x + icon_w + gap;
    u8g2_font_big22_draw_string(text_x2, big_y, batt_str, UI_COLOR_TEXT, UI_COLOR_BG, true);

    /* Signal strength icon on the right side (3 bars) */
    {
        int sig_bar_max_h = 20;
        int sig_x = LCD_WIDTH - 10 - 18 - 20; /* old pos 144, now 20px left */
        int sig_y = row1_base_y + (UI_STATUS_ROW1_H + sig_bar_max_h) / 2;
        draw_signal_icon(sig_x, sig_y, ui_signal_level, UI_COLOR_TEXT);
    }

    /* Disable framebuffer hook */
    lcd_clear_framebuffer();

    /* Flush composed Row 1 to screen in one DMA burst */
    lcd_draw_bitmap(0, row1_base_y, LCD_WIDTH, UI_STATUS_ROW1_H, row1_buf);
}

static void draw_row2_connection(void)
{
    if (row2_buf == NULL) return;

    int row2_base_y = UI_STATUS_ROW2_Y + UI_TOP_MARGIN;

    /* Activate framebuffer: all draw ops go into RAM instead of SPI */
    lcd_set_framebuffer(row2_buf, 0, row2_base_y, LCD_WIDTH, UI_STATUS_ROW2_H);

    /* Clear Row 2 buffer to background */
    lcd_fill_rect(0, row2_base_y, LCD_WIDTH, UI_STATUS_ROW2_H, UI_COLOR_BG);

    int row2_y = row2_base_y + (UI_STATUS_ROW2_H - FONT_BIG22_HEIGHT) / 2;

    /* Status indicator dot (10x10) */
    int dot_size = 10;
    int dot_x = 10;
    int dot_y = row2_base_y + (UI_STATUS_ROW2_H - dot_size) / 2;
    lcd_color_t dot_color = ui_connected ? UI_COLOR_STATUS_ON : UI_COLOR_STATUS_OFF;
    lcd_fill_rect(dot_x, dot_y, dot_size, dot_size, dot_color);

    /* Status text */
    int text_x = dot_x + dot_size + 8;
    const char *status_str = ui_connected ? "Online" : "Offline";
    lcd_color_t status_color = ui_connected ? UI_COLOR_STATUS_ON : UI_COLOR_STATUS_OFF;
    u8g2_font_big22_draw_string(text_x, row2_y, status_str, status_color, UI_COLOR_BG, true);

    /* Disable framebuffer hook */
    lcd_clear_framebuffer();

    /* Flush the composed row to screen in one DMA burst (no flicker) */
    lcd_draw_bitmap(0, row2_base_y, LCD_WIDTH, UI_STATUS_ROW2_H, row2_buf);
}

static void draw_status_bar(void)
{
    draw_row1_battery();
    draw_row2_connection();
}

/**
 * @brief Draw a 4-bar signal strength icon (phone-style)
 * @param x      Left edge of the icon
 * @param y      Bottom edge of the icon (bars grow upward)
 * @param level  0 ~ 4 (0 = no signal, 4 = full)
 * @param color  Active bar color
 */
static void draw_signal_icon(int x, int y, int level, lcd_color_t color)
{
    if (level < 0) level = 0;
    if (level > 3) level = 3;

    /* 3 bars: widths 4px, gap 2px, heights 8/14/20 */
    static const int bar_heights[3] = { 8, 14, 20 };
    int bar_w = 4;
    int gap   = 2;
    lcd_color_t dim = UI_COLOR_TEXT_DIM;

    for (int i = 0; i < 3; i++) {
        int bx = x + i * (bar_w + gap);
        int by = y - bar_heights[i];
        lcd_color_t c = (i < level) ? color : dim;
        lcd_fill_rect(bx, by, bar_w, bar_heights[i], c);
    }
}

/**
 * @brief Draw a 28x16 battery icon with fill level
 */
static void draw_battery_icon(int x, int y, int percent, lcd_color_t color)
{
    /* Clamp percent */
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    /* Battery body: 24x16, outline */
    lcd_draw_rect(x, y, 24, 16, color);

    /* Battery positive terminal: 4x8 on the right */
    lcd_fill_rect(x + 24, y + 4, 4, 8, color);

    /* Fill level inside (with 1px margin from outline) */
    int fill_w = (percent * 20) / 100;  /* 20 = inner width */
    if (fill_w < 1 && percent > 0) fill_w = 1;

    lcd_color_t fill_color;
    if (percent <= 20) {
        fill_color = LCD_COLOR_RED;
    } else if (percent <= 50) {
        fill_color = LCD_COLOR_YELLOW;
    } else {
        fill_color = LCD_COLOR_GREEN;
    }

    if (fill_w > 0) {
        lcd_fill_rect(x + 2, y + 2, fill_w, 12, fill_color);
    }
}

void lcd_ui_set_connected(bool connected)
{
    if (ui_connected == connected) return;
    ui_connected = connected;
    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
    draw_row2_connection();
    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
}

void lcd_ui_set_battery(int percent, int millivolts)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    ui_battery_percent = percent;
    ui_battery_mv = millivolts;
    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
    draw_row1_battery();
    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
}

void lcd_ui_set_signal_level(int level)
{
    if (level < 0) level = 0;
    if (level > 4) level = 4;
    if (ui_signal_level == level) return;
    ui_signal_level = level;
    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
    draw_row1_battery();
    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
}

void lcd_ui_set_keypad_pressed(bool pressed)
{
    if (ui_keypad_pressed == pressed) return;
    ui_keypad_pressed = pressed;
    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
    draw_keypad_only();
    draw_keypad_status();
    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
}

void lcd_ui_set_keypad_talking(bool talking)
{
    if (ui_keypad_talking == talking) return;
    ui_keypad_talking = talking;
    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
    draw_keypad_status();
    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
}

/* ── Unified keypad state API ──────────────────────────────────────── */

void lcd_ui_set_keypad_state(lcd_ui_keypad_state_t state)
{
    bool new_pressed   = (state == LCD_KEYPAD_PRESSING);
    bool new_listening = (state == LCD_KEYPAD_LISTENING);

    if (ui_keypad_pressed == new_pressed && ui_keypad_talking == new_listening)
        return;

    ui_keypad_pressed = new_pressed;
    ui_keypad_talking = new_listening;

    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
    draw_keypad_only();
    draw_keypad_status();
    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
}

#endif
#if 0 /* ── Logo rendering ────────────────────────────────────────────────── */
/* Legacy logo / keypad status code.  Commented out for v1.0 UI. */

/*
 * Logo rendering.
 *
 * Option A (default): Text mode — renders "/WhisPrompt_" using built-in font.
 *   The trailing "_" is an orange cursor-style underline.
 *
 * Option B (bitmap): To use a real bitmap logo exported from your design:
 *   1. Export logo as PNG (transparent background, ~140x50 px)
 *   2. python tools/png_to_bitmap.py logo.png --name logo --output main/logo_bitmap.h
 *   3. Uncomment the #define USE_BITMAP_LOGO below
 */

/* Uncomment to use bitmap logo instead of text rendering: */
#define USE_BITMAP_LOGO

#ifdef USE_BITMAP_LOGO
#include "logo_bitmap.h"
#endif
#include "keypad_bitmap.h"
#include "keypad_pressed_bitmap.h"

/* Draw only the keypad button — caller must hold ui_draw_mutex.
 * This does NOT touch the logo bitmap above it.
 * Displays the original bitmap directly without any overlay modification. */
static void draw_keypad_only(void)
{
    int logo_y = UI_LOGO_AREA_TOP + 2;
    int key_x  = (LCD_WIDTH - KEYPAD_WIDTH) / 2;
    int key_y  = logo_y + LOGO_HEIGHT + 12;

    const uint16_t *src = ui_keypad_pressed ? keypad_pressed_bitmap : keypad_bitmap;
    lcd_draw_bitmap(key_x, key_y, KEYPAD_WIDTH, KEYPAD_HEIGHT, src);
}

/* Draw keypad status text below the keypad button.
 * Uses a dedicated framebuffer to avoid flicker.
 * Caller must hold ui_draw_mutex.
 *   Waiting   : "Waiting" centred, no dots
 *   Pressing  : "Pressing" + animated dots, left-aligned at fixed anchor
 *   Listening : "Listening" + animated dots, left-aligned (prepared, not enabled yet) */
static void draw_keypad_status(void)
{
    if (keypad_status_buf == NULL) return;

    int logo_y = UI_LOGO_AREA_TOP + 2;
    int key_y  = logo_y + LOGO_HEIGHT + 12;
    int text_y = key_y + KEYPAD_HEIGHT + 12;

    /* 1. Activate framebuffer: all draw ops go into RAM */
    lcd_set_framebuffer(keypad_status_buf, 0, text_y, LCD_WIDTH, KEYPAD_STATUS_H);

    /* 2. Clear framebuffer to background */
    lcd_fill_rect(0, text_y, LCD_WIDTH, KEYPAD_STATUS_H, UI_COLOR_BG);

    /* 3. Determine base string and dot animation */
    const char *base_str;
    int dot_count = 0;
    bool animate = false;
    if (ui_keypad_pressed) {
        base_str = "Pressing";
        dot_count = ui_dot_phase;
        animate = true;
    } else if (ui_keypad_talking) {
        base_str = "Listening";
        dot_count = ui_dot_phase;
        animate = true;
    } else {
        base_str = "Waiting";
        dot_count = 0;  /* No dots for Waiting */
        animate = false;
    }

    static const char *dots[] = { "", ".", "..", "..." };
    char status_str[20];
    snprintf(status_str, sizeof(status_str), "%s%s", base_str, dots[dot_count]);

    /* 4. Layout:
     *    - Waiting  : centred (static width, no movement)
     *    - Pressing/Listening : left-aligned at fixed anchor so dots
     *      grow on the right without shifting the base text. */
    int text_x;
    int text_y_centered = text_y + (KEYPAD_STATUS_H - FONT_SMALL_HEIGHT) / 2;

    if (animate) {
        /* Anchor using the widest animated string (Pressing...) */
        int max_w = u8g2_font_small_string_width("Pressing...");
        text_x = (LCD_WIDTH - max_w) / 2;
    } else {
        int text_w = u8g2_font_small_string_width(status_str);
        text_x = (LCD_WIDTH - text_w) / 2;
    }

    u8g2_font_small_draw_string(text_x, text_y_centered, status_str, UI_COLOR_TEXT, UI_COLOR_BG, true);

    /* 5. Disable framebuffer hook */
    lcd_clear_framebuffer();

    /* 6. Flush composed buffer to screen in one DMA burst (no flicker) */
    lcd_draw_bitmap(0, text_y, LCD_WIDTH, KEYPAD_STATUS_H, keypad_status_buf);
}

/* Internal logo drawing — caller must hold ui_draw_mutex */
static void draw_logo_internal(void)
{
    /* Clear logo area with background color */
    lcd_fill_rect(0, UI_LOGO_AREA_TOP, LCD_WIDTH, UI_LOGO_AREA_H, UI_COLOR_BG);

    /* ── Bitmap logo mode ── */
    int logo_x = (LCD_WIDTH - LOGO_WIDTH) / 2;
    int logo_y = UI_LOGO_AREA_TOP + 2;  /* Close to status bar, only 2px gap */

    lcd_draw_bitmap(logo_x, logo_y, LOGO_WIDTH, LOGO_HEIGHT, logo_bitmap);

    /* Hide cursor if currently in off-phase */
    if (!logo_cursor_visible) {
        draw_logo_cursor(logo_x, logo_y, UI_COLOR_BG);
    }

    draw_keypad_only();
}

void lcd_ui_draw_logo(void)
{
    if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
    draw_logo_internal();
    if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
}

/* ── Keypad dot animation task ──────────────────────────────────────── */

static void __attribute__((unused)) keypad_dot_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(300));  /* 300ms per phase */

        /* Waiting state is static (no dots), skip animation */
        if (!ui_keypad_pressed && !ui_keypad_talking) continue;

        ui_dot_phase = (ui_dot_phase + 1) % 4;

        if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);
        draw_keypad_status();
        if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
    }
}

/* ── Logo cursor blink task ─────────────────────────────────────────── */

static void __attribute__((unused)) logo_cursor_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(530));  /* ~530ms period = typical text-cursor blink */
        logo_cursor_visible = !logo_cursor_visible;

        if (ui_draw_mutex) xSemaphoreTake(ui_draw_mutex, portMAX_DELAY);

        int logo_x = (LCD_WIDTH - LOGO_WIDTH) / 2;
        int logo_y = UI_LOGO_AREA_TOP + 2;
        lcd_color_t color = logo_cursor_visible ? LOGO_CURSOR_COLOR : UI_COLOR_BG;
        draw_logo_cursor(logo_x, logo_y, color);

        if (ui_draw_mutex) xSemaphoreGive(ui_draw_mutex);
    }
}

#endif /* Legacy logo / keypad status code */
