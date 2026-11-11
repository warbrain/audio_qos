#include "scepter_hardware.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_now.h"
#include "sdkconfig.h"
#include "dual_end_audio.h"
#include "dual_end_espnow_link.h"
#include "battery_monitor.h"
#include "imu_module.h"
#include "light_sleep_module.h"
#include "lcd_display.h"
#include "mic_audio.h"
#include "motor_vibration.h"
#include "rgb_led.h"
#include "touchpad_module.h"
#include "user_keys.h"
#include "wheel_module.h"

static const char *TAG = "scepter_hw";

#define SCEPTER_HW_RUNTIME_LOGS 0

#ifndef SCEPTER_TOUCH_PACKET_STATS_LOGS
#define SCEPTER_TOUCH_PACKET_STATS_LOGS 0
#endif
#define SCEPTER_TOUCH_PACKET_STATS_MS 1000

/* Temporary UI verification: key1=Thinking, key2=Approve, key3=Done.
 * Set to 0 when the keys should return to the normal pressed-screen UI. */
#ifndef SCEPTER_UI_KEY_ANIMATION_TEST_ENABLE
#define SCEPTER_UI_KEY_ANIMATION_TEST_ENABLE 0
#endif

#ifndef SCEPTER_HW_TEST_TASK_ENABLE
#define SCEPTER_HW_TEST_TASK_ENABLE 1
#endif

#ifndef SCEPTER_IMU_SLEEP_ENABLE
#define SCEPTER_IMU_SLEEP_ENABLE 1
#endif

#ifndef SCEPTER_IMU_SLEEP_STILL_MS
#define SCEPTER_IMU_SLEEP_STILL_MS LIGHT_SLEEP_MODULE_DEFAULT_FLAT_SLEEP_MS
#endif

#define SCEPTER_SLEEP_ANIMATION_LEAD_MS 5000
#define SCEPTER_OFFLINE_ORIENTATION_AXIS_MIN 2400
#define SCEPTER_OFFLINE_ORIENTATION_FLAT_Z_MIN 2600
#define SCEPTER_OFFLINE_ORIENTATION_DOMINANCE_MARGIN 350
#define SCEPTER_OFFLINE_ORIENTATION_CONFIRM_US 150000

#ifndef CONFIG_IMU_INT1_GPIO
#define CONFIG_IMU_INT1_GPIO 7
#endif

#define SCEPTER_HW_TEST_TASK_STACK_BYTES 2048
#define SCEPTER_HW_TEST_TASK_PRIORITY 3
#define SCEPTER_HW_TEST_BREATH_STEP_MS 20
#define SCEPTER_HW_TEST_BREATH_STEP_PERCENT 2
#define SCEPTER_WAKE_INPUT_HOLD_FRAMES 3

static volatile bool s_mic_tx_enabled = true;
static volatile bool s_comm_recovering;
static volatile bool s_ota_active;
static uint32_t s_audio_frames;
static uint32_t s_audio_errors;
static portMUX_TYPE s_hid_input_lock = portMUX_INITIALIZER_UNLOCKED;
static dual_end_hid_input_payload_t s_hid_input_payload = {
    .version = DUAL_END_HID_INPUT_VERSION,
    .wheel_button_state = DUAL_END_HID_WHEEL_BUTTON_STATE_VALID,
};
static bool s_link_connected;
static bool s_light_sleep_ready;
static volatile bool s_sleep_intro_active;
static volatile bool s_sleep_entry_active;
static uint32_t s_link_wait_drops;
static uint8_t s_display_key_state;
static bool s_display_wheel_button_pressed;
static uint8_t s_wake_key_latch_mask;
static uint8_t s_wake_mouse_button_latch;
static uint8_t s_wake_input_hold_frames;
static uint32_t s_wake_input_latch_generation;
static int64_t s_last_input_activity_us;
static bool s_offline_orientation_candidate_valid;
static lcd_offline_orientation_t s_offline_orientation_candidate =
    LCD_OFFLINE_ORIENTATION_FLAT;
static lcd_offline_orientation_t s_offline_orientation_published =
    LCD_OFFLINE_ORIENTATION_FLAT;
static int64_t s_offline_orientation_candidate_since_us;
#if SCEPTER_TOUCH_PACKET_STATS_LOGS
static int32_t s_touch_last_sent_dx_total;
static int32_t s_touch_last_sent_dy_total;
static int64_t s_touch_packet_stats_start_us;
static uint32_t s_touch_packet_stats_sent;
static uint32_t s_touch_packet_stats_active;
static uint32_t s_touch_packet_stats_catchup;
static uint64_t s_touch_packet_stats_abs_dx;
static uint64_t s_touch_packet_stats_abs_dy;
static uint32_t s_touch_packet_stats_max_dx;
static uint32_t s_touch_packet_stats_max_dy;
#endif

#if SCEPTER_HW_TEST_TASK_ENABLE
static void hardware_test_task(void *arg)
{
    (void)arg;

    TickType_t last_wake_tick = xTaskGetTickCount();
    int brightness_percent = 0;
    int breath_step = SCEPTER_HW_TEST_BREATH_STEP_PERCENT;

    ESP_LOGI(TAG, "hardware test task started: orange LED breathing, period=%dms",
             (200 / SCEPTER_HW_TEST_BREATH_STEP_PERCENT) * SCEPTER_HW_TEST_BREATH_STEP_MS);
    while (true) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            rgb_led_set_orange_brightness((uint8_t)brightness_percent));

        brightness_percent += breath_step;
        if (brightness_percent >= 100) {
            brightness_percent = 100;
            breath_step = -SCEPTER_HW_TEST_BREATH_STEP_PERCENT;
        } else if (brightness_percent <= 0) {
            brightness_percent = 0;
            breath_step = SCEPTER_HW_TEST_BREATH_STEP_PERCENT;
        }

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(SCEPTER_HW_TEST_BREATH_STEP_MS));
    }
}
#endif

static const char *posture_name(imu_posture_t posture)
{
    switch (posture) {
    case IMU_POSTURE_STILL:
        return "STILL";
    case IMU_POSTURE_LIFT:
        return "LIFT";
    case IMU_POSTURE_SHAKE:
        return "SHAKE";
    case IMU_POSTURE_IDLE:
        return "IDLE";
    default:
        return "IDLE";
    }
}

static void scepter_hardware_update_lcd(void)
{
    bool active = s_link_connected && s_mic_tx_enabled && !s_ota_active;
    lcd_display_set_paired(s_link_connected);
    lcd_display_set_text(s_ota_active ? "Updating" :
                         s_comm_recovering ? "Recovering" :
                         active ? "Listening" :
                         s_mic_tx_enabled ? "Waiting" : "Sleeping");
    lcd_display_set_mic(active);
}

static uint8_t key_mask_for_id(user_key_id_t key_id)
{
    switch (key_id) {
    case USER_KEY_ID_1:
        return DUAL_END_SENSOR_KEY_1_MASK;
    case USER_KEY_ID_2:
        return DUAL_END_SENSOR_KEY_2_MASK;
    case USER_KEY_ID_3:
        return DUAL_END_SENSOR_KEY_3_MASK;
    default:
        return 0;
    }
}

static void scepter_cancel_sleep_countdown(void)
{
    if (!s_sleep_intro_active || s_sleep_entry_active) {
        return;
    }
    s_sleep_intro_active = false;
    lcd_display_cancel_sleep_intro();
}

static void scepter_note_input_activity(void)
{
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_hid_input_lock);
    s_last_input_activity_us = now_us;
    portEXIT_CRITICAL(&s_hid_input_lock);
    scepter_cancel_sleep_countdown();
}

static void scepter_latch_wake_input_locked(uint8_t key_mask, uint8_t mouse_button_mask)
{
    s_wake_key_latch_mask |= key_mask;
    s_wake_mouse_button_latch |= mouse_button_mask & 0x03;
    s_wake_input_hold_frames = SCEPTER_WAKE_INPUT_HOLD_FRAMES;
    s_wake_input_latch_generation++;
}

static uint32_t scepter_input_idle_ms(void)
{
    int64_t last_activity_us = 0;
    portENTER_CRITICAL(&s_hid_input_lock);
    last_activity_us = s_last_input_activity_us;
    portEXIT_CRITICAL(&s_hid_input_lock);

    if (last_activity_us <= 0) {
        return UINT32_MAX;
    }
    int64_t now_us = esp_timer_get_time();
    if (now_us <= last_activity_us) {
        return 0;
    }
    uint64_t idle_ms = (uint64_t)(now_us - last_activity_us) / 1000ULL;
    return idle_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)idle_ms;
}

static bool scepter_peripherals_idle(void)
{
    uint8_t key_state = 0;
    uint8_t mouse_buttons = 0;

    portENTER_CRITICAL(&s_hid_input_lock);
    key_state = s_hid_input_payload.key_state;
    mouse_buttons = s_hid_input_payload.mouse_buttons & 0x03;
    portEXIT_CRITICAL(&s_hid_input_lock);

    /*
     * Use both debounced/reported state and live module state. The live checks
     * close the interval before a debounce callback arrives and also cover a
     * control that was already held when its worker task started.
     */
    return key_state == 0 &&
           mouse_buttons == 0 &&
           !user_keys_any_pressed() &&
           !wheel_module_is_button_pressed() &&
           !touchpad_module_is_active();
}

static esp_err_t append_wakeup_io(light_sleep_module_wakeup_io_t *wake_ios,
                                  size_t capacity,
                                  size_t *count,
                                  gpio_num_t gpio_num,
                                  gpio_int_type_t wake_level,
                                  gpio_int_type_t runtime_intr_type,
                                  bool runtime_intr_enabled)
{
    ESP_RETURN_ON_FALSE(wake_ios && count && *count < capacity,
                        ESP_ERR_INVALID_SIZE,
                        TAG,
                        "wake GPIO list is full");
    wake_ios[*count] = (light_sleep_module_wakeup_io_t) {
        .gpio_num = gpio_num,
        .wake_level = wake_level,
        .runtime_intr_type = runtime_intr_type,
        .runtime_intr_enabled = runtime_intr_enabled,
    };
    (*count)++;
    return ESP_OK;
}

static esp_err_t scepter_hardware_prepare_wakeup_ios(
    light_sleep_module_wakeup_io_t *wake_ios,
    size_t capacity,
    size_t *wake_io_count,
    void *user_ctx)
{
    (void)user_ctx;
    ESP_RETURN_ON_FALSE(wake_ios && wake_io_count,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "wake GPIO output is null");

    size_t count = 0;
    ESP_RETURN_ON_ERROR(append_wakeup_io(wake_ios,
                                         capacity,
                                         &count,
                                         CONFIG_IMU_INT1_GPIO,
                                         GPIO_INTR_HIGH_LEVEL,
                                         GPIO_INTR_DISABLE,
                                         false),
                        TAG,
                        "add IMU wake GPIO failed");

    gpio_num_t key_gpios[USER_KEYS_COUNT] = {0};
    ESP_RETURN_ON_FALSE(user_keys_get_gpios(key_gpios, USER_KEYS_COUNT) == USER_KEYS_COUNT,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "get key wake GPIOs failed");
    for (size_t i = 0; i < USER_KEYS_COUNT; i++) {
        ESP_RETURN_ON_ERROR(append_wakeup_io(wake_ios,
                                             capacity,
                                             &count,
                                             key_gpios[i],
                                             GPIO_INTR_LOW_LEVEL,
                                             GPIO_INTR_ANYEDGE,
                                             true),
                            TAG,
                            "add key wake GPIO failed");
    }

    wheel_module_wakeup_state_t wheel = {0};
    wheel_module_arm_wakeup();
    wheel_module_get_wakeup_state(&wheel);
    ESP_RETURN_ON_ERROR(append_wakeup_io(wake_ios,
                                         capacity,
                                         &count,
                                         wheel.a_gpio,
                                         (wheel.encoder_state & 0x02) ?
                                             GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL,
                                         GPIO_INTR_ANYEDGE,
                                         true),
                        TAG,
                        "add wheel A wake GPIO failed");
    ESP_RETURN_ON_ERROR(append_wakeup_io(wake_ios,
                                         capacity,
                                         &count,
                                         wheel.b_gpio,
                                         (wheel.encoder_state & 0x01) ?
                                             GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL,
                                         GPIO_INTR_ANYEDGE,
                                         true),
                        TAG,
                        "add wheel B wake GPIO failed");
    ESP_RETURN_ON_ERROR(append_wakeup_io(wake_ios,
                                         capacity,
                                         &count,
                                         wheel.button_gpio,
                                         GPIO_INTR_LOW_LEVEL,
                                         GPIO_INTR_ANYEDGE,
                                         true),
                        TAG,
                        "add wheel button wake GPIO failed");

    ESP_RETURN_ON_ERROR(append_wakeup_io(wake_ios,
                                         capacity,
                                         &count,
                                         touchpad_module_get_interrupt_gpio(),
                                         GPIO_INTR_LOW_LEVEL,
                                         GPIO_INTR_NEGEDGE,
                                         true),
                        TAG,
                        "add touch wake GPIO failed");

    *wake_io_count = count;
    return ESP_OK;
}

static void scepter_hardware_wakeup_event_cb(uint64_t wake_gpio_mask, void *user_ctx)
{
    (void)user_ctx;
    scepter_note_input_activity();

    uint8_t wake_key_ids = user_keys_get_ids_for_gpio_mask(wake_gpio_mask);
    uint8_t wake_key_mask = 0;
    for (size_t i = 0; i < USER_KEYS_COUNT; i++) {
        if (wake_key_ids & (1U << i)) {
            wake_key_mask |= key_mask_for_id((user_key_id_t)i);
        }
    }
    if (wake_key_mask != 0) {
        portENTER_CRITICAL(&s_hid_input_lock);
        scepter_latch_wake_input_locked(wake_key_mask, 0);
        portEXIT_CRITICAL(&s_hid_input_lock);
        ESP_ERROR_CHECK_WITHOUT_ABORT(user_keys_notify_wakeup(wake_gpio_mask));
    }

    wheel_module_wakeup_state_t wheel = {0};
    wheel_module_get_wakeup_state(&wheel);
    uint64_t wheel_mask = (1ULL << wheel.a_gpio) |
                          (1ULL << wheel.b_gpio) |
                          (1ULL << wheel.button_gpio);
    if (wake_gpio_mask & wheel_mask) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(wheel_module_notify_wakeup(wake_gpio_mask));
    }

    gpio_num_t touch_gpio = touchpad_module_get_interrupt_gpio();
    if (wake_gpio_mask & (1ULL << touch_gpio)) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(touchpad_module_process_wakeup());
    }

    ESP_LOGI(TAG,
             "input wake mask=0x%" PRIx64 " keys=0x%02x wheel=%d touch=%d",
             wake_gpio_mask,
             wake_key_mask,
             (wake_gpio_mask & wheel_mask) != 0,
             (wake_gpio_mask & (1ULL << touch_gpio)) != 0);
}

#if SCEPTER_TOUCH_PACKET_STATS_LOGS
static uint32_t abs_i32_to_u32(int32_t value)
{
    if (value >= 0) {
        return (uint32_t)value;
    }
    return (uint32_t)(-(int64_t)value);
}

static void touch_packet_stats_on_sent(const dual_end_hid_input_payload_t *payload)
{
    if (!payload) {
        return;
    }

    int32_t packet_dx = payload->touch_dx_total - s_touch_last_sent_dx_total;
    int32_t packet_dy = payload->touch_dy_total - s_touch_last_sent_dy_total;
    s_touch_last_sent_dx_total = payload->touch_dx_total;
    s_touch_last_sent_dy_total = payload->touch_dy_total;

    uint32_t abs_dx = abs_i32_to_u32(packet_dx);
    uint32_t abs_dy = abs_i32_to_u32(packet_dy);
    s_touch_packet_stats_sent++;
    if (packet_dx != 0 || packet_dy != 0) {
        s_touch_packet_stats_active++;
        s_touch_packet_stats_abs_dx += abs_dx;
        s_touch_packet_stats_abs_dy += abs_dy;
        if (abs_dx > s_touch_packet_stats_max_dx) {
            s_touch_packet_stats_max_dx = abs_dx;
        }
        if (abs_dy > s_touch_packet_stats_max_dy) {
            s_touch_packet_stats_max_dy = abs_dy;
        }
        if (abs_dx > INT8_MAX || abs_dy > INT8_MAX) {
            s_touch_packet_stats_catchup++;
        }
    }

    int64_t now_us = esp_timer_get_time();
    if (s_touch_packet_stats_start_us == 0) {
        s_touch_packet_stats_start_us = now_us;
    }
    if ((now_us - s_touch_packet_stats_start_us) >=
        SCEPTER_TOUCH_PACKET_STATS_MS * 1000LL) {
        if (s_touch_packet_stats_active > 0) {
            ESP_LOGI(TAG,
                     "touch packets sent=%" PRIu32 " active=%" PRIu32
                     " avg_abs=(%" PRIu64 ",%" PRIu64 ")"
                     " max_abs=(%" PRIu32 ",%" PRIu32 ")"
                     " catchup_gt127=%" PRIu32 " last=(%" PRId32 ",%" PRId32 ")",
                     s_touch_packet_stats_sent,
                     s_touch_packet_stats_active,
                     s_touch_packet_stats_abs_dx / s_touch_packet_stats_active,
                     s_touch_packet_stats_abs_dy / s_touch_packet_stats_active,
                     s_touch_packet_stats_max_dx,
                     s_touch_packet_stats_max_dy,
                     s_touch_packet_stats_catchup,
                     packet_dx,
                     packet_dy);
        }

        s_touch_packet_stats_start_us = now_us;
        s_touch_packet_stats_sent = 0;
        s_touch_packet_stats_active = 0;
        s_touch_packet_stats_catchup = 0;
        s_touch_packet_stats_abs_dx = 0;
        s_touch_packet_stats_abs_dy = 0;
        s_touch_packet_stats_max_dx = 0;
        s_touch_packet_stats_max_dy = 0;
    }
}
#endif

static void mic_frame_cb(const int16_t *samples,
                         size_t sample_count,
                         const dual_end_audio_analysis_metadata_t *analysis,
                         void *user_ctx)
{
    (void)user_ctx;

    if (!s_mic_tx_enabled || s_comm_recovering || s_ota_active || !samples || sample_count == 0) {
        return;
    }
    if (!dual_end_espnow_link_is_connected()) {
        s_link_wait_drops++;
        if ((s_link_wait_drops % 50) == 1) {
            ESP_LOGI(TAG,
                     "mic frame held until ESP-NOW link is connected drops=%" PRIu32,
                     s_link_wait_drops);
        }
        return;
    }
    if (sample_count != DUAL_END_AUDIO_FRAME_SAMPLE_VALUES) {
        s_audio_errors++;
        if ((s_audio_errors % 50) == 1) {
            ESP_LOGW(TAG, "drop mic frame samples=%u expected=%u errors=%" PRIu32,
                     (unsigned)sample_count,
                     (unsigned)DUAL_END_AUDIO_FRAME_SAMPLE_VALUES,
                     s_audio_errors);
        }
        return;
    }

    dual_end_hid_input_payload_t hardware_payload;
    uint8_t wake_key_latch = 0;
    uint8_t wake_mouse_button_latch = 0;
    uint8_t wake_input_hold_frames = 0;
    uint32_t wake_input_latch_generation = 0;
    portENTER_CRITICAL(&s_hid_input_lock);
    hardware_payload = s_hid_input_payload;
    wake_key_latch = s_wake_key_latch_mask;
    wake_mouse_button_latch = s_wake_mouse_button_latch;
    wake_input_hold_frames = s_wake_input_hold_frames;
    wake_input_latch_generation = s_wake_input_latch_generation;
    hardware_payload.key_state |= wake_key_latch;
    hardware_payload.mouse_buttons |= wake_mouse_button_latch;
    portEXIT_CRITICAL(&s_hid_input_lock);

    esp_err_t ret = dual_end_audio_sender_send_pcm_with_hardware_and_analysis(
        samples,
        sample_count,
        &hardware_payload,
        analysis);
    if (ret == ESP_OK) {
#if SCEPTER_TOUCH_PACKET_STATS_LOGS
        touch_packet_stats_on_sent(&hardware_payload);
#endif
        portENTER_CRITICAL(&s_hid_input_lock);
        /* Clear only one-shot flags that were accepted into the TX queue.
         * Flags raised concurrently remain latched for the next audio frame. */
        s_hid_input_payload.flags &= (uint8_t)~hardware_payload.flags;
        if (s_wake_input_latch_generation == wake_input_latch_generation &&
            wake_input_hold_frames > 0) {
            if (s_wake_input_hold_frames > 1) {
                s_wake_input_hold_frames--;
            } else {
                s_wake_input_hold_frames = 0;
                s_wake_key_latch_mask &= (uint8_t)~wake_key_latch;
                s_wake_mouse_button_latch &= (uint8_t)~wake_mouse_button_latch;
            }
        }
        portEXIT_CRITICAL(&s_hid_input_lock);
        s_audio_frames++;
    } else {
        s_audio_errors++;
        if ((s_audio_errors % 50) == 1) {
            ESP_LOGW(TAG, "audio frame send failed ret=%s errors=%" PRIu32,
                     esp_err_to_name(ret), s_audio_errors);
        }
    }
}

static void mic_stats_cb(const mic_audio_stats_t *stats, void *user_ctx)
{
    (void)user_ctx;
    if (!SCEPTER_HW_RUNTIME_LOGS || !stats) {
        return;
    }

    ESP_LOGI(TAG,
             "mic in/out=%" PRIu32 "/%" PRIu32 " bytes=%" PRIu32
             " raw[L=%" PRIu32 "/%" PRId32 " R=%" PRIu32 "/%" PRId32 "]"
             " pcm[L=%" PRIu32 "/%" PRId32 " R=%" PRIu32 "/%" PRId32 "]"
             " fixed_gain=%ux right_gain=%ux soft_limit=%" PRIu32 " min_gain=%u/1000"
             " packets/s=%" PRIu32 " sample_values/s=%" PRIu32
             " analysis=%" PRIu32 " avg/max=%" PRIu32 "/%" PRIu32 "us"
             " vad=%u gate_open=%u"
             " flags=0x%02x tdoa=%d/256 peak/conf/coh=%u/%u/%u"
             " quality=%u/%u hf/motor=%u/%u clip=%u/%u"
             " tx_frames=%" PRIu32 " tx_errors=%" PRIu32,
             stats->input_frames,
             stats->output_frames,
             stats->bytes_read,
             stats->left_raw_rms,
             stats->left_raw_peak,
             stats->right_raw_rms,
             stats->right_raw_peak,
             stats->left_pcm_rms,
             stats->left_pcm_peak,
             stats->right_pcm_rms,
             stats->right_pcm_peak,
             (unsigned)stats->output_gain_x,
             (unsigned)stats->right_output_gain_x,
             stats->soft_limited_frames,
             (unsigned)stats->minimum_gain_permille,
             stats->packets_per_sec,
             stats->sample_values_per_sec,
             stats->analysis_frames,
             stats->analysis_average_us,
             stats->analysis_max_us,
             (unsigned)stats->vad_enabled,
             (unsigned)stats->vad_open,
             stats->latest_analysis.flags,
             stats->latest_analysis.tdoa_samples_q8_8,
             stats->latest_analysis.gcc_peak_q15,
             stats->latest_analysis.confidence_q15,
             stats->latest_analysis.coherence_q15,
             stats->latest_analysis.left_quality_q15,
             stats->latest_analysis.right_quality_q15,
             stats->latest_analysis.high_frequency_q15,
             stats->latest_analysis.motor_q15,
             stats->latest_analysis.clip_left_count,
             stats->latest_analysis.clip_right_count,
             s_audio_frames,
             s_audio_errors);
}

static void scepter_hardware_set_tx_enabled(bool enabled, void *user_ctx)
{
    (void)user_ctx;
    s_mic_tx_enabled = enabled;
    scepter_hardware_update_lcd();
}

static void scepter_hardware_set_comm_recovering(bool enabled, void *user_ctx)
{
    (void)user_ctx;
    s_comm_recovering = enabled;
    scepter_hardware_update_lcd();
}

static esp_err_t scepter_hardware_recover_comm(void *user_ctx)
{
    (void)user_ctx;
    return dual_end_espnow_link_recover_peer(NULL);
}

static esp_err_t scepter_hardware_send_hello(void *user_ctx)
{
    (void)user_ctx;
    return dual_end_espnow_link_send_hello();
}

static esp_err_t scepter_hardware_sleep_prepare_cb(void *user_ctx)
{
    (void)user_ctx;
    ESP_RETURN_ON_ERROR(lcd_display_prepare_sleep(), TAG, "LCD sleep prepare failed");
    return mic_audio_set_enabled(false);
}

static void scepter_hardware_sleep_wake_cb(void *user_ctx)
{
    (void)user_ctx;
    esp_err_t mic_ret = mic_audio_set_enabled(true);
    if (mic_ret != ESP_OK) {
        ESP_LOGW(TAG, "microphone resume failed: %s", esp_err_to_name(mic_ret));
    }
    lcd_display_resume_after_sleep();
    battery_monitor_force_next_sync();
}

static bool imu_offline_orientation_from_sample(
    const imu_sample_t *sample,
    lcd_offline_orientation_t *orientation)
{
    int32_t ax = sample->accel_x;
    int32_t ay = sample->accel_y;
    int32_t az = sample->accel_z;
    int32_t abs_ax = ax < 0 ? -ax : ax;
    int32_t abs_ay = ay < 0 ? -ay : ay;
    int32_t abs_az = az < 0 ? -az : az;
    const int32_t margin = SCEPTER_OFFLINE_ORIENTATION_DOMINANCE_MARGIN;

    if (abs_az >= SCEPTER_OFFLINE_ORIENTATION_FLAT_Z_MIN &&
        abs_az >= abs_ax + margin &&
        abs_az >= abs_ay + margin) {
        *orientation = LCD_OFFLINE_ORIENTATION_FLAT;
        return true;
    }

    if (abs_ay >= SCEPTER_OFFLINE_ORIENTATION_AXIS_MIN &&
        abs_ay >= abs_ax + margin &&
        abs_ay >= abs_az + margin) {
        /* With the PCB Y axis pointing toward the physical top, +Y means the
         * bottom edge is down. Move the character toward that lower edge. */
        *orientation = ay >= 0 ? LCD_OFFLINE_ORIENTATION_UPRIGHT :
                                LCD_OFFLINE_ORIENTATION_UPSIDE_DOWN;
        return true;
    }

    if (abs_ax >= SCEPTER_OFFLINE_ORIENTATION_AXIS_MIN &&
        abs_ax >= abs_ay + margin &&
        abs_ax >= abs_az + margin) {
        /* +X means the left edge is down; -X means the right edge is down. */
        *orientation = ax >= 0 ? LCD_OFFLINE_ORIENTATION_LEFT :
                                LCD_OFFLINE_ORIENTATION_RIGHT;
        return true;
    }

    return false;
}

static void imu_sample_cb(const imu_sample_t *sample, void *user_ctx)
{
    (void)user_ctx;
    if (sample == NULL) {
        return;
    }

    lcd_offline_orientation_t orientation;
    if (!imu_offline_orientation_from_sample(sample, &orientation)) {
        s_offline_orientation_candidate_valid = false;
        return;
    }

    int64_t now_us = esp_timer_get_time();
    if (!s_offline_orientation_candidate_valid ||
        orientation != s_offline_orientation_candidate) {
        s_offline_orientation_candidate_valid = true;
        s_offline_orientation_candidate = orientation;
        s_offline_orientation_candidate_since_us = now_us;
        return;
    }

    if (orientation != s_offline_orientation_published &&
        now_us - s_offline_orientation_candidate_since_us >=
            SCEPTER_OFFLINE_ORIENTATION_CONFIRM_US) {
        s_offline_orientation_published = orientation;
        lcd_display_set_offline_orientation(orientation);
    }
}

static void imu_posture_cb(imu_posture_t posture, uint32_t stable_ms, void *user_ctx)
{
    (void)user_ctx;

    if (posture == IMU_POSTURE_SHAKE) {
        lcd_display_on_imu_shake();
    }

    if (posture == IMU_POSTURE_STILL) {
        uint32_t input_idle_ms = scepter_input_idle_ms();
        uint32_t sleep_ms = light_sleep_module_get_flat_sleep_ms();
        uint32_t animation_start_ms = sleep_ms > SCEPTER_SLEEP_ANIMATION_LEAD_MS ?
                                      sleep_ms - SCEPTER_SLEEP_ANIMATION_LEAD_MS : 0;
        bool peripherals_idle = scepter_peripherals_idle();

        /* An input can remain active without generating additional edges.
         * Never treat a held key/button/touch as idle, and cancel an already
         * running pre-sleep animation immediately. Release callbacks refresh
         * s_last_input_activity_us, so the timeout restarts from release. */
        if (!peripherals_idle) {
            scepter_cancel_sleep_countdown();
            return;
        }

        bool can_sleep = SCEPTER_IMU_SLEEP_ENABLE &&
                         s_light_sleep_ready &&
                         !s_ota_active &&
                         s_mic_tx_enabled &&
                         peripherals_idle;

        /* At 10 seconds of continuous stillness/input inactivity, start the
         * sleep intro but keep audio and ESP-NOW running for another 5 seconds. */
        if (can_sleep && !s_sleep_intro_active &&
            stable_ms >= animation_start_ms && input_idle_ms >= animation_start_ms) {
            s_sleep_intro_active = true;
            lcd_display_start_sleep_intro();
            ESP_LOGI(TAG, "sleep countdown: intro at posture=%" PRIu32
                          "ms input_idle=%" PRIu32 "ms",
                     stable_ms, input_idle_ms);
        }

        /* Enter light sleep only after the complete 15-second condition has
         * remained true. Any input or posture change cancels the countdown. */
        if (can_sleep && s_sleep_intro_active && !s_sleep_entry_active &&
            stable_ms >= sleep_ms && input_idle_ms >= sleep_ms) {
            s_sleep_entry_active = true;
            esp_err_t ret = light_sleep_module_enter(stable_ms);
            s_sleep_entry_active = false;
            s_sleep_intro_active = false;
            if (ret != ESP_OK) {
                /* No-op if a real wake animation has already begun; otherwise
                 * restore Normal immediately after an aborted sleep entry. */
                lcd_display_cancel_sleep_intro();
                ESP_LOGW(TAG,
                         "light sleep skipped: posture=%s stable=%" PRIu32 "ms ret=%s",
                         posture_name(posture),
                         stable_ms,
                         esp_err_to_name(ret));
            }
        }
        return;
    }

    scepter_cancel_sleep_countdown();

    if (SCEPTER_IMU_SLEEP_ENABLE && !s_mic_tx_enabled) {
        scepter_hardware_set_tx_enabled(true, NULL);
        ESP_LOGI(TAG, "mic tx enabled: posture=%s", posture_name(posture));
    }
}

static void touchpad_motion_cb(const touchpad_motion_t *motion, void *user_ctx)
{
    (void)user_ctx;
    if (!motion) {
        return;
    }

    scepter_note_input_activity();

    portENTER_CRITICAL(&s_hid_input_lock);
    s_hid_input_payload.touch_dx_total += motion->dx;
    s_hid_input_payload.touch_dy_total += motion->dy;
    s_hid_input_payload.mouse_buttons = motion->buttons & 0x03;
    if ((motion->buttons & 0x03) != 0 && (!s_mic_tx_enabled || s_comm_recovering)) {
        scepter_latch_wake_input_locked(0, motion->buttons);
    }
    s_hid_input_payload.wheel_v_total += motion->vertical;
    s_hid_input_payload.wheel_h_total += motion->horizontal;
    if (motion->buttons & 0x10) {
        s_hid_input_payload.flags |= DUAL_END_HID_FLAG_ZOOM_IN;
    }
    if (motion->buttons & 0x20) {
        s_hid_input_payload.flags |= DUAL_END_HID_FLAG_ZOOM_OUT;
    }
    portEXIT_CRITICAL(&s_hid_input_lock);

    lcd_display_on_touchpad_event();

    if (SCEPTER_HW_RUNTIME_LOGS) {
        ESP_LOGI(TAG, "touch dx=%d dy=%d buttons=%u v=%d h=%d",
                 motion->dx, motion->dy, motion->buttons, motion->vertical, motion->horizontal);
    }
}

static void user_key_state_cb(user_key_id_t key_id, bool pressed, void *user_ctx)
{
    (void)user_ctx;
    scepter_note_input_activity();
    uint8_t mask = key_mask_for_id(key_id);
    if (mask) {
        bool display_pressed;
        portENTER_CRITICAL(&s_hid_input_lock);
        if (pressed) {
            s_hid_input_payload.key_state |= mask;
            s_hid_input_payload.key_press_counter[key_id]++;
            s_display_key_state |= mask;
            if (!s_mic_tx_enabled || s_comm_recovering) {
                scepter_latch_wake_input_locked(mask, 0);
            }
        } else {
            s_hid_input_payload.key_state &= (uint8_t)~mask;
            s_display_key_state &= (uint8_t)~mask;
        }
        display_pressed =
            s_display_key_state != 0 || s_display_wheel_button_pressed;
        portEXIT_CRITICAL(&s_hid_input_lock);

#if SCEPTER_UI_KEY_ANIMATION_TEST_ENABLE
        if (pressed) {
            switch (key_id) {
            case USER_KEY_ID_1:
                lcd_display_play_thinking_once();
                break;
            case USER_KEY_ID_2:
                lcd_display_play_approve_once();
                break;
            case USER_KEY_ID_3:
                lcd_display_play_done_once();
                break;
            default:
                break;
            }
        }
#else
        lcd_display_set_keypad_pressed(display_pressed);
#endif
    }

    if (SCEPTER_HW_RUNTIME_LOGS) {
        ESP_LOGI(TAG, "key%u %s", (unsigned)key_id + 1, pressed ? "down" : "up");
    }
}

static void wheel_report_cb(wheel_module_state_t state, void *user_ctx)
{
    (void)user_ctx;
    scepter_note_input_activity();
    bool display_pressed;
    portENTER_CRITICAL(&s_hid_input_lock);
    if (state == WHEEL_MODULE_STATE_SCROLL_UP) {
        s_hid_input_payload.wheel_v_total++;
    } else if (state == WHEEL_MODULE_STATE_SCROLL_DOWN) {
        s_hid_input_payload.wheel_v_total--;
    } else if (state == WHEEL_MODULE_STATE_BUTTON_PRESSED) {
        s_hid_input_payload.wheel_button_counter++;
        s_hid_input_payload.wheel_button_state =
            DUAL_END_HID_WHEEL_BUTTON_STATE_VALID |
            DUAL_END_HID_WHEEL_BUTTON_PRESSED;
        s_display_wheel_button_pressed = true;
    } else if (state == WHEEL_MODULE_STATE_BUTTON_RELEASED) {
        s_hid_input_payload.wheel_button_state =
            DUAL_END_HID_WHEEL_BUTTON_STATE_VALID;
        s_display_wheel_button_pressed = false;
    }
    display_pressed =
        s_display_key_state != 0 || s_display_wheel_button_pressed;
    portEXIT_CRITICAL(&s_hid_input_lock);

    if (state == WHEEL_MODULE_STATE_SCROLL_UP ||
        state == WHEEL_MODULE_STATE_SCROLL_DOWN) {
        lcd_display_on_wheel_event();
    } else {
        /* Releasing one input keeps the overlay visible while any other
         * normal key or the wheel button remains held. */
        lcd_display_set_keypad_pressed(display_pressed);
    }

    if (SCEPTER_HW_RUNTIME_LOGS) {
        ESP_LOGI(TAG, "wheel state=%u", (unsigned)state);
    }
}

esp_err_t scepter_hardware_start(void)
{
    ESP_LOGI(TAG, "starting scepter real hardware peripherals");

    scepter_hardware_update_lcd();

    wheel_module_set_report_callback(wheel_report_cb, NULL);
    user_keys_set_pressed_callback(NULL, NULL);
    user_keys_set_state_callback(user_key_state_cb, NULL);
    touchpad_module_set_motion_callback(touchpad_motion_cb, NULL);
    imu_module_set_sample_callback(imu_sample_cb, NULL);
    imu_module_set_posture_callback(imu_posture_cb, NULL);
    mic_audio_set_frame_callback(mic_frame_cb, NULL);
    mic_audio_set_stats_callback(SCEPTER_HW_RUNTIME_LOGS ? mic_stats_cb : NULL, NULL);

    ESP_RETURN_ON_ERROR(wheel_module_init(), TAG, "wheel init");
    s_display_wheel_button_pressed = wheel_module_is_button_pressed();
    portENTER_CRITICAL(&s_hid_input_lock);
    s_hid_input_payload.wheel_button_state =
        DUAL_END_HID_WHEEL_BUTTON_STATE_VALID |
        (s_display_wheel_button_pressed
             ? DUAL_END_HID_WHEEL_BUTTON_PRESSED
             : 0);
    portEXIT_CRITICAL(&s_hid_input_lock);
    ESP_RETURN_ON_ERROR(user_keys_init(), TAG, "user keys init");
    ESP_RETURN_ON_ERROR(touchpad_module_init(), TAG, "touchpad init");

    esp_err_t imu_ret = imu_module_init();
    bool imu_ready = imu_ret == ESP_OK;
    if (!imu_ready) {
        ESP_LOGW(TAG, "IMU disabled: init failed (%s), transmitter will continue", esp_err_to_name(imu_ret));
    }
    if (imu_ready && SCEPTER_IMU_SLEEP_ENABLE) {
        light_sleep_module_config_t sleep_config = {
            .flat_sleep_ms = SCEPTER_IMU_SLEEP_STILL_MS,
            .set_tx_enabled = scepter_hardware_set_tx_enabled,
            .set_comm_recovering = scepter_hardware_set_comm_recovering,
            .set_mic_enabled = NULL,
            .recover_comm = scepter_hardware_recover_comm,
            .send_hello = scepter_hardware_send_hello,
            .prepare_wakeup = scepter_hardware_prepare_wakeup_ios,
            .wakeup_event = scepter_hardware_wakeup_event_cb,
            .before_sleep = scepter_hardware_sleep_prepare_cb,
            .on_wake = scepter_hardware_sleep_wake_cb,
        };
        esp_err_t sleep_ret = light_sleep_module_init(&sleep_config);
        s_light_sleep_ready = sleep_ret == ESP_OK;
        if (!s_light_sleep_ready) {
            ESP_LOGW(TAG, "light sleep disabled: init failed (%s)", esp_err_to_name(sleep_ret));
        }
    }

    ESP_RETURN_ON_ERROR(mic_audio_init(), TAG, "mic audio init");
    ESP_RETURN_ON_ERROR(motor_vibration_init(), TAG, "motor init");
    ESP_RETURN_ON_ERROR(rgb_led_init(), TAG, "rgb led init");

#if SCEPTER_HW_TEST_TASK_ENABLE
    BaseType_t hw_test_task_ok = xTaskCreate(hardware_test_task,
                                             "hw_test",
                                             SCEPTER_HW_TEST_TASK_STACK_BYTES,
                                             NULL,
                                             SCEPTER_HW_TEST_TASK_PRIORITY,
                                             NULL);
    ESP_RETURN_ON_FALSE(hw_test_task_ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create hardware test task");
#endif

    ESP_RETURN_ON_ERROR(wheel_module_start(), TAG, "wheel start");
    ESP_RETURN_ON_ERROR(user_keys_start(), TAG, "user keys start");
    ESP_RETURN_ON_ERROR(touchpad_module_start(), TAG, "touchpad start");
    if (imu_ready) {
        ESP_RETURN_ON_ERROR(imu_module_start(), TAG, "imu start");
    }

    lcd_display_start();
    ESP_RETURN_ON_ERROR(battery_monitor_start(), TAG, "battery monitor start");
    ESP_RETURN_ON_ERROR(mic_audio_start(), TAG, "mic audio start");

    ESP_LOGI(TAG, "scepter real hardware started; audio source=I2S, codec=G722 stereo, air packet=400B");
    return ESP_OK;
}

esp_err_t scepter_hardware_vibrate(uint32_t duration_ms)
{
    return motor_vibration_pulse(duration_ms);
}

esp_err_t scepter_hardware_set_mic_fixed_gain(uint8_t gain_x, bool persist)
{
    return mic_audio_set_output_gain(gain_x, persist);
}

esp_err_t scepter_hardware_play_ui_animation(uint8_t command)
{
    switch (command) {
    case DUAL_END_CONTROL_CMD_UI_THINKING:
        lcd_display_play_thinking_once();
        break;
    case DUAL_END_CONTROL_CMD_UI_APPROVE:
        lcd_display_play_approve_once();
        break;
    case DUAL_END_CONTROL_CMD_UI_DONE:
        lcd_display_play_done_once();
        break;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "remote UI animation command=0x%02x", command);
    return ESP_OK;
}

void scepter_hardware_set_connected(bool connected)
{
    if (s_link_connected == connected) {
        return;
    }

    s_link_connected = connected;
    if (connected) {
        s_link_wait_drops = 0;
    }
    scepter_hardware_update_lcd();
    ESP_LOGI(TAG, "link display %s", connected ? "connected" : "waiting");
}


void scepter_hardware_set_ota_active(bool active)
{
    if (s_ota_active == active) {
        return;
    }

    s_ota_active = active;
    if (active) {
        scepter_cancel_sleep_countdown();
    }
    lcd_display_set_ota_active(active);
    scepter_hardware_update_lcd();
    ESP_LOGI(TAG, "OTA transfer %s: audio/HID tx %s",
             active ? "started" : "stopped",
             active ? "paused" : "resumed");
}
