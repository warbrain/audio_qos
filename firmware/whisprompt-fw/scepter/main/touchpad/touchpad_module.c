#include "touchpad_module.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_isr_service.h"
#include "sdkconfig.h"

static const char *TAG = "touchpad";

#define TOUCH_READ_LEN           5
#define TOUCH_FRAME_HEADER       0x54
#define TOUCH_DEBUG_PRINT_MS     1000
#define TOUCH_RAW_TRACE          0
#define TOUCH_RAW_TRACE_MS       200
#define TOUCH_READ_RETRY_COUNT   5
#define TOUCH_READ_RETRY_DELAY_MS 1
#define TOUCH_USE_INT            1
#define TOUCH_INT_READ_DELAY_MS  0
#ifndef TOUCH_STATS_LOG_ENABLED
#define TOUCH_STATS_LOG_ENABLED  1
#endif
#ifndef TOUCH_STATS_LOG_MS
#define TOUCH_STATS_LOG_MS       1000
#endif
#define TOUCH_GESTURE_ZOOM_IN          0x03
#define TOUCH_GESTURE_ZOOM_OUT         0x04
#define TOUCH_GESTURE_VERTICAL_WHEEL   0x05
#define TOUCH_GESTURE_HORIZONTAL_WHEEL 0x06
#define TOUCH_BUTTON_FLAG_ZOOM_IN      0x10
#define TOUCH_BUTTON_FLAG_ZOOM_OUT     0x20
#define MS_TO_TICKS_MIN_1(ms) (pdMS_TO_TICKS(ms) > 0 ? pdMS_TO_TICKS(ms) : 1)

#ifndef CONFIG_TOUCH_AXIS_LOCK_RATIO
#define CONFIG_TOUCH_AXIS_LOCK_RATIO 0
#endif

#ifndef CONFIG_TOUCH_SDA_GPIO
#define CONFIG_TOUCH_SDA_GPIO 8
#endif

#ifndef CONFIG_TOUCH_SCL_GPIO
#define CONFIG_TOUCH_SCL_GPIO 18
#endif

#ifndef CONFIG_TOUCH_INT_GPIO
#define CONFIG_TOUCH_INT_GPIO 17
#endif

#ifndef CONFIG_TOUCH_I2C_ADDR
#define CONFIG_TOUCH_I2C_ADDR 0x10
#endif

#ifndef CONFIG_TOUCH_I2C_FREQ_HZ
#define CONFIG_TOUCH_I2C_FREQ_HZ 50000
#endif

#ifndef CONFIG_TOUCH_I2C_TIMEOUT_MS
#define CONFIG_TOUCH_I2C_TIMEOUT_MS 20
#endif

#ifndef CONFIG_TOUCH_READ_INTERVAL_MS
#define CONFIG_TOUCH_READ_INTERVAL_MS 10
#endif

#ifndef CONFIG_TOUCH_MOUSE_GAIN
#define CONFIG_TOUCH_MOUSE_GAIN 280
#endif

#ifndef CONFIG_TOUCH_FINE_GAIN
#define CONFIG_TOUCH_FINE_GAIN 160
#endif

#ifndef CONFIG_TOUCH_MEDIUM_GAIN
#define CONFIG_TOUCH_MEDIUM_GAIN 200
#endif

#ifndef CONFIG_TOUCH_FINE_THRESHOLD
#define CONFIG_TOUCH_FINE_THRESHOLD 1
#endif

#ifndef CONFIG_TOUCH_MEDIUM_THRESHOLD
#define CONFIG_TOUCH_MEDIUM_THRESHOLD 6
#endif

#ifndef CONFIG_TOUCH_FAST_THRESHOLD
#define CONFIG_TOUCH_FAST_THRESHOLD 12
#endif

#ifndef CONFIG_TOUCH_SMOOTH_ALPHA
#define CONFIG_TOUCH_SMOOTH_ALPHA 100
#endif

#ifndef CONFIG_TOUCH_DEADZONE
#define CONFIG_TOUCH_DEADZONE 0
#endif

#ifndef CONFIG_TOUCH_MAX_STEP
#define CONFIG_TOUCH_MAX_STEP 0
#endif

#ifndef CONFIG_TOUCH_SWAP_XY
#define CONFIG_TOUCH_SWAP_XY 1
#endif

#ifndef CONFIG_TOUCH_INVERT_X
#define CONFIG_TOUCH_INVERT_X 0
#endif

#ifndef CONFIG_TOUCH_INVERT_Y
#define CONFIG_TOUCH_INVERT_Y 1
#endif

static float s_touch_smooth_dx;
static float s_touch_smooth_dy;
static float s_touch_rem_x;
static float s_touch_rem_y;
static uint32_t s_touch_frame_count;
#if TOUCH_STATS_LOG_ENABLED
static uint32_t s_touch_valid_frame_count;
static uint32_t s_touch_process_count;
static uint32_t s_touch_motion_event_count;
static uint32_t s_touch_no_header_count;
static uint32_t s_touch_coalesced_irq_count;
static uint32_t s_touch_max_irq_batch;
#endif
static uint32_t s_touch_fail_count;
static int64_t s_touch_last_log_us;
static touchpad_motion_cb_t s_motion_callback;
static void *s_motion_user_ctx;
static i2c_master_bus_handle_t s_touch_i2c_bus;
static i2c_master_dev_handle_t s_touch_i2c_dev;
static volatile uint8_t s_touch_last_buttons;
static TaskHandle_t s_touch_task_handle;
static volatile uint32_t s_touch_irq_count;
#if TOUCH_STATS_LOG_ENABLED
static int64_t s_touch_last_stats_us;
static uint32_t s_touch_prev_irq_count;
static uint32_t s_touch_prev_frame_count;
static uint32_t s_touch_prev_valid_frame_count;
static uint32_t s_touch_prev_process_count;
static uint32_t s_touch_prev_motion_event_count;
static uint32_t s_touch_prev_no_header_count;
static uint32_t s_touch_prev_coalesced_irq_count;
static uint32_t s_touch_prev_fail_count;
#endif

#if TOUCH_RAW_TRACE
static int64_t s_touch_last_raw_trace_us;
#endif

static int abs_i32(int value)
{
    return value >= 0 ? value : -value;
}

static int round_float_to_int(float value)
{
    return value >= 0.0f ? (int)(value + 0.5f) : (int)(value - 0.5f);
}

static float interpolate_gain_percent(int delta,
                                      int start_delta,
                                      int end_delta,
                                      int start_gain,
                                      int end_gain)
{
    if (end_delta <= start_delta) {
        return (float)end_gain;
    }

    float position = (float)(delta - start_delta) /
                     (float)(end_delta - start_delta);
    return (float)start_gain + position * (float)(end_gain - start_gain);
}

static float touch_gain_percent_for_delta(int delta)
{
    if (delta <= CONFIG_TOUCH_FINE_THRESHOLD) {
        return (float)CONFIG_TOUCH_FINE_GAIN;
    }
    if (delta <= CONFIG_TOUCH_MEDIUM_THRESHOLD) {
        return interpolate_gain_percent(delta,
                                        CONFIG_TOUCH_FINE_THRESHOLD,
                                        CONFIG_TOUCH_MEDIUM_THRESHOLD,
                                        CONFIG_TOUCH_FINE_GAIN,
                                        CONFIG_TOUCH_MEDIUM_GAIN);
    }
    if (delta < CONFIG_TOUCH_FAST_THRESHOLD) {
        return interpolate_gain_percent(delta,
                                        CONFIG_TOUCH_MEDIUM_THRESHOLD,
                                        CONFIG_TOUCH_FAST_THRESHOLD,
                                        CONFIG_TOUCH_MEDIUM_GAIN,
                                        CONFIG_TOUCH_MOUSE_GAIN);
    }
    return (float)CONFIG_TOUCH_MOUSE_GAIN;
}

#if TOUCH_STATS_LOG_ENABLED
static void touch_log_stats_if_due(void)
{
    int64_t now_us = esp_timer_get_time();
    if (s_touch_last_stats_us == 0) {
        /* Emit the first diagnostic sample immediately; later samples use the
         * normal one-second window while touch interrupts continue arriving. */
        s_touch_last_stats_us = now_us - TOUCH_STATS_LOG_MS * 1000LL;
    }
    if ((now_us - s_touch_last_stats_us) < TOUCH_STATS_LOG_MS * 1000LL) {
        return;
    }

    uint32_t irq = s_touch_irq_count - s_touch_prev_irq_count;
    uint32_t reads = s_touch_frame_count - s_touch_prev_frame_count;
    uint32_t valid = s_touch_valid_frame_count - s_touch_prev_valid_frame_count;
    uint32_t processed = s_touch_process_count - s_touch_prev_process_count;
    uint32_t motion = s_touch_motion_event_count - s_touch_prev_motion_event_count;
    uint32_t no_header = s_touch_no_header_count - s_touch_prev_no_header_count;
    uint32_t coalesced = s_touch_coalesced_irq_count - s_touch_prev_coalesced_irq_count;
    uint32_t failures = s_touch_fail_count - s_touch_prev_fail_count;

    if (irq > 0 || reads > 0 || processed > 0 || failures > 0) {
        ESP_LOGI(TAG,
                 "stats irq=%" PRIu32 " process=%" PRIu32
                 " reads=%" PRIu32 " valid=%" PRIu32
                 " motion=%" PRIu32 " fail=%" PRIu32
                 " no_header=%" PRIu32 " coalesced=%" PRIu32
                 " max_batch=%" PRIu32 " i2c=%dHz",
                 irq,
                 processed,
                 reads,
                 valid,
                 motion,
                 failures,
                 no_header,
                 coalesced,
                 s_touch_max_irq_batch,
                 CONFIG_TOUCH_I2C_FREQ_HZ);
    }

    s_touch_last_stats_us = now_us;
    s_touch_prev_irq_count = s_touch_irq_count;
    s_touch_prev_frame_count = s_touch_frame_count;
    s_touch_prev_valid_frame_count = s_touch_valid_frame_count;
    s_touch_prev_process_count = s_touch_process_count;
    s_touch_prev_motion_event_count = s_touch_motion_event_count;
    s_touch_prev_no_header_count = s_touch_no_header_count;
    s_touch_prev_coalesced_irq_count = s_touch_coalesced_irq_count;
    s_touch_prev_fail_count = s_touch_fail_count;
}
#endif

static void IRAM_ATTR touch_int_isr(void *arg)
{
    (void)arg;
    s_touch_irq_count++;
    BaseType_t high_task_woken = pdFALSE;
    if (s_touch_task_handle) {
        vTaskNotifyGiveFromISR(s_touch_task_handle, &high_task_woken);
    }
    if (high_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static int limit_step(int value, int max_abs)
{
    if (value > max_abs) {
        return max_abs;
    }
    if (value < -max_abs) {
        return -max_abs;
    }
    return value;
}

static int apply_deadzone(int value, int deadzone)
{
    if (deadzone > 0 && value >= -deadzone && value <= deadzone) {
        return 0;
    }
    return value;
}

static int16_t clamp_i16(int value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static bool touch_read_raw(uint8_t *buf, uint8_t len)
{
    if (!buf || len == 0) {
        return false;
    }

    memset(buf, 0, len);
    if (!s_touch_i2c_dev) {
        return false;
    }

    return i2c_master_receive(s_touch_i2c_dev,
                              buf,
                              len,
                              CONFIG_TOUCH_I2C_TIMEOUT_MS) == ESP_OK;
}

#if TOUCH_RAW_TRACE
static void touch_trace_raw_frame(const uint8_t *buf, bool valid, uint8_t buttons, int8_t dx_raw, int8_t dy_raw)
{
    int64_t now_us = esp_timer_get_time();
    if (now_us - s_touch_last_raw_trace_us >= TOUCH_RAW_TRACE_MS * 1000LL) {
        s_touch_last_raw_trace_us = now_us;
        ESP_LOGI(TAG,
                 "raw %02x %02x %02x %02x %02x valid=%d buttons=0x%02x dx_raw=%d dy_raw=%d int=%d irq=%" PRIu32,
                 buf[0],
                 buf[1],
                 buf[2],
                 buf[3],
                 buf[4],
                 valid ? 1 : 0,
                 buttons,
                 dx_raw,
                 dy_raw,
                 gpio_get_level(CONFIG_TOUCH_INT_GPIO),
                 s_touch_irq_count);
    }
}
#else
#define touch_trace_raw_frame(buf, valid, buttons, dx_raw, dy_raw) do { } while (0)
#endif

static bool touch_read_motion(int16_t *dx_out,
                              int16_t *dy_out,
                              uint8_t *buttons_out,
                              int8_t *vertical_out,
                              int8_t *horizontal_out)
{
    uint8_t buf[TOUCH_READ_LEN] = {0};
    if (dx_out) {
        *dx_out = 0;
    }
    if (dy_out) {
        *dy_out = 0;
    }
    if (buttons_out) {
        *buttons_out = 0;
    }
    if (vertical_out) {
        *vertical_out = 0;
    }
    if (horizontal_out) {
        *horizontal_out = 0;
    }

    bool found = false;
    for (int attempt = 0; attempt < TOUCH_READ_RETRY_COUNT && !found; attempt++) {
        if (!touch_read_raw(buf, TOUCH_READ_LEN)) {
            s_touch_fail_count++;
            int64_t now_us = esp_timer_get_time();
            if (now_us - s_touch_last_log_us >= 500000) {
                s_touch_last_log_us = now_us;
                ESP_LOGD(TAG, "touch read failed, fail=%" PRIu32, s_touch_fail_count);
            }
            return false;
        }

        s_touch_frame_count++;
        found = buf[0] == TOUCH_FRAME_HEADER;
        if (!found && attempt + 1 < TOUCH_READ_RETRY_COUNT) {
            vTaskDelay(MS_TO_TICKS_MIN_1(TOUCH_READ_RETRY_DELAY_MS));
        }
    }

    if (!found) {
#if TOUCH_STATS_LOG_ENABLED
        s_touch_no_header_count++;
#endif
        touch_trace_raw_frame(buf, false, 0, 0, 0);
        int64_t now_us = esp_timer_get_time();
        if (now_us - s_touch_last_log_us >= TOUCH_DEBUG_PRINT_MS * 1000LL) {
            s_touch_last_log_us = now_us;
            ESP_LOGD(TAG,
                     "touch no header: %02x %02x %02x %02x %02x",
                     buf[0], buf[1], buf[2], buf[3], buf[4]);
        }
        return false;
    }
#if TOUCH_STATS_LOG_ENABLED
    s_touch_valid_frame_count++;
#endif

    uint8_t buttons = buf[1] & 0x03;
    int8_t dx_raw = (int8_t)buf[2];
    int8_t dy_raw = (int8_t)buf[3];
    uint8_t gesture = (buf[4] >> 4) & 0x0f;
    uint8_t z_delta = buf[4] & 0x07;
    int8_t z = (buf[4] & 0x08) ? -(int8_t)z_delta : (int8_t)z_delta;
    int8_t vertical = 0;
    int8_t horizontal = 0;

    if (gesture == TOUCH_GESTURE_ZOOM_IN) {
        buttons |= TOUCH_BUTTON_FLAG_ZOOM_IN;
    } else if (gesture == TOUCH_GESTURE_ZOOM_OUT) {
        buttons |= TOUCH_BUTTON_FLAG_ZOOM_OUT;
    } else if (z_delta != 0) {
        if (gesture == TOUCH_GESTURE_VERTICAL_WHEEL) {
            horizontal = -z;
        } else if (gesture == TOUCH_GESTURE_HORIZONTAL_WHEEL) {
            vertical = z;
        }
    }
    touch_trace_raw_frame(buf, true, buttons, dx_raw, dy_raw);

    int delta_x = abs_i32(dx_raw);
    int delta_y = abs_i32(dy_raw);
    int delta = delta_x > delta_y ? delta_x : delta_y;
    float gain = touch_gain_percent_for_delta(delta) / 100.0f;
    float dx_f = (float)dx_raw * gain;
    float dy_f = (float)dy_raw * gain;

#if CONFIG_TOUCH_SWAP_XY
    float tmp_f = dx_f;
    dx_f = dy_f;
    dy_f = tmp_f;
#endif

#if CONFIG_TOUCH_INVERT_X
    dx_f = -dx_f;
#endif
#if CONFIG_TOUCH_INVERT_Y
    dy_f = -dy_f;
#endif

    float alpha = (float)CONFIG_TOUCH_SMOOTH_ALPHA / 100.0f;
    s_touch_smooth_dx =
        alpha * dx_f + (1.0f - alpha) * s_touch_smooth_dx;
    s_touch_smooth_dy =
        alpha * dy_f + (1.0f - alpha) * s_touch_smooth_dy;

    s_touch_rem_x += s_touch_smooth_dx;
    s_touch_rem_y += s_touch_smooth_dy;

    int dx = round_float_to_int(s_touch_rem_x);
    int dy = round_float_to_int(s_touch_rem_y);

    dx = apply_deadzone(dx, CONFIG_TOUCH_DEADZONE);
    dy = apply_deadzone(dy, CONFIG_TOUCH_DEADZONE);

    int abs_dx = abs_i32(dx);
    int abs_dy = abs_i32(dy);
    int axis_lock_ratio = CONFIG_TOUCH_AXIS_LOCK_RATIO;
    if (axis_lock_ratio > 0 && (abs_dx > 0 || abs_dy > 0)) {
        if ((abs_dx * 100) >= (abs_dy * axis_lock_ratio)) {
            dy = 0;
            s_touch_smooth_dy = 0.0f;
            s_touch_rem_y = 0.0f;
        } else if ((abs_dy * 100) >= (abs_dx * axis_lock_ratio)) {
            dx = 0;
            s_touch_smooth_dx = 0.0f;
            s_touch_rem_x = 0.0f;
        }
    }

    if (CONFIG_TOUCH_MAX_STEP > 0) {
        dx = limit_step(dx, CONFIG_TOUCH_MAX_STEP);
        dy = limit_step(dy, CONFIG_TOUCH_MAX_STEP);
    }

    s_touch_rem_x -= (float)dx;
    s_touch_rem_y -= (float)dy;

    if (dx_out) {
        *dx_out = clamp_i16(dx);
    }
    if (dy_out) {
        *dy_out = clamp_i16(dy);
    }
    if (buttons_out) {
        *buttons_out = buttons;
    }
    if (vertical_out) {
        *vertical_out = vertical;
    }
    if (horizontal_out) {
        *horizontal_out = horizontal;
    }

    return true;
}

static bool touch_process_one_frame(void)
{
    int16_t dx = 0;
    int16_t dy = 0;
    uint8_t buttons = 0;
    int8_t vertical = 0;
    int8_t horizontal = 0;

    bool read_ok = touch_read_motion(&dx, &dy, &buttons, &vertical, &horizontal);
    if (read_ok && s_motion_callback) {
        bool zoom_event = (buttons & (TOUCH_BUTTON_FLAG_ZOOM_IN | TOUCH_BUTTON_FLAG_ZOOM_OUT)) != 0;
        bool changed = dx != 0 || dy != 0 || vertical != 0 || horizontal != 0 ||
                       zoom_event || ((buttons & 0x03) != s_touch_last_buttons);
        if (changed) {
#if TOUCH_STATS_LOG_ENABLED
            s_touch_motion_event_count++;
#endif
            s_touch_last_buttons = buttons & 0x03;
            touchpad_motion_t motion = {
                .dx = dx,
                .dy = dy,
                .buttons = buttons,
                .vertical = vertical,
                .horizontal = horizontal,
            };
            s_motion_callback(&motion, s_motion_user_ctx);
        }
    } else if (!read_ok && s_touch_last_buttons != 0 && s_motion_callback) {
        s_touch_last_buttons = 0;
        touchpad_motion_t motion = {
            .dx = 0,
            .dy = 0,
            .buttons = 0,
        };
        s_motion_callback(&motion, s_motion_user_ctx);
    }
    return read_ok;
}

static void touch_task(void *arg)
{
    (void)arg;
    s_touch_task_handle = xTaskGetCurrentTaskHandle();

    while (1) {
#if TOUCH_USE_INT
        uint32_t irq_events = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#if TOUCH_STATS_LOG_ENABLED
        if (irq_events > s_touch_max_irq_batch) {
            s_touch_max_irq_batch = irq_events;
        }
        if (irq_events > 1) {
            s_touch_coalesced_irq_count += irq_events - 1;
        }
#endif
        if (irq_events > 0 && TOUCH_INT_READ_DELAY_MS > 0) {
            vTaskDelay(MS_TO_TICKS_MIN_1(TOUCH_INT_READ_DELAY_MS));
        }
#endif

#if TOUCH_STATS_LOG_ENABLED
        s_touch_process_count++;
#endif
        touch_process_one_frame();
#if TOUCH_STATS_LOG_ENABLED
        touch_log_stats_if_due();
#endif

#if !TOUCH_USE_INT
        vTaskDelay(MS_TO_TICKS_MIN_1(CONFIG_TOUCH_READ_INTERVAL_MS));
#endif
    }
}

esp_err_t touchpad_module_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = TOUCH_USE_INT ? GPIO_INTR_NEGEDGE : GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONFIG_TOUCH_INT_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "touch int gpio config failed");

#if TOUCH_USE_INT
    ESP_RETURN_ON_ERROR(gpio_isr_service_install_once(), TAG, "install touch gpio isr failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CONFIG_TOUCH_INT_GPIO, touch_int_isr, NULL),
                        TAG,
                        "add touch gpio isr failed");
#endif

    i2c_master_bus_config_t bus_conf = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_TOUCH_SDA_GPIO,
        .scl_io_num = CONFIG_TOUCH_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_conf, &s_touch_i2c_bus), TAG, "touch i2c bus failed");

    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_TOUCH_I2C_ADDR,
        .scl_speed_hz = CONFIG_TOUCH_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_touch_i2c_bus, &dev_conf, &s_touch_i2c_dev),
                        TAG,
                        "touch i2c device failed");

    ESP_LOGI(TAG, "I2C init OK: SDA=%d SCL=%d INT=%d addr=0x%02x freq=%d",
             CONFIG_TOUCH_SDA_GPIO,
             CONFIG_TOUCH_SCL_GPIO,
             CONFIG_TOUCH_INT_GPIO,
             CONFIG_TOUCH_I2C_ADDR,
             CONFIG_TOUCH_I2C_FREQ_HZ);
    return ESP_OK;
}

void touchpad_module_set_motion_callback(touchpad_motion_cb_t callback, void *user_ctx)
{
    s_motion_callback = callback;
    s_motion_user_ctx = user_ctx;
}

esp_err_t touchpad_module_start(void)
{
    BaseType_t ok = xTaskCreate(touch_task, "touch_task", 4096, NULL, 4, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create touch task failed");
    return ESP_OK;
}

gpio_num_t touchpad_module_get_interrupt_gpio(void)
{
    return CONFIG_TOUCH_INT_GPIO;
}

bool touchpad_module_is_active(void)
{
    /* A held touch button remains in s_touch_last_buttons. A low interrupt
     * means unread/pending touch activity and must also inhibit sleep. */
    return s_touch_last_buttons != 0 ||
           gpio_get_level(CONFIG_TOUCH_INT_GPIO) == 0;
}

esp_err_t touchpad_module_process_wakeup(void)
{
    ESP_RETURN_ON_FALSE(s_touch_i2c_dev, ESP_ERR_INVALID_STATE, TAG, "touchpad is not initialized");
    ESP_RETURN_ON_FALSE(touch_process_one_frame(), ESP_FAIL, TAG, "read wake touch frame failed");
    return ESP_OK;
}
