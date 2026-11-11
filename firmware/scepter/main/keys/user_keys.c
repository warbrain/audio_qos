#include "user_keys.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gpio_isr_service.h"
#include "sdkconfig.h"

static const char *TAG = "user_keys";

#if USER_KEYS_LOG_ENABLE
#define USER_KEYS_LOGI(...) ESP_LOGI(__VA_ARGS__)
#else
#define USER_KEYS_LOGI(...) do { } while (0)
#endif

#define USER_KEYS_QUEUE_SIZE 16
#define USER_KEYS_TASK_STACK_BYTES 4096

#ifndef CONFIG_USER_KEY1_GPIO
#define CONFIG_USER_KEY1_GPIO 9
#endif

#ifndef CONFIG_USER_KEY2_GPIO
#define CONFIG_USER_KEY2_GPIO 10
#endif

#ifndef CONFIG_USER_KEY3_GPIO
#define CONFIG_USER_KEY3_GPIO 0
#endif

#ifndef CONFIG_USER_KEYS_DEBOUNCE_MS
#define CONFIG_USER_KEYS_DEBOUNCE_MS 20
#endif

#ifndef CONFIG_USER_KEYS_REPEAT_MS
#define CONFIG_USER_KEYS_REPEAT_MS 100
#endif

typedef struct {
    user_key_id_t key_id;
} user_key_isr_event_t;

static const gpio_num_t s_key_gpios[USER_KEYS_COUNT] = {
    CONFIG_USER_KEY1_GPIO,
    CONFIG_USER_KEY2_GPIO,
    CONFIG_USER_KEY3_GPIO,
};

static QueueHandle_t s_key_isr_queue;
static user_keys_pressed_cb_t s_pressed_callback;
static void *s_pressed_user_ctx;
static user_keys_state_cb_t s_state_callback;
static void *s_state_user_ctx;

static bool user_key_is_pressed(user_key_id_t key_id)
{
    return gpio_get_level(s_key_gpios[key_id]) == 0;
}

static void user_keys_report_pressed(user_key_id_t key_id)
{
    if (s_pressed_callback) {
        s_pressed_callback(key_id, s_pressed_user_ctx);
    }
}

static void user_keys_report_state(user_key_id_t key_id, bool pressed)
{
    if (s_state_callback) {
        s_state_callback(key_id, pressed, s_state_user_ctx);
    }
}

static bool user_keys_array_any_pressed(const bool pressed[USER_KEYS_COUNT])
{
    for (int i = 0; i < USER_KEYS_COUNT; i++) {
        if (pressed[i]) {
            return true;
        }
    }
    return false;
}

static bool user_keys_tick_reached(TickType_t now, TickType_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static TickType_t user_keys_ticks_until(TickType_t now, TickType_t deadline)
{
    return user_keys_tick_reached(now, deadline) ? 0 : deadline - now;
}

static void user_keys_report_all_pressed(const bool pressed[USER_KEYS_COUNT])
{
    for (int i = 0; i < USER_KEYS_COUNT; i++) {
        if (pressed[i]) {
            user_keys_report_pressed((user_key_id_t)i);
        }
    }
}

static void IRAM_ATTR user_key_isr_handler(void *arg)
{
    user_key_isr_event_t event = {
        .key_id = (user_key_id_t)(uintptr_t)arg,
    };
    BaseType_t high_task_woken = pdFALSE;

    if (s_key_isr_queue) {
        (void)xQueueSendFromISR(s_key_isr_queue, &event, &high_task_woken);
        if (high_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static void user_keys_task(void *arg)
{
    (void)arg;

    bool stable_pressed[USER_KEYS_COUNT] = {0};
    bool candidate_pressed[USER_KEYS_COUNT] = {0};
    bool debounce_pending[USER_KEYS_COUNT] = {0};
    TickType_t debounce_deadline[USER_KEYS_COUNT] = {0};
    user_key_isr_event_t event = {0};
    TickType_t next_repeat_tick = 0;

    for (int i = 0; i < USER_KEYS_COUNT; i++) {
        stable_pressed[i] = user_key_is_pressed((user_key_id_t)i);
        candidate_pressed[i] = stable_pressed[i];
    }
    if (user_keys_array_any_pressed(stable_pressed)) {
        next_repeat_tick = xTaskGetTickCount() + pdMS_TO_TICKS(CONFIG_USER_KEYS_REPEAT_MS);
    }

    while (1) {
        TickType_t now = xTaskGetTickCount();

        for (int i = 0; i < USER_KEYS_COUNT; i++) {
            if (!debounce_pending[i] ||
                !user_keys_tick_reached(now, debounce_deadline[i])) {
                continue;
            }

            bool sampled_pressed = user_key_is_pressed((user_key_id_t)i);
            if (sampled_pressed != candidate_pressed[i]) {
                candidate_pressed[i] = sampled_pressed;
                debounce_deadline[i] =
                    now + pdMS_TO_TICKS(CONFIG_USER_KEYS_DEBOUNCE_MS);
                continue;
            }

            debounce_pending[i] = false;
            if (candidate_pressed[i] == stable_pressed[i]) {
                continue;
            }

            bool had_pressed_key = user_keys_array_any_pressed(stable_pressed);
            stable_pressed[i] = candidate_pressed[i];
            user_keys_report_state((user_key_id_t)i, stable_pressed[i]);
            if (stable_pressed[i]) {
                USER_KEYS_LOGI(TAG,
                               "pressed: key%d levels=[%d,%d,%d]",
                               i + 1,
                               gpio_get_level(s_key_gpios[0]),
                               gpio_get_level(s_key_gpios[1]),
                               gpio_get_level(s_key_gpios[2]));
                user_keys_report_pressed((user_key_id_t)i);
                if (!had_pressed_key) {
                    next_repeat_tick =
                        now + pdMS_TO_TICKS(CONFIG_USER_KEYS_REPEAT_MS);
                }
            }
        }

        bool any_pressed = user_keys_array_any_pressed(stable_pressed);
        if (any_pressed && user_keys_tick_reached(now, next_repeat_tick)) {
            user_keys_report_all_pressed(stable_pressed);
            next_repeat_tick = now + pdMS_TO_TICKS(CONFIG_USER_KEYS_REPEAT_MS);
        }

        TickType_t wait_ticks = portMAX_DELAY;
        now = xTaskGetTickCount();
        for (int i = 0; i < USER_KEYS_COUNT; i++) {
            if (!debounce_pending[i]) {
                continue;
            }
            TickType_t remaining =
                user_keys_ticks_until(now, debounce_deadline[i]);
            if (wait_ticks == portMAX_DELAY || remaining < wait_ticks) {
                wait_ticks = remaining;
            }
        }
        if (any_pressed) {
            TickType_t repeat_remaining =
                user_keys_ticks_until(now, next_repeat_tick);
            if (wait_ticks == portMAX_DELAY || repeat_remaining < wait_ticks) {
                wait_ticks = repeat_remaining;
            }
        }

        if (xQueueReceive(s_key_isr_queue, &event, wait_ticks) == pdTRUE) {
            do {
                user_key_id_t key_id = event.key_id;
                if (key_id >= USER_KEYS_COUNT) {
                    continue;
                }
                bool sampled_pressed = user_key_is_pressed(key_id);
                if (sampled_pressed != candidate_pressed[key_id]) {
                    candidate_pressed[key_id] = sampled_pressed;
                    debounce_pending[key_id] = true;
                    debounce_deadline[key_id] =
                        xTaskGetTickCount() +
                        pdMS_TO_TICKS(CONFIG_USER_KEYS_DEBOUNCE_MS);
                }
            } while (xQueueReceive(s_key_isr_queue, &event, 0) == pdTRUE);
        }
    }
}

esp_err_t user_keys_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONFIG_USER_KEY1_GPIO)
                      | (1ULL << CONFIG_USER_KEY2_GPIO)
                      | (1ULL << CONFIG_USER_KEY3_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "key gpio config failed");

    s_key_isr_queue = xQueueCreate(USER_KEYS_QUEUE_SIZE, sizeof(user_key_isr_event_t));
    ESP_RETURN_ON_FALSE(s_key_isr_queue, ESP_ERR_NO_MEM, TAG, "create isr queue failed");

    USER_KEYS_LOGI(TAG, "init OK: key1=%d key2=%d key3=%d active=low",
                   CONFIG_USER_KEY1_GPIO,
                   CONFIG_USER_KEY2_GPIO,
                   CONFIG_USER_KEY3_GPIO);
    return ESP_OK;
}

void user_keys_set_pressed_callback(user_keys_pressed_cb_t callback, void *user_ctx)
{
    s_pressed_callback = callback;
    s_pressed_user_ctx = user_ctx;
}

void user_keys_set_state_callback(user_keys_state_cb_t callback, void *user_ctx)
{
    s_state_callback = callback;
    s_state_user_ctx = user_ctx;
}

bool user_keys_any_pressed(void)
{
    for (size_t i = 0; i < USER_KEYS_COUNT; i++) {
        if (user_key_is_pressed((user_key_id_t)i)) {
            return true;
        }
    }
    return false;
}

esp_err_t user_keys_start(void)
{
    ESP_RETURN_ON_FALSE(s_key_isr_queue, ESP_ERR_INVALID_STATE, TAG, "user keys are not initialized");

    ESP_RETURN_ON_ERROR(gpio_isr_service_install_once(), TAG, "install isr service failed");

    for (int i = 0; i < USER_KEYS_COUNT; i++) {
        ESP_RETURN_ON_ERROR(gpio_isr_handler_add(s_key_gpios[i],
                                                 user_key_isr_handler,
                                                 (void *)(uintptr_t)i),
                            TAG,
                            "add key isr failed");
    }

    BaseType_t ok = xTaskCreate(user_keys_task, "user_keys", USER_KEYS_TASK_STACK_BYTES, NULL, 6, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create key task failed");
    return ESP_OK;
}

size_t user_keys_get_gpios(gpio_num_t *gpios, size_t capacity)
{
    if (!gpios || capacity < USER_KEYS_COUNT) {
        return 0;
    }
    for (size_t i = 0; i < USER_KEYS_COUNT; i++) {
        gpios[i] = s_key_gpios[i];
    }
    return USER_KEYS_COUNT;
}

uint8_t user_keys_get_ids_for_gpio_mask(uint64_t gpio_mask)
{
    uint8_t key_ids = 0;
    for (size_t i = 0; i < USER_KEYS_COUNT; i++) {
        if (gpio_mask & (1ULL << s_key_gpios[i])) {
            key_ids |= (uint8_t)(1U << i);
        }
    }
    return key_ids;
}

esp_err_t user_keys_notify_wakeup(uint64_t gpio_mask)
{
    ESP_RETURN_ON_FALSE(s_key_isr_queue, ESP_ERR_INVALID_STATE, TAG, "key queue is not ready");

    for (size_t i = 0; i < USER_KEYS_COUNT; i++) {
        if ((gpio_mask & (1ULL << s_key_gpios[i])) == 0) {
            continue;
        }
        user_key_isr_event_t event = {
            .key_id = (user_key_id_t)i,
        };
        ESP_RETURN_ON_FALSE(xQueueSend(s_key_isr_queue, &event, 0) == pdTRUE,
                            ESP_ERR_TIMEOUT,
                            TAG,
                            "queue wake key failed");
    }
    return ESP_OK;
}
