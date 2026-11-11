#include "wheel_module.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gpio_isr_service.h"
#include "sdkconfig.h"

static const char *TAG = "wheel";

#define WHEEL_ISR_QUEUE_SIZE 32
#define WHEEL_PCNT_SETTLE_MS 1

#ifndef CONFIG_WHEEL_BUTTON_DEBOUNCE_MS
#define CONFIG_WHEEL_BUTTON_DEBOUNCE_MS 20
#endif

#ifndef CONFIG_WHEEL_A_GPIO
#define CONFIG_WHEEL_A_GPIO 12
#endif

#ifndef CONFIG_WHEEL_B_GPIO
#define CONFIG_WHEEL_B_GPIO 13
#endif

#ifndef CONFIG_WHEEL_BUTTON_GPIO
#define CONFIG_WHEEL_BUTTON_GPIO 15
#endif

#ifndef CONFIG_WHEEL_DETENT_STEPS
/* This encoder produces one complete four-edge quadrature cycle per
 * mechanical detent. Report only after the full cycle so one physical
 * wheel step maps to one HID scroll step. */
#define CONFIG_WHEEL_DETENT_STEPS 4
#endif

#ifndef CONFIG_WHEEL_REVERSE
#define CONFIG_WHEEL_REVERSE 0
#endif

typedef enum {
    WHEEL_ISR_SOURCE_ENCODER,
    WHEEL_ISR_SOURCE_BUTTON,
    WHEEL_ISR_SOURCE_WAKE_ENCODER,
    WHEEL_ISR_SOURCE_WAKE_BUTTON,
} wheel_isr_source_t;

typedef struct {
    wheel_isr_source_t source;
    uint8_t input_state;
    int8_t wake_direction;
} wheel_isr_event_t;

static QueueHandle_t s_wheel_isr_queue;
static int32_t s_wheel_step_accum;
static pcnt_unit_handle_t s_wheel_pcnt_unit;
static int s_wheel_pcnt_last_count;
static uint8_t s_wheel_wakeup_reference_state;
static volatile bool s_wheel_button_pressed;
static volatile uint32_t s_wheel_isr_queue_drops;
static wheel_module_report_cb_t s_report_callback;
static void *s_report_user_ctx;

static uint8_t wheel_read_state(void)
{
    uint8_t a = (uint8_t)gpio_get_level(CONFIG_WHEEL_A_GPIO);
    uint8_t b = (uint8_t)gpio_get_level(CONFIG_WHEEL_B_GPIO);
    return (uint8_t)((a << 1) | b);
}

static void wheel_report_state(wheel_module_state_t state)
{
    if (s_report_callback) {
        s_report_callback(state, s_report_user_ctx);
    }
}

static int8_t wheel_transition_delta(uint8_t previous_state, uint8_t state)
{
    static const int8_t transition_table[16] = {
        0, -1,  1,  0,
        1,  0,  0, -1,
       -1,  0,  0,  1,
        0,  1, -1,  0,
    };

    uint8_t index = (uint8_t)(((previous_state & 0x03) << 2) |
                              (state & 0x03));
    return transition_table[index & 0x0f];
}

static esp_err_t wheel_pcnt_init(void)
{
    pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &s_wheel_pcnt_unit),
                        TAG,
                        "create PCNT unit failed");

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_wheel_pcnt_unit,
                                                     &filter_config),
                        TAG,
                        "configure PCNT glitch filter failed");

    pcnt_chan_config_t channel_config = {
        .edge_gpio_num = CONFIG_WHEEL_A_GPIO,
        .level_gpio_num = CONFIG_WHEEL_B_GPIO,
    };
    pcnt_channel_handle_t channel_a = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_wheel_pcnt_unit,
                                         &channel_config,
                                         &channel_a),
                        TAG,
                        "create PCNT A channel failed");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(channel_a,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE),
        TAG,
        "configure PCNT A edge action failed");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(channel_a,
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG,
        "configure PCNT A level action failed");

    channel_config.edge_gpio_num = CONFIG_WHEEL_B_GPIO;
    channel_config.level_gpio_num = CONFIG_WHEEL_A_GPIO;
    pcnt_channel_handle_t channel_b = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_wheel_pcnt_unit,
                                         &channel_config,
                                         &channel_b),
                        TAG,
                        "create PCNT B channel failed");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(channel_b,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG,
        "configure PCNT B edge action failed");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(channel_b,
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG,
        "configure PCNT B level action failed");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_wheel_pcnt_unit),
                        TAG,
                        "enable PCNT unit failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_wheel_pcnt_unit),
                        TAG,
                        "clear PCNT unit failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_wheel_pcnt_unit),
                        TAG,
                        "start PCNT unit failed");
    s_wheel_pcnt_last_count = 0;
    return ESP_OK;
}

static void wheel_pcnt_sync(bool discard)
{
    int count = 0;
    esp_err_t ret = pcnt_unit_get_count(s_wheel_pcnt_unit, &count);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "read PCNT count failed: %s", esp_err_to_name(ret));
        return;
    }

    int delta = count - s_wheel_pcnt_last_count;
    s_wheel_pcnt_last_count = count;
    if (!discard && delta != 0) {
        /* The PCNT quadrature configuration counts in the opposite sign to
         * the legacy state table. Negate it to preserve scroll direction. */
        s_wheel_step_accum -= delta;
    }
}

static int32_t wheel_take_detents(void)
{
    int32_t detents = s_wheel_step_accum / CONFIG_WHEEL_DETENT_STEPS;

    if (detents != 0) {
        s_wheel_step_accum -= detents * CONFIG_WHEEL_DETENT_STEPS;
    }
#if CONFIG_WHEEL_REVERSE
    detents = -detents;
#endif

    return detents;
}

static void wheel_report_pending_detents(void)
{
    int32_t detents = wheel_take_detents();
    while (detents > 0) {
        wheel_report_state(WHEEL_MODULE_STATE_SCROLL_UP);
        detents--;
    }
    while (detents < 0) {
        wheel_report_state(WHEEL_MODULE_STATE_SCROLL_DOWN);
        detents++;
    }
}

static void wheel_process_pcnt_count(bool *suppress_wake_cycle)
{
    wheel_pcnt_sync(*suppress_wake_cycle);
    if (*suppress_wake_cycle) {
        if (wheel_read_state() == s_wheel_wakeup_reference_state) {
            *suppress_wake_cycle = false;
            s_wheel_step_accum = 0;
        }
        return;
    }
    wheel_report_pending_detents();
}

static bool wheel_tick_reached(TickType_t now, TickType_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static TickType_t wheel_ticks_until(TickType_t now, TickType_t deadline)
{
    return wheel_tick_reached(now, deadline) ? 0 : deadline - now;
}

static void IRAM_ATTR wheel_encoder_isr_handler(void *arg)
{
    (void)arg;
    wheel_isr_event_t event = {
        .source = WHEEL_ISR_SOURCE_ENCODER,
        /* PCNT preserves the count; the phase snapshot is only used to track
         * the end of a wake-up cycle without delaying the ISR. */
        .input_state = wheel_read_state(),
    };
    BaseType_t high_task_woken = pdFALSE;

    if (s_wheel_isr_queue) {
        if (xQueueSendFromISR(s_wheel_isr_queue, &event, &high_task_woken) != pdTRUE) {
            s_wheel_isr_queue_drops++;
        }
        if (high_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static void IRAM_ATTR wheel_button_isr_handler(void *arg)
{
    (void)arg;
    wheel_isr_event_t event = {
        .source = WHEEL_ISR_SOURCE_BUTTON,
        .input_state = (uint8_t)gpio_get_level(CONFIG_WHEEL_BUTTON_GPIO),
    };
    BaseType_t high_task_woken = pdFALSE;

    if (s_wheel_isr_queue) {
        if (xQueueSendFromISR(s_wheel_isr_queue, &event, &high_task_woken) != pdTRUE) {
            s_wheel_isr_queue_drops++;
        }
        if (high_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static void wheel_task(void *arg)
{
    (void)arg;

    bool stable_button_pressed = gpio_get_level(CONFIG_WHEEL_BUTTON_GPIO) == 0;
    bool candidate_button_pressed = stable_button_pressed;
    bool button_debounce_pending = false;
    TickType_t button_debounce_deadline = 0;
    s_wheel_button_pressed = stable_button_pressed;
    wheel_isr_event_t event = {0};
    uint32_t last_reported_queue_drops = 0;
    bool suppress_wake_cycle = false;
    bool encoder_sync_pending = false;
    TickType_t encoder_sync_deadline = 0;

    while (1) {
        TickType_t now = xTaskGetTickCount();

        if (button_debounce_pending &&
            wheel_tick_reached(now, button_debounce_deadline)) {
            bool sampled_button_pressed =
                gpio_get_level(CONFIG_WHEEL_BUTTON_GPIO) == 0;

            if (sampled_button_pressed != candidate_button_pressed) {
                candidate_button_pressed = sampled_button_pressed;
                button_debounce_deadline =
                    now + pdMS_TO_TICKS(CONFIG_WHEEL_BUTTON_DEBOUNCE_MS);
            } else {
                button_debounce_pending = false;
                if (candidate_button_pressed != stable_button_pressed) {
                    stable_button_pressed = candidate_button_pressed;
                    s_wheel_button_pressed = stable_button_pressed;
                    wheel_report_state(stable_button_pressed
                                           ? WHEEL_MODULE_STATE_BUTTON_PRESSED
                                           : WHEEL_MODULE_STATE_BUTTON_RELEASED);
                }
            }
        }

        if (encoder_sync_pending &&
            wheel_tick_reached(now, encoder_sync_deadline)) {
            encoder_sync_pending = false;
            /* GPIO and PCNT observe the same pins through different hardware
             * paths. Re-read once after 1 ms so the final edge of a detent is
             * not left pending in PCNT with no later GPIO event to wake us. */
            wheel_process_pcnt_count(&suppress_wake_cycle);
        }

        uint32_t queue_drops = s_wheel_isr_queue_drops;
        if (queue_drops != last_reported_queue_drops) {
            ESP_LOGW(TAG, "ISR queue overflow drops=%" PRIu32, queue_drops);
            last_reported_queue_drops = queue_drops;
        }

        TickType_t wait_ticks = button_debounce_pending
                                    ? wheel_ticks_until(now, button_debounce_deadline)
                                    : portMAX_DELAY;
        if (encoder_sync_pending) {
            TickType_t encoder_wait =
                wheel_ticks_until(now, encoder_sync_deadline);
            if (wait_ticks == portMAX_DELAY || encoder_wait < wait_ticks) {
                wait_ticks = encoder_wait;
            }
        }
        if (xQueueReceive(s_wheel_isr_queue, &event, wait_ticks) != pdTRUE) {
            continue;
        }

        if (event.source == WHEEL_ISR_SOURCE_ENCODER ||
            event.source == WHEEL_ISR_SOURCE_WAKE_ENCODER) {
            if (event.source == WHEEL_ISR_SOURCE_WAKE_ENCODER) {
                /* PCNT cannot guarantee counting the edge that woke the APB
                 * domain. Recover exactly one physical detent from the
                 * captured wake pin, then discard the remaining edges of that
                 * same cycle until the encoder returns to its rest state. */
                wheel_pcnt_sync(true);
                s_wheel_step_accum = 0;
                if (event.wake_direction > 0) {
                    wheel_report_state(WHEEL_MODULE_STATE_SCROLL_UP);
                } else if (event.wake_direction < 0) {
                    wheel_report_state(WHEEL_MODULE_STATE_SCROLL_DOWN);
                }
                suppress_wake_cycle =
                    wheel_read_state() != s_wheel_wakeup_reference_state;
                encoder_sync_pending = suppress_wake_cycle;
                encoder_sync_deadline =
                    xTaskGetTickCount() + pdMS_TO_TICKS(WHEEL_PCNT_SETTLE_MS);
                continue;
            }

            wheel_process_pcnt_count(&suppress_wake_cycle);
            encoder_sync_pending = true;
            encoder_sync_deadline =
                xTaskGetTickCount() + pdMS_TO_TICKS(WHEEL_PCNT_SETTLE_MS);
            continue;
        }

        if (event.source == WHEEL_ISR_SOURCE_WAKE_BUTTON &&
            event.input_state != 0) {
            /* A short click may already be released by the time the CPU has
             * resumed. The wake GPIO mask proves that the active-low press
             * occurred, so preserve it as one complete click. */
            wheel_report_state(WHEEL_MODULE_STATE_BUTTON_PRESSED);
            wheel_report_state(WHEEL_MODULE_STATE_BUTTON_RELEASED);
            stable_button_pressed = false;
            candidate_button_pressed = false;
            button_debounce_pending = false;
            s_wheel_button_pressed = false;
            continue;
        }

        bool sampled_button_pressed = event.input_state == 0;
        if (sampled_button_pressed != candidate_button_pressed) {
            candidate_button_pressed = sampled_button_pressed;
            button_debounce_pending = true;
            button_debounce_deadline =
                xTaskGetTickCount() +
                pdMS_TO_TICKS(CONFIG_WHEEL_BUTTON_DEBOUNCE_MS);
        }
    }
}

esp_err_t wheel_module_init(void)
{
    gpio_config_t encoder_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONFIG_WHEEL_A_GPIO)
                      | (1ULL << CONFIG_WHEEL_B_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&encoder_conf), TAG, "encoder gpio config failed");

    gpio_config_t button_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONFIG_WHEEL_BUTTON_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&button_conf), TAG, "button gpio config failed");
    ESP_RETURN_ON_ERROR(wheel_pcnt_init(), TAG, "PCNT quadrature init failed");

    s_wheel_isr_queue = xQueueCreate(WHEEL_ISR_QUEUE_SIZE, sizeof(wheel_isr_event_t));
    ESP_RETURN_ON_FALSE(s_wheel_isr_queue, ESP_ERR_NO_MEM, TAG, "create isr queue failed");

    s_wheel_wakeup_reference_state = wheel_read_state();

    ESP_LOGI(TAG, "init OK: A=%d B=%d button=%d state=%u button_level=%d decoder=PCNT",
             CONFIG_WHEEL_A_GPIO,
             CONFIG_WHEEL_B_GPIO,
             CONFIG_WHEEL_BUTTON_GPIO,
             s_wheel_wakeup_reference_state,
             gpio_get_level(CONFIG_WHEEL_BUTTON_GPIO));
    return ESP_OK;
}

void wheel_module_set_report_callback(wheel_module_report_cb_t callback, void *user_ctx)
{
    s_report_callback = callback;
    s_report_user_ctx = user_ctx;
}

esp_err_t wheel_module_start(void)
{
    ESP_RETURN_ON_FALSE(s_wheel_isr_queue, ESP_ERR_INVALID_STATE, TAG, "wheel module is not initialized");

    ESP_RETURN_ON_ERROR(gpio_isr_service_install_once(), TAG, "install isr service failed");

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CONFIG_WHEEL_A_GPIO, wheel_encoder_isr_handler, NULL),
                        TAG,
                        "add A isr failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CONFIG_WHEEL_B_GPIO, wheel_encoder_isr_handler, NULL),
                        TAG,
                        "add B isr failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CONFIG_WHEEL_BUTTON_GPIO, wheel_button_isr_handler, NULL),
                        TAG,
                        "add button isr failed");

    BaseType_t ok = xTaskCreate(wheel_task, "wheel_task", 3072, NULL, 6, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create wheel task failed");
    return ESP_OK;
}

void wheel_module_get_wakeup_state(wheel_module_wakeup_state_t *state)
{
    if (!state) {
        return;
    }
    state->a_gpio = CONFIG_WHEEL_A_GPIO;
    state->b_gpio = CONFIG_WHEEL_B_GPIO;
    state->button_gpio = CONFIG_WHEEL_BUTTON_GPIO;
    state->encoder_state = wheel_read_state();
}

void wheel_module_arm_wakeup(void)
{
    s_wheel_wakeup_reference_state = wheel_read_state();
}

bool wheel_module_is_button_pressed(void)
{
    /* Include the live active-low level so a press that is still inside the
     * debounce interval also blocks the sleep countdown. */
    return s_wheel_button_pressed ||
           gpio_get_level(CONFIG_WHEEL_BUTTON_GPIO) == 0;
}

esp_err_t wheel_module_notify_wakeup(uint64_t gpio_mask)
{
    ESP_RETURN_ON_FALSE(s_wheel_isr_queue, ESP_ERR_INVALID_STATE, TAG, "wheel queue is not ready");

    uint64_t encoder_mask = (1ULL << CONFIG_WHEEL_A_GPIO) | (1ULL << CONFIG_WHEEL_B_GPIO);
    if (gpio_mask & encoder_mask) {
        uint8_t changed_state_bits = 0;
        if (gpio_mask & (1ULL << CONFIG_WHEEL_A_GPIO)) {
            changed_state_bits |= 0x02;
        }
        if (gpio_mask & (1ULL << CONFIG_WHEEL_B_GPIO)) {
            changed_state_bits |= 0x01;
        }

        uint8_t current_state = wheel_read_state();
        uint8_t first_state = current_state;
        if (changed_state_bits == 0x01 || changed_state_bits == 0x02) {
            first_state = s_wheel_wakeup_reference_state ^ changed_state_bits;
        }
        int8_t wake_direction = wheel_transition_delta(
            s_wheel_wakeup_reference_state,
            first_state);
#if CONFIG_WHEEL_REVERSE
        wake_direction = -wake_direction;
#endif
        wheel_isr_event_t event = {
            .source = WHEEL_ISR_SOURCE_WAKE_ENCODER,
            .input_state = current_state,
            .wake_direction = wake_direction,
        };
        ESP_RETURN_ON_FALSE(xQueueSend(s_wheel_isr_queue, &event, 0) == pdTRUE,
                            ESP_ERR_TIMEOUT,
                            TAG,
                            "queue wake encoder failed");
    }

    if (gpio_mask & (1ULL << CONFIG_WHEEL_BUTTON_GPIO)) {
        wheel_isr_event_t event = {
            .source = WHEEL_ISR_SOURCE_WAKE_BUTTON,
            .input_state = (uint8_t)gpio_get_level(CONFIG_WHEEL_BUTTON_GPIO),
        };
        ESP_RETURN_ON_FALSE(xQueueSend(s_wheel_isr_queue, &event, 0) == pdTRUE,
                            ESP_ERR_TIMEOUT,
                            TAG,
                            "queue wake wheel button failed");
    }
    return ESP_OK;
}
