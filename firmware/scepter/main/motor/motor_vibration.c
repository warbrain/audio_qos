#include "motor_vibration.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mic_audio.h"

static const char *TAG = "motor_vibration";

#define MOTOR_PWM_GPIO       GPIO_NUM_44
#define MOTOR_LEDC_TIMER     LEDC_TIMER_1
#define MOTOR_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_CHANNEL   LEDC_CHANNEL_1
#define MOTOR_LEDC_DUTY_RES  LEDC_TIMER_10_BIT
#define MOTOR_LEDC_FREQUENCY 1000
#define MOTOR_LEDC_DUTY_ON   512
#define MOTOR_LEDC_DUTY_OFF  0
#define MOTOR_TASK_STACK_BYTES 2048
#define MOTOR_TASK_PRIORITY    3

static bool s_motor_inited;
static bool s_motor_enabled;
static TaskHandle_t s_motor_task_handle;

static TickType_t motor_duration_ticks(uint32_t duration_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(duration_ms);
    return ticks > 0 ? ticks : 1;
}

static void motor_vibration_task(void *arg)
{
    (void)arg;

    while (true) {
        uint32_t duration_ms = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &duration_ms, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        ESP_ERROR_CHECK_WITHOUT_ABORT(motor_vibration_set_enabled(true));

        while (true) {
            uint32_t next_duration_ms = 0;
            if (xTaskNotifyWait(0,
                                UINT32_MAX,
                                &next_duration_ms,
                                motor_duration_ticks(duration_ms)) == pdTRUE) {
                duration_ms = next_duration_ms;
                continue;
            }

            ESP_ERROR_CHECK_WITHOUT_ABORT(motor_vibration_set_enabled(false));
            break;
        }
    }
}

esp_err_t motor_vibration_init(void)
{
    if (s_motor_inited) {
        return ESP_OK;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << MOTOR_PWM_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "motor gpio config failed");
    gpio_set_level(MOTOR_PWM_GPIO, 0);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = MOTOR_LEDC_MODE,
        .timer_num = MOTOR_LEDC_TIMER,
        .duty_resolution = MOTOR_LEDC_DUTY_RES,
        .freq_hz = MOTOR_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&ledc_timer), TAG, "motor ledc timer config failed");

    ledc_channel_config_t ledc_channel = {
        .gpio_num = MOTOR_PWM_GPIO,
        .speed_mode = MOTOR_LEDC_MODE,
        .channel = MOTOR_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = MOTOR_LEDC_TIMER,
        .duty = MOTOR_LEDC_DUTY_OFF,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ledc_channel), TAG, "motor ledc channel config failed");

    s_motor_inited = true;
    s_motor_enabled = false;

    BaseType_t task_ok = xTaskCreate(motor_vibration_task,
                                     "motor_vibration",
                                     MOTOR_TASK_STACK_BYTES,
                                     NULL,
                                     MOTOR_TASK_PRIORITY,
                                     &s_motor_task_handle);
    if (task_ok != pdPASS) {
        s_motor_inited = false;
        s_motor_task_handle = NULL;
        ESP_LOGE(TAG, "create motor task failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "init OK: GPIO44 PWM ready at %d Hz duty 0%%", MOTOR_LEDC_FREQUENCY);
    return ESP_OK;
}

esp_err_t motor_vibration_set_enabled(bool enabled)
{
    ESP_RETURN_ON_FALSE(s_motor_inited, ESP_ERR_INVALID_STATE, TAG, "motor not initialized");

    if (s_motor_enabled == enabled) {
        return ESP_OK;
    }

    uint32_t duty = enabled ? MOTOR_LEDC_DUTY_ON : MOTOR_LEDC_DUTY_OFF;
    ESP_RETURN_ON_ERROR(ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL, duty),
                        TAG,
                        "motor set duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL),
                        TAG,
                        "motor update duty failed");

    s_motor_enabled = enabled;
    mic_audio_set_motor_active(enabled);
    return ESP_OK;
}

esp_err_t motor_vibration_pulse(uint32_t duration_ms)
{
    ESP_RETURN_ON_FALSE(s_motor_inited && s_motor_task_handle != NULL,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "motor not initialized");
    ESP_RETURN_ON_FALSE(duration_ms > 0, ESP_ERR_INVALID_ARG, TAG, "duration must be greater than 0 ms");
    ESP_RETURN_ON_FALSE(xTaskNotify(s_motor_task_handle,
                                    duration_ms,
                                    eSetValueWithOverwrite) == pdPASS,
                        ESP_FAIL,
                        TAG,
                        "notify motor task failed");
    return ESP_OK;
}
