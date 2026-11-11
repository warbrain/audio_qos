
#include "rgb_led.h"

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "rgb_led";

#define RGB_LED_R_GPIO GPIO_NUM_46
#define RGB_LED_G_GPIO GPIO_NUM_45
#define RGB_LED_B_GPIO GPIO_NUM_43
#define RGB_LED_LEDC_MODE LEDC_LOW_SPEED_MODE
#define RGB_LED_LEDC_TIMER LEDC_TIMER_1
#define RGB_LED_LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define RGB_LED_LEDC_FREQ_HZ 1000
#define RGB_LED_MAX_DUTY ((1U << 10) - 1U)
#ifndef RGB_LED_BRIGHTNESS_PERCENT
#define RGB_LED_BRIGHTNESS_PERCENT 10
#endif
#define RGB_LED_ORANGE_GREEN_PERCENT 8

static bool s_rgb_led_inited;
static rgb_led_state_t s_rgb_led_state;
static uint32_t s_rgb_led_duty;

static uint32_t brightness_to_duty(uint32_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    return (RGB_LED_MAX_DUTY * percent) / 100;
}

static esp_err_t rgb_led_set_channel_duty(ledc_channel_t channel, uint32_t duty)
{
    ESP_RETURN_ON_ERROR(ledc_set_duty(RGB_LED_LEDC_MODE, channel, duty), TAG, "set duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(RGB_LED_LEDC_MODE, channel), TAG, "update duty failed");
    return ESP_OK;
}

static esp_err_t rgb_led_set_channel(ledc_channel_t channel, bool enabled)
{
    return rgb_led_set_channel_duty(channel, enabled ? s_rgb_led_duty : 0);
}

static esp_err_t rgb_led_apply(bool red, bool green, bool blue)
{
    ESP_RETURN_ON_ERROR(rgb_led_set_channel(LEDC_CHANNEL_2, red), TAG, "set red failed");
    ESP_RETURN_ON_ERROR(rgb_led_set_channel(LEDC_CHANNEL_3, green), TAG, "set green failed");
    ESP_RETURN_ON_ERROR(rgb_led_set_channel(LEDC_CHANNEL_4, blue), TAG, "set blue failed");

    s_rgb_led_state.red = red;
    s_rgb_led_state.green = green;
    s_rgb_led_state.blue = blue;
    return ESP_OK;
}

esp_err_t rgb_led_init(void)
{
    if (s_rgb_led_inited) {
        return ESP_OK;
    }

    s_rgb_led_duty = brightness_to_duty(RGB_LED_BRIGHTNESS_PERCENT);

    ledc_timer_config_t timer_cfg = {
        .speed_mode = RGB_LED_LEDC_MODE,
        .timer_num = RGB_LED_LEDC_TIMER,
        .duty_resolution = RGB_LED_LEDC_DUTY_RES,
        .freq_hz = RGB_LED_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "ledc timer config failed");

    const ledc_channel_config_t channels[] = {
        {
            .gpio_num = RGB_LED_R_GPIO,
            .speed_mode = RGB_LED_LEDC_MODE,
            .channel = LEDC_CHANNEL_2,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = RGB_LED_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
        },
        {
            .gpio_num = RGB_LED_G_GPIO,
            .speed_mode = RGB_LED_LEDC_MODE,
            .channel = LEDC_CHANNEL_3,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = RGB_LED_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
        },
        {
            .gpio_num = RGB_LED_B_GPIO,
            .speed_mode = RGB_LED_LEDC_MODE,
            .channel = LEDC_CHANNEL_4,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = RGB_LED_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
        },
    };

    for (size_t i = 0; i < sizeof(channels) / sizeof(channels[0]); i++) {
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channels[i]), TAG, "ledc channel config failed");
    }

    s_rgb_led_inited = true;
    ESP_RETURN_ON_ERROR(rgb_led_apply(false, false, false), TAG, "rgb off failed");

    ESP_LOGI(TAG, "init OK: R=%d G=%d B=%d active=high pwm=%dHz brightness=%d%% duty=%lu",
             RGB_LED_R_GPIO,
             RGB_LED_G_GPIO,
             RGB_LED_B_GPIO,
             RGB_LED_LEDC_FREQ_HZ,
             RGB_LED_BRIGHTNESS_PERCENT,
             (unsigned long)s_rgb_led_duty);
    return ESP_OK;
}

esp_err_t rgb_led_set(bool red, bool green, bool blue)
{
    ESP_RETURN_ON_FALSE(s_rgb_led_inited, ESP_ERR_INVALID_STATE, TAG, "rgb led not initialized");
    return rgb_led_apply(red, green, blue);
}

esp_err_t rgb_led_off(void)
{
    return rgb_led_set(false, false, false);
}

esp_err_t rgb_led_set_orange(void)
{
    return rgb_led_set_orange_brightness(100);
}

esp_err_t rgb_led_set_orange_brightness(uint8_t brightness_percent)
{
    ESP_RETURN_ON_FALSE(s_rgb_led_inited, ESP_ERR_INVALID_STATE, TAG, "rgb led not initialized");
    ESP_RETURN_ON_FALSE(brightness_percent <= 100,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "orange brightness must be 0-100%%");

    uint32_t red_duty = (s_rgb_led_duty * brightness_percent) / 100;
    uint32_t green_duty = (red_duty * RGB_LED_ORANGE_GREEN_PERCENT) / 100;
    ESP_RETURN_ON_ERROR(rgb_led_set_channel_duty(LEDC_CHANNEL_2, red_duty), TAG, "set red failed");
    ESP_RETURN_ON_ERROR(rgb_led_set_channel_duty(LEDC_CHANNEL_3, green_duty), TAG, "set green failed");
    ESP_RETURN_ON_ERROR(rgb_led_set_channel_duty(LEDC_CHANNEL_4, 0), TAG, "set blue failed");

    s_rgb_led_state.red = red_duty > 0;
    s_rgb_led_state.green = green_duty > 0;
    s_rgb_led_state.blue = false;
    return ESP_OK;
}

bool rgb_led_get_state(rgb_led_state_t *state)
{
    if (!state || !s_rgb_led_inited) {
        return false;
    }

    *state = s_rgb_led_state;
    return true;
}
