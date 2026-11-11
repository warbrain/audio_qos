#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool red;
    bool green;
    bool blue;
} rgb_led_state_t;

esp_err_t rgb_led_init(void);
esp_err_t rgb_led_set(bool red, bool green, bool blue);
esp_err_t rgb_led_off(void);
esp_err_t rgb_led_set_orange(void);
esp_err_t rgb_led_set_orange_brightness(uint8_t brightness_percent);
bool rgb_led_get_state(rgb_led_state_t *state);
