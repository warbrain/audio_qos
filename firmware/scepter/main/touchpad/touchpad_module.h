#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    int16_t dx;
    int16_t dy;
    uint8_t buttons;
    int8_t vertical;
    int8_t horizontal;
} touchpad_motion_t;

typedef void (*touchpad_motion_cb_t)(const touchpad_motion_t *motion, void *user_ctx);

esp_err_t touchpad_module_init(void);
void touchpad_module_set_motion_callback(touchpad_motion_cb_t callback, void *user_ctx);
esp_err_t touchpad_module_start(void);
bool touchpad_module_is_active(void);
gpio_num_t touchpad_module_get_interrupt_gpio(void);
esp_err_t touchpad_module_process_wakeup(void);
