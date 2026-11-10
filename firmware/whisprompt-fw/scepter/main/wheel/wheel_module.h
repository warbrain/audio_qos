#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef enum {
    WHEEL_MODULE_STATE_SCROLL_UP,
    WHEEL_MODULE_STATE_SCROLL_DOWN,
    WHEEL_MODULE_STATE_BUTTON_PRESSED,
    WHEEL_MODULE_STATE_BUTTON_RELEASED,
} wheel_module_state_t;

typedef void (*wheel_module_report_cb_t)(wheel_module_state_t state, void *user_ctx);

typedef struct {
    gpio_num_t a_gpio;
    gpio_num_t b_gpio;
    gpio_num_t button_gpio;
    uint8_t encoder_state;
} wheel_module_wakeup_state_t;

esp_err_t wheel_module_init(void);
void wheel_module_set_report_callback(wheel_module_report_cb_t callback, void *user_ctx);
esp_err_t wheel_module_start(void);
bool wheel_module_is_button_pressed(void);
void wheel_module_get_wakeup_state(wheel_module_wakeup_state_t *state);
void wheel_module_arm_wakeup(void);
esp_err_t wheel_module_notify_wakeup(uint64_t gpio_mask);
