#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifndef USER_KEYS_LOG_ENABLE
#define USER_KEYS_LOG_ENABLE 0
#endif

#define USER_KEYS_COUNT 3

typedef enum {
    USER_KEY_ID_1 = 0,
    USER_KEY_ID_2,
    USER_KEY_ID_3,
} user_key_id_t;

typedef void (*user_keys_pressed_cb_t)(user_key_id_t key_id, void *user_ctx);
typedef void (*user_keys_state_cb_t)(user_key_id_t key_id, bool pressed, void *user_ctx);

esp_err_t user_keys_init(void);
void user_keys_set_pressed_callback(user_keys_pressed_cb_t callback, void *user_ctx);
void user_keys_set_state_callback(user_keys_state_cb_t callback, void *user_ctx);
esp_err_t user_keys_start(void);
bool user_keys_any_pressed(void);
size_t user_keys_get_gpios(gpio_num_t *gpios, size_t capacity);
uint8_t user_keys_get_ids_for_gpio_mask(uint64_t gpio_mask);
esp_err_t user_keys_notify_wakeup(uint64_t gpio_mask);
