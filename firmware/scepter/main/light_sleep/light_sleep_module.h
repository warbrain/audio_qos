#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIGHT_SLEEP_MODULE_DEFAULT_FLAT_SLEEP_MS 15000
#define LIGHT_SLEEP_MODULE_DEFAULT_IDLE_WAIT_US 1000000
#define LIGHT_SLEEP_MODULE_DEFAULT_IDLE_STABLE_US 300000
#define LIGHT_SLEEP_MODULE_DEFAULT_IDLE_STEP_US 1000

typedef void (*light_sleep_module_set_bool_cb_t)(bool enabled, void *user_ctx);
typedef esp_err_t (*light_sleep_module_recover_comm_cb_t)(void *user_ctx);
typedef esp_err_t (*light_sleep_module_send_hello_cb_t)(void *user_ctx);
typedef esp_err_t (*light_sleep_module_before_sleep_cb_t)(void *user_ctx);
typedef void (*light_sleep_module_notify_cb_t)(void *user_ctx);

typedef struct {
    gpio_num_t gpio_num;
    gpio_int_type_t wake_level;
    gpio_int_type_t runtime_intr_type;
    bool runtime_intr_enabled;
} light_sleep_module_wakeup_io_t;

typedef esp_err_t (*light_sleep_module_prepare_wakeup_cb_t)(
    light_sleep_module_wakeup_io_t *wake_ios,
    size_t capacity,
    size_t *wake_io_count,
    void *user_ctx);
typedef void (*light_sleep_module_wakeup_event_cb_t)(uint64_t wake_gpio_mask,
                                                     void *user_ctx);

typedef struct {
    uint32_t flat_sleep_ms;
    uint32_t int_idle_wait_us;
    uint32_t int_idle_stable_us;
    uint32_t int_idle_step_us;
    light_sleep_module_set_bool_cb_t set_tx_enabled;
    light_sleep_module_set_bool_cb_t set_comm_recovering;
    light_sleep_module_set_bool_cb_t set_mic_enabled;
    light_sleep_module_recover_comm_cb_t recover_comm;
    light_sleep_module_send_hello_cb_t send_hello;
    light_sleep_module_prepare_wakeup_cb_t prepare_wakeup;
    light_sleep_module_wakeup_event_cb_t wakeup_event;
    light_sleep_module_before_sleep_cb_t before_sleep;
    light_sleep_module_notify_cb_t on_wake;
    void *user_ctx;
} light_sleep_module_config_t;

esp_err_t light_sleep_module_init(const light_sleep_module_config_t *config);
uint32_t light_sleep_module_get_flat_sleep_ms(void);
esp_err_t light_sleep_module_enter(uint32_t stable_ms);

#ifdef __cplusplus
}
#endif
