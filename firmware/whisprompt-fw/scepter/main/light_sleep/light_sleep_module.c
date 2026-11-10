#include "light_sleep_module.h"

#include <inttypes.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"

static const char *TAG = "light_sleep";

#define LIGHT_SLEEP_MODULE_MAX_WAKE_IOS 12

static light_sleep_module_config_t s_config;
static bool s_inited;
static bool s_sleeping;

static uint32_t default_if_zero(uint32_t value, uint32_t default_value)
{
    return value == 0 ? default_value : value;
}

static bool wake_level_is_valid(gpio_int_type_t wake_level)
{
    return wake_level == GPIO_INTR_LOW_LEVEL || wake_level == GPIO_INTR_HIGH_LEVEL;
}

static uint64_t active_wakeup_mask(const light_sleep_module_wakeup_io_t *wake_ios,
                                   size_t wake_io_count)
{
    uint64_t active_mask = 0;
    for (size_t i = 0; i < wake_io_count; i++) {
        int active_level = wake_ios[i].wake_level == GPIO_INTR_HIGH_LEVEL ? 1 : 0;
        if (gpio_get_level(wake_ios[i].gpio_num) == active_level) {
            active_mask |= 1ULL << wake_ios[i].gpio_num;
        }
    }
    return active_mask;
}

static bool wait_wakeup_ios_idle(const light_sleep_module_wakeup_io_t *wake_ios,
                                 size_t wake_io_count)
{
    uint32_t waited_us = 0;
    uint32_t stable_idle_us = 0;

    while (waited_us < s_config.int_idle_wait_us) {
        if (active_wakeup_mask(wake_ios, wake_io_count) == 0) {
            stable_idle_us += s_config.int_idle_step_us;
            if (stable_idle_us >= s_config.int_idle_stable_us) {
                return true;
            }
        } else {
            stable_idle_us = 0;
        }

        esp_rom_delay_us(s_config.int_idle_step_us);
        waited_us += s_config.int_idle_step_us;
    }

    return false;
}

static esp_err_t prepare_wakeup_ios(light_sleep_module_wakeup_io_t *wake_ios,
                                    size_t *wake_io_count)
{
    *wake_io_count = 0;
    ESP_RETURN_ON_ERROR(s_config.prepare_wakeup(wake_ios,
                                                LIGHT_SLEEP_MODULE_MAX_WAKE_IOS,
                                                wake_io_count,
                                                s_config.user_ctx),
                        TAG,
                        "prepare wake GPIOs failed");
    ESP_RETURN_ON_FALSE(*wake_io_count > 0 &&
                        *wake_io_count <= LIGHT_SLEEP_MODULE_MAX_WAKE_IOS,
                        ESP_ERR_INVALID_SIZE,
                        TAG,
                        "invalid wake GPIO count");

    uint64_t seen_mask = 0;
    for (size_t i = 0; i < *wake_io_count; i++) {
        const light_sleep_module_wakeup_io_t *io = &wake_ios[i];
        ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(io->gpio_num),
                            ESP_ERR_INVALID_ARG,
                            TAG,
                            "invalid wake GPIO");
        ESP_RETURN_ON_FALSE(wake_level_is_valid(io->wake_level),
                            ESP_ERR_INVALID_ARG,
                            TAG,
                            "wake GPIO must use a level trigger");
        uint64_t gpio_mask = 1ULL << io->gpio_num;
        ESP_RETURN_ON_FALSE((seen_mask & gpio_mask) == 0,
                            ESP_ERR_INVALID_ARG,
                            TAG,
                            "duplicate wake GPIO");
        seen_mask |= gpio_mask;
    }
    return ESP_OK;
}

static void restore_runtime_gpio_interrupts(const light_sleep_module_wakeup_io_t *wake_ios,
                                            size_t wake_io_count)
{
    for (size_t i = 0; i < wake_io_count; i++) {
        const light_sleep_module_wakeup_io_t *io = &wake_ios[i];
        (void)gpio_intr_disable(io->gpio_num);
        (void)gpio_wakeup_disable(io->gpio_num);
        (void)gpio_set_intr_type(io->gpio_num, io->runtime_intr_type);
        if (io->runtime_intr_enabled) {
            (void)gpio_intr_enable(io->gpio_num);
        }
    }
}

static esp_err_t enable_gpio_wakeup(const light_sleep_module_wakeup_io_t *wake_ios,
                                    size_t wake_io_count)
{
    esp_err_t ret = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "disable previous wake sources failed: %s", esp_err_to_name(ret));
    }

    for (size_t i = 0; i < wake_io_count; i++) {
        const light_sleep_module_wakeup_io_t *io = &wake_ios[i];
        ESP_RETURN_ON_ERROR(gpio_intr_disable(io->gpio_num), TAG, "disable GPIO interrupt failed");
        ESP_RETURN_ON_ERROR(gpio_wakeup_enable(io->gpio_num, io->wake_level),
                            TAG,
                            "enable GPIO wakeup failed");
    }
    return esp_sleep_enable_gpio_wakeup();
}

static void restore_local_peripherals(void)
{
    if (s_config.on_wake) {
        s_config.on_wake(s_config.user_ctx);
    }
    s_config.set_tx_enabled(true, s_config.user_ctx);
    if (s_config.set_mic_enabled) {
        s_config.set_mic_enabled(true, s_config.user_ctx);
    }
}

esp_err_t light_sleep_module_init(const light_sleep_module_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "config is null");
    ESP_RETURN_ON_FALSE(config->set_tx_enabled,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "set tx callback is null");
    ESP_RETURN_ON_FALSE(config->recover_comm,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "recover comm callback is null");
    ESP_RETURN_ON_FALSE(config->prepare_wakeup,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "prepare wakeup callback is null");

    memcpy(&s_config, config, sizeof(s_config));
    s_config.flat_sleep_ms = default_if_zero(s_config.flat_sleep_ms,
                                             LIGHT_SLEEP_MODULE_DEFAULT_FLAT_SLEEP_MS);
    s_config.int_idle_wait_us = default_if_zero(s_config.int_idle_wait_us,
                                                LIGHT_SLEEP_MODULE_DEFAULT_IDLE_WAIT_US);
    s_config.int_idle_stable_us = default_if_zero(s_config.int_idle_stable_us,
                                                  LIGHT_SLEEP_MODULE_DEFAULT_IDLE_STABLE_US);
    s_config.int_idle_step_us = default_if_zero(s_config.int_idle_step_us,
                                                LIGHT_SLEEP_MODULE_DEFAULT_IDLE_STEP_US);

    s_inited = true;
    ESP_LOGI(TAG,
             "init OK: multi-GPIO wake flat_sleep=%" PRIu32 "ms idle_wait=%" PRIu32 "us",
             s_config.flat_sleep_ms,
             s_config.int_idle_wait_us);
    return ESP_OK;
}

uint32_t light_sleep_module_get_flat_sleep_ms(void)
{
    return s_inited ? s_config.flat_sleep_ms : LIGHT_SLEEP_MODULE_DEFAULT_FLAT_SLEEP_MS;
}

esp_err_t light_sleep_module_enter(uint32_t stable_ms)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "module is not initialized");
    ESP_RETURN_ON_FALSE(!s_sleeping, ESP_ERR_INVALID_STATE, TAG, "sleep is already in progress");

    light_sleep_module_wakeup_io_t wake_ios[LIGHT_SLEEP_MODULE_MAX_WAKE_IOS] = {0};
    size_t wake_io_count = 0;
    ESP_RETURN_ON_ERROR(prepare_wakeup_ios(wake_ios, &wake_io_count), TAG, "get wake GPIOs failed");

    if (!wait_wakeup_ios_idle(wake_ios, wake_io_count)) {
        ESP_LOGW(TAG, "skip light sleep: wake GPIOs did not remain idle");
        return ESP_ERR_INVALID_STATE;
    }

    s_sleeping = true;
    s_config.set_tx_enabled(false, s_config.user_ctx);
    if (s_config.set_mic_enabled) {
        s_config.set_mic_enabled(false, s_config.user_ctx);
    }
    esp_err_t ret = ESP_OK;
    if (s_config.before_sleep) {
        ret = s_config.before_sleep(s_config.user_ctx);
        if (ret != ESP_OK) {
            ESP_LOGI(TAG, "light sleep cancelled during prepare: %s", esp_err_to_name(ret));
            restore_local_peripherals();
            s_sleeping = false;
            return ret;
        }
    }

    /* Re-sample dynamic wake levels after peripheral shutdown. */
    ret = prepare_wakeup_ios(wake_ios, &wake_io_count);
    if (ret != ESP_OK) {
        restore_local_peripherals();
        s_sleeping = false;
        return ret;
    }

    ESP_LOGI(TAG,
             "flat stable %" PRIu32 "ms, entering light sleep with %u wake GPIOs",
             stable_ms, (unsigned)wake_io_count);

    ret = enable_gpio_wakeup(wake_ios, wake_io_count);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "enable GPIO wakeup failed: %s", esp_err_to_name(ret));
        restore_runtime_gpio_interrupts(wake_ios, wake_io_count);
        restore_local_peripherals();
        s_sleeping = false;
        return ret;
    }

    uint64_t active_mask = active_wakeup_mask(wake_ios, wake_io_count);
    if (active_mask != 0) {
        ESP_LOGI(TAG, "input arrived during sleep entry, aborting sleep mask=0x%" PRIx64,
                 active_mask);
        if (s_config.wakeup_event) {
            s_config.wakeup_event(active_mask, s_config.user_ctx);
        }
        restore_runtime_gpio_interrupts(wake_ios, wake_io_count);
        restore_local_peripherals();
        s_sleeping = false;
        return ESP_ERR_INVALID_STATE;
    }

    ret = esp_light_sleep_start();
    /* ESP32-S3 does not expose a per-pin GPIO wake status API. Level wake
     * sources remain active until their input is consumed, so sample them
     * immediately on return, before restoring drivers or other peripherals. */
    uint64_t wake_gpio_mask = active_wakeup_mask(wake_ios, wake_io_count);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "light sleep failed: %s", esp_err_to_name(ret));
        wake_gpio_mask |= active_wakeup_mask(wake_ios, wake_io_count);
        if (wake_gpio_mask != 0 && s_config.wakeup_event) {
            s_config.wakeup_event(wake_gpio_mask, s_config.user_ctx);
        }
        restore_runtime_gpio_interrupts(wake_ios, wake_io_count);
        restore_local_peripherals();
        s_sleeping = false;
        return ret;
    }

    ESP_LOGI(TAG, "woke from light sleep gpio_mask=0x%" PRIx64 ", restoring comm",
             wake_gpio_mask);

    if (wake_gpio_mask != 0 && s_config.wakeup_event) {
        s_config.wakeup_event(wake_gpio_mask, s_config.user_ctx);
    }
    restore_runtime_gpio_interrupts(wake_ios, wake_io_count);

    if (s_config.on_wake) {
        s_config.on_wake(s_config.user_ctx);
    }

    if (s_config.set_comm_recovering) {
        s_config.set_comm_recovering(true, s_config.user_ctx);
    }
    ret = s_config.recover_comm(s_config.user_ctx);
    if (s_config.set_comm_recovering) {
        s_config.set_comm_recovering(false, s_config.user_ctx);
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "recover comm after sleep failed: %s", esp_err_to_name(ret));
    }

    s_config.set_tx_enabled(true, s_config.user_ctx);
    if (s_config.set_mic_enabled) {
        s_config.set_mic_enabled(true, s_config.user_ctx);
    }
    ESP_LOGI(TAG, "tx enabled: woke from light sleep");

    if (s_config.send_hello) {
        esp_err_t hello_ret = s_config.send_hello(s_config.user_ctx);
        if (hello_ret != ESP_OK) {
            ESP_LOGW(TAG, "send wake HELLO failed: %s", esp_err_to_name(hello_ret));
        }
    }

    s_sleeping = false;
    return ret;
}
