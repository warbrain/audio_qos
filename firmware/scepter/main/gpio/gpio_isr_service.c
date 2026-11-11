#include "gpio_isr_service.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "gpio_isr_service";

static bool s_gpio_isr_service_installed;

esp_err_t gpio_isr_service_install_once(void)
{
    if (s_gpio_isr_service_installed) {
        return ESP_OK;
    }

    esp_err_t ret = gpio_install_isr_service(0);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        s_gpio_isr_service_installed = true;
        return ESP_OK;
    }

    ESP_LOGE(TAG, "install failed: %s", esp_err_to_name(ret));
    return ret;
}
