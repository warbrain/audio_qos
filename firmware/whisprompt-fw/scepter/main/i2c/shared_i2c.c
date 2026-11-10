#include "shared_i2c.h"

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "shared_i2c";

static i2c_master_bus_handle_t s_shared_i2c_bus;

esp_err_t shared_i2c_get_bus(i2c_master_bus_handle_t *bus)
{
    ESP_RETURN_ON_FALSE(bus, ESP_ERR_INVALID_ARG, TAG, "bus out is null");

    if (!s_shared_i2c_bus) {
        i2c_master_bus_config_t bus_conf = {
            .i2c_port = SHARED_I2C_PORT,
            .sda_io_num = SHARED_I2C_SDA_GPIO,
            .scl_io_num = SHARED_I2C_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_conf, &s_shared_i2c_bus),
                            TAG,
                            "new shared bus failed");
        ESP_LOGI(TAG,
                 "bus init OK: port=%d SDA=%d SCL=%d",
                 SHARED_I2C_PORT,
                 SHARED_I2C_SDA_GPIO,
                 SHARED_I2C_SCL_GPIO);
    }

    *bus = s_shared_i2c_bus;
    return ESP_OK;
}

