#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#define SHARED_I2C_PORT       I2C_NUM_1
#define SHARED_I2C_SDA_GPIO   6
#define SHARED_I2C_SCL_GPIO   5

esp_err_t shared_i2c_get_bus(i2c_master_bus_handle_t *bus);

