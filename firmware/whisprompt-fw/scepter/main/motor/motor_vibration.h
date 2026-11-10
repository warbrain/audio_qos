#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t motor_vibration_init(void);
esp_err_t motor_vibration_set_enabled(bool enabled);
esp_err_t motor_vibration_pulse(uint32_t duration_ms);
