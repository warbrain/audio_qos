#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t scepter_hardware_start(void);
esp_err_t scepter_hardware_vibrate(uint32_t duration_ms);
esp_err_t scepter_hardware_play_ui_animation(uint8_t command);
esp_err_t scepter_hardware_set_mic_fixed_gain(uint8_t gain_x, bool persist);
void scepter_hardware_set_connected(bool connected);
void scepter_hardware_set_ota_active(bool active);

#ifdef __cplusplus
}
#endif
