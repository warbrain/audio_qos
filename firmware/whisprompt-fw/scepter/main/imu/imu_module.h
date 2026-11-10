#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint8_t data_status;
} imu_sample_t;

typedef enum {
    IMU_POSTURE_IDLE = 0,
    IMU_POSTURE_STILL,
    IMU_POSTURE_LIFT,
    IMU_POSTURE_SHAKE,
} imu_posture_t;

typedef void (*imu_sample_cb_t)(const imu_sample_t *sample, void *user_ctx);
typedef void (*imu_posture_cb_t)(imu_posture_t posture, uint32_t stable_ms, void *user_ctx);

esp_err_t imu_module_init(void);
void imu_module_set_sample_callback(imu_sample_cb_t callback, void *user_ctx);
void imu_module_set_posture_callback(imu_posture_cb_t callback, void *user_ctx);
esp_err_t imu_module_start(void);
