#include "imu_module.h"

#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_isr_service.h"
#include "sdkconfig.h"
#include "shared_i2c.h"

static const char *TAG = "imu";

#ifndef IMU_TRACE_LOGS
#define IMU_TRACE_LOGS           0
#endif
#define IMU_I2C_ADDR_PRIMARY      0x18
#define IMU_I2C_ADDR_SECONDARY    0x19
#define IMU_WHO_AM_I_VALUE        0x6A

#ifndef CONFIG_IMU_SDA_GPIO
#define CONFIG_IMU_SDA_GPIO       SHARED_I2C_SDA_GPIO
#endif

#ifndef CONFIG_IMU_SCL_GPIO
#define CONFIG_IMU_SCL_GPIO       SHARED_I2C_SCL_GPIO
#endif

#ifndef CONFIG_IMU_INT1_GPIO
#define CONFIG_IMU_INT1_GPIO      7
#endif

#ifndef CONFIG_IMU_INT2_GPIO
#define CONFIG_IMU_INT2_GPIO      42
#endif

#ifndef CONFIG_IMU_I2C_FREQ_HZ
#define CONFIG_IMU_I2C_FREQ_HZ    400000
#endif

#ifndef CONFIG_IMU_I2C_TIMEOUT_MS
#define CONFIG_IMU_I2C_TIMEOUT_MS 100
#endif

#ifndef CONFIG_IMU_READ_INTERVAL_MS
#define CONFIG_IMU_READ_INTERVAL_MS 1
#endif

#ifndef CONFIG_IMU_LOG_INTERVAL_MS
#define CONFIG_IMU_LOG_INTERVAL_MS 1000
#endif

#ifndef CONFIG_IMU_USE_INT_READ
#define CONFIG_IMU_USE_INT_READ 0
#endif

#define IMU_INT_BURST_READ_MS    120
#define IMU_INT_GPIO             CONFIG_IMU_INT1_GPIO
#define IMU_STILL_POSTURE_NOTIFY_MS 10000
#define IMU_STILL_POSTURE_NOTIFY_RETRY_MS 1000

#if IMU_TRACE_LOGS
#define IMU_LOGI(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define IMU_LOGI(fmt, ...) do { } while (0)
#endif
#define IMU_LOGW(fmt, ...) ESP_LOGW(TAG, fmt, ##__VA_ARGS__)

#define IMU_FILTER_SHIFT             1
#define IMU_ONE_G_RAW                4096
#define IMU_STATIC_FLAT_ACC_X_RAW    (-134)
#define IMU_STATIC_FLAT_ACC_Y_RAW    (-54)
#define IMU_STATIC_FLAT_ACC_Z_RAW    4504
#define IMU_STATIC_FLAT_GYRO_X_RAW   (-4)
#define IMU_STATIC_FLAT_GYRO_Y_RAW   14
#define IMU_STATIC_FLAT_GYRO_Z_RAW   (-20)
#define IMU_STATIC_FLAT_Z_REF        4504
#define IMU_GRAVITY_AXIS_MIN         2800
#define IMU_FLAT_ENTER_XY_MAX        900
#define IMU_FLAT_EXIT_XY_MAX         1800
#define IMU_FLAT_ENTER_Z_MIN         3000
#define IMU_FLAT_EXIT_Z_MIN          2400
#define IMU_FLAT_ENTER_GYRO_MAX      250
#define IMU_FLAT_EXIT_GYRO_MAX       1000
#define IMU_FLAT_ENTER_CONFIRM_FRAMES 20
#define IMU_FLAT_EXIT_CONFIRM_FRAMES 12
#define IMU_LIFT_Z_LEAVE_DELTA       900
#define IMU_LIFT_Z_BELOW_REF_DELTA   500
#define IMU_LIFT_Y_GRAVITY_MIN       2200
#define IMU_LIFT_Z_DROP_DELTA        120
#define IMU_LIFT_FRAME_DELTA_MIN     180
#define IMU_LIFT_GYRO_MIN            80
#define IMU_SHAKE_GYRO_MAX           4200
#define IMU_SHAKE_ACC_MAG_DELTA      3000
#define IMU_SHAKE_ACC_FRAME_DELTA    3600
#define IMU_SHAKE_REVERSAL_GYRO_MIN  1200
#define IMU_SHAKE_REVERSAL_MIN_COUNT 2
#define IMU_STILL_GYRO_MAX           700
#define IMU_STILL_ACC_FRAME_DELTA_MAX 260
#define IMU_STILL_ACC_MAG_DELTA_MAX  1400
#define IMU_FRAMES_FROM_MS(ms)       (((ms) + CONFIG_IMU_READ_INTERVAL_MS - 1) / CONFIG_IMU_READ_INTERVAL_MS)
#define IMU_MOTION_WINDOW_SIZE       IMU_FRAMES_FROM_MS(100)
#define IMU_MOTION_WINDOW_MIN_FRAMES IMU_FRAMES_FROM_MS(60)
#define IMU_SHAKE_HIT_MIN_FRAMES     IMU_FRAMES_FROM_MS(25)
#define IMU_ACTION_HOLD_FRAMES       IMU_FRAMES_FROM_MS(250)
#define IMU_LIFT_ACTIVE_FRAMES       IMU_FRAMES_FROM_MS(500)
#define IMU_LIFT_CONTEXT_FRAMES      1
#define IMU_LIFT_ARM_FLAT_FRAMES     IMU_FRAMES_FROM_MS(300)
#define IMU_LIFT_RECENT_FLAT_FRAMES  IMU_FRAMES_FROM_MS(1500)
#define IMU_LIFT_SHAKE_SUPPRESS_FRAMES IMU_FRAMES_FROM_MS(200)
#define IMU_LIFT_CANDIDATE_FRAMES    IMU_FRAMES_FROM_MS(500)
#define IMU_LIFT_CANDIDATE_AXIS_MIN  1600
#define IMU_LIFT_CANDIDATE_Z_DELTA   900
#define IMU_LIFT_DROP_SUPPRESS_FRAMES IMU_FRAMES_FROM_MS(700)
#define IMU_DROP_HOLD_FRAMES         IMU_FRAMES_FROM_MS(700)
#define IMU_DROP_CANDIDATE_FRAMES    IMU_FRAMES_FROM_MS(500)
#define IMU_DROP_CANDIDATE_AXIS_MIN  2400
#define IMU_DROP_CANDIDATE_Z_DELTA   2400
#define IMU_DROP_Z_RECOVER_DELTA     500
#define IMU_DROP_Z_ABOVE_DELTA       900
#define IMU_DROP_XY_GRAVITY_MAX      1600
#define IMU_DROP_GYRO_MAX            2500
#define IMU_DROP_FRAME_DELTA_MIN     180
#define IMU_DROP_ROTATE_Y_MIN        2600
#define IMU_DROP_ROTATE_Z_BELOW_DELTA 900
#define IMU_DROP_ROTATE_CURRENT_Y_MAX 1800
#define IMU_DROP_ROTATE_CURRENT_Z_MIN (-1000)
#define IMU_DROP_ROTATE_Z_RISE_DELTA 900

#define ICM42688P_REG_WHO_AM_I    0x01
#define ICM42688P_REG_COM_CFG     0x05
#define ICM42688P_REG_INT_CFG1    0x06
#define ICM42688P_REG_INT_CFG2    0x07
#define ICM42688P_REG_DATA_STAT   0x0B
#define ICM42688P_REG_ACC_XH      0x0C
#define ICM42688P_REG_ACC_CONF    0x40
#define ICM42688P_REG_ACC_RANGE   0x41
#define ICM42688P_REG_GYR_CONF    0x42
#define ICM42688P_REG_GYR_RANGE   0x43
#define ICM42688P_REG_SOFT_RST    0x4A
#define ICM42688P_REG_PWR_CTRL    0x7D
#define ICM42688P_REG_AOI1_CFG    0x30
#define ICM42688P_REG_AOI1_STAT   0x31
#define ICM42688P_REG_AOI1_THS    0x32
#define ICM42688P_REG_AOI1_DUR    0x33
#define ICM42688P_REG_AOI_CFG     0x3F

#define ICM42688P_INT1_ACTIVE_HIGH_AOI1 0x0B
#define ICM42688P_AOI1_6D_Y_HIGH       0x48
#define ICM42688P_AOI1_THS_WAKE        0x08
#define ICM42688P_AOI1_DUR_WAKE        0x08
#define ICM42688P_AOI1_RAW_LATCH       0x01

static i2c_master_bus_handle_t s_imu_i2c_bus;
static i2c_master_dev_handle_t s_imu_i2c_dev;
static imu_sample_cb_t s_sample_callback;
static void *s_sample_user_ctx;
static imu_posture_cb_t s_posture_callback;
static void *s_posture_user_ctx;
static int32_t s_accel_x_bias;
static int32_t s_accel_y_bias;
static int32_t s_accel_z_bias;
static int32_t s_gyro_x_bias;
static int32_t s_gyro_y_bias;
static int32_t s_gyro_z_bias;
static int32_t s_filtered_accel_x;
static int32_t s_filtered_accel_y;
static int32_t s_filtered_accel_z = IMU_STATIC_FLAT_Z_REF;
static int32_t s_filtered_gyro_x;
static int32_t s_filtered_gyro_y;
static int32_t s_filtered_gyro_z;
static bool s_filter_initialized;
static int16_t s_flat_acc_z_ref = IMU_STATIC_FLAT_Z_REF;
static int16_t s_prev_accel_x;
static int16_t s_prev_accel_y;
static int16_t s_prev_accel_z = IMU_STATIC_FLAT_Z_REF;
static bool s_prev_accel_valid;
static bool s_was_flat = true;
static bool s_flat_latched = true;
static int s_flat_enter_confirm_frames;
static int s_flat_exit_confirm_frames;
static int s_stable_flat_frames;
static int s_lift_armed_frames;
static int s_lift_hold_frames;
static int s_lift_active_frames;
static int s_lift_context_frames;
static int s_drop_hold_frames;
static int s_shake_hold_frames;
static int s_lift_shake_suppress_frames;
static int s_lift_candidate_frames;
static int s_drop_candidate_frames;
static TaskHandle_t s_imu_task_handle;
static volatile uint32_t s_imu_int_count;
static volatile bool s_imu_int_armed = true;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int32_t acc_mag_delta;
    int32_t frame_delta;
    bool flat;
} imu_motion_window_entry_t;

static imu_motion_window_entry_t s_motion_window[IMU_MOTION_WINDOW_SIZE];
static int s_motion_window_index;
static int s_motion_window_count;

typedef enum {
    IMU_MOTION_IDLE = 0,
    IMU_MOTION_STILL,
    IMU_MOTION_LIFT,
    IMU_MOTION_DROP,
    IMU_MOTION_SHAKE,
} imu_motion_state_t;

#if IMU_TRACE_LOGS
static const char *imu_motion_state_name(imu_motion_state_t state)
{
    switch (state) {
    case IMU_MOTION_STILL:
        return "STILL";
    case IMU_MOTION_LIFT:
        return "LIFT";
    case IMU_MOTION_DROP:
        return "IDLE";
    case IMU_MOTION_SHAKE:
        return "SHAKE";
    case IMU_MOTION_IDLE:
    default:
        return "IDLE";
    }
}
#endif

static imu_posture_t imu_posture_from_motion_state(imu_motion_state_t state)
{
    switch (state) {
    case IMU_MOTION_STILL:
        return IMU_POSTURE_STILL;
    case IMU_MOTION_LIFT:
        return IMU_POSTURE_LIFT;
    case IMU_MOTION_DROP:
        return IMU_POSTURE_IDLE;
    case IMU_MOTION_SHAKE:
        return IMU_POSTURE_SHAKE;
    case IMU_MOTION_IDLE:
    default:
        return IMU_POSTURE_IDLE;
    }
}

static int16_t be16_to_i16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] << 8 | data[1]);
}

static esp_err_t imu_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(s_imu_i2c_dev, &reg, 1, value, 1, CONFIG_IMU_I2C_TIMEOUT_MS);
}

static esp_err_t imu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(s_imu_i2c_dev, data, sizeof(data), CONFIG_IMU_I2C_TIMEOUT_MS);
}

static esp_err_t imu_read_data(imu_sample_t *sample)
{
    uint8_t reg = ICM42688P_REG_DATA_STAT;
    uint8_t data[13] = {0};
    esp_err_t ret = i2c_master_transmit_receive(s_imu_i2c_dev,
                                                &reg,
                                                1,
                                                data,
                                                sizeof(data),
                                                CONFIG_IMU_I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return ret;
    }

    sample->data_status = data[0];
    sample->accel_x = be16_to_i16(&data[1]);
    sample->accel_y = be16_to_i16(&data[3]);
    sample->accel_z = be16_to_i16(&data[5]);
    sample->gyro_x = be16_to_i16(&data[7]);
    sample->gyro_y = be16_to_i16(&data[9]);
    sample->gyro_z = be16_to_i16(&data[11]);
    return ESP_OK;
}

static int32_t abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int32_t max3_i32(int32_t a, int32_t b, int32_t c)
{
    int32_t max = a > b ? a : b;
    return max > c ? max : c;
}

static int32_t isqrt_i64(int64_t value)
{
    int64_t root = 0;
    int64_t bit = 1LL << 62;

    while (bit > value) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (value >= root + bit) {
            value -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }

    return (int32_t)root;
}

static bool imu_update_flat_latch(int32_t abs_ax, int32_t abs_ay, int32_t abs_az, int32_t gyro_max)
{
    bool flat_enter = gyro_max <= IMU_FLAT_ENTER_GYRO_MAX &&
                      abs_ax <= IMU_FLAT_ENTER_XY_MAX &&
                      abs_ay <= IMU_FLAT_ENTER_XY_MAX &&
                      abs_az >= IMU_FLAT_ENTER_Z_MIN;
    bool flat_hold = gyro_max <= IMU_FLAT_EXIT_GYRO_MAX &&
                     abs_ax <= IMU_FLAT_EXIT_XY_MAX &&
                     abs_ay <= IMU_FLAT_EXIT_XY_MAX &&
                     abs_az >= IMU_FLAT_EXIT_Z_MIN;
    bool flat_force_exit = abs_ay >= IMU_LIFT_Y_GRAVITY_MIN ||
                           abs_az < IMU_FLAT_EXIT_Z_MIN;

    if (s_flat_latched) {
        if (flat_force_exit) {
            s_flat_latched = false;
            s_flat_enter_confirm_frames = 0;
            s_flat_exit_confirm_frames = 0;
            return false;
        }
        if (!flat_hold) {
            if (s_flat_exit_confirm_frames < IMU_FLAT_EXIT_CONFIRM_FRAMES) {
                s_flat_exit_confirm_frames++;
            }
            if (s_flat_exit_confirm_frames >= IMU_FLAT_EXIT_CONFIRM_FRAMES) {
                s_flat_latched = false;
                s_flat_enter_confirm_frames = 0;
            }
        } else {
            s_flat_exit_confirm_frames = 0;
        }
        return s_flat_latched;
    }

    s_flat_exit_confirm_frames = 0;

    if (flat_enter) {
        if (s_flat_enter_confirm_frames < IMU_FLAT_ENTER_CONFIRM_FRAMES) {
            s_flat_enter_confirm_frames++;
        }
        if (s_flat_enter_confirm_frames >= IMU_FLAT_ENTER_CONFIRM_FRAMES) {
            s_flat_latched = true;
        }
    } else {
        s_flat_enter_confirm_frames = 0;
    }

    return s_flat_latched;
}

static int16_t clamp_i16_from_i32(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static int sign_with_threshold(int32_t value, int32_t threshold)
{
    if (value >= threshold) {
        return 1;
    }
    if (value <= -threshold) {
        return -1;
    }
    return 0;
}

#if CONFIG_IMU_USE_INT_READ
static void IRAM_ATTR imu_int_isr(void *arg)
{
    (void)arg;

    if (!s_imu_int_armed) {
        return;
    }

    s_imu_int_armed = false;
    gpio_intr_disable(IMU_INT_GPIO);
    s_imu_int_count++;
    if (s_imu_task_handle) {
        BaseType_t high_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_imu_task_handle, &high_task_woken);
        if (high_task_woken) {
            portYIELD_FROM_ISR();
        }
    }
}
#endif

static void imu_apply_calibration_and_filter(const imu_sample_t *raw, imu_sample_t *out)
{
    imu_sample_t corrected = {
        .accel_x = clamp_i16_from_i32((int32_t)raw->accel_x - s_accel_x_bias),
        .accel_y = clamp_i16_from_i32((int32_t)raw->accel_y - s_accel_y_bias),
        .accel_z = clamp_i16_from_i32((int32_t)raw->accel_z - s_accel_z_bias),
        .gyro_x = clamp_i16_from_i32((int32_t)raw->gyro_x - s_gyro_x_bias),
        .gyro_y = clamp_i16_from_i32((int32_t)raw->gyro_y - s_gyro_y_bias),
        .gyro_z = clamp_i16_from_i32((int32_t)raw->gyro_z - s_gyro_z_bias),
        .data_status = raw->data_status,
    };

    if (!s_filter_initialized) {
        s_filtered_accel_x = corrected.accel_x;
        s_filtered_accel_y = corrected.accel_y;
        s_filtered_accel_z = corrected.accel_z;
        s_filtered_gyro_x = corrected.gyro_x;
        s_filtered_gyro_y = corrected.gyro_y;
        s_filtered_gyro_z = corrected.gyro_z;
        s_filter_initialized = true;
    } else {
        s_filtered_accel_x += ((int32_t)corrected.accel_x - s_filtered_accel_x) >> IMU_FILTER_SHIFT;
        s_filtered_accel_y += ((int32_t)corrected.accel_y - s_filtered_accel_y) >> IMU_FILTER_SHIFT;
        s_filtered_accel_z += ((int32_t)corrected.accel_z - s_filtered_accel_z) >> IMU_FILTER_SHIFT;
        s_filtered_gyro_x += ((int32_t)corrected.gyro_x - s_filtered_gyro_x) >> IMU_FILTER_SHIFT;
        s_filtered_gyro_y += ((int32_t)corrected.gyro_y - s_filtered_gyro_y) >> IMU_FILTER_SHIFT;
        s_filtered_gyro_z += ((int32_t)corrected.gyro_z - s_filtered_gyro_z) >> IMU_FILTER_SHIFT;
    }

    out->accel_x = clamp_i16_from_i32(s_filtered_accel_x);
    out->accel_y = clamp_i16_from_i32(s_filtered_accel_y);
    out->accel_z = clamp_i16_from_i32(s_filtered_accel_z);
    out->gyro_x = clamp_i16_from_i32(s_filtered_gyro_x);
    out->gyro_y = clamp_i16_from_i32(s_filtered_gyro_y);
    out->gyro_z = clamp_i16_from_i32(s_filtered_gyro_z);
    out->data_status = raw->data_status;
}

static imu_motion_state_t imu_classify_motion(const imu_sample_t *sample)
{
    int32_t abs_ax = abs_i32(sample->accel_x);
    int32_t abs_ay = abs_i32(sample->accel_y);
    int32_t abs_az = abs_i32(sample->accel_z);
    int32_t gyro_max = max3_i32(abs_i32(sample->gyro_x),
                                abs_i32(sample->gyro_y),
                                abs_i32(sample->gyro_z));
    int32_t acc_mag = isqrt_i64((int64_t)sample->accel_x * sample->accel_x +
                                (int64_t)sample->accel_y * sample->accel_y +
                                (int64_t)sample->accel_z * sample->accel_z);
    int32_t acc_mag_delta = abs_i32(acc_mag - IMU_ONE_G_RAW);
    int32_t frame_delta = 0;
    bool is_flat = imu_update_flat_latch(abs_ax, abs_ay, abs_az, gyro_max);
    bool lift_armed = s_lift_armed_frames > 0;
    bool shake_now = false;
    bool lift_now = false;
    int32_t z_below_ref = (int32_t)s_flat_acc_z_ref - sample->accel_z;

    if (s_prev_accel_valid) {
        frame_delta = max3_i32(abs_i32((int32_t)sample->accel_x - s_prev_accel_x),
                               abs_i32((int32_t)sample->accel_y - s_prev_accel_y),
                               abs_i32((int32_t)sample->accel_z - s_prev_accel_z));
    }

    s_motion_window[s_motion_window_index] = (imu_motion_window_entry_t) {
        .accel_x = sample->accel_x,
        .accel_y = sample->accel_y,
        .accel_z = sample->accel_z,
        .gyro_x = sample->gyro_x,
        .gyro_y = sample->gyro_y,
        .gyro_z = sample->gyro_z,
        .acc_mag_delta = acc_mag_delta,
        .frame_delta = frame_delta,
        .flat = is_flat,
    };
    s_motion_window_index = (s_motion_window_index + 1) % IMU_MOTION_WINDOW_SIZE;
    if (s_motion_window_count < IMU_MOTION_WINDOW_SIZE) {
        s_motion_window_count++;
    }

    int32_t window_gyro_max = gyro_max;
    int32_t window_acc_mag_delta_max = acc_mag_delta;
    int32_t window_frame_delta_max = frame_delta;
    int32_t window_z_min = sample->accel_z;
    int32_t window_z_max = sample->accel_z;
    int32_t window_abs_x_max = abs_ax;
    int32_t window_abs_y_max = abs_ay;
    int shake_hit_count = 0;
    int gyro_x_sign_last = 0;
    int gyro_y_sign_last = 0;
    int gyro_z_sign_last = 0;
    int gyro_x_reversals = 0;
    int gyro_y_reversals = 0;
    int gyro_z_reversals = 0;

    for (int i = 0; i < s_motion_window_count; i++) {
        const imu_motion_window_entry_t *entry = &s_motion_window[i];
        int32_t entry_gyro_max = max3_i32(abs_i32(entry->gyro_x),
                                          abs_i32(entry->gyro_y),
                                          abs_i32(entry->gyro_z));
        int32_t entry_abs_x = abs_i32(entry->accel_x);
        int32_t entry_abs_y = abs_i32(entry->accel_y);
        int gyro_x_sign = sign_with_threshold(entry->gyro_x, IMU_SHAKE_REVERSAL_GYRO_MIN);
        int gyro_y_sign = sign_with_threshold(entry->gyro_y, IMU_SHAKE_REVERSAL_GYRO_MIN);
        int gyro_z_sign = sign_with_threshold(entry->gyro_z, IMU_SHAKE_REVERSAL_GYRO_MIN);
        if (gyro_x_sign != 0) {
            if (gyro_x_sign_last != 0 && gyro_x_sign != gyro_x_sign_last) {
                gyro_x_reversals++;
            }
            gyro_x_sign_last = gyro_x_sign;
        }
        if (gyro_y_sign != 0) {
            if (gyro_y_sign_last != 0 && gyro_y_sign != gyro_y_sign_last) {
                gyro_y_reversals++;
            }
            gyro_y_sign_last = gyro_y_sign;
        }
        if (gyro_z_sign != 0) {
            if (gyro_z_sign_last != 0 && gyro_z_sign != gyro_z_sign_last) {
                gyro_z_reversals++;
            }
            gyro_z_sign_last = gyro_z_sign;
        }
        if (entry_gyro_max > window_gyro_max) {
            window_gyro_max = entry_gyro_max;
        }
        if (entry_abs_x > window_abs_x_max) {
            window_abs_x_max = entry_abs_x;
        }
        if (entry_abs_y > window_abs_y_max) {
            window_abs_y_max = entry_abs_y;
        }
        if (entry->acc_mag_delta > window_acc_mag_delta_max) {
            window_acc_mag_delta_max = entry->acc_mag_delta;
        }
        if (entry->frame_delta > window_frame_delta_max) {
            window_frame_delta_max = entry->frame_delta;
        }
        if (entry->accel_z < window_z_min) {
            window_z_min = entry->accel_z;
        }
        if (entry->accel_z > window_z_max) {
            window_z_max = entry->accel_z;
        }
        if (entry_gyro_max >= IMU_SHAKE_GYRO_MAX ||
            entry->acc_mag_delta >= IMU_SHAKE_ACC_MAG_DELTA ||
            entry->frame_delta >= IMU_SHAKE_ACC_FRAME_DELTA) {
            shake_hit_count++;
        }
    }

    int32_t window_z_below_ref = (int32_t)s_flat_acc_z_ref - window_z_min;
    int32_t window_z_above_ref = window_z_max - (int32_t)s_flat_acc_z_ref;
    bool window_ready = s_motion_window_count >= IMU_MOTION_WINDOW_MIN_FRAMES;
    bool shake_has_reversal = gyro_x_reversals >= IMU_SHAKE_REVERSAL_MIN_COUNT ||
                              gyro_y_reversals >= IMU_SHAKE_REVERSAL_MIN_COUNT ||
                              gyro_z_reversals >= IMU_SHAKE_REVERSAL_MIN_COUNT;
    bool still_now = window_ready &&
                     window_gyro_max <= IMU_STILL_GYRO_MAX &&
                     window_frame_delta_max <= IMU_STILL_ACC_FRAME_DELTA_MAX &&
                     window_acc_mag_delta_max <= IMU_STILL_ACC_MAG_DELTA_MAX;
    bool y_gravity_lift = abs_ay >= IMU_LIFT_Y_GRAVITY_MIN &&
                          window_abs_y_max >= IMU_LIFT_Y_GRAVITY_MIN;
    bool z_drop_lift = z_below_ref >= IMU_LIFT_Z_BELOW_REF_DELTA &&
                       window_z_below_ref >= IMU_LIFT_Z_BELOW_REF_DELTA;
    bool current_lift_pose = z_below_ref >= IMU_LIFT_Z_BELOW_REF_DELTA &&
                             abs_ay >= IMU_LIFT_Y_GRAVITY_MIN;

    lift_now = lift_armed &&
               !is_flat &&
               ((window_ready &&
                 z_drop_lift &&
                 y_gravity_lift &&
                 (window_frame_delta_max >= IMU_LIFT_FRAME_DELTA_MIN ||
                  window_gyro_max >= IMU_LIFT_GYRO_MIN ||
                  (window_z_max - window_z_min) >= IMU_LIFT_Z_LEAVE_DELTA ||
                  y_gravity_lift)) ||
                current_lift_pose);

    bool lift_candidate_now = window_ready &&
                              lift_armed &&
                              (window_frame_delta_max >= IMU_LIFT_FRAME_DELTA_MIN ||
                               window_gyro_max >= IMU_LIFT_GYRO_MIN) &&
                              (window_z_below_ref >= IMU_LIFT_Z_BELOW_REF_DELTA ||
                               window_z_above_ref >= IMU_LIFT_CANDIDATE_Z_DELTA ||
                               window_abs_y_max >= IMU_LIFT_CANDIDATE_AXIS_MIN ||
                               window_abs_x_max >= IMU_LIFT_CANDIDATE_AXIS_MIN);

    bool z_recover_drop = sample->accel_z >= ((int32_t)s_flat_acc_z_ref - IMU_DROP_Z_RECOVER_DELTA) &&
                          sample->accel_z <= ((int32_t)s_flat_acc_z_ref + IMU_DROP_Z_ABOVE_DELTA);
    bool xy_gravity_drop = abs_ax <= IMU_DROP_XY_GRAVITY_MAX &&
                           abs_ay <= IMU_DROP_XY_GRAVITY_MAX;
    bool gyro_quiet_drop = gyro_max <= IMU_DROP_GYRO_MAX;
    bool drop_pose_now = z_recover_drop &&
                         xy_gravity_drop &&
                         gyro_quiet_drop &&
                         (is_flat ||
                          window_frame_delta_max >= IMU_DROP_FRAME_DELTA_MIN ||
                          window_gyro_max >= IMU_LIFT_GYRO_MIN);
    bool drop_allowed = s_lift_context_frames > 0;
    bool drop_candidate_now = window_ready &&
                              drop_allowed &&
                              drop_pose_now;

    bool lift_related_suppress = lift_armed ||
                                 lift_candidate_now ||
                                 s_lift_candidate_frames > 0 ||
                                 drop_candidate_now ||
                                 s_drop_candidate_frames > 0 ||
                                 s_drop_hold_frames > 0 ||
                                 s_lift_active_frames > 0 ||
                                 s_lift_hold_frames > 0 ||
                                 s_lift_shake_suppress_frames > 0 ||
                                 (is_flat && !s_was_flat);

    shake_now = window_ready &&
                !lift_now &&
                s_lift_active_frames == 0 &&
                s_lift_candidate_frames == 0 &&
                !lift_candidate_now &&
                s_drop_candidate_frames == 0 &&
                !drop_candidate_now &&
                !lift_related_suppress &&
                shake_has_reversal &&
                shake_hit_count >= IMU_SHAKE_HIT_MIN_FRAMES &&
                (window_acc_mag_delta_max >= IMU_SHAKE_ACC_MAG_DELTA ||
                 window_frame_delta_max >= IMU_SHAKE_ACC_FRAME_DELTA);

    if (is_flat) {
        if (s_stable_flat_frames < IMU_LIFT_ARM_FLAT_FRAMES) {
            s_stable_flat_frames++;
        }
        if (s_stable_flat_frames >= IMU_LIFT_ARM_FLAT_FRAMES) {
            s_lift_armed_frames = IMU_LIFT_RECENT_FLAT_FRAMES;
        }
    } else {
        s_stable_flat_frames = 0;
        if (s_lift_armed_frames > 0) {
            s_lift_armed_frames--;
        }
    }

    s_prev_accel_x = sample->accel_x;
    s_prev_accel_y = sample->accel_y;
    s_prev_accel_z = sample->accel_z;
    s_prev_accel_valid = true;

    if (s_lift_shake_suppress_frames > 0) {
        s_lift_shake_suppress_frames--;
    }
    bool return_to_flat_after_lift = is_flat &&
                                     !s_was_flat &&
                                     s_lift_context_frames > 0;
    if (is_flat && !s_was_flat) {
        s_lift_shake_suppress_frames = IMU_LIFT_DROP_SUPPRESS_FRAMES;
        if (return_to_flat_after_lift) {
            s_drop_hold_frames = IMU_DROP_HOLD_FRAMES;
        }
        s_lift_hold_frames = 0;
        s_lift_active_frames = 0;
        s_lift_context_frames = 0;
        s_lift_candidate_frames = 0;
        s_shake_hold_frames = 0;
    }
    if (s_drop_candidate_frames > 0) {
        s_drop_candidate_frames--;
    }
    if (drop_candidate_now) {
        s_drop_candidate_frames = IMU_DROP_CANDIDATE_FRAMES;
        s_drop_hold_frames = IMU_DROP_HOLD_FRAMES;
        s_lift_shake_suppress_frames = IMU_LIFT_DROP_SUPPRESS_FRAMES;
        s_lift_hold_frames = 0;
        s_lift_active_frames = 0;
        s_lift_context_frames = 0;
        s_lift_candidate_frames = 0;
        s_shake_hold_frames = 0;
    }
    if (s_lift_candidate_frames > 0) {
        s_lift_candidate_frames--;
    }
    if (lift_candidate_now) {
        s_lift_candidate_frames = IMU_LIFT_CANDIDATE_FRAMES;
    }

    if (shake_now && s_lift_shake_suppress_frames == 0) {
        s_shake_hold_frames = IMU_ACTION_HOLD_FRAMES;
        s_lift_hold_frames = 0;
        s_lift_active_frames = 0;
        s_lift_candidate_frames = 0;
    } else if (lift_now) {
        s_lift_hold_frames = IMU_ACTION_HOLD_FRAMES;
        s_lift_active_frames = IMU_LIFT_ACTIVE_FRAMES;
        s_lift_context_frames = IMU_LIFT_CONTEXT_FRAMES;
        s_lift_shake_suppress_frames = IMU_LIFT_SHAKE_SUPPRESS_FRAMES;
        s_lift_candidate_frames = 0;
    } else {
        if (s_shake_hold_frames > 0) {
            s_shake_hold_frames--;
        }
        if (s_lift_hold_frames > 0) {
            s_lift_hold_frames--;
        }
        if (s_drop_hold_frames > 0) {
            s_drop_hold_frames--;
        }
        if (is_flat) {
            s_lift_active_frames = 0;
        } else if (s_lift_active_frames > 0) {
            s_lift_active_frames--;
        }
    }

    s_was_flat = is_flat;

    if (is_flat && s_drop_hold_frames > 0) {
        s_drop_hold_frames = 0;
        s_drop_candidate_frames = 0;
        return still_now ? IMU_MOTION_STILL : IMU_MOTION_IDLE;
    }

    if (s_drop_hold_frames > 0) {
        return IMU_MOTION_IDLE;
    }

    if (s_shake_hold_frames > 0) {
        return IMU_MOTION_SHAKE;
    }

    if (s_lift_hold_frames > 0 || s_lift_active_frames > 0) {
        return IMU_MOTION_LIFT;
    }

    if (still_now) {
        return IMU_MOTION_STILL;
    }

    return IMU_MOTION_IDLE;
}

static esp_err_t imu_add_device(uint8_t address)
{
    s_imu_i2c_dev = NULL;
    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = CONFIG_IMU_I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(s_imu_i2c_bus, &dev_conf, &s_imu_i2c_dev);
}

static void imu_recover_i2c_after_read_error(esp_err_t err, uint32_t fail_count)
{
    if (err != ESP_ERR_TIMEOUT && err != ESP_ERR_INVALID_STATE) {
        return;
    }

    esp_err_t reset_ret = i2c_master_bus_reset(s_imu_i2c_bus);
    if (reset_ret == ESP_OK) {
        if (fail_count <= 5 || (fail_count % 100) == 0) {
            ESP_LOGW(TAG,
                     "i2c bus reset after read error=%s fail_count=%" PRIu32,
                     esp_err_to_name(err),
                     fail_count);
        }
    } else if (fail_count <= 5 || (fail_count % 100) == 0) {
        ESP_LOGW(TAG,
                 "i2c bus reset failed after read error=%s reset=%s fail_count=%" PRIu32,
                 esp_err_to_name(err),
                 esp_err_to_name(reset_ret),
                 fail_count);
    }

    vTaskDelay(pdMS_TO_TICKS(2));
}

static esp_err_t imu_probe_device(void)
{
    uint8_t who = 0;
    esp_err_t ret = imu_add_device(IMU_I2C_ADDR_PRIMARY);
    if (ret == ESP_OK) {
        ret = imu_read_reg(ICM42688P_REG_WHO_AM_I, &who);
        if (ret == ESP_OK && who == IMU_WHO_AM_I_VALUE) {
            ESP_LOGI(TAG, "ICM-42688P found addr=0x%02x who=0x%02x", IMU_I2C_ADDR_PRIMARY, who);
            return ESP_OK;
        }
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_rm_device(s_imu_i2c_dev));
        s_imu_i2c_dev = NULL;
    }

    ret = imu_add_device(IMU_I2C_ADDR_SECONDARY);
    if (ret == ESP_OK) {
        ret = imu_read_reg(ICM42688P_REG_WHO_AM_I, &who);
        if (ret == ESP_OK && who == IMU_WHO_AM_I_VALUE) {
            ESP_LOGI(TAG, "ICM-42688P found addr=0x%02x who=0x%02x", IMU_I2C_ADDR_SECONDARY, who);
            return ESP_OK;
        }
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_rm_device(s_imu_i2c_dev));
        s_imu_i2c_dev = NULL;
    }

    ESP_LOGW(TAG, "ICM-42688P probe failed, who=0x%02x err=%s", who, esp_err_to_name(ret));
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t imu_configure(void)
{
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_SOFT_RST, 0xA5), TAG, "soft reset failed");
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_PWR_CTRL, 0x0E), TAG, "power on failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_COM_CFG, 0x50), TAG, "com config failed");
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_ACC_RANGE, 0x02), TAG, "acc range failed");
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_GYR_RANGE, 0x00), TAG, "gyro range failed");
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_ACC_CONF, 0xAC), TAG, "acc config failed");
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_GYR_CONF, 0xAC), TAG, "gyro config failed");
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_INT_CFG1, 0x00), TAG, "int1 disable failed");
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_INT_CFG2, 0x00), TAG, "int2 disable failed");
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_AOI1_CFG, ICM42688P_AOI1_6D_Y_HIGH),
                        TAG,
                        "aoi1 cfg failed");
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_AOI1_THS, ICM42688P_AOI1_THS_WAKE),
                        TAG,
                        "aoi1 threshold failed");
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_AOI1_DUR, ICM42688P_AOI1_DUR_WAKE),
                        TAG,
                        "aoi1 duration failed");
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_AOI_CFG, ICM42688P_AOI1_RAW_LATCH),
                        TAG,
                        "aoi common cfg failed");
    uint8_t aoi_stat = 0;
    (void)imu_read_reg(ICM42688P_REG_AOI1_STAT, &aoi_stat);
    ESP_RETURN_ON_ERROR(imu_write_reg(ICM42688P_REG_INT_CFG1, ICM42688P_INT1_ACTIVE_HIGH_AOI1),
                        TAG,
                        "int1 aoi config failed");
    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

static esp_err_t imu_apply_static_calibration(void)
{
    int32_t acc_x_avg = IMU_STATIC_FLAT_ACC_X_RAW;
    int32_t acc_y_avg = IMU_STATIC_FLAT_ACC_Y_RAW;
    int32_t acc_z_avg = IMU_STATIC_FLAT_ACC_Z_RAW;

    s_flat_acc_z_ref = IMU_STATIC_FLAT_Z_REF;
    s_accel_x_bias = acc_x_avg;
    s_accel_y_bias = acc_y_avg;
    s_accel_z_bias = acc_z_avg - s_flat_acc_z_ref;
    s_gyro_x_bias = IMU_STATIC_FLAT_GYRO_X_RAW;
    s_gyro_y_bias = IMU_STATIC_FLAT_GYRO_Y_RAW;
    s_gyro_z_bias = IMU_STATIC_FLAT_GYRO_Z_RAW;
    s_filter_initialized = false;

    ESP_LOGI(TAG,
             "static calibration: flat_raw=(%" PRId32 ",%" PRId32 ",%" PRId32 ") gyro_bias=(%" PRId32 ",%" PRId32 ",%" PRId32 ") acc_bias=(%" PRId32 ",%" PRId32 ",%" PRId32 ") z_ref=%d",
             acc_x_avg,
             acc_y_avg,
             acc_z_avg,
             s_gyro_x_bias,
             s_gyro_y_bias,
             s_gyro_z_bias,
             s_accel_x_bias,
             s_accel_y_bias,
             s_accel_z_bias,
             s_flat_acc_z_ref);
    return ESP_OK;
}

static void imu_task(void *arg)
{
    (void)arg;

    s_imu_task_handle = xTaskGetCurrentTaskHandle();
    ESP_ERROR_CHECK_WITHOUT_ABORT(imu_apply_static_calibration());

    uint32_t fail_count = 0;
#if CONFIG_IMU_USE_INT_READ
    uint32_t int_burst_reads = 0;
#endif
    TickType_t last_log_tick = 0;
    imu_motion_state_t last_state = IMU_MOTION_IDLE;
    imu_motion_state_t last_posture_state = IMU_MOTION_IDLE;
    TickType_t posture_enter_tick = 0;
    bool posture_initialized = false;
    TickType_t last_still_notify_tick = 0;
    while (1) {
        imu_sample_t raw = {0};
        imu_sample_t sample = {0};
        esp_err_t ret = imu_read_data(&raw);
        if (ret == ESP_OK) {
            fail_count = 0;
            imu_apply_calibration_and_filter(&raw, &sample);
            imu_motion_state_t state = imu_classify_motion(&sample);
            TickType_t now = xTaskGetTickCount();
            if (!posture_initialized || state != last_posture_state) {
                posture_initialized = true;
                last_posture_state = state;
                posture_enter_tick = now;
                last_still_notify_tick = 0;
                if (s_posture_callback) {
                    s_posture_callback(imu_posture_from_motion_state(state), 0, s_posture_user_ctx);
                }
            }
            uint32_t posture_stable_ms = (uint32_t)((now - posture_enter_tick) * portTICK_PERIOD_MS);
            if (state == IMU_MOTION_STILL &&
                posture_stable_ms >= IMU_STILL_POSTURE_NOTIFY_MS &&
                (last_still_notify_tick == 0 ||
                 (now - last_still_notify_tick) >= pdMS_TO_TICKS(IMU_STILL_POSTURE_NOTIFY_RETRY_MS))) {
                last_still_notify_tick = now;
                if (s_posture_callback) {
                    s_posture_callback(imu_posture_from_motion_state(state),
                                       posture_stable_ms,
                                       s_posture_user_ctx);
                }
            }
            if (state != last_state ||
                (now - last_log_tick) >= pdMS_TO_TICKS(CONFIG_IMU_LOG_INTERVAL_MS)) {
                last_state = state;
                last_log_tick = now;
                IMU_LOGI("state=%s acc=(%d,%d,%d) gyro=(%d,%d,%d) stat=0x%02x int1=%d int2=%d z_ref=%d",
                         imu_motion_state_name(state),
                         sample.accel_x,
                         sample.accel_y,
                         sample.accel_z,
                         sample.gyro_x,
                         sample.gyro_y,
                         sample.gyro_z,
                         sample.data_status,
                         gpio_get_level(CONFIG_IMU_INT1_GPIO),
                         gpio_get_level(CONFIG_IMU_INT2_GPIO),
                         s_flat_acc_z_ref);
            }
            if (s_sample_callback) {
                s_sample_callback(&sample, s_sample_user_ctx);
            }
        } else {
            fail_count++;
            imu_recover_i2c_after_read_error(ret, fail_count);
            if (fail_count <= 5 || (fail_count % 100) == 0) {
                IMU_LOGW("read failed: %s fail_count=%" PRIu32, esp_err_to_name(ret), fail_count);
            }
        }

#if CONFIG_IMU_USE_INT_READ
        if (int_burst_reads > 0) {
            int_burst_reads--;
            vTaskDelay(pdMS_TO_TICKS(CONFIG_IMU_READ_INTERVAL_MS));
            continue;
        }

        s_imu_int_armed = true;
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_intr_enable(IMU_INT_GPIO));
        uint32_t notified = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (notified > 0) {
            s_imu_int_armed = false;
            ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_intr_disable(IMU_INT_GPIO));
            int int1_level = gpio_get_level(CONFIG_IMU_INT1_GPIO);
            int int2_level = gpio_get_level(CONFIG_IMU_INT2_GPIO);
            uint8_t aoi_stat = 0;
            (void)imu_read_reg(ICM42688P_REG_AOI1_STAT, &aoi_stat);
            int_burst_reads = IMU_FRAMES_FROM_MS(IMU_INT_BURST_READ_MS);
            IMU_LOGI("interrupt wake: count=%" PRIu32 " aoi1=0x%02x int1=%d int2=%d",
                     s_imu_int_count,
                     aoi_stat,
                     int1_level,
                     int2_level);
        }
#else
        vTaskDelay(pdMS_TO_TICKS(CONFIG_IMU_READ_INTERVAL_MS));
#endif
    }
}

esp_err_t imu_module_init(void)
{
    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << CONFIG_IMU_INT1_GPIO) | (1ULL << CONFIG_IMU_INT2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&int_conf), TAG, "imu int gpio failed");

#if CONFIG_IMU_USE_INT_READ
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(IMU_INT_GPIO, GPIO_INTR_HIGH_LEVEL),
                        TAG,
                        "set imu int level failed");
    ESP_RETURN_ON_ERROR(gpio_isr_service_install_once(), TAG, "install imu gpio isr failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(IMU_INT_GPIO, imu_int_isr, NULL),
                        TAG,
                        "add imu int isr failed");
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_intr_disable(IMU_INT_GPIO));
#endif

    ESP_RETURN_ON_ERROR(shared_i2c_get_bus(&s_imu_i2c_bus), TAG, "imu shared i2c bus failed");
    ESP_RETURN_ON_ERROR(imu_probe_device(), TAG, "imu probe failed");
    ESP_RETURN_ON_ERROR(imu_configure(), TAG, "imu configure failed");

    ESP_LOGI(TAG,
             "init OK: SDA=%d SCL=%d INT1=%d INT2=%d freq=%dHz interval=%dms",
             CONFIG_IMU_SDA_GPIO,
             CONFIG_IMU_SCL_GPIO,
             CONFIG_IMU_INT1_GPIO,
             CONFIG_IMU_INT2_GPIO,
             CONFIG_IMU_I2C_FREQ_HZ,
             CONFIG_IMU_READ_INTERVAL_MS);
    return ESP_OK;
}

void imu_module_set_sample_callback(imu_sample_cb_t callback, void *user_ctx)
{
    s_sample_callback = callback;
    s_sample_user_ctx = user_ctx;
}

void imu_module_set_posture_callback(imu_posture_cb_t callback, void *user_ctx)
{
    s_posture_callback = callback;
    s_posture_user_ctx = user_ctx;
}

esp_err_t imu_module_start(void)
{
    ESP_RETURN_ON_FALSE(s_imu_i2c_dev, ESP_ERR_INVALID_STATE, TAG, "imu is not initialized");
    BaseType_t ok = xTaskCreate(imu_task, "imu_task", 3072, NULL, 4, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create imu task failed");
    return ESP_OK;
}
