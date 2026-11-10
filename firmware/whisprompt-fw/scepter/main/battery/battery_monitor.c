#include "battery_monitor.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "lcd_display.h"
#include "sdkconfig.h"

#define BATTERY_ADC_ATTEN   ADC_ATTEN_DB_12
#define BATTERY_ADC_SAMPLES 16
#define BATTERY_RISE_IGNORE_MV 20
#define BATTERY_RISE_CONFIRM_MV 40
#define BATTERY_RISE_CONFIRM_SAMPLES 3
#define BATTERY_RISE_STEP_MV 10
#define BATTERY_UNPLUG_DETECT_DELAY_MS 1000
#define BATTERY_TASK_STACK_BYTES 3072
#define BATTERY_TASK_PRIORITY 2
#define BATTERY_RUNTIME_LOGS 0

#ifndef CONFIG_BATTERY_ADC_GPIO
#define CONFIG_BATTERY_ADC_GPIO 4
#endif

#ifndef CONFIG_BATTERY_CHRG_GPIO
#define CONFIG_BATTERY_CHRG_GPIO 11
#endif

#ifndef CONFIG_BATTERY_LOG_INTERVAL_MS
#define CONFIG_BATTERY_LOG_INTERVAL_MS 1000
#endif

#ifndef CONFIG_BATTERY_DIVIDER_TOP_KOHM
#define CONFIG_BATTERY_DIVIDER_TOP_KOHM 100
#endif

#ifndef CONFIG_BATTERY_DIVIDER_BOTTOM_KOHM
#define CONFIG_BATTERY_DIVIDER_BOTTOM_KOHM 100
#endif

#ifndef CONFIG_BATTERY_EMPTY_MV
#define CONFIG_BATTERY_EMPTY_MV 3000
#endif

#ifndef CONFIG_BATTERY_FULL_MV
#define CONFIG_BATTERY_FULL_MV 4100
#endif

#ifndef CONFIG_BATTERY_CHARGING_LIFT_MV
/* The 100k/100k divider is already undone by battery_read_mv(), so this
 * compensation is expressed at the battery terminal, not at the ADC pin. */
#define CONFIG_BATTERY_CHARGING_LIFT_MV 60
#endif

static const char *TAG = "battery_monitor";

static adc_oneshot_unit_handle_t s_battery_adc;
static adc_channel_t s_battery_adc_channel;
static adc_cali_handle_t s_battery_adc_cali;
static bool s_battery_adc_calibrated;
static int s_battery_display_mv = -1;
static int s_battery_rise_candidate_mv = -1;
static int s_battery_rise_confirm_count;
static volatile bool s_battery_force_sync_once;
static TaskHandle_t s_battery_task_handle;
static portMUX_TYPE s_battery_lock = portMUX_INITIALIZER_UNLOCKED;
static battery_monitor_snapshot_t s_battery_snapshot;

static void battery_store_snapshot(int voltage_mv, int percent, bool charging, bool valid)
{
    portENTER_CRITICAL(&s_battery_lock);
    s_battery_snapshot.voltage_mv = voltage_mv;
    s_battery_snapshot.percent = percent;
    s_battery_snapshot.charging = charging;
    s_battery_snapshot.valid = valid;
    portEXIT_CRITICAL(&s_battery_lock);
}

static esp_err_t battery_charge_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << CONFIG_BATTERY_CHRG_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "charge gpio config failed");
    ESP_LOGI(TAG, "charge detect GPIO=%d active=low", CONFIG_BATTERY_CHRG_GPIO);
    return ESP_OK;
}

static bool battery_is_charging(void)
{
    return gpio_get_level(CONFIG_BATTERY_CHRG_GPIO) == 0;
}

static bool battery_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                                         adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t curve_cfg = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&curve_cfg, &handle);
    if (ret == ESP_OK) {
        calibrated = true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t line_cfg = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&line_cfg, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (!calibrated) {
        ESP_LOGW(TAG, "ADC calibration unavailable, using raw estimate");
    }
    return calibrated;
}

static esp_err_t battery_adc_init(void)
{
    adc_unit_t unit = ADC_UNIT_1;
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(CONFIG_BATTERY_ADC_GPIO, &unit, &s_battery_adc_channel),
                        TAG,
                        "battery gpio is not ADC capable");

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = unit,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_battery_adc), TAG, "adc unit init failed");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_battery_adc, s_battery_adc_channel, &chan_cfg),
                        TAG,
                        "adc channel config failed");

    s_battery_adc_calibrated = battery_adc_calibration_init(unit,
                                                            s_battery_adc_channel,
                                                            BATTERY_ADC_ATTEN,
                                                            &s_battery_adc_cali);
    ESP_LOGI(TAG, "init OK: GPIO=%d unit=%d channel=%d divider=%dk/%dk empty=%dmV full=%dmV",
             CONFIG_BATTERY_ADC_GPIO,
             unit,
             s_battery_adc_channel,
             CONFIG_BATTERY_DIVIDER_TOP_KOHM,
             CONFIG_BATTERY_DIVIDER_BOTTOM_KOHM,
             CONFIG_BATTERY_EMPTY_MV,
             CONFIG_BATTERY_FULL_MV);
    return ESP_OK;
}

static int battery_percent_from_mv(int mv)
{
    int span = CONFIG_BATTERY_FULL_MV - CONFIG_BATTERY_EMPTY_MV;
    if (span <= 0) {
        return 0;
    }

    int percent = ((mv - CONFIG_BATTERY_EMPTY_MV) * 100 + span / 2) / span;
    if (percent < 1) {
        return 1;
    }
    if (percent > 100) {
        return 100;
    }
    return percent;
}

static esp_err_t battery_read_mv(int *battery_mv, int *pin_mv_out)
{
    int raw_sum = 0;
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        int raw = 0;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_battery_adc, s_battery_adc_channel, &raw),
                            TAG,
                            "adc read failed");
        raw_sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    int raw_avg = raw_sum / BATTERY_ADC_SAMPLES;
    int pin_mv = (raw_avg * 3300) / 4095;
    if (s_battery_adc_calibrated && s_battery_adc_cali) {
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_battery_adc_cali, raw_avg, &pin_mv),
                            TAG,
                            "adc calibration failed");
    }

    int top = CONFIG_BATTERY_DIVIDER_TOP_KOHM;
    int bottom = CONFIG_BATTERY_DIVIDER_BOTTOM_KOHM;
    *pin_mv_out = pin_mv;
    *battery_mv = (pin_mv * (top + bottom) + bottom / 2) / bottom;
    return ESP_OK;
}

static int battery_filter_display_mv(int measured_mv, bool charging)
{
    if (s_battery_force_sync_once) {
        s_battery_force_sync_once = false;
        s_battery_display_mv = measured_mv;
        s_battery_rise_candidate_mv = -1;
        s_battery_rise_confirm_count = 0;
        return s_battery_display_mv;
    }

    if (s_battery_display_mv < 0) {
        s_battery_display_mv = measured_mv;
        s_battery_rise_candidate_mv = -1;
        s_battery_rise_confirm_count = 0;
        return s_battery_display_mv;
    }

    if (!charging) {
        s_battery_rise_candidate_mv = -1;
        s_battery_rise_confirm_count = 0;
        if (measured_mv >= s_battery_display_mv) {
            return s_battery_display_mv;
        }

        int next_mv = s_battery_display_mv - BATTERY_RISE_STEP_MV;
        s_battery_display_mv = (measured_mv > next_mv) ? measured_mv : next_mv;
        return s_battery_display_mv;
    }

    if (measured_mv <= s_battery_display_mv) {
        s_battery_rise_candidate_mv = -1;
        s_battery_rise_confirm_count = 0;
        return s_battery_display_mv;
    }

    if (measured_mv <= s_battery_display_mv + BATTERY_RISE_IGNORE_MV) {
        s_battery_rise_candidate_mv = -1;
        s_battery_rise_confirm_count = 0;
        return s_battery_display_mv;
    }

    if (measured_mv < s_battery_display_mv + BATTERY_RISE_CONFIRM_MV) {
        return s_battery_display_mv;
    }

    if (s_battery_rise_candidate_mv < 0 ||
        measured_mv < s_battery_rise_candidate_mv - BATTERY_RISE_IGNORE_MV ||
        measured_mv > s_battery_rise_candidate_mv + BATTERY_RISE_IGNORE_MV) {
        s_battery_rise_candidate_mv = measured_mv;
        s_battery_rise_confirm_count = 1;
        return s_battery_display_mv;
    }

    s_battery_rise_confirm_count++;
    if (s_battery_rise_confirm_count >= BATTERY_RISE_CONFIRM_SAMPLES) {
        int next_mv = s_battery_display_mv + BATTERY_RISE_STEP_MV;
        s_battery_display_mv = (measured_mv < next_mv) ? measured_mv : next_mv;
        s_battery_rise_candidate_mv = -1;
        s_battery_rise_confirm_count = 0;
    }

    return s_battery_display_mv;
}

static void battery_task(void *arg)
{
    (void)arg;
    s_battery_task_handle = xTaskGetCurrentTaskHandle();

    if (battery_charge_gpio_init() != ESP_OK || battery_adc_init() != ESP_OK) {
        lcd_display_set_battery(-1, 0, false);
        battery_store_snapshot(0, 0, false, false);
        s_battery_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    bool last_charging = battery_is_charging();
    TickType_t unplug_detect_tick = 0;

    while (1) {
        bool charging = battery_is_charging();
        TickType_t now = xTaskGetTickCount();
        if (!last_charging && charging) {
            s_battery_rise_candidate_mv = -1;
            s_battery_rise_confirm_count = 0;
        }
        if (last_charging && !charging) {
            unplug_detect_tick = now + pdMS_TO_TICKS(BATTERY_UNPLUG_DETECT_DELAY_MS);
            if (s_battery_display_mv >= 0) {
                int percent = battery_percent_from_mv(s_battery_display_mv);
                lcd_display_set_battery(percent, s_battery_display_mv, false);
                battery_store_snapshot(s_battery_display_mv, percent, false, true);
            }
        }
        last_charging = charging;

        if (!charging && unplug_detect_tick != 0) {
            TickType_t remain = unplug_detect_tick - now;
            if ((int32_t)remain > 0) {
                (void)ulTaskNotifyTake(pdTRUE, remain);
                continue;
            }
            unplug_detect_tick = 0;
        }

        int battery_mv = 0;
        int pin_mv = 0;
        esp_err_t ret = battery_read_mv(&battery_mv, &pin_mv);
        if (ret == ESP_OK) {
            int adjusted_mv = battery_mv;
            if (charging) {
                adjusted_mv -= CONFIG_BATTERY_CHARGING_LIFT_MV;
            }
            if (adjusted_mv < 0) {
                adjusted_mv = 0;
            }
            int display_mv = battery_filter_display_mv(adjusted_mv, charging);
            int percent = battery_percent_from_mv(display_mv);
            lcd_display_set_battery(percent, display_mv, charging);
            battery_store_snapshot(display_mv, percent, charging, true);
            if (BATTERY_RUNTIME_LOGS) {
                ESP_LOGI(TAG,
                         "raw=%dmV adjusted=%dmV display=%dmV pin=%dmV %d%% charging=%d compensation=%dmV",
                         battery_mv,
                         adjusted_mv,
                         display_mv,
                         pin_mv,
                         percent,
                         charging,
                         charging ? CONFIG_BATTERY_CHARGING_LIFT_MV : 0);
            }
        } else {
            if (BATTERY_RUNTIME_LOGS) {
                ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(ret));
            }
            lcd_display_set_battery(-1, 0, charging);
            battery_store_snapshot(0, 0, charging, false);
        }
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CONFIG_BATTERY_LOG_INTERVAL_MS));
    }
}

esp_err_t battery_monitor_start(void)
{
    BaseType_t ok = xTaskCreate(battery_task,
                                "battery_task",
                                BATTERY_TASK_STACK_BYTES,
                                NULL,
                                BATTERY_TASK_PRIORITY,
                                &s_battery_task_handle);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create battery task failed");
    return ESP_OK;
}

bool battery_monitor_get_snapshot(battery_monitor_snapshot_t *snapshot)
{
    if (!snapshot) {
        return false;
    }

    portENTER_CRITICAL(&s_battery_lock);
    *snapshot = s_battery_snapshot;
    portEXIT_CRITICAL(&s_battery_lock);
    return snapshot->valid;
}

void battery_monitor_force_next_sync(void)
{
    s_battery_force_sync_once = true;
    TaskHandle_t task = s_battery_task_handle;
    if (task) {
        xTaskNotifyGive(task);
    }
}
