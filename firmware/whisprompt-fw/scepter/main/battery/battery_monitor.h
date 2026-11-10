#pragma once

#include <stdbool.h>

#include "esp_err.h"

typedef struct {
    int voltage_mv;
    int percent;
    bool charging;
    bool valid;
} battery_monitor_snapshot_t;

esp_err_t battery_monitor_start(void);
bool battery_monitor_get_snapshot(battery_monitor_snapshot_t *snapshot);
void battery_monitor_force_next_sync(void);
