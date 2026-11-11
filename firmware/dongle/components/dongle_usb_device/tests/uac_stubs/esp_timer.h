#pragma once
#include "uac_test_boundary.h"
static inline int64_t esp_timer_get_time(void) { return (int64_t)uac_test_tick * 1000LL; }
