#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#endif
#ifndef SOC_CPU_CORES_NUM
#define SOC_CPU_CORES_NUM 2
#endif
#if defined(CONFIG_FREERTOS_UNICORE) && CONFIG_FREERTOS_UNICORE
#define DUAL_END_POLICY_UNICORE 1
#else
#define DUAL_END_POLICY_UNICORE 0
#endif
/* Keep both synchronized codecs on one audio owner; Wi-Fi/USB/control on 0.
 * A single-core build safely maps both roles to core 0. Priorities stay local. */
#define DUAL_END_AUDIO_TASK_CORE ((DUAL_END_POLICY_UNICORE || SOC_CPU_CORES_NUM < 2) ? 0 : 1)
#define DUAL_END_COMM_TASK_CORE 0
static inline unsigned dual_end_policy_audio_core(unsigned cores, bool unicore)
{
    return unicore || cores < 2 ? 0U : 1U;
}
static inline unsigned dual_end_policy_comm_core(void) { return 0U; }
