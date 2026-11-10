#pragma once

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef int esp_err_t;
enum {
    ESP_OK = 0,
    ESP_FAIL = -1,
    ESP_ERR_NO_MEM = 0x101,
    ESP_ERR_INVALID_ARG = 0x102,
    ESP_ERR_INVALID_STATE = 0x103,
    ESP_ERR_INVALID_SIZE = 0x104,
    ESP_ERR_NOT_SUPPORTED = 0x106,
};

typedef uint32_t TickType_t;
typedef struct { unsigned depth; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define pdMS_TO_TICKS(milliseconds) ((TickType_t)(milliseconds))

extern TickType_t uac_test_tick;

static inline TickType_t xTaskGetTickCount(void)
{
    return uac_test_tick;
}

static inline void uac_test_enter(portMUX_TYPE *lock)
{
    ++lock->depth;
}

static inline void uac_test_exit(portMUX_TYPE *lock)
{
    assert(lock->depth > 0U);
    --lock->depth;
}

#define portENTER_CRITICAL(lock) uac_test_enter(lock)
#define portEXIT_CRITICAL(lock) uac_test_exit(lock)
#define portENTER_CRITICAL_SAFE(lock) uac_test_enter(lock)
#define portEXIT_CRITICAL_SAFE(lock) uac_test_exit(lock)
static inline bool xPortInIsrContext(void) { return false; }
static inline TickType_t xTaskGetTickCountFromISR(void) { return uac_test_tick; }

static inline void uac_test_log(const char *tag, const char *format, ...)
{
    (void)tag;
    (void)format;
}

#define ESP_LOGI(tag, ...) uac_test_log(tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) uac_test_log(tag, __VA_ARGS__)
#define ESP_LOGD(tag, ...) uac_test_log(tag, __VA_ARGS__)
#define ESP_RETURN_ON_FALSE(condition, error, tag, ...) do { \
    if (!(condition)) { uac_test_log(tag, __VA_ARGS__); return (error); } \
} while (0)
