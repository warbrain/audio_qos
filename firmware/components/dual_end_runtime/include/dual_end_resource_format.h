#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define DUAL_END_RESOURCE_PARTS 10U
typedef struct {
    uint8_t role;
    uint32_t boot_id, sequence;
    uint64_t sampled_us, interval_us;
    int32_t cpu_busy_permille[2]; /* -1 means unavailable/first observation. */
    uint32_t internal_free, internal_min, internal_largest;
    uint32_t psram_free, psram_min, psram_largest;
    uint32_t logger_stack, telemetry_drop;
    bool io_available;
    uint32_t io[8];
} dual_end_resource_snapshot_t;
/* Idle counters are 32-bit ESP timer microseconds. Modular subtraction handles
 * one wrap; windows >=2^32 us or impossible deltas return unknown, never 0%. */
int32_t dual_end_resource_cpu(uint32_t before, uint32_t after, uint64_t window_us);
size_t dual_end_resource_format(const dual_end_resource_snapshot_t *snapshot,
    unsigned part, char *buffer, size_t capacity);
