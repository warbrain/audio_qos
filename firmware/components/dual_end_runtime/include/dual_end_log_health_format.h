#pragma once
#include <stddef.h>
#include <stdint.h>
#include "dual_end_log_transport.h"
#define DUAL_END_LOG_HEALTH_PARTS 5U
/* Boot-cumulative diagnostic transport counters, not audio packet loss. */
size_t dual_end_log_health_format(const dual_end_log_transport_stats_t *stats,
    uint32_t boot_id, unsigned role, uint32_t seq, uint64_t sampled_us,
    unsigned part, char *buffer, size_t capacity);
