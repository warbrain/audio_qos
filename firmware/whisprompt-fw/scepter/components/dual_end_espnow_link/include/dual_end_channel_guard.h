#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Tokens identify transactions, not an ordering across coordinator boots.
 * The coordinator uses a random boot seed. Retired transactions stay fenced
 * against late queued SCAN/PROPOSE/COMMIT packets (eight-entry RX event queue). */
#define DUAL_END_CHANNEL_FLAG_MANUAL_TARGET 0x02U
#define DUAL_END_CHANNEL_FLAG_FAILED 0x04U
#define DUAL_END_CHANNEL_RETIRED_TOKENS 16U
typedef struct {
    uint32_t generation;
    uint32_t retired[DUAL_END_CHANNEL_RETIRED_TOKENS];
    uint8_t retired_next, target, phase; /* 0 idle, 1 scanning, 2 offered, 3 committed */
} dual_end_channel_guard_t;

static inline bool dual_end_channel_retired(const dual_end_channel_guard_t *g, uint32_t token)
{
    if (!token) return true;
    for (unsigned i = 0; i < DUAL_END_CHANNEL_RETIRED_TOKENS; ++i)
        if (g->retired[i] == token) return true;
    return false;
}
static inline void dual_end_channel_close(dual_end_channel_guard_t *g)
{
    if (g->generation && !dual_end_channel_retired(g, g->generation)) {
        g->retired[g->retired_next] = g->generation;
        g->retired_next = (g->retired_next + 1U) % DUAL_END_CHANNEL_RETIRED_TOKENS;
    }
    g->phase = 0;
}
static inline bool dual_end_channel_begin(dual_end_channel_guard_t *g, uint32_t token)
{
    if (dual_end_channel_retired(g, token)) return false;
    if (g->phase) return g->generation == token && g->phase == 1;
    g->generation = token;
    g->phase = 1;
    return true;
}
static inline bool dual_end_channel_offer(dual_end_channel_guard_t *g, uint32_t token,
                                          uint8_t target, uint8_t current)
{
    if (target < 1 || target > 11 || target == current || dual_end_channel_retired(g, token))
        return false;
    if (g->phase && (g->generation != token || g->phase == 3 ||
        (g->phase == 2 && g->target != target))) return false;
    g->generation = token;
    g->target = target;
    g->phase = 2;
    return true;
}
static inline bool dual_end_channel_commit(dual_end_channel_guard_t *g, uint32_t token,
                                           uint8_t target)
{
    if (g->phase != 2 || token != g->generation || target != g->target) return false;
    g->phase = 3; /* Duplicates must not move the already armed deadline. */
    return true;
}
static inline bool dual_end_channel_cancel(dual_end_channel_guard_t *g, uint32_t token)
{
    if (!g->phase || token != g->generation) return false;
    dual_end_channel_close(g);
    return true;
}
static inline uint32_t dual_end_channel_next_token(uint32_t current)
{
    ++current;
    return current ? current : 1U;
}
