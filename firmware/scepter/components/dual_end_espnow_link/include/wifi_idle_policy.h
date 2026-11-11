#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Runtime RF duty cycling, independent of Light Sleep and system sleep.
 * The 1 Hz business heartbeat is still owned by input_tx_schedule.h. */
#define DUAL_END_WIFI_IDLE_HOLD_US INT64_C(300000)
#define DUAL_END_WIFI_IDLE_RX_WINDOW_US INT64_C(120000)
#define DUAL_END_WIFI_IDLE_FAULT_HOLD_US INT64_C(2000000)
#define DUAL_END_WIFI_IDLE_RETRY_US INT64_C(250000)

typedef struct {
    bool allowed;
    bool response_pending;
    int64_t hold_until_us;
    int64_t response_since_us;
    int64_t response_until_us;
    int64_t last_response_us;
    int64_t fault_until_us;
    int64_t retry_after_us;
} dual_end_wifi_idle_policy_t;

static inline void dual_end_wifi_idle_constraints(dual_end_wifi_idle_policy_t *state,
                                                   bool allowed, int64_t activity_us,
                                                   int64_t now_us)
{
    state->allowed = allowed;
    int64_t until = (!allowed ? now_us : activity_us) + DUAL_END_WIFI_IDLE_HOLD_US;
    if (until > state->hold_until_us) state->hold_until_us = until;
}
static inline void dual_end_wifi_idle_resumed(dual_end_wifi_idle_policy_t *state,
                                              int64_t completed_us)
{
    int64_t until = completed_us + DUAL_END_WIFI_IDLE_RX_WINDOW_US;
    if (until > state->hold_until_us) state->hold_until_us = until;
    state->retry_after_us = 0;
}
static inline void dual_end_wifi_idle_submitted(dual_end_wifi_idle_policy_t *state,
                                                int64_t submitted_us)
{
    state->response_since_us = submitted_us;
    state->response_until_us = submitted_us + DUAL_END_WIFI_IDLE_RX_WINDOW_US;
    state->response_pending = state->last_response_us < submitted_us;
}
static inline void dual_end_wifi_idle_received(dual_end_wifi_idle_policy_t *state,
                                               int64_t received_us)
{
    state->last_response_us = received_us;
    if (received_us >= state->response_since_us) state->response_pending = false;
}
static inline void dual_end_wifi_idle_fault(dual_end_wifi_idle_policy_t *state,
                                            int64_t now_us)
{
    state->fault_until_us = now_us + DUAL_END_WIFI_IDLE_FAULT_HOLD_US;
    state->retry_after_us = now_us + DUAL_END_WIFI_IDLE_RETRY_US;
    state->response_pending = false;
}
static inline bool dual_end_wifi_idle_poll(dual_end_wifi_idle_policy_t *state,
                                            int64_t now_us)
{
    if (state->response_pending && now_us >= state->response_until_us) {
        dual_end_wifi_idle_fault(state, now_us);
        return true;
    }
    return false;
}
static inline bool dual_end_wifi_idle_can_stop(const dual_end_wifi_idle_policy_t *state,
                                                bool connected, bool busy, int64_t now_us)
{
    return state->allowed && connected && !busy && !state->response_pending &&
        now_us >= state->hold_until_us && now_us >= state->response_until_us &&
        now_us >= state->fault_until_us;
}
static inline bool dual_end_wifi_idle_must_wake(const dual_end_wifi_idle_policy_t *state,
                                                 bool connected, bool busy, int64_t now_us)
{
    return !state->allowed || !connected || busy || now_us < state->hold_until_us ||
        now_us < state->fault_until_us;
}
