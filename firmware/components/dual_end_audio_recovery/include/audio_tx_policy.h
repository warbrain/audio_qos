#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "dual_end_audio_recovery_protocol.h"

/* Admission margins, not promises about Wi-Fi driver completion time. */
#define DUAL_END_AUDIO_TX_SEND_RESERVE_US 12000LL
#define DUAL_END_AUDIO_TX_FAST_RETRY_MAX_AGE_US 20000LL
#define DUAL_END_AUDIO_TX_PACING_PERIOD_US 20000LL
#define DUAL_END_AUDIO_TX_CATCHUP_BURST_MAX 2U

/* Caller serializes the observation and acquisition across every producer.
 * Repairs leave one shared request slot available for fresh media/control. */
static inline bool audio_tx_slot_may_acquire(unsigned free_slots, bool repair)
{
    return free_slots > (repair ? 1U : 0U);
}

static inline bool audio_tx_submit_has_budget(int64_t now_us, int64_t created_us)
{
    return now_us >= created_us &&
           now_us - created_us <= DUAL_END_AUDIO_TX_MAX_AGE_US -
                                  DUAL_END_AUDIO_TX_SEND_RESERVE_US;
}

/* Legacy helper name retained: the caller now uses this only after an immediate
 * NO_MEM rejection, never to retransmit an original after a MAC failure. */
static inline bool audio_tx_fast_retry_allowed(int64_t now_us, int64_t created_us,
                                               bool priority_pending,
                                               bool newer_audio_pending)
{
    return !priority_pending && !newer_audio_pending &&
           audio_tx_submit_has_budget(now_us, created_us) &&
           now_us - created_us < DUAL_END_AUDIO_TX_FAST_RETRY_MAX_AGE_US;
}

typedef struct {
    int64_t accepted_us[DUAL_END_AUDIO_TX_CATCHUP_BURST_MAX];
    unsigned count;
} audio_tx_pacing_t;

/* The single TX worker owns this rolling window. Repair/control bypass it. */
static inline int64_t audio_tx_pacing_wait_us(const audio_tx_pacing_t *state,
                                              int64_t now_us)
{
    if (state->count < DUAL_END_AUDIO_TX_CATCHUP_BURST_MAX) {
        return 0;
    }
    int64_t remaining = state->accepted_us[0] + DUAL_END_AUDIO_TX_PACING_PERIOD_US - now_us;
    return remaining > 0 ? remaining : 0;
}

static inline void audio_tx_pacing_note_accepted(audio_tx_pacing_t *state, int64_t now_us)
{
    if (state->count < DUAL_END_AUDIO_TX_CATCHUP_BURST_MAX) {
        state->accepted_us[state->count++] = now_us;
    } else {
        state->accepted_us[0] = state->accepted_us[1];
        state->accepted_us[1] = now_us;
    }
}
