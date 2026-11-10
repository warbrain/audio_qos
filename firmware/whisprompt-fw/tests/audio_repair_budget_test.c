#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "audio_recovery_tx.h"
#include "audio_session_gate.h"
#include "audio_control_retry_policy.h"
#include "audio_tx_policy.h"

static void channel_pause_session_gate(void)
{
    assert(audio_session_gate_transition(true, false, false,
                                         true, false, true) ==
           AUDIO_SESSION_GATE_INVALIDATE);
    assert(audio_session_gate_transition(true, false, true,
                                         true, false, true) ==
           AUDIO_SESSION_GATE_KEEP);
    assert(audio_session_gate_transition(true, false, true,
                                         true, false, false) ==
           AUDIO_SESSION_GATE_START);
    assert(audio_session_gate_transition(true, false, false,
                                         true, false, false) ==
           AUDIO_SESSION_GATE_KEEP);

    /* Resuming the channel while another gate remains closed cannot create a
     * session; opening the final gate creates exactly one. */
    assert(audio_session_gate_transition(true, true, true,
                                         true, true, false) ==
           AUDIO_SESSION_GATE_KEEP);
    assert(audio_session_gate_transition(true, true, false,
                                         true, false, false) ==
           AUDIO_SESSION_GATE_START);
    assert(audio_session_gate_transition(false, false, true,
                                         false, false, false) ==
           AUDIO_SESSION_GATE_KEEP);
    assert(audio_session_gate_transition(false, false, false,
                                         true, false, false) ==
           AUDIO_SESSION_GATE_START);
}

static void store(audio_recovery_tx_t *cache, uint32_t sequence, int64_t now_us)
{
    dual_end_audio_frame_header_t audio = {0};
    uint8_t encoded[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES] = {0};
    audio_recovery_tx_store(cache, sequence, now_us, &audio, encoded);
}

static uint32_t reserve(audio_recovery_tx_t *cache, uint32_t sequence, int64_t now_us)
{
    const audio_recovery_tx_entry_t *entry = NULL;
    uint32_t token = 0;
    assert(audio_recovery_tx_reserve(cache, cache->session_id, sequence, now_us,
                                    &token, &entry) == AUDIO_RECOVERY_TX_HIT);
    assert(token != 0 && entry != NULL && entry->sequence == sequence);
    return token;
}

static void submit(audio_recovery_tx_t *cache, uint32_t sequence, uint32_t token,
                   int64_t now_us)
{
    assert(audio_recovery_tx_prepare(cache, cache->session_id, sequence, token, now_us));
    assert(audio_recovery_tx_accepted(cache, cache->session_id, sequence, token, now_us));
}

static void expired_queue_and_duplicate(void)
{
    audio_recovery_tx_t cache;
    audio_recovery_tx_reset(&cache, 7);
    store(&cache, 10, 1000);
    store(&cache, 11, 2000);
    uint32_t first = reserve(&cache, 10, 5000);
    uint32_t second = reserve(&cache, 11, 5000);
    assert(cache.budget_used == 0);
    assert(audio_recovery_tx_budget_available(&cache, 5000));
    const audio_recovery_tx_entry_t *entry;
    uint32_t duplicate;
    assert(audio_recovery_tx_reserve(&cache, 7, 10, 5000, &duplicate, &entry) ==
           AUDIO_RECOVERY_TX_PENDING);
    assert(duplicate == 0 && entry == NULL);
    assert(!audio_recovery_tx_prepare(&cache, 7, 10, first, 401000));
    assert(audio_recovery_tx_complete(&cache, 7, 10, first));
    assert(cache.budget_used == 0 && audio_recovery_tx_budget_available(&cache, 401000));
    assert(audio_recovery_tx_prepare(&cache, 7, 11, second, 401000));
    assert(audio_recovery_tx_complete(&cache, 7, 11, second)); /* ESP-NOW sync error. */
    assert(cache.budget_used == 0);
}

static void submissions_and_spacing(void)
{
    audio_recovery_tx_t cache;
    audio_recovery_tx_reset(&cache, 7);
    store(&cache, 10, 1000);
    uint32_t first = reserve(&cache, 10, 2000);
    submit(&cache, 10, first, 3000);
    assert(cache.budget_used == 1);
    const audio_recovery_tx_entry_t *entry;
    assert(audio_recovery_tx_get(&cache, 7, 10, 50000, &entry) == AUDIO_RECOVERY_TX_PENDING);
    assert(!audio_recovery_tx_accepted(&cache, 7, 10, first, 4000));
    assert(cache.budget_used == 1); /* Duplicate accepted notification is harmless. */
    assert(audio_recovery_tx_complete(&cache, 7, 10, first)); /* MAC failure. */
    assert(cache.budget_used == 1); /* Failed on-air transmission consumes quota. */
    assert(audio_recovery_tx_get(&cache, 7, 10, 14999, &entry) == AUDIO_RECOVERY_TX_TOO_SOON);
    uint32_t second = reserve(&cache, 10, 15000);
    submit(&cache, 10, second, 15000);
    assert(audio_recovery_tx_complete(&cache, 7, 10, second));
    assert(cache.budget_used == 2);
    assert(audio_recovery_tx_get(&cache, 7, 10, 27000, &entry) == AUDIO_RECOVERY_TX_LIMIT);
    assert(audio_recovery_tx_budget_available(&cache, 102999));
    assert(audio_recovery_tx_budget_available(&cache, 103000));
    assert(cache.budget_used == 1); /* Rolling window retains second submission. */
    store(&cache, 11, 103000);
    uint32_t third = reserve(&cache, 11, 103000);
    submit(&cache, 11, third, 103000);
    assert(audio_recovery_tx_complete(&cache, 7, 11, third));
    assert(audio_recovery_tx_budget_available(&cache, 114999));
    assert(audio_recovery_tx_budget_available(&cache, 115000));
}

static void stale_completion_and_session_reset(void)
{
    audio_recovery_tx_t cache;
    audio_recovery_tx_reset(&cache, 7);
    store(&cache, UINT32_MAX, 1000);
    uint32_t first = reserve(&cache, UINT32_MAX, 2000);
    assert(audio_recovery_tx_complete(&cache, 7, UINT32_MAX, first));
    uint32_t next = reserve(&cache, UINT32_MAX, 2000);
    assert(first != next);
    assert(!audio_recovery_tx_complete(&cache, 7, UINT32_MAX, first));
    submit(&cache, UINT32_MAX, next, 2000);
    assert(audio_recovery_tx_complete(&cache, 7, UINT32_MAX, next));
    store(&cache, 0, 3000);
    uint32_t wrapped = reserve(&cache, 0, 3000);
    submit(&cache, 0, wrapped, 3000);
    audio_recovery_tx_session_reset(&cache, 8);
    assert(cache.budget_used == 2 && audio_recovery_tx_budget_available(&cache, 4000));
    assert(!audio_recovery_tx_complete(&cache, 7, 0, wrapped));
    store(&cache, 0, 102000);
    uint32_t fresh = reserve(&cache, 0, 102000);
    assert(fresh != wrapped);
    assert(!audio_recovery_tx_complete(&cache, 8, 0, wrapped));
    assert(audio_recovery_tx_prepare(&cache, 8, 0, fresh, 102000));
    /* Reusing a ring slot invalidates its queued token but keeps global quota. */
    store(&cache, DUAL_END_AUDIO_TX_CACHE_WINDOW, 102000);
    assert(!audio_recovery_tx_prepare(&cache, 8, 0, fresh, 102000));
    uint32_t reuse = reserve(&cache, DUAL_END_AUDIO_TX_CACHE_WINDOW, 102000);
    assert(!audio_recovery_tx_complete(&cache, 8, 0, fresh));
    assert(audio_recovery_tx_prepare(&cache, 8, DUAL_END_AUDIO_TX_CACHE_WINDOW, reuse, 102000));
    assert(audio_recovery_tx_complete(&cache, 8, DUAL_END_AUDIO_TX_CACHE_WINDOW, reuse));
    /* Repeated session ID plus repeated sequence cannot revive old tokens. */
    audio_recovery_tx_session_reset(&cache, 8);
    store(&cache, DUAL_END_AUDIO_TX_CACHE_WINDOW, 103000);
    uint32_t same = reserve(&cache, DUAL_END_AUDIO_TX_CACHE_WINDOW, 103000);
    assert(same != reuse && !audio_recovery_tx_complete(
        &cache, 8, DUAL_END_AUDIO_TX_CACHE_WINDOW, reuse));
    submit(&cache, DUAL_END_AUDIO_TX_CACHE_WINDOW, same, 103000);
}

static void bounded_reservations_and_deadline_crossing(void)
{
    audio_recovery_tx_t cache;
    audio_recovery_tx_reset(&cache, 7);
    store(&cache, 10, 1000);
    store(&cache, 11, 1000);
    store(&cache, 12, 1000);
    uint32_t first = reserve(&cache, 10, 1000);
    uint32_t second = reserve(&cache, 11, 1000);
    for (uint32_t i = 2; i < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS; ++i) {
        store(&cache, 20 + i, 1000);
        (void)reserve(&cache, 20 + i, 1000);
    }
    const audio_recovery_tx_entry_t *entry;
    uint32_t third;
    assert(audio_recovery_tx_reserve(&cache, 7, 12, 1000, &third, &entry) ==
           AUDIO_RECOVERY_TX_BUDGET);
    assert(audio_recovery_tx_prepare(&cache, 7, 10, first, 400999));
    assert(audio_recovery_tx_accepted(&cache, 7, 10, first, 401001));
    assert(cache.budget_used == 1); /* Crossing deadline inside driver still counts. */
    assert(!audio_recovery_tx_prepare(&cache, 7, 11, second, 401001));
    assert(audio_recovery_tx_complete(&cache, 7, 11, second));
    assert(audio_recovery_tx_complete(&cache, 7, 10, first));
}

static void reservation_identifier_wrap(void)
{
    audio_recovery_tx_t cache;
    audio_recovery_tx_reset(&cache, 7);
    store(&cache, 10, 1000);
    store(&cache, 11, 1000);
    uint32_t first = reserve(&cache, 10, 1000);
    assert(first == 1);
    cache.next_reservation_id = UINT32_MAX;
    uint32_t second = reserve(&cache, 11, 1000);
    assert(second == 2); /* Skip zero and the live token on unsigned wrap. */
    assert(audio_recovery_tx_prepare(&cache, 7, 10, first, 1000));
    assert(audio_recovery_tx_prepare(&cache, 7, 11, second, 1000));
}

static void full_rolling_quota_is_preserved_across_reset(void)
{
    audio_recovery_tx_t cache;
    audio_recovery_tx_reset(&cache, 9);
    for (uint32_t i = 0; i < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS; ++i) {
        store(&cache, i, 1000);
        uint32_t token = reserve(&cache, i, 1000 + i * 1000);
        submit(&cache, i, token, 1000 + i * 1000);
        assert(audio_recovery_tx_complete(&cache, 9, i, token));
    }
    assert(!audio_recovery_tx_budget_available(&cache, 100000));
    audio_recovery_tx_session_reset(&cache, 10);
    assert(!audio_recovery_tx_budget_available(&cache, 100999));
    assert(audio_recovery_tx_budget_available(&cache, 101000));
    assert(cache.budget_used == DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS - 1);
}


static void request_rounds_do_not_raise_per_frame_actual_limit(void)
{
    audio_recovery_tx_t cache;
    audio_recovery_tx_reset(&cache, 7);
    store(&cache, 10, 1000);
    assert(DUAL_END_AUDIO_NACK_MAX_REQUESTS == 4);
    assert(DUAL_END_AUDIO_TX_REPAIR_MAX_ATTEMPTS == 2);
    for (unsigned attempt = 0; attempt < DUAL_END_AUDIO_TX_REPAIR_MAX_ATTEMPTS; ++attempt) {
        int64_t now = 2000 + (int64_t)attempt * DUAL_END_AUDIO_REPAIR_MIN_INTERVAL_US;
        uint32_t token = reserve(&cache, 10, now);
        submit(&cache, 10, token, now);
        assert(audio_recovery_tx_complete(&cache, 7, 10, token));
    }
    const audio_recovery_tx_entry_t *entry;
    uint32_t token;
    for (unsigned request = 2; request < DUAL_END_AUDIO_NACK_MAX_REQUESTS; ++request) {
        int64_t now = 2000 + (int64_t)request * DUAL_END_AUDIO_NACK_INTERVAL_US;
        assert(audio_recovery_tx_budget_available(&cache, now));
        assert(audio_recovery_tx_reserve(&cache, 7, 10, now, &token, &entry) == AUDIO_RECOVERY_TX_LIMIT);
        assert(token == 0 && entry == NULL);
    }
    assert(cache.entries[10 % DUAL_END_AUDIO_TX_CACHE_WINDOW].repair_count == 2);
}

static void repair_preserves_the_last_shared_slot(void)
{
    assert(!audio_tx_slot_may_acquire(0, false));
    assert(audio_tx_slot_may_acquire(1, false));
    assert(!audio_tx_slot_may_acquire(0, true));
    assert(!audio_tx_slot_may_acquire(1, true));
    assert(audio_tx_slot_may_acquire(2, true));

    unsigned free_slots = 2;
    assert(audio_tx_slot_may_acquire(free_slots, true));
    free_slots--;
    assert(free_slots == 1);
    assert(!audio_tx_slot_may_acquire(free_slots, true));
    assert(audio_tx_slot_may_acquire(free_slots, false));
}

static void audio_session_begin_retries_only_once_for_no_mem(void)
{
    assert(dual_end_audio_control_max_attempts() == 2U);
    assert(dual_end_audio_control_retry_allowed(0U, true));
    assert(!dual_end_audio_control_retry_allowed(0U, false));
    assert(!dual_end_audio_control_retry_allowed(1U, true));
}

int main(void)
{
    channel_pause_session_gate();
    repair_preserves_the_last_shared_slot();
    audio_session_begin_retries_only_once_for_no_mem();
    request_rounds_do_not_raise_per_frame_actual_limit();
    full_rolling_quota_is_preserved_across_reset();
    expired_queue_and_duplicate();
    submissions_and_spacing();
    stale_completion_and_session_reset();
    bounded_reservations_and_deadline_crossing();
    reservation_identifier_wrap();
    puts("audio repair budget tests passed");
    return 0;
}
