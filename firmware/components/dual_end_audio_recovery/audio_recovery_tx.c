#include "audio_recovery_tx.h"
#include <string.h>

static bool cache_age_valid(int64_t now_us, int64_t created_us)
{
    return now_us >= created_us && now_us - created_us < DUAL_END_AUDIO_TX_CACHE_MAX_AGE_US;
}

void audio_recovery_tx_reset(audio_recovery_tx_t *cache, uint32_t session_id)
{
    memset(cache, 0, sizeof(*cache));
    cache->session_id = session_id;
}

void audio_recovery_tx_session_reset(audio_recovery_tx_t *cache, uint32_t session_id)
{
    memset(cache->entries, 0, sizeof(cache->entries));
    memset(cache->reservations, 0, sizeof(cache->reservations));
    cache->session_id = session_id;
}

static void refresh_budget(audio_recovery_tx_t *cache, int64_t now_us)
{
    unsigned retained = 0;
    for (unsigned i = 0; i < cache->budget_used; i++) {
        int64_t submitted_us = cache->submitted_us[i];
        if (now_us < submitted_us ||
            now_us - submitted_us < DUAL_END_AUDIO_REPAIR_BUDGET_US) {
            cache->submitted_us[retained++] = submitted_us;
        }
    }
    cache->budget_used = (uint8_t)retained;
}

static audio_recovery_tx_reservation_t *find_reservation(audio_recovery_tx_t *cache,
    uint32_t session_id, uint32_t sequence, uint32_t reservation_id)
{
    if (reservation_id == 0 || session_id == 0 || session_id != cache->session_id) {
        return NULL;
    }
    for (unsigned i = 0; i < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS; i++) {
        audio_recovery_tx_reservation_t *reservation = &cache->reservations[i];
        if (reservation->id == reservation_id && reservation->session_id == session_id &&
            reservation->sequence == sequence) {
            return reservation;
        }
    }
    return NULL;
}

static uint32_t next_reservation_id(audio_recovery_tx_t *cache)
{
    bool occupied;
    do {
        cache->next_reservation_id++;
        occupied = cache->next_reservation_id == 0;
        for (unsigned i = 0; i < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS; i++) {
            occupied |= cache->reservations[i].id == cache->next_reservation_id;
        }
    } while (occupied);
    return cache->next_reservation_id;
}

void audio_recovery_tx_store(audio_recovery_tx_t *cache, uint32_t sequence,
                             int64_t created_us,
                             const dual_end_audio_frame_header_t *audio,
                             const uint8_t encoded[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES])
{
    audio_recovery_tx_entry_t *entry =
        &cache->entries[sequence % DUAL_END_AUDIO_TX_CACHE_WINDOW];
    /* Evicted queue items retain their old token, so their later preparation
     * or completion cannot affect this slot's replacement. Submitted quota
     * remains consumed even if the cached codewords are replaced. */
    for (unsigned i = 0; i < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS; i++) {
        audio_recovery_tx_reservation_t *reservation = &cache->reservations[i];
        if (reservation->id != 0 && entry->valid &&
            reservation->session_id == cache->session_id &&
            reservation->sequence == entry->sequence &&
            reservation->created_us == entry->created_us) {
            memset(reservation, 0, sizeof(*reservation));
        }
    }
    memset(entry, 0, sizeof(*entry));
    entry->valid = true;
    entry->sequence = sequence;
    entry->created_us = created_us;
    entry->audio = *audio;
    memcpy(entry->encoded, encoded, sizeof(entry->encoded));
}

audio_recovery_tx_result_t audio_recovery_tx_get(audio_recovery_tx_t *cache,
    uint32_t session_id, uint32_t sequence, int64_t now_us,
    const audio_recovery_tx_entry_t **entry)
{
    *entry = NULL;
    if (session_id == 0 || session_id != cache->session_id) {
        return AUDIO_RECOVERY_TX_SESSION;
    }
    audio_recovery_tx_entry_t *candidate =
        &cache->entries[sequence % DUAL_END_AUDIO_TX_CACHE_WINDOW];
    if (!candidate->valid || candidate->sequence != sequence) {
        return AUDIO_RECOVERY_TX_MISS;
    }
    if (!cache_age_valid(now_us, candidate->created_us)) {
        return AUDIO_RECOVERY_TX_EXPIRED;
    }
    for (unsigned i = 0; i < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS; i++) {
        const audio_recovery_tx_reservation_t *reservation = &cache->reservations[i];
        if (reservation->id != 0 && reservation->session_id == session_id &&
            reservation->sequence == sequence) {
            return AUDIO_RECOVERY_TX_PENDING;
        }
    }
    if (candidate->repair_count >= DUAL_END_AUDIO_TX_REPAIR_MAX_ATTEMPTS) {
        return AUDIO_RECOVERY_TX_LIMIT;
    }
    if (candidate->repair_count != 0 &&
        now_us - candidate->last_repair_us < DUAL_END_AUDIO_REPAIR_MIN_INTERVAL_US) {
        return AUDIO_RECOVERY_TX_TOO_SOON;
    }
    *entry = candidate;
    return AUDIO_RECOVERY_TX_HIT;
}

bool audio_recovery_tx_budget_available(audio_recovery_tx_t *cache, int64_t now_us)
{
    refresh_budget(cache, now_us);
    unsigned reserved = 0;
    unsigned occupied = 0;
    for (unsigned i = 0; i < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS; i++) {
        if (cache->reservations[i].id != 0) {
            occupied++;
            reserved += !cache->reservations[i].submitted;
        }
    }
    return occupied < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS &&
           cache->budget_used + reserved < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS;
}

audio_recovery_tx_result_t audio_recovery_tx_reserve(audio_recovery_tx_t *cache,
    uint32_t session_id, uint32_t sequence, int64_t now_us, uint32_t *reservation_id,
    const audio_recovery_tx_entry_t **entry)
{
    *reservation_id = 0;
    audio_recovery_tx_result_t result = audio_recovery_tx_get(
        cache, session_id, sequence, now_us, entry);
    if (result != AUDIO_RECOVERY_TX_HIT) {
        return result;
    }
    if (!audio_recovery_tx_budget_available(cache, now_us)) {
        *entry = NULL;
        return AUDIO_RECOVERY_TX_BUDGET;
    }
    for (unsigned i = 0; i < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS; i++) {
        audio_recovery_tx_reservation_t *reservation = &cache->reservations[i];
        if (reservation->id == 0) {
            uint32_t token = next_reservation_id(cache);
            *reservation = (audio_recovery_tx_reservation_t) {
                .id = token,
                .session_id = session_id,
                .sequence = sequence,
                .created_us = (*entry)->created_us,
            };
            *reservation_id = reservation->id;
            return AUDIO_RECOVERY_TX_HIT;
        }
    }
    *entry = NULL;
    return AUDIO_RECOVERY_TX_BUDGET;
}

bool audio_recovery_tx_prepare(audio_recovery_tx_t *cache, uint32_t session_id,
    uint32_t sequence, uint32_t reservation_id, int64_t now_us)
{
    audio_recovery_tx_reservation_t *reservation = find_reservation(
        cache, session_id, sequence, reservation_id);
    if (reservation == NULL || reservation->submitted) {
        return false;
    }
    audio_recovery_tx_entry_t *entry =
        &cache->entries[sequence % DUAL_END_AUDIO_TX_CACHE_WINDOW];
    refresh_budget(cache, now_us);
    return entry->valid && entry->sequence == sequence &&
           entry->created_us == reservation->created_us &&
           cache_age_valid(now_us, entry->created_us) &&
           entry->repair_count < DUAL_END_AUDIO_TX_REPAIR_MAX_ATTEMPTS &&
           (entry->repair_count == 0 ||
            (now_us >= entry->last_repair_us &&
             now_us - entry->last_repair_us >= DUAL_END_AUDIO_REPAIR_MIN_INTERVAL_US)) &&
           cache->budget_used < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS;
}

bool audio_recovery_tx_accepted(audio_recovery_tx_t *cache, uint32_t session_id,
    uint32_t sequence, uint32_t reservation_id, int64_t now_us)
{
    /* prepare and driver submission are serialized by the transport. Acceptance
     * must still be counted if the frame crosses its deadline in esp_now_send. */
    audio_recovery_tx_reservation_t *reservation = find_reservation(
        cache, session_id, sequence, reservation_id);
    if (reservation == NULL || reservation->submitted) {
        return false;
    }
    refresh_budget(cache, now_us);
    if (cache->budget_used >= DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS) {
        return false;
    }
    reservation->submitted = true;
    cache->submitted_us[cache->budget_used++] = now_us;
    audio_recovery_tx_entry_t *entry =
        &cache->entries[sequence % DUAL_END_AUDIO_TX_CACHE_WINDOW];
    if (entry->valid && entry->sequence == sequence &&
        entry->created_us == reservation->created_us) {
        entry->repair_count++;
        entry->last_repair_us = now_us;
    }
    return true;
}

bool audio_recovery_tx_complete(audio_recovery_tx_t *cache, uint32_t session_id,
    uint32_t sequence, uint32_t reservation_id)
{
    audio_recovery_tx_reservation_t *reservation = find_reservation(
        cache, session_id, sequence, reservation_id);
    if (reservation == NULL) {
        return false;
    }
    /* Success/failure does not refund an actual submission. Cancellation before
     * submission simply frees the reserved capacity. */
    memset(reservation, 0, sizeof(*reservation));
    return true;
}

void audio_recovery_tx_commit(audio_recovery_tx_t *cache, uint32_t sequence,
                               int64_t now_us)
{
    const audio_recovery_tx_entry_t *entry;
    uint32_t reservation_id;
    if (audio_recovery_tx_reserve(cache, cache->session_id, sequence, now_us,
                                 &reservation_id, &entry) == AUDIO_RECOVERY_TX_HIT) {
        audio_recovery_tx_accepted(cache, cache->session_id, sequence, reservation_id, now_us);
        audio_recovery_tx_complete(cache, cache->session_id, sequence, reservation_id);
    }
}
