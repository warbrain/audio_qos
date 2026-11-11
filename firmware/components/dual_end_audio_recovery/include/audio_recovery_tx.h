#pragma once
#include "dual_end_audio_recovery_protocol.h"

typedef struct {
    bool valid;
    uint32_t sequence;
    int64_t created_us;
    int64_t last_repair_us;
    uint8_t repair_count;
    dual_end_audio_frame_header_t audio;
    uint8_t encoded[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES];
} audio_recovery_tx_entry_t;

typedef struct {
    uint32_t id;
    uint32_t session_id;
    uint32_t sequence;
    int64_t created_us;
    bool submitted;
} audio_recovery_tx_reservation_t;

typedef struct {
    uint32_t session_id;
    audio_recovery_tx_entry_t entries[DUAL_END_AUDIO_TX_CACHE_WINDOW];
    /* Actual driver acceptances in a rolling 100 ms window. Pending queue
     * reservations occupy free capacity without consuming these timestamps. */
    int64_t submitted_us[DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS];
    uint8_t budget_used;
    uint32_t next_reservation_id;
    audio_recovery_tx_reservation_t reservations[DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS];
} audio_recovery_tx_t;

typedef enum {
    AUDIO_RECOVERY_TX_HIT,
    AUDIO_RECOVERY_TX_SESSION,
    AUDIO_RECOVERY_TX_MISS,
    AUDIO_RECOVERY_TX_EXPIRED,
    AUDIO_RECOVERY_TX_TOO_SOON,
    AUDIO_RECOVERY_TX_LIMIT,
    AUDIO_RECOVERY_TX_PENDING,
    AUDIO_RECOVERY_TX_BUDGET,
} audio_recovery_tx_result_t;

/* Initialization only. Session changes use session_reset to preserve actual
 * submissions and token identity across reset/late-completion races. */
void audio_recovery_tx_reset(audio_recovery_tx_t *cache, uint32_t session_id);
void audio_recovery_tx_session_reset(audio_recovery_tx_t *cache, uint32_t session_id);
void audio_recovery_tx_store(audio_recovery_tx_t *cache, uint32_t sequence,
                             int64_t created_us,
                             const dual_end_audio_frame_header_t *audio,
                             const uint8_t encoded[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES]);
audio_recovery_tx_result_t audio_recovery_tx_get(audio_recovery_tx_t *cache,
    uint32_t session_id, uint32_t sequence, int64_t now_us,
    const audio_recovery_tx_entry_t **entry);
bool audio_recovery_tx_budget_available(audio_recovery_tx_t *cache, int64_t now_us);
audio_recovery_tx_result_t audio_recovery_tx_reserve(audio_recovery_tx_t *cache,
    uint32_t session_id, uint32_t sequence, int64_t now_us, uint32_t *reservation_id,
    const audio_recovery_tx_entry_t **entry);
bool audio_recovery_tx_prepare(audio_recovery_tx_t *cache, uint32_t session_id,
    uint32_t sequence, uint32_t reservation_id, int64_t now_us);
bool audio_recovery_tx_accepted(audio_recovery_tx_t *cache, uint32_t session_id,
    uint32_t sequence, uint32_t reservation_id, int64_t now_us);
bool audio_recovery_tx_complete(audio_recovery_tx_t *cache, uint32_t session_id,
    uint32_t sequence, uint32_t reservation_id);
/* Synchronous compatibility helper for host callers with no queue. Production
 * uses reserve/prepare/accepted/complete at the real driver boundary. */
void audio_recovery_tx_commit(audio_recovery_tx_t *cache, uint32_t sequence,
                               int64_t now_us);
