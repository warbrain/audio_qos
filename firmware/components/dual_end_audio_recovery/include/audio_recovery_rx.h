#pragma once

#include "dual_end_audio_recovery_protocol.h"

typedef enum {
    AUDIO_RX_SESSION_REJECTED,
    AUDIO_RX_SESSION_NEW,
    AUDIO_RX_SESSION_DUPLICATE,
} audio_rx_session_result_t;

typedef enum {
    AUDIO_RX_REJECTED,
    AUDIO_RX_ACCEPTED,
    AUDIO_RX_DUPLICATE,
    AUDIO_RX_LATE,
} audio_rx_push_result_t;

typedef enum {
    AUDIO_RX_FRAME,
    AUDIO_RX_MISSING,
    AUDIO_RX_NACK,
    AUDIO_RX_RESYNC,
    AUDIO_RX_END,
} audio_rx_event_kind_t;

typedef struct {
    audio_rx_event_kind_t kind;
    uint32_t session_id;
    uint32_t sequence;
    uint32_t first_sequence;
    uint16_t missing_mask;
    uint32_t nack_token;
    int64_t playout_us;
    uint8_t remaining_ms;
    int rssi;
    bool repaired;
    uint8_t encoded[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES];
} audio_rx_event_t;

typedef struct {
    bool present;
    bool repaired;
    uint32_t sequence;
    int64_t arrived_us;
    int rssi;
    uint8_t encoded[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES];
} audio_rx_slot_t;

typedef struct {
    uint32_t accepted;
    uint32_t repair_ontime;
    uint32_t repair_late;
    uint32_t duplicates;
    uint32_t late;
    uint32_t rejected;
    uint32_t output_frames;
    uint32_t recovered;
    /* Active timeline frames absent at their decode deadline, including the
     * inferred tail after unexpected source loss until the 300 ms idle abort.
     * An explicit PTT STOP bounds that tail at end_sequence. This is not
     * a pure over-the-air packet-loss counter. */
    uint32_t permanent_loss;
    uint32_t gap_events;
    uint32_t nack_requests;
    uint32_t nack_accepted;
    uint32_t nack_cancelled;
    uint32_t nack_expired;
    uint32_t nack_rejected;
    uint32_t resync_requests;
    /* Lifetime counters; new BEGIN does not reset them. These distinguish
     * recovery interruptions from active-timeline deadline loss. */
    uint32_t session_count;
    uint32_t abort_count;
    /* Accepted frames still buffered when abort/new BEGIN discards them.
     * Never includes inferred missing packets or an unobserved outage. */
    uint32_t aborted_buffered_frames;
    /* Arrived before deadline but could not be released for decoding in time. */
    uint32_t expired_buffered_frames;
} audio_rx_stats_t;

typedef struct {
    bool active;
    bool playout_anchored;
    uint32_t playout_anchor_sequence;
    int64_t playout_anchor_us;
    bool has_session;
    bool resync_pending;
    bool gap_active;
    bool nack_pending;
    bool ending;
    uint32_t end_sequence;
    int64_t end_started_us;
    uint32_t session_id;
    uint32_t first_sequence;
    uint32_t next_sequence;
    uint32_t received_history;
    uint32_t nack_token;
    uint32_t gap_generation;
    uint32_t nack_gap_generation;
    uint32_t retired_ids[8];
    uint8_t retired_count;
    uint8_t retired_cursor;
    uint8_t nack_rounds;
    uint8_t consecutive_missing;
    int64_t started_us;
    int64_t last_arrival_us;
    int64_t gap_since_us;
    int64_t last_nack_us;
    int64_t next_nack_admission_us;
    int64_t next_resync_us;
    audio_rx_slot_t slots[DUAL_END_AUDIO_RECOVERY_WINDOW];
    audio_rx_stats_t stats;
} audio_recovery_rx_t;

/* All operations are owned by one receiver task. No allocation or IDF calls.
 * Drain next(now) BEFORE push as well as after it. This commits expired gaps
 * before a late packet can fill them. Poll next even when no packets arrive.
 * push's now_us is the radio receive timestamp; next uses current task time.
 * encoded always contains 320 bytes: primary mono G.722, then raw mono G.722.
 * Both channels are delivered, repaired, dropped or resynchronized together. */
void audio_recovery_rx_init(audio_recovery_rx_t *rx);
/* Receiver-local USB timeline, never a timestamp from the sender clock.
 * anchor_us denotes the first-sample USB output time for anchor_sequence. */
void audio_recovery_rx_set_playout_anchor(audio_recovery_rx_t *rx,
    uint32_t sequence, int64_t playout_us);
int64_t audio_recovery_rx_playout_us(const audio_recovery_rx_t *rx, uint32_t sequence);
audio_rx_session_result_t audio_recovery_rx_begin(audio_recovery_rx_t *rx,
    uint32_t session_id, uint32_t first_sequence, int64_t now_us);
audio_rx_push_result_t audio_recovery_rx_push(audio_recovery_rx_t *rx,
    uint32_t session_id, uint32_t sequence, const uint8_t *encoded,
    int rssi, bool repaired, int64_t now_us);
bool audio_recovery_rx_next(audio_recovery_rx_t *rx, int64_t now_us,
    audio_rx_event_t *event);
/* A NACK event is a reservation, not a transmitted request. Refresh it while
 * queued; finish only with the driver's actual submission result, on RX task. */
typedef enum {
    AUDIO_RX_NACK_ACCEPTED,
    AUDIO_RX_NACK_CANCELLED,
    AUDIO_RX_NACK_EXPIRED,
    AUDIO_RX_NACK_REJECTED,
} audio_rx_nack_result_t;
bool audio_recovery_rx_refresh_nack(const audio_recovery_rx_t *rx,
    audio_rx_event_t *event, int64_t now_us, int64_t *latest_send_us);
void audio_recovery_rx_finish_nack(audio_recovery_rx_t *rx, uint32_t token,
    audio_rx_nack_result_t result, int64_t submitted_us);
void audio_recovery_rx_abort(audio_recovery_rx_t *rx, int64_t now_us);
/* A PTT STOP is terminal. Drain only frames before the exclusive cutoff;
 * it must never infer an endless silent tail or request a new session. */
void audio_recovery_rx_stop(audio_recovery_rx_t *rx);
void audio_recovery_rx_end(audio_recovery_rx_t *rx, uint32_t end_sequence, int64_t now_us);
/* A valid original frame after a dongle-only reboot can request a BEGIN.
 * The bootstrap RESYNC uses session_id=0/first_sequence=0, at most every 60 ms. */
void audio_recovery_rx_request_session(audio_recovery_rx_t *rx, int64_t now_us);
