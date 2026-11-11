#include "audio_recovery_rx.h"

#include <string.h>

int64_t audio_recovery_rx_playout_us(const audio_recovery_rx_t *rx, uint32_t sequence)
{
    if (!rx->playout_anchored) return 0;
    uint32_t distance = sequence - rx->playout_anchor_sequence;
    int64_t signed_distance = distance < UINT32_C(0x80000000) ?
        (int64_t)distance : (int64_t)distance - INT64_C(4294967296);
    return rx->playout_anchor_us + signed_distance * DUAL_END_AUDIO_FRAME_MS * 1000LL;
}

void audio_recovery_rx_set_playout_anchor(audio_recovery_rx_t *rx,
    uint32_t sequence, int64_t playout_us)
{
    if (!rx->active || !rx->playout_anchored || playout_us <= 0) return;
    rx->playout_anchor_sequence = sequence;
    rx->playout_anchor_us = playout_us;
}

static int64_t decode_deadline(const audio_recovery_rx_t *rx, uint32_t sequence)
{
    return audio_recovery_rx_playout_us(rx, sequence) - DUAL_END_AUDIO_DECODE_MARGIN_US;
}

static uint8_t remaining_ms(const audio_recovery_rx_t *rx, int64_t now_us)
{
    int64_t remaining = decode_deadline(rx, rx->next_sequence) - now_us;
    if (remaining <= 0) return 0;
    remaining /= 1000; /* Round down; never advertise time already consumed. */
    return (uint8_t)(remaining > DUAL_END_AUDIO_PLAYOUT_DELAY_US / 1000 ?
        DUAL_END_AUDIO_PLAYOUT_DELAY_US / 1000 : remaining);
}

static void discard_buffered_frames(audio_recovery_rx_t *rx)
{
    for (unsigned i = 0; i < DUAL_END_AUDIO_RECOVERY_WINDOW; ++i) {
        if (rx->slots[i].present) {
            ++rx->stats.aborted_buffered_frames;
        }
    }
    memset(rx->slots, 0, sizeof(rx->slots));
}

void audio_recovery_rx_init(audio_recovery_rx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
}

void audio_recovery_rx_stop(audio_recovery_rx_t *rx)
{
    rx->active = false;
    rx->playout_anchored = false;
    rx->ending = false;
    rx->gap_active = false;
    rx->nack_pending = false;
    rx->resync_pending = false;
    discard_buffered_frames(rx);
}

void audio_recovery_rx_end(audio_recovery_rx_t *rx, uint32_t end_sequence, int64_t now_us)
{
    /* This compatibility entry point is called only after the owner validates
     * the current session identity. END also cancels a prior abort's RESYNC. */
    if (!rx->active) {
        audio_recovery_rx_stop(rx);
        return;
    }
    if (rx->ending) return;
    rx->ending = true;
    rx->end_started_us = now_us;
    rx->end_sequence = end_sequence;
    /* Without an original there is no receiver-local playout anchor. An
     * explicit STOP still terminates that empty session, without a RESYNC.
     * A corrupt distant cutoff must not create unbounded NACK/silence. */
    if (!rx->playout_anchored ||
        dual_end_audio_seq_before(end_sequence, rx->next_sequence) ||
        end_sequence - rx->next_sequence > DUAL_END_AUDIO_RECOVERY_WINDOW) {
        rx->end_sequence = rx->next_sequence;
    }
    for (unsigned i = 0; i < DUAL_END_AUDIO_RECOVERY_WINDOW; ++i) {
        audio_rx_slot_t *slot = &rx->slots[i];
        if (slot->present && !dual_end_audio_seq_before(slot->sequence, rx->end_sequence)) {
            slot->present = false;
            ++rx->stats.aborted_buffered_frames;
        }
    }
}

void audio_recovery_rx_abort(audio_recovery_rx_t *rx, int64_t now_us)
{
    if (rx->active) {
        ++rx->stats.abort_count;
        /* A decoder/sink failure can abandon a finite drain, but an explicit
         * END remains terminal. Preserve failure/discard counts without ever
         * reviving the session through RESYNC. */
        if (rx->ending) {
            audio_recovery_rx_stop(rx);
            return;
        }
        rx->active = false;
        rx->ending = false;
        rx->playout_anchored = false;
        rx->gap_active = false;
        discard_buffered_frames(rx);
        rx->resync_pending = true;
        rx->next_resync_us = now_us;
    }
}

void audio_recovery_rx_request_session(audio_recovery_rx_t *rx, int64_t now_us)
{
    if (!rx->has_session && !rx->resync_pending) {
        rx->resync_pending = true;
        rx->next_resync_us = now_us;
    }
}

audio_rx_session_result_t audio_recovery_rx_begin(audio_recovery_rx_t *rx,
    uint32_t session_id, uint32_t first_sequence, int64_t now_us)
{
    if (session_id == 0) {
        return AUDIO_RX_SESSION_REJECTED;
    }
    if (rx->has_session && rx->session_id == session_id) {
        return rx->active && rx->first_sequence == first_sequence ?
            AUDIO_RX_SESSION_DUPLICATE : AUDIO_RX_SESSION_REJECTED;
    }
    for (unsigned i = 0; i < rx->retired_count; ++i) {
        if (rx->retired_ids[i] == session_id) {
            return AUDIO_RX_SESSION_REJECTED;
        }
    }
    if (rx->has_session) {
        rx->retired_ids[rx->retired_cursor] = rx->session_id;
        rx->retired_cursor = (uint8_t)((rx->retired_cursor + 1U) % 8U);
        if (rx->retired_count < 8U) {
            ++rx->retired_count;
        }
    }
    discard_buffered_frames(rx);
    ++rx->stats.session_count;
    rx->active = true;
    rx->ending = false;
    rx->playout_anchored = false;
    rx->playout_anchor_us = 0;
    rx->playout_anchor_sequence = first_sequence;
    rx->has_session = true;
    rx->resync_pending = false;
    rx->session_id = session_id;
    rx->first_sequence = first_sequence;
    rx->next_sequence = first_sequence;
    rx->received_history = 0;
    rx->started_us = now_us;
    rx->last_arrival_us = now_us;
    rx->gap_active = false;
    rx->nack_pending = false;
    rx->next_nack_admission_us = now_us;
    rx->nack_rounds = 0;
    rx->consecutive_missing = 0;
    return AUDIO_RX_SESSION_NEW;
}

audio_rx_push_result_t audio_recovery_rx_push(audio_recovery_rx_t *rx,
    uint32_t session_id, uint32_t sequence, const uint8_t *encoded,
    int rssi, bool repaired, int64_t now_us)
{
    if (!rx->active || session_id != rx->session_id || !encoded || now_us < rx->started_us ||
        (rx->ending && !dual_end_audio_seq_before(sequence, rx->end_sequence))) {
        ++rx->stats.rejected;
        return AUDIO_RX_REJECTED;
    }
    if (dual_end_audio_seq_before(sequence, rx->next_sequence)) {
        uint32_t age = rx->next_sequence - sequence;
        if (age <= 32U && ((rx->received_history >> (age - 1U)) & 1U)) {
            ++rx->stats.duplicates;
            return AUDIO_RX_DUPLICATE;
        }
        ++rx->stats.late;
        if (repaired) {
            ++rx->stats.repair_late;
        }
        return AUDIO_RX_LATE;
    }
    uint32_t distance = sequence - rx->next_sequence;
    if (distance >= DUAL_END_AUDIO_RECOVERY_WINDOW) {
        ++rx->stats.rejected;
        audio_recovery_rx_abort(rx, now_us);
        return AUDIO_RX_REJECTED;
    }
    if (!rx->playout_anchored) {
        if (repaired) {
            ++rx->stats.rejected;
            return AUDIO_RX_REJECTED;
        }
        rx->playout_anchored = true;
        rx->playout_anchor_sequence = sequence;
        rx->playout_anchor_us = now_us + DUAL_END_AUDIO_PLAYOUT_DELAY_US;
    }
    if (now_us >= decode_deadline(rx, sequence)) {
        ++rx->stats.late;
        if (repaired) ++rx->stats.repair_late;
        return AUDIO_RX_LATE;
    }
    audio_rx_slot_t *slot = &rx->slots[sequence % DUAL_END_AUDIO_RECOVERY_WINDOW];
    if (slot->present) {
        ++rx->stats.duplicates;
        return AUDIO_RX_DUPLICATE;
    }
    slot->present = true;
    slot->repaired = repaired;
    slot->sequence = sequence;
    slot->arrived_us = now_us;
    slot->rssi = rssi;
    memcpy(slot->encoded, encoded, sizeof(slot->encoded));
    rx->last_arrival_us = now_us;
    ++rx->stats.accepted;
    if (repaired) {
        ++rx->stats.repair_ontime;
    }
    return AUDIO_RX_ACCEPTED;
}

bool audio_recovery_rx_next(audio_recovery_rx_t *rx, int64_t now_us,
    audio_rx_event_t *event)
{
    memset(event, 0, sizeof(*event));
    event->session_id = rx->session_id;
    event->first_sequence = rx->first_sequence;
    event->sequence = rx->next_sequence;
    event->playout_us = audio_recovery_rx_playout_us(rx, rx->next_sequence);
    if (rx->ending && rx->next_sequence == rx->end_sequence) {
        audio_recovery_rx_stop(rx);
        event->kind = AUDIO_RX_END;
        return true;
    }
    if (rx->active && !rx->ending &&
        now_us - rx->last_arrival_us >= DUAL_END_AUDIO_SESSION_IDLE_US) {
        audio_recovery_rx_abort(rx, now_us);
    }
    if (!rx->active) {
        if (rx->resync_pending && now_us >= rx->next_resync_us) {
            event->kind = AUDIO_RX_RESYNC;
            rx->next_resync_us = now_us + DUAL_END_AUDIO_SESSION_RETRY_US;
            ++rx->stats.resync_requests;
            return true;
        }
        return false;
    }
    if (!rx->playout_anchored) return false;

    audio_rx_slot_t *slot = &rx->slots[rx->next_sequence % DUAL_END_AUDIO_RECOVERY_WINDOW];
    if (slot->present && slot->sequence == rx->next_sequence &&
        now_us >= decode_deadline(rx, rx->next_sequence)) {
        slot->present = false;
        ++rx->stats.expired_buffered_frames;
    }
    if (slot->present && slot->sequence == rx->next_sequence) {
        event->kind = AUDIO_RX_FRAME;
        event->repaired = slot->repaired;
        event->rssi = slot->rssi;
        memcpy(event->encoded, slot->encoded, sizeof(event->encoded));
        slot->present = false;
        ++rx->next_sequence;
        rx->received_history = (rx->received_history << 1) | 1U;
        rx->consecutive_missing = 0;
        ++rx->stats.output_frames;
        if (event->repaired) ++rx->stats.recovered;
        /* The next missing head owns its own request budget; it may not have
         * appeared in any earlier NACK bitmap. Pending old tokens stay owned. */
        rx->gap_active = false;
        return true;
    }

    /* One deadline per frame on the USB timeline. A long callback must not
     * settle a whole burst at one shared gap deadline or restart the codec. */
    if (now_us >= decode_deadline(rx, rx->next_sequence)) {
        event->kind = AUDIO_RX_MISSING;
        ++rx->next_sequence;
        rx->received_history <<= 1;
        ++rx->stats.permanent_loss;
        if (rx->consecutive_missing != UINT8_MAX) ++rx->consecutive_missing;
        rx->gap_active = false;
        return true;
    }

    bool found = false;
    uint32_t furthest = 0;
    int64_t earliest_us = now_us;
    for (unsigned i = 0; i < DUAL_END_AUDIO_RECOVERY_WINDOW; ++i) {
        if (rx->slots[i].present &&
            (!rx->ending || dual_end_audio_seq_before(rx->slots[i].sequence, rx->end_sequence))) {
            uint32_t distance = rx->slots[i].sequence - rx->next_sequence;
            if (!found || rx->slots[i].arrived_us < earliest_us)
                earliest_us = rx->slots[i].arrived_us;
            if (distance > furthest) furthest = distance;
            found = true;
        }
    }
    if (rx->ending) {
        uint32_t remaining = rx->end_sequence - rx->next_sequence;
        furthest = remaining;
        if (!found) earliest_us = rx->end_started_us;
        found = remaining != 0;
    }
    if (!found) {
        rx->gap_active = false;
        return false;
    }
    if (!rx->gap_active) {
        rx->gap_active = true;
        rx->gap_since_us = earliest_us;
        rx->nack_rounds = 0;
        ++rx->gap_generation;
        rx->next_nack_admission_us = earliest_us;
        rx->last_nack_us = earliest_us;
        ++rx->stats.gap_events;
    }
    if (now_us - rx->gap_since_us >= DUAL_END_AUDIO_REORDER_GRACE_US &&
        decode_deadline(rx, rx->next_sequence) - now_us > DUAL_END_AUDIO_MIN_REPAIR_ROUNDTRIP_US &&
        rx->nack_rounds < DUAL_END_AUDIO_NACK_MAX_REQUESTS &&
        !rx->nack_pending && now_us >= rx->next_nack_admission_us &&
        (rx->nack_rounds == 0 || now_us - rx->last_nack_us >= DUAL_END_AUDIO_NACK_INTERVAL_US)) {
        event->kind = AUDIO_RX_NACK;
        event->remaining_ms = remaining_ms(rx, now_us);
        for (uint32_t offset = 0; offset < furthest; ++offset) {
            uint32_t sequence = rx->next_sequence + offset;
            const audio_rx_slot_t *candidate = &rx->slots[sequence % DUAL_END_AUDIO_RECOVERY_WINDOW];
            if (!candidate->present || candidate->sequence != sequence)
                event->missing_mask |= (uint16_t)(1U << offset);
        }
        rx->nack_pending = true;
        if (++rx->nack_token == 0) ++rx->nack_token;
        rx->nack_gap_generation = rx->gap_generation;
        event->nack_token = rx->nack_token;
        ++rx->stats.nack_requests;
        return true;
    }
    return false;
}

bool audio_recovery_rx_refresh_nack(const audio_recovery_rx_t *rx,
    audio_rx_event_t *event, int64_t now_us, int64_t *latest_send_us)
{
    if (!rx->active || !rx->gap_active || !rx->nack_pending ||
        event->session_id != rx->session_id || event->nack_token != rx->nack_token ||
        rx->nack_gap_generation != rx->gap_generation) {
        return false;
    }
    *latest_send_us = decode_deadline(rx, rx->next_sequence) -
        DUAL_END_AUDIO_MIN_REPAIR_ROUNDTRIP_US;
    if (now_us >= *latest_send_us) {
        return false;
    }
    uint32_t furthest = rx->ending ? rx->end_sequence - rx->next_sequence : 0;
    for (unsigned i = 0; i < DUAL_END_AUDIO_RECOVERY_WINDOW; ++i) {
        const audio_rx_slot_t *slot = &rx->slots[i];
        if (slot->present && !dual_end_audio_seq_before(slot->sequence, rx->next_sequence) &&
            (!rx->ending || dual_end_audio_seq_before(slot->sequence, rx->end_sequence))) {
            uint32_t distance = slot->sequence - rx->next_sequence;
            if (distance > furthest) {
                furthest = distance;
            }
        }
    }
    event->sequence = rx->next_sequence;
    event->playout_us = audio_recovery_rx_playout_us(rx, rx->next_sequence);
    event->remaining_ms = remaining_ms(rx, now_us);
    event->missing_mask = 0;
    for (uint32_t offset = 0; offset < furthest; ++offset) {
        uint32_t sequence = rx->next_sequence + offset;
        const audio_rx_slot_t *slot = &rx->slots[sequence % DUAL_END_AUDIO_RECOVERY_WINDOW];
        if (!slot->present || slot->sequence != sequence) {
            event->missing_mask |= (uint16_t)(1U << offset);
        }
    }
    return event->missing_mask != 0;
}

void audio_recovery_rx_finish_nack(audio_recovery_rx_t *rx, uint32_t token,
    audio_rx_nack_result_t result, int64_t submitted_us)
{
    switch (result) {
    case AUDIO_RX_NACK_ACCEPTED: ++rx->stats.nack_accepted; break;
    case AUDIO_RX_NACK_CANCELLED: ++rx->stats.nack_cancelled; break;
    case AUDIO_RX_NACK_EXPIRED: ++rx->stats.nack_expired; break;
    case AUDIO_RX_NACK_REJECTED: ++rx->stats.nack_rejected; break;
    }
    if (!rx->nack_pending || rx->nack_token != token) {
        return; /* A prior session's accepted notification is only telemetry. */
    }
    rx->nack_pending = false;
    if (rx->active && rx->gap_active && rx->nack_gap_generation == rx->gap_generation &&
        result == AUDIO_RX_NACK_ACCEPTED) {
        ++rx->nack_rounds;
        rx->last_nack_us = submitted_us;
    }
    /* Bound immediate driver failures without spending on-air request rounds. */
    rx->next_nack_admission_us = submitted_us + DUAL_END_AUDIO_REORDER_GRACE_US;
}
