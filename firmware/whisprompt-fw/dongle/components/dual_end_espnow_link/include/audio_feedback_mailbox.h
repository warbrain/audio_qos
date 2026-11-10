#pragma once

#include "dual_end_audio_recovery_protocol.h"

/* Shared RX/TX handoff only; caller serializes these short copy operations.
 * Recovery state and decoder remain exclusively owned by the RX task. */
typedef enum {
    DUAL_END_NACK_ACCEPTED,
    DUAL_END_NACK_CANCELLED,
    DUAL_END_NACK_EXPIRED,
    DUAL_END_NACK_REJECTED,
} dual_end_nack_result_kind_t;

typedef struct {
    uint32_t token;
    int64_t submitted_us;
    dual_end_nack_result_kind_t kind;
} dual_end_nack_result_t;

typedef struct {
    uint32_t token;
    int64_t latest_send_us;
    dual_end_audio_nack_packet_t packet;
} dual_end_nack_snapshot_t;

typedef struct {
    bool pending;
    uint32_t claimed_token;
    dual_end_nack_snapshot_t latest;
} dual_end_nack_mailbox_t;

/* Repeated publication refreshes one request, including its shrinking mask.
 * A request already handed to the driver cannot be re-enqueued. */
static inline bool dual_end_nack_mailbox_publish(dual_end_nack_mailbox_t *box,
    const dual_end_nack_snapshot_t *snapshot)
{
    if (snapshot->token == box->claimed_token ||
        (box->pending && box->latest.token != snapshot->token)) {
        return false;
    }
    box->latest = *snapshot;
    box->pending = true;
    return true;
}

static inline bool dual_end_nack_mailbox_cancel(dual_end_nack_mailbox_t *box,
    uint32_t token)
{
    if (!box->pending || box->latest.token != token) {
        return false;
    }
    box->pending = false;
    return true;
}

static inline bool dual_end_nack_mailbox_claim(dual_end_nack_mailbox_t *box,
    dual_end_nack_snapshot_t *snapshot, unsigned result_queue_spaces)
{
    /* TX is the sole result producer: once capacity is observed it cannot
     * shrink before this producer posts, while RX may only free more space. */
    if (!box->pending || result_queue_spaces == 0) {
        return false;
    }
    *snapshot = box->latest;
    box->claimed_token = snapshot->token;
    box->pending = false;
    return true;
}
