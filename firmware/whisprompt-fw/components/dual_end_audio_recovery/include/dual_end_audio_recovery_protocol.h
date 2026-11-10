#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "dual_end_link_config.h"

/* Version 5 keeps the 0.4.2 session/NACK/deadline protocol.  The dual-mic
 * product runs a continuous stream, so the normal packet's stream_id is the
 * session identity and no PTT/input announcement participates in recovery. */
#define DUAL_END_AUDIO_RECOVERY_VERSION 5
#define DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES DUAL_END_AUDIO_ENCODED_FRAME_BYTES
#define DUAL_END_AUDIO_RECOVERY_WINDOW 16U
/* Cache lifetime and audio playback deadlines are independent. Originals
 * keep their existing bounded submission age. Repair uses the receiver budget. */
#define DUAL_END_AUDIO_TX_CACHE_WINDOW 32U
#define DUAL_END_AUDIO_TX_CACHE_MAX_AGE_US 400000LL
#define DUAL_END_AUDIO_PLAYOUT_DELAY_US 232000LL
#define DUAL_END_AUDIO_DECODE_MARGIN_US 8000LL
#define DUAL_END_AUDIO_NACK_TRANSIT_RESERVE_US 8000LL
#define DUAL_END_AUDIO_REPAIR_SEND_RESERVE_US 4000LL
#define DUAL_END_AUDIO_MIN_REPAIR_ROUNDTRIP_US \
    (DUAL_END_AUDIO_NACK_TRANSIT_RESERVE_US + DUAL_END_AUDIO_REPAIR_SEND_RESERVE_US)
#define DUAL_END_AUDIO_TX_MAX_AGE_US 100000LL
#define DUAL_END_AUDIO_REORDER_GRACE_US 4000LL
/* Spread requests across the normal 200 ms recovery opportunity. Requests
 * are not repair submissions: pending work/quota may suppress a whole NACK. */
#define DUAL_END_AUDIO_NACK_INTERVAL_US 60000LL
#define DUAL_END_AUDIO_NACK_MAX_REQUESTS 4U
#define DUAL_END_AUDIO_TX_REPAIR_MAX_ATTEMPTS 2U
#define DUAL_END_AUDIO_REPAIR_MIN_INTERVAL_US 12000LL
#define DUAL_END_AUDIO_SESSION_RETRY_US 60000LL
#define DUAL_END_AUDIO_SESSION_IDLE_US 300000LL
#define DUAL_END_AUDIO_REPAIR_BUDGET_US 100000LL
#define DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS 4U

/* Kept separate from the legacy enum so both target copies stay wire-identical. */
#define SCEPTER_MSG_AUDIO_SESSION 0x11
#define SCEPTER_MSG_AUDIO_NACK 0x12
#define SCEPTER_MSG_AUDIO_REPAIR 0x13

typedef enum {
    DUAL_END_AUDIO_SESSION_BEGIN = 1,
    DUAL_END_AUDIO_SESSION_ACK = 2,
    DUAL_END_AUDIO_SESSION_RESYNC = 3,
} dual_end_audio_session_command_t;

/* BEGIN/ACK and ordinary RESYNC use a nonzero session_id. A receiver that has
 * never seen BEGIN may request cold-start synchronization with RESYNC and
 * session_id=first_sequence=0. The sender accepts that request only while an
 * established session is ready; repeats cannot interrupt an ACK handshake. */
typedef struct {
    dual_end_packet_header_t packet;
    uint8_t version;
    uint8_t command;
    uint16_t reserved;
    uint32_t session_id;
    uint32_t first_sequence;
} __attribute__((packed)) dual_end_audio_session_packet_t;

typedef struct {
    dual_end_packet_header_t packet;
    uint8_t version;
    uint8_t remaining_ms;
    uint16_t missing_mask;
    uint32_t session_id;
    uint32_t base_sequence;
} __attribute__((packed)) dual_end_audio_nack_packet_t;

typedef struct {
    dual_end_packet_header_t packet;
    uint32_t session_id;
    dual_end_audio_frame_header_t audio;
    /* CRC covers the two channel codewords in their normal-packet order.
     * HID, capture timing and analysis metadata are deliberately not replayed. */
    uint8_t encoded[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES];
} __attribute__((packed)) dual_end_audio_repair_packet_t;

_Static_assert(sizeof(dual_end_audio_session_packet_t) == 22, "audio session wire size");
_Static_assert(sizeof(dual_end_audio_nack_packet_t) == 22, "audio NACK wire size");
_Static_assert(sizeof(dual_end_audio_repair_packet_t) == 350, "dual audio repair wire size");
_Static_assert(sizeof(dual_end_audio_repair_packet_t) <= DUAL_END_ESPNOW_MAX_DATA_BYTES,
               "audio repair exceeds ESP-NOW payload limit");
_Static_assert(DUAL_END_AUDIO_CHANNELS == 2, "repair protocol requires two audio channels");
_Static_assert(DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES == 320,
               "dual-microphone repair payload must stay 320 bytes");
_Static_assert(DUAL_END_ESPNOW_PADDING_BYTES == 30,
               "normal dual-microphone packet metadata reservation");
_Static_assert(DUAL_END_AUDIO_PACKET_BYTES == 400, "original audio wire size");

/* The caller validates received metadata and the whole-payload CRC first.
 * Keep a single sequence/window/deadline for the two independent codecs. */
static inline void dual_end_audio_recovery_pack(
    uint8_t encoded[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES],
    const dual_end_audio_hardware_payload_t *payload)
{
    memcpy(encoded, payload->audio, DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES);
}

/* Unsigned subtraction gives defined, wrap-safe ordering within half the space. */
static inline bool dual_end_audio_seq_before(uint32_t a, uint32_t b)
{
    return a != b && (a - b) >= UINT32_C(0x80000000);
}

static inline bool dual_end_audio_deadline_valid(int64_t now_us, int64_t created_us)
{
    return now_us >= created_us && now_us - created_us < DUAL_END_AUDIO_TX_MAX_AGE_US;
}

static inline bool dual_end_audio_recovery_header_valid(
    const dual_end_packet_header_t *header, size_t length, uint8_t type, uint8_t role)
{
    return header != NULL && length >= sizeof(*header) &&
           header->protocol_version == SCEPTER_PROTOCOL_VERSION &&
           header->source_role == role && header->message_type == type &&
           header->payload_len == length - sizeof(*header);
}
