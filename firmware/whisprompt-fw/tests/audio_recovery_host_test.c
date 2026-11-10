/* Executes the same portable C state machines linked into both firmwares.
 * Times are deterministic microseconds; no hardware, network or codec stubs
 * are needed to exercise the recovery window and original-byte TX history. */
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "audio_recovery_tx.h"
#include "audio_recovery_rx.h"

static unsigned tests_run;
#define RUN(test) do { test(); ++tests_run; printf("PASS %s\n", #test); fflush(stdout); } while (0)

static dual_end_audio_frame_header_t frame_header(void)
{
    dual_end_audio_frame_header_t header = {0};
    header.codec = DUAL_END_AUDIO_CODEC_G722;
    header.channels = 2;
    header.sample_rate_hz = 16000;
    header.samples_per_channel = 320;
    header.pcm_bytes_after_decode = 1280;
    header.current_encoded_bytes = 320;
    return header;
}

static void encoded_frame(uint32_t sequence, uint8_t *bytes)
{
    for (size_t i = 0; i < DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES; ++i) {
        bytes[i] = (uint8_t)(sequence * 31U + i * 7U);
    }
}

static void cache_store(audio_recovery_tx_t *cache, uint32_t seq, int64_t now)
{
    const dual_end_audio_frame_header_t header = frame_header();
    uint8_t bytes[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES];
    encoded_frame(seq, bytes);
    audio_recovery_tx_store(cache, seq, now, &header, bytes);
    memset(bytes, 0xFF, sizeof(bytes)); /* Caller buffer lifetime must not matter. */
}

static void test_wire_layout_and_reservation(void)
{
    assert(sizeof(dual_end_packet_header_t) == 10);
    assert(sizeof(dual_end_audio_frame_header_t) == 16);
    assert(sizeof(dual_end_hid_input_payload_t) == 24);
    assert(DUAL_END_ESPNOW_PADDING_BYTES == 30);
    assert(sizeof(dual_end_audio_hardware_payload_t) == 374);
    assert(offsetof(dual_end_audio_hardware_payload_t, hardware) == 320);
    assert(offsetof(dual_end_audio_hardware_payload_t, stream_id) == 344);
    assert(sizeof(dual_end_audio_session_packet_t) == 22);
    assert(sizeof(dual_end_audio_nack_packet_t) == 22);
    assert(sizeof(dual_end_audio_repair_packet_t) == 350);
    assert(DUAL_END_AUDIO_RECOVERY_VERSION == 5);
    assert(DUAL_END_AUDIO_G722_FRAME_BYTES == 320);
    assert(DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES == 320);
    assert(DUAL_END_AUDIO_CHANNELS == 2);
    assert(DUAL_END_AUDIO_FRAME_BYTES == 1280);
    assert(offsetof(dual_end_audio_repair_packet_t, encoded) == 30);
    assert(sizeof(dual_end_packet_header_t) + sizeof(dual_end_audio_frame_header_t) +
           sizeof(dual_end_audio_hardware_payload_t) == 400);
}

static void test_dual_mic_repair_pack_excludes_metadata(void)
{
    dual_end_audio_hardware_payload_t payload;
    memset(&payload, 0xA5, sizeof(payload));
    for (size_t i = 0; i < sizeof(payload.audio); ++i) {
        payload.audio[i] = (uint8_t)(i * 7U + 3U);
    }
    uint8_t combined[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES];
    dual_end_audio_recovery_pack(combined, &payload);
    assert(memcmp(combined, payload.audio, sizeof(combined)) == 0);
    memset(&payload.hardware, 0, sizeof(payload.hardware));
    payload.stream_id = 0;
    payload.capture_sample_index = 0;
    memset(&payload.analysis, 0, sizeof(payload.analysis));
    assert(memcmp(combined, payload.audio, sizeof(combined)) == 0);
}

static void test_control_header_validation(void)
{
    dual_end_audio_nack_packet_t packet = {0};
    packet.packet.protocol_version = SCEPTER_PROTOCOL_VERSION;
    packet.packet.source_role = SCEPTER_ROLE_DONGLE;
    packet.packet.message_type = SCEPTER_MSG_AUDIO_NACK;
    packet.packet.payload_len = sizeof(packet) - sizeof(packet.packet);
    assert(dual_end_audio_recovery_header_valid(&packet.packet, sizeof(packet),
                                               SCEPTER_MSG_AUDIO_NACK, SCEPTER_ROLE_DONGLE));
    assert(!dual_end_audio_recovery_header_valid(NULL, sizeof(packet),
                                                SCEPTER_MSG_AUDIO_NACK, SCEPTER_ROLE_DONGLE));
    assert(!dual_end_audio_recovery_header_valid(&packet.packet, sizeof(packet) - 1,
                                                SCEPTER_MSG_AUDIO_NACK, SCEPTER_ROLE_DONGLE));
    assert(!dual_end_audio_recovery_header_valid(&packet.packet, sizeof(packet),
                                                SCEPTER_MSG_AUDIO_NACK, SCEPTER_ROLE_SCEPTER));
    packet.packet.protocol_version++;
    assert(!dual_end_audio_recovery_header_valid(&packet.packet, sizeof(packet),
                                                SCEPTER_MSG_AUDIO_NACK, SCEPTER_ROLE_DONGLE));
}

static void test_sequence_wrap_order(void)
{
    assert(dual_end_audio_seq_before(UINT32_MAX, 0));
    assert(dual_end_audio_seq_before(UINT32_MAX - 2, 1));
    assert(!dual_end_audio_seq_before(0, UINT32_MAX));
    assert(!dual_end_audio_seq_before(123, 123));
    assert(dual_end_audio_seq_before(100, 101));
}

static void test_tx_original_bytes_are_copied(void)
{
    audio_recovery_tx_t tx;
    audio_recovery_tx_reset(&tx, 17);
    cache_store(&tx, 101, 1000000);
    const audio_recovery_tx_entry_t *entry = NULL;
    assert(audio_recovery_tx_get(&tx, 17, 101, 1000100, &entry) == AUDIO_RECOVERY_TX_HIT);
    uint8_t expected[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES];
    encoded_frame(101, expected);
    assert(memcmp(entry->encoded, expected, sizeof(expected)) == 0);
    assert(entry->audio.current_encoded_bytes == DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES);
    assert(entry->created_us == 1000000);
}

static void test_tx_session_and_cache_miss(void)
{
    audio_recovery_tx_t tx;
    audio_recovery_tx_reset(&tx, 17);
    cache_store(&tx, 101, 1000000);
    const audio_recovery_tx_entry_t *entry = NULL;
    assert(audio_recovery_tx_get(&tx, 18, 101, 1000100, &entry) == AUDIO_RECOVERY_TX_SESSION);
    assert(entry == NULL);
    assert(audio_recovery_tx_get(&tx, 0, 101, 1000100, &entry) == AUDIO_RECOVERY_TX_SESSION);
    assert(audio_recovery_tx_get(&tx, 17, 102, 1000100, &entry) == AUDIO_RECOVERY_TX_MISS);
    audio_recovery_tx_reset(&tx, 18);
    assert(audio_recovery_tx_get(&tx, 18, 101, 1000100, &entry) == AUDIO_RECOVERY_TX_MISS);
}

static void test_tx_deadline_boundaries(void)
{
    const int64_t created_us = 1000000;
    const int64_t deadline_us = created_us + DUAL_END_AUDIO_TX_CACHE_MAX_AGE_US;
    audio_recovery_tx_t tx;
    audio_recovery_tx_reset(&tx, 17);
    cache_store(&tx, 101, created_us);
    const audio_recovery_tx_entry_t *entry = NULL;
    assert(audio_recovery_tx_get(&tx, 17, 101, deadline_us - 1, &entry) == AUDIO_RECOVERY_TX_HIT);
    assert(audio_recovery_tx_get(&tx, 17, 101, deadline_us, &entry) == AUDIO_RECOVERY_TX_EXPIRED);
    assert(entry == NULL);
    assert(audio_recovery_tx_get(&tx, 17, 101, created_us - 1, &entry) == AUDIO_RECOVERY_TX_EXPIRED);
    assert(dual_end_audio_deadline_valid(created_us + DUAL_END_AUDIO_TX_MAX_AGE_US - 1, created_us));
    assert(!dual_end_audio_deadline_valid(deadline_us, created_us));
    assert(!dual_end_audio_deadline_valid(created_us + 450000, created_us));
}

static void test_tx_extended_deadline_preserves_original_repair_bytes(void)
{
    const int64_t created_us = 1000000;
    const int64_t previous_max_age_us = 80000;
    const int64_t repair_age_us = (previous_max_age_us + DUAL_END_AUDIO_TX_MAX_AGE_US) / 2;
    assert(DUAL_END_AUDIO_TX_MAX_AGE_US == 100000);
    assert(repair_age_us > previous_max_age_us && repair_age_us < DUAL_END_AUDIO_TX_MAX_AGE_US);
    audio_recovery_tx_t tx;
    audio_recovery_tx_reset(&tx, 17);
    cache_store(&tx, 101, created_us);
    const audio_recovery_tx_entry_t *entry = NULL;
    assert(audio_recovery_tx_get(&tx, 17, 101, created_us + repair_age_us, &entry) == AUDIO_RECOVERY_TX_HIT);
    uint8_t expected[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES];
    encoded_frame(101, expected);
    assert(memcmp(entry->encoded, expected, sizeof(expected)) == 0);
    assert(audio_recovery_tx_budget_available(&tx, created_us + repair_age_us));
    audio_recovery_tx_commit(&tx, 101, created_us + repair_age_us);
    assert(tx.budget_used == 1);
    assert(audio_recovery_tx_get(&tx, 17, 101,
        created_us + DUAL_END_AUDIO_TX_CACHE_MAX_AGE_US, &entry) == AUDIO_RECOVERY_TX_EXPIRED);
    assert(entry == NULL);
}

static void test_tx_ring_replacement_and_wrap(void)
{
    audio_recovery_tx_t tx;
    audio_recovery_tx_reset(&tx, 17);
    cache_store(&tx, 101, 1000000);
    cache_store(&tx, 101 + DUAL_END_AUDIO_TX_CACHE_WINDOW, 1000010);
    const audio_recovery_tx_entry_t *entry = NULL;
    assert(audio_recovery_tx_get(&tx, 17, 101, 1000020, &entry) == AUDIO_RECOVERY_TX_MISS);
    assert(audio_recovery_tx_get(&tx, 17, 101 + DUAL_END_AUDIO_TX_CACHE_WINDOW, 1000020, &entry) == AUDIO_RECOVERY_TX_HIT);
    cache_store(&tx, UINT32_MAX, 1000030);
    cache_store(&tx, 0, 1000040);
    assert(audio_recovery_tx_get(&tx, 17, UINT32_MAX, 1000050, &entry) == AUDIO_RECOVERY_TX_HIT);
    assert(audio_recovery_tx_get(&tx, 17, 0, 1000050, &entry) == AUDIO_RECOVERY_TX_HIT);
}

static void test_tx_duplicate_nack_and_repair_limit(void)
{
    const int64_t first_repair_us = 1010000;
    const int64_t second_repair_us = first_repair_us + DUAL_END_AUDIO_REPAIR_MIN_INTERVAL_US;
    audio_recovery_tx_t tx;
    audio_recovery_tx_reset(&tx, 17);
    cache_store(&tx, 101, 1000000);
    const audio_recovery_tx_entry_t *entry = NULL;
    assert(audio_recovery_tx_budget_available(&tx, first_repair_us));
    assert(audio_recovery_tx_get(&tx, 17, 101, first_repair_us, &entry) == AUDIO_RECOVERY_TX_HIT);
    /* No commit when local enqueue fails: the missing frame remains eligible. */
    assert(audio_recovery_tx_get(&tx, 17, 101, first_repair_us + 1, &entry) == AUDIO_RECOVERY_TX_HIT);
    audio_recovery_tx_commit(&tx, 101, first_repair_us);
    assert(audio_recovery_tx_get(&tx, 17, 101, second_repair_us - 1, &entry) == AUDIO_RECOVERY_TX_TOO_SOON);
    assert(audio_recovery_tx_get(&tx, 17, 101, second_repair_us, &entry) == AUDIO_RECOVERY_TX_HIT);
    audio_recovery_tx_commit(&tx, 101, second_repair_us);
    assert(audio_recovery_tx_get(&tx, 17, 101,
        second_repair_us + DUAL_END_AUDIO_REPAIR_MIN_INTERVAL_US, &entry) == AUDIO_RECOVERY_TX_LIMIT);
}

static void test_tx_global_repair_budget(void)
{
    const int64_t start = 1010000;
    audio_recovery_tx_t tx;
    audio_recovery_tx_reset(&tx, 17);
    for (uint32_t i = 0; i < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS; ++i) {
        cache_store(&tx, i, 1000000);
        assert(audio_recovery_tx_budget_available(&tx, start));
        audio_recovery_tx_commit(&tx, i, start);
    }
    assert(!audio_recovery_tx_budget_available(&tx, start + DUAL_END_AUDIO_REPAIR_BUDGET_US - 1));
    assert(audio_recovery_tx_budget_available(&tx, start + DUAL_END_AUDIO_REPAIR_BUDGET_US));
}

static audio_rx_push_result_t rx_push(audio_recovery_rx_t *rx, uint32_t session,
                                     uint32_t sequence, int64_t now, bool repair)
{
    uint8_t bytes[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES];
    encoded_frame(sequence, bytes);
    return audio_recovery_rx_push(rx, session, sequence, bytes, -61, repair, now);
}

static audio_rx_event_t expect_event(audio_recovery_rx_t *rx, int64_t now,
                                     audio_rx_event_kind_t kind, uint32_t seq)
{
    audio_rx_event_t event;
    assert(audio_recovery_rx_next(rx, now, &event));
    assert(event.kind == kind);
    assert(event.sequence == seq);
    if (kind == AUDIO_RX_NACK) {
        /* Legacy scenarios assume an immediately available radio. Dedicated
         * admission tests below use next/finish separately to model queueing. */
        audio_recovery_rx_finish_nack(rx, event.nack_token, AUDIO_RX_NACK_ACCEPTED, now);
    }
    if (kind == AUDIO_RX_FRAME) {
        uint8_t expected[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES];
        encoded_frame(seq, expected);
        assert(memcmp(event.encoded, expected, sizeof(expected)) == 0);
    }
    return event;
}

static void expect_quiet(audio_recovery_rx_t *rx, int64_t now)
{
    audio_rx_event_t event;
    assert(!audio_recovery_rx_next(rx, now, &event));
}

static void begin_with_frame100(audio_recovery_rx_t *rx)
{
    audio_recovery_rx_init(rx);
    assert(audio_recovery_rx_begin(rx, 17, 100, 1000000) == AUDIO_RX_SESSION_NEW);
    assert(rx_push(rx, 17, 100, 1000000, false) == AUDIO_RX_ACCEPTED);
    expect_event(rx, 1000000, AUDIO_RX_FRAME, 100);
}

static void test_rx_requires_valid_session(void)
{
    audio_recovery_rx_t rx;
    audio_recovery_rx_init(&rx);
    assert(rx_push(&rx, 17, 100, 1000000, false) == AUDIO_RX_REJECTED);
    assert(audio_recovery_rx_begin(&rx, 0, 100, 1000000) == AUDIO_RX_SESSION_REJECTED);
    assert(audio_recovery_rx_begin(&rx, 17, 100, 1000000) == AUDIO_RX_SESSION_NEW);
    assert(rx_push(&rx, 18, 100, 1000001, true) == AUDIO_RX_REJECTED);
    expect_quiet(&rx, 1000001);
}

static void test_rx_in_order_has_no_artificial_hold(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    for (uint32_t seq = 101; seq <= 200; ++seq) {
        const int64_t now = 1000000 + (seq - 100) * 20000LL;
        assert(rx_push(&rx, 17, seq, now, false) == AUDIO_RX_ACCEPTED);
        expect_event(&rx, now, AUDIO_RX_FRAME, seq);
        expect_quiet(&rx, now);
    }
    assert(rx.stats.output_frames == 101);
    assert(rx.stats.nack_requests == 0);
    assert(rx.stats.permanent_loss == 0);
}

static void test_rx_duplicates_before_and_after_output(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 100, 1000001, true) == AUDIO_RX_DUPLICATE);
    assert(rx_push(&rx, 17, 102, 1020000, false) == AUDIO_RX_ACCEPTED);
    assert(rx_push(&rx, 17, 102, 1020001, false) == AUDIO_RX_DUPLICATE);
    assert(rx_push(&rx, 17, 101, 1021000, false) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1021000, AUDIO_RX_FRAME, 101);
    expect_event(&rx, 1021000, AUDIO_RX_FRAME, 102);
    expect_quiet(&rx, 1021000);
    assert(rx.stats.duplicates == 2);
    assert(rx.stats.output_frames == 3);
    assert(rx.stats.permanent_loss == 0);
}

static void test_rx_hid_priority_arrival_order_is_repaired(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 103, 1020000, false) == AUDIO_RX_ACCEPTED);
    expect_quiet(&rx, 1020000);
    assert(rx_push(&rx, 17, 101, 1021000, false) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1021000, AUDIO_RX_FRAME, 101);
    expect_quiet(&rx, 1021000);
    assert(rx_push(&rx, 17, 102, 1022000, false) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1022000, AUDIO_RX_FRAME, 102);
    expect_event(&rx, 1022000, AUDIO_RX_FRAME, 103);
    assert(rx.stats.nack_requests == 0);
    assert(rx.stats.permanent_loss == 0);
}

static void test_rx_single_loss_recovered_once(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 102, 1020000, false) == AUDIO_RX_ACCEPTED);
    expect_quiet(&rx, 1023999);
    audio_rx_event_t nack = expect_event(&rx, 1024000, AUDIO_RX_NACK, 101);
    assert(nack.missing_mask == 1 && nack.session_id == 17);
    assert(nack.first_sequence == 100);
    assert(rx_push(&rx, 17, 101, 1028000, true) == AUDIO_RX_ACCEPTED);
    audio_rx_event_t frame = expect_event(&rx, 1028000, AUDIO_RX_FRAME, 101);
    assert(frame.repaired);
    expect_event(&rx, 1028000, AUDIO_RX_FRAME, 102);
    assert(rx_push(&rx, 17, 101, 1029000, false) == AUDIO_RX_DUPLICATE);
    expect_quiet(&rx, 1060000);
    assert(rx.stats.recovered == 1 && rx.stats.permanent_loss == 0);
}

static void test_rx_lost_nack_is_retried_but_bounded(void)
{
    const int64_t gap_started_us = 1040000;
    const int64_t first_nack_us = gap_started_us + DUAL_END_AUDIO_REORDER_GRACE_US;
    const int64_t deadline_us = 1020000 + DUAL_END_AUDIO_PLAYOUT_DELAY_US - DUAL_END_AUDIO_DECODE_MARGIN_US;
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 102, gap_started_us, false) == AUDIO_RX_ACCEPTED);
    assert(DUAL_END_AUDIO_NACK_MAX_REQUESTS == 4 && DUAL_END_AUDIO_NACK_INTERVAL_US == 60000);
    for (unsigned round = 0; round < DUAL_END_AUDIO_NACK_MAX_REQUESTS; ++round) {
        int64_t due = first_nack_us + (int64_t)round * DUAL_END_AUDIO_NACK_INTERVAL_US;
        expect_quiet(&rx, due - 1);
        audio_rx_event_t nack = expect_event(&rx, due, AUDIO_RX_NACK, 101);
        assert(nack.remaining_ms == 200 - round * 60);
        assert(audio_recovery_rx_playout_us(&rx, 101) - DUAL_END_AUDIO_DECODE_MARGIN_US == deadline_us);
    }
    expect_quiet(&rx, deadline_us - 1);
    expect_event(&rx, deadline_us, AUDIO_RX_MISSING, 101);
    expect_event(&rx, deadline_us, AUDIO_RX_FRAME, 102);
    expect_quiet(&rx, deadline_us);
    assert(rx.stats.nack_requests == DUAL_END_AUDIO_NACK_MAX_REQUESTS);
    assert(rx.stats.permanent_loss == 1);
}

static void test_rx_deadline_and_late_repair_never_replay(void)
{
    const int64_t gap_started_us = 1020000;
    const int64_t deadline_us = 1020000 + DUAL_END_AUDIO_PLAYOUT_DELAY_US - DUAL_END_AUDIO_DECODE_MARGIN_US;
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 102, gap_started_us, false) == AUDIO_RX_ACCEPTED);
    expect_quiet(&rx, gap_started_us);
    /* Production adapter drains time-based events before accepting a packet. */
    expect_event(&rx, deadline_us, AUDIO_RX_MISSING, 101);
    expect_event(&rx, deadline_us, AUDIO_RX_FRAME, 102);
    assert(rx_push(&rx, 17, 101, deadline_us, true) == AUDIO_RX_LATE);
    expect_quiet(&rx, deadline_us + 1);
    assert(rx.stats.late == 1 && rx.stats.recovered == 0);
}

static void test_rx_last_instant_repair_is_useful(void)
{
    const int64_t gap_started_us = 1020000;
    const int64_t repair_us = 1020000 + DUAL_END_AUDIO_PLAYOUT_DELAY_US - DUAL_END_AUDIO_DECODE_MARGIN_US - 1;
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 102, gap_started_us, false) == AUDIO_RX_ACCEPTED);
    expect_quiet(&rx, gap_started_us);
    expect_quiet(&rx, repair_us); /* Too little time left to request another repair. */
    assert(rx_push(&rx, 17, 101, repair_us, true) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, repair_us, AUDIO_RX_FRAME, 101);
    expect_event(&rx, repair_us, AUDIO_RX_FRAME, 102);
    assert(rx.stats.permanent_loss == 0);
}

static void test_rx_consecutive_gaps_have_independent_deadlines(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 103, 1060000, false) == AUDIO_RX_ACCEPTED);
    audio_rx_event_t nack = expect_event(&rx, 1064000, AUDIO_RX_NACK, 101);
    assert(nack.missing_mask == 3);
    const int64_t first = audio_recovery_rx_playout_us(&rx, 101) - DUAL_END_AUDIO_DECODE_MARGIN_US;
    const int64_t second = audio_recovery_rx_playout_us(&rx, 102) - DUAL_END_AUDIO_DECODE_MARGIN_US;
    assert(second - first == 20000);
    expect_event(&rx, first, AUDIO_RX_MISSING, 101);
    /* A later missing frame retains its own deadline and may request again. */
    audio_rx_event_t event;
    while (audio_recovery_rx_next(&rx, first, &event)) {
        assert(event.kind == AUDIO_RX_NACK);
        audio_recovery_rx_finish_nack(&rx, event.nack_token, AUDIO_RX_NACK_ACCEPTED, first);
    }
    assert(rx.stats.permanent_loss == 1);
    expect_event(&rx, second, AUDIO_RX_MISSING, 102);
    expect_event(&rx, second, AUDIO_RX_FRAME, 103);
    assert(rx.stats.permanent_loss == 2 && rx.active);
}

static void test_rx_new_packets_do_not_extend_deadline(void)
{
    const int64_t deadline_us = 1040000 + DUAL_END_AUDIO_PLAYOUT_DELAY_US - DUAL_END_AUDIO_DECODE_MARGIN_US;
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 103, 1040000, false) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1044000, AUDIO_RX_NACK, 101);
    assert(rx_push(&rx, 17, 101, 1048000, true) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1048000, AUDIO_RX_FRAME, 101);
    assert(rx_push(&rx, 17, 104, 1070000, false) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, deadline_us, AUDIO_RX_MISSING, 102);
    expect_event(&rx, deadline_us, AUDIO_RX_FRAME, 103);
    expect_event(&rx, deadline_us, AUDIO_RX_FRAME, 104);
    assert(rx.stats.recovered == 1 && rx.stats.permanent_loss == 1);
}

static void test_rx_partial_repair_reduces_nack_mask(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 103, 1040000, false) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1044000, AUDIO_RX_NACK, 101);
    assert(rx_push(&rx, 17, 101, 1048000, true) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1048000, AUDIO_RX_FRAME, 101);
    audio_rx_event_t nack = expect_event(&rx, 1056000, AUDIO_RX_NACK, 102);
    assert(nack.missing_mask == 1);
    expect_event(&rx, 1056000 + DUAL_END_AUDIO_NACK_INTERVAL_US, AUDIO_RX_NACK, 102);
}

static void test_rx_no_last_moment_nack(void)
{
    const int64_t gap_started_us = 1020000;
    const int64_t deadline_us = 1020000 + DUAL_END_AUDIO_PLAYOUT_DELAY_US - DUAL_END_AUDIO_DECODE_MARGIN_US;
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 102, gap_started_us, false) == AUDIO_RX_ACCEPTED);
    expect_quiet(&rx, deadline_us - 1000);
    expect_event(&rx, deadline_us, AUDIO_RX_MISSING, 101);
    assert(rx.stats.nack_requests == 0);
}

static void test_rx_sequence_wrap_reordering(void)
{
    audio_recovery_rx_t rx;
    audio_recovery_rx_init(&rx);
    assert(audio_recovery_rx_begin(&rx, 17, UINT32_MAX - 1U, 1000000) == AUDIO_RX_SESSION_NEW);
    assert(rx_push(&rx, 17, 0, 1000000, false) == AUDIO_RX_ACCEPTED);
    assert(rx_push(&rx, 17, UINT32_MAX - 1U, 1000100, false) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1000100, AUDIO_RX_FRAME, UINT32_MAX - 1U);
    assert(rx_push(&rx, 17, UINT32_MAX, 1000200, true) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1000200, AUDIO_RX_FRAME, UINT32_MAX);
    expect_event(&rx, 1000200, AUDIO_RX_FRAME, 0);
    assert(rx.next_sequence == 1);
    assert(rx_push(&rx, 17, UINT32_MAX, 1000300, false) == AUDIO_RX_DUPLICATE);
}

static void test_rx_duplicate_session_is_idempotent(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(audio_recovery_rx_begin(&rx, 17, 100, 1020000) == AUDIO_RX_SESSION_DUPLICATE);
    assert(rx.next_sequence == 101 && rx.stats.output_frames == 1);
    assert(audio_recovery_rx_begin(&rx, 17, 999, 1020000) == AUDIO_RX_SESSION_REJECTED);
    assert(rx.next_sequence == 101);
}

static void test_rx_retired_session_and_old_repairs_are_rejected(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(audio_recovery_rx_begin(&rx, 18, 500, 1020000) == AUDIO_RX_SESSION_NEW);
    assert(audio_recovery_rx_begin(&rx, 17, 100, 1020001) == AUDIO_RX_SESSION_REJECTED);
    assert(rx_push(&rx, 17, 500, 1020002, true) == AUDIO_RX_REJECTED);
    assert(rx.session_id == 18 && rx.next_sequence == 500);
    for (uint32_t id = 19; id <= 30; ++id) {
        assert(audio_recovery_rx_begin(&rx, id, id * 100, 1020000 + id) == AUDIO_RX_SESSION_NEW);
    }
    for (uint32_t id = 22; id < 30; ++id) {
        assert(audio_recovery_rx_begin(&rx, id, id * 100, 1040000) == AUDIO_RX_SESSION_REJECTED);
    }
}

static void test_rx_idle_and_duplicates_do_not_keep_stream_alive(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 100, 1200000, false) == AUDIO_RX_DUPLICATE);
    audio_rx_event_t event;
    while (audio_recovery_rx_next(&rx, 1299999, &event)) assert(event.kind == AUDIO_RX_MISSING);
    assert(rx.next_sequence == 104 && rx.stats.permanent_loss == 3);
    expect_event(&rx, 1300000, AUDIO_RX_RESYNC, 104);
    expect_quiet(&rx, 1359999);
    expect_event(&rx, 1360000, AUDIO_RX_RESYNC, 104);
    assert(!rx.active);
    assert(audio_recovery_rx_begin(&rx, 17, 100, 1360001) == AUDIO_RX_SESSION_REJECTED);
    assert(audio_recovery_rx_begin(&rx, 18, 500, 1360002) == AUDIO_RX_SESSION_NEW);
}

static void test_rx_far_jump_resync_is_bounded(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 101 + DUAL_END_AUDIO_RECOVERY_WINDOW, 1020000, false) == AUDIO_RX_REJECTED);
    expect_event(&rx, 1020000, AUDIO_RX_RESYNC, 101);
    expect_quiet(&rx, 1020000);
    assert(rx.stats.permanent_loss == 0);
}

static void test_rx_four_missing_frames_preserve_playback_session(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 105, 1100000, false) == AUDIO_RX_ACCEPTED);
    for (uint32_t seq = 101; seq <= 104; seq++) {
        int64_t deadline = audio_recovery_rx_playout_us(&rx, seq) - DUAL_END_AUDIO_DECODE_MARGIN_US;
        expect_event(&rx, deadline, AUDIO_RX_MISSING, seq);
        assert(rx.active);
    }
    expect_event(&rx, audio_recovery_rx_playout_us(&rx, 104) - DUAL_END_AUDIO_DECODE_MARGIN_US,
                 AUDIO_RX_FRAME, 105);
    assert(rx.stats.permanent_loss == 4 && rx.stats.abort_count == 0);
}

static void test_rx_cold_start_request_is_bounded_and_stops_on_begin(void)
{
    audio_recovery_rx_t rx;
    audio_recovery_rx_init(&rx);
    expect_quiet(&rx, 1000000);
    audio_recovery_rx_request_session(&rx, 1000000);
    audio_rx_event_t event = expect_event(&rx, 1000000, AUDIO_RX_RESYNC, 0);
    assert(event.session_id == 0 && event.first_sequence == 0);
    audio_recovery_rx_request_session(&rx, 1000001);
    expect_quiet(&rx, 1059999);
    expect_event(&rx, 1060000, AUDIO_RX_RESYNC, 0);
    assert(audio_recovery_rx_begin(&rx, 17, 100, 1060001) == AUDIO_RX_SESSION_NEW);
    audio_recovery_rx_request_session(&rx, 1060002);
    expect_quiet(&rx, 1120000);
    assert(rx_push(&rx, 17, 100, 1120001, false) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1120001, AUDIO_RX_FRAME, 100);
}

static void test_rx_packets_queued_before_session_cannot_enter_new_decoder(void)
{
    audio_recovery_rx_t rx;
    audio_recovery_rx_init(&rx);
    assert(audio_recovery_rx_begin(&rx, 17, 100, 1000000) == AUDIO_RX_SESSION_NEW);
    assert(rx_push(&rx, 17, 100, 999999, false) == AUDIO_RX_REJECTED);
    expect_quiet(&rx, 1000001);
    assert(rx_push(&rx, 17, 100, 1000002, false) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1000002, AUDIO_RX_FRAME, 100);
    assert(rx.stats.rejected == 1 && rx.stats.output_frames == 1);
}

static void test_end_to_end_loss_and_duplicate_nack(void)
{
    audio_recovery_tx_t tx;
    audio_recovery_rx_t rx;
    audio_recovery_tx_reset(&tx, 17);
    begin_with_frame100(&rx);
    cache_store(&tx, 101, 1020000); /* Original 101 is deliberately lost. */
    cache_store(&tx, 102, 1040000);
    assert(rx_push(&rx, 17, 102, 1040000, false) == AUDIO_RX_ACCEPTED);
    audio_rx_event_t nack = expect_event(&rx, 1044000, AUDIO_RX_NACK, 101);
    assert(nack.missing_mask == 1);
    const audio_recovery_tx_entry_t *entry = NULL;
    assert(audio_recovery_tx_get(&tx, nack.session_id, nack.sequence, 1046000, &entry) == AUDIO_RECOVERY_TX_HIT);
    assert(audio_recovery_tx_budget_available(&tx, 1046000));
    uint8_t original[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES];
    memcpy(original, entry->encoded, sizeof(original));
    audio_recovery_tx_commit(&tx, nack.sequence, 1046000);
    assert(audio_recovery_tx_get(&tx, 17, 101, 1046001, &entry) == AUDIO_RECOVERY_TX_TOO_SOON);
    expect_quiet(&rx, 1048000);
    assert(audio_recovery_rx_push(&rx, 17, 101, original, -61, true, 1048000) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1048000, AUDIO_RX_FRAME, 101);
    expect_event(&rx, 1048000, AUDIO_RX_FRAME, 102);
    assert(rx.stats.recovered == 1 && rx.stats.permanent_loss == 0);
    assert(tx.budget_used == 1);
}

static void test_extended_gap_recovers_after_previous_deadline(void)
{
    const int64_t previous_gap_wait_us = 40000;
    const int64_t legacy_v2_gap_wait_us = 60000;
    const int64_t gap_started_us = 1060000;
    const int64_t created_us = 1020000;
    const int64_t repair_delay_us = (previous_gap_wait_us + legacy_v2_gap_wait_us) / 2;
    const int64_t repair_us = gap_started_us + repair_delay_us;
    assert(legacy_v2_gap_wait_us == 60000);
    assert(repair_delay_us > previous_gap_wait_us && repair_delay_us < legacy_v2_gap_wait_us);
    assert(repair_us - created_us > 80000);

    audio_recovery_tx_t tx;
    audio_recovery_rx_t rx;
    audio_recovery_tx_reset(&tx, 17);
    begin_with_frame100(&rx);
    cache_store(&tx, 101, created_us); /* Lost original; later packet is delayed too. */
    assert(rx_push(&rx, 17, 102, gap_started_us, false) == AUDIO_RX_ACCEPTED);
    const int64_t first_nack_us = gap_started_us + DUAL_END_AUDIO_REORDER_GRACE_US;
    audio_rx_event_t nack = expect_event(&rx, first_nack_us, AUDIO_RX_NACK, 101);
    expect_quiet(&rx, gap_started_us + previous_gap_wait_us);
    expect_quiet(&rx, repair_us); /* Recovery time is not capped by the old gap wait. */

    const audio_recovery_tx_entry_t *entry = NULL;
    assert(audio_recovery_tx_get(&tx, nack.session_id, nack.sequence,
                                 repair_us, &entry) == AUDIO_RECOVERY_TX_HIT);
    assert(audio_recovery_tx_budget_available(&tx, repair_us));
    assert(audio_recovery_rx_push(&rx, 17, 101, entry->encoded,
                                  -61, true, repair_us) == AUDIO_RX_ACCEPTED);
    audio_recovery_tx_commit(&tx, 101, repair_us);
    audio_rx_event_t frame = expect_event(&rx, repair_us, AUDIO_RX_FRAME, 101);
    assert(frame.repaired);
    expect_event(&rx, repair_us, AUDIO_RX_FRAME, 102);
    expect_quiet(&rx, gap_started_us + legacy_v2_gap_wait_us);
    assert(rx.stats.nack_requests == 1);
    assert(rx.stats.recovered == 1 && rx.stats.permanent_loss == 0);
    assert(tx.budget_used == 1);
}

static void test_rx_nack_admission_does_not_spend_queued_rounds(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 102, 1020000, false) == AUDIO_RX_ACCEPTED);
    audio_rx_event_t nack;
    assert(audio_recovery_rx_next(&rx, 1024000, &nack) && nack.kind == AUDIO_RX_NACK);
    assert(rx.nack_rounds == 0 && rx.stats.nack_accepted == 0);
    expect_quiet(&rx, 1036000); /* Pending behind another callback, no duplicate. */
    audio_recovery_rx_finish_nack(&rx, nack.nack_token, AUDIO_RX_NACK_REJECTED, 1036000);
    assert(rx.nack_rounds == 0);
    expect_quiet(&rx, 1039999); /* Immediate admission failures are bounded. */
    assert(audio_recovery_rx_next(&rx, 1040000, &nack) && nack.kind == AUDIO_RX_NACK);
    audio_recovery_rx_finish_nack(&rx, nack.nack_token, AUDIO_RX_NACK_ACCEPTED, 1041000);
    assert(rx.nack_rounds == 1 && rx.stats.nack_accepted == 1);
    const int64_t second_due = 1041000 + DUAL_END_AUDIO_NACK_INTERVAL_US;
    expect_quiet(&rx, second_due - 1);
    assert(audio_recovery_rx_next(&rx, second_due, &nack) && nack.kind == AUDIO_RX_NACK);
    assert(rx.nack_rounds == 1); /* Second generated request still does not count. */
    audio_recovery_rx_finish_nack(&rx, nack.nack_token, AUDIO_RX_NACK_ACCEPTED, second_due + 1000);
    assert(rx.nack_rounds == 2 && rx.stats.nack_accepted == 2);
    expect_quiet(&rx, second_due + DUAL_END_AUDIO_NACK_INTERVAL_US);
}

static void test_rx_pending_mask_refresh_and_session_invalidation(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 104, 1020000, false) == AUDIO_RX_ACCEPTED);
    audio_rx_event_t nack;
    int64_t latest;
    assert(audio_recovery_rx_next(&rx, 1024000, &nack) && nack.kind == AUDIO_RX_NACK);
    assert(nack.missing_mask == 7);
    assert(rx_push(&rx, 17, 102, 1025000, true) == AUDIO_RX_ACCEPTED);
    assert(audio_recovery_rx_refresh_nack(&rx, &nack, 1025000, &latest));
    assert(nack.missing_mask == 5 && latest == 1020000 + DUAL_END_AUDIO_PLAYOUT_DELAY_US -
        DUAL_END_AUDIO_DECODE_MARGIN_US - DUAL_END_AUDIO_MIN_REPAIR_ROUNDTRIP_US);
    assert(!audio_recovery_rx_refresh_nack(&rx, &nack, latest, &latest));
    uint32_t old_token = nack.nack_token;
    assert(audio_recovery_rx_begin(&rx, 18, 200, 1026000) == AUDIO_RX_SESSION_NEW);
    assert(!audio_recovery_rx_refresh_nack(&rx, &nack, 1026000, &latest));
    assert(rx_push(&rx, 18, 201, 1027000, false) == AUDIO_RX_ACCEPTED);
    assert(audio_recovery_rx_next(&rx, 1031000, &nack) && nack.kind == AUDIO_RX_NACK);
    assert(nack.nack_token != old_token);
    audio_recovery_rx_finish_nack(&rx, old_token, AUDIO_RX_NACK_ACCEPTED, 1025500);
    assert(rx.nack_pending && rx.nack_rounds == 0 && rx.stats.nack_accepted == 1);
    audio_recovery_rx_finish_nack(&rx, nack.nack_token, AUDIO_RX_NACK_ACCEPTED, 1032000);
    assert(rx.nack_rounds == 1);
}

static void test_rx_lifetime_counters_account_discarded_buffered_frames(void)
{
    audio_recovery_rx_t rx;
    audio_rx_event_t event;
    audio_recovery_rx_init(&rx);
    assert(audio_recovery_rx_begin(&rx, 1, 100, 1000000) == AUDIO_RX_SESSION_NEW);
    assert(rx_push(&rx, 1, 100, 1000000, false) == AUDIO_RX_ACCEPTED);
    assert(audio_recovery_rx_next(&rx, 1000000, &event) && event.kind == AUDIO_RX_FRAME);
    assert(rx_push(&rx, 1, 102, 1020000, false) == AUDIO_RX_ACCEPTED);
    assert(rx_push(&rx, 1, 103, 1040000, true) == AUDIO_RX_ACCEPTED);
    audio_recovery_rx_abort(&rx, 1040000);
    audio_recovery_rx_abort(&rx, 1040001);
    assert(rx.stats.session_count == 1 && rx.stats.abort_count == 1);
    assert(rx.stats.aborted_buffered_frames == 2 && rx.stats.permanent_loss == 0);
    assert(rx.stats.repair_ontime == 1 && rx.stats.recovered == 0);

    assert(audio_recovery_rx_begin(&rx, 2, 200, 1050000) == AUDIO_RX_SESSION_NEW);
    assert(rx_push(&rx, 2, 201, 1050000, false) == AUDIO_RX_ACCEPTED);
    assert(rx_push(&rx, 2, 202, 1051000, false) == AUDIO_RX_ACCEPTED);
    assert(audio_recovery_rx_begin(&rx, 2, 200, 1051001) == AUDIO_RX_SESSION_DUPLICATE);
    assert(audio_recovery_rx_begin(&rx, 0, 200, 1051002) == AUDIO_RX_SESSION_REJECTED);
    assert(rx.stats.session_count == 2 && rx.stats.aborted_buffered_frames == 2);
    assert(audio_recovery_rx_begin(&rx, 3, 300, 1060000) == AUDIO_RX_SESSION_NEW);
    assert(rx.stats.session_count == 3 && rx.stats.abort_count == 1);
    assert(rx.stats.accepted == 5 && rx.stats.output_frames == 1);
    assert(rx.stats.aborted_buffered_frames == 4 && rx.stats.permanent_loss == 0);
    assert(rx.stats.accepted == rx.stats.output_frames + rx.stats.aborted_buffered_frames);
}

static void test_rx_72_hour_simulated_stream_with_sequence_wrap(void)
{
    audio_recovery_rx_t rx;
    audio_rx_event_t event;
    const uint8_t encoded[DUAL_END_AUDIO_RECOVERY_ENCODED_BYTES] = {0};
    const uint32_t frame_count = 72U * 60U * 60U * 50U;
    const uint32_t first_sequence = UINT32_MAX - 99U;
    audio_recovery_rx_init(&rx);
    assert(audio_recovery_rx_begin(&rx, 1, first_sequence, 0) == AUDIO_RX_SESSION_NEW);
    for (uint32_t i = 0; i < frame_count; ++i) {
        const int64_t now_us = ((int64_t)i + 1) * 20000;
        const uint32_t sequence = first_sequence + i;
        assert(audio_recovery_rx_push(&rx, 1, sequence, encoded, -45, false, now_us)
               == AUDIO_RX_ACCEPTED);
        assert(audio_recovery_rx_next(&rx, now_us, &event));
        assert(event.kind == AUDIO_RX_FRAME && event.sequence == sequence);
    }
    assert(rx.last_arrival_us == 259200000000LL);
    assert(rx.next_sequence == first_sequence + frame_count);
    assert(rx.stats.accepted == 12960000U && rx.stats.output_frames == 12960000U);
    assert(rx.stats.session_count == 1 && rx.stats.abort_count == 0);
    assert(rx.stats.permanent_loss == 0 && rx.stats.aborted_buffered_frames == 0);
    assert(rx.stats.gap_events == 0 && rx.stats.nack_requests == 0);
    assert(rx.stats.recovered == 0 && rx.stats.rejected == 0);
}

static void test_fixed_playout_and_full_repair_budget(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(audio_recovery_rx_playout_us(&rx, 100) == 1232000);
    assert(rx_push(&rx, 17, 102, 1040000, false) == AUDIO_RX_ACCEPTED);
    audio_rx_event_t nack = expect_event(&rx, 1044000, AUDIO_RX_NACK, 101);
    assert(nack.remaining_ms == 200);
    int64_t latest;
    assert(!audio_recovery_rx_refresh_nack(&rx, &nack, 1045000, &latest)); /* already admitted */
    const int64_t deadline = audio_recovery_rx_playout_us(&rx, 101) - DUAL_END_AUDIO_DECODE_MARGIN_US;
    assert(deadline == 1244000);
    assert(rx_push(&rx, 17, 101, deadline - 1, true) == AUDIO_RX_ACCEPTED);
    audio_rx_event_t repaired = expect_event(&rx, deadline - 1, AUDIO_RX_FRAME, 101);
    assert(repaired.playout_us == 1252000);
    expect_event(&rx, deadline - 1, AUDIO_RX_FRAME, 102);
    audio_recovery_rx_set_playout_anchor(&rx, 103, 1292062);
    assert(audio_recovery_rx_playout_us(&rx, 104) == 1312062);
}

static void test_buffered_repair_is_not_recovered_after_decoder_deadline(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 102, 1040000, false) == AUDIO_RX_ACCEPTED);
    (void)expect_event(&rx, 1044000, AUDIO_RX_NACK, 101);
    int64_t deadline = audio_recovery_rx_playout_us(&rx, 101) - DUAL_END_AUDIO_DECODE_MARGIN_US;
    assert(rx_push(&rx, 17, 101, deadline - 1, true) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, deadline, AUDIO_RX_MISSING, 101);
    expect_event(&rx, deadline, AUDIO_RX_FRAME, 102);
    assert(rx.stats.repair_ontime == 1 && rx.stats.recovered == 0);
    assert(rx.stats.expired_buffered_frames == 1 && rx.stats.permanent_loss == 1);
}

static void test_new_missing_head_has_its_own_nack_budget(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 102, 1040000, false) == AUDIO_RX_ACCEPTED);
    assert(expect_event(&rx, 1044000, AUDIO_RX_NACK, 101).missing_mask == 1);
    assert(expect_event(&rx, 1044000 + DUAL_END_AUDIO_NACK_INTERVAL_US, AUDIO_RX_NACK, 101).missing_mask == 1);
    assert(rx_push(&rx, 17, 104, 1104500, false) == AUDIO_RX_ACCEPTED);
    assert(rx_push(&rx, 17, 101, 1105000, true) == AUDIO_RX_ACCEPTED);
    expect_event(&rx, 1105000, AUDIO_RX_FRAME, 101);
    expect_event(&rx, 1105000, AUDIO_RX_FRAME, 102);
    assert(expect_event(&rx, 1108500, AUDIO_RX_NACK, 103).missing_mask == 1);
    assert(rx.stats.nack_requests == 3 && rx.stats.permanent_loss == 0);
}


static void test_later_nacks_retry_after_rolling_quota_returns(void)
{
    audio_recovery_tx_t tx;
    audio_recovery_rx_t rx;
    audio_recovery_tx_reset(&tx, 17);
    begin_with_frame100(&rx);
    cache_store(&tx, 101, 1020000);
    assert(rx_push(&rx, 17, 102, 1040000, false) == AUDIO_RX_ACCEPTED);
    const int64_t first_us = 1044000;
    const int64_t deadline_us = 1244000;
    for (unsigned i = 0; i < DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS; ++i) {
        cache_store(&tx, 200 + i, first_us - 1000);
        audio_recovery_tx_commit(&tx, 200 + i, first_us - 1000);
    }
    assert(tx.budget_used == DUAL_END_AUDIO_REPAIR_BUDGET_PACKETS);
    unsigned actual_repairs = 0;
    for (unsigned round = 0; round < DUAL_END_AUDIO_NACK_MAX_REQUESTS; ++round) {
        const int64_t now = first_us + (int64_t)round * DUAL_END_AUDIO_NACK_INTERVAL_US;
        audio_rx_event_t nack = expect_event(&rx, now, AUDIO_RX_NACK, 101);
        assert(nack.remaining_ms == 200 - round * 60);
        assert(audio_recovery_rx_playout_us(&rx, 101) - DUAL_END_AUDIO_DECODE_MARGIN_US == deadline_us);
        uint32_t token;
        const audio_recovery_tx_entry_t *entry;
        audio_recovery_tx_result_t result = audio_recovery_tx_reserve(
            &tx, 17, 101, now, &token, &entry);
        if (round < 2) {
            assert(result == AUDIO_RECOVERY_TX_BUDGET && token == 0 && entry == NULL);
        } else {
            assert(result == AUDIO_RECOVERY_TX_HIT);
            assert(audio_recovery_tx_prepare(&tx, 17, 101, token, now + 1000));
            assert(audio_recovery_tx_accepted(&tx, 17, 101, token, now + 1000));
            ++actual_repairs;
            if (round == 3) {
                assert(audio_recovery_rx_push(&rx, 17, 101, entry->encoded,
                    -61, true, now + 2000) == AUDIO_RX_ACCEPTED);
            } /* The third-round repair is lost; the fourth can still recover it. */
            assert(audio_recovery_tx_complete(&tx, 17, 101, token));
        }
    }
    assert(actual_repairs == DUAL_END_AUDIO_TX_REPAIR_MAX_ATTEMPTS);
    expect_event(&rx, 1226000, AUDIO_RX_FRAME, 101);
    expect_event(&rx, 1226000, AUDIO_RX_FRAME, 102);
    assert(rx.stats.recovered == 1 && rx.stats.permanent_loss == 0);
    const audio_recovery_tx_entry_t *entry;
    assert(audio_recovery_tx_get(&tx, 17, 101, 1238000, &entry) == AUDIO_RECOVERY_TX_LIMIT);
}

static void test_late_nack_rounds_never_extend_playout_deadline(void)
{
    audio_recovery_rx_t rx;
    begin_with_frame100(&rx);
    assert(rx_push(&rx, 17, 102, 1040000, false) == AUDIO_RX_ACCEPTED);
    const int64_t deadline = 1244000;
    audio_rx_event_t nack = expect_event(&rx, deadline - 70000, AUDIO_RX_NACK, 101);
    assert(nack.remaining_ms == 70);
    /* At the next 60 ms slot only 10 ms remains: do not advertise a round
     * which cannot meet the conservative 12 ms minimum path budget. */
    expect_quiet(&rx, deadline - 10000);
    expect_quiet(&rx, deadline - 1);
    assert(audio_recovery_rx_playout_us(&rx, 101) - DUAL_END_AUDIO_DECODE_MARGIN_US == deadline);
    expect_event(&rx, deadline, AUDIO_RX_MISSING, 101);
    assert(rx_push(&rx, 17, 101, deadline + 1, true) == AUDIO_RX_LATE);
    assert(rx.stats.nack_requests == 1 && rx.stats.permanent_loss == 1 && rx.stats.recovered == 0);
}


static void test_stop_clips_buffered_future_and_pending_nack_across_wrap(void)
{
    const uint32_t bases[] = {100U, UINT32_MAX - 1U};
    for (unsigned i = 0; i < sizeof(bases) / sizeof(bases[0]); ++i) {
        uint32_t base = bases[i];
        audio_recovery_rx_t rx;
        audio_recovery_rx_init(&rx);
        assert(audio_recovery_rx_begin(&rx, 17, base, 1000000) == AUDIO_RX_SESSION_NEW);
        assert(rx_push(&rx, 17, base, 1000000, false) == AUDIO_RX_ACCEPTED);
        expect_event(&rx, 1000000, AUDIO_RX_FRAME, base);
        assert(rx_push(&rx, 17, base + 4U, 1080000, false) == AUDIO_RX_ACCEPTED);
        audio_rx_event_t nack;
        assert(audio_recovery_rx_next(&rx, 1084000, &nack) && nack.kind == AUDIO_RX_NACK);
        assert(nack.sequence == base + 1U && nack.missing_mask == 7);
        audio_recovery_rx_end(&rx, base + 3U, 1085000);
        assert(rx.stats.aborted_buffered_frames == 1);
        assert(!rx.slots[(base + 4U) % DUAL_END_AUDIO_RECOVERY_WINDOW].present);
        int64_t latest;
        assert(audio_recovery_rx_refresh_nack(&rx, &nack, 1085000, &latest));
        assert(nack.missing_mask == 3); /* Never request at/after exclusive END. */
        audio_recovery_rx_finish_nack(&rx, nack.nack_token, AUDIO_RX_NACK_ACCEPTED, 1085000);
        assert(rx_push(&rx, 17, base + 3U, 1086000, true) == AUDIO_RX_REJECTED);
        assert(rx_push(&rx, 17, base + 4U, 1086000, false) == AUDIO_RX_REJECTED);
        assert(rx_push(&rx, 17, base + 1U, 1087000, true) == AUDIO_RX_ACCEPTED);
        assert(rx_push(&rx, 17, base + 2U, 1087000, true) == AUDIO_RX_ACCEPTED);
        expect_event(&rx, 1087000, AUDIO_RX_FRAME, base + 1U);
        expect_event(&rx, 1087000, AUDIO_RX_FRAME, base + 2U);
        expect_event(&rx, 1087000, AUDIO_RX_END, base + 3U);
        expect_quiet(&rx, 1400000);
        assert(rx.stats.output_frames == 3 && rx.stats.permanent_loss == 0);
        assert(!rx.active && !rx.resync_pending);
    }
}

static void test_terminal_end_wins_over_abort_and_new_begin_rearms_recovery(void)
{
    /* The owner validates session identity before calling END. Exercise
     * terminal intent both before and after an unexpected abort. */
    for (unsigned order = 0; order < 4; ++order) {
        audio_recovery_rx_t rx;
        begin_with_frame100(&rx);
        assert(rx_push(&rx, 17, 102, 1040000, false) == AUDIO_RX_ACCEPTED);
        audio_rx_event_t pending;
        assert(audio_recovery_rx_next(&rx, 1044000, &pending));
        assert(pending.kind == AUDIO_RX_NACK && rx.nack_pending);
        if (order == 0) {
            audio_recovery_rx_end(&rx, 103, 1045000);
            assert(rx.active && rx.ending && rx.end_sequence == 103);
            assert(rx.slots[102 % DUAL_END_AUDIO_RECOVERY_WINDOW].present);
            audio_recovery_rx_abort(&rx, 1046000);
        } else {
            audio_recovery_rx_abort(&rx, 1045000);
            assert(!rx.active && rx.resync_pending);
            if (order == 2)
                expect_event(&rx, 1045000, AUDIO_RX_RESYNC, 101);
            if (order == 3) audio_recovery_rx_stop(&rx);
            else audio_recovery_rx_end(&rx, 103, 1046000);
        }
        assert(!rx.active && !rx.ending && !rx.resync_pending);
        assert(!rx.gap_active && !rx.nack_pending && !rx.playout_anchored);
        assert(rx.stats.abort_count == 1 && rx.stats.aborted_buffered_frames == 1);
        const uint32_t resyncs = rx.stats.resync_requests;
        expect_quiet(&rx, 1046000);
        expect_quiet(&rx, 2046000);
        /* A late admission for the abandoned NACK cannot revive recovery. */
        audio_recovery_rx_finish_nack(&rx, pending.nack_token,
            AUDIO_RX_NACK_ACCEPTED, 1047000);
        audio_recovery_rx_abort(&rx, 2046001);
        expect_quiet(&rx, 3046000);
        assert(rx.stats.resync_requests == resyncs && rx.stats.abort_count == 1);
        assert(audio_recovery_rx_begin(&rx, 18, 200, 3047000) == AUDIO_RX_SESSION_NEW);
        assert(rx.active && !rx.ending && !rx.resync_pending);
        assert(rx_push(&rx, 18, 200, 3047100, false) == AUDIO_RX_ACCEPTED);
        expect_event(&rx, 3047100, AUDIO_RX_FRAME, 200);
        /* Ordinary unexpected aborts still request and retry recovery after
         * a valid new BEGIN; terminal intent never leaks into the new flow. */
        audio_recovery_rx_abort(&rx, 3048000);
        expect_event(&rx, 3048000, AUDIO_RX_RESYNC, 201);
        expect_quiet(&rx, 3048000 + DUAL_END_AUDIO_SESSION_RETRY_US - 1);
        expect_event(&rx, 3048000 + DUAL_END_AUDIO_SESSION_RETRY_US,
            AUDIO_RX_RESYNC, 201);
        assert(rx.stats.abort_count == 2 && rx.stats.resync_requests == resyncs + 2);
    }
}

int main(void)
{
    RUN(test_terminal_end_wins_over_abort_and_new_begin_rearms_recovery);
    RUN(test_stop_clips_buffered_future_and_pending_nack_across_wrap);
    RUN(test_fixed_playout_and_full_repair_budget);
    RUN(test_buffered_repair_is_not_recovered_after_decoder_deadline);
    RUN(test_new_missing_head_has_its_own_nack_budget);
    RUN(test_later_nacks_retry_after_rolling_quota_returns);
    RUN(test_late_nack_rounds_never_extend_playout_deadline);
    RUN(test_wire_layout_and_reservation);
    RUN(test_dual_mic_repair_pack_excludes_metadata);
    RUN(test_control_header_validation);
    RUN(test_sequence_wrap_order);
    RUN(test_tx_original_bytes_are_copied);
    RUN(test_tx_session_and_cache_miss);
    RUN(test_tx_deadline_boundaries);
    RUN(test_tx_extended_deadline_preserves_original_repair_bytes);
    RUN(test_tx_ring_replacement_and_wrap);
    RUN(test_tx_duplicate_nack_and_repair_limit);
    RUN(test_tx_global_repair_budget);
    RUN(test_rx_requires_valid_session);
    RUN(test_rx_lifetime_counters_account_discarded_buffered_frames);
    RUN(test_rx_72_hour_simulated_stream_with_sequence_wrap);
    RUN(test_rx_nack_admission_does_not_spend_queued_rounds);
    RUN(test_rx_pending_mask_refresh_and_session_invalidation);
    RUN(test_rx_in_order_has_no_artificial_hold);
    RUN(test_rx_duplicates_before_and_after_output);
    RUN(test_rx_hid_priority_arrival_order_is_repaired);
    RUN(test_rx_single_loss_recovered_once);
    RUN(test_rx_lost_nack_is_retried_but_bounded);
    RUN(test_rx_deadline_and_late_repair_never_replay);
    RUN(test_rx_last_instant_repair_is_useful);
    RUN(test_rx_consecutive_gaps_have_independent_deadlines);
    RUN(test_rx_new_packets_do_not_extend_deadline);
    RUN(test_rx_partial_repair_reduces_nack_mask);
    RUN(test_rx_no_last_moment_nack);
    RUN(test_rx_sequence_wrap_reordering);
    RUN(test_rx_duplicate_session_is_idempotent);
    RUN(test_rx_retired_session_and_old_repairs_are_rejected);
    RUN(test_rx_idle_and_duplicates_do_not_keep_stream_alive);
    RUN(test_rx_far_jump_resync_is_bounded);
    RUN(test_rx_four_missing_frames_preserve_playback_session);
    RUN(test_rx_cold_start_request_is_bounded_and_stops_on_begin);
    RUN(test_rx_packets_queued_before_session_cannot_enter_new_decoder);
    RUN(test_end_to_end_loss_and_duplicate_nack);
    RUN(test_extended_gap_recovers_after_previous_deadline);
    printf("audio recovery host tests passed: %u scenarios\n", tests_run);
    return 0;
}
