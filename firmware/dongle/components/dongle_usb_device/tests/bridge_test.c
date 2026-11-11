/* Execute the real PCM ring, startup, drift and session code. Only time,
 * critical-section primitives, error constants and logging are stubbed. */
#include <assert.h>
#include <stdio.h>

#include "uac_test_boundary.h"
TickType_t uac_test_tick;
static void (*uac_after_unlock)(void);
static void uac_interleaved_exit(portMUX_TYPE *lock)
{
    uac_test_exit(lock);
    if (lock->depth == 0 && uac_after_unlock) {
        void (*callback)(void) = uac_after_unlock;
        uac_after_unlock = NULL;
        callback();
    }
}
#undef portEXIT_CRITICAL
#define portEXIT_CRITICAL(lock) uac_interleaved_exit(lock)

/* ESP32 size_t is 32 bits; the x64 host warns about the unchanged uint32_t
 * diagnostic counters receiving size_t increments. Test inputs are bounded
 * to the real 32 KiB ring. Adapt this compiler-only width warning locally. */
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4267)
#endif
#include "../../dongle_uac_bridge/dongle_uac_bridge.c"
#ifdef _MSC_VER
#pragma warning(pop)
#endif

/* The legacy host suite intentionally retains the 32 KiB compatibility
 * allocation. The production-specific companion suite asserts zero. */
_Static_assert(DONGLE_UAC_BRIDGE_LEGACY_RING_STATIC_BYTES == (32 * 1024),
               "legacy bridge regression must exercise the original 32 KiB ring");

static void begin_test(void)
{
    uac_test_tick = 1000;
    uac_test_generation = 0;
    assert(dongle_uac_bridge_start() == ESP_OK);
    assert(s_audio_lock.depth == 0U);
}

static void push_frame(uint32_t sequence, int16_t value)
{
    int16_t pcm[320 * AUDIO_CHANNELS];
    for (size_t i = 0; i < 320U; ++i) {
        pcm[i * AUDIO_CHANNELS] = value;
        pcm[i * AUDIO_CHANNELS + 1] = (int16_t)-value;
    }
    assert(dongle_uac_bridge_write_pcm(pcm, 320, sequence, -50, NULL) == ESP_OK);
    assert(s_audio_lock.depth == 0U);
}

static void push_nine(uint32_t first_sequence, int16_t value)
{
    for (uint32_t i = 0; i < 9U; ++i) {
        push_frame(first_sequence + i, value);
        uac_test_tick += 20;
    }
}

static void assert_samples(const int16_t *pcm, size_t samples, int16_t value)
{
    for (size_t i = 0; i < samples; ++i) {
        assert(pcm[i * AUDIO_CHANNELS] == value);
        assert(pcm[i * AUDIO_CHANNELS + 1] == (int16_t)-value);
    }
}

static void assert_fade_in(const int16_t *pcm, size_t samples, int16_t value)
{
    for (size_t i = 0; i < samples; ++i) {
        int16_t expected = samples > 1 ?
            (int16_t)(((int32_t)value * (int32_t)i) / (int32_t)(samples - 1)) : 0;
        assert(pcm[i * AUDIO_CHANNELS] == expected);
        assert(pcm[i * AUDIO_CHANNELS + 1] == (int16_t)-expected);
    }
}

static void read_usb(int16_t *pcm, size_t samples)
{
    assert(dongle_uac_bridge_read_pcm((uint8_t *)pcm, samples * sizeof(*pcm) * AUDIO_CHANNELS) ==
           samples * sizeof(*pcm) * AUDIO_CHANNELS);
    assert(s_audio_lock.depth == 0U);
}

static void test_start_threshold_and_first_read(void)
{
    begin_test();
    for (uint32_t i = 0; i < 8U; ++i) {
        push_frame(100U + i, (int16_t)(100U + i));
        uac_test_tick += 20;
    }
    dongle_uac_bridge_stats_t stats;
    dongle_uac_bridge_get_stats(&stats);
    assert(stats.start_pending && !stats.rx_active);
    /* The double-microphone public diagnostic keeps its historical unit:
     * interleaved int16_t values, or two values per stereo sample frame. */
    assert(stats.buffer_samples == 2560U * AUDIO_CHANNELS); /* 160 ms is still prebuffering. */
    int16_t out[16 * AUDIO_CHANNELS];
    read_usb(out, 16);
    assert_samples(out, 16, 0);
    assert(audio_buffer_available() == 2560U);

    push_frame(108, 108);
    dongle_uac_bridge_get_stats(&stats);
    assert(!stats.start_pending && stats.rx_active);
    assert(stats.buffer_samples == 2880U * AUDIO_CHANNELS); /* Nine complete 20 ms frames. */
    read_usb(out, 16);
    /* Every PTT utterance retains its first samples, including prebuffer. */
    assert_fade_in(out, 16, 100);
    assert(s_level_drop_samples == 0U);
    assert(audio_buffer_available() == 2864U);
}

static void test_sixty_millisecond_repair_gap_has_reserve(void)
{
    begin_test();
    push_nine(1000, 700);
    int16_t out[16 * AUDIO_CHANNELS];
    read_usb(out, 16);
    uint32_t silence_before = s_uac_silence_fills;
    uint32_t from_buffer_before = s_uac_samples_from_buffer;
    for (unsigned i = 0; i < 60U; ++i) {
        ++uac_test_tick;
        uint32_t sources_before_read = s_uac_samples_from_buffer;
        read_usb(out, 16);
        assert_samples(out, 16, 700);
        assert(s_uac_samples_from_buffer > sources_before_read);
    }
    assert(s_uac_silence_fills == silence_before);
    assert(s_uac_samples_from_buffer > from_buffer_before);
    assert(s_rx_active);
    assert(audio_buffer_available() >= 16U * 50U);
    /* The recovery deadline emits a missing-frame placeholder, followed by
     * queued real successors. Buffered zeros still count as real ring data. */
    push_frame(1009, 0);
    push_frame(1010, 702);
    push_frame(1011, 703);
    assert(s_seq_lost == 0U && s_seq_reorder == 0U);
    assert(audio_buffer_available() >= 16U * 110U);
    bool heard_placeholder = false;
    bool heard_successor = false;
    for (unsigned i = 0; i < 160U; ++i) {
        ++uac_test_tick;
        uint32_t sources_before_read = s_uac_samples_from_buffer;
        read_usb(out, 16);
        assert(s_uac_samples_from_buffer > sources_before_read);
        for (size_t j = 0; j < 16U; ++j) {
            heard_placeholder = heard_placeholder || out[j * AUDIO_CHANNELS] == 0;
            heard_successor = heard_successor || out[j * AUDIO_CHANNELS] == 702;
        }
    }
    assert(heard_placeholder && heard_successor);
    assert(s_uac_silence_fills == silence_before);
    assert(s_seq_lost == 0U && s_seq_reorder == 0U);
}

static void test_timed_missing_placeholder_preserves_order(void)
{
    begin_test();
    for (uint32_t i = 0; i < 9U; ++i) {
        /* Recovery emits an equal-length silent frame for missing seq 504. */
        push_frame(500U + i, i == 4U ? 0 : (int16_t)(500U + i));
        uac_test_tick += 20;
    }
    int16_t out[320 * AUDIO_CHANNELS];
    for (uint32_t i = 0; i < 6U; ++i) {
        read_usb(out, 320);
        if (i == 0) assert_fade_in(out, 320, 500);
        else assert_samples(out, 320, i == 4U ? 0 : (int16_t)(500U + i));
        uac_test_tick += 20;
    }
    assert(s_seq_lost == 0U);
    assert(s_uac_silence_fills == 0U); /* Buffered silence is not an underrun. */
}

static void test_sequence_monotonicity_and_wrap(void)
{
    begin_test();
    push_frame(10, 10);
    push_frame(12, 12);
    push_frame(11, 11);
    push_frame(12, 12);
    push_frame(13, 13);
    assert(s_seq_lost == 1U);
    assert(s_seq_reorder == 2U);
    assert(s_last_seq == 13U);
    assert(s_frames_in == 3U);
    assert(audio_buffer_available() == 960U);
    assert(s_audio_buffer[0][0] == 10);
    assert(s_audio_buffer[320][0] == 12);
    assert(s_audio_buffer[640][0] == 13);

    dongle_uac_bridge_reset_session();
    uint32_t loss_before = s_seq_lost;
    push_frame(UINT32_MAX, 20);
    push_frame(0, 21);
    push_frame(1, 22);
    assert(s_last_seq == 1U && s_seq_lost == loss_before);
}

static void test_session_reset_and_stale_reader_generation(void)
{
    begin_test();
    push_nine(100, 700);
    uint32_t retired_generation = s_session_generation;
    dongle_uac_bridge_reset_session();
    assert(s_session_generation != retired_generation);
    assert(audio_buffer_available() == 0U);
    assert(!s_rx_active && s_start_pending && !s_seq_valid);
    int16_t out[16 * AUDIO_CHANNELS];
    read_usb(out, 16);
    assert_samples(out, 16, 0);
    push_nine(123456, 900);
    assert(s_rx_active && s_seq_lost == 0U);
    assert(audio_buffer_available() == 2880U);

    /* Simulate a USB read which planned its trim before a session reset. */
    audio_buffer_trim_to(1920, retired_generation);
    assert(audio_buffer_available() == 2880U);
    bool silence = false;
    size_t from_buffer = 99;
    assert(audio_buffer_read_silence(out, 16, retired_generation, &silence,
                                    &from_buffer) == 16U);
    assert(silence && from_buffer == 0U);
    assert_samples(out, 16, 0);
    assert(audio_buffer_available() == 2880U);
    read_usb(out, 16);
    assert_fade_in(out, 16, 900);
    assert(audio_buffer_available() == 2864U);
}

static void test_overflow_remains_bounded_and_keeps_recent_audio(void)
{
    begin_test();
    for (uint32_t i = 0; i < 60U; ++i) {
        push_frame(i, 1234);
    }
    assert(audio_buffer_available() == 8191U);
    assert(s_overflow_drop_samples == 60U * 320U - 8191U);
    assert(s_read_idx < 8192U && s_write_idx < 8192U);
    /* A paused USB consumer still trims stale backlog when it resumes. */
    s_last_uac_read_tick = uac_test_tick - AUDIO_UAC_SESSION_RESTART_MS;
    int16_t out[16 * AUDIO_CHANNELS];
    read_usb(out, 16);
    assert_fade_in(out, 16, 1234);
    assert(audio_buffer_available() == 1904U);
    assert(s_uac_silence_fills == 0U);
}

static void prepare_level(size_t samples)
{
    begin_test();
    int16_t pcm[5000 * AUDIO_CHANNELS];
    assert(samples <= 5000U);
    for (size_t i = 0; i < samples; ++i) {
        pcm[i * AUDIO_CHANNELS] = 333;
        pcm[i * AUDIO_CHANNELS + 1] = -333;
    }
    assert(audio_buffer_write(pcm, samples) == samples);
    s_rx_active = true;
    s_fade_in_pending = false; /* Isolate steady-state clock correction. */
}

static void read_level(unsigned reads)
{
    for (unsigned i = 0; i < reads; ++i) {
        int16_t out[16 * AUDIO_CHANNELS];
        bool silence = false;
        size_t from_buffer = 0;
        assert(audio_buffer_read_silence(out, 16, s_session_generation,
                                        &silence, &from_buffer) == 16U);
        assert(!silence && from_buffer > 0U);
        assert_samples(out, 16, 333);
        assert(s_audio_lock.depth == 0U);
    }
}

static void test_revised_water_levels(void)
{
    /* Low-water support begins below 70 ms, high-water trimming above 200 ms. */
    prepare_level(16U * 69U);
    read_level(5);
    assert(s_level_dup_samples == 1U && s_level_drop_samples == 0U);
    assert(audio_buffer_available() == 16U * 69U - 79U);

    prepare_level(16U * 75U);
    read_level(5);
    assert(s_level_dup_samples == 0U && s_level_drop_samples == 0U);

    prepare_level(16U * 180U);
    read_level(5);
    assert(s_level_drop_samples == 0U && s_level_dup_samples == 0U);

    prepare_level(16U * 200U + 100U);
    read_level(5);
    assert(s_level_drop_samples == 1U && s_level_dup_samples == 0U);

    prepare_level(16U * 275U);
    read_level(5);
    assert(s_level_drop_samples == 1U); /* No critical correction below 290 ms. */

    prepare_level(16U * 290U + 100U);
    read_level(1);
    assert(s_level_drop_samples == 4U);
}

static void test_partial_stereo_request_does_not_consume_or_swap_channels(void)
{
    begin_test();
    push_nine(100, 1234);
    size_t before = audio_buffer_available();
    uint8_t partial[7];
    memset(partial, 0x55, sizeof(partial));
    assert(dongle_uac_bridge_read_pcm(partial, sizeof(partial)) == 0);
    assert(audio_buffer_available() == before);
    for (size_t i = 0; i < sizeof(partial); i++) assert(partial[i] == 0);
    /* Real 10 ms USB input callback: 160 stereo frames = 640 bytes. */
    int16_t stereo[160 * AUDIO_CHANNELS];
    assert(dongle_uac_bridge_read_pcm((uint8_t *)stereo, sizeof(stereo)) == 640);
    assert_fade_in(stereo, 160, 1234);
    assert(audio_buffer_available() == 2880U - 160U);
}

static void test_short_ptt_tail_drains_without_threshold_or_slew(void)
{
    begin_test();
    push_frame(100, 400);
    assert(!s_rx_active && s_start_pending);
    dongle_uac_bridge_end_session();
    assert(s_rx_active && !s_start_pending);
    assert(!dongle_uac_bridge_session_drained());
    int16_t out[16 * AUDIO_CHANNELS];
    for (unsigned i = 0; i < 20U; ++i) {
        ++uac_test_tick;
        read_usb(out, 16);
        if (i == 0) assert_fade_in(out, 16, 400);
        else assert_samples(out, 16, 400);
    }
    assert(dongle_uac_bridge_session_drained());
    assert(s_level_drop_samples == 0 && s_level_dup_samples == 0);
    read_usb(out, 16);
    assert_samples(out, 16, 0);
    dongle_uac_bridge_reset_session();
    assert(!dongle_uac_bridge_session_drained() && !s_draining);
    push_frame(200, 700);
    dongle_uac_bridge_end_session();
    read_usb(out, 16);
    assert_fade_in(out, 16, 700);
}

static void test_new_ptt_cancels_old_drain_and_rejects_late_pcm(void)
{
    begin_test();
    push_frame(100, 400);
    dongle_uac_bridge_end_session();
    int16_t late[320 * AUDIO_CHANNELS] = {0};
    assert(dongle_uac_bridge_write_pcm(late, 320, 101, -50, NULL) == ESP_ERR_INVALID_STATE);
    dongle_uac_bridge_reset_session();
    push_frame(200, 900);
    dongle_uac_bridge_end_session();
    int16_t out[16 * AUDIO_CHANNELS];
    read_usb(out, 16);
    assert_fade_in(out, 16, 900);
}

static int16_t ramp_value(size_t frame, size_t channel)
{
    return channel == 0 ? (int16_t)((int32_t)frame * 80 - 8000) :
                          (int16_t)((int32_t)frame * 40 + 1500);
}

static void verify_resampled_level(size_t available, size_t consumed, bool wrap)
{
    prepare_level(available);
    size_t read_base = wrap ? AUDIO_RING_BUFFER_SAMPLES - 80 : 0;
    s_read_idx = read_base;
    s_write_idx = (read_base + available) % AUDIO_RING_BUFFER_SAMPLES;
    for (size_t i = 0; i < 170; ++i) {
        for (size_t channel = 0; channel < AUDIO_CHANNELS; ++channel)
            s_audio_buffer[(read_base + i) % AUDIO_RING_BUFFER_SAMPLES][channel] = ramp_value(i, channel);
    }
    s_high_slew_phase = 4;
    s_low_slew_phase = 4;
    int16_t out[160 * AUDIO_CHANNELS];
    read_usb(out, 160);
    for (size_t i = 0; i < 160; ++i) {
        size_t position = i * (consumed - 1);
        size_t first = position / 159;
        size_t fraction = position % 159;
        size_t second = first + 1 < consumed ? first + 1 : first;
        for (size_t channel = 0; channel < AUDIO_CHANNELS; ++channel) {
            int64_t expected = ((int64_t)ramp_value(first, channel) * (int64_t)(159 - fraction) +
                                (int64_t)ramp_value(second, channel) * (int64_t)fraction) / 159;
            assert(out[i * AUDIO_CHANNELS + channel] == expected);
        }
        /* No skipped prefix, mid-block duplicate or single large correction
         * edge. Both independent ramps use the same fractional source time. */
        if (i != 0) {
            int delta = out[i * AUDIO_CHANNELS] - out[(i - 1) * AUDIO_CHANNELS];
            assert(delta > 0 && delta <= 84);
        }
    }
    assert(out[0] == ramp_value(0, 0));
    assert(out[159 * AUDIO_CHANNELS] == ramp_value(consumed - 1, 0));
    assert(audio_buffer_available() == available - consumed);
    assert(s_uac_samples_from_buffer == consumed);
    assert(s_level_drop_samples == (consumed > 160 ? consumed - 160 : 0));
    assert(s_level_dup_samples == (consumed < 160 ? 160 - consumed : 0));
}

static void test_whole_block_stereo_interpolation_and_ring_wrap(void)
{
    verify_resampled_level(16U * 290U + 100U, 164, false);
    verify_resampled_level(16U * 290U + 100U, 164, true);
    verify_resampled_level(16U * 200U + 100U, 161, false);
    verify_resampled_level(16U * 69U, 159, true);
    verify_resampled_level(16U * 120U, 160, false);

    prepare_level(16U * 290U + 100U);
    int16_t one[AUDIO_CHANNELS];
    read_usb(one, 1);
    assert_samples(one, 1, 333);
    assert(s_level_drop_samples == 0 && s_level_dup_samples == 0);
}

static void test_partial_underflow_fades_and_recovery_fades_in(void)
{
    prepare_level(8);
    for (size_t i = 0; i < 8; ++i) {
        s_audio_buffer[i][0] = 8000;
        s_audio_buffer[i][1] = -12000;
    }
    int16_t out[160 * AUDIO_CHANNELS];
    read_usb(out, 160);
    assert(s_uac_samples_from_buffer == 8 && s_uac_silence_fills == 1);
    for (size_t i = 0; i < 160; ++i) {
        for (size_t channel = 0; channel < AUDIO_CHANNELS; ++channel) {
            int32_t value = channel == 0 ? 8000 : -12000;
            int16_t expected = i < 8 ? (int16_t)value :
                (int16_t)((value * (int32_t)(159 - i)) / 152);
            assert(out[i * AUDIO_CHANNELS + channel] == expected);
        }
    }
    assert(out[159 * AUDIO_CHANNELS] == 0 && out[159 * AUDIO_CHANNELS + 1] == 0);
    assert(s_last_sample[0] == 0 && s_last_sample[1] == 0 && s_fade_in_pending);
    read_usb(out, 160);
    assert_samples(out, 160, 0); /* No repeated stale tail on later empty reads. */
    assert(s_fade_in_pending);
    int16_t recovered[320 * AUDIO_CHANNELS];
    for (size_t i = 0; i < 320; ++i) {
        recovered[i * AUDIO_CHANNELS] = 9000;
        recovered[i * AUDIO_CHANNELS + 1] = -14000;
    }
    assert(audio_buffer_write(recovered, 320) == 320);
    read_usb(out, 160);
    for (size_t i = 0; i < 160; ++i) {
        assert(out[i * AUDIO_CHANNELS] == (int16_t)(9000 * (int32_t)i / 159));
        assert(out[i * AUDIO_CHANNELS + 1] == (int16_t)(-14000 * (int32_t)i / 159));
    }
    assert(!s_fade_in_pending && s_last_sample[0] == 9000 && s_last_sample[1] == -14000);
}

static void test_empty_underflow_and_new_session_do_not_replay_history(void)
{
    prepare_level(0);
    s_last_sample[0] = -12345;
    s_last_sample[1] = 23456;
    int16_t out[160 * AUDIO_CHANNELS];
    read_usb(out, 160);
    for (size_t i = 0; i < 160; ++i) {
        assert(out[i * AUDIO_CHANNELS] == (int16_t)(-12345 * (int32_t)(159 - i) / 160));
        assert(out[i * AUDIO_CHANNELS + 1] == (int16_t)(23456 * (int32_t)(159 - i) / 160));
    }
    assert(s_uac_samples_from_buffer == 0 && s_fade_in_pending);
    dongle_uac_bridge_reset_session();
    read_usb(out, 160);
    assert_samples(out, 160, 0);
    assert(s_last_sample[0] == 0 && s_last_sample[1] == 0 && s_fade_in_pending);
}

static void test_stereo_finite_tail_is_not_resampled_or_extended(void)
{
    begin_test();
    int16_t tail[320 * AUDIO_CHANNELS];
    for (size_t i = 0; i < 320; ++i) {
        tail[i * AUDIO_CHANNELS] = (int16_t)(4000 + i * 10);
        tail[i * AUDIO_CHANNELS + 1] = (int16_t)(-10000 + (int32_t)i * 5);
    }
    assert(dongle_uac_bridge_write_pcm(tail, 320, 600, -50, NULL) == ESP_OK);
    dongle_uac_bridge_end_session();
    int16_t out[160 * AUDIO_CHANNELS];
    read_usb(out, 160);
    for (size_t i = 0; i < 160; ++i) {
        for (size_t channel = 0; channel < AUDIO_CHANNELS; ++channel) {
            int16_t expected = (int16_t)(((int32_t)tail[i * AUDIO_CHANNELS + channel] *
                                           (int32_t)i) / 159);
            assert(out[i * AUDIO_CHANNELS + channel] == expected);
        }
    }
    assert(!dongle_uac_bridge_session_drained());
    read_usb(out, 160);
    assert(memcmp(out, tail + 160 * AUDIO_CHANNELS, sizeof(out)) == 0);
    assert(dongle_uac_bridge_session_drained());
    assert(s_uac_samples_from_buffer == 320 && s_level_drop_samples == 0 && s_level_dup_samples == 0);
    read_usb(out, 160);
    assert_samples(out, 160, 0);
}

static void test_end_session_between_timeout_snapshot_and_commit_keeps_tail(void)
{
    begin_test();
    push_nine(700, 1500);
    s_last_audio_packet_tick = uac_test_tick - AUDIO_RX_TIMEOUT_MS;
    /* The USB reader observes a stale packet before the RX worker commits
     * END. Its second lock must recheck draining rather than erase the tail. */
    uac_after_unlock = dongle_uac_bridge_end_session;
    int16_t out[160 * AUDIO_CHANNELS];
    read_usb(out, 160);
    assert(s_draining && s_rx_active && !s_start_pending);
    assert(s_timeouts == 0 && audio_buffer_available() == 2880U - 160U);
    assert_fade_in(out, 160, 1500);
}

static void arm_end_after_trim_snapshot(void)
{
    uac_after_unlock = dongle_uac_bridge_end_session;
}

static void test_end_session_between_trim_snapshot_and_commit_keeps_tail(void)
{
    begin_test();
    push_nine(800, 1700);
    s_last_uac_read_tick = uac_test_tick - AUDIO_UAC_SESSION_RESTART_MS;
    /* First unlock is the timeout snapshot; second is the trim decision.
     * END arrives after trim_buffer=true but before the helper commits it. */
    uac_after_unlock = arm_end_after_trim_snapshot;
    int16_t out[160 * AUDIO_CHANNELS];
    read_usb(out, 160);
    assert(s_draining && s_rx_active && !s_start_pending);
    assert(s_level_drop_samples == 0 && s_timeouts == 0);
    assert(audio_buffer_available() == 2880U - 160U);
    assert_fade_in(out, 160, 1700);
}


static void refill_after_timeout_commit(void)
{
    push_nine(200, 2300);
}


static void arm_refill_after_timeout_snapshot(void)
{
    uac_after_unlock = refill_after_timeout_commit;
}


static void test_timeout_reader_cannot_consume_newly_refilled_audio(void)
{
    begin_test();
    push_nine(100, 1000);
    uint32_t retired = s_session_generation;
    s_last_audio_packet_tick = uac_test_tick - AUDIO_RX_TIMEOUT_MS;
    /* First unlock ends the timeout snapshot; second commits timeout. The RX
     * task immediately refills before this old USB producer resumes reading. */
    uac_after_unlock = arm_refill_after_timeout_snapshot;
    int16_t out[160 * AUDIO_CHANNELS];
    read_usb(out, 160);
    assert(s_session_generation == retired + 1 && s_timeouts == 1);
    assert_samples(out, 160, 0);
    assert(audio_buffer_available() == 2880 && s_uac_samples_from_buffer == 0);
    /* The next producer retains the normal continuous-stream first-read trim
     * policy, and alone consumes the first restored 10 ms block. */
    read_usb(out, 160);
    assert_fade_in(out, 160, 2300);
    assert(audio_buffer_available() == 2880U - 160U && s_uac_samples_from_buffer == 160);
}


static void push_timed(uint32_t seq, int16_t value, int64_t at) {
    int16_t pcm[320 * AUDIO_CHANNELS];
    for (size_t i = 0; i < 320; ++i) { pcm[i * 2] = value; pcm[i * 2 + 1] = -value; }
    assert(dongle_uac_bridge_write_timed_pcm(pcm, 320, seq, -50, at, NULL) == ESP_OK);
}

static void test_timeline_starts_at_absolute_deadline_with_one_frame(void) {
    begin_test(); int64_t at = 1000000LL + DUAL_END_AUDIO_PLAYOUT_DELAY_US;
    push_timed(100, 1234, at); int16_t out[32];
    uint32_t seq; int64_t time;
    assert(!dongle_uac_bridge_playout_clock(&seq, &time, NULL));
    for (int64_t t = 1000000; t < at; t += 1000) {
        timeline_read_pcm((uint8_t *)out, sizeof(out), t);
        assert_samples(out, 16, 0); assert(s_timeline_cursor == 0);
    }
    timeline_read_pcm((uint8_t *)out, sizeof(out), at);
    assert_samples(out, 16, 1234); assert(s_timeline_first_output_us == at);
    assert(dongle_uac_bridge_playout_clock(&seq, &time, NULL));
    assert(seq == 100 && time == at);
}

static void test_timeline_fractional_start_quantizes_only_to_one_sample(void) {
    begin_test(); int64_t at = 1232501; push_timed(100, 1234, at); int16_t out[32];
    timeline_read_pcm((uint8_t *)out, sizeof(out), 1232000);
    assert_samples(out, 9, 0); assert_samples(out + 18, 7, 1234);
    assert(s_timeline_first_output_us >= at && s_timeline_first_output_us - at < 63);
}

static void test_timeline_missing_time_is_not_replayed_by_late_pcm(void) {
    begin_test(); int64_t at = 1232000; push_timed(100, 100, at);
    push_timed(102, 102, at + 40000); int16_t out[32];
    for (int i = 0; i < 50; ++i) {
        timeline_read_pcm((uint8_t *)out, sizeof(out), at + i * 1000LL);
        assert_samples(out, 16, i < 20 ? 100 : i < 40 ? 0 : 102);
    }
    assert(s_timeline_cursor == 800);
    push_timed(101, 101, at + 20000);
    assert(s_timeline_late_samples == 320);
    timeline_read_pcm((uint8_t *)out, sizeof(out), at + 50000);
    assert_samples(out, 16, 102); assert(s_timeline_cursor == 816);
}

static void test_timeline_slot_collision_drops_only_incoming_frame(void) {
    begin_test();
    const int64_t target = 1000000LL + DUAL_END_AUDIO_PLAYOUT_DELAY_US;
    const uint32_t generation = s_session_generation;
    const uint32_t frames_in = s_frames_in;
    push_timed(100, 1111, target);

    int16_t newer_pcm[320 * AUDIO_CHANNELS];
    for (size_t i = 0; i < 320U; ++i) {
        newer_pcm[i * AUDIO_CHANNELS] = 2222;
        newer_pcm[i * AUDIO_CHANNELS + 1] = -2222;
    }
    /* Sequence + AUDIO_TIMELINE_SLOTS aliases the occupied ring slot while
     * frame 100 is still scheduled. The bridge reports bounded backpressure
     * without resetting the audio session or replacing the earlier frame. */
    assert(dongle_uac_bridge_write_timed_pcm(newer_pcm, 320,
        100U + AUDIO_TIMELINE_SLOTS, -50,
        target + (int64_t)AUDIO_TIMELINE_SLOTS * 20000LL, NULL) == ESP_ERR_NO_MEM);
    assert(s_session_generation == generation && s_frames_in == frames_in + 1U);
    assert(s_overflow_drop_samples == 320U && s_timeline_first_sequence == 100U);

    int16_t out[320 * AUDIO_CHANNELS];
    timeline_read_pcm((uint8_t *)out, sizeof(out), target);
    assert_samples(out, 320, 1111);
    assert(s_session_generation == generation && s_timeline_cursor == 320U);
}

static void run_timeline_clock_model(int ppm, int phase_us, unsigned duration_ms) {
    begin_test();
    const int64_t received = 1000000LL;
    const int64_t target = received + DUAL_END_AUDIO_PLAYOUT_DELAY_US;
    push_timed(100, 100, target);
    uint32_t next_frame = 1;
    uint32_t old_adjust = 0;
    int64_t max_clock_error_us = 0;
    int64_t previous_adjust_us = 0;
    int16_t out[32];
    for (unsigned ms = 0; ms < duration_ms; ++ms) {
        int64_t now = received + phase_us + (int64_t)ms * 1000;
        uac_test_tick = (TickType_t)(now / 1000);
        while (received + (int64_t)next_frame * 20000LL * 1000000LL / (1000000 + ppm) <= now) {
            push_timed(100 + next_frame, (int16_t)(100 + next_frame), target + next_frame * 20000LL);
            ++next_frame;
        }
        timeline_read_pcm((uint8_t *)out, sizeof(out), now);
        uint32_t adjust = s_level_drop_samples + s_level_dup_samples;
        assert(adjust - old_adjust <= 1);
        if (adjust != old_adjust) {
            assert(!previous_adjust_us || now - previous_adjust_us >= 100000);
            previous_adjust_us = now;
        }
        old_adjust = adjust;
        if (s_timeline_started) {
            uint32_t source_seq; int64_t source_time;
            assert(dongle_uac_bridge_playout_clock(&source_seq, &source_time, NULL));
            int64_t ideal = target + (int64_t)(uint32_t)(source_seq - 100) * 20000LL * 1000000LL / (1000000 + ppm);
            int64_t error = source_time - ideal;
            if (error < 0) error = -error;
            if (error > max_clock_error_us) max_clock_error_us = error;
            assert(error < 3000);
        }
        if (s_timeline_started && !ppm) {
            uint32_t seq; int64_t time;
            assert(dongle_uac_bridge_playout_clock(&seq, &time, NULL));
            int64_t exported_start = time - (int64_t)(uint32_t)(seq - 100) * 20000;
            assert(exported_start >= target && exported_start - target < 63);
            assert(adjust == 0);
        }
    }
    assert(s_overflow_drop_samples == 0 && s_timeline_late_samples == 0);
    assert(s_timeline_cursor > 0);
    if (ppm > 0) {
        assert(s_level_dup_samples == 0 && s_level_drop_samples >= 100 && s_level_drop_samples <= 240);
    } else if (ppm < 0) {
        assert(s_level_drop_samples == 0 && s_level_dup_samples >= 100 && s_level_dup_samples <= 240);
    }
    printf("timeline model ppm=%d phase_us=%d duration_ms=%u drop=%u dup=%u max_clock_error_us=%lld\n",
        ppm, phase_us, duration_ms, s_level_drop_samples, s_level_dup_samples, (long long)max_clock_error_us);
    /* USB cursor and its timestamp remain one consistent exported clock. */
    uint32_t seq; int64_t time;
    assert(dongle_uac_bridge_playout_clock(&seq, &time, NULL));
    assert(seq == 100 + (uint32_t)(s_timeline_cursor / 320));
    assert(time == s_timeline_next_output_us - (s_timeline_cursor % 320) * 1000000LL / 16000);
    dongle_uac_bridge_reset_session();
    assert(!dongle_uac_bridge_playout_clock(&seq, &time, NULL));
}

static void test_timeline_drift_is_bounded_and_exported_as_same_clock(void) {
    const int phases[] = {0, 31, 500, 999};
    for (size_t i = 0; i < sizeof(phases) / sizeof(phases[0]); ++i)
        run_timeline_clock_model(0, phases[i], 60000);
    run_timeline_clock_model(100, 317, 120000);
    run_timeline_clock_model(-100, 617, 120000);
}

static void test_unopened_usb_retires_local_pcm_without_reset_or_overflow(void) {
    begin_test(); const int64_t target = 1000000LL + DUAL_END_AUDIO_PLAYOUT_DELAY_US;
    uint32_t generation = s_session_generation;
    for (uint32_t frame = 0; frame <= 1000; ++frame) {
        uac_test_tick = 1000 + frame * 20;
        push_timed(100 + frame, (int16_t)(100 + frame), target + frame * 20000LL);
        assert(s_session_generation == generation && s_timeline_cursor == 0);
        assert(s_overflow_drop_samples == 0 && s_timeline_late_samples == 0);
        dongle_uac_bridge_stats_t stats;
        dongle_uac_bridge_get_stats(&stats);
        assert(stats.buffer_samples <= 16 * 320 * AUDIO_CHANNELS);
    }
    assert(s_timeline_local_retired_samples > 300000 && !s_timeline_started);
    int64_t now = (int64_t)uac_test_tick * 1000;
    int64_t current = (now - target) * 16000 / 1000000;
    int16_t out[32]; ++uac_test_generation;
    timeline_read_pcm((uint8_t *)out, sizeof(out), now);
    assert_samples(out, 16, (int16_t)(100 + current / 320));
    assert(s_timeline_cursor == current + 16);
    assert(s_timeline_resume_skipped_samples == (uint64_t)current);
    assert(s_session_generation == generation && s_overflow_drop_samples == 0);
    uint32_t seq; int64_t time; assert(dongle_uac_bridge_playout_clock(&seq, &time, NULL));
    assert(time - (int64_t)(seq - 100) * 20000 == target);
}

static void test_closed_usb_and_long_in_pause_resume_current_source(void) {
    begin_test(); const int64_t target = 1000000LL + DUAL_END_AUDIO_PLAYOUT_DELAY_US;
    uint32_t generation = s_session_generation, next_frame = 0;
    int16_t out[32];
    for (uint32_t ms = 0; ms < 2100; ++ms) {
        int64_t now = 1000000LL + ms * 1000LL; uac_test_tick = 1000 + ms;
        while (next_frame * 20 <= ms) {
            push_timed(100 + next_frame, (int16_t)(100 + next_frame), target + next_frame * 20000LL);
            ++next_frame;
        }
        if (ms >= 500 && ms < 2000) continue; /* PC closes UAC for 1.5 seconds. */
        if (ms == 2000) ++uac_test_generation;
        timeline_read_pcm((uint8_t *)out, sizeof(out), now);
        if (ms == 2000) {
            int64_t current = (now - target) * 16000 / 1000000;
            assert_samples(out, 16, (int16_t)(100 + current / 320));
            assert(s_timeline_cursor == current + 16);
        }
    }
    assert(s_timeline_pause_count == 1 && s_timeline_max_gap_us == 1501000);
    assert(s_timeline_resume_skipped_samples == 24000);
    assert(s_timeline_local_retired_samples > 20000);
    /* An open endpoint's missing 62ms of IN completions also skips elapsed
     * source positions, without assigning them to the next packet's time. */
    int64_t resume = 3162000;
    while (1000000LL + next_frame * 20000LL <= resume) {
        uac_test_tick = 1000 + next_frame * 20;
        push_timed(100 + next_frame, (int16_t)(100 + next_frame), target + next_frame * 20000LL);
        ++next_frame;
    }
    timeline_read_pcm((uint8_t *)out, sizeof(out), resume);
    int64_t current = (resume - target) * 16000 / 1000000;
    assert_samples(out, 16, (int16_t)(100 + current / 320));
    assert(s_timeline_cursor == current + 16 && s_timeline_pause_count == 2);
    assert(s_timeline_resume_skipped_samples == 24992);
    assert(s_session_generation == generation && s_overflow_drop_samples == 0);
    assert(s_level_drop_samples == 0 && s_level_dup_samples == 0);
}


static void test_timed_ptt_tail_keeps_absolute_start_and_exact_source_length(void)
{
    begin_test();
    const int64_t at = 1000000LL + DUAL_END_AUDIO_PLAYOUT_DELAY_US;
    push_timed(100, 1234, at);
    dongle_uac_bridge_end_session();
    assert(s_draining && !dongle_uac_bridge_session_drained());
    int16_t out[16 * AUDIO_CHANNELS];
    timeline_read_pcm((uint8_t *)out, sizeof(out), at - 1000);
    assert_samples(out, 16, 0);
    assert(s_timeline_cursor == 0 && !s_timeline_started);
    /* A nearly due clock-correction window must not shorten/extend finite PTT. */
    s_timeline_drift_window_samples = AUDIO_TIMELINE_DRIFT_WINDOW_SAMPLES - 16;
    s_timeline_drift_lead_sum = 0;
    for (unsigned i = 0; i < 20; ++i) {
        uac_test_tick = (TickType_t)(at / 1000 + i);
        assert(!dongle_uac_bridge_session_drained());
        timeline_read_pcm((uint8_t *)out, sizeof(out), at + (int64_t)i * 1000);
        assert_samples(out, 16, 1234);
    }
    assert(s_timeline_cursor == 320 && dongle_uac_bridge_session_drained());
    assert(s_uac_samples_from_buffer == 320);
    assert(s_level_drop_samples == 0 && s_level_dup_samples == 0);
    timeline_read_pcm((uint8_t *)out, sizeof(out), at + 20000);
    assert_samples(out, 16, 0);
    assert(s_uac_samples_from_buffer == 320);
}

static void test_timed_ptt_without_usb_reader_retires_only_after_tail_time(void)
{
    begin_test();
    const int64_t at = 1000000LL + DUAL_END_AUDIO_PLAYOUT_DELAY_US;
    push_timed(100, 700, at);
    dongle_uac_bridge_end_session();
    uac_test_tick = (TickType_t)(at / 1000 + 19);
    assert(!dongle_uac_bridge_session_drained());
    uac_test_tick++;
    assert(dongle_uac_bridge_session_drained());
    assert(s_uac_samples_from_buffer == 0); /* Retirement is not USB completion. */
    dongle_uac_bridge_reset_session();
    assert(!dongle_uac_bridge_session_drained() && !s_draining);
    push_timed(200, 900, 1500000);
    dongle_uac_bridge_end_session();
    int16_t out[16 * AUDIO_CHANNELS];
    timeline_read_pcm((uint8_t *)out, sizeof(out), 1500000);
    assert_samples(out, 16, 900);
}

int main(void)
{
    test_timed_ptt_tail_keeps_absolute_start_and_exact_source_length();
    test_timed_ptt_without_usb_reader_retires_only_after_tail_time();
    test_timeout_reader_cannot_consume_newly_refilled_audio();
    test_timeline_starts_at_absolute_deadline_with_one_frame();
    test_timeline_fractional_start_quantizes_only_to_one_sample();
    test_timeline_missing_time_is_not_replayed_by_late_pcm();
    test_timeline_slot_collision_drops_only_incoming_frame();
    test_timeline_drift_is_bounded_and_exported_as_same_clock();
    test_unopened_usb_retires_local_pcm_without_reset_or_overflow();
    test_closed_usb_and_long_in_pause_resume_current_source();

    test_partial_stereo_request_does_not_consume_or_swap_channels();
    test_start_threshold_and_first_read();
    test_sixty_millisecond_repair_gap_has_reserve();
    test_timed_missing_placeholder_preserves_order();
    test_sequence_monotonicity_and_wrap();
    test_session_reset_and_stale_reader_generation();
    test_overflow_remains_bounded_and_keeps_recent_audio();
    test_revised_water_levels();
    test_short_ptt_tail_drains_without_threshold_or_slew();
    test_new_ptt_cancels_old_drain_and_rejects_late_pcm();
    test_whole_block_stereo_interpolation_and_ring_wrap();
    test_partial_underflow_fades_and_recovery_fades_in();
    test_empty_underflow_and_new_session_do_not_replay_history();
    test_stereo_finite_tail_is_not_resampled_or_extended();
    test_end_session_between_timeout_snapshot_and_commit_keeps_tail();
    test_end_session_between_trim_snapshot_and_commit_keeps_tail();
    puts("audio UAC buffer tests passed (25 scenarios)");
    return 0;
}
