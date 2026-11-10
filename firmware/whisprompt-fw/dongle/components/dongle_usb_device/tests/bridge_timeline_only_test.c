/* Compile the real bridge as the production Dongle component does: no
 * compatibility PCM ring, only the absolute-timeline sink. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "uac_test_boundary.h"

TickType_t uac_test_tick;

#ifdef _MSC_VER
#pragma warning(push)
/* ESP32 size_t is 32 bits; the bounded host-only input checks exercise the
 * production timeline on x64, where these diagnostic counter increments are
 * wider. Keep the target source unchanged. */
#pragma warning(disable: 4267)
#endif
#include "../../dongle_uac_bridge/dongle_uac_bridge.c"
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#if !defined(DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING) || \
    DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING != 0
#error "production host test must compile with DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING=0"
#endif
_Static_assert(DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING == 0,
               "production host test must disable the legacy ring");
_Static_assert(DONGLE_UAC_BRIDGE_LEGACY_RING_STATIC_BYTES == 0,
               "the 32 KiB legacy PCM ring must not be compiled in");

static void assert_silence(const int16_t *pcm, size_t samples)
{
    for (size_t i = 0; i < samples * AUDIO_CHANNELS; ++i) {
        assert(pcm[i] == 0);
    }
}

static void assert_stereo_value(const int16_t *pcm, size_t samples, int16_t value)
{
    for (size_t i = 0; i < samples; ++i) {
        assert(pcm[i * AUDIO_CHANNELS] == value);
        assert(pcm[i * AUDIO_CHANNELS + 1] == (int16_t)-value);
    }
}

int main(void)
{
    uac_test_tick = 1000;
    uac_test_generation = 0;
    assert(dongle_uac_bridge_start() == ESP_OK);

    int16_t pcm[DUAL_END_AUDIO_FRAME_SAMPLES * AUDIO_CHANNELS];
    for (size_t i = 0; i < DUAL_END_AUDIO_FRAME_SAMPLES; ++i) {
        pcm[i * AUDIO_CHANNELS] = 700;
        pcm[i * AUDIO_CHANNELS + 1] = -700;
    }
    int16_t out[16 * AUDIO_CHANNELS];

    /* A TinyUSB read before the first timed frame is safe and silent. */
    assert(dongle_uac_bridge_read_pcm((uint8_t *)out, sizeof(out)) == sizeof(out));
    assert_silence(out, 16);

    /* Untimed input cannot silently reactivate or allocate the old ring. */
    assert(dongle_uac_bridge_write_pcm(pcm, DUAL_END_AUDIO_FRAME_SAMPLES,
                                       100, -50, NULL) == ESP_ERR_NOT_SUPPORTED);

    const int64_t playout_us = 1000000LL + DUAL_END_AUDIO_PLAYOUT_DELAY_US;
    assert(dongle_uac_bridge_write_timed_pcm(pcm, DUAL_END_AUDIO_FRAME_SAMPLES,
                                             100, -50, playout_us, NULL) == ESP_OK);
    assert(dongle_uac_bridge_read_pcm((uint8_t *)out, sizeof(out)) == sizeof(out));
    assert_silence(out, 16);

    uac_test_tick = (TickType_t)(playout_us / 1000LL);
    assert(dongle_uac_bridge_read_pcm((uint8_t *)out, sizeof(out)) == sizeof(out));
    assert_stereo_value(out, 16, 700);
    uint32_t sequence = 0;
    int64_t clock_us = 0;
    assert(dongle_uac_bridge_playout_clock(&sequence, &clock_us, NULL));
    assert(sequence == 100 && clock_us == playout_us);

    dongle_uac_bridge_end_session();
    assert(!dongle_uac_bridge_session_drained());
    uac_test_tick = (TickType_t)((playout_us + DUAL_END_AUDIO_FRAME_MS * 1000LL) / 1000LL);
    assert(dongle_uac_bridge_session_drained());

    puts("audio UAC timeline-only production test passed (legacy ring=0)");
    return 0;
}
