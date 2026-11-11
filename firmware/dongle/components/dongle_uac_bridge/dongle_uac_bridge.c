#include "dongle_uac_bridge.h"

#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "dual_end_audio_recovery_protocol.h"
#include "usb_device_uac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dual_end_link_config.h"

static const char *TAG = "dongle_uac_bridge";

#define AUDIO_CHANNELS DUAL_END_AUDIO_CHANNELS
#define AUDIO_SAMPLE_FRAME_BYTES (AUDIO_CHANNELS * sizeof(int16_t))
_Static_assert(AUDIO_CHANNELS == 2, "UAC channel order requires exactly two streams");
#define AUDIO_MS_TO_SAMPLES(ms) ((DUAL_END_AUDIO_SAMPLE_RATE_HZ * (ms)) / 1000)
#ifndef DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
/* Keep the existing host bridge regression as the default compatibility
 * configuration. The production Dongle component explicitly selects 0 in
 * CMake and only accepts the absolute-timeline sink. */
#define DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING 1
#endif
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING != 0 && \
    DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING != 1
#error "DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING must be 0 or 1"
#endif

#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
#define AUDIO_RING_BUFFER_BYTES (32 * 1024)
/* Ring indices and all level/counter units are sample frames (one L/R pair),
 * retaining the existing millisecond policy and diagnostic interpretation. */
#define AUDIO_RING_BUFFER_SAMPLES (AUDIO_RING_BUFFER_BYTES / AUDIO_SAMPLE_FRAME_BYTES)
_Static_assert(AUDIO_RING_BUFFER_BYTES % AUDIO_SAMPLE_FRAME_BYTES == 0,
               "ring must contain complete stereo sample frames");
/* Keep more decoded audio available while the recovery window waits for a
 * missing frame. Storage capacity is unchanged; these are playout watermarks. */
#define AUDIO_START_THRESHOLD_PACKETS 9
#define AUDIO_START_THRESHOLD_SAMPLES (AUDIO_START_THRESHOLD_PACKETS * DUAL_END_AUDIO_FRAME_SAMPLES)
#define AUDIO_LEVEL_LOW_SAMPLES AUDIO_MS_TO_SAMPLES(70)
#define AUDIO_LEVEL_TARGET_SAMPLES AUDIO_MS_TO_SAMPLES(120)
#define AUDIO_LEVEL_HIGH_SAMPLES AUDIO_MS_TO_SAMPLES(200)
#define AUDIO_LEVEL_CRITICAL_SAMPLES AUDIO_MS_TO_SAMPLES(290)
#define AUDIO_LEVEL_MAX_SLEW_SAMPLES 2
#define AUDIO_LEVEL_CRITICAL_SLEW_SAMPLES 4
#define AUDIO_RX_TIMEOUT_MS 300
#define AUDIO_UAC_SESSION_RESTART_MS 50

/* This macro is intentionally visible to the host production configuration
 * test. It proves the 32 KiB legacy allocation is absent before a target map
 * file is available. */
#define DONGLE_UAC_BRIDGE_LEGACY_RING_STATIC_BYTES AUDIO_RING_BUFFER_BYTES

static int16_t s_audio_buffer[AUDIO_RING_BUFFER_SAMPLES][AUDIO_CHANNELS];
static volatile size_t s_write_idx;
static volatile size_t s_read_idx;
#else
#define DONGLE_UAC_BRIDGE_LEGACY_RING_STATIC_BYTES 0U
#endif

static portMUX_TYPE s_audio_lock = portMUX_INITIALIZER_UNLOCKED;

static bool s_bridge_ready;
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
static volatile bool s_buffer_active;
#endif
static volatile bool s_rx_active;
static volatile bool s_start_pending;
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
static volatile TickType_t s_last_audio_packet_tick;
static TickType_t s_last_uac_read_tick;
static bool s_timeout_logged;
static bool s_seq_valid;
static uint32_t s_last_seq;
#endif
static uint32_t s_session_generation;
static bool s_draining;

/* Decoded frames retain their source positions. The USB reader owns the
 * consumption clock; empty slots consume time rather than postponing audio. */
/* 32 slots = 640 ms of playout ring (was 16 = 320 ms). The count must stay a
 * power of two: timeline_sample_unlocked() indexes it with a per-sample
 * modulo on the USB callback path, so a non-power-of-two would become a real
 * division at 16 kHz. */
#define AUDIO_TIMELINE_SLOTS 32U
#define AUDIO_TIMELINE_DRIFT_WINDOW_SAMPLES AUDIO_MS_TO_SAMPLES(100)
#define AUDIO_TIMELINE_DRIFT_TOLERANCE_SAMPLES AUDIO_MS_TO_SAMPLES(2)
typedef struct {
    bool present;
    int64_t frame_index;
    int16_t pcm[DUAL_END_AUDIO_FRAME_SAMPLES * AUDIO_CHANNELS];
} audio_timeline_slot_t;
static audio_timeline_slot_t s_timeline_slots[AUDIO_TIMELINE_SLOTS];
static bool s_timeline_enabled;
static bool s_timeline_started;
static uint32_t s_timeline_first_sequence;
static int64_t s_timeline_start_us;
static int64_t s_timeline_cursor;
static int64_t s_timeline_tail;
static int64_t s_timeline_next_output_us;
static uint32_t s_timeline_drift_window_samples;
static int64_t s_timeline_drift_lead_sum;
static int64_t s_timeline_first_output_us;
static uint64_t s_timeline_late_samples;
static uint64_t s_timeline_local_retired_samples;
static uint64_t s_timeline_resume_skipped_samples;
static uint32_t s_timeline_usb_generation;
static bool s_timeline_usb_generation_valid;
static int64_t s_timeline_last_read_us;
static uint32_t s_timeline_pause_count;
static uint32_t s_timeline_max_gap_us;

#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
static int16_t s_last_sample[AUDIO_CHANNELS];
static bool s_fade_in_pending;
static uint8_t s_high_slew_phase;
static uint8_t s_low_slew_phase;
#endif

static uint32_t s_frames_in;
static uint32_t s_samples_in;
static uint32_t s_seq_lost;
static uint32_t s_seq_reorder;
static uint32_t s_timeouts;
static uint32_t s_uac_cb_count;
static uint32_t s_uac_samples_requested;
static uint32_t s_uac_samples_output;
static uint32_t s_uac_samples_from_buffer;
static uint32_t s_uac_silence_fills;
static uint32_t s_level_drop_samples;
static uint32_t s_level_dup_samples;
static uint32_t s_overflow_drop_samples;
static uint32_t s_rssi_count;
static int32_t s_rssi_sum;
static int s_rssi_min = 127;
static int s_rssi_max = -127;

static void timeline_reset_unlocked(void)
{
    memset(s_timeline_slots, 0, sizeof(s_timeline_slots));
    s_timeline_enabled = false;
    s_timeline_started = false;
    s_timeline_first_sequence = 0;
    s_timeline_start_us = 0;
    s_timeline_cursor = 0;
    s_timeline_tail = 0;
    s_timeline_next_output_us = 0;
    s_timeline_drift_window_samples = 0;
    s_timeline_drift_lead_sum = 0;
    s_timeline_first_output_us = 0;
    s_timeline_late_samples = 0;
    s_timeline_local_retired_samples = 0;
    s_timeline_resume_skipped_samples = 0;
    s_timeline_usb_generation = 0;
    s_timeline_usb_generation_valid = false;
    s_timeline_last_read_us = 0;
    s_timeline_pause_count = 0;
    s_timeline_max_gap_us = 0;
}

/* The timeline keeps advancing even when the PC has not opened UAC.
 * Retiring local PCM does not mean a wireless frame was lost or rejected. */
static int64_t timeline_sample_time_unlocked(int64_t position)
{
    int64_t base_position = s_timeline_started ? s_timeline_cursor : 0;
    int64_t base_us = s_timeline_started ? s_timeline_next_output_us : s_timeline_start_us;
    return base_us + (position - base_position) * 1000000LL / DUAL_END_AUDIO_SAMPLE_RATE_HZ;
}
static int64_t timeline_position_at_unlocked(int64_t now_us)
{
    int64_t position = s_timeline_started ? s_timeline_cursor : 0;
    int64_t time_us = s_timeline_started ? s_timeline_next_output_us : s_timeline_start_us;
    if (now_us > time_us) position += (now_us - time_us) * DUAL_END_AUDIO_SAMPLE_RATE_HZ / 1000000LL;
    return position;
}
static void timeline_retire_expired_unlocked(int64_t now_us)
{
    for (size_t i = 0; i < AUDIO_TIMELINE_SLOTS; ++i) {
        audio_timeline_slot_t *slot = &s_timeline_slots[i];
        if (!slot->present) continue;
        int64_t end = (slot->frame_index + 1) * DUAL_END_AUDIO_FRAME_SAMPLES;
        if (timeline_sample_time_unlocked(end) <= now_us) {
            int64_t begin = slot->frame_index * DUAL_END_AUDIO_FRAME_SAMPLES;
            if (begin < s_timeline_cursor) begin = s_timeline_cursor;
            if (end > begin) s_timeline_local_retired_samples += (uint64_t)(end - begin);
            slot->present = false;
        }
    }
}
static uint32_t timeline_buffered_samples_unlocked(int64_t now_us)
{
    int64_t current = timeline_position_at_unlocked(now_us);
    uint32_t samples = 0;
    for (size_t i = 0; i < AUDIO_TIMELINE_SLOTS; ++i) {
        const audio_timeline_slot_t *slot = &s_timeline_slots[i];
        if (!slot->present) continue;
        int64_t end = (slot->frame_index + 1) * DUAL_END_AUDIO_FRAME_SAMPLES;
        int64_t begin = slot->frame_index * DUAL_END_AUDIO_FRAME_SAMPLES;
        if (begin < current) begin = current;
        if (end > begin) samples += (uint32_t)(end - begin);
    }
    return samples;
}

static bool timeline_sample_unlocked(int64_t position, int16_t sample[AUDIO_CHANNELS])
{
    const int64_t frame = position / DUAL_END_AUDIO_FRAME_SAMPLES;
    const size_t offset = (size_t)(position % DUAL_END_AUDIO_FRAME_SAMPLES);
    audio_timeline_slot_t *slot = &s_timeline_slots[(uint64_t)frame % AUDIO_TIMELINE_SLOTS];
    if (!slot->present || slot->frame_index != frame) {
        return false;
    }
    memcpy(sample, &slot->pcm[offset * AUDIO_CHANNELS], AUDIO_SAMPLE_FRAME_BYTES);
    return true;
}

bool dongle_uac_bridge_playout_clock(uint32_t *sequence, int64_t *playout_us, void *ctx)
{
    (void)ctx;
    if (!sequence || !playout_us) return false;
    portENTER_CRITICAL(&s_audio_lock);
    bool valid = s_timeline_enabled && s_timeline_started;
    if (valid) {
        *sequence = s_timeline_first_sequence +
            (uint32_t)(s_timeline_cursor / DUAL_END_AUDIO_FRAME_SAMPLES);
        *playout_us = s_timeline_next_output_us -
            (s_timeline_cursor % DUAL_END_AUDIO_FRAME_SAMPLES) * 1000000LL /
                DUAL_END_AUDIO_SAMPLE_RATE_HZ;
    }
    portEXIT_CRITICAL(&s_audio_lock);
    return valid;
}

/* Identity-rate reads need no per-sample interpolation or 64-bit division.
 * Copy whole in-slot spans, retaining holes and the exact finite drain tail. */
static uint32_t timeline_copy_unlocked(int16_t *out, int64_t position,
                                       size_t samples, bool *filled)
{
    uint32_t real_samples = 0;
    while (samples) {
        if (s_draining && position >= s_timeline_tail) {
            memset(out, 0, samples * AUDIO_SAMPLE_FRAME_BYTES);
            break;
        }
        const int64_t frame = position / DUAL_END_AUDIO_FRAME_SAMPLES;
        const size_t offset = (size_t)(position % DUAL_END_AUDIO_FRAME_SAMPLES);
        size_t chunk = DUAL_END_AUDIO_FRAME_SAMPLES - offset;
        if (chunk > samples) chunk = samples;
        if (s_draining && (int64_t)chunk > s_timeline_tail - position)
            chunk = (size_t)(s_timeline_tail - position);
        const audio_timeline_slot_t *slot =
            &s_timeline_slots[(uint64_t)frame % AUDIO_TIMELINE_SLOTS];
        if (slot->present && slot->frame_index == frame) {
            memcpy(out, &slot->pcm[offset * AUDIO_CHANNELS], chunk * AUDIO_SAMPLE_FRAME_BYTES);
            real_samples += (uint32_t)chunk;
        } else {
            memset(out, 0, chunk * AUDIO_SAMPLE_FRAME_BYTES);
            *filled = true;
        }
        out += chunk * AUDIO_CHANNELS;
        position += (int64_t)chunk;
        samples -= chunk;
    }
    return real_samples;
}

static size_t timeline_read_pcm(uint8_t *buf, size_t len, int64_t now_us)
{
    const size_t requested = len / AUDIO_SAMPLE_FRAME_BYTES;
    int16_t *out = (int16_t *)buf;
    size_t prefix = 0;
    bool filled = false;
    uac_device_stats_t usb = {0};
    uac_device_get_stats(&usb); /* No UAC lock is held while taking the PCM lock. */
    portENTER_CRITICAL(&s_audio_lock);
    if (!s_timeline_enabled) {
        memset(buf, 0, len);
        portEXIT_CRITICAL(&s_audio_lock);
        return len;
    }
    bool reopened = s_timeline_usb_generation_valid &&
        s_timeline_usb_generation != usb.mic_generation;
    s_timeline_usb_generation = usb.mic_generation;
    s_timeline_usb_generation_valid = true;
    int64_t gap_us = s_timeline_last_read_us ? now_us - s_timeline_last_read_us : 0;
    if (gap_us > 0 && (uint64_t)gap_us > s_timeline_max_gap_us)
        s_timeline_max_gap_us = gap_us > UINT32_MAX ? UINT32_MAX : (uint32_t)gap_us;
    if (s_timeline_started && gap_us > 2000) ++s_timeline_pause_count;
    s_timeline_last_read_us = now_us;
    if (s_timeline_started && (reopened || now_us - s_timeline_next_output_us >= 2000)) {
        int64_t current = timeline_position_at_unlocked(now_us);
        int64_t target = timeline_sample_time_unlocked(current);
        if (current > s_timeline_cursor)
            s_timeline_resume_skipped_samples += (uint64_t)(current - s_timeline_cursor);
        s_timeline_cursor = current;
        s_timeline_next_output_us = target;
        s_timeline_drift_window_samples = 0;
        s_timeline_drift_lead_sum = 0;
        if (reopened) uac_device_mark_first_audio_sample(0, target);
    }
    s_uac_cb_count++;
    s_uac_samples_requested += requested;
    if (!s_timeline_started) {
        int64_t wait_us = s_timeline_start_us - now_us;
        if (wait_us > 0) {
            uint64_t wait_samples = ((uint64_t)wait_us * DUAL_END_AUDIO_SAMPLE_RATE_HZ + 999999U) / 1000000U;
            prefix = wait_samples < requested ? (size_t)wait_samples : requested;
            memset(out, 0, prefix * AUDIO_SAMPLE_FRAME_BYTES);
            filled = prefix != 0;
        } else if (wait_us < 0) {
            /* Starting the PC capture late cannot replay stale prebuffer. */
            s_timeline_cursor = -wait_us * DUAL_END_AUDIO_SAMPLE_RATE_HZ / 1000000LL;
            s_timeline_resume_skipped_samples += (uint64_t)s_timeline_cursor;
        }
        if (prefix < requested) {
            s_timeline_started = true;
            s_rx_active = true;
            s_start_pending = false;
            s_timeline_first_output_us = now_us + (int64_t)prefix * 1000000LL /
                DUAL_END_AUDIO_SAMPLE_RATE_HZ;
            uac_device_mark_first_audio_sample(prefix, s_timeline_start_us +
                s_timeline_cursor * 1000000LL / DUAL_END_AUDIO_SAMPLE_RATE_HZ);
        }
    }
    size_t output_samples = requested - prefix;
    int64_t consume = (int64_t)output_samples;
    /* A 20 ms radio frame changes tail in 320-sample steps: with a 232 ms
     * delay, undrifted instantaneous lead spans about 232..252 ms. Average a
     * full 100 ms of USB consumption, centred on delay + half a radio frame.
     * Always retire the window, even with no correction, so phase peaks do
     * not eventually trigger a false drop. Two ms absorbs packet/SOF phase.
     * One sample per window is the maximum slew (625 ppm at 16 kHz). */
    if (output_samples > 0U && s_timeline_started && !s_draining) {
        int64_t lead = s_timeline_tail - s_timeline_cursor;
        s_timeline_drift_lead_sum += lead * (int64_t)output_samples;
        s_timeline_drift_window_samples += (uint32_t)output_samples;
        if (s_timeline_drift_window_samples >= AUDIO_TIMELINE_DRIFT_WINDOW_SAMPLES) {
            int64_t average_lead = s_timeline_drift_lead_sum / s_timeline_drift_window_samples;
            int64_t target_lead = AUDIO_MS_TO_SAMPLES(
                DUAL_END_AUDIO_PLAYOUT_DELAY_US / 1000LL + DUAL_END_AUDIO_FRAME_MS / 2);
            if (output_samples > 1U && average_lead >
                target_lead + AUDIO_TIMELINE_DRIFT_TOLERANCE_SAMPLES) {
                ++consume;
                ++s_level_drop_samples;
            } else if (output_samples > 1U && lead > 1 && average_lead <
                target_lead - AUDIO_TIMELINE_DRIFT_TOLERANCE_SAMPLES) {
                --consume;
                ++s_level_dup_samples;
            }
            s_timeline_drift_lead_sum = 0;
            s_timeline_drift_window_samples = 0;
        }
    }
    if (consume == (int64_t)output_samples) {
        s_uac_samples_from_buffer += timeline_copy_unlocked(
            &out[prefix * AUDIO_CHANNELS], s_timeline_cursor, output_samples, &filled);
    } else {
        for (size_t i = 0; i < output_samples; ++i) {
            int64_t numerator = (int64_t)i * consume;
            int64_t offset = numerator / (int64_t)output_samples;
            int64_t fraction = numerator % (int64_t)output_samples;
            int16_t a[AUDIO_CHANNELS], b[AUDIO_CHANNELS];
            bool tail_done = s_draining && s_timeline_cursor + offset >= s_timeline_tail;
            bool real = !tail_done && timeline_sample_unlocked(s_timeline_cursor + offset, a);
            if (real) {
                if (!fraction || !timeline_sample_unlocked(s_timeline_cursor + offset + 1, b)) {
                    memcpy(b, a, sizeof(b));
                }
                for (size_t ch = 0; ch < AUDIO_CHANNELS; ++ch) {
                    int64_t value = (int64_t)a[ch] * ((int64_t)output_samples - fraction) +
                                    (int64_t)b[ch] * fraction;
                    out[(prefix + i) * AUDIO_CHANNELS + ch] = (int16_t)(value / (int64_t)output_samples);
                }
                ++s_uac_samples_from_buffer;
            } else {
                memset(&out[(prefix + i) * AUDIO_CHANNELS], 0, AUDIO_SAMPLE_FRAME_BYTES);
                if (!tail_done) filled = true;
            }
        }
    }
    s_timeline_cursor += consume;
    if (s_draining && s_timeline_cursor >= s_timeline_tail) s_rx_active = false;
    if (s_timeline_started) {
        s_timeline_next_output_us = now_us + (int64_t)requested * 1000000LL / DUAL_END_AUDIO_SAMPLE_RATE_HZ;
        int64_t oldest_frame = s_timeline_cursor / DUAL_END_AUDIO_FRAME_SAMPLES;
        for (size_t i = 0; i < AUDIO_TIMELINE_SLOTS; ++i) {
            if (s_timeline_slots[i].present && s_timeline_slots[i].frame_index < oldest_frame) {
                s_timeline_slots[i].present = false;
            }
        }
    }
    s_uac_samples_output += requested;
    if (filled) ++s_uac_silence_fills;
    portEXIT_CRITICAL(&s_audio_lock);
    return len;
}

#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
static size_t audio_buffer_available_unlocked(void)
{
    return (s_write_idx + AUDIO_RING_BUFFER_SAMPLES - s_read_idx) % AUDIO_RING_BUFFER_SAMPLES;
}

static size_t audio_buffer_available(void)
{
    portENTER_CRITICAL(&s_audio_lock);
    size_t avail = audio_buffer_available_unlocked();
    portEXIT_CRITICAL(&s_audio_lock);
    return avail;
}

static void audio_buffer_reset(void)
{
    portENTER_CRITICAL(&s_audio_lock);
    s_write_idx = 0;
    s_read_idx = 0;
    memset(s_last_sample, 0, sizeof(s_last_sample));
    s_fade_in_pending = true;
    s_high_slew_phase = 0;
    s_low_slew_phase = 0;
    portEXIT_CRITICAL(&s_audio_lock);
}

static void audio_buffer_trim_to(size_t target_samples, uint32_t generation)
{
    portENTER_CRITICAL(&s_audio_lock);
    size_t avail = audio_buffer_available_unlocked();
    if (generation == s_session_generation && !s_draining && avail > target_samples) {
        size_t drop = avail - target_samples;
        s_read_idx = (s_read_idx + drop) % AUDIO_RING_BUFFER_SAMPLES;
        s_level_drop_samples += drop;
        s_fade_in_pending = true;
    }
    portEXIT_CRITICAL(&s_audio_lock);
}

static size_t audio_buffer_write(const int16_t *src, size_t samples)
{
    if (!src || samples == 0) {
        return 0;
    }

    portENTER_CRITICAL(&s_audio_lock);
    size_t written = 0;
    for (size_t i = 0; i < samples; i++) {
        memcpy(s_audio_buffer[s_write_idx], src + i * AUDIO_CHANNELS, AUDIO_SAMPLE_FRAME_BYTES);
        s_write_idx = (s_write_idx + 1) % AUDIO_RING_BUFFER_SAMPLES;
        if (s_write_idx == s_read_idx) {
            s_read_idx = (s_read_idx + 1) % AUDIO_RING_BUFFER_SAMPLES;
            s_overflow_drop_samples++;
            s_fade_in_pending = true;
        }
        written++;
    }
    portEXIT_CRITICAL(&s_audio_lock);
    return written;
}

/* One common envelope and source position for both channels preserves their
 * time alignment and relative gain. The caller owns s_audio_lock. */
static void audio_fade_in_unlocked(int16_t *dst, size_t samples)
{
    if (!s_fade_in_pending) return;
    for (size_t i = 0; i < samples; ++i) {
        for (size_t channel = 0; channel < AUDIO_CHANNELS; ++channel) {
            dst[i * AUDIO_CHANNELS + channel] = samples > 1 ?
                (int16_t)(((int64_t)dst[i * AUDIO_CHANNELS + channel] * (int64_t)i) /
                          (int64_t)(samples - 1)) : 0;
        }
    }
    s_fade_in_pending = false;
}

static size_t audio_buffer_read_silence(int16_t *dst,
                                        size_t max_samples,
                                        uint32_t generation,
                                        bool *silence_filled,
                                        size_t *samples_from_buffer)
{
    if (silence_filled) {
        *silence_filled = false;
    }
    if (samples_from_buffer) {
        *samples_from_buffer = 0;
    }
    if (!dst || max_samples == 0) {
        return 0;
    }

    portENTER_CRITICAL(&s_audio_lock);

    if (generation != s_session_generation || !s_rx_active) {
        memset(dst, 0, max_samples * AUDIO_SAMPLE_FRAME_BYTES);
        if (silence_filled) {
            *silence_filled = true;
        }
        portEXIT_CRITICAL(&s_audio_lock);
        return max_samples;
    }

    size_t avail = audio_buffer_available_unlocked();
    size_t samples_to_drop = 0;
    size_t samples_to_stretch = 0;

    if (s_draining) {
        /* Finite PTT tail is consumed once without level correction. */
    } else if (avail > AUDIO_LEVEL_CRITICAL_SAMPLES) {
        samples_to_drop = AUDIO_LEVEL_CRITICAL_SLEW_SAMPLES;
    } else if (avail > AUDIO_LEVEL_HIGH_SAMPLES) {
        if (++s_high_slew_phase >= 5) {
            s_high_slew_phase = 0;
            samples_to_drop = 1;
        }
    } else if (avail < AUDIO_LEVEL_LOW_SAMPLES && avail > AUDIO_LEVEL_MAX_SLEW_SAMPLES) {
        if (++s_low_slew_phase >= 5) {
            s_low_slew_phase = 0;
            samples_to_stretch = 1;
        }
    }

    /* Spread clock correction over the entire output block instead of
     * deleting a prefix or repeating a sample at one audible discontinuity. */
    if (max_samples == 1) {
        samples_to_drop = 0;
        samples_to_stretch = 0;
    }
    size_t samples_to_consume = max_samples;
    if (samples_to_drop > 0 && avail >= max_samples &&
        avail - max_samples >= samples_to_drop) {
        samples_to_consume += samples_to_drop;
    } else {
        samples_to_drop = 0;
        if (samples_to_stretch > 0 && samples_to_stretch < max_samples &&
            avail >= max_samples - samples_to_stretch) {
            samples_to_consume -= samples_to_stretch;
        } else {
            samples_to_stretch = 0;
        }
    }

    if (avail < samples_to_consume) {
        if (silence_filled) {
            *silence_filled = true;
        }
        for (size_t i = 0; i < avail; i++) {
            memcpy(dst + i * AUDIO_CHANNELS, s_audio_buffer[s_read_idx], AUDIO_SAMPLE_FRAME_BYTES);
            memcpy(s_last_sample, s_audio_buffer[s_read_idx], AUDIO_SAMPLE_FRAME_BYTES);
            s_read_idx = (s_read_idx + 1) % AUDIO_RING_BUFFER_SAMPLES;
        }
        if (samples_from_buffer) {
            *samples_from_buffer = avail;
        }
        /* Synthesize only the missing suffix; valid buffered stereo frames
         * are consumed exactly once and stay in order. */
        size_t missing = max_samples - avail;
        for (size_t i = 0; i < missing; i++) {
            for (size_t channel = 0; channel < AUDIO_CHANNELS; ++channel) {
                dst[(avail + i) * AUDIO_CHANNELS + channel] =
                    (int16_t)(((int64_t)s_last_sample[channel] *
                               (int64_t)(missing - i - 1)) / (int64_t)missing);
            }
        }
        if (avail > 0) audio_fade_in_unlocked(dst, max_samples);
        memset(s_last_sample, 0, sizeof(s_last_sample));
        /* A transient underflow keeps the RX session alive. Its next real
         * samples need a fresh fade-in even without a 300 ms timeout/reset. */
        s_fade_in_pending = true;
        if (s_draining) s_rx_active = false;
        portEXIT_CRITICAL(&s_audio_lock);
        return max_samples;
    }

    size_t read_base = s_read_idx;
    for (size_t out_idx = 0; out_idx < max_samples; ++out_idx) {
        if (samples_to_consume == max_samples) {
            memcpy(dst + out_idx * AUDIO_CHANNELS,
                   s_audio_buffer[(read_base + out_idx) % AUDIO_RING_BUFFER_SAMPLES],
                   AUDIO_SAMPLE_FRAME_BYTES);
        } else if (max_samples == 1 || samples_to_consume == 1) {
            memcpy(dst + out_idx * AUDIO_CHANNELS, s_audio_buffer[read_base],
                   AUDIO_SAMPLE_FRAME_BYTES);
        } else {
            size_t denominator = max_samples - 1;
            uint64_t position = (uint64_t)out_idx * (samples_to_consume - 1);
            size_t source_idx = (size_t)(position / denominator);
            size_t fraction = (size_t)(position % denominator);
            size_t next_idx = source_idx + 1 < samples_to_consume ? source_idx + 1 : source_idx;
            for (size_t channel = 0; channel < AUDIO_CHANNELS; ++channel) {
                int16_t first = s_audio_buffer[(read_base + source_idx) % AUDIO_RING_BUFFER_SAMPLES][channel];
                int16_t second = s_audio_buffer[(read_base + next_idx) % AUDIO_RING_BUFFER_SAMPLES][channel];
                int64_t weighted = (int64_t)first * (int64_t)(denominator - fraction) +
                                   (int64_t)second * (int64_t)fraction;
                dst[out_idx * AUDIO_CHANNELS + channel] = (int16_t)(weighted / (int64_t)denominator);
            }
        }
    }
    s_read_idx = (s_read_idx + samples_to_consume) % AUDIO_RING_BUFFER_SAMPLES;
    s_level_drop_samples += samples_to_drop;
    s_level_dup_samples += samples_to_stretch;
    audio_fade_in_unlocked(dst, max_samples);
    memcpy(s_last_sample, dst + (max_samples - 1) * AUDIO_CHANNELS, AUDIO_SAMPLE_FRAME_BYTES);
    if (samples_from_buffer) {
        *samples_from_buffer = samples_to_consume;
    }
    if (s_draining && audio_buffer_available_unlocked() == 0) s_rx_active = false;

    portEXIT_CRITICAL(&s_audio_lock);
    return max_samples;
}
#endif

static void audio_stats_reset(void)
{
    s_frames_in = 0;
    s_samples_in = 0;
    s_seq_lost = 0;
    s_seq_reorder = 0;
    s_timeouts = 0;
    s_uac_cb_count = 0;
    s_uac_samples_requested = 0;
    s_uac_samples_output = 0;
    s_uac_samples_from_buffer = 0;
    s_uac_silence_fills = 0;
    s_level_drop_samples = 0;
    s_level_dup_samples = 0;
    s_overflow_drop_samples = 0;
    s_rssi_count = 0;
    s_rssi_sum = 0;
    s_rssi_min = 127;
    s_rssi_max = -127;
}

#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
static void audio_session_start(void)
{
    audio_buffer_reset();
    s_buffer_active = true;
    s_rx_active = false;
    s_start_pending = true;
    s_last_audio_packet_tick = 0;
    s_last_uac_read_tick = 0;
    s_timeout_logged = false;
    s_seq_valid = false;
    s_last_seq = 0;
    ESP_LOGI(TAG,
             "audio buffering session started: threshold=%u samples timeout=%ums target=%u samples",
             (unsigned)AUDIO_START_THRESHOLD_SAMPLES,
             AUDIO_RX_TIMEOUT_MS,
             (unsigned)AUDIO_LEVEL_TARGET_SAMPLES);
}

static bool update_sequence_stats(uint32_t sequence)
{
    if (s_seq_valid) {
        uint32_t expected = s_last_seq + 1;
        if (sequence != expected) {
            uint32_t forward_gap = sequence - expected;
            if (forward_gap < (UINT32_MAX / 2U)) {
                s_seq_lost += forward_gap;
                ESP_LOGD(TAG,
                         "audio sequence gap last=%" PRIu32 " current=%" PRIu32 " lost=%" PRIu32,
                         s_last_seq,
                         sequence,
                         forward_gap);
            } else {
                s_seq_reorder++;
                ESP_LOGD(TAG,
                         "audio sequence reorder last=%" PRIu32 " current=%" PRIu32 " expected=%" PRIu32,
                         s_last_seq,
                         sequence,
                         expected);
                return false;
            }
        }
    }

    s_last_seq = sequence;
    s_seq_valid = true;
    return true;
}
#endif

void dongle_uac_bridge_reset_session(void)
{
    /* A session reset is atomic with the active UAC reader. The timeline and
     * compatibility ring both leave USB clocking silence until new input
     * establishes an eligible playout position. */
    portENTER_CRITICAL(&s_audio_lock);
    ++s_session_generation;
    timeline_reset_unlocked();
    s_draining = false;
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
    s_write_idx = 0;
    s_read_idx = 0;
    memset(s_last_sample, 0, sizeof(s_last_sample));
    s_fade_in_pending = true;
    s_high_slew_phase = 0;
    s_low_slew_phase = 0;
    s_buffer_active = true;
    s_rx_active = false;
    s_start_pending = true;
    s_last_audio_packet_tick = 0;
    s_last_uac_read_tick = 0;
    s_timeout_logged = false;
    s_seq_valid = false;
    s_last_seq = 0;
#else
    s_rx_active = false;
    s_start_pending = false;
#endif
    portEXIT_CRITICAL(&s_audio_lock);
}

void dongle_uac_bridge_end_session(void)
{
    portENTER_CRITICAL(&s_audio_lock);
    s_draining = true;
    s_start_pending = false;
    /* A short utterance may never have reached the nine-packet threshold. */
    if (s_timeline_enabled) {
        s_rx_active = s_timeline_cursor < s_timeline_tail;
    }
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
    else {
        s_rx_active = audio_buffer_available_unlocked() != 0;
    }
#else
    else {
        /* The production path owns no untimed PCM. */
        s_rx_active = false;
    }
#endif
    portEXIT_CRITICAL(&s_audio_lock);
}

bool dongle_uac_bridge_session_drained(void)
{
    portENTER_CRITICAL(&s_audio_lock);
    /* With no PC consumer, wall time still retires the fixed playback tail.
     * USB queued samples are checked separately by the application's sink. */
    bool tail_elapsed = s_timeline_enabled &&
        (s_timeline_cursor >= s_timeline_tail ||
         timeline_position_at_unlocked(esp_timer_get_time()) >= s_timeline_tail);
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
    bool drained = s_draining && (s_timeline_enabled ? tail_elapsed :
                                  audio_buffer_available_unlocked() == 0);
#else
    bool drained = s_draining && (!s_timeline_enabled || tail_elapsed);
#endif
    portEXIT_CRITICAL(&s_audio_lock);
    return drained;
}

esp_err_t dongle_uac_bridge_start(void)
{
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
    audio_buffer_reset();
#endif
    audio_stats_reset();
    timeline_reset_unlocked();
    s_draining = false;
    s_bridge_ready = true;
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
    s_buffer_active = false;
#endif
    s_rx_active = false;
    s_start_pending = false;
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
    s_last_audio_packet_tick = 0;
    s_last_uac_read_tick = 0;
    s_timeout_logged = false;
    s_seq_valid = false;
    ESP_LOGI(TAG,
             "UAC bridge buffer ready: %u Hz, %u-bit, %u ch, ring=%u bytes threshold=%u samples",
             DUAL_END_AUDIO_SAMPLE_RATE_HZ,
             DUAL_END_AUDIO_BITS_PER_SAMPLE,
             AUDIO_CHANNELS,
             AUDIO_RING_BUFFER_BYTES,
             (unsigned)AUDIO_START_THRESHOLD_SAMPLES);
#else
    ESP_LOGI(TAG,
             "UAC bridge timeline ready: %u Hz, %u-bit, %u ch, legacy_ring=disabled",
             DUAL_END_AUDIO_SAMPLE_RATE_HZ,
             DUAL_END_AUDIO_BITS_PER_SAMPLE,
             AUDIO_CHANNELS);
#endif
    ESP_LOGI(TAG, "USB UAC bridge connected to TinyUSB microphone input callback");
    return ESP_OK;
}

size_t dongle_uac_bridge_read_pcm(uint8_t *buf, size_t len)
{
    if (!buf || len == 0) {
        return 0;
    }

    if (!s_bridge_ready) {
        memset(buf, 0, len);
        return len;
    }

    /* Never consume half a pair or shift the channel order for a malformed
     * caller request. TinyUSB's negotiated stereo requests are 4-byte aligned. */
    if (len % AUDIO_SAMPLE_FRAME_BYTES != 0) {
        memset(buf, 0, len);
        return 0;
    }
    size_t samples_needed = len / AUDIO_SAMPLE_FRAME_BYTES;
    if (samples_needed == 0) {
        memset(buf, 0, len);
        return len;
    }

    if (s_timeline_enabled) {
        int64_t output_us = esp_timer_get_time();
        (void)uac_device_input_playout_time(&output_us);
        return timeline_read_pcm(buf, len, output_us);
    }

#if !DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
    /* The production receiver always uses write_timed_pcm. Before its first
     * absolute frame, return an entire UAC block of silence without keeping
     * any legacy PCM state or allocation. */
    s_uac_cb_count++;
    s_uac_samples_requested += samples_needed;
    s_uac_samples_output += samples_needed;
    s_uac_silence_fills++;
    memset(buf, 0, len);
    return len;
#else
    s_uac_cb_count++;
    s_uac_samples_requested += samples_needed;

    TickType_t now = xTaskGetTickCount();
    portENTER_CRITICAL(&s_audio_lock);
    TickType_t last_packet = s_last_audio_packet_tick;
    uint32_t generation = s_session_generation;
    bool rx_active = s_rx_active;
    bool draining = s_draining;
    portEXIT_CRITICAL(&s_audio_lock);
    if (rx_active && !draining &&
        last_packet != 0 &&
        (now - last_packet) >= pdMS_TO_TICKS(AUDIO_RX_TIMEOUT_MS)) {
        bool did_timeout = false;
        portENTER_CRITICAL(&s_audio_lock);
        if (generation == s_session_generation && last_packet == s_last_audio_packet_tick && !s_draining) {
            /* Retire this reader before RX can refill the ring. Its USB
             * producer still belongs to the pre-timeout generation. */
            ++s_session_generation;
            s_rx_active = false;
            s_start_pending = true;
            s_seq_valid = false;
            s_timeouts++;
            s_write_idx = 0;
            s_read_idx = 0;
            memset(s_last_sample, 0, sizeof(s_last_sample));
            s_fade_in_pending = true;
            s_high_slew_phase = 0;
            s_low_slew_phase = 0;
            did_timeout = true;
        }
        portEXIT_CRITICAL(&s_audio_lock);
        if (did_timeout && !s_timeout_logged) {
            ESP_LOGW(TAG,
                     "audio RX timeout after %ums, clearing buffer and switching to silence",
                     AUDIO_RX_TIMEOUT_MS);
            s_timeout_logged = true;
        }
    } else if (last_packet != 0) {
        s_timeout_logged = false;
    }

    if (!s_rx_active) {
        memset(buf, 0, len);
        s_uac_samples_output += samples_needed;
        s_uac_silence_fills++;
        return len;
    }

    portENTER_CRITICAL(&s_audio_lock);
    bool trim_buffer = generation == s_session_generation && !s_draining &&
        s_last_uac_read_tick != 0 &&
        (now - s_last_uac_read_tick) >= pdMS_TO_TICKS(AUDIO_UAC_SESSION_RESTART_MS);
    if (generation == s_session_generation) {
        s_last_uac_read_tick = now;
    }
    portEXIT_CRITICAL(&s_audio_lock);
    if (trim_buffer) {
        audio_buffer_trim_to(AUDIO_LEVEL_TARGET_SAMPLES, generation);
    }

    bool silence_filled = false;
    size_t samples_from_buffer = 0;
    size_t samples_read = audio_buffer_read_silence((int16_t *)buf,
                                                    samples_needed,
                                                    generation,
                                                    &silence_filled,
                                                    &samples_from_buffer);
    if (samples_read < samples_needed) {
        memset(buf + samples_read * AUDIO_SAMPLE_FRAME_BYTES, 0, len - samples_read * AUDIO_SAMPLE_FRAME_BYTES);
    }
    if (len > samples_read * AUDIO_SAMPLE_FRAME_BYTES) {
        memset(buf + samples_read * AUDIO_SAMPLE_FRAME_BYTES, 0, len - samples_read * AUDIO_SAMPLE_FRAME_BYTES);
    }

    s_uac_samples_output += samples_read;
    s_uac_samples_from_buffer += samples_from_buffer;
    if (silence_filled) {
        s_uac_silence_fills++;
    }

    return len;
#endif
}

esp_err_t dongle_uac_bridge_write_pcm(const int16_t *pcm,
                                       size_t samples,
                                       uint32_t sequence,
                                       int rssi,
                                       void *ctx)
{
#if !DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
    (void)pcm;
    (void)samples;
    (void)sequence;
    (void)rssi;
    (void)ctx;
    /* No hidden fallback allocation: production accepts only fixed-timeline
     * PCM supplied by the receiver's timed API. */
    return ESP_ERR_NOT_SUPPORTED;
#else
    (void)ctx;
    ESP_RETURN_ON_FALSE(s_bridge_ready, ESP_ERR_INVALID_STATE, TAG, "UAC bridge not started");
    ESP_RETURN_ON_FALSE(pcm, ESP_ERR_INVALID_ARG, TAG, "pcm is null");
    ESP_RETURN_ON_FALSE(samples > 0, ESP_ERR_INVALID_SIZE, TAG, "pcm samples is zero");
    /* The original double-microphone callback counted interleaved int16_t
     * values (640 per frame). The 0.4.2 timeline counts stereo sample frames
     * (320 per frame). Accept both while the application keeps its old call. */
    if (samples == DUAL_END_AUDIO_FRAME_SAMPLE_VALUES) {
        samples /= AUDIO_CHANNELS;
    }
    ESP_RETURN_ON_FALSE(!s_draining, ESP_ERR_INVALID_STATE, TAG, "PTT session already draining");

    if (!s_buffer_active) {
        audio_session_start();
    }

    if (!update_sequence_stats(sequence)) {
        /* Defensive duplicate/late filtering; recovery normally delivers
         * strictly ordered frames, including timed silence placeholders. */
        return ESP_OK;
    }
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
    s_last_audio_packet_tick = xTaskGetTickCount();
#endif
    s_frames_in++;
    s_samples_in += samples;
    s_rssi_count++;
    s_rssi_sum += rssi;
    if (rssi < s_rssi_min) {
        s_rssi_min = rssi;
    }
    if (rssi > s_rssi_max) {
        s_rssi_max = rssi;
    }

    (void)audio_buffer_write(pcm, samples);

    size_t avail = audio_buffer_available();
    if (s_start_pending && avail >= AUDIO_START_THRESHOLD_SAMPLES) {
        s_start_pending = false;
        s_rx_active = true;
        ESP_LOGI(TAG,
                 "audio output enabled after buffering %u samples",
                 (unsigned)AUDIO_START_THRESHOLD_SAMPLES);
    }

    if ((s_frames_in % 50) == 0) {
        int rssi_avg = s_rssi_count ? (int)(s_rssi_sum / (int32_t)s_rssi_count) : 0;
        ESP_LOGD(TAG,
                 "audio rx seq=%" PRIu32 " frames=%" PRIu32 " samples=%" PRIu32
                 " avail=%u active=%d start_pending=%d lost=%" PRIu32 " reorder=%" PRIu32
                 " timeout=%" PRIu32 " silence=%" PRIu32 " drop=%" PRIu32 " dup=%" PRIu32
                 " overflow=%" PRIu32 " rssi[min=%d max=%d avg=%d]",
                 sequence,
                 s_frames_in,
                 s_samples_in,
                 (unsigned)avail,
                 s_rx_active ? 1 : 0,
                 s_start_pending ? 1 : 0,
                 s_seq_lost,
                 s_seq_reorder,
                 s_timeouts,
                 s_uac_silence_fills,
                 s_level_drop_samples,
                 s_level_dup_samples,
                 s_overflow_drop_samples,
                 s_rssi_min,
                 s_rssi_max,
                 rssi_avg);
        (void)rssi_avg;
    }

    return ESP_OK;
#endif
}

esp_err_t dongle_uac_bridge_write_timed_pcm(const int16_t *pcm,
    size_t samples, uint32_t sequence, int rssi, int64_t playout_us, void *ctx)
{
    (void)ctx;
    if (!s_bridge_ready) return ESP_ERR_INVALID_STATE;
    if (!pcm || samples != DUAL_END_AUDIO_FRAME_SAMPLES || playout_us <= 0) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_audio_lock);
    if (s_draining) {
        portEXIT_CRITICAL(&s_audio_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_timeline_enabled) {
        timeline_reset_unlocked();
        s_timeline_enabled = true;
        s_timeline_first_sequence = sequence;
        s_timeline_start_us = playout_us;
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
        s_buffer_active = true;
#endif
        s_start_pending = true;
        s_rx_active = false;
    }
    timeline_retire_expired_unlocked(esp_timer_get_time());
    int64_t frame_index = (int32_t)(sequence - s_timeline_first_sequence);
    int64_t begin = frame_index * DUAL_END_AUDIO_FRAME_SAMPLES;
    int64_t end = begin + (int64_t)samples;
    if (frame_index < 0 || end <= s_timeline_cursor) {
        s_timeline_late_samples += samples;
        portEXIT_CRITICAL(&s_audio_lock);
        return ESP_OK;
    }
    if (begin < s_timeline_cursor) {
        s_timeline_late_samples += (uint64_t)(s_timeline_cursor - begin);
    }
    audio_timeline_slot_t *slot = &s_timeline_slots[(uint64_t)frame_index % AUDIO_TIMELINE_SLOTS];
    if (slot->present && slot->frame_index == frame_index) {
        portEXIT_CRITICAL(&s_audio_lock);
        return ESP_OK;
    }
    if (slot->present && (slot->frame_index + 1) * DUAL_END_AUDIO_FRAME_SAMPLES > s_timeline_cursor) {
        s_overflow_drop_samples += (uint32_t)samples;
        portEXIT_CRITICAL(&s_audio_lock);
        /* A bounded timeline slot can still hold live PCM when a repaired RX
         * burst arrives more than one ring ahead of USB playback. Preserve
         * the older scheduled frame, count the newer frame as a local sink
         * drop, and let the recovery session continue. ESP_ERR_NO_MEM is a
         * documented soft backpressure result for this one condition. */
        return ESP_ERR_NO_MEM;
    }
    slot->frame_index = frame_index;
    memcpy(slot->pcm, pcm, samples * AUDIO_SAMPLE_FRAME_BYTES);
    slot->present = true;
    if (end > s_timeline_tail) s_timeline_tail = end;
    ++s_frames_in;
    s_samples_in += (uint32_t)samples;
    ++s_rssi_count;
    s_rssi_sum += rssi;
    if (rssi < s_rssi_min) s_rssi_min = rssi;
    if (rssi > s_rssi_max) s_rssi_max = rssi;
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
    s_last_audio_packet_tick = xTaskGetTickCount();
#endif
    portEXIT_CRITICAL(&s_audio_lock);
    return ESP_OK;
}

void dongle_uac_bridge_get_stats(dongle_uac_bridge_stats_t *out_stats)
{
    if (!out_stats) {
        return;
    }

    portENTER_CRITICAL(&s_audio_lock);
    out_stats->rx_active = s_rx_active;
    out_stats->start_pending = s_start_pending;
    out_stats->frames_in = s_frames_in;
    out_stats->sequence_lost = s_seq_lost;
    out_stats->sequence_reordered = s_seq_reorder;
    out_stats->timeouts = s_timeouts;
    out_stats->silence_fills = s_uac_silence_fills;
    out_stats->level_drop_samples = s_level_drop_samples;
    out_stats->level_dup_samples = s_level_dup_samples;
    out_stats->overflow_drop_samples = s_overflow_drop_samples;
    uint32_t buffered_frames = s_timeline_enabled ?
        timeline_buffered_samples_unlocked(esp_timer_get_time()) : 0;
#if DONGLE_UAC_BRIDGE_ENABLE_LEGACY_RING
    if (!s_timeline_enabled) {
        buffered_frames = (uint32_t)audio_buffer_available_unlocked();
    }
#endif
    out_stats->buffer_samples = buffered_frames * AUDIO_CHANNELS;
    out_stats->concealed_frames = 0;
    out_stats->session_generation = s_session_generation;
    out_stats->timeline_enabled = s_timeline_enabled;
    out_stats->timeline_started = s_timeline_started;
    out_stats->timeline_start_us = s_timeline_start_us;
    out_stats->timeline_first_output_us = s_timeline_first_output_us;
    out_stats->timeline_cursor_samples = (uint64_t)s_timeline_cursor;
    out_stats->timeline_late_samples = s_timeline_late_samples;
    out_stats->timeline_local_retired_samples = s_timeline_local_retired_samples;
    out_stats->timeline_resume_skipped_samples = s_timeline_resume_skipped_samples;
    out_stats->timeline_pause_count = s_timeline_pause_count;
    out_stats->timeline_max_gap_us = s_timeline_max_gap_us;
    portEXIT_CRITICAL(&s_audio_lock);
}

uint32_t dongle_uac_bridge_timeline_capacity_ms(void)
{
    return AUDIO_TIMELINE_SLOTS * DUAL_END_AUDIO_FRAME_MS;
}

uint32_t dongle_uac_bridge_session_generation(void)
{
    portENTER_CRITICAL(&s_audio_lock);
    const uint32_t generation = s_session_generation;
    portEXIT_CRITICAL(&s_audio_lock);
    return generation;
}
