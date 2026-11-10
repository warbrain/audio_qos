#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Timing policy only. A generation number does not fence an old SDK callback:
 * the owner must stop the old Wi-Fi task before rearming a new transmission. */
#define AUDIO_RADIO_HARD_TIMEOUT_US INT64_C(2000000)
/* Only explicit maintenance/test requests are rate-limited by this cooldown.
 * An accepted send with no callback gets the full 2 s on every attempt. */
#define AUDIO_RADIO_RECOVERY_COOLDOWN_US INT64_C(10000000)
#define AUDIO_RADIO_WAIT_SLICE_MS 10U

typedef enum {
    AUDIO_RADIO_READY = 0,
    AUDIO_RADIO_QUIESCING,
    AUDIO_RADIO_RECOVERING,
    AUDIO_RADIO_RESTORING,
    AUDIO_RADIO_FAILED_OFF,
} audio_radio_state_t;

typedef enum {
    AUDIO_RADIO_REASON_NONE = 0,
    AUDIO_RADIO_REASON_EARLY, /* Reserved telemetry value; never auto-triggered. */
    AUDIO_RADIO_REASON_HARD,
    AUDIO_RADIO_REASON_FORCED,
} audio_radio_reason_t;

static inline bool audio_radio_can_submit(audio_radio_state_t state)
{
    return state == AUDIO_RADIO_READY;
}

static inline bool audio_radio_manual_recovery_allowed(int64_t now_us,
                                              int64_t last_recovery_us)
{
    return last_recovery_us == 0 ||
           (now_us >= last_recovery_us &&
            now_us - last_recovery_us >= AUDIO_RADIO_RECOVERY_COOLDOWN_US);
}

static inline audio_radio_reason_t audio_radio_recovery_due(
    int64_t now_us, int64_t submitted_us, int64_t last_recovery_us,
    bool forced)
{
    (void)last_recovery_us;
    if (forced) {
        return AUDIO_RADIO_REASON_FORCED;
    }
    if (submitted_us < 0 || now_us < submitted_us) {
        return AUDIO_RADIO_REASON_NONE;
    }
    int64_t elapsed_us = now_us - submitted_us;
    if (elapsed_us >= AUDIO_RADIO_HARD_TIMEOUT_US) {
        return AUDIO_RADIO_REASON_HARD;
    }
    return AUDIO_RADIO_REASON_NONE;
}
