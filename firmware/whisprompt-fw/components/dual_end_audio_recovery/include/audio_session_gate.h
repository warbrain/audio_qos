#pragma once

#include <stdbool.h>

typedef enum {
    AUDIO_SESSION_GATE_KEEP = 0,
    AUDIO_SESSION_GATE_INVALIDATE,
    AUDIO_SESSION_GATE_START,
} audio_session_gate_action_t;

/* Recovery sessions exist only while the transport can carry audio.  Edge
 * actions keep a bounded RF pause from repeatedly rotating the session ID:
 * enter an unavailable state once, then start exactly once when all gates
 * reopen. */
static inline audio_session_gate_action_t audio_session_gate_transition(
    bool was_connected, bool was_ota_paused, bool was_channel_paused,
    bool connected, bool ota_paused, bool channel_paused)
{
    const bool was_available = was_connected && !was_ota_paused &&
                               !was_channel_paused;
    const bool available = connected && !ota_paused && !channel_paused;
    if (was_available == available) {
        return AUDIO_SESSION_GATE_KEEP;
    }
    return available ? AUDIO_SESSION_GATE_START :
                       AUDIO_SESSION_GATE_INVALIDATE;
}

