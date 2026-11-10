#pragma once

#include <stdbool.h>

/* AUDIO_SESSION_BEGIN is idempotent for a matching session/first sequence.
 * Retry one immediate ESP-NOW NO_MEM rejection before the normal session
 * retry interval. Other error classes must wait for the normal state gate. */
#define DUAL_END_AUDIO_CONTROL_NO_MEM_RETRY_COUNT 1U

static inline unsigned dual_end_audio_control_max_attempts(void)
{
    return 1U + DUAL_END_AUDIO_CONTROL_NO_MEM_RETRY_COUNT;
}

static inline bool dual_end_audio_control_retry_allowed(unsigned failed_attempt,
                                                        bool no_mem)
{
    return no_mem && failed_attempt < DUAL_END_AUDIO_CONTROL_NO_MEM_RETRY_COUNT;
}
