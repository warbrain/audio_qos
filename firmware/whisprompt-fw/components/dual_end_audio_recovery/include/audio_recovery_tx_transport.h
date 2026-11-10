#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Implemented by the Scepter audio adapter. A successful prepare retains its
 * mutex until submitted is called, including synchronous driver failure.
 * The transport may hold its TX mutex; audio enqueue/discard are asynchronous.
 * No wait for a radio callback may occur inside this short transaction. */
bool dual_end_audio_repair_tx_prepare(uint32_t session_id, uint32_t sequence,
                                      uint32_t reservation_id, int64_t now_us);
void dual_end_audio_repair_tx_submitted(uint32_t session_id, uint32_t sequence,
                                       uint32_t reservation_id, bool accepted,
                                       int64_t now_us);
void dual_end_audio_repair_tx_complete(uint32_t session_id, uint32_t sequence,
                                      uint32_t reservation_id, bool submitted,
                                      bool success, int64_t now_us);

#ifdef __cplusplus
}
#endif
