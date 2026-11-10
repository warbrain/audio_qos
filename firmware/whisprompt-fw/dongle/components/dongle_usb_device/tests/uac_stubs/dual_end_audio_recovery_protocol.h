#pragma once

/* The legacy ring-buffer host test does not exercise the recovery state
 * machine. It only needs the current fixed playout target used by the bridge. */
#define DUAL_END_AUDIO_PLAYOUT_DELAY_US 232000LL
