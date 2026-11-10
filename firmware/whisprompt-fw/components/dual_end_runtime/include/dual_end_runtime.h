#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "dual_end_link_config.h"
#include "dual_end_task_policy.h"
#include "dual_end_perf_stats.h"
typedef enum {
    DUAL_END_STAGE_ENCODE = 0,
    DUAL_END_STAGE_MIC_PIPELINE,
    DUAL_END_STAGE_DECODE,
    DUAL_END_STAGE_RX_POLL,
    DUAL_END_STAGE_COUNT
} dual_end_perf_stage_t;

/* A bounded copy of the diagnostic caches. This deliberately exposes only
 * identity and window metadata: callers never receive a live producer window
 * and therefore cannot make the audio owner scan memory or format JSON. */
typedef struct {
    bool runtime_ready;
    bool resource_ready;
    uint8_t role;
    uint8_t perf_ready_mask;
    uint32_t boot_id;
    uint32_t resource_sequence;
    uint64_t resource_sampled_us;
    uint32_t perf_window_sequence[DUAL_END_STAGE_COUNT];
    uint32_t perf_session_id[DUAL_END_STAGE_COUNT];
    uint8_t perf_codec[DUAL_END_STAGE_COUNT];
} dual_end_runtime_cache_identity_t;
/* Optional startup identity registration, before runtime_start. Nonzero IDs
 * let early startup diagnostics and the runtime share one boot identity. */
esp_err_t dual_end_runtime_set_boot_id(uint32_t boot_id);
esp_err_t dual_end_runtime_start(scepter_role_t role);
/* Each stage has exactly one producer task; separate stages may have different
 * producers (for example asynchronous encoding and microphone DSP). Stage
 * windows remain owner-local; shared nonzero window IDs and saturating drop
 * counters use atomics. Snapshot is copied into a nonblocking diagnostic queue;
 * no formatting or logging on the audio owner. No audio packet/PCM queue added. */
void dual_end_runtime_record(dual_end_perf_stage_t stage, uint8_t codec,
    uint32_t session_id, int64_t elapsed_us, bool success);
void dual_end_runtime_record_outcome(dual_end_perf_stage_t stage, uint8_t codec,
    uint32_t session_id, int64_t elapsed_us, dual_end_owner_outcome_t outcome);

/* Immutable boot identity. Zero means optional runtime initialization failed.
 * Owner context is queried ONLY by that stage's producer after runtime_record;
 * diagnostic tasks must consume an independently copied owner snapshot. */
uint32_t dual_end_runtime_boot_id(void);
/* Identity-only diagnostics accessor. Zero means identity initialization has
 * not completed; a nonzero value matches device_boot/BUILD_INFO even when
 * optional profiler allocation failed. No allocation, scan, lock or logging. */
uint32_t dual_end_runtime_identity_boot_id(void);
/* Copies already-published resource/performance identity under the runtime
 * cache lock. It never samples heap, stack, CPU, codecs, or producer state. */
bool dual_end_runtime_cache_identity(dual_end_runtime_cache_identity_t *out);
bool dual_end_runtime_owner_context(dual_end_perf_stage_t stage,
    uint32_t *session_id, uint8_t *codec, uint64_t *sampled_us);

/* Register only before runtime_start: probe executes on the low-priority
 * logger, never in a USB command callback or audio owner. */
typedef bool (*dual_end_runtime_io_probe_t)(uint32_t values[8]);
esp_err_t dual_end_runtime_set_io_probe(dual_end_runtime_io_probe_t probe);
/* Exact read-only TELEMETRY token part / PERF token stage part. Token echoes
 * identify replies; unavailable cache remains explicit and OTA is untouched.
 *
 * PERF_SNAPSHOT token stage part is additive.  Part zero creates (or
 * idempotently resumes) a one-record cache lease; parts 1..12 must use the
 * identical token and stage or receive snapshot_lease_mismatch.  This lets a
 * strict host reconstruct one immutable 13-part performance window while the
 * low-priority logger continues publishing newer PERF windows.  Legacy PERF
 * remains byte-compatible and intentionally has no lease semantics.
 * TELEMETRY uses the same part-zero/token lease for its resource snapshot.
 * All leases expire 10 seconds after their first part-zero request; retries
 * do not extend that lifetime. A new part zero may then start a fresh lease.
 * BUILD_INFO token 0 returns the immutable running ELF SHA256, separately
 * from the unchanged six-part device_boot event, even when optional profiling
 * allocation fails after identity initialization. */
bool dual_end_runtime_command(const char *line,char *response,size_t capacity);
