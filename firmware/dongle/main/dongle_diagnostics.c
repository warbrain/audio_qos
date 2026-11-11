#include "dongle_diagnostics.h"

#include "dongle_diagnostics_format.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio_rx_telemetry.h"
#include "dongle_uac_bridge.h"
#include "dongle_usb_device.h"
#include "dual_end_espnow_link.h"
#include "dual_end_runtime.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* This worker copies several bounded cache structs. Keep a 4 KiB stack and
 * expose its live low-water value through the v2 RX evidence part. */
#define DONGLE_DIAGNOSTICS_TASK_STACK_BYTES 4096U
#define DONGLE_DIAGNOSTICS_TASK_PRIORITY 1U
#define DONGLE_DIAGNOSTICS_SAMPLE_MS 1000U
#define DONGLE_DIAGNOSTICS_LEASE_US UINT64_C(10000000)
#define DONGLE_CDC_LOG_HEALTH_INTERVAL_US UINT64_C(5000000)

/* The compact contract exposes the four existing runtime stages by index.
 * Fail the firmware build if a future runtime changes that bounded set. */
_Static_assert(DONGLE_DIAGNOSTICS_PERF_STAGES == DUAL_END_STAGE_COUNT,
               "diagnostic perf-stage schema must track runtime stages");

typedef struct {
    uint32_t run_id;
    uint32_t arm_id;
    uint32_t sequence;
} diagnostics_context_t;

/* v1 intentionally relabels the current cache when DIAG_CONTEXT arrives.
 * Keep v2 separate: it is eligible only after an immutable RX-owner sample
 * published after this context's committed boundary and bound to one complete
 * RX/UAC/runtime identity. */
typedef struct {
    uint32_t run_id;
    uint32_t arm_id;
    uint32_t sequence;
    uint64_t set_us;
    uint32_t rx_cache_sequence_floor;
    uint32_t bound_session_id;
    uint32_t bound_rx_generation;
    uint32_t bound_uac_generation;
    bool required;
    bool identity_bound;
} diagnostics_v2_context_t;

static portMUX_TYPE s_diagnostics_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_diagnostics_task;
static diagnostics_context_t s_context;
static diagnostics_v2_context_t s_v2_context;
static dongle_diagnostics_snapshot_t s_cache;
static bool s_cache_ready;
typedef struct {
    bool valid;
    uint32_t token;
    uint64_t started_us;
    dongle_diagnostics_snapshot_t snapshot;
} diagnostics_snapshot_lease_t;
/* One fixed cache copy per existing command family, never an audio queue. */
static diagnostics_snapshot_lease_t s_snapshot_leases[2];

static uint64_t positive_us(int64_t value)
{
    return value > 0 ? (uint64_t)value : 0;
}

static uint32_t increment_nonzero(uint32_t value)
{
    ++value;
    return value ? value : 1U;
}

static uint64_t owner_age_us(uint64_t sampled_us, int64_t owner_sampled_us)
{
    if (owner_sampled_us <= 0) return 0;
    const uint64_t owner_us = (uint64_t)owner_sampled_us;
    return sampled_us >= owner_us ? sampled_us - owner_us : 0;
}

#define DONGLE_DIAGNOSTICS_RX_PERF_MASK \
    ((uint8_t)((1U << DUAL_END_STAGE_DECODE) | (1U << DUAL_END_STAGE_RX_POLL)))

static bool rx_identity_present(const audio_rx_telemetry_snapshot_t *rx)
{
    return rx && rx->sampled_us > 0 && rx->session_id != 0 &&
        rx->session_generation != 0 && rx->codec != UINT8_MAX;
}

static bool runtime_identity_known(const dual_end_runtime_cache_identity_t *runtime)
{
    return runtime && runtime->runtime_ready && runtime->boot_id != 0 &&
        (runtime->perf_ready_mask & DONGLE_DIAGNOSTICS_RX_PERF_MASK) ==
            DONGLE_DIAGNOSTICS_RX_PERF_MASK;
}

static bool runtime_identity_matches_rx(const dual_end_runtime_cache_identity_t *runtime,
                                        const audio_rx_telemetry_snapshot_t *rx)
{
    if (!runtime_identity_known(runtime) || !rx_identity_present(rx)) return false;
    const unsigned stages[] = {DUAL_END_STAGE_DECODE, DUAL_END_STAGE_RX_POLL};
    for (unsigned i = 0; i < sizeof(stages) / sizeof(stages[0]); ++i) {
        const unsigned stage = stages[i];
        if (runtime->perf_session_id[stage] != rx->session_id ||
            runtime->perf_codec[stage] != rx->codec) return false;
    }
    return true;
}

static uint32_t diagnostics_stack_min(void)
{
    /* ESP-IDF reports this FreeRTOS high-water mark in bytes. This helper is
     * invoked only by diagnostics_task, never a CDC command or RX producer. */
    return (uint32_t)uxTaskGetStackHighWaterMark(NULL);
}

static void diagnostics_sample(void)
{
    static uint64_t last_log_health_us;
    dual_end_runtime_cache_identity_t runtime = {0};
    audio_rx_telemetry_cache_t rx_cache = {0};
    dongle_uac_bridge_stats_t uac = {0};
    dual_end_espnow_link_diagnostics_t link = {0};
    const bool runtime_ready = dual_end_runtime_cache_identity(&runtime);
    const bool rx_ready = audio_rx_telemetry_get_latest(&rx_cache);
    dongle_uac_bridge_get_stats(&uac);
    const bool link_ready = dual_end_espnow_link_get_diagnostics(&link) == ESP_OK;
    const audio_rx_telemetry_snapshot_t *rx = rx_ready ? &rx_cache.snapshot : NULL;
    const bool rx_identity_ready = rx_identity_present(rx);
    const bool runtime_identity_ready = runtime_identity_known(&runtime);
    const bool runtime_identity_mixed = rx_identity_ready && runtime_identity_ready &&
        !runtime_identity_matches_rx(&runtime, rx);
    const bool uac_identity_mixed = rx_identity_ready &&
        rx->uac_session_generation != uac.session_generation;
    const bool v2_identity_ready = rx_identity_ready && runtime_identity_ready &&
        !runtime_identity_mixed && !uac_identity_mixed;
    const bool v2_identity_mixed = runtime_identity_mixed || uac_identity_mixed;

    dongle_diagnostics_snapshot_t next = {0};
    dongle_usb_device_stats_t usb = {0};
    dongle_usb_device_get_stats(&usb);
    dongle_cdc_log_stats_t cdc_log={0};
    dongle_usb_cdc_get_log_stats(&cdc_log);
    next.cdc_log_attempted=cdc_log.attempted;
    next.cdc_log_enqueued=cdc_log.enqueued;
    next.cdc_log_dropped=cdc_log.dropped;
    next.cdc_log_truncated=cdc_log.truncated;
    next.cdc_log_write_failed=cdc_log.write_failed;
    next.cdc_connected = usb.cdc_connected;
    next.cdc_dtr = usb.cdc_dtr;
    next.cdc_rx_commands = usb.cdc_rx_commands;
    next.cdc_rx_queue_overflows = usb.cdc_rx_queue_overflows;
    next.cdc_response_drops = usb.cdc_response_drops;
    next.cdc_response_tx_queued_bytes = usb.cdc_response_tx_queued_bytes;
    next.cdc_response_tx_incomplete = usb.cdc_response_tx_incomplete;
    next.runtime_ready = runtime_ready && runtime.runtime_ready;
    next.resource_cache_ready = runtime.resource_ready;
    next.rx_cache_ready = rx_ready;
    next.role = runtime.role;
    next.boot_id = runtime.boot_id;
    next.sampled_us = (uint64_t)esp_timer_get_time();
    next.diagnostics_stack_min_bytes = diagnostics_stack_min();
    next.resource_sequence = runtime.resource_sequence;
    next.resource_sampled_us = runtime.resource_sampled_us;
    next.perf_ready_mask = runtime.perf_ready_mask;
    for (unsigned i = 0; i < DONGLE_DIAGNOSTICS_PERF_STAGES; ++i) {
        next.perf_window_sequence[i] = runtime.perf_window_sequence[i];
    }

    /* The RX logger remains authoritative because it is copied by the RX
     * owner. Before its first completed record, a completed runtime decode
     * window gives the host a useful, explicitly cached codec/session hint. */
    next.codec = UINT8_MAX;
    if (runtime.perf_ready_mask & (1U << DUAL_END_STAGE_DECODE)) {
        next.codec = runtime.perf_codec[DUAL_END_STAGE_DECODE];
        next.session_id = runtime.perf_session_id[DUAL_END_STAGE_DECODE];
    } else if (runtime.perf_ready_mask & (1U << DUAL_END_STAGE_RX_POLL)) {
        next.codec = runtime.perf_codec[DUAL_END_STAGE_RX_POLL];
        next.session_id = runtime.perf_session_id[DUAL_END_STAGE_RX_POLL];
    }
    if (rx_ready) {
        next.codec = rx->codec;
        next.session_id = rx->session_id;
        next.rx_active = rx->active;
        next.rx_accepted = rx->stats.accepted;
        next.rx_output_frames = rx->stats.output_frames;
        next.rx_permanent_loss = rx->stats.permanent_loss;
        next.rx_gap_events = rx->stats.gap_events;
        next.rx_recovered = rx->stats.recovered;
        next.rx_duplicates = rx->stats.duplicates;
        next.rx_late = rx->stats.late;
        next.rx_rejected = rx->stats.rejected;
        next.rx_abort_count = rx->stats.abort_count;
        next.rx_nack_requests = rx->stats.nack_requests;
        next.rx_nack_accepted = rx->stats.nack_accepted;
        next.rx_nack_cancelled = rx->stats.nack_cancelled;
        next.rx_nack_expired = rx->stats.nack_expired;
        next.rx_nack_rejected = rx->stats.nack_rejected;
        next.rx_resync_requests = rx->stats.resync_requests;
        next.rx_delivered_decoded = rx->delivered_frames;
        next.rx_masked_frames = rx->masked_frames;
        next.rx_recovered_decoded = rx->recovered_decoded_frames;
        next.rx_crc_failures = rx->crc_error_frames;
        next.rx_decode_failures = rx->decode_error_frames;
        next.rx_invalid_frames = rx->invalid_frames;
        next.rx_feedback_queue_failures = rx->feedback_queue_failures;
        next.rx_nack_queued = rx->nack_queued;
        next.rx_poll_max_us = rx->poll_max_us;
        next.rx_decode_max_us = rx->decode_frame_max_us;
        next.rx_poll_audio_max = rx->poll_audio_max;
        next.rx_cache_sequence = rx_cache.sequence;
        next.rx_snapshot_drops = rx_cache.snapshots_dropped;
        next.rx_event_drops = rx_cache.events_dropped;
        next.rx_stack_min_bytes = rx->rx_stack_min_bytes;
        /* These IDs are copied only from the immutable RX owner record. The
         * current runtime/UAC caches are compared below before v2 may expose
         * this sample, so a one-second diagnostics join cannot silently blend
         * a retired RX record with a newer session or generation. */
        next.v2_rx_session_id = rx->session_id;
        next.v2_rx_session_generation = rx->session_generation;
        next.v2_uac_session_generation = rx->uac_session_generation;
        next.v2_rx_cache_sequence = rx_cache.sequence;
        /* Strict v2 evidence comes only from the completed owner snapshot and
         * the logger cache that copied it. Do not substitute live RX globals
         * or link-level fallback metrics when either cache fact is absent. */
        if (rx->sampled_us > 0) {
            next.rx_owner_timing_ready = true;
            next.rx_owner_sampled_us = (uint64_t)rx->sampled_us;
            next.rx_owner_age_us = owner_age_us(next.sampled_us, rx->sampled_us);
            next.rx_cache_published_us = rx_cache.cache_published_us;
            next.rx_owner_stack_min_bytes = rx->rx_stack_min_bytes;
            next.rx_telemetry_stack_min_bytes = rx_cache.telemetry_stack_min_bytes;
        }
    }

    next.uac_rx_active = uac.rx_active;
    next.uac_start_pending = uac.start_pending;
    next.uac_frames_in = uac.frames_in;
    next.uac_sequence_lost = uac.sequence_lost;
    next.uac_sequence_reordered = uac.sequence_reordered;
    next.uac_timeouts = uac.timeouts;
    next.uac_silence_fills = uac.silence_fills;
    next.uac_buffer_samples = uac.buffer_samples;
    next.uac_overflow_drops = uac.overflow_drop_samples;
    next.uac_level_drop_samples = uac.level_drop_samples;
    next.uac_level_dup_samples = uac.level_dup_samples;
    next.uac_timeline_enabled = uac.timeline_enabled;
    next.uac_timeline_started = uac.timeline_started;
    next.uac_timeline_cursor_samples = uac.timeline_cursor_samples;
    next.uac_timeline_late_samples = uac.timeline_late_samples;
    next.uac_timeline_local_retired_samples = uac.timeline_local_retired_samples;
    next.uac_timeline_resume_skipped_samples = uac.timeline_resume_skipped_samples;
    next.uac_timeline_pause_count = uac.timeline_pause_count;
    next.uac_timeline_max_gap_us = uac.timeline_max_gap_us;
    next.uac_timeline_start_us = positive_us(uac.timeline_start_us);
    next.uac_timeline_first_output_us = positive_us(uac.timeline_first_output_us);

    if (link_ready) {
        next.link_connected = link.connected;
        next.channel_pending = link.channel_pending;
        next.channel = link.current_channel;
        next.channel_state = link.channel_state;
        next.rx_queue_drops = link.rx_queue_drops;
        next.rx_invalid_lengths = link.rx_invalid_lengths;
        next.tx_accepted = link.tx_accepted;
        next.tx_callbacks = link.tx_callbacks;
        next.tx_immediate_failures = link.tx_immediate_failures;
        next.tx_soft_timeouts = link.tx_soft_timeouts;
        next.tx_late_callbacks = link.tx_late_callbacks;
        next.tx_hard_recoveries = link.tx_hard_recoveries;
        next.tx_recovery_failures = link.tx_recovery_failures;
        if (!rx_ready) next.rx_stack_min_bytes = link.rx_stack_min_bytes;
        next.tx_stack_min_bytes = link.tx_stack_min_bytes;
        next.link_stack_min_bytes = link.link_stack_min_bytes;
    }

    /* One short critical copy joins context with the independently gathered
     * snapshots. No producer, codec, ESP-NOW callback, or USB command waits. */
    portENTER_CRITICAL(&s_diagnostics_lock);
    next.run_id = s_context.run_id;
    next.arm_id = s_context.arm_id;
    next.context_sequence = s_context.sequence;
    next.context_set = next.run_id != 0 || next.arm_id != 0;
    next.v2_run_id = s_v2_context.run_id;
    next.v2_arm_id = s_v2_context.arm_id;
    next.v2_context_sequence = s_v2_context.sequence;
    next.v2_context_set = s_v2_context.required;

    /* A context applies only to a cache record published after DIAG_CONTEXT,
     * never to the v1-relabeled cache that may still be in s_cache. The
     * sequence floor makes the boundary robust even if an owner timestamp is
     * equal to the command timestamp on a coarse timer tick. */
    const bool v2_fresh_rx = next.rx_owner_timing_ready &&
        next.rx_cache_sequence != s_v2_context.rx_cache_sequence_floor &&
        next.rx_owner_sampled_us >= s_v2_context.set_us &&
        next.rx_cache_published_us >= s_v2_context.set_us;
    bool v2_bound_identity_matches = !s_v2_context.identity_bound ||
        (next.v2_rx_session_id == s_v2_context.bound_session_id &&
         next.v2_rx_session_generation == s_v2_context.bound_rx_generation &&
         next.v2_uac_session_generation == s_v2_context.bound_uac_generation);
    if (s_v2_context.required && v2_fresh_rx && v2_identity_ready &&
        !s_v2_context.identity_bound) {
        s_v2_context.bound_session_id = next.v2_rx_session_id;
        s_v2_context.bound_rx_generation = next.v2_rx_session_generation;
        s_v2_context.bound_uac_generation = next.v2_uac_session_generation;
        s_v2_context.identity_bound = true;
        v2_bound_identity_matches = true;
    }
    next.v2_identity_mixed = v2_identity_mixed ||
        (s_v2_context.identity_bound && !v2_bound_identity_matches);
    next.v2_identity_ready = v2_identity_ready && !next.v2_identity_mixed;
    next.v2_context_ready = !s_v2_context.required ? next.v2_identity_ready :
        (v2_fresh_rx && next.v2_identity_ready && s_v2_context.identity_bound);
    next.cache_sequence = increment_nonzero(s_cache.cache_sequence);
    s_cache = next;
    s_cache_ready = next.runtime_ready && next.boot_id != 0;
    portEXIT_CRITICAL(&s_diagnostics_lock);
    /* Reuse this worker-local immutable sample. Do not add a second large
     * snapshot to the 4 KiB diagnostics task stack or wait while holding locks. */
    if(next.runtime_ready && next.boot_id && (!last_log_health_us ||
       next.sampled_us-last_log_health_us>=DONGLE_CDC_LOG_HEALTH_INTERVAL_US)){
        char json[181];
        for(unsigned part=0;part<DONGLE_CDC_LOG_HEALTH_PARTS;++part){
            if(dongle_diagnostics_cdc_log_format_part(&next,part,json,sizeof(json)))
                ESP_LOGI("dongle_diag","%s",json);
            vTaskDelay(1);
        }
        last_log_health_us=next.sampled_us;
    }
}

static void diagnostics_task(void *arg)
{
    (void)arg;
    for (;;) {
        diagnostics_sample();
        vTaskDelay(pdMS_TO_TICKS(DONGLE_DIAGNOSTICS_SAMPLE_MS));
    }
}

esp_err_t dongle_diagnostics_start(void)
{
    if (s_diagnostics_task) return ESP_OK;
    BaseType_t created = xTaskCreatePinnedToCore(diagnostics_task,
        "dongle_diag", DONGLE_DIAGNOSTICS_TASK_STACK_BYTES, NULL,
        DONGLE_DIAGNOSTICS_TASK_PRIORITY, &s_diagnostics_task,
        DUAL_END_COMM_TASK_CORE);
    if (created != pdPASS) {
        s_diagnostics_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static bool parse_numbers(const char *text, uint32_t values[3], unsigned count)
{
    if (!text || !values || count == 0 || count > 3) return false;
    for (unsigned i = 0; i < count; ++i) {
        while (*text == ' ' || *text == '\t') ++text;
        if (*text < '0' || *text > '9') return false;
        errno = 0;
        char *end = NULL;
        const unsigned long long parsed = strtoull(text, &end, 10);
        if (errno != 0 || end == text || parsed > UINT32_MAX) return false;
        values[i] = (uint32_t)parsed;
        text = end;
        if (i + 1 < count && *text != ' ' && *text != '\t') return false;
    }
    while (*text == ' ' || *text == '\t') ++text;
    return *text == '\0' && values[0] != 0;
}

static void response_error_for(char *response, size_t capacity, const char *command,
                               const char *detail)
{
    if (!response || !capacity) return;
    const int written = snprintf(response, capacity,
                                 "ERR %s %s\n", command, detail);
    if (written < 0 || (size_t)written >= capacity) response[0] = '\0';
}

static void response_error(char *response, size_t capacity, const char *detail)
{
    response_error_for(response, capacity, "DONGLE_TELEMETRY", detail);
}

static bool set_context(uint32_t token, uint32_t run_id, uint32_t arm_id,
                        char *response, size_t capacity)
{
    uint32_t sequence;
    uint32_t v2_sequence;
    /* The v2 floor must come from the latest immutable RX cache itself, not
     * from the 1 Hz diagnostics mirror.  Otherwise a record already published
     * before DIAG_CONTEXT could look newer merely because the mirror lagged. */
    audio_rx_telemetry_cache_t latest_rx_cache = {0};
    const bool latest_rx_ready = audio_rx_telemetry_get_latest(&latest_rx_cache);
    const uint32_t rx_cache_sequence_floor = latest_rx_ready ?
        latest_rx_cache.sequence : 0;
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_snapshot_leases[0].valid=false;
    s_snapshot_leases[1].valid=false;
    s_context.run_id = run_id;
    s_context.arm_id = arm_id;
    s_context.sequence = increment_nonzero(s_context.sequence);
    sequence = s_context.sequence;
    s_v2_context.run_id = run_id;
    s_v2_context.arm_id = arm_id;
    s_v2_context.sequence = increment_nonzero(s_v2_context.sequence);
    v2_sequence = s_v2_context.sequence;
    /* Do not let a sample observed between the legacy update and the clock
     * boundary qualify. UINT64_MAX remains pending until committed below. */
    s_v2_context.set_us = UINT64_MAX;
    /* Every v2 DIAG_CONTEXT, including a clear (0/0), requires one new
     * immutable RX cache record. v1 keeps its legacy immediate relabel. */
    s_v2_context.required = true;
    s_v2_context.rx_cache_sequence_floor = rx_cache_sequence_floor;
    s_v2_context.bound_session_id = 0;
    s_v2_context.bound_rx_generation = 0;
    s_v2_context.bound_uac_generation = 0;
    s_v2_context.identity_bound = false;
    /* Make a just-acknowledged context visible on the current cached sample;
     * the next periodic sample refreshes counters under the same IDs. */
    if (s_cache_ready) {
        s_cache.run_id = run_id;
        s_cache.arm_id = arm_id;
        s_cache.context_sequence = sequence;
        s_cache.context_set = run_id != 0 || arm_id != 0;
    }
    portEXIT_CRITICAL(&s_diagnostics_lock);

    /* Capture the v2 boundary after its context is visible. Avoid a clock
     * call under the short critical section; a concurrent sample sees the
     * UINT64_MAX sentinel and therefore stays pending. */
    const uint64_t v2_set_us = positive_us(esp_timer_get_time());
    portENTER_CRITICAL(&s_diagnostics_lock);
    if (s_v2_context.sequence == v2_sequence) {
        s_v2_context.set_us = v2_set_us ? v2_set_us : UINT64_MAX;
    }
    portEXIT_CRITICAL(&s_diagnostics_lock);
    const int written = snprintf(response, capacity,
        "OK DIAG_CONTEXT token=%" PRIu32 " run=%" PRIu32
        " arm=%" PRIu32 " ctx=%" PRIu32 "\n",
        token, run_id, arm_id, sequence);
    if (written < 0 || (size_t)written >= capacity) {
        response_error(response, capacity, "response_capacity");
    }
    return true;
}

static bool send_cached_part(uint32_t token, unsigned part, bool version2,
                             char *response, size_t capacity)
{
    const char *command = version2 ? "DONGLE_TELEMETRY_V2" : "DONGLE_TELEMETRY";
    dongle_diagnostics_snapshot_t snapshot={0};
    diagnostics_v2_context_t v2_context;
    bool ready;
    bool new_lease=false,mismatch=false;
    const uint64_t query_us=positive_us(esp_timer_get_time());
    portENTER_CRITICAL(&s_diagnostics_lock);
    diagnostics_snapshot_lease_t *lease=&s_snapshot_leases[version2?1U:0U];
    if(lease->valid && (query_us<lease->started_us ||
       query_us-lease->started_us>=DONGLE_DIAGNOSTICS_LEASE_US))lease->valid=false;
    if(part==0U){
        if(lease->valid && lease->token==token){
            ready=true;snapshot=lease->snapshot;
        }else{
            ready=s_cache_ready;
            if(ready){snapshot=s_cache;new_lease=true;}
        }
    }else if(!lease->valid || lease->token!=token){
        ready=false;mismatch=true;
    }else{
        ready=true;snapshot=lease->snapshot;
    }
    v2_context = s_v2_context;
    portEXIT_CRITICAL(&s_diagnostics_lock);
    if(mismatch){
        const int written=snprintf(response,capacity,
            "ERR %s token=%" PRIu32 " snapshot_lease_mismatch\n",command,token);
        if(written<0 || (size_t)written>=capacity)
            response_error_for(response,capacity,command,"response_capacity");
        return true;
    }
    if (!ready) {
        const int written = snprintf(response, capacity,
            "ERR %s token=%" PRIu32 " not_ready\n", command, token);
        if (written < 0 || (size_t)written >= capacity) {
            response_error_for(response, capacity, command, "response_capacity");
        }
        return true;
    }

    /* v1 deliberately keeps its legacy immediate relabel semantics. v2 first
     * binds a post-context immutable RX record to its session/generation IDs,
     * then refuses a later runtime/UAC join that no longer matches that bind. */
    if (version2 &&
        (snapshot.v2_context_sequence != v2_context.sequence ||
         snapshot.v2_run_id != v2_context.run_id ||
         snapshot.v2_arm_id != v2_context.arm_id)) {
        const int written = snprintf(response, capacity,
            "ERR DONGLE_TELEMETRY_V2 token=%" PRIu32 " context_pending\n", token);
        if (written < 0 || (size_t)written >= capacity) {
            response_error_for(response, capacity, command, "response_capacity");
        }
        return true;
    }
    if (version2 && snapshot.v2_identity_mixed) {
        const int written = snprintf(response, capacity,
            "ERR DONGLE_TELEMETRY_V2 token=%" PRIu32 " mixed_identity\n", token);
        if (written < 0 || (size_t)written >= capacity) {
            response_error_for(response, capacity, command, "response_capacity");
        }
        return true;
    }
    if (version2 && v2_context.required &&
        (!snapshot.v2_context_ready || !v2_context.identity_bound ||
         snapshot.v2_rx_cache_sequence == v2_context.rx_cache_sequence_floor ||
         snapshot.v2_rx_session_id != v2_context.bound_session_id ||
         snapshot.v2_rx_session_generation != v2_context.bound_rx_generation ||
         snapshot.v2_uac_session_generation != v2_context.bound_uac_generation)) {
        const int written = snprintf(response, capacity,
            "ERR DONGLE_TELEMETRY_V2 token=%" PRIu32 " context_pending\n", token);
        if (written < 0 || (size_t)written >= capacity) {
            response_error_for(response, capacity, command, "response_capacity");
        }
        return true;
    }
    if (version2 && !snapshot.v2_identity_ready) {
        const int written = snprintf(response, capacity,
            "ERR DONGLE_TELEMETRY_V2 token=%" PRIu32 " identity_pending\n", token);
        if (written < 0 || (size_t)written >= capacity) {
            response_error_for(response, capacity, command, "response_capacity");
        }
        return true;
    }

    if(new_lease){
        /* Publish only a fully admitted part zero. A pending v2 reply must
         * not freeze an old cache and prevent a later retry from becoming ready. */
        portENTER_CRITICAL(&s_diagnostics_lock);
        const bool context_changed=s_context.sequence!=snapshot.context_sequence ||
            s_v2_context.sequence!=v2_context.sequence;
        if(!context_changed){
            lease=&s_snapshot_leases[version2?1U:0U];
            lease->valid=true;lease->token=token;lease->started_us=query_us;
            lease->snapshot=snapshot;
        }
        portEXIT_CRITICAL(&s_diagnostics_lock);
        if(context_changed){
            const int written=snprintf(response,capacity,
                "ERR %s token=%" PRIu32 " context_pending\n",command,token);
            if(written<0 || (size_t)written>=capacity)
                response_error_for(response,capacity,command,"response_capacity");
            return true;
        }
    }

    char json[DONGLE_DIAGNOSTICS_JSON_BUFFER_BYTES];
    const size_t formatted = version2 ?
        dongle_diagnostics_format_v2_part(&snapshot, part, json, sizeof(json)) :
        dongle_diagnostics_format_part(&snapshot, part, json, sizeof(json));
    if (!formatted) {
        response_error_for(response, capacity, command, "format");
        return true;
    }
    const int written = snprintf(response, capacity,
        "OK %s token=%" PRIu32 " %s\n", command, token, json);
    if (written < 0 || (size_t)written >= capacity) {
        response_error_for(response, capacity, command, "response_capacity");
    }
    return true;
}

bool dongle_diagnostics_handle_command(const char *line,
                                       char *response,
                                       size_t capacity,
                                       void *ctx)
{
    (void)ctx;
    if (!line || !response || !capacity) return false;

    static const char context_prefix[] = "DIAG_CONTEXT";
    static const char telemetry_v2_prefix[] = "DONGLE_TELEMETRY_V2";
    static const char telemetry_prefix[] = "DONGLE_TELEMETRY";
    if (strncmp(line, context_prefix, sizeof(context_prefix) - 1U) == 0) {
        const char *arguments = line + sizeof(context_prefix) - 1U;
        uint32_t values[3] = {0};
        if ((*arguments != ' ' && *arguments != '\t') ||
            !parse_numbers(arguments, values, 3)) {
            const int written = snprintf(response, capacity,
                                         "ERR DIAG_CONTEXT arguments\n");
            if (written < 0 || (size_t)written >= capacity) response[0] = '\0';
            return true;
        }
        return set_context(values[0], values[1], values[2], response, capacity);
    }
    /* Check v2 before the legacy prefix because the latter is a strict prefix
     * of this opt-in command. v1 remains byte-for-byte compatible. */
    if (strncmp(line, telemetry_v2_prefix, sizeof(telemetry_v2_prefix) - 1U) == 0) {
        const char *arguments = line + sizeof(telemetry_v2_prefix) - 1U;
        uint32_t values[3] = {0};
        if ((*arguments != ' ' && *arguments != '\t') ||
            !parse_numbers(arguments, values, 2) ||
            values[1] >= DONGLE_DIAGNOSTICS_V2_PARTS) {
            response_error_for(response, capacity, "DONGLE_TELEMETRY_V2", "arguments");
            return true;
        }
        return send_cached_part(values[0], values[1], true, response, capacity);
    }
    if (strncmp(line, telemetry_prefix, sizeof(telemetry_prefix) - 1U) == 0) {
        const char *arguments = line + sizeof(telemetry_prefix) - 1U;
        uint32_t values[3] = {0};
        if ((*arguments != ' ' && *arguments != '\t') ||
            !parse_numbers(arguments, values, 2) ||
            values[1] >= DONGLE_DIAGNOSTICS_PARTS) {
            response_error(response, capacity, "arguments");
            return true;
        }
        return send_cached_part(values[0], values[1], false, response, capacity);
    }
    return false;
}
