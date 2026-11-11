#include "scepter_diagnostics.h"

#include "scepter_diagnostics_format.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dual_end_link_config.h"
#include "dual_end_task_policy.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mic_audio.h"

#define SCEPTER_DIAGNOSTICS_TASK_STACK_BYTES 3072U
#define SCEPTER_DIAGNOSTICS_TASK_PRIORITY 1U
#define SCEPTER_DIAGNOSTICS_SAMPLE_MS 250U

typedef struct {
    uint32_t run_id;
    uint32_t arm_id;
    uint32_t sequence;
    uint64_t set_us;
    /* A context ACK must not relabel the most recently published immutable
     * window.  This is sampled from mic_audio before the ACK is returned and
     * forms a cache boundary in addition to the owner close timestamp. */
    uint32_t cache_sequence_floor;
    uint32_t bound_boot_id;
    uint32_t bound_session_id;
    uint32_t bound_capture_generation;
    uint8_t bound_codec;
    uint8_t bound_mode;
    uint16_t bound_generation;
    bool required;
    bool bound_codec_known;
    bool identity_bound;
} scepter_diagnostics_context_t;

/* The cache worker is the only writer.  It copies a completed owner window
 * once per immutable sequence; no CDC command ever reads a live queue,
 * console report-ready bit, DSP state, encoder, or ESP-NOW state. */
static portMUX_TYPE s_diagnostics_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_diagnostics_task;
static scepter_diagnostics_context_t s_context;
static mic_audio_diagnostics_snapshot_t s_cache;
static bool s_cache_ready;
/* A context ACK never relabels an older audio window.  These fields bind a
 * source cache only after its owner-side close timestamp is at or after the
 * acknowledged context instant. */
static uint32_t s_cache_context_sequence;
static bool s_cache_context_ready;

static uint32_t increment_nonzero(uint32_t value)
{
    ++value;
    return value ? value : 1U;
}

static bool context_identity_matches(
    const scepter_diagnostics_context_t *context,
    const mic_audio_diagnostics_snapshot_t *snapshot)
{
    if (!context || !snapshot || !context->identity_bound) return false;
    const mic_pipeline_telemetry_t *pipeline = &snapshot->pipeline;
    return pipeline->boot_id == context->bound_boot_id &&
        pipeline->session_id == context->bound_session_id &&
        pipeline->capture_generation == context->bound_capture_generation &&
        pipeline->codec == context->bound_codec &&
        pipeline->codec_known == context->bound_codec_known &&
        pipeline->mode == context->bound_mode &&
        pipeline->generation == context->bound_generation;
}

static void context_bind_identity(scepter_diagnostics_context_t *context,
                                  const mic_audio_diagnostics_snapshot_t *snapshot)
{
    const mic_pipeline_telemetry_t *pipeline = &snapshot->pipeline;
    context->bound_boot_id = pipeline->boot_id;
    context->bound_session_id = pipeline->session_id;
    context->bound_capture_generation = pipeline->capture_generation;
    context->bound_codec = pipeline->codec;
    context->bound_codec_known = pipeline->codec_known;
    context->bound_mode = pipeline->mode;
    context->bound_generation = pipeline->generation;
    context->identity_bound = true;
}

/* Called only with s_diagnostics_lock held.  Keep the acceptance decision in
 * one place because a cache can arrive while DIAG_CONTEXT is being published:
 * it sees the provisional UINT64_MAX boundary first, then must be checked
 * again against the final immutable boundary before its sequence changes.
 * Nothing is accepted while the boundary is provisional, even for a saturated
 * timestamp, and all identity fields must still match the first eligible
 * cache. */
static void context_evaluate_cache_locked(
    const mic_audio_diagnostics_snapshot_t *snapshot)
{
    s_cache_context_sequence = 0;
    s_cache_context_ready = false;
    if (!snapshot || !snapshot->ready || !snapshot->cache_sequence) return;

    if (!s_context.required) {
        s_cache_context_sequence = s_context.sequence;
        s_cache_context_ready = true;
        return;
    }

    const bool context_finalized = s_context.set_us != UINT64_MAX;
    const bool context_fresh_cache = context_finalized &&
        snapshot->cache_sequence != s_context.cache_sequence_floor &&
        snapshot->pipeline.sampled_us >= s_context.set_us;
    if (!context_fresh_cache) return;

    if (!s_context.identity_bound) {
        context_bind_identity(&s_context, snapshot);
    }
    if (context_identity_matches(&s_context, snapshot)) {
        s_cache_context_sequence = s_context.sequence;
        s_cache_context_ready = true;
    }
}

static void diagnostics_sample(void)
{
    mic_audio_diagnostics_snapshot_t next = {0};
    if (!mic_audio_copy_diagnostics_snapshot(&next) ||
        !next.ready || !next.cache_sequence || !next.pipeline.boot_id) {
        return;
    }

    /* Never mutate a cache sequence after publication.  A multipart host
     * read either sees all parts of this immutable snapshot or sees a newer
     * cache value and can reject the mixed group by cache/session/mode IDs. */
    portENTER_CRITICAL(&s_diagnostics_lock);
    if (!s_cache_ready || next.cache_sequence != s_cache.cache_sequence) {
        s_cache = next;
        s_cache_ready = true;
        /* A nonzero DIAG_CONTEXT becomes usable only from a new immutable
         * owner window.  Timestamp alone is insufficient: the sampled source
         * cache can have closed before the command while the low-priority
         * mirror lags.  The cache floor prevents that older window from being
         * relabelled, then the first eligible window binds its full pipeline
         * identity.  A later identity change fails closed rather than joining
         * unrelated multipart samples under one run/arm. */
        context_evaluate_cache_locked(&next);
    }
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

static void diagnostics_task(void *arg)
{
    (void)arg;
    for (;;) {
        diagnostics_sample();
        vTaskDelay(pdMS_TO_TICKS(SCEPTER_DIAGNOSTICS_SAMPLE_MS));
    }
}

esp_err_t scepter_diagnostics_start(void)
{
    if (s_diagnostics_task) return ESP_OK;
    const BaseType_t created = xTaskCreatePinnedToCore(
        diagnostics_task, "scepter_diag", SCEPTER_DIAGNOSTICS_TASK_STACK_BYTES,
        NULL, SCEPTER_DIAGNOSTICS_TASK_PRIORITY, &s_diagnostics_task,
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
        if (i + 1U < count && *text != ' ' && *text != '\t') return false;
    }
    while (*text == ' ' || *text == '\t') ++text;
    return *text == '\0' && values[0] != 0;
}

static void response_error(char *response, size_t capacity, const char *detail)
{
    if (!response || !capacity) return;
    const int written = snprintf(response, capacity,
                                 "ERR SCEPTER_TELEMETRY %s\n", detail);
    if (written < 0 || (size_t)written >= capacity) response[0] = '\0';
}

static bool set_context(uint32_t token, uint32_t run_id, uint32_t arm_id,
                        char *response, size_t capacity)
{
    uint32_t sequence;
    /* Publish an impossible timestamp while the context boundary is being
     * assembled.  The cache worker can only leave it pending, never accept a
     * partially initialized context. */
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_context.run_id = run_id;
    s_context.arm_id = arm_id;
    s_context.sequence = increment_nonzero(s_context.sequence);
    s_context.set_us = UINT64_MAX;
    s_context.cache_sequence_floor = 0;
    s_context.identity_bound = false;
    s_context.required = run_id != 0 || arm_id != 0;
    sequence = s_context.sequence;
    s_cache_context_sequence = 0;
    s_cache_context_ready = false;
    portEXIT_CRITICAL(&s_diagnostics_lock);

    /* Linearize the guard after the context is visible.  Reading the source
     * cache, instead of the 250 ms mirror, makes the floor cover a completed
     * window even when the diagnostics worker has not copied it yet.  A new
     * cache observed between this read and the commit is still safe: it must
     * carry an owner close timestamp at/after this boundary to qualify. */
    const uint64_t set_us = (uint64_t)esp_timer_get_time();
    mic_audio_diagnostics_snapshot_t latest = {0};
    const bool latest_ready = mic_audio_copy_diagnostics_snapshot(&latest);
    const uint32_t cache_sequence_floor = latest_ready && latest.ready ?
        latest.cache_sequence : 0;
    portENTER_CRITICAL(&s_diagnostics_lock);
    if (s_context.sequence == sequence) {
        s_context.set_us = set_us;
        s_context.cache_sequence_floor = cache_sequence_floor;
        /* A new immutable cache can have been mirrored while this context was
         * intentionally provisional.  Re-evaluate that exact cache after the
         * final boundary commits; requiring sequence/floor/time/identity here
         * prevents stale or mixed attribution while avoiding an unnecessary
         * wait for cache N+1. */
        if (s_cache_ready) {
            context_evaluate_cache_locked(&s_cache);
        }
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

static bool send_cached_part(uint32_t token, unsigned kind, unsigned part,
                             char *response, size_t capacity)
{
    mic_audio_diagnostics_snapshot_t snapshot = {0};
    scepter_diagnostics_context_t context = {0};
    bool ready = false;
    bool context_ready = false;
    portENTER_CRITICAL(&s_diagnostics_lock);
    ready = s_cache_ready;
    if (ready) snapshot = s_cache;
    context = s_context;
    context_ready = s_cache_context_ready &&
        s_cache_context_sequence == context.sequence &&
        (!context.required ||
         (context.identity_bound && context_identity_matches(&context, &snapshot)));
    portEXIT_CRITICAL(&s_diagnostics_lock);
    if (!ready) {
        const int written = snprintf(response, capacity,
            "ERR SCEPTER_TELEMETRY token=%" PRIu32 " not_ready\n", token);
        if (written < 0 || (size_t)written >= capacity) {
            response_error(response, capacity, "response_capacity");
        }
        return true;
    }
    if (context.required && !context_ready) {
        const int written = snprintf(response, capacity,
            "ERR SCEPTER_TELEMETRY token=%" PRIu32 " context_pending\n", token);
        if (written < 0 || (size_t)written >= capacity) {
            response_error(response, capacity, "response_capacity");
        }
        return true;
    }
    if ((kind == SCEPTER_DIAGNOSTICS_KIND_SPATIAL ||
         kind == SCEPTER_DIAGNOSTICS_KIND_WIENER) && !snapshot.spatial_ready) {
        const int written = snprintf(response, capacity,
            "ERR SCEPTER_TELEMETRY token=%" PRIu32 " spatial_identity\n", token);
        if (written < 0 || (size_t)written >= capacity) {
            response_error(response, capacity, "response_capacity");
        }
        return true;
    }

    char json[SCEPTER_DIAGNOSTICS_JSON_BUFFER_BYTES];
    const char *event_name = NULL;
    unsigned schema = 0;
    bool active = false;
    if (!scepter_diagnostics_format_part(&snapshot, kind, part, json,
                                         sizeof(json), &event_name, &schema,
                                         &active) || !event_name || !schema) {
        response_error(response, capacity, "format");
        return true;
    }

    char codec[6];
    if (snapshot.pipeline.codec_known) {
        (void)snprintf(codec, sizeof(codec), "%u",
                       (unsigned)snapshot.pipeline.codec);
    } else {
        memcpy(codec, "none", sizeof("none"));
    }
    const int written = snprintf(response, capacity,
        "OK SCEPTER_TELEMETRY token=%" PRIu32
        " kind=%u event=%s schema=%u run=%" PRIu32 " arm=%" PRIu32
        " ctx=%" PRIu32 " cache=%" PRIu32 " boot=%08" PRIx32
        " session=%" PRIu32 " codec=%s mode=%u generation=%u"
        " capture_generation=%" PRIu32 " active=%u %s\n",
        token, kind, event_name, schema, context.run_id, context.arm_id,
        context.sequence, snapshot.cache_sequence, snapshot.pipeline.boot_id,
        snapshot.pipeline.session_id, codec, (unsigned)snapshot.pipeline.mode,
        (unsigned)snapshot.pipeline.generation,
        snapshot.pipeline.capture_generation, active ? 1U : 0U, json);
    if (written < 0 || (size_t)written >= capacity) {
        response_error(response, capacity, "response_capacity");
    }
    return true;
}

bool scepter_diagnostics_handle_command(const char *line,
                                        char *response,
                                        size_t capacity,
                                        void *ctx)
{
    (void)ctx;
    if (!line || !response || !capacity) return false;

    static const char context_prefix[] = "DIAG_CONTEXT";
    static const char telemetry_prefix[] = "SCEPTER_TELEMETRY";
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
    if (strncmp(line, telemetry_prefix, sizeof(telemetry_prefix) - 1U) == 0) {
        const char *arguments = line + sizeof(telemetry_prefix) - 1U;
        uint32_t values[3] = {0};
        if ((*arguments != ' ' && *arguments != '\t') ||
            !parse_numbers(arguments, values, 3) ||
            values[1] >= SCEPTER_DIAGNOSTICS_KIND_COUNT ||
            values[2] >= scepter_diagnostics_part_count(values[1])) {
            response_error(response, capacity, "arguments");
            return true;
        }
        return send_cached_part(values[0], values[1], values[2], response, capacity);
    }
    return false;
}
