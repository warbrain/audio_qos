#include "dongle_diagnostics_format.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static size_t finish(char *buffer, size_t capacity, int written)
{
    if (written <= 0 || (size_t)written >= capacity ||
        (size_t)written > DONGLE_DIAGNOSTICS_JSON_MAX_BYTES) {
        if (capacity) buffer[0] = '\0';
        return 0;
    }
    return (size_t)written;
}

/* Keep this exact prefix in every part. Hosts may reject a mixed multipart
 * sample by comparing b/q/run/arm/cx/s before calculating a delta. */
#define DIAG_COMMON \
    "{\"e\":\"dongle.diag\",\"v\":%u,\"p\":%u,\"n\":%u,\"b\":\"%08" PRIx32 \
    "\",\"r\":%u,\"q\":%" PRIu32 \
    ",\"run\":%" PRIu32 \
    ",\"arm\":%" PRIu32 \
    ",\"cx\":%" PRIu32 \
    ",\"s\":%" PRIu32 \
    ",\"t\":\"%" PRIu64 "\","

#define DIAG_ARGS(snapshot, schema, part_count, part) \
    (schema), (part), (part_count), (snapshot)->boot_id, (unsigned)(snapshot)->role, \
    (snapshot)->cache_sequence, run_id, arm_id, context_sequence, \
    (snapshot)->session_id, (snapshot)->sampled_us

static size_t format_part(const dongle_diagnostics_snapshot_t *snapshot,
                          unsigned schema,
                          unsigned part_count,
                          unsigned part,
                          char *buffer,
                          size_t capacity)
{
    if (!buffer || !capacity) return 0;
    buffer[0] = '\0';
    if (!snapshot || part >= part_count || !snapshot->boot_id) return 0;

    const bool version2 = schema == DONGLE_DIAGNOSTICS_V2_SCHEMA;
    const uint32_t run_id = version2 ? snapshot->v2_run_id : snapshot->run_id;
    const uint32_t arm_id = version2 ? snapshot->v2_arm_id : snapshot->arm_id;
    const uint32_t context_sequence = version2 ? snapshot->v2_context_sequence :
                                                  snapshot->context_sequence;
    const bool context_set = version2 ? snapshot->v2_context_set : snapshot->context_set;

    char codec[6];
    if (snapshot->codec == UINT8_MAX) {
        memcpy(codec, "null", sizeof("null"));
    } else {
        (void)snprintf(codec, sizeof(codec), "%u", (unsigned)snapshot->codec);
    }

    int written;
    switch (part) {
    case 0:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"co\":%s,\"m\":%u,\"cs\":%u,\"rr\":%u,\"cr\":%u,\"rs\":%" PRIu32
            ",\"rt\":\"%" PRIu64 "\"}",
            DIAG_ARGS(snapshot, schema, part_count, part), codec,
            snapshot->uac_timeline_enabled ? 1U : 0U,
            context_set ? 1U : 0U,
            snapshot->runtime_ready ? 1U : 0U,
            snapshot->resource_cache_ready ? 1U : 0U,
            snapshot->resource_sequence, snapshot->resource_sampled_us);
        break;
    case 1:
        if (schema == DONGLE_DIAGNOSTICS_V2_SCHEMA) {
            written = snprintf(buffer, capacity, DIAG_COMMON
                "\"pe\":%" PRIu32 ",\"pd\":%" PRIu32 ",\"pp\":%" PRIu32 ",\"pr\":%u"
                ",\"cc\":%u,\"cd\":%u,\"crx\":%" PRIu32 ",\"cqo\":%" PRIu32
                ",\"crd\":%" PRIu32 ",\"ctq\":\"%" PRIu64 "\",\"cti\":%" PRIu32 "}",
                DIAG_ARGS(snapshot, schema, part_count, part),
                snapshot->perf_window_sequence[0], snapshot->perf_window_sequence[2],
                snapshot->perf_window_sequence[3], (unsigned)snapshot->perf_ready_mask,
                snapshot->cdc_connected ? 1U : 0U, snapshot->cdc_dtr ? 1U : 0U,
                snapshot->cdc_rx_commands, snapshot->cdc_rx_queue_overflows,
                snapshot->cdc_response_drops, snapshot->cdc_response_tx_queued_bytes,
                snapshot->cdc_response_tx_incomplete);
        } else {
            written = snprintf(buffer, capacity, DIAG_COMMON
                "\"pe\":%" PRIu32 ",\"pd\":%" PRIu32 ",\"pp\":%" PRIu32 ",\"pr\":%u}",
                DIAG_ARGS(snapshot, schema, part_count, part),
                snapshot->perf_window_sequence[0], snapshot->perf_window_sequence[2],
                snapshot->perf_window_sequence[3], (unsigned)snapshot->perf_ready_mask);
        }
        break;
    case 2:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"a\":%u,\"ac\":%" PRIu32 ",\"of\":%" PRIu32 ",\"lo\":%" PRIu32
            ",\"ga\":%" PRIu32 "}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->rx_active ? 1U : 0U,
            snapshot->rx_accepted, snapshot->rx_output_frames,
            snapshot->rx_permanent_loss, snapshot->rx_gap_events);
        break;
    case 3:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"rv\":%" PRIu32 ",\"du\":%" PRIu32 ",\"la\":%" PRIu32 ",\"re\":%" PRIu32
            ",\"ab\":%" PRIu32 "}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->rx_recovered, snapshot->rx_duplicates,
            snapshot->rx_late, snapshot->rx_rejected, snapshot->rx_abort_count);
        break;
    case 4:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"nr\":%" PRIu32 ",\"no\":%" PRIu32 ",\"nc\":%" PRIu32 ",\"ne\":%" PRIu32
            ",\"nj\":%" PRIu32 ",\"sy\":%" PRIu32 "}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->rx_nack_requests,
            snapshot->rx_nack_accepted, snapshot->rx_nack_cancelled,
            snapshot->rx_nack_expired, snapshot->rx_nack_rejected,
            snapshot->rx_resync_requests);
        break;
    case 5:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"fd\":%" PRIu32 ",\"mk\":%" PRIu32 ",\"rd\":%" PRIu32 ",\"ce\":%" PRIu32
            ",\"de\":%" PRIu32 ",\"iv\":%" PRIu32 "}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->rx_delivered_decoded,
            snapshot->rx_masked_frames, snapshot->rx_recovered_decoded,
            snapshot->rx_crc_failures, snapshot->rx_decode_failures,
            snapshot->rx_invalid_frames);
        break;
    case 6:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"fq\":%" PRIu32 ",\"nq\":%" PRIu32 ",\"pm\":%" PRIu32 ",\"dm\":%" PRIu32
            ",\"pa\":%" PRIu32 "}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->rx_feedback_queue_failures,
            snapshot->rx_nack_queued, snapshot->rx_poll_max_us,
            snapshot->rx_decode_max_us, snapshot->rx_poll_audio_max);
        break;
    case 7:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"ua\":%u,\"up\":%u,\"uf\":%" PRIu32 ",\"ul\":%" PRIu32
            ",\"uo\":%" PRIu32 ",\"ut\":%" PRIu32 "}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->uac_rx_active ? 1U : 0U,
            snapshot->uac_start_pending ? 1U : 0U, snapshot->uac_frames_in,
            snapshot->uac_sequence_lost, snapshot->uac_sequence_reordered,
            snapshot->uac_timeouts);
        break;
    case 8:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"us\":%" PRIu32 ",\"ub\":%" PRIu32 ",\"uov\":%" PRIu32
            ",\"uld\":%" PRIu32 ",\"ulp\":%" PRIu32 "}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->uac_silence_fills,
            snapshot->uac_buffer_samples, snapshot->uac_overflow_drops,
            snapshot->uac_level_drop_samples, snapshot->uac_level_dup_samples);
        break;
    case 9:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"te\":%u,\"ts\":%u,\"tc\":\"%" PRIu64 "\",\"tl\":\"%" PRIu64
            "\",\"tr\":\"%" PRIu64 "\"}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->uac_timeline_enabled ? 1U : 0U,
            snapshot->uac_timeline_started ? 1U : 0U,
            snapshot->uac_timeline_cursor_samples,
            snapshot->uac_timeline_late_samples,
            snapshot->uac_timeline_local_retired_samples);
        break;
    case 10:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"tk\":\"%" PRIu64 "\",\"tp\":%" PRIu32 ",\"tg\":%" PRIu32
            ",\"tb\":\"%" PRIu64 "\",\"to\":\"%" PRIu64 "\"}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->uac_timeline_resume_skipped_samples,
            snapshot->uac_timeline_pause_count, snapshot->uac_timeline_max_gap_us,
            snapshot->uac_timeline_start_us, snapshot->uac_timeline_first_output_us);
        break;
    case 11:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"lk\":%u,\"ch\":%u,\"cst\":%u,\"cp\":%u,\"rq\":%" PRIu32
            ",\"ri\":%" PRIu32 ",\"ta\":%" PRIu32 ",\"tcb\":%" PRIu32
            ",\"tf\":%" PRIu32 "}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->link_connected ? 1U : 0U,
            (unsigned)snapshot->channel, (unsigned)snapshot->channel_state,
            snapshot->channel_pending ? 1U : 0U, snapshot->rx_queue_drops,
            snapshot->rx_invalid_lengths, snapshot->tx_accepted,
            snapshot->tx_callbacks, snapshot->tx_immediate_failures);
        break;
    case 12:
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"xr\":%u,\"xq\":%" PRIu32 ",\"xd\":%" PRIu32 ",\"xe\":%" PRIu32
            ",\"sr\":%" PRIu32 ",\"st\":%" PRIu32 ",\"sl\":%" PRIu32
            ",\"xs\":%" PRIu32 ",\"xl\":%" PRIu32 ",\"xh\":%" PRIu32
            ",\"xf\":%" PRIu32 "}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->rx_cache_ready ? 1U : 0U,
            snapshot->rx_cache_sequence, snapshot->rx_snapshot_drops,
            snapshot->rx_event_drops, snapshot->rx_stack_min_bytes,
            snapshot->tx_stack_min_bytes, snapshot->link_stack_min_bytes,
            snapshot->tx_soft_timeouts, snapshot->tx_late_callbacks,
            snapshot->tx_hard_recoveries, snapshot->tx_recovery_failures);
        break;
    case 13: {
        if (schema != DONGLE_DIAGNOSTICS_V2_SCHEMA) return 0;
        char sampled[24] = "null";
        char age[24] = "null";
        char published[24] = "null";
        char owner_stack[16] = "null";
        char telemetry_stack[16] = "null";
        if (snapshot->rx_owner_timing_ready) {
            (void)snprintf(sampled, sizeof(sampled), "\"%" PRIu64 "\"",
                           snapshot->rx_owner_sampled_us);
            (void)snprintf(age, sizeof(age), "\"%" PRIu64 "\"",
                           snapshot->rx_owner_age_us);
            (void)snprintf(published, sizeof(published), "\"%" PRIu64 "\"",
                           snapshot->rx_cache_published_us);
            (void)snprintf(owner_stack, sizeof(owner_stack), "%" PRIu32,
                           snapshot->rx_owner_stack_min_bytes);
            (void)snprintf(telemetry_stack, sizeof(telemetry_stack), "%" PRIu32,
                           snapshot->rx_telemetry_stack_min_bytes);
        }
        written = snprintf(buffer, capacity, DIAG_COMMON
            "\"xr\":%u,\"xq\":%" PRIu32 ",\"rxt\":%s,\"rxa\":%s,\"rxc\":%s"
            ",\"rxs\":%s,\"rxw\":%s,\"dgw\":%" PRIu32 "}",
            DIAG_ARGS(snapshot, schema, part_count, part), snapshot->rx_cache_ready ? 1U : 0U,
            snapshot->rx_cache_sequence, sampled, age, published, owner_stack,
            telemetry_stack, snapshot->diagnostics_stack_min_bytes);
        break;
    }
    default:
        return 0;
    }
    return finish(buffer, capacity, written);
}

size_t dongle_diagnostics_format_part(const dongle_diagnostics_snapshot_t *snapshot,
                                      unsigned part,
                                      char *buffer,
                                      size_t capacity)
{
    return format_part(snapshot, DONGLE_DIAGNOSTICS_SCHEMA,
                       DONGLE_DIAGNOSTICS_PARTS, part, buffer, capacity);
}

size_t dongle_diagnostics_format_v2_part(const dongle_diagnostics_snapshot_t *snapshot,
                                         unsigned part,
                                         char *buffer,
                                         size_t capacity)
{
    return format_part(snapshot, DONGLE_DIAGNOSTICS_V2_SCHEMA,
                       DONGLE_DIAGNOSTICS_V2_PARTS, part, buffer, capacity);
}

size_t dongle_diagnostics_cdc_log_format_part(const dongle_diagnostics_snapshot_t *snapshot,
                                              unsigned part, char *buffer, size_t capacity)
{
    if(!buffer || !capacity)return 0;
    buffer[0]='\0';
    if(!snapshot || !snapshot->boot_id || part>=DONGLE_CDC_LOG_HEALTH_PARTS)return 0;
    int prefix=snprintf(buffer,capacity,
        "{\"event\":\"audio.cdc_log_health\",\"schema\":1,\"boot_id\":\"%08" PRIx32
        "\",\"role\":%u,\"seq\":%" PRIu32 ",\"part\":%u",
        snapshot->boot_id,(unsigned)snapshot->role,snapshot->cache_sequence,part);
    if(prefix<0 || (size_t)prefix>=capacity){buffer[0]='\0';return 0;}
    char *out=buffer+prefix;
    const size_t left=capacity-(size_t)prefix;
    int fields=-1;
    switch(part){
    case 0:fields=snprintf(out,left,",\"part_count\":6,\"sampled_us\":\"%" PRIu64 "\"}",snapshot->sampled_us);break;
    case 1:fields=snprintf(out,left,",\"attempted\":%" PRIu32 "}",snapshot->cdc_log_attempted);break;
    case 2:fields=snprintf(out,left,",\"enqueued\":%" PRIu32 "}",snapshot->cdc_log_enqueued);break;
    case 3:fields=snprintf(out,left,",\"dropped\":%" PRIu32 "}",snapshot->cdc_log_dropped);break;
    case 4:fields=snprintf(out,left,",\"truncated\":%" PRIu32 "}",snapshot->cdc_log_truncated);break;
    case 5:fields=snprintf(out,left,",\"write_failed\":%" PRIu32 "}",snapshot->cdc_log_write_failed);break;
    }
    if(fields<0 || (size_t)fields>=left || (size_t)prefix+(size_t)fields>180U){buffer[0]='\0';return 0;}
    return (size_t)prefix+(size_t)fields;
}
