#include "mic_pipeline_telemetry.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

/* The detailed events intentionally omit spatial generation: consumers join
 * them to audio.pipeline on boot_id/role/session_id/seq, which carries g and
 * capture generation without repeating another long identifier in every part. */
static int observability_prefix(char *out, size_t capacity, const char *event,
                                const mic_pipeline_telemetry_t *record, unsigned part)
{
    char codec[8];
    if (record->codec_known) snprintf(codec, sizeof(codec), "%u", (unsigned)record->codec);
    else snprintf(codec, sizeof(codec), "null");
    return snprintf(out, capacity,
        "{\"event\":\"%s\",\"schema\":1,\"boot_id\":\"%08" PRIx32
        "\",\"role\":0,\"session_id\":%" PRIu32 ",\"seq\":%" PRIu32
        ",\"part\":%u,\"c\":%s,\"m\":%u",
        event, record->boot_id, record->session_id, record->sequence, part, codec,
        (unsigned)record->mode);
}

static size_t observability_finish_with_limit(char *buffer, size_t capacity,
                                              size_t line_limit, int prefix, int fields)
{
    if (prefix < 0 || (size_t)prefix >= capacity || fields < 0 ||
        (size_t)fields >= capacity - (size_t)prefix ||
        (size_t)prefix + (size_t)fields >= line_limit) {
        if (buffer && capacity) buffer[0] = '\0';
        return 0;
    }
    return (size_t)prefix + (size_t)fields;
}

static size_t observability_finish(char *buffer, size_t capacity, int prefix, int fields)
{
    return observability_finish_with_limit(buffer, capacity,
                                           MIC_OBSERVABILITY_TELEMETRY_BYTES,
                                           prefix, fields);
}

static bool observability_record_valid(const mic_pipeline_telemetry_t *record)
{
    return record && record->boot_id && record->mode <= MIC_PIPELINE_SPATIAL_MAX_MODE &&
        (!record->codec_known || record->codec == 3U || record->codec == 4U || record->codec == 5U);
}

static uint32_t capture_rms(const mic_capture_health_t *health, unsigned channel)
{
    if (!health->samples[channel]) return 0;
    const double rms = sqrt((double)health->sum_sq[channel] / (double)health->samples[channel]);
    return rms >= (double)UINT32_MAX ? UINT32_MAX : (uint32_t)(rms + 0.5);
}

static bool capture_dbfs(double amplitude, double *dbfs)
{
    if (!dbfs || amplitude <= 0.0) return false;
    if (amplitude > 32768.0) amplitude = 32768.0;
    *dbfs = 20.0 * log10(amplitude / 32768.0);
    return isfinite(*dbfs);
}

static bool capture_rms_dbfs(const mic_capture_health_t *health, unsigned channel,
                             double *dbfs)
{
    if (!health || channel >= MIC_CAPTURE_HEALTH_CHANNELS ||
        !health->samples[channel] || !dbfs) return false;
    const double rms = sqrt((double)health->sum_sq[channel] /
                            (double)health->samples[channel]);
    return capture_dbfs(rms, dbfs);
}

static bool capture_clip_percent(const mic_capture_health_t *health, double *percent)
{
    if (!health || !percent) return false;
    const double samples = (double)health->samples[0] + (double)health->samples[1];
    if (samples <= 0.0) return false;
    const double clipped = (double)health->clip_samples[0] +
                           (double)health->clip_samples[1];
    *percent = 100.0 * clipped / samples;
    return isfinite(*percent) && *percent >= 0.0 && *percent <= 100.0;
}

size_t mic_capture_health_telemetry_format(const mic_pipeline_telemetry_t *record,
                                            unsigned part, char *buffer, size_t capacity)
{
    if (buffer && capacity) buffer[0] = '\0';
    if (!buffer || !capacity || !observability_record_valid(record) ||
        part >= MIC_CAPTURE_HEALTH_TELEMETRY_PARTS) return 0;
    const mic_capture_health_t *health = &record->capture_health;
    const uint32_t left_rms = capture_rms(health, 0);
    const uint32_t right_rms = capture_rms(health, 1);
    const uint32_t high = left_rms > right_rms ? left_rms : right_rms;
    const uint32_t low = left_rms < right_rms ? left_rms : right_rms;
    const uint32_t balance_q15 = high ? (uint32_t)(((uint64_t)low * 32767U + high / 2U) / high) : 0;
    const int prefix = observability_prefix(buffer, capacity, "audio.capture_health", record, part);
    if (prefix < 0 || (size_t)prefix >= capacity) { buffer[0] = '\0'; return 0; }
    char *tail = buffer + prefix;
    const size_t left_capacity = capacity - (size_t)prefix;
    int fields = -1;
    switch (part) {
    case 0: fields = snprintf(tail, left_capacity, ",\"part_count\":%u}", MIC_CAPTURE_HEALTH_TELEMETRY_PARTS); break;
    case 1: fields = snprintf(tail, left_capacity, ",\"sampled_us\":\"%" PRIu64 "\"}", record->sampled_us); break;
    case 2: fields = snprintf(tail, left_capacity, ",\"l_rms\":%" PRIu32 ",\"r_rms\":%" PRIu32 "}", left_rms, right_rms); break;
    case 3: fields = snprintf(tail, left_capacity, ",\"l_peak\":%" PRIu32 ",\"r_peak\":%" PRIu32 "}", health->peak[0], health->peak[1]); break;
    case 4: fields = snprintf(tail, left_capacity, ",\"l_zero\":\"%" PRIu64 "\"}", health->zero_samples[0]); break;
    case 5: fields = snprintf(tail, left_capacity, ",\"r_zero\":\"%" PRIu64 "\"}", health->zero_samples[1]); break;
    case 6: fields = snprintf(tail, left_capacity, ",\"l_quiet\":\"%" PRIu64 "\"}", health->quiet_samples[0]); break;
    case 7: fields = snprintf(tail, left_capacity, ",\"r_quiet\":\"%" PRIu64 "\"}", health->quiet_samples[1]); break;
    case 8: fields = snprintf(tail, left_capacity, ",\"l_clip\":\"%" PRIu64 "\"}", health->clip_samples[0]); break;
    case 9: fields = snprintf(tail, left_capacity, ",\"r_clip\":\"%" PRIu64 "\"}", health->clip_samples[1]); break;
    case 10: fields = snprintf(tail, left_capacity, ",\"l_under\":%" PRIu32 ",\"r_under\":%" PRIu32 "}", health->underlevel_blocks[0], health->underlevel_blocks[1]); break;
    case 11: fields = snprintf(tail, left_capacity, ",\"balance_q15\":%" PRIu32 ",\"blocks\":%" PRIu32 "}", balance_q15, health->blocks); break;
    case 12: fields = snprintf(tail, left_capacity, ",\"l_samples\":\"%" PRIu64 "\"}", health->samples[0]); break;
    case 13: fields = snprintf(tail, left_capacity, ",\"r_samples\":\"%" PRIu64 "\"}", health->samples[1]); break;
    case 14: {
        double left_dbfs = 0.0, right_dbfs = 0.0;
        const bool left_valid = capture_rms_dbfs(health, 0, &left_dbfs);
        const bool right_valid = capture_rms_dbfs(health, 1, &right_dbfs);
        if (left_valid && right_valid)
            fields = snprintf(tail, left_capacity,
                ",\"rms_dbfs_l\":%.1f,\"rms_dbfs_r\":%.1f}", left_dbfs, right_dbfs);
        else if (left_valid)
            fields = snprintf(tail, left_capacity,
                ",\"rms_dbfs_l\":%.1f,\"rms_dbfs_r\":null}", left_dbfs);
        else if (right_valid)
            fields = snprintf(tail, left_capacity,
                ",\"rms_dbfs_l\":null,\"rms_dbfs_r\":%.1f}", right_dbfs);
        else
            fields = snprintf(tail, left_capacity,
                ",\"rms_dbfs_l\":null,\"rms_dbfs_r\":null}");
        break;
    }
    case 15: {
        double left_dbfs = 0.0, right_dbfs = 0.0;
        const bool left_valid = capture_dbfs((double)health->peak[0], &left_dbfs);
        const bool right_valid = capture_dbfs((double)health->peak[1], &right_dbfs);
        if (left_valid && right_valid)
            fields = snprintf(tail, left_capacity,
                ",\"peak_dbfs_l\":%.1f,\"peak_dbfs_r\":%.1f}", left_dbfs, right_dbfs);
        else if (left_valid)
            fields = snprintf(tail, left_capacity,
                ",\"peak_dbfs_l\":%.1f,\"peak_dbfs_r\":null}", left_dbfs);
        else if (right_valid)
            fields = snprintf(tail, left_capacity,
                ",\"peak_dbfs_l\":null,\"peak_dbfs_r\":%.1f}", right_dbfs);
        else
            fields = snprintf(tail, left_capacity,
                ",\"peak_dbfs_l\":null,\"peak_dbfs_r\":null}");
        break;
    }
    case 16: {
        double clip_percent = 0.0;
        if (capture_clip_percent(health, &clip_percent))
            fields = snprintf(tail, left_capacity, ",\"clip_pct\":%.2f}", clip_percent);
        else
            fields = snprintf(tail, left_capacity, ",\"clip_pct\":null}");
        break;
    }
    case 17: {
        const bool left_valid = record->output_gain_l_x >= 1U && record->output_gain_l_x <= 32U;
        const bool right_valid = record->output_gain_r_x >= 1U && record->output_gain_r_x <= 32U;
        if (left_valid && right_valid)
            fields = snprintf(tail, left_capacity,
                ",\"gain_l_x\":%u,\"gain_r_x\":%u}",
                (unsigned)record->output_gain_l_x, (unsigned)record->output_gain_r_x);
        else if (left_valid)
            fields = snprintf(tail, left_capacity,
                ",\"gain_l_x\":%u,\"gain_r_x\":null}",
                (unsigned)record->output_gain_l_x);
        else if (right_valid)
            fields = snprintf(tail, left_capacity,
                ",\"gain_l_x\":null,\"gain_r_x\":%u}",
                (unsigned)record->output_gain_r_x);
        else
            fields = snprintf(tail, left_capacity,
                ",\"gain_l_x\":null,\"gain_r_x\":null}");
        break;
    }
    }
    return observability_finish(buffer, capacity, prefix, fields);
}

static int metric_total(char *tail, size_t capacity, const char *name,
                        const mic_pipeline_metric_t *metric)
{
    return snprintf(tail, capacity, ",\"%s_us\":\"%" PRIu64 "\"}", name, metric->us);
}

static int metric_count_low(char *tail, size_t capacity, const char *name,
                            const mic_pipeline_metric_t *metric)
{
    if (metric->n)
        return snprintf(tail, capacity, ",\"%s_n\":%" PRIu32 ",\"%s_lo\":%" PRIu32 "}",
                        name, metric->n, name, metric->lo);
    return snprintf(tail, capacity, ",\"%s_n\":0,\"%s_lo\":null}", name, name);
}

static int metric_high(char *tail, size_t capacity, const char *name,
                       const mic_pipeline_metric_t *metric)
{
    if (metric->n) return snprintf(tail, capacity, ",\"%s_hi\":%" PRIu32 "}", name, metric->hi);
    return snprintf(tail, capacity, ",\"%s_hi\":null}", name);
}

static int wiener_count_p50(char *tail, size_t capacity, const char *name,
                            const mic_wiener_timing_summary_t *summary)
{
    if (!summary->available)
        return snprintf(tail, capacity,
            ",\"%s_n\":0,\"%s_p50\":null}", name, name);
    if (summary->p50_overflow)
        return snprintf(tail, capacity,
            ",\"%s_n\":%" PRIu32 ",\"%s_p50\":\">=%u\"}",
            name, summary->count, name, MIC_WIENER_TIMING_MAX_US);
    return snprintf(tail, capacity,
        ",\"%s_n\":%" PRIu32 ",\"%s_p50\":%" PRIu32 "}",
        name, summary->count, name, summary->p50_us);
}

static int wiener_p95(char *tail, size_t capacity, const char *name,
                      const mic_wiener_timing_summary_t *summary)
{
    if (!summary->available)
        return snprintf(tail, capacity, ",\"%s_p95\":null}", name);
    if (summary->p95_overflow)
        return snprintf(tail, capacity, ",\"%s_p95\":\">=%u\"}",
                        name, MIC_WIENER_TIMING_MAX_US);
    return snprintf(tail, capacity, ",\"%s_p95\":%" PRIu32 "}",
                    name, summary->p95_us);
}

static int wiener_max(char *tail, size_t capacity, const char *name,
                      const mic_wiener_timing_summary_t *summary)
{
    if (!summary->available)
        return snprintf(tail, capacity, ",\"%s_max\":null}", name);
    return snprintf(tail, capacity, ",\"%s_max\":%" PRIu32 "}",
                    name, summary->max_us);
}

size_t mic_owner_timing_telemetry_format(const mic_pipeline_telemetry_t *record,
                                          unsigned part, char *buffer, size_t capacity)
{
    if (buffer && capacity) buffer[0] = '\0';
    if (!buffer || !capacity || !observability_record_valid(record) ||
        part >= MIC_OWNER_TIMING_TELEMETRY_PARTS) return 0;
    const int prefix = observability_prefix(buffer, capacity, "audio.owner_timing", record, part);
    if (prefix < 0 || (size_t)prefix >= capacity) { buffer[0] = '\0'; return 0; }
    char *tail = buffer + prefix;
    const size_t left_capacity = capacity - (size_t)prefix;
    int fields = -1;
    switch (part) {
    case 0: fields = snprintf(tail, left_capacity, ",\"part_count\":%u}", MIC_OWNER_TIMING_TELEMETRY_PARTS); break;
    case 1: fields = snprintf(tail, left_capacity, ",\"sampled_us\":\"%" PRIu64 "\"}", record->sampled_us); break;
    case 2: fields = metric_total(tail, left_capacity, "mw", &record->owner_mutex_wait); break;
    case 3: fields = metric_count_low(tail, left_capacity, "mw", &record->owner_mutex_wait); break;
    case 4: fields = metric_high(tail, left_capacity, "mw", &record->owner_mutex_wait); break;
    case 5: fields = metric_total(tail, left_capacity, "enc", &record->owner_encode); break;
    case 6: fields = metric_count_low(tail, left_capacity, "enc", &record->owner_encode); break;
    case 7: fields = metric_high(tail, left_capacity, "enc", &record->owner_encode); break;
    case 8: fields = metric_total(tail, left_capacity, "rs", &record->owner_radio_submit); break;
    case 9: fields = metric_count_low(tail, left_capacity, "rs", &record->owner_radio_submit); break;
    case 10: fields = metric_high(tail, left_capacity, "rs", &record->owner_radio_submit); break;
    case 11:
        if (record->mode == 4U)
            fields = wiener_count_p50(tail, left_capacity, "wiener_us",
                                      &record->wiener_timing[0]);
        else if (record->mode == 5U)
            fields = wiener_count_p50(tail, left_capacity, "wiener_l_us",
                                      &record->wiener_timing[0]);
        else fields = snprintf(tail, left_capacity, ",\"wiener_timing_active\":false}");
        break;
    case 12:
        if (record->mode == 4U)
            fields = wiener_p95(tail, left_capacity, "wiener_us",
                                &record->wiener_timing[0]);
        else if (record->mode == 5U)
            fields = wiener_p95(tail, left_capacity, "wiener_l_us",
                                &record->wiener_timing[0]);
        else fields = snprintf(tail, left_capacity, ",\"wiener_timing_active\":false}");
        break;
    case 13:
        if (record->mode == 4U)
            fields = wiener_max(tail, left_capacity, "wiener_us",
                                &record->wiener_timing[0]);
        else if (record->mode == 5U)
            fields = wiener_max(tail, left_capacity, "wiener_l_us",
                                &record->wiener_timing[0]);
        else fields = snprintf(tail, left_capacity, ",\"wiener_timing_active\":false}");
        break;
    case 14:
        if (record->mode == 5U)
            fields = wiener_count_p50(tail, left_capacity, "wiener_r_us",
                                      &record->wiener_timing[1]);
        else fields = snprintf(tail, left_capacity, ",\"wiener_r_timing_active\":false}");
        break;
    case 15:
        if (record->mode == 5U)
            fields = wiener_p95(tail, left_capacity, "wiener_r_us",
                                &record->wiener_timing[1]);
        else fields = snprintf(tail, left_capacity, ",\"wiener_r_timing_active\":false}");
        break;
    case 16:
        if (record->mode == 5U)
            fields = wiener_max(tail, left_capacity, "wiener_r_us",
                                &record->wiener_timing[1]);
        else fields = snprintf(tail, left_capacity, ",\"wiener_r_timing_active\":false}");
        break;
    }
    return observability_finish(buffer, capacity, prefix, fields);
}

static size_t mic_nlms_telemetry_format_impl(const mic_pipeline_telemetry_t *record,
                                              unsigned part, char *buffer,
                                              size_t capacity, bool allow_inactive)
{
    if (buffer && capacity) buffer[0] = '\0';
    if (!buffer || !capacity || !observability_record_valid(record) ||
        (!allow_inactive && !record->nlms.blocks) ||
        part >= MIC_NLMS_TELEMETRY_PARTS) return 0;
    const mic_nlms_telemetry_t *nlms = &record->nlms;
    const int prefix = observability_prefix(buffer, capacity, "audio.nlms", record, part);
    if (prefix < 0 || (size_t)prefix >= capacity) { buffer[0] = '\0'; return 0; }
    char *tail = buffer + prefix;
    const size_t left_capacity = capacity - (size_t)prefix;
    int fields = -1;
    switch (part) {
    case 0: fields = snprintf(tail, left_capacity, ",\"part_count\":%u}", MIC_NLMS_TELEMETRY_PARTS); break;
    case 1: fields = snprintf(tail, left_capacity, ",\"sampled_us\":\"%" PRIu64 "\"}", record->sampled_us); break;
    case 2: fields = snprintf(tail, left_capacity, ",\"last_reason\":%u,\"converged\":%s}",
                              (unsigned)nlms->last_reason, nlms->converged ? "true" : "false"); break;
    case 3: fields = snprintf(tail, left_capacity, ",\"blocks\":\"%" PRIu64 "\"}", nlms->blocks); break;
    case 4: fields = snprintf(tail, left_capacity, ",\"updated_blocks\":\"%" PRIu64 "\"}", nlms->updated_blocks); break;
    case 5: fields = snprintf(tail, left_capacity, ",\"frozen_blocks\":\"%" PRIu64 "\"}", nlms->frozen_blocks); break;
    case 6: fields = snprintf(tail, left_capacity, ",\"updated_samples\":\"%" PRIu64 "\"}", nlms->updated_samples); break;
    case 7: fields = snprintf(tail, left_capacity, ",\"frozen_samples\":\"%" PRIu64 "\"}", nlms->frozen_samples); break;
    case 8: fields = snprintf(tail, left_capacity, ",\"converged_blocks\":\"%" PRIu64 "\"}", nlms->converged_blocks); break;
    case 9: fields = snprintf(tail, left_capacity, ",\"first_converged_frame\":\"%" PRIu64 "\"}", nlms->first_converged_frame); break;
    case 10: fields = snprintf(tail, left_capacity, ",\"convergence_events\":%" PRIu32 "}", nlms->convergence_events); break;
    case 11: fields = snprintf(tail, left_capacity, ",\"update_streak\":%" PRIu32 "}", nlms->update_streak); break;
    case 12: fields = snprintf(tail, left_capacity, ",\"max_update_streak\":%" PRIu32 "}", nlms->max_update_streak); break;
    case 13: fields = snprintf(tail, left_capacity, ",\"bounded_updates\":%" PRIu32 "}", nlms->bounded_updates); break;
    case 14: fields = snprintf(tail, left_capacity, ",\"resets\":%" PRIu32 "}", nlms->resets); break;
    default: {
        const unsigned reason = part - 15U;
        fields = snprintf(tail, left_capacity, ",\"reason_%u\":\"%" PRIu64 "\"}",
                          reason, nlms->reasons[reason]);
        break;
    }
    }
    return observability_finish(buffer, capacity, prefix, fields);
}

size_t mic_nlms_telemetry_format(const mic_pipeline_telemetry_t *record,
                                  unsigned part, char *buffer, size_t capacity)
{
    return mic_nlms_telemetry_format_impl(record, part, buffer, capacity, false);
}

size_t mic_nlms_telemetry_cache_format(const mic_pipeline_telemetry_t *record,
                                        unsigned part, char *buffer, size_t capacity)
{
    return mic_nlms_telemetry_format_impl(record, part, buffer, capacity, true);
}

size_t mic_nlms_presence_telemetry_format(const mic_pipeline_telemetry_t *record,
                                           unsigned part, char *buffer, size_t capacity)
{
    if (buffer && capacity) buffer[0] = '\0';
    if (!buffer || !capacity || !observability_record_valid(record) ||
        !record->nlms.blocks || part >= MIC_NLMS_PRESENCE_TELEMETRY_PARTS) return 0;
    const mic_nlms_telemetry_t *nlms = &record->nlms;
    const int prefix = observability_prefix(buffer, capacity, "audio.nlms_presence", record, part);
    if (prefix < 0 || (size_t)prefix >= capacity) { buffer[0] = '\0'; return 0; }
    char *tail = buffer + prefix;
    const size_t left_capacity = capacity - (size_t)prefix;
    int fields;
    if (part == 0U) {
        fields = snprintf(tail, left_capacity,
                          ",\"part_count\":%u,\"has_first_converged_frame\":%s}",
                          MIC_NLMS_PRESENCE_TELEMETRY_PARTS,
                          nlms->has_first_converged_frame ? "true" : "false");
    } else if (nlms->has_first_converged_frame) {
        fields = snprintf(tail, left_capacity, ",\"first_converged_frame\":\"%" PRIu64 "\"}",
                          nlms->first_converged_frame);
    } else {
        fields = snprintf(tail, left_capacity, ",\"first_converged_frame\":null}");
    }
    return observability_finish_with_limit(buffer, capacity,
                                           MIC_NLMS_PRESENCE_TELEMETRY_BYTES,
                                           prefix, fields);
}


size_t mic_dispatch_telemetry_format(const mic_pipeline_telemetry_t *record,
                                    unsigned part, char *buffer, size_t capacity)
{
    if (buffer && capacity) buffer[0] = '\0';
    if (!buffer || !capacity || !observability_record_valid(record) ||
        part >= MIC_DISPATCH_TELEMETRY_PARTS) return 0;
    const int prefix = observability_prefix(buffer, capacity, "audio.dispatch", record, part);
    if (prefix < 0 || (size_t)prefix >= capacity) { buffer[0] = '\0'; return 0; }
    char *tail = buffer + prefix;
    const size_t left = capacity - (size_t)prefix;
    int fields = -1;
    if (part >= 2U && part <= 10U) {
        const mic_pipeline_metric_t *metrics[] = { &record->dispatch_queue_wait,
            &record->dispatch_control_wait, &record->dispatch_frame_age };
        const char *keys[] = { "qw", "cw", "fa" };
        const unsigned index = (part - 2U) / 3U;
        switch ((part - 2U) % 3U) {
        case 0: fields = metric_total(tail, left, keys[index], metrics[index]); break;
        case 1: fields = metric_count_low(tail, left, keys[index], metrics[index]); break;
        case 2: fields = metric_high(tail, left, keys[index], metrics[index]); break;
        }
    } else switch (part) {
    case 0: fields = snprintf(tail, left, ",\"part_count\":%u}", MIC_DISPATCH_TELEMETRY_PARTS); break;
    case 1: fields = snprintf(tail, left, ",\"sampled_us\":\"%" PRIu64 "\"}", record->sampled_us); break;
    case 11: fields = snprintf(tail, left, ",\"queue_full\":%" PRIu32 "}", record->dispatch_queue_full); break;
    case 12: fields = snprintf(tail, left, ",\"invalidated\":%" PRIu32 "}", record->dispatch_invalidated); break;
    case 13: fields = snprintf(tail, left, ",\"unavailable\":%" PRIu32 "}", record->dispatch_unavailable); break;
    case 14: fields = snprintf(tail, left, ",\"event_lost\":%" PRIu32 "}", record->dispatch_event_lost); break;
    case 15: fields = snprintf(tail, left, ",\"queue_hi\":%" PRIu32 "}", record->dispatch_queue_highwater); break;
    case 16: fields = snprintf(tail, left, ",\"late_events\":%" PRIu32 "}", record->dispatch_late_events); break;
    }
    return observability_finish(buffer, capacity, prefix, fields);
}

size_t mic_output_health_telemetry_format(const mic_pipeline_telemetry_t *record,
                                         unsigned part, char *buffer, size_t capacity)
{
    if (buffer && capacity) buffer[0] = '\0';
    if (!buffer || !capacity || !observability_record_valid(record) ||
        part >= MIC_OUTPUT_HEALTH_TELEMETRY_PARTS) return 0;
    const int prefix = observability_prefix(buffer, capacity, "audio.output", record, part);
    if (prefix < 0 || (size_t)prefix >= capacity) { buffer[0] = '\0'; return 0; }
    char *tail = buffer + prefix;
    const size_t left = capacity - (size_t)prefix;
    const mic_output_health_t *output = &record->capture_health.output;
    int fields = -1;
    switch (part) {
    case 0: fields = snprintf(tail, left, ",\"part_count\":%u}", MIC_OUTPUT_HEALTH_TELEMETRY_PARTS); break;
    case 1: fields = snprintf(tail, left, ",\"sampled_us\":\"%" PRIu64 "\"}", record->sampled_us); break;
    case 2: fields = snprintf(tail, left, ",\"samples\":\"%" PRIu64 "\"}", output->samples); break;
    case 3:
        if (output->samples)
            fields = snprintf(tail, left, ",\"l_rms\":%.1f,\"r_rms\":%.1f}",
                sqrt((double)output->sum_sq[0] / (double)output->samples),
                sqrt((double)output->sum_sq[1] / (double)output->samples));
        else fields = snprintf(tail, left, ",\"l_rms\":null,\"r_rms\":null}");
        break;
    case 4:
        if (output->samples)
            fields = snprintf(tail, left, ",\"l_peak\":%" PRIu32 ",\"r_peak\":%" PRIu32 "}",
                              output->peak[0], output->peak[1]);
        else fields = snprintf(tail, left, ",\"l_peak\":null,\"r_peak\":null}");
        break;
    case 5: fields = snprintf(tail, left, ",\"limited_frames\":\"%" PRIu64 "\"}", output->limited_frames); break;
    case 6:
        if (output->samples && output->limited_frames <= output->samples)
            fields = snprintf(tail, left, ",\"limited_pct\":%.3f}",
                              100.0 * (double)output->limited_frames / (double)output->samples);
        else fields = snprintf(tail, left, ",\"limited_pct\":null}");
        break;
    }
    return observability_finish(buffer, capacity, prefix, fields);
}
