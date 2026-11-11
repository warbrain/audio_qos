#include "scepter_diagnostics_format.h"

#include "dual_end_spatial.h"
#include "mic_observability_telemetry.h"

#include <inttypes.h>
#include <stdio.h>

unsigned scepter_diagnostics_part_count(unsigned kind)
{
    switch (kind) {
    case SCEPTER_DIAGNOSTICS_KIND_SPATIAL:
        return DUAL_END_SPATIAL_TELEMETRY_PARTS;
    case SCEPTER_DIAGNOSTICS_KIND_PIPELINE:
        return MIC_PIPELINE_TELEMETRY_PARTS;
    case SCEPTER_DIAGNOSTICS_KIND_CAPTURE_HEALTH:
        return MIC_CAPTURE_HEALTH_TELEMETRY_PARTS;
    case SCEPTER_DIAGNOSTICS_KIND_OWNER_TIMING:
        return MIC_OWNER_TIMING_TELEMETRY_PARTS;
    case SCEPTER_DIAGNOSTICS_KIND_NLMS:
        return MIC_NLMS_TELEMETRY_PARTS;
    case SCEPTER_DIAGNOSTICS_KIND_WIENER:
        return SCEPTER_DIAGNOSTICS_WIENER_PARTS;
    default:
        return 0;
    }
}

static size_t wiener_format_part(
    const mic_audio_diagnostics_snapshot_t *snapshot,
    unsigned part,
    char *buffer,
    size_t capacity)
{
    if (!snapshot || !snapshot->spatial_ready || !buffer || !capacity) return 0;

    const dual_end_spatial_telemetry_t *spatial = &snapshot->spatial;
    const mic_wiener_diagnostics_t *wiener = &snapshot->wiener;
    const bool stereo = spatial->applied == DUAL_END_SPATIAL_WIENER_STEREO;
    const unsigned part_count = stereo ? 8U : 5U;
    if (part >= part_count) return 0;
    const bool active = wiener->available &&
        (spatial->applied == DUAL_END_SPATIAL_DAS_WIENER || stereo) &&
        wiener->channels == (stereo ? 2U : 1U);
    int prefix = snprintf(buffer, capacity,
        "{\"event\":\"audio.wiener\",\"schema\":1,\"boot_id\":\"%08" PRIx32
        "\",\"role\":0,\"session_id\":%" PRIu32 ",\"seq\":%" PRIu32
        ",\"part\":%u",
        spatial->boot_id, spatial->session_id, snapshot->cache_sequence, part);
    if (prefix < 0 || (size_t)prefix >= capacity) {
        buffer[0] = '\0';
        return 0;
    }

    char *out = buffer + prefix;
    size_t left = capacity - (size_t)prefix;
    int fields = -1;
    switch (part) {
    case 0:
        fields = snprintf(out, left,
            ",\"part_count\":%u,\"sampled_us\":\"%" PRIu64 "\"}",
            part_count, spatial->sampled_us);
        break;
    case 1:
        if (stereo) {
            fields = snprintf(out, left,
                ",\"active\":%s,\"available\":%s,\"channels\":2,\"lv\":%s,\"rv\":%s}",
                active ? "true" : "false", active ? "true" : "false",
                active ? (wiener->noise_valid[0] ? "true" : "false") : "null",
                active ? (wiener->noise_valid[1] ? "true" : "false") : "null");
        } else {
            fields = snprintf(out, left,
                ",\"active\":%s,\"available\":%s,\"noise_valid\":%s}",
                active ? "true" : "false", active ? "true" : "false",
                active ? (wiener->noise_valid[0] ? "true" : "false") : "null");
        }
        break;
    case 2:
        if (stereo) {
            fields = active ? snprintf(out, left,
                ",\"lh\":\"%" PRIu64 "\"}", wiener->processed_hops[0]) :
                snprintf(out, left, ",\"lh\":null}");
        } else {
            fields = active ? snprintf(out, left,
                ",\"processed_hops\":\"%" PRIu64 "\"}", wiener->processed_hops[0]) :
                snprintf(out, left, ",\"processed_hops\":null}");
        }
        break;
    case 3:
        if (stereo) {
            fields = active ? snprintf(out, left,
                ",\"rh\":\"%" PRIu64 "\"}", wiener->processed_hops[1]) :
                snprintf(out, left, ",\"rh\":null}");
        } else {
            fields = active ? snprintf(out, left,
                ",\"queue_underflows\":\"%" PRIu64 "\"}", wiener->queue_underflows[0]) :
                snprintf(out, left, ",\"queue_underflows\":null}");
        }
        break;
    case 4:
        if (stereo) {
            fields = active ? snprintf(out, left,
                ",\"lu\":\"%" PRIu64 "\"}", wiener->queue_underflows[0]) :
                snprintf(out, left, ",\"lu\":null}");
        } else {
            fields = active ? snprintf(out, left,
                ",\"queue_overflows\":\"%" PRIu64 "\"}", wiener->queue_overflows[0]) :
                snprintf(out, left, ",\"queue_overflows\":null}");
        }
        break;
    case 5:
        fields = active ? snprintf(out, left,
            ",\"ru\":\"%" PRIu64 "\"}", wiener->queue_underflows[1]) :
            snprintf(out, left, ",\"ru\":null}");
        break;
    case 6:
        fields = active ? snprintf(out, left,
            ",\"lo\":\"%" PRIu64 "\"}", wiener->queue_overflows[0]) :
            snprintf(out, left, ",\"lo\":null}");
        break;
    case 7:
        fields = active ? snprintf(out, left,
            ",\"ro\":\"%" PRIu64 "\"}", wiener->queue_overflows[1]) :
            snprintf(out, left, ",\"ro\":null}");
        break;
    }
    if (fields < 0 || (size_t)fields >= left ||
        (size_t)prefix + (size_t)fields >= DUAL_END_TELEMETRY_JSON_BYTES) {
        buffer[0] = '\0';
        return 0;
    }
    return (size_t)prefix + (size_t)fields;
}

size_t scepter_diagnostics_format_part(
    const mic_audio_diagnostics_snapshot_t *snapshot,
    unsigned kind,
    unsigned part,
    char *buffer,
    size_t capacity,
    const char **event_name,
    unsigned *schema,
    bool *active)
{
    if (buffer && capacity) buffer[0] = '\0';
    if (event_name) *event_name = NULL;
    if (schema) *schema = 0;
    if (active) *active = false;
    if (!snapshot || !snapshot->ready || !buffer || !capacity ||
        part >= scepter_diagnostics_part_count(kind)) {
        return 0;
    }

    switch (kind) {
    case SCEPTER_DIAGNOSTICS_KIND_SPATIAL:
        if (!snapshot->spatial_ready) return 0;
        if (event_name) *event_name = "audio.spatial";
        if (schema) *schema = 2U;
        if (active) *active = true;
        return dual_end_spatial_telemetry_format(&snapshot->spatial, part,
                                                 buffer, capacity);
    case SCEPTER_DIAGNOSTICS_KIND_PIPELINE:
        if (event_name) *event_name = "audio.pipeline";
        if (schema) *schema = 1U;
        if (active) *active = true;
        return mic_pipeline_telemetry_format(&snapshot->pipeline, part,
                                             buffer, capacity);
    case SCEPTER_DIAGNOSTICS_KIND_CAPTURE_HEALTH:
        if (event_name) *event_name = "audio.capture_health";
        if (schema) *schema = 1U;
        if (active) *active = true;
        return mic_capture_health_telemetry_format(&snapshot->pipeline, part,
                                                   buffer, capacity);
    case SCEPTER_DIAGNOSTICS_KIND_OWNER_TIMING:
        if (event_name) *event_name = "audio.owner_timing";
        if (schema) *schema = 1U;
        if (active) *active = true;
        return mic_owner_timing_telemetry_format(&snapshot->pipeline, part,
                                                 buffer, capacity);
    case SCEPTER_DIAGNOSTICS_KIND_NLMS:
        if (event_name) *event_name = "audio.nlms";
        if (schema) *schema = 1U;
        if (active) *active = snapshot->pipeline.nlms.blocks != 0;
        return mic_nlms_telemetry_cache_format(&snapshot->pipeline, part,
                                                buffer, capacity);
    case SCEPTER_DIAGNOSTICS_KIND_WIENER:
        if (event_name) *event_name = "audio.wiener";
        if (schema) *schema = 1U;
        if (active) *active = snapshot->wiener.available;
        return wiener_format_part(snapshot, part, buffer, capacity);
    default:
        return 0;
    }
}
