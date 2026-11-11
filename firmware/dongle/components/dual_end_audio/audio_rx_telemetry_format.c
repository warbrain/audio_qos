#include "audio_rx_telemetry.h"

#include "dongle_uac_bridge.h"

#include <inttypes.h>
#include "esp_log.h"

static const char *TAG = "dual_end_audio";

void audio_rx_telemetry_log_snapshot(const audio_rx_telemetry_snapshot_t *snapshot)
{
    const audio_rx_stats_t *stats = &snapshot->stats;
    ESP_LOGI(TAG, "audio recovery session=%" PRIu32 " active=%d accepted=%" PRIu32
             " decoded=%" PRIu32 " recovered=%" PRIu32 " permanent_loss=%" PRIu32
             " masked=%" PRIu32 " dup=%" PRIu32 " late=%" PRIu32
             " rejected=%" PRIu32 " crc_err=%" PRIu32 " decode_err=%" PRIu32
             " invalid=%" PRIu32 " gaps=%" PRIu32 " nack=%" PRIu32
             " resync=%" PRIu32 " feedback_drop=%" PRIu32
             " nack_queued=%" PRIu32 " nack_accepted=%" PRIu32
             " nack_cancelled=%" PRIu32 " nack_expired=%" PRIu32
             " nack_rejected=%" PRIu32 " gap_deadline_us=%" PRId64
             " repair_ontime=%" PRIu32 " repair_late=%" PRIu32
             " output_frames=%" PRIu32 " recovered_decoded=%" PRIu32
             " timed_sink_drop=%" PRIu32 " concealed_repeat=%" PRIu32
             " sessions=%" PRIu32
             " aborts=%" PRIu32 " aborted_buffered=%" PRIu32,
             snapshot->session_id, snapshot->active ? 1 : 0,
             stats->accepted, snapshot->delivered_frames, stats->recovered,
             stats->permanent_loss, snapshot->masked_frames, stats->duplicates,
             stats->late, stats->rejected, snapshot->crc_error_frames,
             snapshot->decode_error_frames, snapshot->invalid_frames, stats->gap_events,
             stats->nack_requests, stats->resync_requests, snapshot->feedback_queue_failures,
             snapshot->nack_queued, stats->nack_accepted, stats->nack_cancelled,
             stats->nack_expired, stats->nack_rejected,
             snapshot->gap_deadline_us,
             stats->repair_ontime, stats->repair_late,
             stats->output_frames, snapshot->recovered_decoded_frames,
             snapshot->timed_sink_drop_frames, snapshot->concealed_repeat_frames,
             stats->session_count, stats->abort_count, stats->aborted_buffered_frames);
    /* All values are lifetime counters except uptime/session state.
     * permanent_loss counts active timeline decode-deadline gaps, including
     * the inferred stop tail before idle abort. recovered_decoded confirms
     * both codecs succeeded, before the PCM sink or USB playback. */
    ESP_LOGI(TAG, "STABILITY_RX schema=1 uptime_ms=%" PRId64
             " accepted=%" PRIu32 " output_frames=%" PRIu32
             " permanent_loss=%" PRIu32 " recovered=%" PRIu32
             " recovered_decoded=%" PRIu32 " repair_ontime=%" PRIu32
             " repair_late=%" PRIu32 " duplicates=%" PRIu32
             " late=%" PRIu32 " rejected=%" PRIu32 " gap_events=%" PRIu32
             " nack_requests=%" PRIu32 " nack_accepted=%" PRIu32
             " nack_cancelled=%" PRIu32 " nack_expired=%" PRIu32
             " nack_rejected=%" PRIu32 " resync_requests=%" PRIu32
             " sessions=%" PRIu32 " aborts=%" PRIu32
             " aborted_buffered=%" PRIu32 " concealed_repeat=%" PRIu32
             " decode_fail=%" PRIu32
             " session_active=%d session_id=%" PRIu32,
             snapshot->sampled_us / 1000, stats->accepted, stats->output_frames,
             stats->permanent_loss, stats->recovered, snapshot->recovered_decoded_frames,
             stats->repair_ontime, stats->repair_late, stats->duplicates,
             stats->late, stats->rejected, stats->gap_events,
             stats->nack_requests, stats->nack_accepted, stats->nack_cancelled,
             stats->nack_expired, stats->nack_rejected, stats->resync_requests,
             stats->session_count, stats->abort_count, stats->aborted_buffered_frames,
             snapshot->concealed_repeat_frames,
             snapshot->decode_error_frames, snapshot->active ? 1 : 0,
             snapshot->session_id);
    ESP_LOGI(TAG, "AUDIO_TIMELINE_RX target_ms=%" PRId64 " ring_ms=%" PRIu32
             " hid_dispatches=%" PRIu32 " hid_wait_max_us=%" PRIu32
             " expired_buffered=%" PRIu32 " decode_frame_max_us=%" PRIu32
             " poll_max_us=%" PRIu32 " poll_audio_max=%" PRIu32
             " hid_wait_seq=%" PRIu32 " hid_wait_at_us=%" PRId64,
             (int64_t)(DUAL_END_AUDIO_PLAYOUT_DELAY_US / 1000LL),
             dongle_uac_bridge_timeline_capacity_ms(),
             snapshot->hid_dispatches, snapshot->hid_wait_max_us, stats->expired_buffered_frames,
             snapshot->decode_frame_max_us, snapshot->poll_max_us, snapshot->poll_audio_max,
             snapshot->hid_wait_peak_sequence, snapshot->hid_wait_peak_us);
}
