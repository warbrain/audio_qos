#include "audio_rx_telemetry.h"
#include "dual_end_runtime.h"
#include "dual_end_telemetry_format.h"

#include <inttypes.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define AUDIO_RX_TELEMETRY_STACK_BYTES 6144
#define AUDIO_RX_TELEMETRY_PRIORITY 3
#define AUDIO_RX_SNAPSHOT_QUEUE_DEPTH 2
#define AUDIO_RX_EVENT_QUEUE_DEPTH 8

static const char *TAG = "dual_end_audio";
typedef struct {
    int64_t sampled_us;
    uint32_t session_generation;
    uint8_t codec;
    bool active, valid;
} telemetry_log_state_t;

/* Cache freshness is independent of text volume. Lifecycle transitions are
 * visible promptly, while ordinary 1 Hz copies produce one log batch / 10 s. */
static bool telemetry_log_due(telemetry_log_state_t *state,
                              const audio_rx_telemetry_snapshot_t *snapshot)
{
    const bool due = !state->valid ||
        state->session_generation != snapshot->session_generation ||
        state->active != snapshot->active || state->codec != snapshot->codec ||
        snapshot->sampled_us - state->sampled_us >= 10000000LL;
    if (due) {
        state->sampled_us = snapshot->sampled_us;
        state->session_generation = snapshot->session_generation;
        state->active = snapshot->active;
        state->codec = snapshot->codec;
        state->valid = true;
    }
    return due;
}

typedef struct {
    audio_rx_telemetry_snapshot_t snapshot;
    uint32_t snapshots_dropped, events_dropped, sequence;
} telemetry_record_t;
typedef struct {
    audio_rx_telemetry_event_kind_t kind;
    uint32_t a, b, c, d, e;
} telemetry_event_t;

static QueueHandle_t s_snapshots, s_events;
static TaskHandle_t s_telemetry_task;
/* Telemetry is observability only. Once allocation has failed, publishing
 * remains a bounded no-op so it can never hold up audio receiver startup. */
static bool s_telemetry_unavailable;
/* Producer storage/counters belong exclusively to RX after initialization.
 * QueueSend copies values, never pointers. Worker storage belongs to worker. */
static telemetry_record_t s_publish_record;
static uint32_t s_snapshots_dropped, s_events_dropped, s_snapshot_sequence;
/* Written only by the background logger after dequeue. The RX owner keeps its
 * existing queue-only, nonblocking publish path without a new lock or copy. */
static portMUX_TYPE s_latest_lock = portMUX_INITIALIZER_UNLOCKED;
static audio_rx_telemetry_cache_t s_latest_cache;
static bool s_latest_ready;

bool audio_rx_telemetry_get_latest(audio_rx_telemetry_cache_t *out)
{
    if (!out) return false;
    portENTER_CRITICAL(&s_latest_lock);
    const bool ready = s_latest_ready;
    if (ready) *out = s_latest_cache;
    portEXIT_CRITICAL(&s_latest_lock);
    return ready;
}

uint32_t audio_rx_telemetry_stack_min(void)
{
    /* ESP-IDF returns bytes, unlike upstream FreeRTOS's word convention. */
    return (uint32_t)uxTaskGetStackHighWaterMark(NULL);
}

bool audio_rx_telemetry_enabled(void)
{
    return s_telemetry_task != NULL;
}

bool audio_rx_telemetry_publish(const audio_rx_telemetry_snapshot_t *snapshot)
{
    if (!snapshot || !s_telemetry_task) return false;
    s_publish_record.snapshot = *snapshot;
    s_publish_record.snapshots_dropped = s_snapshots_dropped;
    s_publish_record.events_dropped = s_events_dropped;
    /* Zero is reserved for "no completed cache" in the CDC v2 context
     * boundary.  Keep published sequences nonzero across the very rare wrap. */
    if (++s_snapshot_sequence == 0U) ++s_snapshot_sequence;
    s_publish_record.sequence = s_snapshot_sequence;
    if (xQueueSend(s_snapshots, &s_publish_record, 0) != pdTRUE) {
        if(s_snapshots_dropped!=UINT32_MAX)++s_snapshots_dropped;
        return false;
    }
    xTaskNotifyGive(s_telemetry_task);
    return true;
}

void audio_rx_telemetry_event(audio_rx_telemetry_event_kind_t kind,
    uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e)
{
    const telemetry_event_t event = {.kind = kind, .a = a, .b = b, .c = c, .d = d, .e = e};
    if (!s_telemetry_task || xQueueSend(s_events, &event, 0) != pdTRUE) {
        if(s_events_dropped!=UINT32_MAX)++s_events_dropped;
        return;
    }
    xTaskNotifyGive(s_telemetry_task);
}

static void telemetry_log_event(const telemetry_event_t *event)
{
    switch (event->kind) {
    case AUDIO_RX_LOG_DECODERS_READY:
        ESP_LOGI(TAG, "two independent G722 mono decoders ready: bitrate=%d each",
                 DUAL_END_AUDIO_G722_BITRATE_BPS);
        break;
    case AUDIO_RX_LOG_SESSION_BEGIN:
        ESP_LOGI(TAG, "audio session begin id=%" PRIu32 " first=%" PRIu32, event->a, event->b);
        break;
    case AUDIO_RX_LOG_DECODER_OPEN_FAILED:
        ESP_LOGE(TAG, "open G722 stream=%" PRIu32 " err=%" PRId32, event->a, (int32_t)event->b);
        break;
    case AUDIO_RX_LOG_DECODER_NOT_READY:
        ESP_LOGE(TAG, "G722 stream not initialized");
        break;
    case AUDIO_RX_LOG_DECODE_FAILED:
        ESP_LOGE(TAG, "G722 decode err=%" PRId32 " consumed=%" PRIu32 "/%d decoded=%" PRIu32
                 "/%d needed=%" PRIu32, (int32_t)event->a, event->b,
                 DUAL_END_AUDIO_G722_FRAME_BYTES, event->c, DUAL_END_AUDIO_FRAME_BYTES, event->d);
        break;
    }
}

static void telemetry_task(void *arg)
{
    (void)arg;
    static telemetry_record_t record;
    static telemetry_event_t event;
    telemetry_log_state_t log_state = {0};
    int64_t last_stack_scan_us = 0;
    uint32_t telemetry_stack_min_bytes = 0;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* Bound each event batch so error floods cannot starve snapshots. */
        for (unsigned i = 0; i < AUDIO_RX_EVENT_QUEUE_DEPTH; ++i) {
            if (xQueueReceive(s_events, &event, 0) != pdTRUE) break;
            telemetry_log_event(&event);
        }
        while (xQueueReceive(s_snapshots, &record, 0) == pdTRUE) {
            const int64_t age_us = esp_timer_get_time() - record.snapshot.sampled_us;
            const uint64_t cache_published_us = (uint64_t)esp_timer_get_time();
            const bool log_due = telemetry_log_due(&log_state, &record.snapshot);
            if (!telemetry_stack_min_bytes ||
                record.snapshot.sampled_us - last_stack_scan_us >= 10000000LL) {
                telemetry_stack_min_bytes = audio_rx_telemetry_stack_min();
                last_stack_scan_us = record.snapshot.sampled_us;
            }
            /* Cache a completed owner copy before formatting/log transport.
             * The bounded diagnostic query can therefore remain DTR-agnostic
             * and never touch recovery state or wait for this worker. */
            portENTER_CRITICAL(&s_latest_lock);
            s_latest_cache.snapshot = record.snapshot;
            s_latest_cache.sequence = record.sequence;
            s_latest_cache.snapshots_dropped = record.snapshots_dropped;
            s_latest_cache.events_dropped = record.events_dropped;
            s_latest_cache.cache_published_us = cache_published_us;
            s_latest_cache.telemetry_stack_min_bytes = telemetry_stack_min_bytes;
            s_latest_ready = true;
            portEXIT_CRITICAL(&s_latest_lock);
            if (!log_due) continue;
            audio_rx_telemetry_log_snapshot(&record.snapshot);
            /* The RX task copied session + bound decoder codec with all
             * counters before enqueueing. Reading live RX globals here would
             * mix sessions during a codec transition. Null codec is explicit
             * before the first valid packet binds a decoder. */
            dual_end_rx_telemetry_t detail={.boot_id=dual_end_runtime_boot_id(),
                .role=1,.session_id=record.snapshot.session_id,.sequence=record.sequence,
                .codec=record.snapshot.codec,.codec_known=record.snapshot.codec!=UINT8_MAX,
                .active=record.snapshot.active,.sampled_us=(uint64_t)record.snapshot.sampled_us,
                .snapshot_age_us=age_us>0?(uint64_t)age_us:0,
                .snapshot_dropped=record.snapshots_dropped,.event_dropped=record.events_dropped,
                .rx_stack_min_bytes=record.snapshot.rx_stack_min_bytes,
                .telemetry_stack_min_bytes=telemetry_stack_min_bytes};
            char detail_line[DUAL_END_TELEMETRY_JSON_BYTES];
            for(unsigned part=0;part<DUAL_END_RX_TELEMETRY_PARTS;++part)
                if(dual_end_rx_telemetry_format(&detail,part,detail_line,sizeof(detail_line)))
                    ESP_LOGI(TAG,"%s",detail_line);
            ESP_LOGI(TAG, "AUDIO_RX_TASK schema=1 rx_stack_min_bytes=%" PRIu32
                     " telemetry_stack_min_bytes=%" PRIu32 " snapshot_dropped=%" PRIu32
                     " event_dropped=%" PRIu32 " snapshot_age_us=%" PRId64,
                     record.snapshot.rx_stack_min_bytes, telemetry_stack_min_bytes,
                     record.snapshots_dropped, record.events_dropped, age_us > 0 ? age_us : 0);
        }
    }
}

esp_err_t audio_rx_telemetry_start(void)
{
    if (s_telemetry_task) return ESP_OK;
    if (s_telemetry_unavailable) return ESP_OK;
    s_snapshots = xQueueCreate(AUDIO_RX_SNAPSHOT_QUEUE_DEPTH, sizeof(telemetry_record_t));
    s_events = xQueueCreate(AUDIO_RX_EVENT_QUEUE_DEPTH, sizeof(telemetry_event_t));
    if (!s_snapshots || !s_events ||
        xTaskCreatePinnedToCore(telemetry_task, "audio_rx_log", AUDIO_RX_TELEMETRY_STACK_BYTES,
                    NULL, AUDIO_RX_TELEMETRY_PRIORITY, &s_telemetry_task, DUAL_END_COMM_TASK_CORE) != pdPASS) {
        if (s_snapshots) vQueueDelete(s_snapshots);
        if (s_events) vQueueDelete(s_events);
        s_snapshots = s_events = NULL;
        s_telemetry_task = NULL;
        s_telemetry_unavailable = true;
        ESP_LOGW(TAG, "RX telemetry disabled: allocation/task creation failed; audio continues");
        return ESP_OK;
    }
    return ESP_OK;
}
