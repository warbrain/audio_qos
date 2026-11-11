#include "dual_end_runtime.h"
#include "dual_end_perf_format.h"
#include "dual_end_resource_format.h"
#include "dual_end_log_health_format.h"
#include <stdlib.h>
#include <errno.h>
#include "freertos/idf_additions.h"
#include <inttypes.h>
#include <stdatomic.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define RUNTIME_WINDOW_US INT64_C(5000000)
#define RUNTIME_QUEUE_DEPTH 4U
#define RUNTIME_SNAPSHOT_LEASE_US UINT64_C(10000000)
#define RUNTIME_LOGGER_STACK_BYTES 6144U
#define RUNTIME_LOGGER_PRIORITY 2U
static const char *TAG = "dual_end_runtime";
typedef struct {
    dual_end_perf_record_t record;
    TaskHandle_t owner;
} runtime_snapshot_t;
typedef struct {
    bool active;
    int64_t started_us, ended_us;
    dual_end_perf_stats_t stats;
    TaskHandle_t owner;
} runtime_window_t;
static QueueHandle_t s_snapshots;
static runtime_window_t s_windows[DUAL_END_STAGE_COUNT];
static uint32_t s_boot_id;
static atomic_uint s_window_seq;
static atomic_uint s_telemetry_drop;
static scepter_role_t s_role;
static esp_chip_info_t s_chip;
static char s_device_id[24], s_fw[32];
static bool s_attempted;
static char s_elf_sha256[65];
static atomic_bool s_enabled;
static atomic_bool s_identity_ready;

static void runtime_emit(const char *format, ...) __attribute__((format(printf, 1, 2)));
static void runtime_emit(const char *format, ...)
{
    char line[DUAL_END_PERF_JSON_BUFFER_BYTES];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length > 0 && (size_t)length < sizeof(line)) ESP_LOGI(TAG, "%s", line);
    /* Only the background logger calls this helper. Yield between multipart
     * entries so a bounded USB queue can drain; never sleep an audio owner. */
    vTaskDelay(1);
}
/* Repeat with each complete window: a monitor attached after boot still gets
 * actual identity. Values are boot constants, never inferred from USB 0101. */
static void runtime_log_identity(int logger_core)
{
    runtime_emit("{\"event\":\"audio.device_boot\",\"schema\":1,\"part\":0,\"boot_id\":\"%08" PRIx32
        "\",\"role\":%u,\"device_id\":\"%s\"}", s_boot_id, (unsigned)s_role, s_device_id);
    runtime_emit("{\"event\":\"audio.device_boot\",\"schema\":1,\"part\":1,\"boot_id\":\"%08" PRIx32
        "\",\"role\":%u,\"target\":\"%s\",\"cores\":%u,\"revision\":%u}",
        s_boot_id, (unsigned)s_role, CONFIG_IDF_TARGET, (unsigned)s_chip.cores, (unsigned)s_chip.revision);
    runtime_emit("{\"event\":\"audio.device_boot\",\"schema\":1,\"part\":2,\"boot_id\":\"%08" PRIx32
        "\",\"role\":%u,\"fw\":\"%s\"}", s_boot_id, (unsigned)s_role, s_fw);
    runtime_emit("{\"event\":\"audio.device_boot\",\"schema\":1,\"part\":3,\"boot_id\":\"%08" PRIx32
        "\",\"role\":%u,\"cpu_mhz\":%u,\"audio_core\":%u,\"comm_core\":%u}",
        s_boot_id, (unsigned)s_role, (unsigned)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        (unsigned)DUAL_END_AUDIO_TASK_CORE, (unsigned)DUAL_END_COMM_TASK_CORE);
    runtime_emit("{\"event\":\"audio.device_boot\",\"schema\":1,\"part\":4,\"boot_id\":\"%08" PRIx32
        "\",\"role\":%u,\"enabled\":%s,\"queue_capacity\":%u,\"logger_stack\":%u,\"logger_priority\":%u}",
        s_boot_id, (unsigned)s_role, atomic_load_explicit(&s_enabled, memory_order_acquire) ? "true" : "false", RUNTIME_QUEUE_DEPTH,
        RUNTIME_LOGGER_STACK_BYTES, RUNTIME_LOGGER_PRIORITY);
    runtime_emit("{\"event\":\"audio.device_boot\",\"schema\":1,\"part\":5,\"boot_id\":\"%08" PRIx32
        "\",\"role\":%u,\"queue_item_bytes\":%u,\"logger_core\":%d,\"resources_scope\":\"logger_time\"}",
        s_boot_id, (unsigned)s_role, (unsigned)sizeof(runtime_snapshot_t), logger_core);
}
/* Build identity is immutable and remains readable after boot. Keep the
 * six legacy device_boot parts untouched for older multipart collectors. */
static size_t runtime_format_build_info(char *out, size_t capacity)
{
    if (!out || !capacity) return 0;
    const int written=snprintf(out,capacity,
        "{\"event\":\"audio.device_build\",\"schema\":1,\"part\":0,\"part_count\":1,"
        "\"boot_id\":\"%08" PRIx32 "\",\"role\":%u,\"elf_sha256\":\"%s\"}",
        s_boot_id,(unsigned)s_role,s_elf_sha256);
    if (written<=0 || (size_t)written>=capacity ||
        (size_t)written>DUAL_END_PERF_JSON_MAX_BYTES) { out[0]='\0'; return 0; }
    return (size_t)written;
}

static portMUX_TYPE s_resource_lock=portMUX_INITIALIZER_UNLOCKED;
static dual_end_resource_snapshot_t s_resource_cache;
static bool s_resource_ready,s_perf_ready[DUAL_END_STAGE_COUNT];
static dual_end_perf_record_t s_perf_cache[DUAL_END_STAGE_COUNT];
/* PERF is retained for compatibility and copies the newest cached window on
 * every request.  Strict hosts use this one-record lease instead: part zero
 * freezes one already-published cache record, and every later part reads the
 * same copy even when the logger publishes a newer window in between.  It is
 * intentionally small, static, and protected by the existing cache lock; no
 * producer, codec, heap, or USB wait is added to the audio path. */
typedef struct {
    bool valid;
    uint32_t token;
    uint8_t stage;
    uint64_t started_us;
    dual_end_perf_record_t record;
} runtime_perf_snapshot_lease_t;
static runtime_perf_snapshot_lease_t s_perf_snapshot_lease;
typedef struct {
    bool valid;
    uint32_t token;
    uint64_t started_us;
    dual_end_resource_snapshot_t record;
} runtime_resource_snapshot_lease_t;
static runtime_resource_snapshot_lease_t s_resource_snapshot_lease;
static dual_end_runtime_io_probe_t s_io_probe;

esp_err_t dual_end_runtime_set_boot_id(uint32_t boot_id)
{
    if (s_attempted) return ESP_ERR_INVALID_STATE;
    if (!boot_id) return ESP_ERR_INVALID_ARG;
    s_boot_id=boot_id;
    return ESP_OK;
}

esp_err_t dual_end_runtime_set_io_probe(dual_end_runtime_io_probe_t probe)
{
    if(s_attempted)return ESP_ERR_INVALID_STATE;
    s_io_probe=probe;return ESP_OK;
}
static void runtime_sample_resources(void)
{
    static uint32_t sequence,previous_idle[2];
    static uint64_t previous_us;
    const uint64_t now=(uint64_t)esp_timer_get_time();
    dual_end_resource_snapshot_t r={.role=(uint8_t)s_role,.boot_id=s_boot_id,
        .sequence=++sequence,.sampled_us=now,.interval_us=previous_us?now-previous_us:0,
        .cpu_busy_permille={-1,-1}};
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS && CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER && !CONFIG_FREERTOS_SMP
    for(unsigned core=0;core<configNUMBER_OF_CORES && core<2;++core){
        uint32_t idle=(uint32_t)ulTaskGetIdleRunTimeCounterForCore((BaseType_t)core);
        if(previous_us)r.cpu_busy_permille[core]=dual_end_resource_cpu(previous_idle[core],idle,r.interval_us);
        previous_idle[core]=idle;
    }
#else
    (void)previous_idle;
#endif
    previous_us=now;
    /* No all-task snapshot or scheduler suspension. Heap/stack inspection and
     * formatting live here on the low-priority Core0 logger, every five seconds.
     * CPU is an idle-counter estimate; ISR attribution follows FreeRTOS. */
    const uint32_t internal=MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT;
    const uint32_t psram=MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT;
    r.internal_free=(uint32_t)heap_caps_get_free_size(internal);
    r.internal_min=(uint32_t)heap_caps_get_minimum_free_size(internal);
    r.internal_largest=(uint32_t)heap_caps_get_largest_free_block(internal);
    r.psram_free=(uint32_t)heap_caps_get_free_size(psram);
    r.psram_min=(uint32_t)heap_caps_get_minimum_free_size(psram);
    r.psram_largest=(uint32_t)heap_caps_get_largest_free_block(psram);
    r.logger_stack=(uint32_t)uxTaskGetStackHighWaterMark(NULL);
    r.telemetry_drop=atomic_load_explicit(&s_telemetry_drop,memory_order_relaxed);
    if(s_io_probe)r.io_available=s_io_probe(r.io);
    portENTER_CRITICAL(&s_resource_lock);s_resource_cache=r;s_resource_ready=true;portEXIT_CRITICAL(&s_resource_lock);
}
static void runtime_log_resources(void)
{
    /* The Core0 logger owns the freshly sampled cache. USB TELEMETRY reads
     * the same snapshot; neither path makes the real-time producer scan heap. */
    dual_end_resource_snapshot_t snapshot;
    portENTER_CRITICAL(&s_resource_lock);snapshot=s_resource_cache;portEXIT_CRITICAL(&s_resource_lock);
    char line[DUAL_END_PERF_JSON_BUFFER_BYTES];
    for(unsigned part=0;part<DUAL_END_RESOURCE_PARTS;++part) {
        if(dual_end_resource_format(&snapshot,part,line,sizeof(line)))ESP_LOGI(TAG,"%s",line);
        vTaskDelay(1);
    }
}
static void runtime_log_transport_health(void)
{
    static uint32_t sequence;
    dual_end_log_transport_stats_t stats;
    dual_end_log_transport_snapshot(&stats);
    const uint64_t sampled_us=(uint64_t)esp_timer_get_time();
    const uint32_t seq=++sequence;
    char line[DUAL_END_PERF_JSON_BUFFER_BYTES];
    for(unsigned part=0;part<DUAL_END_LOG_HEALTH_PARTS;++part) {
        if(dual_end_log_health_format(&stats,s_boot_id,(unsigned)s_role,seq,sampled_us,part,line,sizeof(line)))ESP_LOGI(TAG,"%s",line);
        vTaskDelay(1);
    }
}
static bool runtime_parse_numbers(const char *p,uint32_t values[3],unsigned count)
{
    for(unsigned i=0;i<count;++i){
        while(*p==' '||*p=='\t')++p;
        if(*p<'0'||*p>'9')return false;
        errno=0;char *end=NULL;unsigned long long n=strtoull(p,&end,10);
        if(errno||end==p||n>UINT32_MAX)return false;
        values[i]=(uint32_t)n;p=end;
        if(i+1<count && *p!=' ' && *p!='\t')return false;
    }
    while(*p==' '||*p=='\t')++p;
    return *p=='\0' && values[0]!=0;
}
bool dual_end_runtime_command(const char *line,char *response,size_t capacity)
{
    if(!line||!response||!capacity)return false;
    bool resources=strncmp(line,"TELEMETRY ",10)==0;
    bool perf_snapshot=strncmp(line,"PERF_SNAPSHOT ",14)==0;
    bool perf=strncmp(line,"PERF ",5)==0;
    bool build_info=strncmp(line,"BUILD_INFO ",11)==0;
    if(!resources&&!perf_snapshot&&!perf&&!build_info)return false;
    uint32_t numbers[3]={0};
    const char *arguments=line+(build_info?11:(resources?10:(perf_snapshot?14:5)));
    const unsigned argument_count=(resources||build_info)?2:3;
    if(!runtime_parse_numbers(arguments,numbers,argument_count)
       ||(build_info?numbers[1]!=0U:(resources?numbers[1]>=DUAL_END_RESOURCE_PARTS:
          numbers[1]>=DUAL_END_STAGE_COUNT || numbers[2]>=DUAL_END_PERF_PART_COUNT))){
        /* Preserve the legacy PERF/TELEMETRY error spelling byte-for-byte. */
        snprintf(response,capacity,"ERR %s arguments\n",build_info?"BUILD_INFO":(perf_snapshot?"PERF_SNAPSHOT":"TELEMETRY"));return true;
    }
    char json[DUAL_END_PERF_JSON_BUFFER_BYTES];size_t length=0;bool ready;
    /* Copy only cached counters. USB command handling never performs heap/stack
     * scans, CPU collection, codec work, waits for a sample, or changes state. */
    const uint64_t query_us=(uint64_t)esp_timer_get_time();
    if(build_info){
        ready=atomic_load_explicit(&s_identity_ready,memory_order_acquire);
        if(ready)length=runtime_format_build_info(json,sizeof(json));
    }else if(resources){
        dual_end_resource_snapshot_t snapshot;
        bool mismatch=false;
        portENTER_CRITICAL(&s_resource_lock);
        if(s_resource_snapshot_lease.valid &&
           (query_us<s_resource_snapshot_lease.started_us ||
            query_us-s_resource_snapshot_lease.started_us>=RUNTIME_SNAPSHOT_LEASE_US))
            s_resource_snapshot_lease.valid=false;
        if(numbers[1]==0U){
            if(s_resource_snapshot_lease.valid && s_resource_snapshot_lease.token==numbers[0]){
                ready=true;snapshot=s_resource_snapshot_lease.record;
            }else{
                ready=s_resource_ready;
                if(ready){
                    s_resource_snapshot_lease.valid=true;
                    s_resource_snapshot_lease.token=numbers[0];
                    s_resource_snapshot_lease.started_us=query_us;
                    s_resource_snapshot_lease.record=s_resource_cache;
                    snapshot=s_resource_snapshot_lease.record;
                }
            }
        }else if(!s_resource_snapshot_lease.valid || s_resource_snapshot_lease.token!=numbers[0]){
            mismatch=true;ready=false;
        }else{
            ready=true;snapshot=s_resource_snapshot_lease.record;
        }
        portEXIT_CRITICAL(&s_resource_lock);
        if(mismatch){
            snprintf(response,capacity,"ERR TELEMETRY token=%" PRIu32 " snapshot_lease_mismatch\n",numbers[0]);return true;
        }
        if(ready)length=dual_end_resource_format(&snapshot,numbers[1],json,sizeof(json));
    }else if(perf){
        dual_end_perf_record_t snapshot;
        portENTER_CRITICAL(&s_resource_lock);ready=s_perf_ready[numbers[1]];snapshot=s_perf_cache[numbers[1]];portEXIT_CRITICAL(&s_resource_lock);
        if(ready)length=dual_end_perf_format_part(&snapshot,numbers[2],json,sizeof(json));
    }else{
        dual_end_perf_record_t snapshot;
        bool mismatch=false;
        portENTER_CRITICAL(&s_resource_lock);
        if(s_perf_snapshot_lease.valid &&
           (query_us<s_perf_snapshot_lease.started_us ||
            query_us-s_perf_snapshot_lease.started_us>=RUNTIME_SNAPSHOT_LEASE_US))
            s_perf_snapshot_lease.valid=false;
        if(numbers[2]==0U){
            /* Repeating a lost part-zero reply is idempotent.  A new
             * token/stage starts a new lease from an already-published cache. */
            if(s_perf_snapshot_lease.valid &&
               s_perf_snapshot_lease.token==numbers[0] &&
               s_perf_snapshot_lease.stage==numbers[1]){
                ready=true;snapshot=s_perf_snapshot_lease.record;
            }else{
                ready=s_perf_ready[numbers[1]];
                if(ready){
                    s_perf_snapshot_lease.valid=true;
                    s_perf_snapshot_lease.token=numbers[0];
                    s_perf_snapshot_lease.stage=(uint8_t)numbers[1];
                    s_perf_snapshot_lease.started_us=query_us;
                    s_perf_snapshot_lease.record=s_perf_cache[numbers[1]];
                    snapshot=s_perf_snapshot_lease.record;
                }
            }
        }else if(!s_perf_snapshot_lease.valid ||
                 s_perf_snapshot_lease.token!=numbers[0] ||
                 s_perf_snapshot_lease.stage!=numbers[1]){
            mismatch=true;ready=false;
        }else{
            ready=true;snapshot=s_perf_snapshot_lease.record;
        }
        portEXIT_CRITICAL(&s_resource_lock);
        if(mismatch){
            snprintf(response,capacity,"ERR PERF_SNAPSHOT token=%" PRIu32 " snapshot_lease_mismatch\n",numbers[0]);return true;
        }
        if(ready)length=dual_end_perf_format_part(&snapshot,numbers[2],json,sizeof(json));
    }
    if(!ready||!length){snprintf(response,capacity,"ERR %s token=%" PRIu32 " not_ready\n",build_info?"BUILD_INFO":(perf_snapshot?"PERF_SNAPSHOT":"TELEMETRY"),numbers[0]);return true;}
    const char *command_name=build_info?"BUILD_INFO":(resources?"TELEMETRY":(perf_snapshot?"PERF_SNAPSHOT":"PERF"));
    int n=snprintf(response,capacity,"OK %s token=%" PRIu32 " %s\n",command_name,numbers[0],json);
    if(n<0||(size_t)n>=capacity)snprintf(response,capacity,"ERR %s response_capacity\n",build_info?"BUILD_INFO":(perf_snapshot?"PERF_SNAPSHOT":"TELEMETRY"));
    return true;
}

static void runtime_logger(void *arg)
{
    (void)arg;
    static runtime_snapshot_t snapshot;
    char line[DUAL_END_PERF_JSON_BUFFER_BYTES];
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    runtime_log_identity(xPortGetCoreID());
    if(runtime_format_build_info(line,sizeof(line)))ESP_LOGI(TAG,"%s",line);
    int64_t previous_resources_us = 0;
    int64_t last_perf_log_us[DUAL_END_STAGE_COUNT] = {0};
    int64_t last_identity_us = esp_timer_get_time();
    for (;;) {
        int64_t resources_now = esp_timer_get_time();
        if(resources_now-previous_resources_us>=RUNTIME_WINDOW_US || !previous_resources_us){runtime_sample_resources();runtime_log_resources();runtime_log_transport_health();previous_resources_us=resources_now;}
        /* Identity/resource visibility must not depend on active codec frames:
         * a passive monitor can attach while capture is stopped or unpaired. */
        if(resources_now-last_identity_us>=INT64_C(30000000)) {
            runtime_log_identity(xPortGetCoreID());
            if(runtime_format_build_info(line,sizeof(line)))ESP_LOGI(TAG,"%s",line);
            last_identity_us=resources_now;
        }
        if (xQueueReceive(s_snapshots, &snapshot, pdMS_TO_TICKS(250)) != pdTRUE) continue;
        dual_end_perf_record_t *r = &snapshot.record;
        /* Resources are observed here, after queueing, not pure codec heap.
         * These owners live until reboot; their watermark is lifetime minimum. */
        r->resource_at_us = (uint64_t)esp_timer_get_time();
        r->heap_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        r->heap_min_internal = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        r->heap_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        r->stack_min = snapshot.owner ? (uint32_t)uxTaskGetStackHighWaterMark(snapshot.owner) : 0;
        portENTER_CRITICAL(&s_resource_lock);s_perf_cache[r->stage]=*r;s_perf_ready[r->stage]=true;portEXIT_CRITICAL(&s_resource_lock);
        /* A resync closes a window every ~300 ms. Keep every latest window
         * available to PERF, but bound console output independently of session
         * churn: diagnostics must not amplify a radio failure into CDC starvation. */
        const int64_t log_now = esp_timer_get_time();
        if (last_perf_log_us[r->stage] &&
            log_now - last_perf_log_us[r->stage] < RUNTIME_WINDOW_US) continue;
        last_perf_log_us[r->stage] = log_now;
        for (unsigned part = 0; part < DUAL_END_PERF_PART_COUNT; ++part) {
            if (dual_end_perf_format_part(r, part, line, sizeof(line))) ESP_LOGI(TAG, "%s", line);
            /* Bound USB-console bursts on Core0; never sleep the producer. */
            vTaskDelay(1);
        }
        if (r->stage == DUAL_END_STAGE_MIC_PIPELINE) {
            for (unsigned part = 0; part < DUAL_END_OWNER_OUTCOME_PART_COUNT; ++part) {
                if (dual_end_owner_outcome_format_part(r, part, line, sizeof(line))) ESP_LOGI(TAG, "%s", line);
                vTaskDelay(1);
            }
        }
    }
}
esp_err_t dual_end_runtime_start(scepter_role_t role)
{
    if (s_attempted) return atomic_load_explicit(&s_enabled, memory_order_acquire) ? ESP_OK : ESP_ERR_INVALID_STATE;
    s_attempted = true;
    s_role = role;
    /* Reuse the pairing-owned USB driver. Optional sink failure retains the
     * original backend and leaves audio, communications and OTA usable. */
    const esp_err_t transport_error=dual_end_log_transport_start(role);
    if(transport_error!=ESP_OK)ESP_LOGW(TAG,"USB diagnostic sink unavailable: %s",esp_err_to_name(transport_error));
    if(!s_boot_id) do { s_boot_id=esp_random(); } while(!s_boot_id);
    const esp_app_desc_t *description=esp_app_get_description();
    for(unsigned i=0;i<32U;++i)
        snprintf(s_elf_sha256+2U*i,3,"%02x",(unsigned)description->app_elf_sha256[i]);
    esp_chip_info(&s_chip);
    snprintf(s_fw, sizeof(s_fw), "%s", esp_app_get_description()->version);
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(s_device_id, sizeof(s_device_id), "%s-%02x%02x%02x%02x%02x%02x",
            role == SCEPTER_ROLE_SCEPTER ? "SCEPTER" : "DONGLE",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else snprintf(s_device_id, sizeof(s_device_id), "unknown");
    /* Build evidence does not depend on optional profiling allocation. */
    atomic_store_explicit(&s_identity_ready,true,memory_order_release);
    s_snapshots = xQueueCreate(RUNTIME_QUEUE_DEPTH, sizeof(runtime_snapshot_t));
    if (s_snapshots) {
        TaskHandle_t logger = NULL;
        /* The logger never owns DMA or the radio driver. Its 6 KiB stack must
         * not consume the internal heap needed by ESP-NOW packet allocation. */
#if CONFIG_SPIRAM && CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
        BaseType_t created = xTaskCreatePinnedToCoreWithCaps(runtime_logger,
            "codec_perf_log", RUNTIME_LOGGER_STACK_BYTES, NULL,
            RUNTIME_LOGGER_PRIORITY, &logger, DUAL_END_COMM_TASK_CORE,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
        BaseType_t created = xTaskCreatePinnedToCore(runtime_logger,
            "codec_perf_log", RUNTIME_LOGGER_STACK_BYTES, NULL,
            RUNTIME_LOGGER_PRIORITY, &logger, DUAL_END_COMM_TASK_CORE);
#endif
        if (created != pdPASS) {
            vQueueDelete(s_snapshots);
            s_snapshots = NULL;
        } else {
            /* The audio owner may already run. Publish only fully prepared
             * lifetime resources; failure never exposes a queue then deletes it. */
            atomic_store_explicit(&s_enabled, true, memory_order_release);
            xTaskNotifyGive(logger);
        }
    }
    if (!atomic_load_explicit(&s_enabled, memory_order_acquire)) {
        runtime_log_identity(-1);
        char build_line[DUAL_END_PERF_JSON_BUFFER_BYTES];
        if(runtime_format_build_info(build_line,sizeof(build_line)))ESP_LOGI(TAG,"%s",build_line);
        ESP_LOGW(TAG, "profiling_disabled: snapshot/logger allocation failed; audio unchanged");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
/* Different stages may close on different producer tasks. Allocate their
 * shared wire identity atomically without locking or scanning owner windows.
 * Zero stays reserved, including at the uint32 wrap boundary. */
static uint32_t runtime_next_window_id(void)
{
    unsigned value=atomic_load_explicit(&s_window_seq,memory_order_relaxed);
    for(;;){
        const unsigned next=value==UINT32_MAX ? 1U : value+1U;
        if(atomic_compare_exchange_weak_explicit(&s_window_seq,&value,next,
            memory_order_relaxed,memory_order_relaxed))return (uint32_t)next;
    }
}

/* Saturation must be one atomic transition: a load/check followed by fetch-add
 * could wrap when two stage owners both observed UINT32_MAX-1. */
static void runtime_note_telemetry_drop(void)
{
    unsigned value=atomic_load_explicit(&s_telemetry_drop,memory_order_relaxed);
    while(value!=UINT32_MAX && !atomic_compare_exchange_weak_explicit(
        &s_telemetry_drop,&value,value+1U,memory_order_relaxed,memory_order_relaxed)){}
}

static void runtime_publish(runtime_window_t *w, dual_end_perf_stage_t stage)
{
    if (!w->stats.samples && !w->stats.invalid_duration) return;
    runtime_snapshot_t snapshot = {
        .record = {.role = (uint8_t)s_role, .stage = (uint8_t)stage,
            .boot_id = s_boot_id, .window_seq = runtime_next_window_id(),
            .start_us = (uint64_t)w->started_us, .end_us = (uint64_t)w->ended_us,
            .stats = w->stats, .telemetry_drop =
                atomic_load_explicit(&s_telemetry_drop,memory_order_relaxed)},
        .owner = w->owner};
    /* Diagnostics must never stall the audio owner or steal recovery slots. */
    if (xQueueSend(s_snapshots, &snapshot, 0) != pdTRUE)
        runtime_note_telemetry_drop();
}
void dual_end_runtime_record(dual_end_perf_stage_t stage, uint8_t codec,
    uint32_t session_id, int64_t elapsed_us, bool success)
{
    dual_end_runtime_record_outcome(stage, codec, session_id, elapsed_us,
        success ? DUAL_END_OUTCOME_OK : DUAL_END_OUTCOME_OPERATION_FAILED);
}
void dual_end_runtime_record_outcome(dual_end_perf_stage_t stage, uint8_t codec,
    uint32_t session_id, int64_t elapsed_us, dual_end_owner_outcome_t outcome)
{
    if (!atomic_load_explicit(&s_enabled, memory_order_acquire) || !s_snapshots || stage >= DUAL_END_STAGE_COUNT || (int)stage < 0) return;
    runtime_window_t *w = &s_windows[stage];
    const int64_t now = esp_timer_get_time();
    if (w->active && (w->stats.codec != codec || w->stats.session_id != session_id)) {
        runtime_publish(w, stage);
        w->active = false;
    }
    if (!w->active) {
        dual_end_perf_reset(&w->stats, codec, session_id, DUAL_END_AUDIO_TASK_CORE);
        w->started_us = elapsed_us >= 0 && elapsed_us < now ? now - elapsed_us : now;
        w->owner = xTaskGetCurrentTaskHandle();
        w->active = true;
    }
    (void)dual_end_perf_record_outcome(&w->stats, elapsed_us, xPortGetCoreID(), outcome);
    w->ended_us = now;
    if (now - w->started_us >= RUNTIME_WINDOW_US) {
        runtime_publish(w, stage);
        w->active = false;
    }
}

uint32_t dual_end_runtime_identity_boot_id(void)
{
    return atomic_load_explicit(&s_identity_ready,memory_order_acquire)?s_boot_id:0;
}

uint32_t dual_end_runtime_boot_id(void)
{
    return atomic_load_explicit(&s_enabled,memory_order_acquire)?s_boot_id:0;
}

bool dual_end_runtime_cache_identity(dual_end_runtime_cache_identity_t *out)
{
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    if (!atomic_load_explicit(&s_enabled, memory_order_acquire)) return false;

    /* All fields below originate from caches written by the low-priority
     * logger. Do not reach into s_windows here: those remain owner-only. */
    portENTER_CRITICAL(&s_resource_lock);
    out->runtime_ready = true;
    out->resource_ready = s_resource_ready;
    out->role = (uint8_t)s_role;
    out->boot_id = s_boot_id;
    if (s_resource_ready) {
        out->resource_sequence = s_resource_cache.sequence;
        out->resource_sampled_us = s_resource_cache.sampled_us;
    }
    for (unsigned stage = 0; stage < DUAL_END_STAGE_COUNT; ++stage) {
        if (!s_perf_ready[stage]) continue;
        out->perf_ready_mask |= (uint8_t)(1U << stage);
        out->perf_window_sequence[stage] = s_perf_cache[stage].window_seq;
        out->perf_session_id[stage] = s_perf_cache[stage].stats.session_id;
        out->perf_codec[stage] = s_perf_cache[stage].stats.codec;
    }
    portEXIT_CRITICAL(&s_resource_lock);
    return true;
}

bool dual_end_runtime_owner_context(dual_end_perf_stage_t stage,
    uint32_t *session_id,uint8_t *codec,uint64_t *sampled_us)
{
    if(!session_id||!codec||!sampled_us||stage>=DUAL_END_STAGE_COUNT||(int)stage<0||
       !atomic_load_explicit(&s_enabled,memory_order_acquire))return false;
    /* Same-stage producer storage: no mutex, queue, formatting or clock query.
     * Closing a window preserves its final context until the next record. */
    const runtime_window_t *window=&s_windows[stage];
    if(!window->owner||window->ended_us<=0)return false;
    *session_id=window->stats.session_id;*codec=window->stats.codec;
    *sampled_us=(uint64_t)window->ended_us;
    return *session_id!=0;
}
