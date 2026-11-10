#include "dual_end_espnow_link.h"
#include "dual_end_runtime.h"
#include "dual_end_secure_pairing.h"
#include "dual_end_audio_recovery_protocol.h"
#include "audio_control_retry_policy.h"
#include "audio_tx_policy.h"
#include "audio_radio_recovery_policy.h"
#include "dual_end_channel_guard.h"
#include "audio_recovery_tx_transport.h"
#include "wifi_idle_policy.h"

#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "scepter_espnow";

_Static_assert(DUAL_END_AUDIO_PACKET_BYTES > ESP_NOW_MAX_DATA_LEN,
               "audio packet must be transmitted as ESP-NOW v2");
_Static_assert(DUAL_END_AUDIO_PACKET_BYTES <= ESP_NOW_MAX_DATA_LEN_V2,
               "audio packet exceeds the ESP-IDF ESP-NOW v2 limit");
_Static_assert(DUAL_END_ESPNOW_MAX_DATA_BYTES <= ESP_NOW_MAX_DATA_LEN_V2,
               "framework ESP-NOW buffer exceeds the ESP-IDF v2 limit");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define DUAL_END_WIFI_BANDWIDTH_20MHZ WIFI_BW20
#else
#define DUAL_END_WIFI_BANDWIDTH_20MHZ WIFI_BW_HT20
#endif

#define DUAL_END_HELLO_DISCONNECTED_INTERVAL_MS 1000
#define DUAL_END_HELLO_CONNECTED_INTERVAL_MS 2000
#define DUAL_END_LINK_TIMEOUT_MS 7000
#define DUAL_END_LINK_TASK_STACK_BYTES 4096
#define DUAL_END_LINK_TASK_PRIORITY 4
#define DUAL_END_TX_TASK_STACK_BYTES 6144
#define DUAL_END_TX_TASK_PRIORITY 6
#define DUAL_END_TX_SLOT_COUNT 8
#define DUAL_END_TX_AUDIO_PENDING_COUNT (DUAL_END_TX_SLOT_COUNT - 1)
#define DUAL_END_TX_MAX_DATA_BYTES DUAL_END_ESPNOW_MAX_DATA_BYTES
#define DUAL_END_TX_CALLBACK_SOFT_TIMEOUT_MS 100
#define DUAL_END_TX_CALLBACK_HARD_TIMEOUT_MS ((int)(AUDIO_RADIO_HARD_TIMEOUT_US / 1000))
/* Only an immediate NO_MEM rejection may be resubmitted; never a MAC failure. */
#define DUAL_END_TX_AUDIO_NO_MEM_RETRY_COUNT 1
#define DUAL_END_TX_INPUT_RETRY_COUNT 2
#define DUAL_END_TX_AUDIO_NO_MEM_RETRY_DELAY_MS 1
#define DUAL_END_TX_DIAG_INTERVAL_MS 30000
#define DUAL_END_TX_LATENCY_BUCKET_COUNT 7
/* Jitter allowance around the 1 Hz idle input heartbeat. */
#define DUAL_END_HELLO_DATA_SUPPRESS_MS 1250
/* 信道协商期间暂停音频；干扰判定只统计音频发送回调，避免控制包影响结果。 */
#define DUAL_END_CHANNEL_EVENT_QUEUE_DEPTH 8
#define DUAL_END_CHANNEL_TASK_POLL_MS 50
#define DUAL_END_CHANNEL_SWITCH_DELAY_MS 600
#define DUAL_END_CHANNEL_SWITCH_MIN_DELAY_MS 300
#define DUAL_END_CHANNEL_SWITCH_MAX_DELAY_MS 1500
#define DUAL_END_CHANNEL_CONFIRM_TIMEOUT_MS 5000
#define DUAL_END_CHANNEL_PAUSE_TIMEOUT_MS 4000
#define DUAL_END_CHANNEL_INTERFERENCE_WINDOW_MS 2000
#define DUAL_END_CHANNEL_INTERFERENCE_MIN_PACKETS 20
#define DUAL_END_CHANNEL_INTERFERENCE_MIN_FAILURES 8
#define DUAL_END_CHANNEL_INTERFERENCE_PERCENT 30
#define DUAL_END_CHANNEL_RESCAN_COOLDOWN_MS 30000
#define PAIRING_NVS_NAMESPACE "pairing"
#define LINK_CONFIG_NVS_NAMESPACE "link_cfg"
#define CHANNEL_NVS_KEY "channel"

typedef enum {
    DUAL_END_TX_KIND_UNKNOWN = 0,
    DUAL_END_TX_KIND_AUDIO,
    DUAL_END_TX_KIND_HELLO,
    DUAL_END_TX_KIND_OTA,
    DUAL_END_TX_KIND_AUDIO_REPAIR,
    DUAL_END_TX_KIND_AUDIO_CONTROL,
    DUAL_END_TX_KIND_INPUT,
    DUAL_END_TX_KIND_CONTROL,
    DUAL_END_TX_KIND_COUNT,
} dual_end_tx_kind_t;

typedef struct {
    uint8_t data[DUAL_END_TX_MAX_DATA_BYTES];
    size_t len;
    dual_end_tx_kind_t kind;
    esp_err_t result;
    SemaphoreHandle_t done;
    bool synchronous;
    int64_t created_us;
    int64_t queued_us;
    int64_t deadline_us;
    uint32_t reservation_id;
    uint32_t audio_generation;
    uint32_t radio_generation;
} dual_end_tx_request_t;
typedef enum {
    TX_PIPE_QUEUE, TX_PIPE_PEER, TX_PIPE_PREPARE, TX_PIPE_API,
    TX_PIPE_CALLBACK, TX_PIPE_WAKE, TX_PIPE_COUNT
} dual_end_tx_pipe_stage_t;
typedef struct {
    uint64_t count;
    uint32_t max_us;
} dual_end_tx_pipe_stat_t;

typedef struct {
    uint32_t done;
    uint32_t ok;
    uint32_t fail;
    uint32_t consecutive_fail;
    uint32_t fail_streak_start_ms;
    uint32_t last_fail_streak;
    uint32_t last_fail_streak_ms;
    uint32_t max_fail_streak;
    uint32_t max_fail_streak_ms;
} dual_end_tx_kind_stats_t;

/* Completed esp_now_send calls, separated by application attempt type. Legacy
 * fast_retry fields now count only resubmissions after immediate NO_MEM. Driver
 * internal RF retries are not exposed by ESP-NOW. A MAC ACK is not an RX decode. */
typedef struct {
    uint64_t calls;
    uint64_t submitted;
    uint64_t mac_ok;
    uint64_t mac_fail;
    uint64_t immediate_fail;
    uint64_t unresolved;
} dual_end_tx_attempt_stats_t;

typedef struct {
    uint8_t command;
    uint8_t channel;
    uint8_t flags;
    uint32_t generation;
    uint16_t switch_delay_ms;
    int16_t channel_score;
    uint32_t received_ms;
} dual_end_channel_event_t;

static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN];
static bool s_peer_valid;
static bool s_peer_from_nvs;
static dual_end_pairing_active_record_t s_active_pairing;
static bool s_active_pairing_valid;
static dual_end_espnow_recv_cb_t s_recv_cb;
static void *s_recv_ctx;
static dual_end_espnow_status_cb_t s_status_cb;
static void *s_status_ctx;
static scepter_role_t s_role;
static TaskHandle_t s_link_task;
static TaskHandle_t s_tx_task;
static QueueHandle_t s_tx_free_queue;
static QueueHandle_t s_tx_priority_queue;
static QueueHandle_t s_tx_audio_queue;
static QueueHandle_t s_tx_input_queue;
static QueueHandle_t s_channel_event_queue;
static SemaphoreHandle_t s_tx_wake_sem;
static SemaphoreHandle_t s_tx_callback_sem;
/* Serializes producer ownership transfer from the shared free-slot queue into
 * a work queue. The worker never takes it and may return slots concurrently. */
static SemaphoreHandle_t s_tx_submit_mutex;
static SemaphoreHandle_t s_peer_mutex;
static dual_end_tx_request_t s_tx_slots[DUAL_END_TX_SLOT_COUNT];
static volatile bool s_radio_suspended;
static audio_radio_state_t s_radio_state;
static audio_radio_reason_t s_radio_reason;
static bool s_radio_forced_pending;
static bool s_radio_recovery_dirty;
static uint32_t s_radio_generation;
static uint32_t s_radio_early_count;
static uint32_t s_radio_hard_count;
static uint32_t s_radio_forced_count;
static uint32_t s_radio_success_count;
static uint32_t s_radio_fail_count;
static uint32_t s_radio_api_last_us;
static uint32_t s_radio_api_max_us;
static uint32_t s_radio_first_ok_last_us;
static uint32_t s_radio_first_ok_max_us;
static int64_t s_radio_last_recovery_us;
static int64_t s_radio_ready_us;
static bool s_radio_awaiting_first_ok;
static bool s_radio_awaiting_first_rx;
static uint32_t s_radio_first_rx_last_us;
static uint32_t s_radio_first_rx_max_us;
static esp_err_t s_radio_last_error;
/* Written under TX stats lock, emitted only by the periodic link task. */
static esp_err_t s_audio_last_immediate_error;
static uint32_t s_audio_no_mem_errors;
static uint32_t s_audio_control_submit_failures;
static uint32_t s_audio_control_no_mem_retries;
static esp_err_t s_audio_control_last_submit_error;

static wifi_config_t s_radio_sta_config;
/* A runtime RF pause preserves the logical link and every admitted TX slot.
 * Only the formal sleep API owns s_radio_suspended and disconnect callbacks. */
static volatile bool s_idle_wifi_stopped;
static dual_end_wifi_idle_policy_t s_idle_policy;
static portMUX_TYPE s_idle_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_idle_stop_count;
static uint32_t s_idle_resume_count;
static uint32_t s_idle_fault_count;
static uint32_t s_idle_resume_max_us;
static volatile bool s_connected;
static bool s_reported_connected;
static int s_reported_rssi;
static int64_t s_last_seen_us;
static int s_last_rssi;
static uint32_t s_hello_sequence;
static uint32_t s_hello_no_mem_count;
static uint32_t s_hello_not_found_count;
static volatile uint32_t s_last_data_submit_ms;
static volatile uint32_t s_tx_done_count;
static volatile uint32_t s_tx_ok_count;
static volatile uint32_t s_tx_fail_count;
static volatile uint32_t s_tx_accepted_count;
static volatile uint32_t s_tx_immediate_fail_count;
static volatile uint32_t s_tx_callback_soft_timeout_count;
static volatile uint32_t s_tx_callback_late_count;
static volatile uint32_t s_tx_callback_hard_recovery_count;
static volatile uint32_t s_tx_callback_recovery_fail_count;
static volatile uint32_t s_tx_callback_orphan_count;
static volatile uint32_t s_tx_callback_latency_last_us;
static volatile uint32_t s_tx_callback_latency_max_us;
static volatile uint32_t s_tx_queue_high_water;
static uint32_t s_tx_callback_latency_buckets[DUAL_END_TX_LATENCY_BUCKET_COUNT];
static uint32_t s_tx_callback_success_over_us[3];
static uint32_t s_tx_callback_failure_over_us[3];
static uint32_t s_tx_queue_wait_max_us;
static uint32_t s_tx_audio_submit_age_max_us;
static uint32_t s_tx_audio_mac_success_after_deadline;
static uint32_t s_tx_audio_fast_retry_suppressed;
static audio_tx_pacing_t s_tx_audio_pacing;
static volatile uint32_t s_tx_audio_submit_count;
static volatile uint32_t s_tx_audio_submit_fail_count;
static volatile uint32_t s_tx_audio_stale_drop_count;
static volatile uint32_t s_tx_ota_submit_count;
static volatile uint32_t s_tx_ota_submit_fail_count;
static volatile int s_last_send_status;
static volatile dual_end_tx_kind_t s_tx_inflight_kind;
static volatile bool s_tx_callback_armed;
static volatile bool s_tx_soft_timeout_active;
static volatile int64_t s_tx_inflight_start_us;
static int64_t s_tx_callback_enter_us;
static dual_end_tx_pipe_stat_t s_tx_pipe[TX_PIPE_COUNT];
static volatile uint8_t s_current_channel;
static int64_t s_channel_switched_us;
static bool s_channel_configuration_dirty;
static volatile bool s_channel_audio_paused;
static volatile bool s_channel_peer_seen_after_switch;
static volatile bool s_channel_rescan_requested;
static uint8_t s_channel_pending_channel;
static uint32_t s_channel_pending_generation;
static dual_end_channel_guard_t s_channel_guard;
static uint32_t s_channel_switch_due_ms;
static uint32_t s_channel_fallback_due_ms;
static uint32_t s_channel_pause_due_ms;
static uint32_t s_channel_disconnect_since_ms;
static uint32_t s_channel_last_rescan_request_ms;
static uint32_t s_channel_interference_window_start_ms;
static uint32_t s_channel_interference_packets;
static uint32_t s_channel_interference_failures;
static uint8_t s_channel_bad_windows;
static uint32_t s_channel_congestion_window_start_ms;
static uint32_t s_channel_congestion_packets;
static uint32_t s_channel_congestion_bad;
static uint8_t s_channel_congestion_bad_windows;
static uint32_t s_channel_congestion_late_count;
static uint32_t s_channel_congestion_expired_count;
static volatile uint32_t s_tx_audio_generation;
static uint8_t s_tx_deferred_input_flags;
static uint32_t s_tx_audio_final_ok;
static uint32_t s_tx_audio_final_fail;
static uint32_t s_tx_audio_retry_recovered;
static uint32_t s_tx_audio_deadline_drop;
static uint32_t s_tx_repair_ok;
static uint32_t s_tx_repair_fail;
static uint32_t s_tx_repair_deadline_drop;
typedef struct {
    bool pending;
    uint32_t sequence;
    uint32_t generation;
    uint32_t radio_generation;
    int64_t created_us;
    unsigned attempts;
} dual_end_tx_pending_final_t;
/* Owned by the sole TX worker; allow an on-time repair to settle a failed
 * original before classifying it as an exhausted frame. */
static dual_end_tx_pending_final_t s_tx_pending_final[DUAL_END_AUDIO_TX_CACHE_WINDOW];
static uint32_t s_tx_audio_repair_recovered;
static uint32_t s_channel_switch_count;
static uint32_t s_channel_fallback_count;
static uint32_t s_channel_rescan_count;
static dual_end_tx_kind_stats_t s_tx_kind_stats[DUAL_END_TX_KIND_COUNT];
static portMUX_TYPE s_tx_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static dual_end_tx_attempt_stats_t s_stability_first;
static dual_end_tx_attempt_stats_t s_stability_retry;
static dual_end_tx_attempt_stats_t s_stability_repair;
static dual_end_tx_attempt_stats_t s_audio_control_attempts;

static portMUX_TYPE s_peer_state_lock = portMUX_INITIALIZER_UNLOCKED;

static void on_espnow_send(const esp_now_send_info_t *tx_info,
                           esp_now_send_status_t status);
static void on_espnow_recv(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data,
                           int len);
static esp_err_t configure_peer(const uint8_t *peer_mac);
static void channel_reset_interference_window(void);

static portMUX_TYPE s_channel_request_lock = portMUX_INITIALIZER_UNLOCKED;
static dual_end_channel_status_t s_channel_status;
static bool s_channel_user_queued;
static uint8_t s_channel_user_target;
static bool s_channel_event_processing;
static bool s_channel_user_avoid;
static uint32_t s_channel_user_deadline_ms;

static void channel_status_update(dual_end_channel_state_t state, uint8_t target,
                                  uint32_t generation, esp_err_t result, bool pending)
{
    portENTER_CRITICAL(&s_channel_request_lock);
    if (s_channel_user_queued) {
        portEXIT_CRITICAL(&s_channel_request_lock);
        return; /* Preserve a not-yet-consumed explicit user's target/result. */
    }
    s_channel_status.state = state;
    s_channel_status.target_channel = target;
    s_channel_status.generation = generation;
    s_channel_status.last_result = result;
    s_channel_status.pending = pending;
    portEXIT_CRITICAL(&s_channel_request_lock);
}

static esp_err_t channel_store_save(uint8_t channel);
static esp_err_t send_channel_control(uint8_t command,
                                      uint8_t channel,
                                      uint8_t flags,
                                      uint32_t generation,
                                      uint16_t switch_delay_ms,
                                      int16_t channel_score);

static void tx_record_attempt(dual_end_tx_kind_t kind, bool retry,
                              esp_err_t submit_result, bool callback_known,
                              bool mac_success)
{
    if (kind != DUAL_END_TX_KIND_AUDIO && kind != DUAL_END_TX_KIND_AUDIO_REPAIR &&
        kind != DUAL_END_TX_KIND_AUDIO_CONTROL) {
        return;
    }
    portENTER_CRITICAL(&s_tx_stats_lock);
    dual_end_tx_attempt_stats_t *stats = kind == DUAL_END_TX_KIND_AUDIO_CONTROL ?
        &s_audio_control_attempts : kind == DUAL_END_TX_KIND_AUDIO_REPAIR ?
        &s_stability_repair : retry ? &s_stability_retry : &s_stability_first;
    stats->calls++;
    if (submit_result != ESP_OK) {
        stats->immediate_fail++;
    } else {
        stats->submitted++;
        if (!callback_known) {
            stats->unresolved++;
        } else if (mac_success) {
            stats->mac_ok++;
        } else {
            stats->mac_fail++;
        }
    }
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static bool peer_mac_is_valid(const uint8_t mac[ESP_NOW_ETH_ALEN])
{
    if (!mac || (mac[0] & 0x01u) != 0) {
        return false;
    }

    bool all_zero = true;
    bool all_ff = true;
    for (size_t i = 0; i < ESP_NOW_ETH_ALEN; i++) {
        all_zero = all_zero && mac[i] == 0;
        all_ff = all_ff && mac[i] == 0xff;
    }
    return !all_zero && !all_ff;
}

static esp_err_t pairing_store_save(const uint8_t peer_mac[ESP_NOW_ETH_ALEN])
{
    dual_end_pairing_active_record_t record;
    dual_end_pairing_make_legacy(&record, peer_mac);
    return dual_end_pairing_store_save_active(&record);
}

static esp_err_t pairing_store_clear(void)
{
    return dual_end_pairing_store_clear_all();
}

static bool pairing_store_load(uint8_t peer_mac[ESP_NOW_ETH_ALEN])
{
    bool migrated_legacy = false;
    if (!dual_end_pairing_store_load_active(&s_active_pairing,
                                            &migrated_legacy)) {
        return false;
    }
    s_active_pairing_valid = true;
    memcpy(peer_mac, s_active_pairing.peer_mac, ESP_NOW_ETH_ALEN);
    if (migrated_legacy) {
        ESP_LOGW(TAG, "migrated legacy plaintext pairing/peer to pairing/active");
    }
    return true;
}

static bool channel_is_valid(uint8_t channel)
{
    return channel >= DUAL_END_ESPNOW_CHANNEL_MIN &&
           channel <= DUAL_END_ESPNOW_CHANNEL_MAX;
}

/* NVS 无记录或记录非法时统一回到信道 6，确保仍能与 Dongle 重新会合。 */
static uint8_t channel_store_load(void)
{
    nvs_handle_t handle;
    uint8_t channel = DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL;
    if (nvs_open(LINK_CONFIG_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        esp_err_t ret = nvs_get_u8(handle, CHANNEL_NVS_KEY, &channel);
        nvs_close(handle);
        if (ret == ESP_OK && channel_is_valid(channel)) {
            return channel;
        }
    }

    /* v0.3.1 stored the radio channel beside pairing secrets. Migrate it to
     * its own namespace so pairing contains only active and pending. */
    if (nvs_open(PAIRING_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL;
    }
    esp_err_t ret = nvs_get_u8(handle, CHANNEL_NVS_KEY, &channel);
    nvs_close(handle);
    if (ret != ESP_OK || !channel_is_valid(channel)) {
        return DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL;
    }
    if (channel_store_save(channel) == ESP_OK &&
        nvs_open(PAIRING_NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        ret = nvs_erase_key(handle, CHANNEL_NVS_KEY);
        if (ret == ESP_OK || ret == ESP_ERR_NVS_NOT_FOUND) {
            (void)nvs_commit(handle);
        }
        nvs_close(handle);
    }
    return channel;
}

static esp_err_t channel_store_save(uint8_t channel)
{
    ESP_RETURN_ON_FALSE(channel_is_valid(channel), ESP_ERR_INVALID_ARG, TAG,
                        "invalid channel for NVS");
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(LINK_CONFIG_NVS_NAMESPACE, NVS_READWRITE, &handle),
                        TAG, "open channel NVS");
    esp_err_t ret = nvs_set_u8(handle, CHANNEL_NVS_KEY, channel);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static void peer_state_set(const uint8_t *peer_mac, bool valid, bool from_nvs)
{
    portENTER_CRITICAL(&s_peer_state_lock);
    if (valid && peer_mac) {
        memcpy(s_peer_mac, peer_mac, ESP_NOW_ETH_ALEN);
    } else {
        memset(s_peer_mac, 0, ESP_NOW_ETH_ALEN);
    }
    s_peer_valid = valid;
    s_peer_from_nvs = valid && from_nvs;
    portEXIT_CRITICAL(&s_peer_state_lock);
}

static uint32_t tx_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static void record_air_result_locked(dual_end_tx_kind_t kind,
                                     esp_now_send_status_t status,
                                     uint32_t now_ms)
{
    if (kind <= DUAL_END_TX_KIND_UNKNOWN || kind >= DUAL_END_TX_KIND_COUNT) {
        return;
    }

    dual_end_tx_kind_stats_t *stats = &s_tx_kind_stats[kind];
    stats->done++;
    if (status == ESP_NOW_SEND_SUCCESS) {
        stats->ok++;
        if (stats->consecutive_fail > 0) {
            uint32_t duration_ms = now_ms - stats->fail_streak_start_ms;
            stats->last_fail_streak = stats->consecutive_fail;
            stats->last_fail_streak_ms = duration_ms;
            if (stats->consecutive_fail > stats->max_fail_streak) {
                stats->max_fail_streak = stats->consecutive_fail;
            }
            if (duration_ms > stats->max_fail_streak_ms) {
                stats->max_fail_streak_ms = duration_ms;
            }
            stats->consecutive_fail = 0;
            stats->fail_streak_start_ms = 0;
        }
    } else {
        stats->fail++;
        if (stats->consecutive_fail == 0) {
            stats->fail_streak_start_ms = now_ms;
        }
        stats->consecutive_fail++;
        uint32_t duration_ms = now_ms - stats->fail_streak_start_ms;
        if (stats->consecutive_fail > stats->max_fail_streak) {
            stats->max_fail_streak = stats->consecutive_fail;
        }
        if (duration_ms > stats->max_fail_streak_ms) {
            stats->max_fail_streak_ms = duration_ms;
        }
    }

}

/* Exhausted originals feed the MAC-failure window. Latency/age use a separate
 * window below, so successful MAC callbacks do not conceal sustained blocking. */
static void tx_record_audio_final(bool success, unsigned accepted_attempts,
                                    uint32_t generation, uint32_t radio_generation)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
    portENTER_CRITICAL(&s_tx_stats_lock);
    if (success) {
        s_tx_audio_final_ok++;
    } else {
        s_tx_audio_final_fail++;
    }
    if (accepted_attempts == 0 || generation != s_tx_audio_generation ||
        radio_generation != s_radio_generation || s_radio_state != AUDIO_RADIO_READY) {
        portEXIT_CRITICAL(&s_tx_stats_lock);
        return;
    }
    if (s_channel_interference_window_start_ms == 0) {
        s_channel_interference_window_start_ms = now_ms;
    }
    uint32_t elapsed = now_ms - s_channel_interference_window_start_ms;
    if (elapsed >= DUAL_END_CHANNEL_INTERFERENCE_WINDOW_MS) {
        bool bad = elapsed < 2U * DUAL_END_CHANNEL_INTERFERENCE_WINDOW_MS &&
            s_channel_interference_packets >= DUAL_END_CHANNEL_INTERFERENCE_MIN_PACKETS &&
            s_channel_interference_failures >= DUAL_END_CHANNEL_INTERFERENCE_MIN_FAILURES &&
            s_channel_interference_failures * 100U >=
                s_channel_interference_packets * DUAL_END_CHANNEL_INTERFERENCE_PERCENT;
        s_channel_bad_windows = bad ? s_channel_bad_windows + 1U : 0U;
        bool cooldown = s_channel_last_rescan_request_ms == 0 ||
            now_ms - s_channel_last_rescan_request_ms >= DUAL_END_CHANNEL_RESCAN_COOLDOWN_MS;
        if (s_channel_bad_windows >= 2U && cooldown) {
            s_channel_rescan_requested = true;
            s_channel_last_rescan_request_ms = now_ms;
            s_channel_bad_windows = 0;
        }
        s_channel_interference_window_start_ms = now_ms;
        s_channel_interference_packets = 0;
        s_channel_interference_failures = 0;
    }
    s_channel_interference_packets++;
    if (!success) {
        s_channel_interference_failures++;
    }
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static void tx_record_congestion(const dual_end_tx_request_t *request,
                                unsigned accepted_attempts, bool expired,
                                uint32_t callback_latency_us)
{
    const int64_t now_us = esp_timer_get_time();
    const uint32_t now_ms = (uint32_t)(now_us / 1000LL);
    const bool age_expired = expired && !audio_tx_submit_has_budget(now_us, request->created_us);
    if (accepted_attempts == 0 && !age_expired) { return; }
    portENTER_CRITICAL(&s_tx_stats_lock);
    if (request->audio_generation != s_tx_audio_generation ||
        request->radio_generation != s_radio_generation ||
        s_radio_state != AUDIO_RADIO_READY || s_channel_audio_paused) {
        portEXIT_CRITICAL(&s_tx_stats_lock);
        return;
    }
    if (s_channel_congestion_window_start_ms == 0) {
        s_channel_congestion_window_start_ms = now_ms;
    }
    uint32_t elapsed = now_ms - s_channel_congestion_window_start_ms;
    if (elapsed >= DUAL_END_CHANNEL_INTERFERENCE_WINDOW_MS) {
        bool bad = elapsed < 2U * DUAL_END_CHANNEL_INTERFERENCE_WINDOW_MS &&
            s_channel_congestion_packets >= DUAL_END_CHANNEL_INTERFERENCE_MIN_PACKETS &&
            s_channel_congestion_bad >= DUAL_END_CHANNEL_INTERFERENCE_MIN_FAILURES &&
            s_channel_congestion_bad * 100U >=
                s_channel_congestion_packets * DUAL_END_CHANNEL_INTERFERENCE_PERCENT;
        s_channel_congestion_bad_windows = bad ? s_channel_congestion_bad_windows + 1U : 0U;
        bool cooldown = s_channel_last_rescan_request_ms == 0 ||
            now_ms - s_channel_last_rescan_request_ms >= DUAL_END_CHANNEL_RESCAN_COOLDOWN_MS;
        if (s_channel_congestion_bad_windows >= 2U && cooldown) {
            s_channel_rescan_requested = true;
            s_channel_last_rescan_request_ms = now_ms;
            s_channel_congestion_bad_windows = 0;
        }
        s_channel_congestion_window_start_ms = now_ms;
        s_channel_congestion_packets = 0;
        s_channel_congestion_bad = 0;
    }
    /* One observation per original, never one per fast retry/repair attempt. */
    bool late = callback_latency_us > 60000U ||
                (accepted_attempts != 0 && now_us - request->created_us >= DUAL_END_AUDIO_TX_MAX_AGE_US);
    s_channel_congestion_packets++;
    if (late || age_expired) { s_channel_congestion_bad++; }
    if (late) { s_channel_congestion_late_count++; }
    if (age_expired) { s_channel_congestion_expired_count++; }
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static void tx_flush_pending_finals(void)
{
    /* Historical MAC classification may wait for the retained repair cache.
     * This is not original admission age or a receiver playback guarantee. */
    int64_t now_us = esp_timer_get_time();
    for (size_t i = 0; i < DUAL_END_AUDIO_TX_CACHE_WINDOW; i++) {
        dual_end_tx_pending_final_t *pending = &s_tx_pending_final[i];
        if (pending->pending &&
            (pending->generation != s_tx_audio_generation ||
             now_us < pending->created_us ||
             now_us - pending->created_us >= DUAL_END_AUDIO_TX_CACHE_MAX_AGE_US)) {
            /* Retired-stream failures remain historical diagnostics but must
             * never influence the current stream/channel's RF window. */
            tx_record_audio_final(false, pending->attempts, pending->generation,
                                  pending->radio_generation);
            pending->pending = false;
        }
    }
}

static void tx_defer_failed_original(const dual_end_tx_request_t *request,
                                     unsigned accepted_attempts)
{
    uint32_t sequence = ((const dual_end_packet_header_t *)request->data)->sequence;
    dual_end_tx_pending_final_t *pending =
        &s_tx_pending_final[sequence % DUAL_END_AUDIO_TX_CACHE_WINDOW];
    if (pending->pending) {
        tx_record_audio_final(false, pending->attempts, pending->generation,
                              pending->radio_generation);
    }
    *pending = (dual_end_tx_pending_final_t) {
        .pending = true, .sequence = sequence,
        .generation = request->audio_generation,
        .radio_generation = request->radio_generation,
        .created_us = request->created_us, .attempts = accepted_attempts,
    };
}

static void tx_record_repair_success(const dual_end_tx_request_t *request)
{
    uint32_t sequence = ((const dual_end_packet_header_t *)request->data)->sequence;
    dual_end_tx_pending_final_t *pending =
        &s_tx_pending_final[sequence % DUAL_END_AUDIO_TX_CACHE_WINDOW];
    if (pending->pending && pending->sequence == sequence &&
        pending->generation == request->audio_generation &&
        request->deadline_us > 0 && esp_timer_get_time() <= request->deadline_us) {
        /* A timely MAC callback is still not proof of receiver playout. */
        tx_record_audio_final(true, 1, request->audio_generation, pending->radio_generation);
        portENTER_CRITICAL(&s_tx_stats_lock);
        s_tx_audio_repair_recovered++;
        portEXIT_CRITICAL(&s_tx_stats_lock);
        pending->pending = false;
    }
}

static size_t tx_latency_bucket(uint32_t latency_us)
{
    static const uint32_t upper_bounds_us[DUAL_END_TX_LATENCY_BUCKET_COUNT - 1] = {
        5000, 10000, 20000, 50000, 100000, 500000,
    };
    for (size_t i = 0; i < DUAL_END_TX_LATENCY_BUCKET_COUNT - 1; i++) {
        if (latency_us <= upper_bounds_us[i]) {
            return i;
        }
    }
    return DUAL_END_TX_LATENCY_BUCKET_COUNT - 1;
}

static uint32_t tx_queue_depth(void)
{
    uint32_t depth = 0;
    if (s_tx_priority_queue != NULL) {
        depth += (uint32_t)uxQueueMessagesWaiting(s_tx_priority_queue);
    }
    if (s_tx_audio_queue != NULL) {
        depth += (uint32_t)uxQueueMessagesWaiting(s_tx_audio_queue);
    }
    if (s_tx_input_queue != NULL) {
        depth += (uint32_t)uxQueueMessagesWaiting(s_tx_input_queue);
    }
    return depth;
}

static void tx_update_queue_high_water(void)
{
    uint32_t depth = tx_queue_depth();
    portENTER_CRITICAL(&s_tx_stats_lock);
    if (depth > s_tx_queue_high_water) {
        s_tx_queue_high_water = depth;
    }
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static void tx_pipe_note(dual_end_tx_pipe_stage_t stage, int64_t elapsed_us)
{
    const uint32_t duration = elapsed_us > UINT32_MAX ? UINT32_MAX :
                              elapsed_us > 0 ? (uint32_t)elapsed_us : 0;
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_tx_pipe[stage].count++;
    if (duration > s_tx_pipe[stage].max_us) { s_tx_pipe[stage].max_us = duration; }
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static bool tx_request_has_budget(const dual_end_tx_request_t *request, int64_t now_us)
{
    if (request->kind == DUAL_END_TX_KIND_AUDIO_REPAIR) {
        return request->deadline_us > 0 && now_us <= request->deadline_us -
               DUAL_END_AUDIO_REPAIR_SEND_RESERVE_US;
    }
    return request->kind == DUAL_END_TX_KIND_AUDIO ?
        audio_tx_submit_has_budget(now_us, request->created_us) :
        dual_end_audio_deadline_valid(now_us, request->created_us);
}

static void tx_arm_callback(dual_end_tx_kind_t kind)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_last_send_status = -1;
    s_tx_inflight_kind = kind;
    s_tx_inflight_start_us = esp_timer_get_time();
    s_tx_callback_enter_us = 0;
    s_tx_soft_timeout_active = false;
    s_tx_callback_armed = true;
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static void tx_note_send_accepted(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_tx_accepted_count++;
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static void tx_note_immediate_failure(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_tx_immediate_fail_count++;
    s_tx_callback_armed = false;
    s_tx_soft_timeout_active = false;
    s_tx_inflight_start_us = 0;
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static bool tx_mark_soft_timeout(void)
{
    bool still_armed;
    portENTER_CRITICAL(&s_tx_stats_lock);
    still_armed = s_tx_callback_armed;
    if (still_armed && !s_tx_soft_timeout_active) {
        s_tx_soft_timeout_active = true;
        s_tx_callback_soft_timeout_count++;
    }
    portEXIT_CRITICAL(&s_tx_stats_lock);
    return still_armed;
}

static bool tx_cancel_callback_debt(void)
{
    bool cancelled;
    portENTER_CRITICAL(&s_tx_stats_lock);
    cancelled = s_tx_callback_armed;
    if (cancelled) {
        s_tx_callback_armed = false;
    }
    portEXIT_CRITICAL(&s_tx_stats_lock);
    return cancelled;
}

static void tx_finish_inflight(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_tx_callback_armed = false;
    s_tx_soft_timeout_active = false;
    s_tx_inflight_start_us = 0;
    s_tx_inflight_kind = DUAL_END_TX_KIND_UNKNOWN;
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static bool radio_can_submit(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    bool ready = !s_radio_suspended && !s_radio_forced_pending &&
                 audio_radio_can_submit(s_radio_state);
    portEXIT_CRITICAL(&s_tx_stats_lock);
    return ready;
}

static void radio_set_state(audio_radio_state_t state)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_radio_state = state;
    s_radio_recovery_dirty = true;
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static esp_err_t radio_request_recovery(void)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_tx_stats_lock);
    bool allowed = s_tx_task != NULL && !s_radio_suspended && !s_radio_forced_pending &&
        (s_radio_state == AUDIO_RADIO_READY || s_radio_state == AUDIO_RADIO_FAILED_OFF) &&
        audio_radio_manual_recovery_allowed(now_us, s_radio_last_recovery_us);
    if (allowed) {
        s_radio_forced_pending = true;
        s_radio_recovery_dirty = true;
    }
    portEXIT_CRITICAL(&s_tx_stats_lock);
    if (!allowed) { return ESP_ERR_INVALID_STATE; }
    xSemaphoreGive(s_tx_wake_sem);
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_request_radio_recovery(void)
{
    return radio_request_recovery();
}

/* Called only by the TX owner with peer_mutex held. Wi-Fi deinit stops the old
 * Wi-Fi task; clearing armed/generation alone is never a callback barrier.
 * Netif and the default event loop live across this driver lifecycle. */
static esp_err_t radio_rebuild_driver_locked(void)
{
    uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0};
    bool peer_valid;
    portENTER_CRITICAL(&s_peer_state_lock);
    memcpy(peer_mac, s_peer_mac, sizeof(peer_mac));
    peer_valid = s_peer_valid;
    portEXIT_CRITICAL(&s_peer_state_lock);

    radio_set_state(AUDIO_RADIO_RECOVERING);
    esp_err_t ret = esp_now_unregister_send_cb();
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_NOT_INIT) { goto failed; }
    ret = esp_now_unregister_recv_cb();
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_NOT_INIT) { goto failed; }
    ret = esp_now_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_NOT_INIT) { goto failed; }
    ret = esp_wifi_stop();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_STARTED && ret != ESP_ERR_WIFI_NOT_INIT) {
        goto failed;
    }
    ret = esp_wifi_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT) { goto failed; }

    /* This drain is after stopping the old Wi-Fi task, never a substitute for
     * that lifecycle boundary. New callbacks remain gated until READY. */
    (void)xSemaphoreTake(s_tx_callback_sem, 0);
    radio_set_state(AUDIO_RADIO_RESTORING);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) { goto failed; }
    ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (ret != ESP_OK) { goto failed; }
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) { goto failed; }
    ret = esp_wifi_set_config(WIFI_IF_STA, &s_radio_sta_config);
    if (ret != ESP_OK) { goto failed; }
    ret = esp_wifi_start();
    if (ret != ESP_OK) { goto failed; }
    ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ret != ESP_OK) { goto failed; }
    ret = esp_wifi_set_bandwidth(WIFI_IF_STA, DUAL_END_WIFI_BANDWIDTH_20MHZ);
    if (ret != ESP_OK) { goto failed; }
    ret = esp_wifi_set_channel(s_current_channel, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) { goto failed; }
    ret = esp_wifi_set_max_tx_power(SCEPTER_ESPNOW_TX_POWER_QDBM);
    if (ret != ESP_OK) { goto failed; }
    ret = esp_now_init();
    if (ret != ESP_OK) { goto failed; }
    ret = esp_now_register_send_cb(on_espnow_send);
    if (ret != ESP_OK) { goto failed; }
    ret = esp_now_register_recv_cb(on_espnow_recv);
    if (ret != ESP_OK) { goto failed; }
    if (peer_valid) { ret = configure_peer(peer_mac); }
    if (ret == ESP_OK) { return ESP_OK; }
failed:
    /* Best effort teardown, but never reopen the submission gate on failure. */
    (void)esp_now_unregister_send_cb();
    (void)esp_now_unregister_recv_cb();
    (void)esp_now_deinit();
    (void)esp_wifi_stop();
    (void)esp_wifi_deinit();
    return ret;
}

static esp_err_t tx_recover_radio_locked(audio_radio_reason_t reason)
{
    const int64_t started_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_radio_state = AUDIO_RADIO_QUIESCING;
    s_radio_reason = reason;
    s_radio_forced_pending = false;
    s_radio_generation++; /* Diagnostic/window epoch, not an SDK callback ID. */
    s_radio_last_recovery_us = started_us;
    s_radio_awaiting_first_ok = false;
    s_radio_awaiting_first_rx = false;
    s_radio_recovery_dirty = true;
    if (reason == AUDIO_RADIO_REASON_EARLY) { s_radio_early_count++; }
    if (reason == AUDIO_RADIO_REASON_HARD) {
        s_radio_hard_count++;
        s_tx_callback_hard_recovery_count++;
    }
    if (reason == AUDIO_RADIO_REASON_FORCED) { s_radio_forced_count++; }
    portEXIT_CRITICAL(&s_tx_stats_lock);
    (void)tx_cancel_callback_debt();
    channel_reset_interference_window();
    esp_err_t ret = radio_rebuild_driver_locked();
    tx_finish_inflight();
    /* A full recovery also wakes a previously duty-cycled radio. Keep a receive
     * window open; a failed rebuild remains closed by s_radio_state. */
    s_idle_wifi_stopped = false;
    portENTER_CRITICAL(&s_idle_lock);
    dual_end_wifi_idle_resumed(&s_idle_policy, esp_timer_get_time());
    s_idle_policy.hold_until_us = esp_timer_get_time() + DUAL_END_WIFI_IDLE_FAULT_HOLD_US;
    portEXIT_CRITICAL(&s_idle_lock);
    const int64_t ready_us = esp_timer_get_time();
    uint32_t elapsed = ready_us - started_us > UINT32_MAX ? UINT32_MAX :
                       (uint32_t)(ready_us - started_us);
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_radio_api_last_us = elapsed;
    if (elapsed > s_radio_api_max_us) { s_radio_api_max_us = elapsed; }
    s_radio_last_error = ret;
    if (ret == ESP_OK) {
        s_radio_success_count++;
        s_radio_ready_us = ready_us;
        s_radio_first_ok_last_us = 0;
        s_radio_first_rx_last_us = 0;
        s_radio_awaiting_first_ok = true;
        s_radio_awaiting_first_rx = true;
        s_radio_state = AUDIO_RADIO_READY;
    } else {
        s_radio_fail_count++;
        s_tx_callback_recovery_fail_count++;
        s_radio_state = AUDIO_RADIO_FAILED_OFF;
    }
    s_radio_recovery_dirty = true;
    portEXIT_CRITICAL(&s_tx_stats_lock);
    return ret;
}

static void radio_run_forced_if_pending_locked(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    bool run = s_radio_forced_pending && !s_radio_suspended;
    if (s_radio_suspended) { s_radio_forced_pending = false; }
    portEXIT_CRITICAL(&s_tx_stats_lock);
    /* Sleep may win the peer mutex after USB accepted the one-shot request.
     * Never restart a radio which the sleep owner has already stopped. */
    if (run) { (void)tx_recover_radio_locked(AUDIO_RADIO_REASON_FORCED); }
}

static esp_err_t tx_recover_espnow_locked(void)
{
    uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0};
    bool peer_valid;
    portENTER_CRITICAL(&s_peer_state_lock);
    memcpy(peer_mac, s_peer_mac, sizeof(peer_mac));
    peer_valid = s_peer_valid;
    portEXIT_CRITICAL(&s_peer_state_lock);

    (void)esp_now_unregister_send_cb();
    (void)esp_now_unregister_recv_cb();
    esp_err_t ret = esp_now_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_NOT_INIT) {
        return ret;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "restore wifi ps");
    ESP_RETURN_ON_ERROR(esp_wifi_set_bandwidth(WIFI_IF_STA, DUAL_END_WIFI_BANDWIDTH_20MHZ),
                        TAG, "restore wifi 20MHz");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(s_current_channel, WIFI_SECOND_CHAN_NONE),
                         TAG, "restore wifi channel");
    ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(SCEPTER_ESPNOW_TX_POWER_QDBM),
                        TAG, "restore wifi tx power");
    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "re-init esp-now");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(on_espnow_send), TAG, "restore send callback");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(on_espnow_recv), TAG, "restore recv callback");
    if (peer_valid) {
        ESP_RETURN_ON_ERROR(configure_peer(peer_mac), TAG, "restore peer after TX timeout");
    }
    (void)xSemaphoreTake(s_tx_callback_sem, 0);
    return ESP_OK;
}

static void idle_note_fault(int64_t now_us)
{
    portENTER_CRITICAL(&s_idle_lock);
    dual_end_wifi_idle_fault(&s_idle_policy, now_us);
    s_idle_fault_count++;
    portEXIT_CRITICAL(&s_idle_lock);
}

static esp_err_t idle_radio_resume_locked(void)
{
    if (!s_idle_wifi_stopped) return ESP_OK;
    if (s_radio_suspended || s_radio_state != AUDIO_RADIO_READY) return ESP_ERR_INVALID_STATE;
    int64_t started_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_idle_lock);
    bool retry_due = started_us >= s_idle_policy.retry_after_us;
    portEXIT_CRITICAL(&s_idle_lock);
    if (!retry_due) return ESP_ERR_TIMEOUT;
    esp_err_t ret = esp_wifi_start();
    if (ret == ESP_OK) ret = tx_recover_espnow_locked();
    int64_t completed_us = esp_timer_get_time();
    if (ret == ESP_OK) {
        s_idle_wifi_stopped = false;
        portENTER_CRITICAL(&s_idle_lock);
        dual_end_wifi_idle_resumed(&s_idle_policy, completed_us);
        s_idle_resume_count++;
        uint32_t elapsed_us = (uint32_t)(completed_us - started_us);
        if (elapsed_us > s_idle_resume_max_us) s_idle_resume_max_us = elapsed_us;
        portEXIT_CRITICAL(&s_idle_lock);
        if (s_idle_resume_count == 1 || s_idle_resume_count % 60 == 0) {
            ESP_LOGI(TAG, "idle RF resumed count=%" PRIu32 " restore_us=%" PRIu32
                     " max_us=%" PRIu32 " stops=%" PRIu32 " faults=%" PRIu32,
                     s_idle_resume_count, elapsed_us, s_idle_resume_max_us,
                     s_idle_stop_count, s_idle_fault_count);
        }
    } else {
        (void)esp_now_unregister_send_cb();
        (void)esp_now_unregister_recv_cb();
        (void)esp_now_deinit();
        (void)esp_wifi_stop();
        idle_note_fault(completed_us);
        ESP_LOGW(TAG, "idle RF restore failed: %s; retry in 250 ms", esp_err_to_name(ret));
    }
    return ret;
}

static bool idle_radio_busy(void)
{
    /* Slot ownership includes a producer before enqueue and a worker after
     * dequeue. Queue depth alone misses both stop-versus-first-input races. */
    return !radio_can_submit() || s_tx_free_queue == NULL ||
           uxQueueMessagesWaiting(s_tx_free_queue) != DUAL_END_TX_SLOT_COUNT ||
           s_tx_callback_armed || s_channel_audio_paused ||
           s_channel_user_queued || s_channel_status.pending || s_channel_event_processing ||
           s_channel_switch_due_ms != 0 || s_channel_pause_due_ms != 0 ||
           s_channel_fallback_due_ms != 0 || s_channel_rescan_requested ||
           (s_channel_event_queue != NULL &&
            uxQueueMessagesWaiting(s_channel_event_queue) != 0);
}

static void idle_radio_service(void)
{
    if (!radio_can_submit() || s_peer_mutex == NULL) return;
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_peer_state_lock);
    bool connected = s_peer_valid && s_last_seen_us > 0 &&
        now_us - s_last_seen_us <= (int64_t)DUAL_END_LINK_TIMEOUT_MS * 1000;
    portEXIT_CRITICAL(&s_peer_state_lock);
    bool busy = idle_radio_busy();
    portENTER_CRITICAL(&s_idle_lock);
    if (dual_end_wifi_idle_poll(&s_idle_policy, now_us)) s_idle_fault_count++;
    bool wake = dual_end_wifi_idle_must_wake(&s_idle_policy, connected, busy, now_us);
    bool stop = dual_end_wifi_idle_can_stop(&s_idle_policy, connected, busy, now_us);
    portEXIT_CRITICAL(&s_idle_lock);
    if ((!s_idle_wifi_stopped || !wake) && (s_idle_wifi_stopped || !stop)) return;
    if (xSemaphoreTake(s_peer_mutex, 0) != pdTRUE) return;
    if (!radio_can_submit()) {
        xSemaphoreGive(s_peer_mutex);
        return;
    }
    if (s_idle_wifi_stopped) {
        (void)idle_radio_resume_locked();
    } else {
        /* Recheck after acquiring the same lock as the sole TX worker. A new
         * producer can still arrive during stop; its retained slot resumes RF. */
        now_us = esp_timer_get_time();
        busy = idle_radio_busy();
        portENTER_CRITICAL(&s_idle_lock);
        stop = dual_end_wifi_idle_can_stop(&s_idle_policy, connected, busy, now_us);
        portEXIT_CRITICAL(&s_idle_lock);
        if (stop) {
            (void)esp_now_unregister_send_cb();
            (void)esp_now_unregister_recv_cb();
            esp_err_t ret = esp_now_deinit();
            if (ret == ESP_OK || ret == ESP_ERR_ESPNOW_NOT_INIT) ret = esp_wifi_stop();
            s_idle_wifi_stopped = true;
            if (ret == ESP_OK || ret == ESP_ERR_WIFI_NOT_STARTED) {
                s_idle_stop_count++;
            } else {
                /* Restore a partially stopped driver and inhibit duty cycling.
                 * The failure hold is set after the immediate rollback attempt. */
                (void)idle_radio_resume_locked();
                idle_note_fault(esp_timer_get_time());
            }
        }
    }
    xSemaphoreGive(s_peer_mutex);
}

void dual_end_espnow_link_set_idle_allowed(bool allowed, int64_t last_activity_us)
{
    portENTER_CRITICAL(&s_idle_lock);
    dual_end_wifi_idle_constraints(&s_idle_policy, allowed, last_activity_us,
                                   esp_timer_get_time());
    portEXIT_CRITICAL(&s_idle_lock);
}

void dual_end_espnow_link_hold_receive_until(int64_t deadline_us)
{
    portENTER_CRITICAL(&s_idle_lock);
    if (deadline_us > s_idle_policy.hold_until_us) {
        s_idle_policy.hold_until_us = deadline_us;
    }
    portEXIT_CRITICAL(&s_idle_lock);
}

bool dual_end_espnow_link_is_idle_stopped(void)
{
    return s_idle_wifi_stopped;
}

static bool tx_wait_for_callback(dual_end_tx_request_t *request)
{
    bool soft_noted = false;
    while (true) {
        if (xSemaphoreTake(s_tx_callback_sem,
                          pdMS_TO_TICKS(AUDIO_RADIO_WAIT_SLICE_MS)) == pdTRUE) {
            return true;
        }
        const int64_t now_us = esp_timer_get_time();
        portENTER_CRITICAL(&s_tx_stats_lock);
        int64_t started_us = s_tx_inflight_start_us;
        audio_radio_reason_t reason = audio_radio_recovery_due(
            now_us, started_us, s_radio_last_recovery_us, s_radio_forced_pending);
        portEXIT_CRITICAL(&s_tx_stats_lock);
        if (!soft_noted && now_us - started_us >=
                           DUAL_END_TX_CALLBACK_SOFT_TIMEOUT_MS * 1000LL) {
            (void)tx_mark_soft_timeout();
            soft_noted = true;
        }
        if (reason == AUDIO_RADIO_REASON_NONE) { continue; }
        /* A callback may have entered just before its semaphore was posted.
         * Complete it normally; any forced request remains for the next loop. */
        if (!tx_cancel_callback_debt() &&
            xSemaphoreTake(s_tx_callback_sem, pdMS_TO_TICKS(10)) == pdTRUE) {
            return true;
        }
        (void)tx_recover_radio_locked(reason);
        request->result = ESP_ERR_TIMEOUT; /* Submitted outcome is UNKNOWN. */
        return false;
    }
}

static dual_end_tx_request_t *tx_take_next_request(void)
{
    /* Match firmware 0.4.2: control and receiver-requested repairs share the
     * priority FIFO and are taken before paced originals.  Repair airtime is
     * already bounded by the 4-packet/100 ms rolling quota, two attempts per
     * frame, the packet deadline and the reserved final shared TX slot. */
    dual_end_tx_request_t *request = NULL;
    if (xQueueReceive(s_tx_priority_queue, &request, 0) == pdTRUE) {
        return request;
    }
    /* Release at most two original submissions in any rolling 20 ms, even
     * when a long callback releases a backlog. Control/repair may preempt. */
    if (audio_tx_pacing_wait_us(&s_tx_audio_pacing, esp_timer_get_time()) <= 0) {
        /* One-shot input keeps its protected storage, but original audio always
         * leaves in sequence order. Control messages retain their own priority. */
        dual_end_tx_request_t *audio = NULL;
        dual_end_tx_request_t *input = NULL;
        (void)xQueueReceive(s_tx_audio_queue, &audio, 0);
        (void)xQueueReceive(s_tx_input_queue, &input, 0);
        if (input != NULL && (audio == NULL || dual_end_audio_seq_before(
                ((const dual_end_packet_header_t *)input->data)->sequence,
                ((const dual_end_packet_header_t *)audio->data)->sequence))) {
            if (audio != NULL) {
                (void)xQueueSendToFront(s_tx_audio_queue, &audio, 0);
            }
            return input;
        }
        if (input != NULL) {
            (void)xQueueSendToFront(s_tx_input_queue, &input, 0);
        }
        if (audio != NULL) {
            return audio;
        }
    }
    return NULL;
}

static bool tx_is_audio(const dual_end_tx_request_t *request)
{
    return request->kind == DUAL_END_TX_KIND_AUDIO ||
           request->kind == DUAL_END_TX_KIND_AUDIO_REPAIR;
}

static void tx_retain_unsent_input(const dual_end_tx_request_t *request)
{
    if (request->kind == DUAL_END_TX_KIND_AUDIO &&
        request->len == DUAL_END_AUDIO_PACKET_BYTES) {
        const dual_end_audio_hardware_payload_t *payload =
            (const dual_end_audio_hardware_payload_t *)(request->data +
             sizeof(dual_end_packet_header_t) + sizeof(dual_end_audio_frame_header_t));
        portENTER_CRITICAL(&s_tx_stats_lock);
        s_tx_deferred_input_flags |= payload->hardware.flags;
        portEXIT_CRITICAL(&s_tx_stats_lock);
    }
}

static void tx_worker_task(void *arg)
{
    (void)arg;

    while (true) {
        tx_flush_pending_finals();
        portENTER_CRITICAL(&s_tx_stats_lock);
        bool forced = s_radio_forced_pending;
        portEXIT_CRITICAL(&s_tx_stats_lock);
        if (forced) {
            xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
            radio_run_forced_if_pending_locked();
            xSemaphoreGive(s_peer_mutex);
        }
        dual_end_tx_request_t *request = tx_take_next_request();
        if (request == NULL) {
            (void)xSemaphoreTake(s_tx_wake_sem, pdMS_TO_TICKS(5));
            continue;
        }

        /* esp_now_send() 成功后会占用唯一的回调债务，直到回调到达或
         * ESP-NOW 实例被重置。旧回调仍可能到达时，绝不能提交下一包。 */
        const int64_t dequeued_us = esp_timer_get_time();
        tx_pipe_note(TX_PIPE_QUEUE, dequeued_us - request->queued_us);
        xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
        int64_t prepare_started_us = esp_timer_get_time();
        tx_pipe_note(TX_PIPE_PEER, prepare_started_us - dequeued_us);
        /* Retain the first short input across an idle wake failure. */
        while (!s_radio_suspended && s_idle_wifi_stopped &&
               idle_radio_resume_locked() != ESP_OK) {
            xSemaphoreGive(s_peer_mutex);
            vTaskDelay(pdMS_TO_TICKS(DUAL_END_WIFI_IDLE_RETRY_US / 1000));
            xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
        }
        unsigned accepted_attempts = 0;
        bool expired = false;
        uint32_t callback_latency_us = 0;
        portENTER_CRITICAL(&s_tx_stats_lock);
        forced = s_radio_forced_pending;
        portEXIT_CRITICAL(&s_tx_stats_lock);
        if (forced) { radio_run_forced_if_pending_locked(); }
        request->radio_generation = s_radio_generation;
        if (!radio_can_submit()) {
            request->result = ESP_ERR_INVALID_STATE;
        } else if (!s_peer_valid) {
            request->result = ESP_ERR_NOT_FOUND;
        } else {
            const unsigned max_attempts = request->kind == DUAL_END_TX_KIND_AUDIO ?
                                          1U + DUAL_END_TX_AUDIO_NO_MEM_RETRY_COUNT :
                                          request->kind == DUAL_END_TX_KIND_AUDIO_CONTROL ?
                                          dual_end_audio_control_max_attempts() :
                                          request->kind == DUAL_END_TX_KIND_INPUT ?
                                          1U + DUAL_END_TX_INPUT_RETRY_COUNT : 1U;
            for (unsigned attempt = 0; attempt < max_attempts; attempt++) {
                if (attempt != 0) { prepare_started_us = esp_timer_get_time(); }
                int64_t now_us = esp_timer_get_time();
                /* Only NO_MEM retries an original audio frame. Recheck admission
                 * after its delay so it cannot displace queued fresh work. */
                if (request->kind == DUAL_END_TX_KIND_AUDIO && attempt != 0 &&
                    (!audio_tx_fast_retry_allowed(now_us, request->created_us,
                         uxQueueMessagesWaiting(s_tx_priority_queue) != 0,
                         uxQueueMessagesWaiting(s_tx_audio_queue) != 0 ||
                         uxQueueMessagesWaiting(s_tx_input_queue) != 0) ||
                     audio_tx_pacing_wait_us(&s_tx_audio_pacing, now_us) > 0)) {
                    portENTER_CRITICAL(&s_tx_stats_lock);
                    s_tx_audio_fast_retry_suppressed++;
                    portEXIT_CRITICAL(&s_tx_stats_lock);
                    break;
                }
                if ((tx_is_audio(request) || request->kind == DUAL_END_TX_KIND_AUDIO_CONTROL) &&
                    ((tx_is_audio(request) && s_channel_audio_paused) ||
                     request->audio_generation != s_tx_audio_generation ||
                     (tx_is_audio(request) && !tx_request_has_budget(request, now_us)))) {
                    request->result = ESP_ERR_TIMEOUT;
                    expired = true;
                    tx_pipe_note(TX_PIPE_PREPARE, esp_timer_get_time() - prepare_started_us);
                    break;
                }
                const dual_end_audio_repair_packet_t *repair =
                    request->kind == DUAL_END_TX_KIND_AUDIO_REPAIR ?
                    (const dual_end_audio_repair_packet_t *)request->data : NULL;
                if (repair != NULL && !dual_end_audio_repair_tx_prepare(
                        repair->session_id, repair->packet.sequence,
                        request->reservation_id, now_us)) {
                    request->result = ESP_ERR_TIMEOUT;
                    expired = true;
                    tx_pipe_note(TX_PIPE_PREPARE, esp_timer_get_time() - prepare_started_us);
                    break;
                }
                now_us = esp_timer_get_time();
                if (repair != NULL && !tx_request_has_budget(request, now_us)) {
                    dual_end_audio_repair_tx_submitted(repair->session_id,
                        repair->packet.sequence, request->reservation_id, false, now_us);
                    request->result = ESP_ERR_TIMEOUT;
                    expired = true;
                    tx_pipe_note(TX_PIPE_PREPARE, esp_timer_get_time() - prepare_started_us);
                    break;
                }
                /* prepare holds the sender lock only across this immediate
                 * driver call; no callback wait or peer-lock acquisition. */
                (void)xSemaphoreTake(s_tx_callback_sem, 0);
                tx_arm_callback(request->kind);
                const int64_t api_enter_us = esp_timer_get_time();
                request->result = esp_now_send(s_peer_mac, request->data, request->len);
                const int64_t api_exit_us = esp_timer_get_time();
                tx_pipe_note(TX_PIPE_PREPARE, api_enter_us - prepare_started_us);
                tx_pipe_note(TX_PIPE_API, api_exit_us - api_enter_us);
                if (repair != NULL) {
                    dual_end_audio_repair_tx_submitted(repair->session_id,
                        repair->packet.sequence, request->reservation_id,
                        request->result == ESP_OK, esp_timer_get_time());
                }
                if (request->result != ESP_OK) {
                    if (request->kind == DUAL_END_TX_KIND_AUDIO_CONTROL) {
                        portENTER_CRITICAL(&s_tx_stats_lock);
                        if (s_audio_control_submit_failures != UINT32_MAX) {
                            ++s_audio_control_submit_failures;
                        }
                        s_audio_control_last_submit_error = request->result;
                        portEXIT_CRITICAL(&s_tx_stats_lock);
                    }
                    if (tx_is_audio(request)) {
                        portENTER_CRITICAL(&s_tx_stats_lock);
                        s_audio_last_immediate_error = request->result;
                        if (request->result == ESP_ERR_ESPNOW_NO_MEM &&
                            s_audio_no_mem_errors != UINT32_MAX) ++s_audio_no_mem_errors;
                        portEXIT_CRITICAL(&s_tx_stats_lock);
                    }
                    tx_record_attempt(request->kind, attempt != 0,
                                      request->result, false, false);
                    tx_note_immediate_failure();
                    const bool retry_allowed =
                        request->kind == DUAL_END_TX_KIND_AUDIO_CONTROL ?
                            dual_end_audio_control_retry_allowed(
                                attempt, request->result == ESP_ERR_ESPNOW_NO_MEM) :
                            attempt + 1U < max_attempts;
                    if (request->result == ESP_ERR_ESPNOW_NO_MEM && retry_allowed &&
                        attempt + 1U < max_attempts) {
                        if (request->kind == DUAL_END_TX_KIND_AUDIO_CONTROL) {
                            portENTER_CRITICAL(&s_tx_stats_lock);
                            if (s_audio_control_no_mem_retries != UINT32_MAX) {
                                ++s_audio_control_no_mem_retries;
                            }
                            portEXIT_CRITICAL(&s_tx_stats_lock);
                        }
                        vTaskDelay(pdMS_TO_TICKS(DUAL_END_TX_AUDIO_NO_MEM_RETRY_DELAY_MS));
                        continue;
                    }
                    break;
                }
                tx_note_send_accepted();
                if (request->kind == DUAL_END_TX_KIND_INPUT) {
                    s_last_data_submit_ms = tx_now_ms();
                    portENTER_CRITICAL(&s_idle_lock);
                    dual_end_wifi_idle_submitted(&s_idle_policy, now_us);
                    portEXIT_CRITICAL(&s_idle_lock);
                }
                accepted_attempts++;
                if (request->kind == DUAL_END_TX_KIND_AUDIO) {
                    audio_tx_pacing_note_accepted(&s_tx_audio_pacing, now_us);
                }
                if (tx_is_audio(request)) {
                    int64_t wait_us = now_us - request->queued_us;
                    int64_t age_us = now_us - request->created_us;
                    portENTER_CRITICAL(&s_tx_stats_lock);
                    if (wait_us > s_tx_queue_wait_max_us) {
                        s_tx_queue_wait_max_us = wait_us > UINT32_MAX ? UINT32_MAX : (uint32_t)wait_us;
                    }
                    if (age_us > s_tx_audio_submit_age_max_us) {
                        s_tx_audio_submit_age_max_us = age_us > UINT32_MAX ? UINT32_MAX : (uint32_t)age_us;
                    }
                    portEXIT_CRITICAL(&s_tx_stats_lock);
                }

                if (!tx_wait_for_callback(request)) {
                    tx_record_attempt(request->kind, attempt != 0, ESP_OK, false, false);
                    break;
                }

                int64_t callback_enter_us;
                portENTER_CRITICAL(&s_tx_stats_lock);
                callback_enter_us = s_tx_callback_enter_us;
                portEXIT_CRITICAL(&s_tx_stats_lock);
                tx_pipe_note(TX_PIPE_CALLBACK, callback_enter_us - api_enter_us);
                tx_pipe_note(TX_PIPE_WAKE, esp_timer_get_time() - callback_enter_us);
                if (callback_enter_us - api_enter_us > callback_latency_us) {
                    callback_latency_us = callback_enter_us - api_enter_us > UINT32_MAX ?
                                          UINT32_MAX : (uint32_t)(callback_enter_us - api_enter_us);
                }
                tx_record_attempt(request->kind, attempt != 0, ESP_OK, true,
                                  s_last_send_status == ESP_NOW_SEND_SUCCESS);

                if (s_last_send_status == ESP_NOW_SEND_SUCCESS) {
                    request->result = ESP_OK;
                    if (request->kind == DUAL_END_TX_KIND_AUDIO && attempt != 0) {
                        /* The first NO_MEM call was never accepted by the driver,
                         * so accepted_attempts is still one after this recovery. */
                        portENTER_CRITICAL(&s_tx_stats_lock);
                        s_tx_audio_retry_recovered++;
                        portEXIT_CRITICAL(&s_tx_stats_lock);
                    }
                    break;
                }

                request->result = ESP_FAIL;
                /* Media MAC failure proceeds to receiver-driven repair. Short
                 * input retains 042's separate bounded delivery retries. */
                if (request->kind == DUAL_END_TX_KIND_INPUT && attempt + 1U < max_attempts) {
                    vTaskDelay(pdMS_TO_TICKS(DUAL_END_TX_AUDIO_NO_MEM_RETRY_DELAY_MS));
                    continue;
                }
                break;
            }
        }
        if (request->kind == DUAL_END_TX_KIND_INPUT && request->result != ESP_OK &&
            !s_radio_suspended) idle_note_fault(esp_timer_get_time());
        xSemaphoreGive(s_peer_mutex);
        tx_finish_inflight();

        if (request->kind == DUAL_END_TX_KIND_AUDIO_REPAIR) {
            const dual_end_audio_repair_packet_t *repair =
                (const dual_end_audio_repair_packet_t *)request->data;
            dual_end_audio_repair_tx_complete(repair->session_id,
                repair->packet.sequence, request->reservation_id,
                accepted_attempts != 0, request->result == ESP_OK, esp_timer_get_time());
        }
        if (tx_is_audio(request) && request->result == ESP_OK &&
            !dual_end_audio_deadline_valid(esp_timer_get_time(), request->created_us)) {
            /* A late MAC success is not evidence of late receiver delivery. */
            portENTER_CRITICAL(&s_tx_stats_lock);
            s_tx_audio_mac_success_after_deadline++;
            portEXIT_CRITICAL(&s_tx_stats_lock);
        }

        if (expired && tx_is_audio(request)) {
            portENTER_CRITICAL(&s_tx_stats_lock);
            if (request->kind == DUAL_END_TX_KIND_AUDIO) {
                s_tx_audio_deadline_drop++;
            } else {
                s_tx_repair_deadline_drop++;
            }
            portEXIT_CRITICAL(&s_tx_stats_lock);
        }
        if (request->kind == DUAL_END_TX_KIND_AUDIO) {
            tx_record_congestion(request, accepted_attempts, expired, callback_latency_us);
            if (accepted_attempts == 0 && request->result != ESP_OK) {
                tx_retain_unsent_input(request);
            }
            if (!expired || accepted_attempts != 0) {
                if (request->result == ESP_OK) {
                    tx_record_audio_final(true, accepted_attempts, request->audio_generation,
                                          request->radio_generation);
                } else {
                    tx_defer_failed_original(request, accepted_attempts);
                }
            }
        } else if (request->kind == DUAL_END_TX_KIND_AUDIO_REPAIR && !expired) {
            portENTER_CRITICAL(&s_tx_stats_lock);
            if (request->result == ESP_OK) {
                s_tx_repair_ok++;
            } else {
                s_tx_repair_fail++;
            }
            portEXIT_CRITICAL(&s_tx_stats_lock);
            if (request->result == ESP_OK) {
                tx_record_repair_success(request);
            }
        }

        if (request->synchronous) {
            xSemaphoreGive(request->done);
        } else {
            (void)xQueueSend(s_tx_free_queue, (const void *)&request, portMAX_DELAY);
        }
    }
}

static void tx_worker_cleanup(void)
{
    if (s_tx_task != NULL) {
        vTaskDelete(s_tx_task);
        s_tx_task = NULL;
    }
    for (int i = 0; i < DUAL_END_TX_SLOT_COUNT; i++) {
        if (s_tx_slots[i].done != NULL) {
            vSemaphoreDelete(s_tx_slots[i].done);
            s_tx_slots[i].done = NULL;
        }
    }
    if (s_tx_callback_sem != NULL) {
        vSemaphoreDelete(s_tx_callback_sem);
        s_tx_callback_sem = NULL;
    }
    if (s_tx_wake_sem != NULL) {
        vSemaphoreDelete(s_tx_wake_sem);
        s_tx_wake_sem = NULL;
    }
    if (s_tx_submit_mutex != NULL) {
        vSemaphoreDelete(s_tx_submit_mutex);
        s_tx_submit_mutex = NULL;
    }
    if (s_tx_audio_queue != NULL) {
        vQueueDelete(s_tx_audio_queue);
        s_tx_audio_queue = NULL;
    }
    if (s_tx_input_queue != NULL) {
        vQueueDelete(s_tx_input_queue);
        s_tx_input_queue = NULL;
    }
    if (s_tx_priority_queue != NULL) {
        vQueueDelete(s_tx_priority_queue);
        s_tx_priority_queue = NULL;
    }
    if (s_tx_free_queue != NULL) {
        vQueueDelete(s_tx_free_queue);
        s_tx_free_queue = NULL;
    }
}

static esp_err_t tx_worker_init(void)
{
    s_tx_free_queue = xQueueCreate(DUAL_END_TX_SLOT_COUNT, sizeof(dual_end_tx_request_t *));
    s_tx_priority_queue = xQueueCreate(DUAL_END_TX_SLOT_COUNT,
                                       sizeof(dual_end_tx_request_t *));
    s_tx_audio_queue = xQueueCreate(DUAL_END_TX_AUDIO_PENDING_COUNT,
                                    sizeof(dual_end_tx_request_t *));
    s_tx_input_queue = xQueueCreate(DUAL_END_TX_SLOT_COUNT,
                                     sizeof(dual_end_tx_request_t *));
    s_tx_wake_sem = xSemaphoreCreateBinary();
    s_tx_callback_sem = xSemaphoreCreateBinary();
    s_tx_submit_mutex = xSemaphoreCreateMutex();
    if (s_tx_free_queue == NULL || s_tx_priority_queue == NULL ||
        s_tx_audio_queue == NULL || s_tx_input_queue == NULL || s_tx_wake_sem == NULL ||
        s_tx_callback_sem == NULL || s_tx_submit_mutex == NULL) {
        tx_worker_cleanup();
        return ESP_ERR_NO_MEM;
    }

    for (int i = 0; i < DUAL_END_TX_SLOT_COUNT; i++) {
        s_tx_slots[i].done = xSemaphoreCreateBinary();
        if (s_tx_slots[i].done == NULL) {
            tx_worker_cleanup();
            return ESP_ERR_NO_MEM;
        }
        dual_end_tx_request_t *slot = &s_tx_slots[i];
        if (xQueueSend(s_tx_free_queue, (const void *)&slot, 0) != pdTRUE) {
            tx_worker_cleanup();
            return ESP_FAIL;
        }
    }

    BaseType_t task_ok = xTaskCreatePinnedToCore(tx_worker_task,
                                     "espnow_tx",
                                     DUAL_END_TX_TASK_STACK_BYTES,
                                     NULL,
                                     DUAL_END_TX_TASK_PRIORITY,
                                     &s_tx_task, DUAL_END_COMM_TASK_CORE);
    if (task_ok != pdPASS) {
        tx_worker_cleanup();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t tx_submit_and_wait(const void *payload, int len, dual_end_tx_kind_t kind)
{
    if (payload == NULL || len <= 0 || len > DUAL_END_TX_MAX_DATA_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (kind <= DUAL_END_TX_KIND_UNKNOWN || kind >= DUAL_END_TX_KIND_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!radio_can_submit()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_tx_free_queue == NULL || s_tx_priority_queue == NULL ||
        s_tx_wake_sem == NULL || s_tx_submit_mutex == NULL || s_tx_task == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    dual_end_tx_request_t *request = NULL;
    while (request == NULL) {
        dual_end_tx_request_t *available = NULL;
        if (xQueuePeek(s_tx_free_queue, &available, portMAX_DELAY) != pdTRUE) {
            return ESP_FAIL;
        }
        if (xSemaphoreTake(s_tx_submit_mutex, portMAX_DELAY) != pdTRUE) {
            return ESP_FAIL;
        }
        if (!radio_can_submit()) {
            xSemaphoreGive(s_tx_submit_mutex);
            return ESP_ERR_INVALID_STATE;
        }
        if (xQueueReceive(s_tx_free_queue, (void *)&request, 0) != pdTRUE) {
            request = NULL;
            xSemaphoreGive(s_tx_submit_mutex);
        }
    }

    memcpy(request->data, payload, (size_t)len);
    request->len = (size_t)len;
    request->kind = kind;
    request->result = ESP_FAIL;
    request->synchronous = true;
    request->created_us = esp_timer_get_time();
    request->queued_us = request->created_us;
    request->reservation_id = 0;
    request->deadline_us = 0;
    request->audio_generation = s_tx_audio_generation;
    (void)xSemaphoreTake(request->done, 0);

    if (xQueueSend(s_tx_priority_queue, (const void *)&request, portMAX_DELAY) != pdTRUE) {
        (void)xQueueSend(s_tx_free_queue, (const void *)&request, portMAX_DELAY);
        xSemaphoreGive(s_tx_submit_mutex);
        return ESP_FAIL;
    }
    xSemaphoreGive(s_tx_submit_mutex);
    tx_update_queue_high_water();
    xSemaphoreGive(s_tx_wake_sem);

    (void)xSemaphoreTake(request->done, portMAX_DELAY);
    esp_err_t result = request->result;
    (void)xQueueSend(s_tx_free_queue, (const void *)&request, portMAX_DELAY);
    return result;
}

static bool audio_payload_has_one_shot_input(const void *data, int len)
{
    const size_t payload_offset = sizeof(dual_end_packet_header_t) +
                                  sizeof(dual_end_audio_frame_header_t);
    if (!data || len < (int)(payload_offset + sizeof(dual_end_audio_hardware_payload_t))) {
        return false;
    }
    const dual_end_audio_hardware_payload_t *payload =
        (const dual_end_audio_hardware_payload_t *)((const uint8_t *)data + payload_offset);
    return payload->hardware.flags != 0;
}

static void tx_prepare_async_request(dual_end_tx_request_t *request,
                                      const void *payload,
                                      int len,
                                      dual_end_tx_kind_t kind,
                                      int64_t created_us)
{
    memcpy(request->data, payload, (size_t)len);
    request->len = (size_t)len;
    request->kind = kind;
    request->result = ESP_FAIL;
    request->synchronous = false;
    request->created_us = created_us;
    request->queued_us = esp_timer_get_time();
    request->reservation_id = 0;
    request->deadline_us = 0;
    request->audio_generation = s_tx_audio_generation;
}

static esp_err_t tx_submit_priority_async(const void *payload,
                                           int len,
                                           dual_end_tx_kind_t kind,
                                           bool may_replace_audio,
                                           int64_t created_us,
                                           uint32_t reservation_id,
                                           int64_t deadline_us)
{
    if (payload == NULL || len <= 0 || len > DUAL_END_TX_MAX_DATA_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (kind <= DUAL_END_TX_KIND_UNKNOWN || kind >= DUAL_END_TX_KIND_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!radio_can_submit()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_tx_free_queue == NULL || s_tx_priority_queue == NULL ||
        s_tx_audio_queue == NULL || s_tx_wake_sem == NULL ||
        s_tx_submit_mutex == NULL || s_tx_task == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_tx_submit_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    /* A repair must never consume the last free slot.  Session/control traffic
     * can replace ordinary queued audio, and this reservation also covers the
     * case where only repairs are present during a session transition. Every
     * producer uses s_tx_submit_mutex, so the count and receive are atomic with
     * respect to competing acquisitions on the other core. */
    if (kind == DUAL_END_TX_KIND_AUDIO_REPAIR &&
        !audio_tx_slot_may_acquire(
            (unsigned)uxQueueMessagesWaiting(s_tx_free_queue),
            true)) {
        xSemaphoreGive(s_tx_submit_mutex);
        return ESP_ERR_NO_MEM;
    }

    dual_end_tx_request_t *request = NULL;
    if (xQueueReceive(s_tx_free_queue, (void *)&request, 0) != pdTRUE || request == NULL) {
        if (!may_replace_audio ||
            xQueueReceive(s_tx_audio_queue, (void *)&request, 0) != pdTRUE || request == NULL) {
            xSemaphoreGive(s_tx_submit_mutex);
            return ESP_ERR_NO_MEM;
        }
        s_tx_audio_stale_drop_count++;
    }

    tx_prepare_async_request(request, payload, len, kind, created_us);
    request->reservation_id = reservation_id;
    request->deadline_us = deadline_us;
    QueueHandle_t queue = kind == DUAL_END_TX_KIND_AUDIO ?
                          s_tx_input_queue : s_tx_priority_queue;
    if (xQueueSend(queue, (const void *)&request, 0) != pdTRUE) {
        (void)xQueueSend(s_tx_free_queue, (const void *)&request, portMAX_DELAY);
        xSemaphoreGive(s_tx_submit_mutex);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(s_tx_submit_mutex);
    tx_update_queue_high_water();
    xSemaphoreGive(s_tx_wake_sem);
    return ESP_OK;
}

static esp_err_t tx_submit_audio_async(const void *payload, int len, int64_t created_us)
{
    if (payload == NULL || len <= 0 || len > DUAL_END_TX_MAX_DATA_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!radio_can_submit()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_tx_free_queue == NULL || s_tx_priority_queue == NULL ||
        s_tx_audio_queue == NULL || s_tx_wake_sem == NULL ||
        s_tx_submit_mutex == NULL || s_tx_task == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (audio_payload_has_one_shot_input(payload, len)) {
        /* 一次性 HID 标志不能被更新的音频帧覆盖。 */
        return tx_submit_priority_async(payload,
                                         len,
                                         DUAL_END_TX_KIND_AUDIO,
                                         true,
                                         created_us, 0, 0);
    }

    if (xSemaphoreTake(s_tx_submit_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    /* A slow in-flight callback does not expire every queued frame. Preserve
     * eligible frames; the sole worker checks each generation and deadline.
     * If all slots are occupied, only the oldest ordinary audio is evicted.
     * One-shot inputs use their separate protected queue above. */

    dual_end_tx_request_t *request = NULL;
    if (xQueueReceive(s_tx_free_queue, (void *)&request, 0) != pdTRUE || request == NULL) {
        if (xQueueReceive(s_tx_audio_queue, (void *)&request, 0) != pdTRUE || request == NULL) {
            xSemaphoreGive(s_tx_submit_mutex);
            return ESP_ERR_NO_MEM;
        }
        s_tx_audio_stale_drop_count++;
    }

    tx_prepare_async_request(request, payload, len, DUAL_END_TX_KIND_AUDIO, created_us);
    if (xQueueSend(s_tx_audio_queue, (const void *)&request, 0) != pdTRUE) {
        (void)xQueueSend(s_tx_free_queue, (const void *)&request, portMAX_DELAY);
        xSemaphoreGive(s_tx_submit_mutex);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(s_tx_submit_mutex);
    tx_update_queue_high_water();
    xSemaphoreGive(s_tx_wake_sem);
    return ESP_OK;
}

/* Reject work which was queued before the sleep transition. Synchronous
 * callers own returning their slot to the free queue after they are woken;
 * asynchronous requests are returned here. */
static void tx_reject_queued_requests(QueueHandle_t queue)
{
    if (queue == NULL) {
        return;
    }

    dual_end_tx_request_t *request = NULL;
    while (xQueueReceive(queue, (void *)&request, 0) == pdTRUE) {
        if (request != NULL) {
            request->result = ESP_ERR_INVALID_STATE;
            tx_retain_unsent_input(request);
            if (request->kind == DUAL_END_TX_KIND_AUDIO_REPAIR) {
                const dual_end_audio_repair_packet_t *repair =
                    (const dual_end_audio_repair_packet_t *)request->data;
                dual_end_audio_repair_tx_complete(repair->session_id,
                    repair->packet.sequence, request->reservation_id,
                    false, false, esp_timer_get_time());
            }
            if (request->synchronous) {
                xSemaphoreGive(request->done);
            } else {
                (void)xQueueSend(s_tx_free_queue, (const void *)&request, portMAX_DELAY);
            }
        }
        request = NULL;
    }
}

static bool channel_time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return due_ms != 0 && (int32_t)(now_ms - due_ms) >= 0;
}

/* 切换后旧信道上的排队音频已失去实时性，立即归还 slot，避免 stale_drop 级联。 */

static esp_err_t channel_queue_user_request(uint8_t target, bool avoid_current)
{
    if (!s_link_task || !s_connected || !radio_can_submit()) return ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&s_channel_request_lock);
    if (s_radio_suspended || s_channel_status.pending || s_channel_user_queued) {
        portEXIT_CRITICAL(&s_channel_request_lock);
        return ESP_ERR_INVALID_STATE;
    }
    s_channel_status.request_id++;
    if (target == s_current_channel) {
        s_channel_status.target_channel = target;
        s_channel_status.state = DUAL_END_CHANNEL_SUCCEEDED;
        s_channel_status.last_result = ESP_OK;
        s_channel_status.pending = false;
        portEXIT_CRITICAL(&s_channel_request_lock);
        return ESP_OK;
    }
    s_channel_status.target_channel = target;
    s_channel_status.state = DUAL_END_CHANNEL_QUEUED;
    s_channel_status.pending = true;
    s_channel_status.last_result = ESP_OK;
    s_channel_user_target = target;
    s_channel_user_avoid = avoid_current;
    s_channel_user_queued = true;
    portEXIT_CRITICAL(&s_channel_request_lock);
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_request_channel(uint8_t channel)
{
    if (!channel_is_valid(channel)) return ESP_ERR_INVALID_ARG;
    return channel_queue_user_request(channel, false);
}

esp_err_t dual_end_espnow_link_request_channel_rescan(bool avoid_current)
{
    return channel_queue_user_request(0, avoid_current);
}

esp_err_t dual_end_espnow_link_get_channel_status(dual_end_channel_status_t *status)
{
    if (!status || !s_link_task) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_channel_request_lock);
    *status = s_channel_status;
    portEXIT_CRITICAL(&s_channel_request_lock);
    status->current_channel = s_current_channel;
    status->scan_count = s_channel_rescan_count;
    status->switch_count = s_channel_switch_count;
    status->fallback_count = s_channel_fallback_count;
    status->rx_stack_min_bytes = 0 /* RX runs on the SDK Wi-Fi task; no owned RX stack. */;
    status->tx_stack_min_bytes = s_tx_task ? uxTaskGetStackHighWaterMark(s_tx_task) : 0;
    status->link_stack_min_bytes = uxTaskGetStackHighWaterMark(s_link_task);
    return ESP_OK;
}

/* Request storage is small and independent from all SDK locks. */
static bool channel_take_user_request(uint8_t *target, bool *avoid)
{
    portENTER_CRITICAL(&s_channel_request_lock);
    bool queued = s_channel_user_queued;
    if (queued) {
        *target = s_channel_user_target;
        *avoid = s_channel_user_avoid;
        s_channel_user_queued = false;
    }
    portEXIT_CRITICAL(&s_channel_request_lock);
    return queued;
}

static void channel_drop_queued_audio(void)
{
    if (s_tx_audio_queue == NULL || s_tx_free_queue == NULL) {
        return;
    }
    dual_end_tx_request_t *stale = NULL;
    while (xQueueReceive(s_tx_audio_queue, (void *)&stale, 0) == pdTRUE) {
        if (stale != NULL) {
            s_tx_audio_stale_drop_count++;
            (void)xQueueSend(s_tx_free_queue, (const void *)&stale, portMAX_DELAY);
        }
        stale = NULL;
    }
    while (s_tx_input_queue != NULL &&
           xQueueReceive(s_tx_input_queue, &stale, 0) == pdTRUE) {
        tx_retain_unsent_input(stale);
        s_tx_audio_stale_drop_count++;
        (void)xQueueSend(s_tx_free_queue, &stale, portMAX_DELAY);
    }
    /* Leave the control FIFO untouched. The sole worker rejects old recovery
     * generations before esp_now_send(), preserving OTA/channel ordering. */
}

void dual_end_espnow_link_discard_audio(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_tx_audio_generation++;
    portEXIT_CRITICAL(&s_tx_stats_lock);
    channel_reset_interference_window();
    channel_drop_queued_audio();
}

uint8_t dual_end_espnow_link_take_unsent_input_flags(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    uint8_t flags = s_tx_deferred_input_flags;
    s_tx_deferred_input_flags = 0;
    portEXIT_CRITICAL(&s_tx_stats_lock);
    return flags;
}

static void channel_reset_interference_window(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_channel_interference_window_start_ms = 0;
    s_channel_interference_packets = 0;
    s_channel_interference_failures = 0;
    s_channel_bad_windows = 0;
    s_channel_congestion_window_start_ms = 0;
    s_channel_congestion_packets = 0;
    s_channel_congestion_bad = 0;
    s_channel_congestion_bad_windows = 0;
    s_channel_rescan_requested = false;
    /* Preserve the global rescan cooldown across channels. */
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static void channel_pause_audio(uint32_t now_ms)
{
    s_channel_audio_paused = true;
    s_channel_pause_due_ms = now_ms + DUAL_END_CHANNEL_PAUSE_TIMEOUT_MS;
    channel_reset_interference_window();
    dual_end_espnow_link_discard_audio();
}

/* Wi-Fi 信道和 ESP-NOW peer 在同一互斥区内更新，避免发送线程看到中间状态。 */
static esp_err_t channel_switch_runtime(uint8_t channel, const char *reason)
{
    ESP_RETURN_ON_FALSE(channel_is_valid(channel), ESP_ERR_INVALID_ARG, TAG,
                        "invalid runtime channel");
    ESP_RETURN_ON_FALSE(s_peer_mutex, ESP_ERR_INVALID_STATE, TAG,
                        "link is not initialized");
    ESP_RETURN_ON_FALSE(radio_can_submit(), ESP_ERR_INVALID_STATE, TAG,
                        "radio is suspended");
    if (channel == s_current_channel && !s_channel_configuration_dirty) {
        return ESP_OK;
    }

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    if (!radio_can_submit()) {
        xSemaphoreGive(s_peer_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (ret == ESP_OK && s_peer_valid) {
        ret = configure_peer(s_peer_mac);
        if (ret != ESP_OK) {
            esp_err_t rollback = esp_wifi_set_channel(s_current_channel, WIFI_SECOND_CHAN_NONE);
            if (rollback == ESP_OK) rollback = configure_peer(s_peer_mac);
            if (rollback != ESP_OK) {
                uint8_t actual = s_current_channel;
                wifi_second_chan_t secondary;
                if (esp_wifi_get_channel(&actual, &secondary) == ESP_OK) s_current_channel = actual;
                s_connected = false;
                s_channel_configuration_dirty = true;
                s_channel_peer_seen_after_switch = false;
                portENTER_CRITICAL(&s_peer_state_lock);
                s_last_seen_us = 0;
                portEXIT_CRITICAL(&s_peer_state_lock);
                s_channel_fallback_due_ms = tx_now_ms() + DUAL_END_CHANNEL_CONFIRM_TIMEOUT_MS;
                ESP_LOGE(TAG, "channel rollback failed: %s", esp_err_to_name(rollback));
            }
        }
    }
    if (ret == ESP_OK) {
        uint8_t old_channel = s_current_channel;
        s_current_channel = channel;
        s_channel_switched_us = esp_timer_get_time();
        s_channel_configuration_dirty = false;
        channel_reset_interference_window();
        s_connected = false;
        portENTER_CRITICAL(&s_peer_state_lock);
        s_last_seen_us = 0;
        portEXIT_CRITICAL(&s_peer_state_lock);
        ESP_LOGW(TAG, "channel switch %u -> %u reason=%s",
                 old_channel, channel, reason ? reason : "unknown");
    }
    xSemaphoreGive(s_peer_mutex);
    return ret;
}

static bool is_peer_addr(const uint8_t *addr)
{
    uint8_t peer_mac[ESP_NOW_ETH_ALEN];
    bool valid;
    portENTER_CRITICAL(&s_peer_state_lock);
    memcpy(peer_mac, s_peer_mac, sizeof(peer_mac));
    valid = s_peer_valid;
    portEXIT_CRITICAL(&s_peer_state_lock);
    return valid && addr && memcmp(addr, peer_mac, ESP_NOW_ETH_ALEN) == 0;
}

static bool is_valid_framework_packet(const uint8_t *data, int len)
{
    if (!data || len < (int)sizeof(dual_end_packet_header_t)) {
        return false;
    }

    const dual_end_packet_header_t *header = (const dual_end_packet_header_t *)data;
    return header->protocol_version == SCEPTER_PROTOCOL_VERSION &&
           header->source_role != s_role &&
           header->payload_len <= (uint16_t)(len - (int)sizeof(*header));
}

static void mark_peer_seen(int rssi)
{
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_peer_state_lock);
    s_last_seen_us = now_us;
    s_last_rssi = rssi;
    portEXIT_CRITICAL(&s_peer_state_lock);
    portENTER_CRITICAL(&s_idle_lock);
    dual_end_wifi_idle_received(&s_idle_policy, now_us);
    portEXIT_CRITICAL(&s_idle_lock);
    s_connected = true;
    if (s_channel_fallback_due_ms != 0) {
        s_channel_peer_seen_after_switch = true;
    }
}

static bool tx_data_suppresses_hello(bool connected, uint32_t now_ms)
{
    return connected && s_last_data_submit_ms > 0 &&
        now_ms - s_last_data_submit_ms <= DUAL_END_HELLO_DATA_SUPPRESS_MS;
}

static esp_err_t send_hello(void)
{
    dual_end_packet_header_t hello = {
        .protocol_version = SCEPTER_PROTOCOL_VERSION,
        .source_role = s_role,
        .message_type = SCEPTER_MSG_HELLO,
        .flags = 0,
        .sequence = s_hello_sequence++,
        .payload_len = 0,
    };
    return tx_submit_priority_async(&hello,
                                     sizeof(hello),
                                     DUAL_END_TX_KIND_HELLO,
                                      false,
                                      esp_timer_get_time(), 0, 0);
}

static esp_err_t send_channel_control(uint8_t command,
                                      uint8_t channel,
                                      uint8_t flags,
                                      uint32_t generation,
                                      uint16_t switch_delay_ms,
                                      int16_t channel_score)
{
    dual_end_channel_control_packet_t packet = {
        .packet = {
            .protocol_version = SCEPTER_PROTOCOL_VERSION,
            .source_role = s_role,
            .message_type = SCEPTER_MSG_CHANNEL_CONTROL,
            .flags = 0,
            .sequence = generation,
            .payload_len = DUAL_END_CHANNEL_CONTROL_PAYLOAD_BYTES,
        },
        .control_version = DUAL_END_CHANNEL_CONTROL_VERSION,
        .command = command,
        .channel = channel,
        .flags = flags,
        .generation = generation,
        .switch_delay_ms = switch_delay_ms,
        .channel_score = channel_score,
    };
    return tx_submit_and_wait(&packet, sizeof(packet), DUAL_END_TX_KIND_HELLO);
}

static esp_err_t configure_peer(const uint8_t *peer_mac)
{
    ESP_RETURN_ON_FALSE(peer_mac, ESP_ERR_INVALID_ARG, TAG, "peer mac is null");

    bool encrypted = s_active_pairing_valid &&
                     memcmp(s_active_pairing.peer_mac,
                            peer_mac,
                            ESP_NOW_ETH_ALEN) == 0 &&
                     dual_end_pairing_active_security(&s_active_pairing) ==
                         DUAL_END_PAIRING_SECURITY_ENCRYPTED;
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    /* peer.channel=0 表示跟随 STA 当前信道，允许协同运行时跳频，
     * 无需删除并重建配对项。 */
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = encrypted;
    if (encrypted) {
        ESP_RETURN_ON_ERROR(esp_now_set_pmk(s_active_pairing.pmk),
                            TAG,
                            "set esp-now PMK");
        memcpy(peer.lmk, s_active_pairing.lmk, sizeof(peer.lmk));
    }

    esp_err_t ret = esp_now_is_peer_exist(peer_mac) ?
                    esp_now_mod_peer(&peer) :
                    esp_now_add_peer(&peer);
    ESP_RETURN_ON_ERROR(ret, TAG, "add/update unicast peer");

    esp_now_rate_config_t rate_config = {
        .phymode = WIFI_PHY_MODE_11B,
        .rate = WIFI_PHY_RATE_5M_S, /* ESP-IDF 5M means 5.5 Mbps, short preamble. */
        .ersu = false,
        .dcm = false,
    };
    ESP_RETURN_ON_ERROR(esp_now_set_peer_rate_config(peer_mac, &rate_config),
                        TAG,
                        "set esp-now peer rate");
    uint8_t primary = 0; wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
    esp_err_t channel_ret = esp_wifi_get_channel(&primary, &secondary);
    ESP_LOGI(TAG, "ESP-NOW peer security=%.12s generation=%" PRIu32
             " exist=%d wifi_ch=%u/%u (%.64s)", encrypted ? "ccmp" : "legacy_plain", encrypted ? s_active_pairing.generation : 0, esp_now_is_peer_exist(peer_mac) ? 1 : 0, primary, (unsigned)secondary, esp_err_to_name(channel_ret));
    return ESP_OK;
}

/* Scepter 是协商执行端：收到提议后确认，收到提交后按约定延时切换。 */
static void channel_process_event(const dual_end_channel_event_t *event, uint32_t now_ms)
{
    if (!event) return;
    portENTER_CRITICAL(&s_channel_request_lock);
    if (s_radio_suspended) {
        portEXIT_CRITICAL(&s_channel_request_lock);
        return;
    }
    s_channel_event_processing = true;
    portEXIT_CRITICAL(&s_channel_request_lock);
    switch (event->command) {
    case DUAL_END_CHANNEL_CMD_SCAN_BEGIN:
        if (!dual_end_channel_begin(&s_channel_guard, event->generation)) break;
        s_channel_pending_generation = event->generation;
        channel_status_update(DUAL_END_CHANNEL_SCANNING, s_current_channel,
                              event->generation, ESP_OK, true);
        channel_pause_audio(now_ms);
        ESP_LOGI(TAG, "channel scan begin generation=%" PRIu32 " ch=%u",
                 event->generation, s_current_channel);
        break;
    case DUAL_END_CHANNEL_CMD_SCAN_END: {
        bool scan_finished = s_channel_guard.phase == 1 &&
                             !(event->flags & DUAL_END_CHANNEL_FLAG_FAILED);
        if (!dual_end_channel_cancel(&s_channel_guard, event->generation)) break;
        channel_status_update(scan_finished ? DUAL_END_CHANNEL_SUCCEEDED : DUAL_END_CHANNEL_FAILED,
                              s_current_channel, event->generation,
                              scan_finished ? ESP_OK : ESP_FAIL, false);
        s_channel_audio_paused = false;
        s_channel_pause_due_ms = 0;
        s_channel_switch_due_ms = 0;
        ESP_LOGI(TAG, "channel scan end generation=%" PRIu32 " stay=%u",
                 event->generation, s_current_channel);
        break;
    }
    case DUAL_END_CHANNEL_CMD_SWITCH_PROPOSE:
        if (!dual_end_channel_offer(&s_channel_guard, event->generation,
                                    event->channel, s_current_channel)) break;
        channel_status_update(DUAL_END_CHANNEL_PROPOSING, event->channel,
                              event->generation, ESP_OK, true);
        channel_pause_audio(now_ms);
        s_channel_pending_channel = event->channel;
        s_channel_pending_generation = event->generation;
        ESP_LOGI(TAG,
                 "channel proposal generation=%" PRIu32 " %u -> %u score=%d flags=0x%02x",
                 event->generation, s_current_channel, event->channel,
                 event->channel_score, event->flags);
        if (send_channel_control(DUAL_END_CHANNEL_CMD_SWITCH_ACCEPT,
                                 event->channel,
                                 event->flags,
                                 event->generation,
                                 event->switch_delay_ms,
                                 event->channel_score) != ESP_OK) {
            ESP_LOGW(TAG, "channel accept failed generation=%" PRIu32,
                     event->generation);
        }
        break;
    case DUAL_END_CHANNEL_CMD_SWITCH_COMMIT: {
        if (!dual_end_channel_commit(&s_channel_guard, event->generation,
                                     event->channel)) break;
        channel_status_update(DUAL_END_CHANNEL_SWITCHING, event->channel,
                              event->generation, ESP_OK, true);
        channel_pause_audio(now_ms);
        uint32_t delay_ms = event->switch_delay_ms;
        if (delay_ms < DUAL_END_CHANNEL_SWITCH_MIN_DELAY_MS) {
            delay_ms = DUAL_END_CHANNEL_SWITCH_MIN_DELAY_MS;
        } else if (delay_ms > DUAL_END_CHANNEL_SWITCH_MAX_DELAY_MS) {
            delay_ms = DUAL_END_CHANNEL_SWITCH_MAX_DELAY_MS;
        }
        /* Use reception time, not the time after an earlier synchronous
         * control callback returns. Exact cross-device clock sync is not assumed. */
        s_channel_switch_due_ms = event->received_ms + delay_ms;
        if (!s_channel_switch_due_ms) s_channel_switch_due_ms = 1;
        ESP_LOGI(TAG,
                 "channel commit generation=%" PRIu32 " target=%u delay=%" PRIu32 "ms",
                 event->generation, event->channel, delay_ms);
        break;
    }
    default:
        break;
    }
    portENTER_CRITICAL(&s_channel_request_lock);
    s_channel_event_processing = false;
    portEXIT_CRITICAL(&s_channel_request_lock);
}

static void notify_status_if_changed(bool connected, int rssi)
{
    bool connection_changed = connected != s_reported_connected;
    bool rssi_changed = rssi != s_reported_rssi;
    if (!connection_changed && !rssi_changed) {
        return;
    }

    s_reported_connected = connected;
    s_reported_rssi = rssi;
    if (connection_changed) {
        ESP_LOGI(TAG, "peer %s rssi=%d", connected ? "connected" : "disconnected", rssi);
    }
    if (s_status_cb) {
        s_status_cb(connected, rssi, s_status_ctx);
    }
}

/* Link-task observations, including initial connection wait. Durations use
 * monotonic 64-bit microseconds and survive the 32-bit millisecond wrap. */
static void stability_log_link(bool connected, int rssi, bool suspended)
{
    static bool initialized, was_connected;
    static uint64_t disconnects;
    static int64_t last_us, last_log_us, outage_start_us, disconnected_us, max_outage_us;
    int64_t now_us = esp_timer_get_time();
    if (!initialized) {
        initialized = true;
        last_us = now_us;
        outage_start_us = now_us;
    }
    if (!was_connected) {
        disconnected_us += now_us - last_us;
        int64_t outage_us = now_us - outage_start_us;
        if (outage_us > max_outage_us) {
            max_outage_us = outage_us;
        }
    }
    if (was_connected && !connected) {
        disconnects++;
        outage_start_us = now_us;
    }
    was_connected = connected;
    last_us = now_us;
    if (last_log_us != 0 && now_us - last_log_us < 30000000LL) {
        return;
    }
    last_log_us = now_us;
    {
        /* Snapshot before pacing these background/startup diagnostics. */
        const uint64_t log_uptime_ms = (uint64_t)(now_us / 1000LL);
        const uint32_t log_boot_id = dual_end_runtime_identity_boot_id();
        const char log_boot_ready = log_boot_id ? '1' : '0';
        const int32_t log_reset_reason_0 = (int)esp_reset_reason();
        const int32_t log_connected_0 = connected ? 1 : 0;
        const int32_t log_rssi_0 = rssi;
        const uint32_t log_channel_0 = s_current_channel;
        const int32_t log_suspended_0 = suspended ? 1 : 0;
        const uint64_t log_disconnects_0 = disconnects;
        const uint64_t log_disconnected_ms_0 = (uint64_t)(disconnected_us / 1000LL);
        const uint64_t log_max_disconnect_ms_0 = (uint64_t)(max_outage_us / 1000LL);
        const uint32_t log_hop_0 = s_channel_switch_count;
        const uint32_t log_fallback_0 = s_channel_fallback_count;
        const uint32_t log_heap_free_0 = esp_get_free_heap_size();
        const uint32_t log_heap_min_0 = esp_get_minimum_free_heap_size();
        const uint32_t log_dma_free_0 = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const uint32_t log_dma_largest_0 = (uint32_t)heap_caps_get_largest_free_block(
                  MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const uint32_t log_dma_min_0 = (uint32_t)heap_caps_get_minimum_free_size(
                  MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        ESP_LOGI(TAG,
            "STABILITY_LINK schema=1 role=scepter part=0 parts=5 uptime_ms=%" PRIu64 " boot_id=%08" PRIx32 " reset_reason=%d rate_mbps=5.5 connected=%d rssi=%d boot_ready=%c",
            log_uptime_ms, log_boot_id, log_reset_reason_0, log_connected_0, log_rssi_0, log_boot_ready);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "STABILITY_LINK schema=1 role=scepter part=1 parts=5 uptime_ms=%" PRIu64 " boot_id=%08" PRIx32 " channel=%u suspended=%d disconnects=%" PRIu64,
            log_uptime_ms, log_boot_id, log_channel_0, log_suspended_0, log_disconnects_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "STABILITY_LINK schema=1 role=scepter part=2 parts=5 uptime_ms=%" PRIu64 " boot_id=%08" PRIx32 " disconnected_ms=%" PRIu64 " max_disconnect_ms=%" PRIu64,
            log_uptime_ms, log_boot_id, log_disconnected_ms_0, log_max_disconnect_ms_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "STABILITY_LINK schema=1 role=scepter part=3 parts=5 uptime_ms=%" PRIu64 " boot_id=%08" PRIx32 " hop=%" PRIu32 " fallback=%" PRIu32 " heap_free=%" PRIu32 " heap_min=%" PRIu32,
            log_uptime_ms, log_boot_id, log_hop_0, log_fallback_0, log_heap_free_0, log_heap_min_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "STABILITY_LINK schema=1 role=scepter part=4 parts=5 uptime_ms=%" PRIu64 " boot_id=%08" PRIx32 " dma_free=%" PRIu32 " dma_largest=%" PRIu32 " dma_min=%" PRIu32,
            log_uptime_ms, log_boot_id, log_dma_free_0, log_dma_largest_0, log_dma_min_0);
    }
}

static void stability_log_tx(void)
{
    dual_end_tx_attempt_stats_t first, retry, repair, control;
    portENTER_CRITICAL(&s_tx_stats_lock);
    first = s_stability_first;
    retry = s_stability_retry;
    repair = s_stability_repair;
    control = s_audio_control_attempts;
    const esp_err_t last_error = s_audio_last_immediate_error;
    const uint32_t no_mem_errors = s_audio_no_mem_errors;
    const uint32_t control_submit_failures = s_audio_control_submit_failures;
    const uint32_t control_no_mem_retries = s_audio_control_no_mem_retries;
    const esp_err_t control_last_error = s_audio_control_last_submit_error;
    portEXIT_CRITICAL(&s_tx_stats_lock);
    /* Lifetime counters: a historical error is not proof of a current failure.
     * Two u64 counters per entry leave room for the ESP log prefix even at
     * saturation. Every part shares one sample time; legacy field names stay. */
    const uint64_t stability_uptime_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
    ESP_LOGI(TAG, "AUDIO_TX_ERROR schema=1 last_immediate=%s no_mem_total=%" PRIu32,
             esp_err_to_name(last_error), no_mem_errors);
    ESP_LOGI(TAG,
             "AUDIO_CONTROL_TX schema=1 part=0 parts=5 uptime_ms=%" PRIu64
             " calls=%" PRIu64 " submitted=%" PRIu64,
             stability_uptime_ms, control.calls, control.submitted);
    ESP_LOGI(TAG,
             "AUDIO_CONTROL_TX schema=1 part=1 parts=5 uptime_ms=%" PRIu64
             " mac_ok=%" PRIu64 " mac_fail=%" PRIu64,
             stability_uptime_ms, control.mac_ok, control.mac_fail);
    ESP_LOGI(TAG,
             "AUDIO_CONTROL_TX schema=1 part=2 parts=5 uptime_ms=%" PRIu64
             " immediate_fail=%" PRIu64 " unresolved=%" PRIu64,
             stability_uptime_ms, control.immediate_fail, control.unresolved);
    ESP_LOGI(TAG, "AUDIO_CONTROL_TX schema=1 part=3 parts=5 uptime_ms=%" PRIu64
             " no_mem_retries=%" PRIu32 " submit_failures=%" PRIu32,
             stability_uptime_ms, control_no_mem_retries, control_submit_failures);
    ESP_LOGI(TAG, "AUDIO_CONTROL_TX schema=1 part=4 parts=5 uptime_ms=%" PRIu64
             " last_error=%s", stability_uptime_ms, esp_err_to_name(control_last_error));
    ESP_LOGI(TAG,
             "STABILITY_TX_A schema=1 retry_policy=1 part=0 parts=6 uptime_ms=%" PRIu64
             " original_first_calls=%" PRIu64 " original_first_submitted=%" PRIu64,
             stability_uptime_ms, first.calls, first.submitted);
    ESP_LOGI(TAG,
             "STABILITY_TX_A schema=1 retry_policy=1 part=1 parts=6 uptime_ms=%" PRIu64
             " original_first_mac_ok=%" PRIu64 " original_first_mac_fail=%" PRIu64,
             stability_uptime_ms, first.mac_ok, first.mac_fail);
    ESP_LOGI(TAG,
             "STABILITY_TX_A schema=1 retry_policy=1 part=2 parts=6 uptime_ms=%" PRIu64
             " original_first_immediate_fail=%" PRIu64 " original_first_unresolved=%" PRIu64,
             stability_uptime_ms, first.immediate_fail, first.unresolved);
    ESP_LOGI(TAG,
             "STABILITY_TX_A schema=1 retry_policy=1 part=3 parts=6 uptime_ms=%" PRIu64
             " fast_retry_calls=%" PRIu64 " fast_retry_submitted=%" PRIu64,
             stability_uptime_ms, retry.calls, retry.submitted);
    ESP_LOGI(TAG,
             "STABILITY_TX_A schema=1 retry_policy=1 part=4 parts=6 uptime_ms=%" PRIu64
             " fast_retry_mac_ok=%" PRIu64 " fast_retry_mac_fail=%" PRIu64,
             stability_uptime_ms, retry.mac_ok, retry.mac_fail);
    ESP_LOGI(TAG,
             "STABILITY_TX_A schema=1 retry_policy=1 part=5 parts=6 uptime_ms=%" PRIu64
             " fast_retry_immediate_fail=%" PRIu64 " fast_retry_unresolved=%" PRIu64,
             stability_uptime_ms, retry.immediate_fail, retry.unresolved);
    ESP_LOGI(TAG,
             "STABILITY_TX_B schema=1 retry_policy=1 part=0 parts=3 uptime_ms=%" PRIu64
             " repair_calls=%" PRIu64 " repair_submitted=%" PRIu64,
             stability_uptime_ms, repair.calls, repair.submitted);
    ESP_LOGI(TAG,
             "STABILITY_TX_B schema=1 retry_policy=1 part=1 parts=3 uptime_ms=%" PRIu64
             " repair_mac_ok=%" PRIu64 " repair_mac_fail=%" PRIu64,
             stability_uptime_ms, repair.mac_ok, repair.mac_fail);
    ESP_LOGI(TAG,
             "STABILITY_TX_B schema=1 retry_policy=1 part=2 parts=3 uptime_ms=%" PRIu64
             " repair_immediate_fail=%" PRIu64 " repair_unresolved=%" PRIu64,
             stability_uptime_ms, repair.immediate_fail, repair.unresolved);
}

void dual_end_espnow_link_get_radio_recovery_stats(dual_end_radio_recovery_stats_t *stats)
{
    if (stats == NULL) { return; }
    portENTER_CRITICAL(&s_tx_stats_lock);
    *stats = (dual_end_radio_recovery_stats_t) {
        .state = s_radio_state, .reason = s_radio_reason, .generation = s_radio_generation,
        .early = s_radio_early_count, .hard = s_radio_hard_count, .forced = s_radio_forced_count,
        .success = s_radio_success_count, .fail = s_radio_fail_count,
        .reset_last_us = s_radio_api_last_us, .reset_max_us = s_radio_api_max_us,
        .first_cb_last_us = s_radio_first_ok_last_us, .first_cb_max_us = s_radio_first_ok_max_us,
        .first_cb_pending = s_radio_awaiting_first_ok,
        .first_rx_last_us = s_radio_first_rx_last_us, .first_rx_max_us = s_radio_first_rx_max_us,
        .first_rx_pending = s_radio_awaiting_first_rx, .error = s_radio_last_error,
        .congestion_late = s_channel_congestion_late_count,
        .congestion_expired = s_channel_congestion_expired_count,
    };
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static void radio_log_recovery(void)
{
    static int64_t last_log_us;
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_tx_stats_lock);
    bool emit = s_radio_recovery_dirty || now_us - last_log_us >= 30000000LL;
    if (emit) { s_radio_recovery_dirty = false; }
    portEXIT_CRITICAL(&s_tx_stats_lock);
    if (!emit) { return; }
    last_log_us = now_us;
    dual_end_radio_recovery_stats_t stats;
    dual_end_espnow_link_get_radio_recovery_stats(&stats);
    {
        /* Snapshot before pacing these background/startup diagnostics. */
        const uint64_t log_uptime_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
        const uint32_t log_state_0 = stats.state;
        const uint32_t log_reason_0 = stats.reason;
        const uint32_t log_generation_0 = stats.generation;
        const uint32_t log_early_0 = stats.early;
        const uint32_t log_hard_0 = stats.hard;
        const uint32_t log_forced_0 = stats.forced;
        const uint32_t log_success_0 = stats.success;
        const uint32_t log_fail_0 = stats.fail;
        const uint32_t log_reset_last_us_0 = stats.reset_last_us;
        const uint32_t log_reset_max_us_0 = stats.reset_max_us;
        const uint32_t log_first_cb_last_us_0 = stats.first_cb_last_us;
        const uint32_t log_first_cb_max_us_0 = stats.first_cb_max_us;
        const uint32_t log_first_cb_pending_0 = stats.first_cb_pending;
        const uint32_t log_first_rx_last_us_0 = stats.first_rx_last_us;
        const uint32_t log_first_rx_max_us_0 = stats.first_rx_max_us;
        const uint32_t log_first_rx_pending_0 = stats.first_rx_pending;
        const int32_t log_error_0 = stats.error;
        const uint32_t log_congestion_late_0 = stats.congestion_late;
        const uint32_t log_congestion_expired_0 = stats.congestion_expired;
        ESP_LOGI(TAG,
            "RADIO_RECOVERY schema=1 role=scepter part=0 parts=5 uptime_ms=%" PRIu64 " state=%" PRIu32 " reason=%" PRIu32 " generation=%" PRIu32 " early=%" PRIu32 " hard=%" PRIu32,
            log_uptime_ms, log_state_0, log_reason_0, log_generation_0, log_early_0, log_hard_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "RADIO_RECOVERY schema=1 role=scepter part=1 parts=5 uptime_ms=%" PRIu64 " forced=%" PRIu32 " success=%" PRIu32 " fail=%" PRIu32 " reset_last_us=%" PRIu32 " reset_max_us=%" PRIu32,
            log_uptime_ms, log_forced_0, log_success_0, log_fail_0, log_reset_last_us_0, log_reset_max_us_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "RADIO_RECOVERY schema=1 role=scepter part=2 parts=5 uptime_ms=%" PRIu64 " first_cb_last_us=%" PRIu32 " first_cb_max_us=%" PRIu32 " first_cb_pending=%" PRIu32,
            log_uptime_ms, log_first_cb_last_us_0, log_first_cb_max_us_0, log_first_cb_pending_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "RADIO_RECOVERY schema=1 role=scepter part=3 parts=5 uptime_ms=%" PRIu64 " first_rx_last_us=%" PRIu32 " first_rx_max_us=%" PRIu32 " first_rx_pending=%" PRIu32 " error=%" PRId32,
            log_uptime_ms, log_first_rx_last_us_0, log_first_rx_max_us_0, log_first_rx_pending_0, log_error_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "RADIO_RECOVERY schema=1 role=scepter part=4 parts=5 uptime_ms=%" PRIu64 " congestion_late=%" PRIu32 " congestion_expired=%" PRIu32,
            log_uptime_ms, log_congestion_late_0, log_congestion_expired_0);
    }
}

static void link_status_task(void *arg)
{
    (void)arg;
    TickType_t last_hello_tick = 0;
    TickType_t last_diag_tick = 0;

    while (true) {
        radio_log_recovery();
        if (!radio_can_submit()) {
            stability_log_link(false, s_last_rssi, true);
            vTaskDelay(pdMS_TO_TICKS(DUAL_END_CHANNEL_TASK_POLL_MS));
            continue;
        }
        idle_radio_service();
        if (s_idle_wifi_stopped) {
            /* Intentional idle RF-off preserves logical link status. */
            stability_log_link(s_connected, s_last_rssi, false);
            vTaskDelay(pdMS_TO_TICKS(DUAL_END_CHANNEL_TASK_POLL_MS));
            continue;
        }
        uint32_t now_ms = tx_now_ms();
        dual_end_channel_event_t channel_event;
        while (s_channel_event_queue != NULL &&
               xQueueReceive(s_channel_event_queue, &channel_event, 0) == pdTRUE) {
            channel_process_event(&channel_event, tx_now_ms());
        }

        /* 延时切换给双方留出发送重复 COMMIT 和清空旧信道队列的时间。 */
        now_ms = tx_now_ms();
        if (channel_time_reached(now_ms, s_channel_switch_due_ms)) {
            uint8_t target_channel = s_channel_pending_channel;
            uint32_t generation = s_channel_pending_generation;
            s_channel_switch_due_ms = 0;
            esp_err_t switch_ret = channel_switch_runtime(target_channel, "coordinated");
            if (switch_ret == ESP_OK) {
                s_channel_switch_count++;
                s_channel_peer_seen_after_switch = false;
                s_channel_fallback_due_ms = tx_now_ms() + DUAL_END_CHANNEL_CONFIRM_TIMEOUT_MS;
                channel_status_update(DUAL_END_CHANNEL_CONFIRMING, target_channel, generation, ESP_OK, true);
                s_channel_audio_paused = false;
                s_channel_pause_due_ms = 0;
                esp_err_t confirm_ret = send_channel_control(
                    DUAL_END_CHANNEL_CMD_SWITCH_CONFIRM,
                    target_channel,
                    0,
                    generation,
                    0,
                    0);
                if (confirm_ret != ESP_OK) {
                    ESP_LOGW(TAG, "channel confirm send failed: %s",
                             esp_err_to_name(confirm_ret));
                }
            } else {
                s_channel_audio_paused = false;
                s_channel_pause_due_ms = 0;
                dual_end_channel_close(&s_channel_guard);
                channel_status_update(DUAL_END_CHANNEL_FAILED, target_channel, generation, switch_ret, false);
                ESP_LOGE(TAG, "channel switch failed target=%u: %s",
                         target_channel, esp_err_to_name(switch_ret));
            }
        }

        /* 新信道收到任一合法对端包即可确认链路，随后取消信道 6 回退计时。 */
        if (!s_channel_configuration_dirty && s_channel_peer_seen_after_switch && s_channel_fallback_due_ms != 0) {
            esp_err_t save_ret = channel_store_save(s_current_channel);
            if (save_ret != ESP_OK) {
                ESP_LOGW(TAG, "save confirmed channel failed: %s",
                         esp_err_to_name(save_ret));
            }
            ESP_LOGI(TAG, "channel confirmed generation=%" PRIu32 " ch=%u",
                     s_channel_pending_generation, s_current_channel);
            s_channel_fallback_due_ms = 0;
            s_channel_peer_seen_after_switch = false;
            dual_end_channel_close(&s_channel_guard);
            channel_status_update(DUAL_END_CHANNEL_SUCCEEDED, s_current_channel,
                                  s_channel_pending_generation, ESP_OK, false);
        }

        /* 协商包丢失时的保险：不能让音频因暂停状态永久静音。 */
        if (channel_time_reached(now_ms, s_channel_pause_due_ms)) {
            ESP_LOGW(TAG, "channel negotiation pause timeout; resume audio ch=%u",
                     s_current_channel);
            s_channel_audio_paused = false;
            s_channel_pause_due_ms = 0;
            s_channel_switch_due_ms = 0;
            dual_end_channel_close(&s_channel_guard);
            channel_status_update(DUAL_END_CHANNEL_FAILED, s_channel_pending_channel,
                                  s_channel_pending_generation, ESP_ERR_TIMEOUT, false);
        }

        bool connected = false;
        int rssi;
        int64_t last_seen_us;
        portENTER_CRITICAL(&s_peer_state_lock);
        rssi = s_last_rssi;
        last_seen_us = s_last_seen_us;
        portEXIT_CRITICAL(&s_peer_state_lock);
        int64_t now_us = esp_timer_get_time();
        if (last_seen_us > 0 &&
            (now_us - last_seen_us) <= ((int64_t)DUAL_END_LINK_TIMEOUT_MS * 1000)) {
            connected = true;
        }
        s_connected = connected;
        stability_log_link(connected, rssi, false);
        notify_status_if_changed(connected, rssi);

        if (connected) {
            s_channel_disconnect_since_ms = 0;
        } else if (s_current_channel != DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL &&
                   s_channel_disconnect_since_ms == 0) {
            s_channel_disconnect_since_ms = now_ms;
        }
        /* 非会合信道连续失联或切换未确认时，双方独立回到信道 6。 */
        bool disconnected_fallback =
            s_current_channel != DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL &&
            s_channel_disconnect_since_ms != 0 &&
            now_ms - s_channel_disconnect_since_ms >= DUAL_END_CHANNEL_CONFIRM_TIMEOUT_MS;
        if (channel_time_reached(now_ms, s_channel_fallback_due_ms) ||
            disconnected_fallback) {
            uint8_t failed_channel = s_current_channel;
            s_channel_fallback_due_ms = 0;
            s_channel_switch_due_ms = 0;
            s_channel_disconnect_since_ms = 0;
            s_channel_audio_paused = false;
            s_channel_pause_due_ms = 0;
            dual_end_channel_close(&s_channel_guard);
            channel_status_update(DUAL_END_CHANNEL_FAILED, s_channel_pending_channel,
                                  s_channel_pending_generation, ESP_ERR_TIMEOUT, false);
            if (channel_switch_runtime(DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL,
                                       "confirmation-timeout") == ESP_OK) {
                s_channel_fallback_count++;
                (void)channel_store_save(DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL);
                ESP_LOGW(TAG, "channel %u failed; rendezvous fallback ch=%u",
                         failed_channel, DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL);
            }
        }

        /* Scepter 只报告干扰，扫描和最终信道选择始终由 Dongle 统一完成。 */
        uint8_t user_target;
        bool user_avoid;
        if (channel_take_user_request(&user_target, &user_avoid)) {
            uint8_t flags = user_target ? DUAL_END_CHANNEL_FLAG_MANUAL_TARGET :
                            user_avoid ? DUAL_END_CHANNEL_FLAG_INTERFERENCE : 0;
            esp_err_t user_ret = send_channel_control(DUAL_END_CHANNEL_CMD_RESCAN_REQUEST,
                                    user_target, flags, 0, 0, 0);
            s_channel_user_deadline_ms = tx_now_ms() + 15000U;
            if (user_ret != ESP_OK) {
                channel_status_update(DUAL_END_CHANNEL_FAILED, user_target, 0, user_ret, false);
                s_channel_user_deadline_ms = 0;
            }
        }
        if (s_channel_user_deadline_ms && channel_time_reached(tx_now_ms(), s_channel_user_deadline_ms)) {
            s_channel_user_deadline_ms = 0;
            if (s_channel_status.pending && s_channel_status.state == DUAL_END_CHANNEL_QUEUED)
                channel_status_update(DUAL_END_CHANNEL_FAILED, s_channel_status.target_channel,
                                      s_channel_pending_generation, ESP_ERR_TIMEOUT, false);
        }
        bool request_rescan;
        portENTER_CRITICAL(&s_tx_stats_lock);
        request_rescan = s_channel_rescan_requested;
        portEXIT_CRITICAL(&s_tx_stats_lock);
        if (request_rescan && connected && !s_channel_audio_paused) {
            esp_err_t request_ret = send_channel_control(
                DUAL_END_CHANNEL_CMD_RESCAN_REQUEST,
                s_current_channel,
                DUAL_END_CHANNEL_FLAG_INTERFERENCE,
                s_channel_pending_generation,
                0,
                0);
            if (request_ret == ESP_OK) {
                portENTER_CRITICAL(&s_tx_stats_lock);
                s_channel_rescan_requested = false;
                portEXIT_CRITICAL(&s_tx_stats_lock);
                s_channel_rescan_count++;
                ESP_LOGW(TAG,
                         "interference rescan requested ch=%u packets=%" PRIu32
                         " failures=%" PRIu32,
                         s_current_channel,
                         s_channel_interference_packets,
                         s_channel_interference_failures);
            }
        }

        TickType_t now_tick = xTaskGetTickCount();
        uint32_t hello_interval_ms = connected ?
                                     DUAL_END_HELLO_CONNECTED_INTERVAL_MS :
                                     DUAL_END_HELLO_DISCONNECTED_INTERVAL_MS;
        if (last_hello_tick == 0 ||
            (now_tick - last_hello_tick) >= pdMS_TO_TICKS(hello_interval_ms)) {
            last_hello_tick = now_tick;
            esp_err_t ret = tx_data_suppresses_hello(connected, tx_now_ms()) ?
                            ESP_OK : send_hello();
            if (ret == ESP_ERR_NO_MEM || ret == ESP_ERR_ESPNOW_NO_MEM) {
                s_hello_no_mem_count++;
                if ((s_hello_no_mem_count % 20) == 1) {
                    ESP_LOGW(TAG,
                             "hello send skipped: ESP-NOW queue full count=%" PRIu32,
                             s_hello_no_mem_count);
                }
            } else if (ret == ESP_ERR_NOT_FOUND) {
                s_hello_not_found_count++;
                if ((s_hello_not_found_count % 5) == 1) {
                    ESP_LOGW(TAG, "hello send: no peer configured yet count=%" PRIu32,
                             s_hello_not_found_count);
                }
            } else if (ret != ESP_OK) {
                ESP_LOGW(TAG, "hello send failed: %s", esp_err_to_name(ret));
            }
        }

        if (last_diag_tick == 0 ||
            (now_tick - last_diag_tick) >= pdMS_TO_TICKS(DUAL_END_TX_DIAG_INTERVAL_MS)) {
            last_diag_tick = now_tick;
            dual_end_channel_status_t channel_diag;
            if (dual_end_espnow_link_get_channel_status(&channel_diag) == ESP_OK) {
                ESP_LOGI(TAG, "LINK_TASK_STACK schema=1 rx_min_bytes=%" PRIu32
                         " tx_min_bytes=%" PRIu32 " link_min_bytes=%" PRIu32,
                         channel_diag.rx_stack_min_bytes, channel_diag.tx_stack_min_bytes,
                         channel_diag.link_stack_min_bytes);
            }

            stability_log_tx();
            uint32_t accepted;
            uint32_t callbacks;
            uint32_t soft_timeout;
            uint32_t late;
            uint32_t recover;
            uint32_t recovery_fail;
            uint32_t orphan;
            uint32_t immediate_fail;
            uint32_t latency_last;
            uint32_t latency_max;
            uint32_t queue_high_water;
            uint32_t histogram[DUAL_END_TX_LATENCY_BUCKET_COUNT];
            bool callback_debt;
            portENTER_CRITICAL(&s_tx_stats_lock);
            accepted = s_tx_accepted_count;
            callbacks = s_tx_done_count;
            soft_timeout = s_tx_callback_soft_timeout_count;
            late = s_tx_callback_late_count;
            recover = s_tx_callback_hard_recovery_count;
            recovery_fail = s_tx_callback_recovery_fail_count;
            orphan = s_tx_callback_orphan_count;
            immediate_fail = s_tx_immediate_fail_count;
            latency_last = s_tx_callback_latency_last_us;
            latency_max = s_tx_callback_latency_max_us;
            queue_high_water = s_tx_queue_high_water;
            callback_debt = s_tx_callback_armed;
            memcpy(histogram, s_tx_callback_latency_buckets, sizeof(histogram));
            portEXIT_CRITICAL(&s_tx_stats_lock);
            {
                /* Snapshot before pacing these background/startup diagnostics. */
                const uint64_t log_uptime_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
                const uint32_t log_accepted_0 = accepted;
                const uint32_t log_cb_0 = callbacks;
                const int32_t log_debt_0 = callback_debt ? 1 : 0;
                const uint32_t log_soft_0 = soft_timeout;
                const uint32_t log_late_0 = late;
                const uint32_t log_recover_0 = recover;
                const uint32_t log_recover_1 = recovery_fail;
                const uint32_t log_orphan_0 = orphan;
                const uint32_t log_immediate_0 = immediate_fail;
                const uint32_t log_latency_us_0 = latency_last;
                const uint32_t log_latency_us_1 = latency_max;
                const uint32_t log_hist_0 = histogram[0];
                const uint32_t log_hist_1 = histogram[1];
                const uint32_t log_hist_2 = histogram[2];
                const uint32_t log_hist_3 = histogram[3];
                const uint32_t log_hist_4 = histogram[4];
                const uint32_t log_hist_5 = histogram[5];
                const uint32_t log_hist_6 = histogram[6];
                const uint32_t log_queue_0 = tx_queue_depth();
                const uint32_t log_queue_1 = queue_high_water;
                const int32_t log_slots_0 = DUAL_END_TX_SLOT_COUNT;
                const int32_t log_audio_pending_0 = DUAL_END_TX_AUDIO_PENDING_COUNT;
                const int32_t log_collapse_ms_0 = DUAL_END_TX_CALLBACK_SOFT_TIMEOUT_MS;
                const uint32_t log_ch_0 = s_current_channel;
                const uint32_t log_hop_0 = s_channel_switch_count;
                const uint32_t log_fallback_0 = s_channel_fallback_count;
                const uint32_t log_rescan_0 = s_channel_rescan_count;
                const int32_t log_paused_0 = s_channel_audio_paused ? 1 : 0;
                ESP_LOGI(TAG,
                    "tx_diag schema=1 part=0 parts=5 uptime_ms=%" PRIu64 " accepted=%" PRIu32 " cb=%" PRIu32 " debt=%d soft=%" PRIu32 " late=%" PRIu32 " recover=%" PRIu32 "/%" PRIu32,
                    log_uptime_ms, log_accepted_0, log_cb_0, log_debt_0, log_soft_0, log_late_0, log_recover_0, log_recover_1);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "tx_diag schema=1 part=1 parts=5 uptime_ms=%" PRIu64 " orphan=%" PRIu32 " immediate=%" PRIu32 " latency_us=%" PRIu32 "/%" PRIu32,
                    log_uptime_ms, log_orphan_0, log_immediate_0, log_latency_us_0, log_latency_us_1);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "tx_diag schema=1 part=2 parts=5 uptime_ms=%" PRIu64 " hist=[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "] queue=%" PRIu32 "/%" PRIu32,
                    log_uptime_ms, log_hist_0, log_hist_1, log_hist_2, log_hist_3, log_hist_4, log_hist_5, log_hist_6, log_queue_0, log_queue_1);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "tx_diag schema=1 part=3 parts=5 uptime_ms=%" PRIu64 " slots=%d audio_pending=%d collapse_ms=%d ch=%u rate=5.5M hop=%" PRIu32,
                    log_uptime_ms, log_slots_0, log_audio_pending_0, log_collapse_ms_0, log_ch_0, log_hop_0);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "tx_diag schema=1 part=4 parts=5 uptime_ms=%" PRIu64 " fallback=%" PRIu32 " rescan=%" PRIu32 " paused=%d",
                    log_uptime_ms, log_fallback_0, log_rescan_0, log_paused_0);
            }
            dual_end_espnow_link_stats_t policy_stats;
            dual_end_espnow_link_get_stats(&policy_stats);
            dual_end_tx_pipe_stat_t pipe[TX_PIPE_COUNT];
            portENTER_CRITICAL(&s_tx_stats_lock);
            memcpy(pipe, s_tx_pipe, sizeof(pipe));
            portEXIT_CRITICAL(&s_tx_stats_lock);
            {
                /* Snapshot before pacing these background/startup diagnostics. */
                const uint64_t log_uptime_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
                const uint64_t log_queue_count_0 = pipe[TX_PIPE_QUEUE].count;
                const uint32_t log_queue_max_us_0 = pipe[TX_PIPE_QUEUE].max_us;
                const uint64_t log_peer_count_0 = pipe[TX_PIPE_PEER].count;
                const uint32_t log_peer_max_us_0 = pipe[TX_PIPE_PEER].max_us;
                const uint64_t log_prepare_count_0 = pipe[TX_PIPE_PREPARE].count;
                const uint32_t log_prepare_max_us_0 = pipe[TX_PIPE_PREPARE].max_us;
                const uint64_t log_api_count_0 = pipe[TX_PIPE_API].count;
                const uint32_t log_api_max_us_0 = pipe[TX_PIPE_API].max_us;
                const uint64_t log_callback_count_0 = pipe[TX_PIPE_CALLBACK].count;
                const uint32_t log_callback_max_us_0 = pipe[TX_PIPE_CALLBACK].max_us;
                const uint64_t log_wake_count_0 = pipe[TX_PIPE_WAKE].count;
                const uint32_t log_wake_max_us_0 = pipe[TX_PIPE_WAKE].max_us;
                ESP_LOGI(TAG,
                    "TX_PIPE schema=1 part=0 parts=3 uptime_ms=%" PRIu64 " queue_count=%" PRIu64 " queue_max_us=%" PRIu32 " peer_count=%" PRIu64 " peer_max_us=%" PRIu32,
                    log_uptime_ms, log_queue_count_0, log_queue_max_us_0, log_peer_count_0, log_peer_max_us_0);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "TX_PIPE schema=1 part=1 parts=3 uptime_ms=%" PRIu64 " prepare_count=%" PRIu64 " prepare_max_us=%" PRIu32 " api_count=%" PRIu64 " api_max_us=%" PRIu32,
                    log_uptime_ms, log_prepare_count_0, log_prepare_max_us_0, log_api_count_0, log_api_max_us_0);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "TX_PIPE schema=1 part=2 parts=3 uptime_ms=%" PRIu64 " callback_count=%" PRIu64 " callback_max_us=%" PRIu32 " wake_count=%" PRIu64 " wake_max_us=%" PRIu32,
                    log_uptime_ms, log_callback_count_0, log_callback_max_us_0, log_wake_count_0, log_wake_max_us_0);
            }
            {
                /* Snapshot before pacing these background/startup diagnostics. */
                const uint64_t log_uptime_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
                const uint32_t log_cb_ok_over_20_60_100ms_0 = policy_stats.tx_callback_success_over_us[0];
                const uint32_t log_cb_ok_over_20_60_100ms_1 = policy_stats.tx_callback_success_over_us[1];
                const uint32_t log_cb_ok_over_20_60_100ms_2 = policy_stats.tx_callback_success_over_us[2];
                const uint32_t log_cb_fail_over_20_60_100ms_0 = policy_stats.tx_callback_failure_over_us[0];
                const uint32_t log_cb_fail_over_20_60_100ms_1 = policy_stats.tx_callback_failure_over_us[1];
                const uint32_t log_cb_fail_over_20_60_100ms_2 = policy_stats.tx_callback_failure_over_us[2];
                const uint32_t log_queue_wait_max_us_0 = policy_stats.tx_queue_wait_max_us;
                const uint32_t log_submit_age_max_us_0 = policy_stats.tx_audio_submit_age_max_us;
                const uint32_t log_mac_ok_after_100ms_0 = policy_stats.tx_audio_mac_success_after_deadline;
                const uint32_t log_fast_retry_suppressed_0 = policy_stats.tx_audio_fast_retry_suppressed;
                ESP_LOGI(TAG,
                    "tx_policy schema=1 part=0 parts=3 uptime_ms=%" PRIu64 " cb_ok_over_20_60_100ms=[%" PRIu32 ",%" PRIu32 ",%" PRIu32 "] cb_fail_over_20_60_100ms=[%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]",
                    log_uptime_ms, log_cb_ok_over_20_60_100ms_0, log_cb_ok_over_20_60_100ms_1, log_cb_ok_over_20_60_100ms_2, log_cb_fail_over_20_60_100ms_0, log_cb_fail_over_20_60_100ms_1, log_cb_fail_over_20_60_100ms_2);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "tx_policy schema=1 part=1 parts=3 uptime_ms=%" PRIu64 " queue_wait_max_us=%" PRIu32 " submit_age_max_us=%" PRIu32 " mac_ok_after_100ms=%" PRIu32 " fast_retry_suppressed=%" PRIu32,
                    log_uptime_ms, log_queue_wait_max_us_0, log_submit_age_max_us_0, log_mac_ok_after_100ms_0, log_fast_retry_suppressed_0);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "tx_policy schema=1 part=2 parts=3 uptime_ms=%" PRIu64 " reserve_ms=12 original_limit=2/20ms",
                    log_uptime_ms);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(DUAL_END_CHANNEL_TASK_POLL_MS));
    }
}

static void on_espnow_send(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    const uint8_t *mac_addr = tx_info ? tx_info->des_addr : NULL;
    int64_t now_us = esp_timer_get_time();
    bool deliver = false;
    portENTER_CRITICAL(&s_tx_stats_lock);
    if (s_tx_callback_armed) {
        dual_end_tx_kind_t kind = s_tx_inflight_kind;
        int64_t elapsed_us = now_us - s_tx_inflight_start_us;
        uint32_t latency_us = elapsed_us > UINT32_MAX ? UINT32_MAX :
                              elapsed_us > 0 ? (uint32_t)elapsed_us : 0;
        s_tx_callback_armed = false;
        s_tx_callback_enter_us = now_us;
        s_tx_done_count++;
        s_last_send_status = (int)status;
        if (status == ESP_NOW_SEND_SUCCESS) {
            s_tx_ok_count++;
            if (kind == DUAL_END_TX_KIND_AUDIO && s_radio_state == AUDIO_RADIO_READY &&
                s_radio_awaiting_first_ok) {
                int64_t elapsed = now_us - s_radio_last_recovery_us;
                s_radio_first_ok_last_us = elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
                if (s_radio_first_ok_last_us > s_radio_first_ok_max_us) {
                    s_radio_first_ok_max_us = s_radio_first_ok_last_us;
                }
                s_radio_awaiting_first_ok = false;
                s_radio_recovery_dirty = true;
            }
        } else {
            s_tx_fail_count++;
        }
        if (s_tx_soft_timeout_active) {
            s_tx_callback_late_count++;
        }
        s_tx_callback_latency_last_us = latency_us;
        if (latency_us > s_tx_callback_latency_max_us) {
            s_tx_callback_latency_max_us = latency_us;
        }
        s_tx_callback_latency_buckets[tx_latency_bucket(latency_us)]++;
        const uint32_t thresholds_us[] = {20000U, 60000U, 100000U};
        uint32_t *tails = status == ESP_NOW_SEND_SUCCESS ?
                          s_tx_callback_success_over_us : s_tx_callback_failure_over_us;
        for (unsigned i = 0; i < 3; ++i) {
            if (latency_us > thresholds_us[i]) {
                tails[i]++;
            }
        }
        record_air_result_locked(kind, status, (uint32_t)(now_us / 1000LL));
        deliver = true;
    } else {
        s_tx_callback_orphan_count++;
    }
    portEXIT_CRITICAL(&s_tx_stats_lock);

    if (deliver && s_tx_callback_sem != NULL) {
        xSemaphoreGive(s_tx_callback_sem);
    }
    ESP_LOGD(TAG, "send status=%d mac=%02x:%02x:%02x:%02x:%02x:%02x",
             status,
             mac_addr ? mac_addr[0] : 0,
             mac_addr ? mac_addr[1] : 0,
             mac_addr ? mac_addr[2] : 0,
             mac_addr ? mac_addr[3] : 0,
             mac_addr ? mac_addr[4] : 0,
             mac_addr ? mac_addr[5] : 0);
    (void)mac_addr;
}

static void on_espnow_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (!radio_can_submit()) return;
    int rssi = 0;
    const uint8_t *src_addr = NULL;
    if (recv_info) {
        src_addr = recv_info->src_addr;
        rssi = recv_info->rx_ctrl ? recv_info->rx_ctrl->rssi : 0;
    }

    if (recv_info && recv_info->rx_ctrl &&
        recv_info->rx_ctrl->channel != s_current_channel) return;
    if (!is_peer_addr(src_addr)) {
        return;
    }

    /* Give authenticated remote control / OTA dispatch time to acquire its
     * application locks and publish maintenance state. HELLO alone is idle. */
    if (len < (int)sizeof(dual_end_packet_header_t) ||
        ((const dual_end_packet_header_t *)data)->message_type != SCEPTER_MSG_HELLO) {
        portENTER_CRITICAL(&s_idle_lock);
        int64_t until_us = esp_timer_get_time() + DUAL_END_WIFI_IDLE_FAULT_HOLD_US;
        if (until_us > s_idle_policy.hold_until_us) s_idle_policy.hold_until_us = until_us;
        portEXIT_CRITICAL(&s_idle_lock);
    }

    if (is_valid_framework_packet(data, len)) {
        portENTER_CRITICAL(&s_tx_stats_lock);
        if (s_radio_state == AUDIO_RADIO_READY && s_radio_awaiting_first_rx) {
            int64_t elapsed = esp_timer_get_time() - s_radio_last_recovery_us;
            s_radio_first_rx_last_us = elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
            if (s_radio_first_rx_last_us > s_radio_first_rx_max_us) {
                s_radio_first_rx_max_us = s_radio_first_rx_last_us;
            }
            s_radio_awaiting_first_rx = false;
            s_radio_recovery_dirty = true;
        }
        portEXIT_CRITICAL(&s_tx_stats_lock);
        mark_peer_seen(rssi);
        const dual_end_packet_header_t *header = (const dual_end_packet_header_t *)data;
        if (header->message_type == SCEPTER_MSG_HELLO) {
            return;
        }
        if (header->message_type == SCEPTER_MSG_CHANNEL_CONTROL) {
            if (len != (int)sizeof(dual_end_channel_control_packet_t) ||
                header->payload_len != DUAL_END_CHANNEL_CONTROL_PAYLOAD_BYTES) {
                ESP_LOGW(TAG, "invalid channel control length=%d payload=%u",
                         len, header->payload_len);
                return;
            }
            const dual_end_channel_control_packet_t *packet =
                (const dual_end_channel_control_packet_t *)data;
            if (packet->control_version != DUAL_END_CHANNEL_CONTROL_VERSION ||
                s_channel_event_queue == NULL) {
                return;
            }
            dual_end_channel_event_t event = {
                .command = packet->command,
                .channel = packet->channel,
                .flags = packet->flags,
                .generation = packet->generation,
                .switch_delay_ms = packet->switch_delay_ms,
                .channel_score = packet->channel_score,
                .received_ms = tx_now_ms(),
            };
            if (xQueueSend(s_channel_event_queue, &event, 0) != pdTRUE) {
                dual_end_channel_event_t stale;
                if (xQueueReceive(s_channel_event_queue, &stale, 0) == pdTRUE) {
                    (void)xQueueSend(s_channel_event_queue, &event, 0);
                }
                ESP_LOGW(TAG, "channel event queue overflow");
            }
            return;
        }
    }

    if (s_recv_cb) {
        s_recv_cb(src_addr, data, len, rssi, s_recv_ctx);
    } else {
        ESP_LOGD(TAG, "recv %d bytes rssi=%d", len, rssi);
    }
}

esp_err_t dual_end_espnow_link_init(scepter_role_t role)
{
    s_role = role;
    s_radio_suspended = false;
    s_idle_wifi_stopped = false;
    memset(&s_idle_policy, 0, sizeof(s_idle_policy));
    s_idle_stop_count = 0;
    s_idle_resume_count = 0;
    s_idle_fault_count = 0;
    s_idle_resume_max_us = 0;
    s_radio_state = AUDIO_RADIO_READY;
    s_radio_reason = AUDIO_RADIO_REASON_NONE;
    s_radio_forced_pending = false;
    s_radio_recovery_dirty = true;
    s_radio_generation = 0;
    s_radio_last_recovery_us = 0;
    s_radio_awaiting_first_ok = false;
    s_radio_awaiting_first_rx = false;
    s_connected = false;
    s_reported_connected = false;
    s_reported_rssi = 0;
    s_last_seen_us = 0;
    s_last_rssi = 0;
    s_hello_sequence = 0;
    s_hello_no_mem_count = 0;
    s_last_data_submit_ms = 0;
    s_tx_done_count = 0;
    s_tx_ok_count = 0;
    s_tx_fail_count = 0;
    s_tx_accepted_count = 0;
    s_tx_immediate_fail_count = 0;
    s_tx_callback_soft_timeout_count = 0;
    s_tx_callback_late_count = 0;
    s_tx_callback_hard_recovery_count = 0;
    s_tx_callback_recovery_fail_count = 0;
    s_tx_callback_orphan_count = 0;
    s_tx_callback_latency_last_us = 0;
    s_tx_callback_latency_max_us = 0;
    s_tx_queue_high_water = 0;
    memset(s_tx_callback_latency_buckets, 0, sizeof(s_tx_callback_latency_buckets));
    memset(s_tx_callback_success_over_us, 0, sizeof(s_tx_callback_success_over_us));
    memset(s_tx_callback_failure_over_us, 0, sizeof(s_tx_callback_failure_over_us));
    memset(&s_tx_audio_pacing, 0, sizeof(s_tx_audio_pacing));
    s_tx_queue_wait_max_us = 0;
    s_tx_audio_submit_age_max_us = 0;
    s_tx_audio_mac_success_after_deadline = 0;
    s_tx_audio_fast_retry_suppressed = 0;
    s_tx_audio_submit_count = 0;
    s_tx_audio_submit_fail_count = 0;
    s_tx_audio_stale_drop_count = 0;
    s_tx_audio_generation = 0;
    s_tx_deferred_input_flags = 0;
    s_tx_audio_final_ok = 0;
    s_tx_audio_final_fail = 0;
    s_tx_audio_retry_recovered = 0;
    s_tx_audio_repair_recovered = 0;
    s_tx_audio_deadline_drop = 0;
    s_tx_repair_ok = 0;
    s_tx_repair_fail = 0;
    s_tx_repair_deadline_drop = 0;
    memset(s_tx_pending_final, 0, sizeof(s_tx_pending_final));
    s_tx_ota_submit_count = 0;
    s_tx_ota_submit_fail_count = 0;
    s_last_send_status = 0;
    s_tx_inflight_kind = DUAL_END_TX_KIND_UNKNOWN;
    s_tx_callback_armed = false;
    s_tx_soft_timeout_active = false;
    memset(s_tx_pipe, 0, sizeof(s_tx_pipe));
    s_tx_callback_enter_us = 0;
    s_tx_inflight_start_us = 0;
    s_current_channel = DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL;
    s_channel_audio_paused = false;
    s_channel_peer_seen_after_switch = false;
    s_channel_rescan_requested = false;
    s_channel_pending_channel = DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL;
    s_channel_pending_generation = 0;
    s_channel_switch_due_ms = 0;
    s_channel_fallback_due_ms = 0;
    s_channel_pause_due_ms = 0;
    s_channel_disconnect_since_ms = 0;
    s_channel_last_rescan_request_ms = 0;
    s_channel_interference_window_start_ms = 0;
    s_channel_interference_packets = 0;
    s_channel_interference_failures = 0;
    s_channel_bad_windows = 0;
    s_channel_switch_count = 0;
    s_channel_fallback_count = 0;
    s_channel_rescan_count = 0;
    memset(s_tx_kind_stats, 0, sizeof(s_tx_kind_stats));
    memset(&s_stability_first, 0, sizeof(s_stability_first));
    memset(&s_stability_retry, 0, sizeof(s_stability_retry));
    memset(&s_stability_repair, 0, sizeof(s_stability_repair));
    memset(&s_audio_control_attempts, 0, sizeof(s_audio_control_attempts));
    memset(&s_active_pairing, 0, sizeof(s_active_pairing));
    s_active_pairing_valid = false;
    peer_state_set(NULL, false, false);

    /* ESP-NOW 使用保存在 RAM 中的 Wi-Fi 配置；仅配对设备信息持久化。 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "erase nvs");
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "init nvs");
    /* 先恢复上次确认成功的工作信道；若记录无效，channel_store_load 返回信道 6。 */
    s_current_channel = channel_store_load();
    s_channel_pending_channel = s_current_channel;
    s_peer_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_peer_mutex, ESP_ERR_NO_MEM, TAG, "create peer mutex");
    s_channel_event_queue = xQueueCreate(DUAL_END_CHANNEL_EVENT_QUEUE_DEPTH,
                                         sizeof(dual_end_channel_event_t));
    ESP_RETURN_ON_FALSE(s_channel_event_queue, ESP_ERR_NO_MEM, TAG,
                        "create channel event queue");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "init netif");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "create event loop");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "init wifi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_get_config(WIFI_IF_STA, &s_radio_sta_config), TAG,
                        "save station config for recovery");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start wifi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "wifi ps none");
    ESP_RETURN_ON_ERROR(esp_wifi_set_bandwidth(WIFI_IF_STA, DUAL_END_WIFI_BANDWIDTH_20MHZ),
                        TAG, "wifi 20MHz");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(s_current_channel, WIFI_SECOND_CHAN_NONE), TAG, "wifi channel");
    ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(SCEPTER_ESPNOW_TX_POWER_QDBM), TAG, "wifi tx power");

    uint8_t sta_mac[ESP_NOW_ETH_ALEN] = {0};
    ESP_RETURN_ON_ERROR(esp_wifi_get_mac(WIFI_IF_STA, sta_mac), TAG, "get sta mac");

    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "init esp-now");
    uint32_t espnow_version = 0;
    ESP_RETURN_ON_ERROR(esp_now_get_version(&espnow_version), TAG, "get esp-now version");
    ESP_LOGI(TAG,
             "ESP-NOW driver version=%" PRIu32
             " mode=v2 audio_payload=%d v1_max=%d v2_max=%d",
             espnow_version,
             DUAL_END_AUDIO_PACKET_BYTES,
             ESP_NOW_MAX_DATA_LEN,
             ESP_NOW_MAX_DATA_LEN_V2);
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(on_espnow_send), TAG, "send callback");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(on_espnow_recv), TAG, "recv callback");

    uint8_t stored_peer[ESP_NOW_ETH_ALEN] = {0};
    if (pairing_store_load(stored_peer)) {
        ESP_RETURN_ON_ERROR(configure_peer(stored_peer), TAG, "configure stored peer");
        peer_state_set(stored_peer, true, true);
    }
    ESP_RETURN_ON_ERROR(tx_worker_init(), TAG, "start serialized ESP-NOW TX worker");

    ESP_LOGI(TAG,
             "device_identity role=%s sta_mac=%02x:%02x:%02x:%02x:%02x:%02x",
             scepter_role_name(role),
             sta_mac[0], sta_mac[1], sta_mac[2],
             sta_mac[3], sta_mac[4], sta_mac[5]);

    if (s_peer_valid) {
        {
            /* Snapshot before pacing these background/startup diagnostics. */
            const char *const log_role = scepter_role_name(role);
            const uint64_t log_uptime_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
            const uint32_t log_peer_0 = stored_peer[0];
            const uint32_t log_peer_1 = stored_peer[1];
            const uint32_t log_peer_2 = stored_peer[2];
            const uint32_t log_peer_3 = stored_peer[3];
            const uint32_t log_peer_4 = stored_peer[4];
            const uint32_t log_peer_5 = stored_peer[5];
            const uint32_t log_ch_0 = s_current_channel;
            const uint32_t log_rendezvous_0 = DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL;
            const int32_t log_tx_power_qdbm_0 = SCEPTER_ESPNOW_TX_POWER_QDBM;
            const int32_t log_hello_0 = DUAL_END_HELLO_DISCONNECTED_INTERVAL_MS;
            const int32_t log_link_timeout_0 = DUAL_END_LINK_TIMEOUT_MS;
            const int32_t log_cb_soft_hard_0 = DUAL_END_TX_CALLBACK_SOFT_TIMEOUT_MS;
            const int32_t log_cb_soft_hard_1 = DUAL_END_TX_CALLBACK_HARD_TIMEOUT_MS;
            const int32_t log_slots_0 = DUAL_END_TX_SLOT_COUNT;
            const int32_t log_audio_pending_0 = DUAL_END_TX_AUDIO_PENDING_COUNT;
            ESP_LOGI(TAG,
                "ESP-NOW link ready for %.8s: part=0 parts=3 uptime_ms=%" PRIu64 " peer=%02x:%02x:%02x:%02x:%02x:%02x source=nvs ch=%u rendezvous=%u",
                log_role, log_uptime_ms, log_peer_0, log_peer_1, log_peer_2, log_peer_3, log_peer_4, log_peer_5, log_ch_0, log_rendezvous_0);
            vTaskDelay(1);
            ESP_LOGI(TAG,
                "ESP-NOW link ready for %.8s: part=1 parts=3 uptime_ms=%" PRIu64 " auto_channel=1 rate=5.5M tx_power_qdbm=%d hello=%dms link_timeout=%dms",
                log_role, log_uptime_ms, log_tx_power_qdbm_0, log_hello_0, log_link_timeout_0);
            vTaskDelay(1);
            ESP_LOGI(TAG,
                "ESP-NOW link ready for %.8s: part=2 parts=3 uptime_ms=%" PRIu64 " cb_soft/hard=%d/%dms slots=%d audio_pending=%d",
                log_role, log_uptime_ms, log_cb_soft_hard_0, log_cb_soft_hard_1, log_slots_0, log_audio_pending_0);
        }
    } else {
        ESP_LOGW(TAG,
                 "ESP-NOW link ready for %s without peer; write pairing/peer from host",
                 scepter_role_name(role));
    }

    BaseType_t task_ok = xTaskCreatePinnedToCore(link_status_task,
                                     "espnow_link",
                                     DUAL_END_LINK_TASK_STACK_BYTES,
                                     NULL,
                                     DUAL_END_LINK_TASK_PRIORITY,
                                     &s_link_task, DUAL_END_COMM_TASK_CORE);
    ESP_RETURN_ON_FALSE(task_ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create link status task");
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_send_input(const void *payload, int len)
{
    if (!payload || len < (int)sizeof(dual_end_packet_header_t) ||
        len > DUAL_END_TX_MAX_DATA_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }
    const dual_end_packet_header_t *header = (const dual_end_packet_header_t *)payload;
    if (header->protocol_version != SCEPTER_PROTOCOL_VERSION ||
        header->source_role != SCEPTER_ROLE_SCEPTER ||
        header->payload_len != (uint16_t)(len - sizeof(*header))) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = tx_submit_priority_async(payload, len, DUAL_END_TX_KIND_INPUT,
                                             false, esp_timer_get_time(), 0, 0);
    if (ret == ESP_OK) s_last_data_submit_ms = tx_now_ms();
    return ret;
}

esp_err_t dual_end_espnow_link_send_audio(const void *payload, int len)
{
    return dual_end_espnow_link_send_audio_at(payload, len, esp_timer_get_time());
}

esp_err_t dual_end_espnow_link_send_audio_at(const void *payload, int len, int64_t created_us)
{
    s_tx_audio_submit_count++;
    if (s_channel_audio_paused) {
        s_tx_audio_stale_drop_count++;
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = tx_submit_audio_async(payload, len, created_us);
    if (ret == ESP_OK) s_last_data_submit_ms = tx_now_ms();
    if (ret != ESP_OK) {
        s_tx_audio_submit_fail_count++;
    }
    return ret;
}

esp_err_t dual_end_espnow_link_send_audio_control(const void *payload, int len)
{
    return tx_submit_priority_async(payload, len, DUAL_END_TX_KIND_AUDIO_CONTROL,
                                    true, esp_timer_get_time(), 0, 0);
}

esp_err_t dual_end_espnow_link_send_audio_repair(const void *payload, int len,
                                               int64_t created_us,
                                               uint32_t reservation_id,
                                               int64_t deadline_us)
{
    if (payload == NULL || len != sizeof(dual_end_audio_repair_packet_t) ||
        reservation_id == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_channel_audio_paused || deadline_us <= 0 ||
        esp_timer_get_time() > deadline_us - DUAL_END_AUDIO_REPAIR_SEND_RESERVE_US) {
        return ESP_ERR_INVALID_STATE;
    }
    return tx_submit_priority_async(payload, len, DUAL_END_TX_KIND_AUDIO_REPAIR,
                                    false, created_us, reservation_id, deadline_us);
}

static esp_err_t tx_wait_ota_ready(uint32_t timeout_ms)
{
    const uint32_t started_ms = tx_now_ms();
    while (true) {
        channel_drop_queued_audio();

        bool audio_inflight;
        portENTER_CRITICAL(&s_tx_stats_lock);
        bool failed = s_radio_state == AUDIO_RADIO_FAILED_OFF || s_radio_suspended;
        bool ready = audio_radio_can_submit(s_radio_state) && !s_radio_forced_pending;
        audio_inflight = s_tx_callback_armed &&
                         (s_tx_inflight_kind == DUAL_END_TX_KIND_AUDIO ||
                          s_tx_inflight_kind == DUAL_END_TX_KIND_AUDIO_REPAIR ||
                          s_tx_inflight_kind == DUAL_END_TX_KIND_AUDIO_CONTROL);
        portEXIT_CRITICAL(&s_tx_stats_lock);

        if (failed) { return ESP_ERR_INVALID_STATE; }
        if (ready && !audio_inflight && uxQueueMessagesWaiting(s_tx_audio_queue) == 0 &&
            uxQueueMessagesWaiting(s_tx_input_queue) == 0) {
            return ESP_OK;
        }
        if (tx_now_ms() - started_ms >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t dual_end_espnow_link_prepare_ota(uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(timeout_ms > 0, ESP_ERR_INVALID_ARG, TAG,
                        "invalid OTA TX quiesce timeout");
    ESP_RETURN_ON_FALSE(s_tx_audio_queue && s_tx_free_queue && s_tx_task,
                        ESP_ERR_INVALID_STATE, TAG, "TX worker is not initialized");
    /* The caller pauses audio. A cleared callback debt does not mean the radio
     * is ready: full driver recovery may still be running. This polling wait
     * has a deadline, but the synchronous SDK rebuild APIs have no time bound. */
    dual_end_espnow_link_discard_audio();
    return tx_wait_ota_ready(timeout_ms);
}

void dual_end_espnow_link_set_ota_active(bool active)
{
    /* Compatibility entry point used by the dual-mic application.  Its audio
     * producer is paused in the same callback.  Purge admitted media when OTA
     * starts so the synchronous OTA path cannot sit behind stale frames. */
    if (active) {
        dual_end_espnow_link_discard_audio();
    }
}

esp_err_t dual_end_espnow_link_set_tx_power(int8_t tx_power_qdbm)
{
    return esp_wifi_set_max_tx_power(tx_power_qdbm);
}

esp_err_t dual_end_espnow_link_send_control(const void *payload, int len)
{
    return tx_submit_priority_async(payload, len, DUAL_END_TX_KIND_CONTROL,
                                    false, esp_timer_get_time(), 0, 0);
}

esp_err_t dual_end_espnow_link_send_ota(const void *payload, int len)
{
    s_tx_ota_submit_count++;
    esp_err_t ret = tx_submit_and_wait(payload, len, DUAL_END_TX_KIND_OTA);
    if (ret != ESP_OK) {
        s_tx_ota_submit_fail_count++;
    }
    return ret;
}

esp_err_t dual_end_espnow_link_set_peer(const uint8_t peer_mac[ESP_NOW_ETH_ALEN],
                                        bool persist)
{
    ESP_RETURN_ON_FALSE(peer_mac_is_valid(peer_mac),
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid peer mac");
    ESP_RETURN_ON_FALSE(s_peer_mutex, ESP_ERR_INVALID_STATE, TAG, "link is not initialized");

    uint8_t local_mac[ESP_NOW_ETH_ALEN] = {0};
    ESP_RETURN_ON_ERROR(esp_wifi_get_mac(WIFI_IF_STA, local_mac), TAG, "get local mac");
    ESP_RETURN_ON_FALSE(memcmp(local_mac, peer_mac, ESP_NOW_ETH_ALEN) != 0,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "peer mac equals local mac");

    ESP_RETURN_ON_ERROR(dual_end_espnow_link_resume_radio(), TAG, "resume RF for pairing maintenance");
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    uint8_t old_peer[ESP_NOW_ETH_ALEN] = {0};
    bool old_valid;
    bool old_from_nvs;
    dual_end_pairing_active_record_t old_active = s_active_pairing;
    bool old_active_valid = s_active_pairing_valid;
    portENTER_CRITICAL(&s_peer_state_lock);
    memcpy(old_peer, s_peer_mac, sizeof(old_peer));
    old_valid = s_peer_valid;
    old_from_nvs = s_peer_from_nvs;
    portEXIT_CRITICAL(&s_peer_state_lock);
    bool changed = !old_valid || memcmp(old_peer, peer_mac, ESP_NOW_ETH_ALEN) != 0;

    bool reuse_active = !persist && old_active_valid &&
                        memcmp(old_active.peer_mac,
                               peer_mac,
                               ESP_NOW_ETH_ALEN) == 0;
    if (!reuse_active) {
        dual_end_pairing_make_legacy(&s_active_pairing, peer_mac);
        s_active_pairing_valid = true;
    }
    esp_err_t ret = configure_peer(peer_mac);
    if (ret == ESP_OK && persist) {
        ret = pairing_store_save(peer_mac);
    }
    if (ret != ESP_OK) {
        if (changed && esp_now_is_peer_exist(peer_mac)) {
            (void)esp_now_del_peer(peer_mac);
        }
        s_active_pairing = old_active;
        s_active_pairing_valid = old_active_valid;
        if (old_valid) {
            (void)configure_peer(old_peer);
            peer_state_set(old_peer, true, old_from_nvs);
        }
        xSemaphoreGive(s_peer_mutex);
        return ret;
    }

    if (changed && old_valid && esp_now_is_peer_exist(old_peer)) {
        esp_err_t delete_ret = esp_now_del_peer(old_peer);
        if (delete_ret != ESP_OK) {
            ESP_LOGW(TAG, "old peer delete failed: %s", esp_err_to_name(delete_ret));
        }
    }
    peer_state_set(peer_mac, true, persist ? true : old_from_nvs && !changed);
    xSemaphoreGive(s_peer_mutex);

    s_connected = false;
    s_last_seen_us = 0;
    notify_status_if_changed(false, s_last_rssi);
    ESP_LOGI(TAG,
             "ESP-NOW peer updated: %02x:%02x:%02x:%02x:%02x:%02x persisted=%d",
             peer_mac[0], peer_mac[1], peer_mac[2],
             peer_mac[3], peer_mac[4], peer_mac[5],
             persist ? 1 : 0);
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_clear_peer(bool erase_nvs)
{
    ESP_RETURN_ON_FALSE(s_peer_mutex, ESP_ERR_INVALID_STATE, TAG, "link is not initialized");
    ESP_RETURN_ON_ERROR(dual_end_espnow_link_resume_radio(), TAG, "resume RF for pairing maintenance");
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    if (erase_nvs) {
        esp_err_t ret = pairing_store_clear();
        if (ret != ESP_OK) {
            xSemaphoreGive(s_peer_mutex);
            return ret;
        }
    }

    uint8_t old_peer[ESP_NOW_ETH_ALEN] = {0};
    bool old_valid;
    portENTER_CRITICAL(&s_peer_state_lock);
    memcpy(old_peer, s_peer_mac, sizeof(old_peer));
    old_valid = s_peer_valid;
    portEXIT_CRITICAL(&s_peer_state_lock);
    if (old_valid && esp_now_is_peer_exist(old_peer)) {
        esp_err_t ret = esp_now_del_peer(old_peer);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "peer delete failed while clearing: %s", esp_err_to_name(ret));
        }
    }
    memset(&s_active_pairing, 0, sizeof(s_active_pairing));
    s_active_pairing_valid = false;
    peer_state_set(NULL, false, false);
    xSemaphoreGive(s_peer_mutex);

    s_connected = false;
    s_last_seen_us = 0;
    notify_status_if_changed(false, 0);
    ESP_LOGI(TAG, "ESP-NOW peer cleared erase_nvs=%d", erase_nvs ? 1 : 0);
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_get_peer(uint8_t peer_mac[ESP_NOW_ETH_ALEN],
                                        bool *from_nvs)
{
    ESP_RETURN_ON_FALSE(peer_mac, ESP_ERR_INVALID_ARG, TAG, "peer output is null");
    bool valid;
    portENTER_CRITICAL(&s_peer_state_lock);
    valid = s_peer_valid;
    if (valid) {
        memcpy(peer_mac, s_peer_mac, ESP_NOW_ETH_ALEN);
    } else {
        memset(peer_mac, 0, ESP_NOW_ETH_ALEN);
    }
    if (from_nvs) {
        *from_nvs = valid && s_peer_from_nvs;
    }
    portEXIT_CRITICAL(&s_peer_state_lock);
    return valid ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static esp_err_t restore_pairing_previous_locked(
    const dual_end_pairing_pending_record_t *pending)
{
    esp_err_t ret = pending->previous_valid ?
                    dual_end_pairing_store_save_active(&pending->previous) :
                    dual_end_pairing_store_erase_active();
    if (ret != ESP_OK) {
        return ret;
    }

    if (pending->previous_valid) {
        s_active_pairing = pending->previous;
        s_active_pairing_valid = true;
        ret = configure_peer(pending->previous.peer_mac);
        if (ret != ESP_OK) {
            return ret;
        }
        peer_state_set(pending->previous.peer_mac, true, true);
    } else {
        memset(&s_active_pairing, 0, sizeof(s_active_pairing));
        s_active_pairing_valid = false;
        peer_state_set(NULL, false, false);
    }

    if ((!pending->previous_valid ||
         memcmp(pending->candidate.peer_mac,
                pending->previous.peer_mac,
                ESP_NOW_ETH_ALEN) != 0) &&
        esp_now_is_peer_exist(pending->candidate.peer_mac)) {
        (void)esp_now_del_peer(pending->candidate.peer_mac);
    }
    s_connected = false;
    s_last_seen_us = 0;
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_pair_prepare(
    const uint8_t peer_mac[ESP_NOW_ETH_ALEN],
    const uint8_t lmk[DUAL_END_PAIRING_KEY_BYTES],
    const uint8_t pmk[DUAL_END_PAIRING_KEY_BYTES],
    uint32_t generation,
    uint32_t transaction_id,
    const uint8_t key_id[DUAL_END_PAIRING_KEY_ID_BYTES])
{
    ESP_RETURN_ON_FALSE(s_peer_mutex, ESP_ERR_INVALID_STATE, TAG,
                        "link is not initialized");
    ESP_RETURN_ON_FALSE(peer_mac_is_valid(peer_mac) && generation != 0 &&
                        transaction_id != 0,
                        ESP_ERR_INVALID_ARG, TAG, "invalid secure pairing fields");
    uint8_t local_mac[ESP_NOW_ETH_ALEN] = {0};
    ESP_RETURN_ON_ERROR(esp_wifi_get_mac(WIFI_IF_STA, local_mac), TAG,
                        "get local mac");
    ESP_RETURN_ON_FALSE(memcmp(local_mac, peer_mac, ESP_NOW_ETH_ALEN) != 0,
                        ESP_ERR_INVALID_ARG, TAG, "peer mac equals local mac");

    dual_end_pairing_active_record_t candidate;
    dual_end_pairing_make_secure(&candidate, peer_mac, lmk, pmk, generation,
                                 transaction_id, key_id);
    ESP_RETURN_ON_FALSE(dual_end_pairing_active_is_valid(&candidate),
                        ESP_ERR_INVALID_ARG, TAG, "invalid pairing secret");

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    dual_end_pairing_pending_record_t existing;
    if (dual_end_pairing_store_load_pending(&existing)) {
        bool idempotent = existing.transaction_id == transaction_id &&
                          dual_end_pairing_records_equal(&existing.candidate,
                                                         &candidate);
        xSemaphoreGive(s_peer_mutex);
        return idempotent ? ESP_OK : ESP_ERR_INVALID_STATE;
    }
    if (s_active_pairing_valid &&
        dual_end_pairing_active_security(&s_active_pairing) ==
            DUAL_END_PAIRING_SECURITY_ENCRYPTED &&
        generation <= s_active_pairing.generation) {
        xSemaphoreGive(s_peer_mutex);
        ESP_LOGE(TAG,
                 "reject non-monotonic pairing generation new=%" PRIu32
                 " active=%" PRIu32,
                 generation, s_active_pairing.generation);
        return ESP_ERR_INVALID_STATE;
    }

    dual_end_pairing_pending_record_t pending;
    dual_end_pairing_make_pending(&pending,
                                  &candidate,
                                  s_active_pairing_valid ?
                                      &s_active_pairing : NULL);
    esp_err_t ret = dual_end_pairing_store_save_pending(&pending);
    xSemaphoreGive(s_peer_mutex);
    return ret;
}

esp_err_t dual_end_espnow_link_pair_commit(uint32_t transaction_id)
{
    ESP_RETURN_ON_FALSE(s_peer_mutex && transaction_id != 0,
                        ESP_ERR_INVALID_ARG, TAG, "invalid pairing transaction");
    ESP_RETURN_ON_ERROR(dual_end_espnow_link_resume_radio(), TAG, "resume RF for pairing maintenance");
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    dual_end_pairing_pending_record_t pending;
    if (!dual_end_pairing_store_load_pending(&pending)) {
        bool already_active = s_active_pairing_valid &&
                              s_active_pairing.transaction_id == transaction_id &&
                              dual_end_pairing_active_security(&s_active_pairing) ==
                                  DUAL_END_PAIRING_SECURITY_ENCRYPTED;
        xSemaphoreGive(s_peer_mutex);
        return already_active ? ESP_OK : ESP_ERR_NOT_FOUND;
    }
    if (pending.transaction_id != transaction_id) {
        xSemaphoreGive(s_peer_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (pending.state == DUAL_END_PAIRING_PENDING_COMMITTED) {
        xSemaphoreGive(s_peer_mutex);
        return ESP_OK;
    }

    esp_err_t ret = dual_end_pairing_store_save_active(&pending.candidate);
    if (ret == ESP_OK) {
        s_active_pairing = pending.candidate;
        s_active_pairing_valid = true;
        ret = configure_peer(pending.candidate.peer_mac);
    }
    if (ret == ESP_OK) {
        pending.state = DUAL_END_PAIRING_PENDING_COMMITTED;
        ret = dual_end_pairing_store_save_pending(&pending);
    }
    if (ret != ESP_OK) {
        esp_err_t restore_ret = restore_pairing_previous_locked(&pending);
        if (restore_ret != ESP_OK) {
            ESP_LOGE(TAG, "pair commit rollback failed: %s",
                     esp_err_to_name(restore_ret));
        }
        xSemaphoreGive(s_peer_mutex);
        return ret;
    }

    if (pending.previous_valid &&
        memcmp(pending.previous.peer_mac,
               pending.candidate.peer_mac,
               ESP_NOW_ETH_ALEN) != 0 &&
        esp_now_is_peer_exist(pending.previous.peer_mac)) {
        (void)esp_now_del_peer(pending.previous.peer_mac);
    }
    peer_state_set(pending.candidate.peer_mac, true, true);
    s_connected = false;
    s_last_seen_us = 0;
    xSemaphoreGive(s_peer_mutex);
    notify_status_if_changed(false, s_last_rssi);
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_pair_confirm(uint32_t transaction_id)
{
    ESP_RETURN_ON_FALSE(s_peer_mutex && transaction_id != 0,
                        ESP_ERR_INVALID_ARG, TAG, "invalid pairing transaction");
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    dual_end_pairing_pending_record_t pending;
    if (!dual_end_pairing_store_load_pending(&pending)) {
        bool confirmed = s_active_pairing_valid &&
                         s_active_pairing.transaction_id == transaction_id;
        xSemaphoreGive(s_peer_mutex);
        return confirmed ? ESP_OK : ESP_ERR_NOT_FOUND;
    }
    if (pending.transaction_id != transaction_id ||
        pending.state != DUAL_END_PAIRING_PENDING_COMMITTED ||
        !dual_end_pairing_records_equal(&pending.candidate,
                                        &s_active_pairing)) {
        xSemaphoreGive(s_peer_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_connected) {
        xSemaphoreGive(s_peer_mutex);
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t ret = dual_end_pairing_store_erase_pending();
    xSemaphoreGive(s_peer_mutex);
    return ret;
}

esp_err_t dual_end_espnow_link_pair_rollback(uint32_t transaction_id)
{
    ESP_RETURN_ON_FALSE(s_peer_mutex && transaction_id != 0,
                        ESP_ERR_INVALID_ARG, TAG, "invalid pairing transaction");
    ESP_RETURN_ON_ERROR(dual_end_espnow_link_resume_radio(), TAG, "resume RF for pairing maintenance");
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    dual_end_pairing_pending_record_t pending;
    if (!dual_end_pairing_store_load_pending(&pending)) {
        xSemaphoreGive(s_peer_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    if (pending.transaction_id != transaction_id) {
        xSemaphoreGive(s_peer_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = restore_pairing_previous_locked(&pending);
    if (ret == ESP_OK) {
        ret = dual_end_pairing_store_erase_pending();
    }
    xSemaphoreGive(s_peer_mutex);
    notify_status_if_changed(false, s_last_rssi);
    return ret;
}

esp_err_t dual_end_espnow_link_get_pairing_status(
    dual_end_pairing_status_t *status)
{
    ESP_RETURN_ON_FALSE(status, ESP_ERR_INVALID_ARG, TAG,
                        "pairing status is null");
    ESP_RETURN_ON_FALSE(s_peer_mutex, ESP_ERR_INVALID_STATE, TAG,
                        "link is not initialized");
    memset(status, 0, sizeof(*status));
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    status->paired = s_active_pairing_valid;
    status->from_nvs = s_active_pairing_valid && s_peer_from_nvs;
    if (s_active_pairing_valid) {
        status->security =
            dual_end_pairing_active_security(&s_active_pairing);
        memcpy(status->peer_mac, s_active_pairing.peer_mac,
               sizeof(status->peer_mac));
        status->generation = s_active_pairing.generation;
        memcpy(status->key_id, s_active_pairing.key_id,
               sizeof(status->key_id));
    }
    dual_end_pairing_pending_record_t pending;
    if (dual_end_pairing_store_load_pending(&pending)) {
        status->pending_state = pending.state;
        status->pending_transaction_id = pending.transaction_id;
    }
    xSemaphoreGive(s_peer_mutex);
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_suspend_radio(void)
{
    ESP_RETURN_ON_FALSE(s_peer_mutex, ESP_ERR_INVALID_STATE, TAG,
                        "link is not initialized");
    if (s_radio_suspended) {
        return ESP_OK;
    }

    /* Sleep must not carry an armed channel commit across the sleep interval.
     * The event worker and request API share this short admission lock. */
    portENTER_CRITICAL(&s_channel_request_lock);
    if (s_channel_status.pending || s_channel_user_queued || s_channel_event_processing ||
        (s_channel_event_queue && uxQueueMessagesWaiting(s_channel_event_queue))) {
        portEXIT_CRITICAL(&s_channel_request_lock);
        return ESP_ERR_INVALID_STATE;
    }
    s_radio_suspended = true;
    portEXIT_CRITICAL(&s_channel_request_lock);
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    tx_reject_queued_requests(s_tx_priority_queue);
    tx_reject_queued_requests(s_tx_audio_queue);
    tx_reject_queued_requests(s_tx_input_queue);
    tx_finish_inflight();
    if (s_tx_callback_sem != NULL) {
        (void)xSemaphoreTake(s_tx_callback_sem, 0);
    }

    (void)esp_now_unregister_send_cb();
    (void)esp_now_unregister_recv_cb();
    esp_err_t ret = esp_now_deinit();
    if (ret == ESP_ERR_ESPNOW_NOT_INIT) {
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_stop();
        if (ret == ESP_ERR_WIFI_NOT_STARTED) {
            ret = ESP_OK;
        }
    }
    if (ret == ESP_OK) {
        s_idle_wifi_stopped = false;
        portENTER_CRITICAL(&s_idle_lock);
        memset(&s_idle_policy, 0, sizeof(s_idle_policy));
        portEXIT_CRITICAL(&s_idle_lock);
        s_connected = false;
        portENTER_CRITICAL(&s_peer_state_lock);
        s_last_seen_us = 0;
        portEXIT_CRITICAL(&s_peer_state_lock);
    }
    xSemaphoreGive(s_peer_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "radio suspend failed: %s", esp_err_to_name(ret));
        esp_err_t rollback_ret = dual_end_espnow_link_resume_radio();
        if (rollback_ret != ESP_OK) {
            ESP_LOGE(TAG,
                     "radio suspend rollback failed: %s",
                     esp_err_to_name(rollback_ret));
        }
        return ret;
    }

    notify_status_if_changed(false, s_last_rssi);
    ESP_LOGI(TAG, "radio suspended: ESP-NOW deinitialized, Wi-Fi stopped");
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_resume_radio(void)
{
    ESP_RETURN_ON_FALSE(s_peer_mutex, ESP_ERR_INVALID_STATE, TAG,
                        "link is not initialized");
    /* Only the recovery owner may reopen a failed full rebuild. */
    ESP_RETURN_ON_FALSE(s_radio_state == AUDIO_RADIO_READY, ESP_ERR_INVALID_STATE, TAG,
                        "radio recovery is not ready");
    if (!s_radio_suspended) {
        xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
        portENTER_CRITICAL(&s_idle_lock);
        /* Explicit resume is a maintenance request, not an idle heartbeat. */
        s_idle_policy.hold_until_us = esp_timer_get_time() + DUAL_END_WIFI_IDLE_FAULT_HOLD_US;
        portEXIT_CRITICAL(&s_idle_lock);
        esp_err_t idle_ret = idle_radio_resume_locked();
        xSemaphoreGive(s_peer_mutex);
        return idle_ret;
    }

    uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0};
    bool peer_valid;
    portENTER_CRITICAL(&s_peer_state_lock);
    memcpy(peer_mac, s_peer_mac, sizeof(peer_mac));
    peer_valid = s_peer_valid;
    portEXIT_CRITICAL(&s_peer_state_lock);

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    esp_err_t ret = esp_wifi_start();
    if (ret == ESP_OK) {
        ret = esp_wifi_set_ps(WIFI_PS_NONE);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_set_bandwidth(WIFI_IF_STA, DUAL_END_WIFI_BANDWIDTH_20MHZ);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_set_channel(s_current_channel, WIFI_SECOND_CHAN_NONE);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_set_max_tx_power(SCEPTER_ESPNOW_TX_POWER_QDBM);
    }
    if (ret == ESP_OK) {
        ret = esp_now_init();
    }
    if (ret == ESP_OK) {
        ret = esp_now_register_send_cb(on_espnow_send);
    }
    if (ret == ESP_OK) {
        ret = esp_now_register_recv_cb(on_espnow_recv);
    }
    if (ret == ESP_OK && peer_valid) {
        ret = configure_peer(peer_mac);
    }

    if (ret == ESP_OK) {
        if (s_tx_callback_sem != NULL) {
            (void)xSemaphoreTake(s_tx_callback_sem, 0);
        }
        tx_finish_inflight();
        s_radio_suspended = false;
        s_idle_wifi_stopped = false;
        portENTER_CRITICAL(&s_idle_lock);
        memset(&s_idle_policy, 0, sizeof(s_idle_policy));
        dual_end_wifi_idle_resumed(&s_idle_policy, esp_timer_get_time());
        portEXIT_CRITICAL(&s_idle_lock);
    } else {
        (void)esp_now_unregister_send_cb();
        (void)esp_now_unregister_recv_cb();
        (void)esp_now_deinit();
        (void)esp_wifi_stop();
    }
    xSemaphoreGive(s_peer_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "radio resume failed; radio kept off: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG,
             "radio resumed: Wi-Fi/ESP-NOW ready ch=%u peer=%d",
             s_current_channel,
             peer_valid ? 1 : 0);
    return ESP_OK;
}

bool dual_end_espnow_link_is_radio_suspended(void)
{
    return s_radio_suspended;
}

esp_err_t dual_end_espnow_link_recover_peer(const uint8_t *peer_mac)
{
    ESP_RETURN_ON_ERROR(dual_end_espnow_link_resume_radio(), TAG,
                        "resume radio before peer recovery");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(s_current_channel, WIFI_SECOND_CHAN_NONE),
                         TAG,
                         "restore wifi channel");
    ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(SCEPTER_ESPNOW_TX_POWER_QDBM),
                        TAG,
                        "restore wifi tx power");
    uint8_t current_peer[ESP_NOW_ETH_ALEN] = {0};
    if (!peer_mac) {
        esp_err_t ret = dual_end_espnow_link_get_peer(current_peer, NULL);
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "ESP-NOW recovery skipped: device is not paired");
            return ESP_OK;
        }
        ESP_RETURN_ON_ERROR(ret, TAG, "get current peer");
        peer_mac = current_peer;
    }
    return dual_end_espnow_link_set_peer(peer_mac, false);
}

esp_err_t dual_end_espnow_link_send_hello(void)
{
    return send_hello();
}

esp_err_t dual_end_espnow_link_register_recv_cb(dual_end_espnow_recv_cb_t cb, void *ctx)
{
    s_recv_cb = cb;
    s_recv_ctx = ctx;
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_register_status_cb(dual_end_espnow_status_cb_t cb, void *ctx)
{
    s_status_cb = cb;
    s_status_ctx = ctx;
    if (s_status_cb) {
        s_status_cb(s_connected, s_last_rssi, s_status_ctx);
    }
    return ESP_OK;
}

bool dual_end_espnow_link_is_connected(void)
{
    return s_connected;
}

bool dual_end_espnow_link_is_audio_paused(void)
{
    return s_channel_audio_paused;
}

static void copy_kind_stats(dual_end_espnow_tx_type_stats_t *dest,
                            const dual_end_tx_kind_stats_t *source,
                            uint32_t now_ms)
{
    dest->done = source->done;
    dest->ok = source->ok;
    dest->fail = source->fail;
    dest->consecutive_fail = source->consecutive_fail;
    dest->consecutive_fail_ms = source->consecutive_fail > 0 ?
                                now_ms - source->fail_streak_start_ms : 0;
    dest->last_fail_streak = source->last_fail_streak;
    dest->last_fail_streak_ms = source->last_fail_streak_ms;
    dest->max_fail_streak = source->max_fail_streak;
    dest->max_fail_streak_ms = source->max_fail_streak_ms;
}

void dual_end_espnow_link_get_stats(dual_end_espnow_link_stats_t *out_stats)
{
    if (!out_stats) {
        return;
    }

    out_stats->tx_done = s_tx_done_count;
    out_stats->tx_ok = s_tx_ok_count;
    out_stats->tx_fail = s_tx_fail_count;
    out_stats->tx_accepted = s_tx_accepted_count;
    out_stats->tx_immediate_fail = s_tx_immediate_fail_count;
    out_stats->tx_callback_soft_timeout = s_tx_callback_soft_timeout_count;
    out_stats->tx_callback_late = s_tx_callback_late_count;
    out_stats->tx_callback_hard_recovery = s_tx_callback_hard_recovery_count;
    out_stats->tx_callback_recovery_fail = s_tx_callback_recovery_fail_count;
    out_stats->tx_callback_orphan = s_tx_callback_orphan_count;
    out_stats->tx_callback_latency_last_us = s_tx_callback_latency_last_us;
    out_stats->tx_callback_latency_max_us = s_tx_callback_latency_max_us;
    out_stats->tx_queue_depth = tx_queue_depth();
    out_stats->tx_queue_high_water = s_tx_queue_high_water;
    out_stats->tx_callback_debt = s_tx_callback_armed ? 1U : 0U;
    out_stats->tx_audio_submit = s_tx_audio_submit_count;
    out_stats->tx_audio_submit_fail = s_tx_audio_submit_fail_count;
    out_stats->tx_audio_stale_drop = s_tx_audio_stale_drop_count;
    out_stats->tx_audio_final_ok = s_tx_audio_final_ok;
    out_stats->tx_audio_final_fail = s_tx_audio_final_fail;
    out_stats->tx_audio_retry_recovered = s_tx_audio_retry_recovered;
    out_stats->tx_audio_repair_recovered = s_tx_audio_repair_recovered;
    out_stats->tx_audio_deadline_drop = s_tx_audio_deadline_drop;
    out_stats->tx_repair_ok = s_tx_repair_ok;
    out_stats->tx_repair_fail = s_tx_repair_fail;
    out_stats->tx_repair_deadline_drop = s_tx_repair_deadline_drop;
    out_stats->tx_ota_submit = s_tx_ota_submit_count;
    out_stats->tx_ota_submit_fail = s_tx_ota_submit_fail_count;
    out_stats->hello_no_mem = s_hello_no_mem_count;
    out_stats->last_send_status = s_last_send_status;
    out_stats->connected = s_connected;
    out_stats->last_rssi = s_last_rssi;

    uint32_t now_ms = tx_now_ms();
    portENTER_CRITICAL(&s_tx_stats_lock);
    memcpy(out_stats->tx_callback_success_over_us, s_tx_callback_success_over_us,
           sizeof(s_tx_callback_success_over_us));
    memcpy(out_stats->tx_callback_failure_over_us, s_tx_callback_failure_over_us,
           sizeof(s_tx_callback_failure_over_us));
    out_stats->tx_queue_wait_max_us = s_tx_queue_wait_max_us;
    out_stats->tx_audio_submit_age_max_us = s_tx_audio_submit_age_max_us;
    out_stats->tx_audio_mac_success_after_deadline = s_tx_audio_mac_success_after_deadline;
    out_stats->tx_audio_fast_retry_suppressed = s_tx_audio_fast_retry_suppressed;
    copy_kind_stats(&out_stats->audio,
                    &s_tx_kind_stats[DUAL_END_TX_KIND_AUDIO],
                    now_ms);
    copy_kind_stats(&out_stats->hello,
                    &s_tx_kind_stats[DUAL_END_TX_KIND_HELLO],
                    now_ms);
    copy_kind_stats(&out_stats->ota,
                    &s_tx_kind_stats[DUAL_END_TX_KIND_OTA],
                    now_ms);
    portEXIT_CRITICAL(&s_tx_stats_lock);
}
