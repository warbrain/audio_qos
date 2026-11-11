#include "dual_end_espnow_link.h"
#include "dual_end_runtime.h"
#include "dual_end_audio_recovery_protocol.h"
#include "dual_end_secure_pairing.h"
#include "audio_radio_recovery_policy.h"
#include "dual_end_channel_guard.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
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

static const char *TAG = "espnow_link";

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
/* Protected input queue merge and timed stereo decode share this RX worker. */
/* esp_audio_codec recommends about 20 KiB for its decoder owner task. */
#define DUAL_END_RX_TASK_STACK_BYTES (20 * 1024)
#define DUAL_END_RX_TASK_PRIORITY 6
#define DUAL_END_RX_QUEUE_DEPTH 4
#define DUAL_END_RX_INPUT_QUEUE_DEPTH 16
#define DUAL_END_AUDIO_FEEDBACK_QUEUE_DEPTH 4
#define DUAL_END_CONTROL_QUEUE_DEPTH 8
#define DUAL_END_TX_CALLBACK_SOFT_TIMEOUT_MS 100
#define DUAL_END_TX_CALLBACK_HARD_TIMEOUT_MS ((int)(AUDIO_RADIO_HARD_TIMEOUT_US / 1000LL))
#define DUAL_END_TX_DIAG_INTERVAL_MS 30000
#define DUAL_END_TX_LATENCY_BUCKET_COUNT 7
/* Dongle 是信道协调端：扫描、提议、提交、确认及失败回退都在链路任务中串行完成。 */
#define DUAL_END_CHANNEL_TASK_POLL_MS 50
#define DUAL_END_CHANNEL_STARTUP_SETTLE_MS 1500
#define DUAL_END_CHANNEL_SWITCH_DELAY_MS 600
#define DUAL_END_CHANNEL_CONFIRM_TIMEOUT_MS 15000
#define DUAL_END_CHANNEL_SCAN_COOLDOWN_MS 30000
#define DUAL_END_CHANNEL_OTA_QUIET_MS 5000
#define DUAL_END_CHANNEL_PROPOSAL_RETRIES 3
#define DUAL_END_CHANNEL_ACCEPT_WAIT_MS 500
#define DUAL_END_CHANNEL_SCAN_MAX_APS 64
#define DUAL_END_CHANNEL_CANDIDATE_COUNT 3
#define PAIRING_NVS_NAMESPACE "pairing"
#define LINK_CONFIG_NVS_NAMESPACE "link_cfg"
#define CHANNEL_NVS_KEY "channel"

typedef struct {
    uint8_t src_addr[ESP_NOW_ETH_ALEN];
    uint8_t data[DUAL_END_ESPNOW_MAX_DATA_BYTES];
    uint16_t len;
    int8_t rssi;
    int64_t received_us;
} dual_end_rx_packet_t;

typedef struct {
    uint8_t data[64];
    int len;
    int64_t deadline_us;
} dual_end_audio_feedback_t;

/* The caller retains its payload until completion. Only the TX worker invokes
 * the SDK, including synchronous OTA/control callers from other tasks. */
typedef struct {
    const void *payload;
    int len;
    SemaphoreHandle_t done;
    esp_err_t result;
    bool check_peer;
    uint8_t expected_peer[ESP_NOW_ETH_ALEN];
    int64_t deadline_us;
} dual_end_control_request_t;

static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN];
static bool s_peer_valid;
static bool s_peer_from_nvs;
static dual_end_pairing_active_record_t s_active_pairing;
static bool s_active_pairing_valid;
static dual_end_espnow_recv_cb_t s_recv_cb;
static void *s_recv_ctx;
static dual_end_espnow_poll_cb_t s_poll_cb;
static void *s_poll_ctx;
static int64_t s_rx_received_us;
static QueueHandle_t s_audio_feedback_queue;
static QueueHandle_t s_control_queue;
static QueueHandle_t s_control_requests;
static TaskHandle_t s_tx_task;
static QueueHandle_t s_audio_nack_results;
static dual_end_nack_mailbox_t s_audio_nack_mailbox;
static portMUX_TYPE s_audio_nack_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_audio_feedback_drop_count;
static uint32_t s_audio_feedback_fail_count;
static uint32_t s_session_feedback_attempt_count;
static uint32_t s_session_feedback_mac_ok_count;
static uint32_t s_session_feedback_deferred_count;
static uint32_t s_session_feedback_fail_count;
static int64_t s_session_feedback_last_log_us;
static dual_end_espnow_status_cb_t s_status_cb;
static void *s_status_ctx;
static dual_end_espnow_hello_cb_t s_hello_cb;
static dual_end_espnow_recv_cb_t s_control_response_cb;
static dual_end_espnow_control_service_cb_t s_control_service_cb;
static void *s_control_cb_ctx;
static QueueHandle_t s_rx_input_queue;
static bool s_idle_hello_pending;
static int64_t s_idle_hello_last_us;
static void *s_hello_ctx;
static scepter_role_t s_role;
static TaskHandle_t s_link_task;
static TaskHandle_t s_rx_task;
static QueueHandle_t s_rx_queue;
static SemaphoreHandle_t s_peer_mutex;
static SemaphoreHandle_t s_tx_callback_sem;
static volatile bool s_connected;
static bool s_reported_connected;
static int s_reported_rssi;
static int64_t s_last_seen_us;
static int s_last_rssi;
static uint32_t s_hello_sequence;
static uint32_t s_hello_no_mem_count;
static volatile uint32_t s_rx_queue_drop_count;
static volatile uint32_t s_rx_invalid_len_count;
static volatile int s_last_send_status;
static volatile uint32_t s_tx_accepted_count;
static volatile uint32_t s_tx_immediate_fail_count;
static volatile uint32_t s_tx_callback_count;
static volatile uint32_t s_tx_callback_soft_timeout_count;
static volatile uint32_t s_tx_callback_late_count;
static volatile uint32_t s_tx_callback_hard_recovery_count;
static volatile uint32_t s_tx_callback_recovery_fail_count;
static volatile uint32_t s_tx_callback_orphan_count;
static volatile uint32_t s_tx_callback_latency_last_us;
static volatile uint32_t s_tx_callback_latency_max_us;
static uint32_t s_tx_callback_latency_buckets[DUAL_END_TX_LATENCY_BUCKET_COUNT];
static uint32_t s_tx_success_over_us[3]; /* Strictly >20, >60, >100 ms. */
static uint32_t s_tx_failure_over_us[3];
static volatile bool s_tx_callback_armed;
static volatile bool s_tx_soft_timeout_active;
static volatile int64_t s_tx_inflight_start_us;
static volatile uint8_t s_current_channel;
static int64_t s_channel_switched_us;
static bool s_channel_configuration_dirty;
static uint8_t s_channel_startup_best;
static int16_t s_channel_startup_score;
static volatile bool s_channel_startup_pending;
static volatile bool s_channel_rescan_pending;
static volatile bool s_channel_force_avoid;
static volatile bool s_channel_peer_seen_after_switch;
static volatile uint32_t s_channel_accept_generation;
static volatile uint32_t s_channel_confirm_generation;
static uint8_t s_channel_desired;
static uint32_t s_channel_generation;
static uint32_t s_channel_fallback_due_ms;
static uint32_t s_channel_disconnect_since_ms;
static uint32_t s_channel_connected_since_ms;
static uint32_t s_channel_last_scan_ms;
static volatile uint32_t s_channel_last_ota_activity_ms;
static volatile bool s_ota_active;
static uint32_t s_channel_scan_count;
static uint32_t s_channel_switch_count;
static uint32_t s_channel_fallback_count;
static uint32_t s_channel_rescan_request_count;
static portMUX_TYPE s_tx_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_peer_state_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    bool connected;
    int rssi;
} link_state_snapshot_t;

typedef struct {
    audio_radio_state_t state;
    audio_radio_reason_t reason;
    uint32_t generation, early, hard, forced, success, fail;
    uint32_t reset_last_us, reset_max_us;
    uint32_t first_cb_last_us, first_cb_max_us;
    uint32_t first_rx_last_us, first_rx_max_us;
    bool first_cb_pending, first_rx_pending;
    esp_err_t last_error;
} dual_end_radio_stats_t;
static dual_end_radio_stats_t s_radio = {.state = AUDIO_RADIO_FAILED_OFF};
static bool s_radio_forced_pending;
static int64_t s_radio_last_recovery_us;
static int64_t s_radio_recovery_started_us;
static uint32_t s_radio_revision;
static bool s_radio_initializing;

static void on_espnow_send(const esp_now_send_info_t *tx_info,
                           esp_now_send_status_t status);
static void on_espnow_recv(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data,
                           int len);
static esp_err_t configure_peer(const uint8_t *peer_mac);

static portMUX_TYPE s_channel_request_lock = portMUX_INITIALIZER_UNLOCKED;
static dual_end_channel_status_t s_channel_status;
static bool s_channel_user_queued;
static uint8_t s_channel_user_target;
static bool s_channel_user_avoid;

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
static esp_err_t send_control_deadline(const void *payload, int len, int64_t deadline_us,
                                       bool allow_pending_nack);
static esp_err_t send_control_peer_deadline(const uint8_t *expected_peer,
    const void *payload, int len, int64_t deadline_us, bool allow_pending_nack);
static void audio_feedback_send_nack(void);
static bool audio_feedback_worker_send_session(void);
static void audio_feedback_log_session_tx(void);
static bool radio_ready(void);
static void radio_service_forced(void);
static void tx_worker_service_sync(void);
static void radio_publish_status(void);
static bool radio_lock_configuration(void);

static void audio_feedback_worker(void *arg)
{
    (void)arg;
    dual_end_audio_feedback_t feedback;
    while (true) {
        radio_service_forced();
        audio_feedback_log_session_tx();
        /* Session ACK/RESYNC has a 60 ms delivery deadline. Drain it before
         * synchronous control work or repair NACKs can occupy this worker. */
        if (audio_feedback_worker_send_session()) {
            vTaskDelay(1);
            continue;
        }
        /* A not-yet-submitted ordinary control never holds the worker ahead
         * of a refreshable NACK. Only an accepted MAC transaction can block. */
        audio_feedback_send_nack();
        tx_worker_service_sync();
        QueueHandle_t queue = s_control_queue;
        if (xQueuePeek(queue, &feedback, 0) == pdTRUE) {
            esp_err_t result = send_control_deadline(feedback.data, feedback.len,
                                                     feedback.deadline_us, false);
            if (result != ESP_ERR_NO_MEM) {
                (void)xQueueReceive(queue, &feedback, 0);
                if (result != ESP_OK) ++s_audio_feedback_fail_count;
            }
        }
        vTaskDelay(1);
    }
}

static bool audio_feedback_worker_send_session(void)
{
    dual_end_audio_feedback_t feedback;
    if (s_audio_feedback_queue == NULL ||
        xQueuePeek(s_audio_feedback_queue, &feedback, 0) != pdTRUE) {
        return false;
    }

    bool is_session = false;
    if (feedback.len == (int)sizeof(dual_end_audio_session_packet_t)) {
        const dual_end_audio_session_packet_t *packet =
            (const dual_end_audio_session_packet_t *)feedback.data;
        is_session = packet->packet.message_type == SCEPTER_MSG_AUDIO_SESSION &&
                     packet->version == DUAL_END_AUDIO_RECOVERY_VERSION &&
                     (packet->command == DUAL_END_AUDIO_SESSION_ACK ||
                      packet->command == DUAL_END_AUDIO_SESSION_RESYNC);
    }
    if (!is_session) {
        (void)xQueueReceive(s_audio_feedback_queue, &feedback, 0);
        ++s_audio_feedback_fail_count;
        ++s_session_feedback_fail_count;
        ESP_LOGW(TAG, "invalid queued session feedback dropped");
        return true;
    }

    if (s_session_feedback_attempt_count != UINT32_MAX) {
        ++s_session_feedback_attempt_count;
    }
    esp_err_t result = send_control_deadline(feedback.data, feedback.len,
                                             feedback.deadline_us, true);
    if (result == ESP_ERR_NO_MEM) {
        if (s_session_feedback_deferred_count != UINT32_MAX) {
            ++s_session_feedback_deferred_count;
        }
    } else {
        (void)xQueueReceive(s_audio_feedback_queue, &feedback, 0);
        if (result == ESP_OK) {
            if (s_session_feedback_mac_ok_count != UINT32_MAX) {
                ++s_session_feedback_mac_ok_count;
            }
        } else {
            if (s_audio_feedback_fail_count != UINT32_MAX) ++s_audio_feedback_fail_count;
            if (s_session_feedback_fail_count != UINT32_MAX) ++s_session_feedback_fail_count;
        }
    }
    return true;
}

static void audio_feedback_log_session_tx(void)
{
    const int64_t now_us = esp_timer_get_time();
    if (now_us - s_session_feedback_last_log_us < 10000000LL) return;
    s_session_feedback_last_log_us = now_us;
    const UBaseType_t queued = s_audio_feedback_queue != NULL ?
        uxQueueMessagesWaiting(s_audio_feedback_queue) : 0;
    ESP_LOGI(TAG, "SESSION_FEEDBACK_TX schema=1 attempts=%" PRIu32
             " mac_ok=%" PRIu32 " deferred_no_mem=%" PRIu32
             " fail=%" PRIu32 " queue_drop=%" PRIu32 " queued=%" PRIu32,
             s_session_feedback_attempt_count, s_session_feedback_mac_ok_count,
             s_session_feedback_deferred_count, s_session_feedback_fail_count,
             s_audio_feedback_drop_count, (uint32_t)queued);
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

static uint32_t link_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

/* Connection and RSSI are reported as one state observation. Keep both reads
 * and all connection-state writes behind the peer-state lock. */
static link_state_snapshot_t link_state_snapshot(void)
{
    link_state_snapshot_t snapshot;
    portENTER_CRITICAL(&s_peer_state_lock);
    snapshot.connected = s_connected;
    snapshot.rssi = s_last_rssi;
    portEXIT_CRITICAL(&s_peer_state_lock);
    return snapshot;
}

static link_state_snapshot_t link_state_invalidate(void)
{
    link_state_snapshot_t snapshot;
    portENTER_CRITICAL(&s_peer_state_lock);
    s_connected = false;
    s_last_seen_us = 0;
    snapshot.connected = s_connected;
    snapshot.rssi = s_last_rssi;
    portEXIT_CRITICAL(&s_peer_state_lock);
    return snapshot;
}

static link_state_snapshot_t link_state_refresh_from_last_seen(int64_t now_us)
{
    link_state_snapshot_t snapshot;
    portENTER_CRITICAL(&s_peer_state_lock);
    s_connected = s_last_seen_us > 0 &&
                  (now_us - s_last_seen_us) <= ((int64_t)DUAL_END_LINK_TIMEOUT_MS * 1000);
    snapshot.connected = s_connected;
    snapshot.rssi = s_last_rssi;
    portEXIT_CRITICAL(&s_peer_state_lock);
    return snapshot;
}

static bool channel_is_valid(uint8_t channel)
{
    return channel >= DUAL_END_ESPNOW_CHANNEL_MIN &&
           channel <= DUAL_END_ESPNOW_CHANNEL_MAX;
}


static esp_err_t channel_queue_user_request(uint8_t target, bool avoid_current)
{
    const link_state_snapshot_t link_state = link_state_snapshot();
    if (!s_link_task || !link_state.connected || !radio_ready()) return ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&s_channel_request_lock);
    if (s_channel_status.pending || s_channel_user_queued) {
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
    status->scan_count = s_channel_scan_count;
    status->switch_count = s_channel_switch_count;
    status->fallback_count = s_channel_fallback_count;
    status->rx_stack_min_bytes = s_rx_task ? uxTaskGetStackHighWaterMark(s_rx_task) : 0;
    status->tx_stack_min_bytes = s_tx_task ? uxTaskGetStackHighWaterMark(s_tx_task) : 0;
    status->link_stack_min_bytes = uxTaskGetStackHighWaterMark(s_link_task);
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_get_diagnostics(dual_end_espnow_link_diagnostics_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(*out));

    dual_end_channel_status_t channel;
    const esp_err_t status = dual_end_espnow_link_get_channel_status(&channel);
    if (status != ESP_OK) return status;

    const link_state_snapshot_t link_state = link_state_snapshot();
    int rssi = link_state.rssi;
    out->connected = link_state.connected;
    if (rssi < -128) rssi = -128;
    if (rssi > 127) rssi = 127;
    out->rssi = (int8_t)rssi;

    out->channel_pending = channel.pending;
    out->current_channel = channel.current_channel;
    out->channel_state = channel.state;
    out->channel_generation = channel.generation;
    out->rx_queue_drops = s_rx_queue_drop_count;
    out->rx_invalid_lengths = s_rx_invalid_len_count;
    out->rx_stack_min_bytes = channel.rx_stack_min_bytes;
    out->tx_stack_min_bytes = channel.tx_stack_min_bytes;
    out->link_stack_min_bytes = channel.link_stack_min_bytes;

    /* The TX callback and radio-recovery counters share this lock with their
     * writers. This read is bounded and has no radio/SDK side effects. */
    portENTER_CRITICAL(&s_tx_stats_lock);
    out->tx_accepted = s_tx_accepted_count;
    out->tx_callbacks = s_tx_callback_count;
    out->tx_immediate_failures = s_tx_immediate_fail_count;
    out->tx_soft_timeouts = s_tx_callback_soft_timeout_count;
    out->tx_late_callbacks = s_tx_callback_late_count;
    out->tx_hard_recoveries = s_tx_callback_hard_recovery_count;
    out->tx_recovery_failures = s_tx_callback_recovery_fail_count;
    portEXIT_CRITICAL(&s_tx_stats_lock);

    out->ready = true;
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

static bool channel_claim_automatic(void)
{
    portENTER_CRITICAL(&s_channel_request_lock);
    bool allowed = !s_channel_user_queued && !s_channel_status.pending;
    if (allowed) s_channel_status.pending = true;
    portEXIT_CRITICAL(&s_channel_request_lock);
    return allowed;
}

static bool channel_time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return due_ms != 0 && (int32_t)(now_ms - due_ms) >= 0;
}

/* NVS 无记录或记录非法时统一回到信道 6，确保两端仍有固定会合点。 */
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

static void tx_arm_callback(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_last_send_status = -1;
    s_tx_inflight_start_us = esp_timer_get_time();
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

static void tx_finish_inflight(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_tx_callback_armed = false;
    s_tx_soft_timeout_active = false;
    s_tx_inflight_start_us = 0;
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static uint32_t radio_duration_us(int64_t elapsed)
{
    return elapsed > UINT32_MAX ? UINT32_MAX : elapsed > 0 ? (uint32_t)elapsed : 0;
}

static bool radio_ready(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    bool ready = audio_radio_can_submit(s_radio.state) && !s_radio_forced_pending;
    portEXIT_CRITICAL(&s_tx_stats_lock);
    return ready;
}

static bool radio_lock_configuration(void)
{
    if (!s_peer_mutex || xSemaphoreTake(s_peer_mutex, portMAX_DELAY) != pdTRUE) return false;
    if (!radio_ready()) {
        xSemaphoreGive(s_peer_mutex);
        return false;
    }
    return true;
}

static void radio_set_state(audio_radio_state_t state)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_radio.state = state;
    ++s_radio_revision;
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

/* Always attempt both Wi-Fi stop and deinit, even if ESP-NOW cleanup failed.
 * Only successful deinit (or already uninitialized) fences the old Wi-Fi task.
 * An epoch number is deliberately never used to match an SDK callback. */
static esp_err_t radio_stop_stack(void)
{
    esp_err_t first = ESP_OK;
    esp_err_t ret = esp_now_unregister_send_cb();
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_NOT_INIT) first = ret;
    ret = esp_now_unregister_recv_cb();
    if (first == ESP_OK && ret != ESP_OK && ret != ESP_ERR_ESPNOW_NOT_INIT) first = ret;
    ret = esp_now_deinit();
    if (first == ESP_OK && ret != ESP_OK && ret != ESP_ERR_ESPNOW_NOT_INIT) first = ret;
    ret = esp_wifi_stop();
    if (first == ESP_OK && ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT &&
        ret != ESP_ERR_WIFI_NOT_STARTED) first = ret;
    ret = esp_wifi_deinit();
    if (first == ESP_OK && ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT) first = ret;
    return first;
}

/* TX worker only, with the peer mutex held throughout the complete lifecycle.
 * The application's netif/event loop and audio timeline remain allocated. */
static esp_err_t radio_recover_locked(audio_radio_reason_t reason)
{
    const int64_t started_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_radio.state = AUDIO_RADIO_QUIESCING;
    s_radio.reason = reason;
    ++s_radio.generation;
    if (reason == AUDIO_RADIO_REASON_EARLY) ++s_radio.early;
    if (reason == AUDIO_RADIO_REASON_HARD) { ++s_radio.hard; ++s_tx_callback_hard_recovery_count; }
    if (reason == AUDIO_RADIO_REASON_FORCED) ++s_radio.forced;
    s_radio_forced_pending = false;
    s_radio_recovery_started_us = started_us;
    s_radio.first_cb_pending = s_radio.first_rx_pending = false;
    ++s_radio_revision;
    portEXIT_CRITICAL(&s_tx_stats_lock);

    radio_set_state(AUDIO_RADIO_RECOVERING);
    esp_err_t ret = radio_stop_stack();
    if (ret == ESP_OK) {
        /* The old task has ended; old semaphore debt can now be retired. */
        tx_finish_inflight();
        (void)xSemaphoreTake(s_tx_callback_sem, 0);
        radio_set_state(AUDIO_RADIO_RESTORING);
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
#define RADIO_RESTORE(call) do { ret = (call); if (ret != ESP_OK) goto finished; } while (0)
        RADIO_RESTORE(esp_wifi_init(&cfg));
        RADIO_RESTORE(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        RADIO_RESTORE(esp_wifi_set_mode(WIFI_MODE_STA));
        RADIO_RESTORE(esp_wifi_start());
        RADIO_RESTORE(esp_wifi_set_ps(WIFI_PS_NONE));
        RADIO_RESTORE(esp_wifi_set_bandwidth(WIFI_IF_STA, DUAL_END_WIFI_BANDWIDTH_20MHZ));
        RADIO_RESTORE(esp_wifi_set_channel(s_current_channel, WIFI_SECOND_CHAN_NONE));
        RADIO_RESTORE(esp_wifi_set_max_tx_power(SCEPTER_ESPNOW_TX_POWER_QDBM));
        RADIO_RESTORE(esp_now_init());
        RADIO_RESTORE(esp_now_register_send_cb(on_espnow_send));
        RADIO_RESTORE(esp_now_register_recv_cb(on_espnow_recv));
        if (s_peer_valid) RADIO_RESTORE(configure_peer(s_peer_mac));
#undef RADIO_RESTORE
    }
finished:
    if (ret != ESP_OK) (void)radio_stop_stack();
    const int64_t ended_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_tx_stats_lock);
    s_radio.reset_last_us = radio_duration_us(ended_us - started_us);
    if (s_radio.reset_last_us > s_radio.reset_max_us) s_radio.reset_max_us = s_radio.reset_last_us;
    s_radio.last_error = ret;
    s_radio_last_recovery_us = ended_us > 0 ? ended_us : 1;
    if (ret == ESP_OK) {
        ++s_radio.success;
        s_radio.first_cb_pending = s_radio.first_rx_pending = true;
        s_radio.state = AUDIO_RADIO_READY;
    } else {
        ++s_radio.fail;
        ++s_tx_callback_recovery_fail_count;
        s_radio.state = AUDIO_RADIO_FAILED_OFF;
    }
    ++s_radio_revision;
    portEXIT_CRITICAL(&s_tx_stats_lock);
    return ret;
}

esp_err_t dual_end_espnow_link_request_radio_recovery(void)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_tx_stats_lock);
    bool accepted = s_tx_task && audio_radio_can_submit(s_radio.state) && !s_radio_forced_pending &&
        audio_radio_manual_recovery_allowed(now_us, s_radio_last_recovery_us);
    if (accepted) s_radio_forced_pending = true;
    portEXIT_CRITICAL(&s_tx_stats_lock);
    return accepted ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static void radio_service_forced(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    bool requested = s_radio_forced_pending && audio_radio_can_submit(s_radio.state);
    portEXIT_CRITICAL(&s_tx_stats_lock);
    if (!requested || xSemaphoreTake(s_peer_mutex, 0) != pdTRUE) return;
    (void)radio_recover_locked(AUDIO_RADIO_REASON_FORCED);
    xSemaphoreGive(s_peer_mutex);
}

static esp_err_t tx_wait_for_callback(int len)
{
    (void)len;
    for (;;) {
        /* A completed transaction is settled before a pending forced reset.
         * The request is then serviced on the next TX-worker iteration. */
        if (xSemaphoreTake(s_tx_callback_sem, 0) == pdTRUE) return ESP_OK;
        const int64_t now_us = esp_timer_get_time();
        portENTER_CRITICAL(&s_tx_stats_lock);
        const int64_t submitted_us = s_tx_inflight_start_us;
        audio_radio_reason_t reason = audio_radio_recovery_due(now_us, submitted_us,
            s_radio_last_recovery_us, s_radio_forced_pending);
        bool armed = s_tx_callback_armed;
        if (reason != AUDIO_RADIO_REASON_NONE && armed) {
            /* Serialize with callback entry before closing the old epoch. */
            s_radio.state = AUDIO_RADIO_QUIESCING;
        }
        portEXIT_CRITICAL(&s_tx_stats_lock);
        if (now_us - submitted_us >= DUAL_END_TX_CALLBACK_SOFT_TIMEOUT_MS * 1000LL) {
            (void)tx_mark_soft_timeout(); /* Deferred telemetry, never log here. */
        }
        if (reason != AUDIO_RADIO_REASON_NONE && armed) {
            (void)radio_recover_locked(reason);
            return ESP_ERR_TIMEOUT; /* The unknown accepted operation is never replayed. */
        }
        if (xSemaphoreTake(s_tx_callback_sem, pdMS_TO_TICKS(AUDIO_RADIO_WAIT_SLICE_MS)) == pdTRUE) {
            return ESP_OK;
        }
    }
}

void dual_end_espnow_link_note_audio_received(int64_t received_us)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_tx_stats_lock);
    if (s_radio.state == AUDIO_RADIO_READY && s_radio.first_rx_pending &&
        received_us >= s_radio_recovery_started_us) {
        s_radio.first_rx_pending = false;
        s_radio.first_rx_last_us = radio_duration_us(now_us - s_radio_recovery_started_us);
        if (s_radio.first_rx_last_us > s_radio.first_rx_max_us) s_radio.first_rx_max_us = s_radio.first_rx_last_us;
        ++s_radio_revision;
    }
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

static void radio_publish_status(void)
{
    static uint32_t reported_revision = UINT32_MAX;
    static int64_t reported_us;
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_tx_stats_lock);
    dual_end_radio_stats_t stats = s_radio;
    uint32_t revision = s_radio_revision;
    portEXIT_CRITICAL(&s_tx_stats_lock);
    if (revision == reported_revision && now_us - reported_us < 10000000LL) return;
    reported_revision = revision;
    reported_us = now_us;
    {
        /* Snapshot before pacing these background/startup diagnostics. */
        const uint64_t log_uptime_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
        const uint32_t log_state_0 = (unsigned)stats.state;
        const uint32_t log_reason_0 = (unsigned)stats.reason;
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
        const uint32_t log_first_cb_pending_0 = (unsigned)stats.first_cb_pending;
        const uint32_t log_first_rx_last_us_0 = stats.first_rx_last_us;
        const uint32_t log_first_rx_max_us_0 = stats.first_rx_max_us;
        const uint32_t log_first_rx_pending_0 = (unsigned)stats.first_rx_pending;
        const int32_t log_last_error_0 = stats.last_error;
        ESP_LOGI(TAG,
            "RADIO_RECOVERY schema=1 part=0 parts=4 uptime_ms=%" PRIu64 " state=%u reason=%u generation=%" PRIu32 " early=%" PRIu32 " hard=%" PRIu32 " forced=%" PRIu32,
            log_uptime_ms, log_state_0, log_reason_0, log_generation_0, log_early_0, log_hard_0, log_forced_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "RADIO_RECOVERY schema=1 part=1 parts=4 uptime_ms=%" PRIu64 " success=%" PRIu32 " fail=%" PRIu32 " reset_last_us=%" PRIu32 " reset_max_us=%" PRIu32 " first_cb_last_us=%" PRIu32,
            log_uptime_ms, log_success_0, log_fail_0, log_reset_last_us_0, log_reset_max_us_0, log_first_cb_last_us_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "RADIO_RECOVERY schema=1 part=2 parts=4 uptime_ms=%" PRIu64 " first_cb_max_us=%" PRIu32 " first_cb_pending=%u first_rx_last_us=%" PRIu32 " first_rx_max_us=%" PRIu32,
            log_uptime_ms, log_first_cb_max_us_0, log_first_cb_pending_0, log_first_rx_last_us_0, log_first_rx_max_us_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "RADIO_RECOVERY schema=1 part=3 parts=4 uptime_ms=%" PRIu64 " first_rx_pending=%u last_error=%d",
            log_uptime_ms, log_first_rx_pending_0, log_last_error_0);
    }
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
    portENTER_CRITICAL(&s_peer_state_lock);
    s_last_seen_us = esp_timer_get_time();
    s_last_rssi = rssi;
    s_connected = true;
    portEXIT_CRITICAL(&s_peer_state_lock);
    if (s_channel_fallback_due_ms != 0) {
        s_channel_peer_seen_after_switch = true;
    }
}

/* Preserve complete stereo audio packets across ordinary control/OTA backlog.
 * HID state is sampled inside each original 400-byte audio packet, so keeping
 * audio in the protected FIFO also prevents input state from being delayed by
 * lower-priority traffic. Normal audio overtakes an older repair packet; other
 * control traffic remains ordered by radio reception time. */
static bool rx_take_next_packet(dual_end_rx_packet_t *packet)
{
    dual_end_rx_packet_t regular;
    dual_end_rx_packet_t input;
    bool have_regular = xQueuePeek(s_rx_queue, &regular, 0) == pdTRUE;
    bool have_input = xQueuePeek(s_rx_input_queue, &input, 0) == pdTRUE;
    if (!have_regular && !have_input) return false;
    bool regular_is_repair = have_regular &&
        regular.len >= sizeof(dual_end_packet_header_t) &&
        ((const dual_end_packet_header_t *)regular.data)->message_type ==
            SCEPTER_MSG_AUDIO_REPAIR;
    QueueHandle_t queue = have_input &&
        (!have_regular || regular_is_repair ||
         input.received_us <= regular.received_us) ?
        s_rx_input_queue : s_rx_queue;
    return xQueueReceive(queue, packet, 0) == pdTRUE;
}

static esp_err_t send_hello(void);

static void idle_hello_service(void)
{
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_peer_state_lock);
    if (!s_idle_hello_pending ||
        (s_idle_hello_last_us != 0 && now_us - s_idle_hello_last_us < 100000)) {
        portEXIT_CRITICAL(&s_peer_state_lock);
        return;
    }
    s_idle_hello_pending = false;
    s_idle_hello_last_us = now_us;
    portEXIT_CRITICAL(&s_peer_state_lock);
    /* Offer a newly opened RF window before the optional HELLO send can block. */
    if (s_control_service_cb) s_control_service_cb(true, s_control_cb_ctx);
    /* The app callback also flushes remote controls queued while RF was off. */
    if (s_hello_cb) s_hello_cb(s_hello_ctx);
    else (void)send_hello();
}

static void idle_hello_worker(void *arg)
{
    (void)arg;
    while (true) {
        if (s_control_service_cb) s_control_service_cb(false, s_control_cb_ctx);
        idle_hello_service();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void rx_worker_task(void *arg)
{
    (void)arg;
    dual_end_rx_packet_t packet;

    while (true) {
        TickType_t poll_ticks = pdMS_TO_TICKS(2);
        if (!rx_take_next_packet(&packet)) {
            if (s_poll_cb) s_poll_cb(s_poll_ctx);
            /* Queue admission wakes this sole owner immediately. A bounded
             * timeout still advances recovery/NACK deadlines without input. */
            (void)ulTaskNotifyTake(pdTRUE, poll_ticks ? poll_ticks : 1);
            continue;
        }

        /* Packets queued on the old channel cannot confirm a new channel. */
        if (packet.received_us < s_channel_switched_us) continue;
        if (!is_peer_addr(packet.src_addr)) {
            continue;
        }

        if (is_valid_framework_packet(packet.data, packet.len)) {
            mark_peer_seen(packet.rssi);
            const dual_end_packet_header_t *header =
                (const dual_end_packet_header_t *)packet.data;
            if (header->message_type == SCEPTER_MSG_CONTROL_RESPONSE) {
                if (s_control_response_cb) {
                    s_control_response_cb(packet.src_addr, packet.data, packet.len,
                                          packet.rssi, s_control_cb_ctx);
                }
                continue;
            }
            if (header->message_type == SCEPTER_MSG_HELLO) {
                /* Keep one consumer of the app's queued remote controls. */
                portENTER_CRITICAL(&s_peer_state_lock);
                s_idle_hello_pending = true;
                portEXIT_CRITICAL(&s_peer_state_lock);
                continue;
            }
            /* 接收任务只登记协商结果和重扫请求，耗时扫描与切换交给链路任务。 */
            if (header->message_type == SCEPTER_MSG_CHANNEL_CONTROL) {
                if (packet.len == sizeof(dual_end_channel_control_packet_t) &&
                    header->payload_len == DUAL_END_CHANNEL_CONTROL_PAYLOAD_BYTES) {
                    const dual_end_channel_control_packet_t *control =
                        (const dual_end_channel_control_packet_t *)packet.data;
                    if (control->control_version == DUAL_END_CHANNEL_CONTROL_VERSION) {
                        switch (control->command) {
                        case DUAL_END_CHANNEL_CMD_SWITCH_ACCEPT:
                            if (control->channel == s_channel_desired &&
                                control->generation == s_channel_generation) {
                                s_channel_accept_generation = control->generation;
                            }
                            break;
                        case DUAL_END_CHANNEL_CMD_SWITCH_CONFIRM:
                            if (control->channel == s_current_channel &&
                                control->generation == s_channel_generation) {
                                s_channel_confirm_generation = control->generation;
                                s_channel_peer_seen_after_switch = true;
                            }
                            break;
                        case DUAL_END_CHANNEL_CMD_RESCAN_REQUEST:
                            if (control->flags & DUAL_END_CHANNEL_FLAG_MANUAL_TARGET) {
                                (void)dual_end_espnow_link_request_channel(control->channel);
                            } else {
                                (void)dual_end_espnow_link_request_channel_rescan(
                                    (control->flags & DUAL_END_CHANNEL_FLAG_INTERFERENCE) != 0);
                            }
                            s_channel_rescan_request_count++;
                            break;
                        default:
                            break;
                        }
                    }
                }
                continue;
            }
        }

        if (s_recv_cb) {
            s_rx_received_us = packet.received_us;
            s_recv_cb(packet.src_addr,
                      packet.data,
                      packet.len,
                      packet.rssi,
                      s_recv_ctx);
        } else {
            ESP_LOGD(TAG, "recv %u bytes rssi=%d", packet.len, packet.rssi);
        }
    }
}

static esp_err_t rx_worker_init(void)
{
    s_control_requests = xQueueCreate(4, sizeof(dual_end_control_request_t *));
    ESP_RETURN_ON_FALSE(s_control_requests, ESP_ERR_NO_MEM, TAG, "create synchronous control queue");
    s_control_queue = xQueueCreate(DUAL_END_CONTROL_QUEUE_DEPTH, sizeof(dual_end_audio_feedback_t));
    ESP_RETURN_ON_FALSE(s_control_queue, ESP_ERR_NO_MEM, TAG, "create async control queue");
    s_audio_feedback_queue = xQueueCreate(DUAL_END_AUDIO_FEEDBACK_QUEUE_DEPTH,
                                         sizeof(dual_end_audio_feedback_t));
    ESP_RETURN_ON_FALSE(s_audio_feedback_queue, ESP_ERR_NO_MEM, TAG, "create audio feedback queue");
    s_audio_nack_results = xQueueCreate(DUAL_END_AUDIO_FEEDBACK_QUEUE_DEPTH,
                                      sizeof(dual_end_nack_result_t));
    ESP_RETURN_ON_FALSE(s_audio_nack_results, ESP_ERR_NO_MEM, TAG, "create NACK admission results");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(audio_feedback_worker, "audio_feedback", 4096,
                                    NULL, 5, &s_tx_task, DUAL_END_COMM_TASK_CORE) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create audio feedback worker");
    s_rx_queue = xQueueCreate(DUAL_END_RX_QUEUE_DEPTH, sizeof(dual_end_rx_packet_t));
    ESP_RETURN_ON_FALSE(s_rx_queue, ESP_ERR_NO_MEM, TAG, "create ESP-NOW RX queue");

    s_rx_input_queue = xQueueCreate(DUAL_END_RX_INPUT_QUEUE_DEPTH, sizeof(dual_end_rx_packet_t));
    ESP_RETURN_ON_FALSE(s_rx_input_queue, ESP_ERR_NO_MEM, TAG, "create protected input RX queue");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(idle_hello_worker, "idle_hello", 4096,
                                    NULL, 4, NULL, DUAL_END_COMM_TASK_CORE) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create idle heartbeat response worker");
    BaseType_t task_ok = xTaskCreatePinnedToCore(rx_worker_task,
                                     "espnow_rx",
                                     DUAL_END_RX_TASK_STACK_BYTES,
                                     NULL,
                                     DUAL_END_RX_TASK_PRIORITY,
                                     &s_rx_task, DUAL_END_AUDIO_TASK_CORE);
    if (task_ok != pdPASS) {
        vQueueDelete(s_rx_queue);
        vQueueDelete(s_rx_input_queue);
        s_rx_input_queue = NULL;
        s_rx_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
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
    return dual_end_espnow_link_queue_control(&hello, sizeof(hello));
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
    return dual_end_espnow_link_send_control(&packet, sizeof(packet));
}

static esp_err_t channel_switch_runtime(uint8_t channel, const char *reason)
{
    ESP_RETURN_ON_FALSE(channel_is_valid(channel), ESP_ERR_INVALID_ARG, TAG,
                        "invalid runtime channel");
    ESP_RETURN_ON_FALSE(s_peer_mutex, ESP_ERR_INVALID_STATE, TAG,
                        "link is not initialized");
    if (channel == s_current_channel && !s_channel_configuration_dirty) {
        return ESP_OK;
    }

    if (!radio_lock_configuration()) return ESP_ERR_INVALID_STATE;
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
                s_channel_configuration_dirty = true;
                s_channel_peer_seen_after_switch = false;
                (void)link_state_invalidate();
                s_channel_fallback_due_ms = link_now_ms() + DUAL_END_CHANNEL_CONFIRM_TIMEOUT_MS;
                ESP_LOGE(TAG, "channel rollback failed: %s", esp_err_to_name(rollback));
            }
        }
    }
    if (ret == ESP_OK) {
        uint8_t old_channel = s_current_channel;
        s_current_channel = channel;
        s_channel_switched_us = esp_timer_get_time();
        s_channel_configuration_dirty = false;
        (void)link_state_invalidate();
        ESP_LOGW(TAG, "channel switch %u -> %u reason=%s",
                 old_channel, channel, reason ? reason : "unknown");
    }
    xSemaphoreGive(s_peer_mutex);
    return ret;
}

/* 仅比较 1/6/11 三个互不重叠的 20 MHz 候选信道，分数越低表示周边占用越轻。 */
static esp_err_t channel_scan_best(bool avoid_current,
                                   uint8_t *best_channel,
                                   int16_t *best_score)
{
    static const uint8_t candidates[DUAL_END_CHANNEL_CANDIDATE_COUNT] = {1, 6, 11};
    ESP_RETURN_ON_FALSE(best_channel && best_score, ESP_ERR_INVALID_ARG, TAG,
                        "channel scan output is null");

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_PASSIVE,
        .scan_time.passive = 60,
        .home_chan_dwell_time = 30,
    };

    wifi_ap_record_t *records = NULL;
    uint16_t record_count = 0;
    esp_err_t ret;
    /* 扫描会临时离开当前工作信道，因此与 ESP-NOW peer 更新共用同一互斥锁。 */
    if (s_radio_initializing) {
        xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    } else if (!radio_lock_configuration()) {
        return ESP_ERR_INVALID_STATE;
    }
    ret = esp_wifi_scan_start(&scan_config, true);
    if (ret == ESP_OK) {
        uint16_t ap_count = 0;
        ret = esp_wifi_scan_get_ap_num(&ap_count);
        if (ret == ESP_OK && ap_count > 0) {
            record_count = ap_count > DUAL_END_CHANNEL_SCAN_MAX_APS ?
                           DUAL_END_CHANNEL_SCAN_MAX_APS : ap_count;
            records = calloc(record_count, sizeof(*records));
            if (records == NULL) {
                ret = ESP_ERR_NO_MEM;
            } else {
                ret = esp_wifi_scan_get_ap_records(&record_count, records);
            }
        }
    }
    /* 无论扫描结果如何都先恢复原工作信道，后续协商仍必须从现有链路发出。 */
    esp_err_t restore_ret = esp_wifi_set_channel(s_current_channel,
                                                  WIFI_SECOND_CHAN_NONE);
    if (restore_ret == ESP_OK && s_peer_valid) {
        restore_ret = configure_peer(s_peer_mac);
    }
    if (restore_ret != ESP_OK) {
        uint8_t actual = s_current_channel;
        wifi_second_chan_t secondary;
        if (esp_wifi_get_channel(&actual, &secondary) == ESP_OK) s_current_channel = actual;
        s_channel_configuration_dirty = true;
                s_channel_peer_seen_after_switch = false;
        (void)link_state_invalidate();
        s_channel_fallback_due_ms = link_now_ms() + DUAL_END_CHANNEL_CONFIRM_TIMEOUT_MS;
    }
    xSemaphoreGive(s_peer_mutex);
    if (ret != ESP_OK || restore_ret != ESP_OK) {
        free(records);
        return restore_ret != ESP_OK ? restore_ret : ret;
    }

    /* 强 AP 权重更高，并按与候选信道的距离递减计入相邻信道干扰。 */
    uint32_t cost[DUAL_END_CHANNEL_CANDIDATE_COUNT] = {0};
    for (uint16_t i = 0; i < record_count; i++) {
        int strength = 100 + records[i].rssi;
        if (strength < 5) {
            strength = 5;
        } else if (strength > 70) {
            strength = 70;
        }
        for (size_t c = 0; c < DUAL_END_CHANNEL_CANDIDATE_COUNT; c++) {
            int delta = (int)records[i].primary - (int)candidates[c];
            if (delta < 0) {
                delta = -delta;
            }
            if (delta <= 4) {
                cost[c] += (uint32_t)(strength * (5 - delta));
            }
        }
    }
    free(records);

    size_t selected = 0;
    for (size_t c = 0; c < DUAL_END_CHANNEL_CANDIDATE_COUNT; c++) {
        if (candidates[c] == s_current_channel) {
            selected = c;
        }
        /* 明确收到干扰请求时抬高当前信道成本，促使选择可用的其他候选信道。 */
        if (avoid_current && candidates[c] == s_current_channel) {
            cost[c] += 10000;
        }
    }
    for (size_t c = 0; c < DUAL_END_CHANNEL_CANDIDATE_COUNT; c++) {
        if (cost[c] < cost[selected]) {
            selected = c;
        }
    }
    *best_channel = candidates[selected];
    if (cost[selected] > (uint32_t)INT16_MAX) {
        *best_score = INT16_MAX;
    } else {
        /* Mask proves the unsigned value is representable before conversion;
         * this avoids implementation-defined narrowing on embedded compilers. */
        *best_score = (int16_t)(cost[selected] & (uint32_t)INT16_MAX);
    }
    s_channel_scan_count++;
    s_channel_last_scan_ms = link_now_ms();
    ESP_LOGI(TAG,
             "channel scan aps=%u cost=[ch1:%" PRIu32 ",ch6:%" PRIu32
             ",ch11:%" PRIu32 "] selected=%u avoid_current=%d",
             record_count, cost[0], cost[1], cost[2], *best_channel,
             avoid_current ? 1 : 0);
    return ESP_OK;
}

/* 每轮 generation 唯一：PROPOSE/ACCEPT 建立共识，重复 COMMIT 降低单包丢失风险。 */
static esp_err_t channel_negotiate_switch(uint8_t target_channel, int16_t channel_score, uint8_t flags, uint32_t generation)
{
    if (!channel_is_valid(target_channel) || target_channel == s_current_channel) {
        channel_status_update(DUAL_END_CHANNEL_SUCCEEDED, s_current_channel,
                              s_channel_generation, ESP_OK, false);
        return ESP_OK;
    }
    if (!generation) generation = dual_end_channel_next_token(s_channel_generation);
    s_channel_generation = generation;
    channel_status_update(DUAL_END_CHANNEL_PROPOSING, target_channel, generation, ESP_OK, true);
    s_channel_desired = target_channel;
    s_channel_accept_generation = 0;

    bool accepted = false;
    for (unsigned attempt = 0; attempt < DUAL_END_CHANNEL_PROPOSAL_RETRIES; attempt++) {
        esp_err_t ret = send_channel_control(DUAL_END_CHANNEL_CMD_SWITCH_PROPOSE,
                                             target_channel,
                                             flags,
                                             generation,
                                             DUAL_END_CHANNEL_SWITCH_DELAY_MS,
                                             channel_score);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "channel proposal attempt=%u failed: %s",
                     attempt + 1, esp_err_to_name(ret));
        }
        for (uint32_t waited = 0; waited < DUAL_END_CHANNEL_ACCEPT_WAIT_MS;
             waited += DUAL_END_CHANNEL_TASK_POLL_MS) {
            if (s_channel_accept_generation == generation) {
                accepted = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(DUAL_END_CHANNEL_TASK_POLL_MS));
        }
        if (accepted) {
            break;
        }
    }
    if (!accepted) {
        (void)send_channel_control(DUAL_END_CHANNEL_CMD_SCAN_END,
                                   s_current_channel, flags | DUAL_END_CHANNEL_FLAG_FAILED, generation, 0, 0);
        return ESP_ERR_TIMEOUT;
    }

    /* COMMIT 连发两次；至少一次进入发送队列后双方才按同一延时切换。 */
    unsigned commit_ok = 0;
    uint32_t commit_started_ms = link_now_ms();
    channel_status_update(DUAL_END_CHANNEL_SWITCHING, target_channel, generation, ESP_OK, true);
    for (unsigned attempt = 0; attempt < 2; attempt++) {
        if (send_channel_control(DUAL_END_CHANNEL_CMD_SWITCH_COMMIT,
                                 target_channel, flags, generation,
                                 DUAL_END_CHANNEL_SWITCH_DELAY_MS, channel_score) == ESP_OK) {
            commit_ok++;
            break; /* Do not let a second callback delay both ends differently. */
        }
    }
    if (commit_ok == 0) {
        (void)send_channel_control(DUAL_END_CHANNEL_CMD_SCAN_END,
                                   s_current_channel, flags | DUAL_END_CHANNEL_FLAG_FAILED, generation, 0, 0);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "channel switch armed generation=%" PRIu32 " %u -> %u",
             generation, s_current_channel, target_channel);
    /* 等待期间 Scepter 已暂停并清空旧信道音频，避免切换后发送过期帧。 */
    uint32_t commit_elapsed_ms = link_now_ms() - commit_started_ms;
    if (commit_elapsed_ms < DUAL_END_CHANNEL_SWITCH_DELAY_MS)
        vTaskDelay(pdMS_TO_TICKS(DUAL_END_CHANNEL_SWITCH_DELAY_MS - commit_elapsed_ms));
    ESP_RETURN_ON_ERROR(channel_switch_runtime(target_channel, "coordinated"),
                        TAG, "switch coordinated channel");
    s_channel_switch_count++;
    s_channel_peer_seen_after_switch = false;
    s_channel_confirm_generation = 0;
    /* 新信道必须在超时前收到对端确认，否则自动回到信道 6。 */
    s_channel_fallback_due_ms = link_now_ms() + DUAL_END_CHANNEL_CONFIRM_TIMEOUT_MS;
    channel_status_update(DUAL_END_CHANNEL_CONFIRMING, target_channel, generation, ESP_OK, true);
    esp_err_t confirm_ret = send_channel_control(DUAL_END_CHANNEL_CMD_SWITCH_CONFIRM,
                                                 target_channel,
                                                 flags,
                                                 generation,
                                                 0,
                                                 channel_score);
    if (confirm_ret != ESP_OK) {
        ESP_LOGW(TAG, "initial channel confirm failed: %s",
                 esp_err_to_name(confirm_ret));
    }
    return ESP_OK;
}

/* 先通知 Scepter 暂停音频，再扫描；若无需切换则用 SCAN_END 恢复音频。 */
static esp_err_t channel_rescan_and_negotiate(bool avoid_current)
{
    uint32_t generation = dual_end_channel_next_token(s_channel_generation);
    s_channel_generation = generation;
    channel_status_update(DUAL_END_CHANNEL_SCANNING, s_current_channel, generation, ESP_OK, true);
    esp_err_t begin_ret = send_channel_control(DUAL_END_CHANNEL_CMD_SCAN_BEGIN,
                               s_current_channel,
                               avoid_current ? DUAL_END_CHANNEL_FLAG_INTERFERENCE : 0,
                               generation,
                               0,
                               0);
    if (begin_ret != ESP_OK) return begin_ret;
    vTaskDelay(pdMS_TO_TICKS(DUAL_END_CHANNEL_TASK_POLL_MS));
    uint8_t best_channel = s_current_channel;
    int16_t best_score = 0;
    esp_err_t ret = channel_scan_best(avoid_current, &best_channel, &best_score);
    if (ret != ESP_OK || best_channel == s_current_channel) {
        (void)send_channel_control(DUAL_END_CHANNEL_CMD_SCAN_END,
                                   s_current_channel,
                                   (avoid_current ? DUAL_END_CHANNEL_FLAG_INTERFERENCE : 0) |
                                       (ret != ESP_OK ? DUAL_END_CHANNEL_FLAG_FAILED : 0),
                                   generation,
                                   0,
                                   best_score);
        channel_status_update(ret == ESP_OK ? DUAL_END_CHANNEL_SUCCEEDED : DUAL_END_CHANNEL_FAILED,
                              s_current_channel, generation, ret, false);
        return ret;
    }
    return channel_negotiate_switch(best_channel,
                                    best_score,
                                    avoid_current ? DUAL_END_CHANNEL_FLAG_INTERFERENCE : 0,
                                    generation);
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
        ESP_LOGI(TAG,
            "STABILITY_LINK schema=1 role=dongle part=0 parts=4 uptime_ms=%" PRIu64 " boot_id=%08" PRIx32 " reset_reason=%d rate_mbps=5.5 connected=%d rssi=%d boot_ready=%c",
            log_uptime_ms, log_boot_id, log_reset_reason_0, log_connected_0, log_rssi_0, log_boot_ready);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "STABILITY_LINK schema=1 role=dongle part=1 parts=4 uptime_ms=%" PRIu64 " boot_id=%08" PRIx32 " channel=%u suspended=%d disconnects=%" PRIu64,
            log_uptime_ms, log_boot_id, log_channel_0, log_suspended_0, log_disconnects_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "STABILITY_LINK schema=1 role=dongle part=2 parts=4 uptime_ms=%" PRIu64 " boot_id=%08" PRIx32 " disconnected_ms=%" PRIu64 " max_disconnect_ms=%" PRIu64,
            log_uptime_ms, log_boot_id, log_disconnected_ms_0, log_max_disconnect_ms_0);
        vTaskDelay(1);
        ESP_LOGI(TAG,
            "STABILITY_LINK schema=1 role=dongle part=3 parts=4 uptime_ms=%" PRIu64 " boot_id=%08" PRIx32 " hop=%" PRIu32 " fallback=%" PRIu32 " heap_free=%" PRIu32 " heap_min=%" PRIu32,
            log_uptime_ms, log_boot_id, log_hop_0, log_fallback_0, log_heap_free_0, log_heap_min_0);
    }
}

static void link_status_task(void *arg)
{
    (void)arg;
    TickType_t last_hello_tick = 0;
    TickType_t last_diag_tick = 0;
    uint32_t reported_rx_queue_drops = 0;
    uint32_t reported_rx_invalid_len = 0;

    while (true) {
        radio_publish_status();
        if (!radio_ready()) {
            vTaskDelay(pdMS_TO_TICKS(DUAL_END_CHANNEL_TASK_POLL_MS));
            continue;
        }
        uint32_t now_ms = link_now_ms();
        int64_t now_us = esp_timer_get_time();
        const link_state_snapshot_t link_state = link_state_refresh_from_last_seen(now_us);
        const bool connected = link_state.connected;
        const int rssi = link_state.rssi;
        stability_log_link(connected, rssi, false);
        notify_status_if_changed(connected, rssi);

        if (connected) {
            if (s_channel_connected_since_ms == 0) {
                s_channel_connected_since_ms = now_ms;
            }
            s_channel_disconnect_since_ms = 0;
        } else {
            s_channel_connected_since_ms = 0;
            if (s_current_channel != DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL &&
                s_channel_disconnect_since_ms == 0) {
                s_channel_disconnect_since_ms = now_ms;
            }
        }

        if (!s_channel_configuration_dirty && s_channel_peer_seen_after_switch && s_channel_fallback_due_ms != 0) {
            esp_err_t save_ret = channel_store_save(s_current_channel);
            if (save_ret != ESP_OK) {
                ESP_LOGW(TAG, "save confirmed channel failed: %s",
                         esp_err_to_name(save_ret));
            }
            ESP_LOGI(TAG, "channel confirmed generation=%" PRIu32 " ch=%u",
                     s_channel_generation, s_current_channel);
            s_channel_peer_seen_after_switch = false;
            s_channel_fallback_due_ms = 0;
            channel_status_update(DUAL_END_CHANNEL_SUCCEEDED, s_current_channel,
                                  s_channel_generation, ESP_OK, false);
        }

        /* 非会合信道连续失联或切换未确认时，双方无需额外通信即可回到信道 6。 */
        bool disconnected_fallback =
            s_current_channel != DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL &&
            s_channel_disconnect_since_ms != 0 &&
            now_ms - s_channel_disconnect_since_ms >= DUAL_END_CHANNEL_CONFIRM_TIMEOUT_MS;
        if (channel_time_reached(now_ms, s_channel_fallback_due_ms) ||
            disconnected_fallback) {
            uint8_t failed_channel = s_current_channel;
            s_channel_fallback_due_ms = 0;
            s_channel_disconnect_since_ms = 0;
            channel_status_update(DUAL_END_CHANNEL_FAILED, s_channel_desired,
                                  s_channel_generation, ESP_ERR_TIMEOUT, false);
            if (channel_switch_runtime(DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL,
                                       "confirmation-timeout") == ESP_OK) {
                s_channel_fallback_count++;
                (void)channel_store_save(DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL);
                s_channel_rescan_pending = true;
                s_channel_force_avoid = true;
                ESP_LOGW(TAG, "channel %u failed; rendezvous fallback ch=%u",
                         failed_channel, DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL);
            }
        }

        /* 启动后先等原信道链路稳定，再发送扫描得到的切换提议。 */
        bool settled = connected && s_channel_connected_since_ms != 0 &&
                       now_ms - s_channel_connected_since_ms >=
                           DUAL_END_CHANNEL_STARTUP_SETTLE_MS;
        bool channel_action = false;
        uint8_t user_target;
        bool user_avoid;
        bool ota_quiet = !s_ota_active &&
                         (s_channel_last_ota_activity_ms == 0 ||
                          now_ms - s_channel_last_ota_activity_ms >= DUAL_END_CHANNEL_OTA_QUIET_MS);
        if (s_channel_fallback_due_ms == 0 && ota_quiet &&
            channel_take_user_request(&user_target, &user_avoid)) {
            s_channel_startup_pending = false;
            channel_action = true;
            esp_err_t user_ret = !connected ? ESP_ERR_INVALID_STATE :
                user_target ? channel_negotiate_switch(user_target, 0,
                                  DUAL_END_CHANNEL_FLAG_MANUAL_TARGET, 0) :
                              channel_rescan_and_negotiate(user_avoid);
            if (user_ret != ESP_OK)
                channel_status_update(DUAL_END_CHANNEL_FAILED, user_target,
                                      s_channel_generation, user_ret, false);
        }
        if (!channel_action && ota_quiet && s_channel_fallback_due_ms == 0 && settled &&
            s_channel_startup_pending && channel_claim_automatic()) {
            s_channel_startup_pending = false;
            /* Leave the working channel only when the link is weak: a hop on a
         * healthy link costs an audio outage and can fail confirmation. */
        const bool startup_link_weak = (rssi == 0) || (rssi < -65);
        if (s_channel_startup_best != s_current_channel && startup_link_weak) {
                esp_err_t hop_ret = channel_negotiate_switch(s_channel_startup_best,
                                                             s_channel_startup_score,
                                                             0, 0);
                if (hop_ret != ESP_OK) {
                    channel_status_update(DUAL_END_CHANNEL_FAILED, s_channel_startup_best,
                                      s_channel_generation, hop_ret, false);
                ESP_LOGW(TAG, "startup channel negotiation failed: %s",
                             esp_err_to_name(hop_ret));
                }
                channel_action = true;
            } else {
                channel_status_update(DUAL_END_CHANNEL_SUCCEEDED, s_current_channel,
                                      s_channel_generation, ESP_OK, false);
                if (s_channel_startup_best != s_current_channel) {
                ESP_LOGI(TAG, "startup hop skipped: ch=%u rssi=%d best=%u score=%d",
                         s_current_channel, rssi, s_channel_startup_best,
                         s_channel_startup_score);
            }
            ESP_LOGI(TAG, "startup channel scan keeps ch=%u score=%d",
                         s_current_channel, s_channel_startup_score);
            }
        }

        /* OTA 活动后保留静默期，并限制扫描频率，避免升级过程中跳频或反复扫描。 */

        bool scan_cooldown_elapsed = s_channel_last_scan_ms == 0 ||
                                     now_ms - s_channel_last_scan_ms >=
                                         DUAL_END_CHANNEL_SCAN_COOLDOWN_MS;
        if (!channel_action && s_channel_fallback_due_ms == 0 && settled && s_channel_rescan_pending &&
            ota_quiet && scan_cooldown_elapsed && channel_claim_automatic()) {
            bool avoid_current = s_channel_force_avoid;
            s_channel_rescan_pending = false;
            s_channel_force_avoid = false;
            esp_err_t scan_ret = channel_rescan_and_negotiate(avoid_current);
            if (scan_ret != ESP_OK) {
                channel_status_update(DUAL_END_CHANNEL_FAILED, s_channel_desired,
                                      s_channel_generation, scan_ret, false);
                ESP_LOGW(TAG, "channel rescan/negotiation ended: %s",
                         esp_err_to_name(scan_ret));
            }
        }

        TickType_t now_tick = xTaskGetTickCount();
        uint32_t hello_interval_ms = connected ?
                                     DUAL_END_HELLO_CONNECTED_INTERVAL_MS :
                                     DUAL_END_HELLO_DISCONNECTED_INTERVAL_MS;
        if (last_hello_tick == 0 ||
            (now_tick - last_hello_tick) >= pdMS_TO_TICKS(hello_interval_ms)) {
            last_hello_tick = now_tick;
            esp_err_t ret = send_hello();
            if (ret == ESP_ERR_ESPNOW_NO_MEM || ret == ESP_ERR_NO_MEM) {
                s_hello_no_mem_count++;
                if ((s_hello_no_mem_count % 20) == 1) {
                    ESP_LOGW(TAG,
                             "hello send skipped: ESP-NOW queue full count=%" PRIu32,
                             s_hello_no_mem_count);
                }
            } else if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "hello send failed: %s", esp_err_to_name(ret));
            }
        }

        uint32_t rx_queue_drops = s_rx_queue_drop_count;
        uint32_t rx_invalid_len = s_rx_invalid_len_count;
        if (rx_queue_drops != reported_rx_queue_drops ||
            rx_invalid_len != reported_rx_invalid_len) {
            ESP_LOGW(TAG,
                     "ESP-NOW RX callback drops: queue_full=%" PRIu32
                     " invalid_len=%" PRIu32,
                     rx_queue_drops,
                     rx_invalid_len);
            reported_rx_queue_drops = rx_queue_drops;
            reported_rx_invalid_len = rx_invalid_len;
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
            uint32_t histogram[DUAL_END_TX_LATENCY_BUCKET_COUNT];
            bool callback_debt;
            portENTER_CRITICAL(&s_tx_stats_lock);
            accepted = s_tx_accepted_count;
            callbacks = s_tx_callback_count;
            soft_timeout = s_tx_callback_soft_timeout_count;
            late = s_tx_callback_late_count;
            recover = s_tx_callback_hard_recovery_count;
            recovery_fail = s_tx_callback_recovery_fail_count;
            orphan = s_tx_callback_orphan_count;
            immediate_fail = s_tx_immediate_fail_count;
            latency_last = s_tx_callback_latency_last_us;
            latency_max = s_tx_callback_latency_max_us;
            callback_debt = s_tx_callback_armed;
            memcpy(histogram, s_tx_callback_latency_buckets, sizeof(histogram));
            uint32_t success_over[3], failure_over[3];
            memcpy(success_over, s_tx_success_over_us, sizeof(success_over));
            memcpy(failure_over, s_tx_failure_over_us, sizeof(failure_over));
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
                const uint32_t log_ch_0 = s_current_channel;
                const uint32_t log_scan_0 = s_channel_scan_count;
                const uint32_t log_hop_0 = s_channel_switch_count;
                const uint32_t log_fallback_0 = s_channel_fallback_count;
                const uint32_t log_rescan_req_0 = s_channel_rescan_request_count;
                const uint32_t log_cb_ok_gt20_60_100_0 = success_over[0];
                const uint32_t log_cb_ok_gt20_60_100_1 = success_over[1];
                const uint32_t log_cb_ok_gt20_60_100_2 = success_over[2];
                const uint32_t log_cb_fail_gt20_60_100_0 = failure_over[0];
                const uint32_t log_cb_fail_gt20_60_100_1 = failure_over[1];
                const uint32_t log_cb_fail_gt20_60_100_2 = failure_over[2];
                ESP_LOGI(TAG,
                    "tx_diag schema=1 part=0 parts=5 uptime_ms=%" PRIu64 " accepted=%" PRIu32 " cb=%" PRIu32 " debt=%d soft=%" PRIu32 " late=%" PRIu32 " recover=%" PRIu32 "/%" PRIu32,
                    log_uptime_ms, log_accepted_0, log_cb_0, log_debt_0, log_soft_0, log_late_0, log_recover_0, log_recover_1);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "tx_diag schema=1 part=1 parts=5 uptime_ms=%" PRIu64 " orphan=%" PRIu32 " immediate=%" PRIu32 " latency_us=%" PRIu32 "/%" PRIu32,
                    log_uptime_ms, log_orphan_0, log_immediate_0, log_latency_us_0, log_latency_us_1);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "tx_diag schema=1 part=2 parts=5 uptime_ms=%" PRIu64 " hist=[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "] ch=%u rate=5.5M scan=%" PRIu32,
                    log_uptime_ms, log_hist_0, log_hist_1, log_hist_2, log_hist_3, log_hist_4, log_hist_5, log_hist_6, log_ch_0, log_scan_0);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "tx_diag schema=1 part=3 parts=5 uptime_ms=%" PRIu64 " hop=%" PRIu32 " fallback=%" PRIu32 " rescan_req=%" PRIu32 " cb_ok_gt20_60_100=[%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]",
                    log_uptime_ms, log_hop_0, log_fallback_0, log_rescan_req_0, log_cb_ok_gt20_60_100_0, log_cb_ok_gt20_60_100_1, log_cb_ok_gt20_60_100_2);
                vTaskDelay(1);
                ESP_LOGI(TAG,
                    "tx_diag schema=1 part=4 parts=5 uptime_ms=%" PRIu64 " cb_fail_gt20_60_100=[%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]",
                    log_uptime_ms, log_cb_fail_gt20_60_100_0, log_cb_fail_gt20_60_100_1, log_cb_fail_gt20_60_100_2);
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
    if (s_tx_callback_armed && s_radio.state == AUDIO_RADIO_READY) {
        int64_t elapsed_us = now_us - s_tx_inflight_start_us;
        uint32_t latency_us = elapsed_us > UINT32_MAX ? UINT32_MAX :
                              elapsed_us > 0 ? (uint32_t)elapsed_us : 0;
        s_tx_callback_armed = false;
        s_tx_callback_count++;
        s_last_send_status = (int)status;
        if (s_tx_soft_timeout_active) {
            s_tx_callback_late_count++;
        }
        s_tx_callback_latency_last_us = latency_us;
        if (latency_us > s_tx_callback_latency_max_us) {
            s_tx_callback_latency_max_us = latency_us;
        }
        s_tx_callback_latency_buckets[tx_latency_bucket(latency_us)]++;
        const uint32_t limits_us[3] = {20000, 60000, 100000};
        uint32_t *over = status == ESP_NOW_SEND_SUCCESS ?
            s_tx_success_over_us : s_tx_failure_over_us;
        for (unsigned i = 0; i < 3; ++i) {
            if (latency_us > limits_us[i]) {
                ++over[i];
            }
        }
        if (status == ESP_NOW_SEND_SUCCESS && s_radio.first_cb_pending) {
            s_radio.first_cb_pending = false;
            s_radio.first_cb_last_us = radio_duration_us(now_us - s_radio_recovery_started_us);
            if (s_radio.first_cb_last_us > s_radio.first_cb_max_us) s_radio.first_cb_max_us = s_radio.first_cb_last_us;
            ++s_radio_revision;
        }
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
    if (!radio_ready()) return;
    if (!recv_info || !recv_info->src_addr || !data || len <= 0 ||
        len > DUAL_END_ESPNOW_MAX_DATA_BYTES || s_rx_queue == NULL) {
        s_rx_invalid_len_count++;
        return;
    }
    if (recv_info->rx_ctrl && recv_info->rx_ctrl->channel != s_current_channel) return;
    if (!is_peer_addr(recv_info->src_addr)) {
        return;
    }
    bool is_audio = len >= (int)sizeof(dual_end_packet_header_t) &&
        ((const dual_end_packet_header_t *)data)->message_type == SCEPTER_MSG_AUDIO_FRAME;
    if (is_audio && (len != DUAL_END_AUDIO_PACKET_BYTES || !s_rx_input_queue)) {
        s_rx_invalid_len_count++;
        return;
    }
    dual_end_rx_packet_t packet = {
        .len = (uint16_t)len,
        .rssi = (int8_t)(recv_info->rx_ctrl ? recv_info->rx_ctrl->rssi : 0),
        .received_us = esp_timer_get_time(),
    };
    memcpy(packet.src_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(packet.data, data, (size_t)len);

    if (is_audio) {
        /* Bounded storage: reject the newest arrival if all protected slots
         * are occupied. The recovery window detects and requests the missing
         * sequence; older audio/HID state remains in strict arrival order. */
        if (xQueueSend(s_rx_input_queue, &packet, 0) != pdTRUE) {
            s_rx_queue_drop_count++;
        } else if (s_rx_task) {
            xTaskNotifyGive(s_rx_task);
        }
        return;
    }
    BaseType_t queued = xQueueSend(s_rx_queue, &packet, 0);
    if (queued != pdTRUE) {
        dual_end_rx_packet_t stale;
        /* 过载时丢弃队列中最旧的数据包并保留最新到达包，以维持实时延迟。 */
        if (xQueueReceive(s_rx_queue, &stale, 0) == pdTRUE) {
            queued = xQueueSend(s_rx_queue, &packet, 0);
        }
        s_rx_queue_drop_count++;
    }
    if (queued == pdTRUE && s_rx_task) xTaskNotifyGive(s_rx_task);
}

esp_err_t dual_end_espnow_link_init(scepter_role_t role)
{
    memset(&s_radio, 0, sizeof(s_radio));
    s_radio.state = AUDIO_RADIO_FAILED_OFF;
    s_radio_initializing = true;
    s_radio_forced_pending = false;
    s_radio_last_recovery_us = s_radio_recovery_started_us = 0;
    s_radio_revision = 0;
    s_role = role;
    s_audio_feedback_drop_count = 0;
    s_audio_feedback_fail_count = 0;
    s_session_feedback_attempt_count = 0;
    s_session_feedback_mac_ok_count = 0;
    s_session_feedback_deferred_count = 0;
    s_session_feedback_fail_count = 0;
    s_session_feedback_last_log_us = 0;
    s_reported_connected = false;
    s_reported_rssi = 0;
    portENTER_CRITICAL(&s_peer_state_lock);
    s_connected = false;
    s_last_seen_us = 0;
    s_last_rssi = 0;
    portEXIT_CRITICAL(&s_peer_state_lock);
    s_hello_sequence = 0;
    s_hello_no_mem_count = 0;
    s_rx_queue_drop_count = 0;
    s_rx_invalid_len_count = 0;
    s_last_send_status = -1;
    s_tx_accepted_count = 0;
    s_tx_immediate_fail_count = 0;
    s_tx_callback_count = 0;
    s_tx_callback_soft_timeout_count = 0;
    s_tx_callback_late_count = 0;
    s_tx_callback_hard_recovery_count = 0;
    s_tx_callback_recovery_fail_count = 0;
    s_tx_callback_orphan_count = 0;
    s_tx_callback_latency_last_us = 0;
    s_tx_callback_latency_max_us = 0;
    memset(s_tx_callback_latency_buckets, 0, sizeof(s_tx_callback_latency_buckets));
    memset(s_tx_success_over_us, 0, sizeof(s_tx_success_over_us));
    memset(s_tx_failure_over_us, 0, sizeof(s_tx_failure_over_us));
    s_tx_callback_armed = false;
    s_tx_soft_timeout_active = false;
    s_tx_inflight_start_us = 0;
    s_current_channel = DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL;
    s_channel_startup_best = DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL;
    s_channel_startup_score = 0;
    s_channel_startup_pending = true;
    s_channel_rescan_pending = false;
    s_channel_force_avoid = false;
    s_channel_peer_seen_after_switch = false;
    s_channel_accept_generation = 0;
    s_channel_confirm_generation = 0;
    s_channel_desired = DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL;
    s_channel_generation = esp_random();
    s_channel_fallback_due_ms = 0;
    s_channel_disconnect_since_ms = 0;
    s_channel_connected_since_ms = 0;
    s_channel_last_scan_ms = 0;
    s_channel_last_ota_activity_ms = 0;
    s_ota_active = false;
    s_channel_scan_count = 0;
    s_channel_switch_count = 0;
    s_channel_fallback_count = 0;
    s_channel_rescan_request_count = 0;
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
    s_channel_startup_best = s_current_channel;
    s_channel_desired = s_current_channel;
    s_peer_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_peer_mutex, ESP_ERR_NO_MEM, TAG, "create peer mutex");
    s_tx_callback_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_tx_callback_sem, ESP_ERR_NO_MEM, TAG, "create TX callback semaphore");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "init netif");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "create event loop");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "init wifi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start wifi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "wifi ps none");
    ESP_RETURN_ON_ERROR(esp_wifi_set_bandwidth(WIFI_IF_STA, DUAL_END_WIFI_BANDWIDTH_20MHZ),
                        TAG, "wifi 20MHz");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(s_current_channel, WIFI_SECOND_CHAN_NONE), TAG, "wifi channel");
    ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(SCEPTER_ESPNOW_TX_POWER_QDBM), TAG, "wifi tx power");

    /* 启动扫描只记录最优候选；必须等 ESP-NOW 原信道重连后再进行正式协商。 */
    esp_err_t scan_ret = channel_scan_best(false,
                                           &s_channel_startup_best,
                                           &s_channel_startup_score);
    if (scan_ret != ESP_OK) {
        s_channel_startup_best = s_current_channel;
        s_channel_startup_score = 0;
        ESP_LOGW(TAG, "startup channel scan failed: %s", esp_err_to_name(scan_ret));
    }

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
    ESP_RETURN_ON_ERROR(rx_worker_init(), TAG, "start ESP-NOW RX worker");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(on_espnow_send), TAG, "send callback");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(on_espnow_recv), TAG, "recv callback");

    uint8_t stored_peer[ESP_NOW_ETH_ALEN] = {0};
    bool have_stored_peer = pairing_store_load(stored_peer);
    if (have_stored_peer) {
        ESP_RETURN_ON_ERROR(configure_peer(stored_peer), TAG, "configure stored peer");
        peer_state_set(stored_peer, true, true);
    }

    s_radio_initializing = false;
    radio_set_state(AUDIO_RADIO_READY);

    ESP_LOGI(TAG,
             "device_identity role=%s sta_mac=%02x:%02x:%02x:%02x:%02x:%02x",
             scepter_role_name(role),
             sta_mac[0], sta_mac[1], sta_mac[2],
             sta_mac[3], sta_mac[4], sta_mac[5]);

    if (have_stored_peer) {
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
            const uint32_t log_scan_best_0 = s_channel_startup_best;
            const int32_t log_tx_power_qdbm_0 = SCEPTER_ESPNOW_TX_POWER_QDBM;
            const int32_t log_hello_0 = DUAL_END_HELLO_DISCONNECTED_INTERVAL_MS;
            const int32_t log_link_timeout_0 = DUAL_END_LINK_TIMEOUT_MS;
            const int32_t log_cb_soft_hard_0 = DUAL_END_TX_CALLBACK_SOFT_TIMEOUT_MS;
            const int32_t log_cb_soft_hard_1 = DUAL_END_TX_CALLBACK_HARD_TIMEOUT_MS;
            ESP_LOGI(TAG,
                "ESP-NOW link ready for %.8s: part=0 parts=3 uptime_ms=%" PRIu64 " peer=%02x:%02x:%02x:%02x:%02x:%02x source=nvs ch=%u rendezvous=%u",
                log_role, log_uptime_ms, log_peer_0, log_peer_1, log_peer_2, log_peer_3, log_peer_4, log_peer_5, log_ch_0, log_rendezvous_0);
            vTaskDelay(1);
            ESP_LOGI(TAG,
                "ESP-NOW link ready for %.8s: part=1 parts=3 uptime_ms=%" PRIu64 " scan_best=%u auto_channel=1 rate=5.5M tx_power_qdbm=%d hello=%dms",
                log_role, log_uptime_ms, log_scan_best_0, log_tx_power_qdbm_0, log_hello_0);
            vTaskDelay(1);
            ESP_LOGI(TAG,
                "ESP-NOW link ready for %.8s: part=2 parts=3 uptime_ms=%" PRIu64 " link_timeout=%dms cb_soft/hard=%d/%dms",
                log_role, log_uptime_ms, log_link_timeout_0, log_cb_soft_hard_0, log_cb_soft_hard_1);
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

esp_err_t dual_end_espnow_link_send_audio(const void *payload, int len)
{
    return dual_end_espnow_link_send_control(payload, len);
}

static esp_err_t queue_control_request(const uint8_t *expected_peer,
    const void *payload, int len, int64_t deadline_us)
{
    if (!payload || len <= 0 || len > DUAL_END_ESPNOW_MAX_DATA_BYTES) return ESP_ERR_INVALID_ARG;
    if (!s_control_requests || !radio_ready()) return ESP_ERR_INVALID_STATE;
    dual_end_control_request_t request = {.payload = payload, .len = len, .result = ESP_ERR_INVALID_STATE};
    request.deadline_us = deadline_us;
    request.check_peer = expected_peer != NULL;
    if (expected_peer) memcpy(request.expected_peer, expected_peer, ESP_NOW_ETH_ALEN);
    request.done = xSemaphoreCreateBinary();
    if (!request.done) return ESP_ERR_NO_MEM;
    dual_end_control_request_t *queued = &request;
    if (xQueueSend(s_control_requests, &queued, 0) != pdTRUE) {
        vSemaphoreDelete(request.done);
        return ESP_ERR_NO_MEM;
    }
    /* The caller retains payload ownership until TX completion or rejection. */
    (void)xSemaphoreTake(request.done, portMAX_DELAY);
    vSemaphoreDelete(request.done);
    return request.result;
}

esp_err_t dual_end_espnow_link_send_control(const void *payload, int len)
{
    return queue_control_request(NULL, payload, len, 0);
}

esp_err_t dual_end_espnow_link_send_control_to_until(const uint8_t peer[6],
    const void *payload, int len, int64_t deadline_us)
{
    if (!peer || deadline_us <= 0) return ESP_ERR_INVALID_ARG;
    return queue_control_request(peer, payload, len, deadline_us);
}

esp_err_t dual_end_espnow_link_register_control_callbacks(
    dual_end_espnow_recv_cb_t response_cb,
    dual_end_espnow_control_service_cb_t service_cb, void *ctx)
{
    s_control_cb_ctx = ctx;
    s_control_response_cb = response_cb;
    s_control_service_cb = service_cb;
    return ESP_OK;
}

static void tx_worker_service_sync(void)
{
    dual_end_control_request_t *request = NULL;
    if (xQueuePeek(s_control_requests, &request, 0) != pdTRUE) return;
    esp_err_t ret = send_control_peer_deadline(
        request->check_peer ? request->expected_peer : NULL,
        request->payload, request->len, request->deadline_us, false);
    if (ret == ESP_ERR_NO_MEM) return;
    (void)xQueueReceive(s_control_requests, &request, 0);
    request->result = ret;
    xSemaphoreGive(request->done);
}

static void audio_feedback_send_nack(void)
{
    if (!radio_ready()) return;
    /* Reserve result capacity before taking the peer mutex. This worker is
     * its sole producer and never waits for RX while retaining the radio. */
    const UBaseType_t result_spaces = uxQueueSpacesAvailable(s_audio_nack_results);
    if (result_spaces == 0) {
        return;
    }
    /* Leave the latest snapshot available for RX refresh while another sender
     * owns the radio. Never dequeue a request merely to wait behind a long tail. */
    if (xSemaphoreTake(s_peer_mutex, 0) != pdTRUE) {
        return;
    }
    if (!radio_ready()) {
        xSemaphoreGive(s_peer_mutex);
        return;
    }
    dual_end_nack_snapshot_t snapshot;
    portENTER_CRITICAL(&s_audio_nack_lock);
    bool claimed = dual_end_nack_mailbox_claim(&s_audio_nack_mailbox, &snapshot,
                                              (unsigned)result_spaces);
    portEXIT_CRITICAL(&s_audio_nack_lock);
    if (!claimed) {
        xSemaphoreGive(s_peer_mutex);
        return;
    }

    dual_end_nack_result_t result = {
        .token = snapshot.token,
        .submitted_us = esp_timer_get_time(),
        .kind = DUAL_END_NACK_REJECTED,
    };
    bool armed = false;
    if (result.submitted_us >= snapshot.latest_send_us) {
        result.kind = DUAL_END_NACK_EXPIRED;
    } else if (s_peer_valid) {
        int64_t remaining_ms = (snapshot.latest_send_us + DUAL_END_AUDIO_MIN_REPAIR_ROUNDTRIP_US -
                                result.submitted_us) / 1000LL;
        if (remaining_ms > 255) remaining_ms = 255;
        snapshot.packet.remaining_ms = (uint8_t)remaining_ms;
        (void)xSemaphoreTake(s_tx_callback_sem, 0);
        tx_arm_callback();
        armed = true;
        esp_err_t ret = esp_now_send(s_peer_mac, (const uint8_t *)&snapshot.packet,
                                    sizeof(snapshot.packet));
        if (ret == ESP_OK) {
            tx_note_send_accepted();
            result.kind = DUAL_END_NACK_ACCEPTED;
        } else {
            tx_note_immediate_failure();
        }
    }
    /* Report acceptance before the potentially 500 ms callback wait. Capacity
     * is guaranteed by the sole-producer preflight above; RX only removes. */
    BaseType_t posted = xQueueSend(s_audio_nack_results, &result, 0);
    configASSERT(posted == pdTRUE);
    (void)posted;
    if (result.kind == DUAL_END_NACK_ACCEPTED) {
        if (tx_wait_for_callback(sizeof(snapshot.packet)) != ESP_OK ||
            s_last_send_status != ESP_NOW_SEND_SUCCESS) {
            ++s_audio_feedback_fail_count;
        }
    }
    if (armed) {
        tx_finish_inflight();
    }
    xSemaphoreGive(s_peer_mutex);
}

static esp_err_t send_control_peer_deadline(const uint8_t *expected_peer,
    const void *payload, int len, int64_t deadline_us, bool allow_pending_nack)
{
    if (!radio_ready()) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_FALSE(payload && len > 0 && len <= DUAL_END_ESPNOW_MAX_DATA_BYTES,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid control packet");
    ESP_RETURN_ON_FALSE(s_peer_mutex, ESP_ERR_INVALID_STATE, TAG, "link is not initialized");
    if (deadline_us && esp_timer_get_time() >= deadline_us) {
        return ESP_ERR_TIMEOUT;
    }
    TickType_t wait_ticks = deadline_us ? 0 : portMAX_DELAY;
    if (xSemaphoreTake(s_peer_mutex, wait_ticks) != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }
    if (!radio_ready()) {
        xSemaphoreGive(s_peer_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (deadline_us && esp_timer_get_time() >= deadline_us) {
        xSemaphoreGive(s_peer_mutex);
        return ESP_ERR_TIMEOUT;
    }
    portENTER_CRITICAL(&s_audio_nack_lock);
    bool nack_pending = s_audio_nack_mailbox.pending;
    portEXIT_CRITICAL(&s_audio_nack_lock);
    if (nack_pending && !allow_pending_nack) {
        xSemaphoreGive(s_peer_mutex);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t ret = ESP_ERR_NOT_FOUND;
    if (s_peer_valid && (!expected_peer || memcmp(expected_peer, s_peer_mac, ESP_NOW_ETH_ALEN) == 0)) {
        /* 同一时刻只允许一个已接受的发送请求在途。软超时只记录延迟；
         * 在回调被消费或达到硬期限并重置 ESP-NOW 实例前，互斥锁保持占用。 */
        (void)xSemaphoreTake(s_tx_callback_sem, 0);
        tx_arm_callback();
        ret = esp_now_send(s_peer_mac, (const uint8_t *)payload, len);
        if (ret != ESP_OK) {
            tx_note_immediate_failure();
        } else {
            tx_note_send_accepted();
            ret = tx_wait_for_callback(len);
            if (ret == ESP_OK && s_last_send_status != ESP_NOW_SEND_SUCCESS) {
                ret = ESP_FAIL;
            }
        }
        tx_finish_inflight();
    }
    xSemaphoreGive(s_peer_mutex);
    return ret;
}

static esp_err_t send_control_deadline(const void *payload, int len, int64_t deadline_us,
                                       bool allow_pending_nack)
{
    return send_control_peer_deadline(NULL, payload, len, deadline_us,
                                      allow_pending_nack);
}

esp_err_t dual_end_espnow_link_send_ota(const void *payload, int len)
{
    s_channel_last_ota_activity_ms = link_now_ms();
    return dual_end_espnow_link_send_control(payload, len);
}

void dual_end_espnow_link_set_ota_active(bool active)
{
    s_ota_active = active;
    s_channel_last_ota_activity_ms = link_now_ms();
}

esp_err_t dual_end_espnow_link_set_tx_power(int8_t tx_power_qdbm)
{
    return esp_wifi_set_max_tx_power(tx_power_qdbm);
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
    return ESP_OK;
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

    if (!radio_lock_configuration()) return ESP_ERR_INVALID_STATE;
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

    const link_state_snapshot_t link_state = link_state_invalidate();
    notify_status_if_changed(false, link_state.rssi);
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
    if (!radio_lock_configuration()) return ESP_ERR_INVALID_STATE;
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

    (void)link_state_invalidate();
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
    (void)link_state_invalidate();
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

    if (!radio_lock_configuration()) return ESP_ERR_INVALID_STATE;
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
    if (!radio_lock_configuration()) return ESP_ERR_INVALID_STATE;
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
    const link_state_snapshot_t link_state = link_state_invalidate();
    xSemaphoreGive(s_peer_mutex);
    notify_status_if_changed(false, link_state.rssi);
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_pair_confirm(uint32_t transaction_id)
{
    ESP_RETURN_ON_FALSE(s_peer_mutex && transaction_id != 0,
                        ESP_ERR_INVALID_ARG, TAG, "invalid pairing transaction");
    if (!radio_lock_configuration()) return ESP_ERR_INVALID_STATE;
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
    const link_state_snapshot_t link_state = link_state_snapshot();
    if (!link_state.connected) {
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
    if (!radio_lock_configuration()) return ESP_ERR_INVALID_STATE;
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
    const link_state_snapshot_t link_state = link_state_snapshot();
    notify_status_if_changed(false, link_state.rssi);
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
    if (!radio_lock_configuration()) return ESP_ERR_INVALID_STATE;
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

esp_err_t dual_end_espnow_link_register_recv_cb(dual_end_espnow_recv_cb_t cb, void *ctx)
{
    s_recv_cb = cb;
    s_recv_ctx = ctx;
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_register_poll_cb(dual_end_espnow_poll_cb_t cb, void *ctx)
{
    s_poll_ctx = ctx;
    s_poll_cb = cb;
    return ESP_OK;
}

int64_t dual_end_espnow_link_rx_received_us(void)
{
    return s_rx_received_us;
}

esp_err_t dual_end_espnow_link_queue_audio_feedback(const void *payload, int len)
{
    return dual_end_espnow_link_queue_audio_feedback_until(payload, len, INT64_MAX);
}

esp_err_t dual_end_espnow_link_queue_control(const void *payload, int len)
{
    if (!radio_ready()) return ESP_ERR_INVALID_STATE;
    if (!payload || !s_control_queue || len <= 0 || len > (int)sizeof(((dual_end_audio_feedback_t *)0)->data)) {
        return ESP_ERR_INVALID_ARG;
    }
    dual_end_audio_feedback_t item = {.len = len, .deadline_us = esp_timer_get_time() + 1000000LL};
    memcpy(item.data, payload, (size_t)len);
    return xQueueSend(s_control_queue, &item, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t dual_end_espnow_link_publish_audio_nack(const dual_end_nack_snapshot_t *snapshot)
{
    if (!radio_ready()) return ESP_ERR_INVALID_STATE;
    if (!snapshot || !snapshot->token || !s_audio_nack_results ||
        !snapshot->packet.missing_mask) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_audio_nack_lock);
    bool full = s_audio_nack_mailbox.pending &&
        s_audio_nack_mailbox.latest.token != snapshot->token;
    (void)dual_end_nack_mailbox_publish(&s_audio_nack_mailbox, snapshot);
    portEXIT_CRITICAL(&s_audio_nack_lock);
    return full ? ESP_ERR_NO_MEM : ESP_OK;
}

bool dual_end_espnow_link_cancel_audio_nack(uint32_t token)
{
    portENTER_CRITICAL(&s_audio_nack_lock);
    bool cancelled = dual_end_nack_mailbox_cancel(&s_audio_nack_mailbox, token);
    portEXIT_CRITICAL(&s_audio_nack_lock);
    return cancelled;
}

bool dual_end_espnow_link_take_nack_result(dual_end_nack_result_t *result)
{
    return result && s_audio_nack_results &&
        xQueueReceive(s_audio_nack_results, result, 0) == pdTRUE;
}

esp_err_t dual_end_espnow_link_queue_audio_feedback_until(const void *payload, int len,
                                                         int64_t latest_send_us)
{
    if (!radio_ready()) return ESP_ERR_INVALID_STATE;
    if (!payload || !s_audio_feedback_queue || len != sizeof(dual_end_audio_session_packet_t)) {
        return ESP_ERR_INVALID_ARG;
    }
    const dual_end_packet_header_t *header = payload;
    /* NACK must use the tokenized mailbox so recovery/cancellation and actual
     * driver acceptance cannot be bypassed by a stale FIFO copy. */
    if (header->message_type != SCEPTER_MSG_AUDIO_SESSION) {
        return ESP_ERR_INVALID_ARG;
    }
    dual_end_audio_feedback_t feedback = {
        .len = len,
        .deadline_us = esp_timer_get_time() + DUAL_END_AUDIO_SESSION_RETRY_US,
    };
    if (latest_send_us < feedback.deadline_us) {
        feedback.deadline_us = latest_send_us;
    }
    if (esp_timer_get_time() >= feedback.deadline_us) {
        return ESP_ERR_TIMEOUT;
    }
    memcpy(feedback.data, payload, (size_t)len);
    BaseType_t result = xQueueSend(s_audio_feedback_queue, &feedback, 0);
    if (result != pdTRUE) {
        ++s_audio_feedback_drop_count;
        if ((s_audio_feedback_drop_count % 32U) == 1U) {
            ESP_LOGW(TAG, "audio feedback queue drops=%" PRIu32 " send_fail=%" PRIu32,
                     s_audio_feedback_drop_count, s_audio_feedback_fail_count);
        }
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_register_status_cb(dual_end_espnow_status_cb_t cb, void *ctx)
{
    s_status_cb = cb;
    s_status_ctx = ctx;
    if (s_status_cb) {
        const link_state_snapshot_t link_state = link_state_snapshot();
        s_status_cb(link_state.connected, link_state.rssi, s_status_ctx);
    }
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_register_hello_cb(dual_end_espnow_hello_cb_t cb,
                                                void *ctx)
{
    s_hello_cb = cb;
    s_hello_ctx = ctx;
    return ESP_OK;
}

esp_err_t dual_end_espnow_link_send_hello(void)
{
    return send_hello();
}

bool dual_end_espnow_link_is_connected(void)
{
    const link_state_snapshot_t link_state = link_state_snapshot();
    return link_state.connected;
}
