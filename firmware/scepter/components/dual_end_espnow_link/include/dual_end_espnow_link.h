#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "dual_end_link_config.h"
#include "dual_end_secure_pairing.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*dual_end_espnow_recv_cb_t)(const uint8_t *src_addr,
                                         const uint8_t *data,
                                         int len,
                                         int rssi,
                                         void *ctx);

typedef void (*dual_end_espnow_status_cb_t)(bool connected,
                                           int rssi,
                                           void *ctx);

typedef struct {
    uint32_t done;
    uint32_t ok;
    uint32_t fail;
    uint32_t consecutive_fail;
    uint32_t consecutive_fail_ms;
    uint32_t last_fail_streak;
    uint32_t last_fail_streak_ms;
    uint32_t max_fail_streak;
    uint32_t max_fail_streak_ms;
} dual_end_espnow_tx_type_stats_t;

typedef struct {
    uint32_t tx_done;
    uint32_t tx_ok;
    uint32_t tx_fail;
    uint32_t tx_accepted;
    uint32_t tx_immediate_fail;
    uint32_t tx_callback_soft_timeout;
    uint32_t tx_callback_late;
    uint32_t tx_callback_hard_recovery;
    uint32_t tx_callback_recovery_fail;
    uint32_t tx_callback_orphan;
    uint32_t tx_callback_latency_last_us;
    uint32_t tx_callback_latency_max_us;
    /* Cumulative >20/60/100 ms callback counts, independently by MAC status. */
    uint32_t tx_callback_success_over_us[3];
    uint32_t tx_callback_failure_over_us[3];
    uint32_t tx_queue_wait_max_us;
    uint32_t tx_audio_submit_age_max_us;
    uint32_t tx_audio_mac_success_after_deadline;
    uint32_t tx_audio_fast_retry_suppressed;
    uint32_t tx_queue_depth;
    uint32_t tx_queue_high_water;
    uint32_t tx_callback_debt;
    uint32_t tx_audio_submit;
    uint32_t tx_audio_submit_fail;
    uint32_t tx_audio_stale_drop;
    uint32_t tx_audio_final_ok;
    uint32_t tx_audio_final_fail;
    uint32_t tx_audio_retry_recovered;
    uint32_t tx_audio_repair_recovered;
    uint32_t tx_audio_deadline_drop;
    uint32_t tx_repair_ok;
    uint32_t tx_repair_fail;
    uint32_t tx_repair_deadline_drop;
    uint32_t tx_ota_submit;
    uint32_t tx_ota_submit_fail;
    uint32_t hello_no_mem;
    int last_send_status;
    bool connected;
    int last_rssi;
    dual_end_espnow_tx_type_stats_t audio;
    dual_end_espnow_tx_type_stats_t hello;
    dual_end_espnow_tx_type_stats_t ota;
} dual_end_espnow_link_stats_t;

typedef struct {
    uint32_t state, reason, generation, early, hard, forced, success, fail;
    uint32_t reset_last_us, reset_max_us;
    /* From recovery trigger, including rebuild time. first_cb is the first
     * original-audio MAC success; first_rx is valid peer feedback, not playout. */
    uint32_t first_cb_last_us, first_cb_max_us, first_cb_pending;
    uint32_t first_rx_last_us, first_rx_max_us, first_rx_pending;
    int32_t error;
    uint32_t congestion_late, congestion_expired;
} dual_end_radio_recovery_stats_t;

/* Queue one diagnostic recovery for the TX worker; never reset from USB/task
 * caller context. A second request, sleep or cooldown is rejected. */
esp_err_t dual_end_espnow_link_request_radio_recovery(void);
void dual_end_espnow_link_get_radio_recovery_stats(dual_end_radio_recovery_stats_t *stats);

typedef enum {
    DUAL_END_CHANNEL_IDLE = 0,
    DUAL_END_CHANNEL_QUEUED,
    DUAL_END_CHANNEL_SCANNING,
    DUAL_END_CHANNEL_PROPOSING,
    DUAL_END_CHANNEL_SWITCHING,
    DUAL_END_CHANNEL_CONFIRMING,
    DUAL_END_CHANNEL_SUCCEEDED,
    DUAL_END_CHANNEL_FAILED,
} dual_end_channel_state_t;

typedef struct {
    uint8_t current_channel, target_channel, state;
    bool pending;
    uint32_t generation, request_id, scan_count, switch_count, fallback_count;
    uint32_t rx_stack_min_bytes, tx_stack_min_bytes, link_stack_min_bytes;
    esp_err_t last_result;
} dual_end_channel_status_t;

/* Accepted means queued, not completed. Both APIs preserve coordinated peer
 * negotiation; no caller changes Wi-Fi directly. Query status until pending=0. */
esp_err_t dual_end_espnow_link_request_channel(uint8_t channel);
esp_err_t dual_end_espnow_link_request_channel_rescan(bool avoid_current);
esp_err_t dual_end_espnow_link_get_channel_status(dual_end_channel_status_t *status);

esp_err_t dual_end_espnow_link_init(scepter_role_t role);
esp_err_t dual_end_espnow_link_register_recv_cb(dual_end_espnow_recv_cb_t cb, void *ctx);
esp_err_t dual_end_espnow_link_register_status_cb(dual_end_espnow_status_cb_t cb, void *ctx);
bool dual_end_espnow_link_is_connected(void);
/* True only during the bounded coordinated channel scan/switch pause.  Audio
 * recovery uses the edge to retire one session and start one after resume. */
bool dual_end_espnow_link_is_audio_paused(void);
/* Dual-mic application compatibility: pausing audio and purging admitted media
 * is coordinated by the existing OTA activity callback. */
void dual_end_espnow_link_set_ota_active(bool active);
esp_err_t dual_end_espnow_link_set_tx_power(int8_t tx_power_qdbm);
/* Short HID/PTT packets retain their ordered FIFO and bounded retries. */
esp_err_t dual_end_espnow_link_send_input(const void *payload, int len);
esp_err_t dual_end_espnow_link_send_audio(const void *payload, int len);
esp_err_t dual_end_espnow_link_send_audio_at(const void *payload, int len, int64_t created_us);
esp_err_t dual_end_espnow_link_send_audio_control(const void *payload, int len);
/* deadline_us is the sender-local conservative decode cutoff, derived from
 * NACK receipt time and remaining_ms; created_us only identifies cached audio. */
esp_err_t dual_end_espnow_link_send_audio_repair(const void *payload, int len,
                                               int64_t created_us,
                                               uint32_t reservation_id,
                                               int64_t deadline_us);
void dual_end_espnow_link_discard_audio(void);
uint8_t dual_end_espnow_link_take_unsent_input_flags(void);
esp_err_t dual_end_espnow_link_prepare_ota(uint32_t timeout_ms);
esp_err_t dual_end_espnow_link_send_ota(const void *payload, int len);
/* Nonblocking application response; ESP_OK means queued, not delivered. */
esp_err_t dual_end_espnow_link_send_control(const void *payload, int len);
esp_err_t dual_end_espnow_link_set_peer(const uint8_t peer_mac[6], bool persist);
esp_err_t dual_end_espnow_link_clear_peer(bool erase_nvs);
esp_err_t dual_end_espnow_link_get_peer(uint8_t peer_mac[6], bool *from_nvs);
esp_err_t dual_end_espnow_link_pair_prepare(
    const uint8_t peer_mac[6],
    const uint8_t lmk[DUAL_END_PAIRING_KEY_BYTES],
    const uint8_t pmk[DUAL_END_PAIRING_KEY_BYTES],
    uint32_t generation,
    uint32_t transaction_id,
    const uint8_t key_id[DUAL_END_PAIRING_KEY_ID_BYTES]);
esp_err_t dual_end_espnow_link_pair_commit(uint32_t transaction_id);
esp_err_t dual_end_espnow_link_pair_confirm(uint32_t transaction_id);
esp_err_t dual_end_espnow_link_pair_rollback(uint32_t transaction_id);
esp_err_t dual_end_espnow_link_get_pairing_status(dual_end_pairing_status_t *status);
esp_err_t dual_end_espnow_link_suspend_radio(void);
esp_err_t dual_end_espnow_link_resume_radio(void);
bool dual_end_espnow_link_is_radio_suspended(void);
/* Runtime RF duty cycling preserves logical pairing and admitted input slots. */
void dual_end_espnow_link_set_idle_allowed(bool allowed, int64_t last_activity_us);
bool dual_end_espnow_link_is_idle_stopped(void);
/* Preserve RX for bounded tail repair; does not change logical sleep policy. */
void dual_end_espnow_link_hold_receive_until(int64_t deadline_us);
esp_err_t dual_end_espnow_link_recover_peer(const uint8_t *peer_mac);
esp_err_t dual_end_espnow_link_send_hello(void);
void dual_end_espnow_link_get_stats(dual_end_espnow_link_stats_t *out_stats);

#ifdef __cplusplus
}
#endif
