#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "dual_end_link_config.h"
#include "dual_end_secure_pairing.h"
#include "audio_feedback_mailbox.h"

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
typedef void (*dual_end_espnow_hello_cb_t)(void *ctx);
typedef void (*dual_end_espnow_poll_cb_t)(void *ctx);
typedef void (*dual_end_espnow_control_service_cb_t)(bool peer_window, void *ctx);

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

/* A cache-safe link snapshot for host diagnostics. Values are counters or
 * already-owned state; this API never sends, scans, reconnects, or waits. */
typedef struct {
    bool ready;
    bool connected;
    bool channel_pending;
    int8_t rssi;
    uint8_t current_channel;
    uint8_t channel_state;
    uint32_t channel_generation;
    uint32_t rx_queue_drops;
    uint32_t rx_invalid_lengths;
    uint32_t tx_accepted;
    uint32_t tx_callbacks;
    uint32_t tx_immediate_failures;
    uint32_t tx_soft_timeouts;
    uint32_t tx_late_callbacks;
    uint32_t tx_hard_recoveries;
    uint32_t tx_recovery_failures;
    uint32_t rx_stack_min_bytes;
    uint32_t tx_stack_min_bytes;
    uint32_t link_stack_min_bytes;
} dual_end_espnow_link_diagnostics_t;

/* Accepted means queued, not completed. Both APIs preserve coordinated peer
 * negotiation; no caller changes Wi-Fi directly. Query status until pending=0. */
esp_err_t dual_end_espnow_link_request_channel(uint8_t channel);
esp_err_t dual_end_espnow_link_request_channel_rescan(bool avoid_current);
esp_err_t dual_end_espnow_link_get_channel_status(dual_end_channel_status_t *status);
esp_err_t dual_end_espnow_link_get_diagnostics(dual_end_espnow_link_diagnostics_t *out);

esp_err_t dual_end_espnow_link_init(scepter_role_t role);
void dual_end_espnow_link_set_ota_active(bool active);
/* One-shot request only. Recovery runs on the TX worker; acceptance is not
 * completion. FAILED_OFF requires a reboot after an unsuccessful recovery. */
esp_err_t dual_end_espnow_link_request_radio_recovery(void);
/* Called only after original audio format/CRC/current-session validation. */
void dual_end_espnow_link_note_audio_received(int64_t received_us);
esp_err_t dual_end_espnow_link_register_recv_cb(dual_end_espnow_recv_cb_t cb, void *ctx);
/* Poll and receive callbacks run on the same RX task; callbacks must not block. */
esp_err_t dual_end_espnow_link_register_poll_cb(dual_end_espnow_poll_cb_t cb, void *ctx);
int64_t dual_end_espnow_link_rx_received_us(void);
/* Session feedback only. NACK uses the refreshable admission mailbox below. */
esp_err_t dual_end_espnow_link_queue_audio_feedback(const void *payload, int len);
esp_err_t dual_end_espnow_link_queue_audio_feedback_until(const void *payload, int len,
                                                         int64_t latest_send_us);
/* RX publishes/coalesces; TX returns admission immediately before callback wait.
 * Cancellation cannot retract a packet already claimed for driver submission. */
esp_err_t dual_end_espnow_link_publish_audio_nack(const dual_end_nack_snapshot_t *snapshot);
bool dual_end_espnow_link_cancel_audio_nack(uint32_t token);
bool dual_end_espnow_link_take_nack_result(dual_end_nack_result_t *result);
esp_err_t dual_end_espnow_link_register_status_cb(dual_end_espnow_status_cb_t cb, void *ctx);
esp_err_t dual_end_espnow_link_register_hello_cb(dual_end_espnow_hello_cb_t cb,
                                                void *ctx);
bool dual_end_espnow_link_is_connected(void);
esp_err_t dual_end_espnow_link_send_hello(void);
esp_err_t dual_end_espnow_link_send_audio(const void *payload, int len);
esp_err_t dual_end_espnow_link_send_control(const void *payload, int len);
/* Copies ordinary small controls/HELLO to TX; never waits for MAC completion. */
esp_err_t dual_end_espnow_link_queue_control(const void *payload, int len);
/* The same TX owner rechecks the peer and deadline before admission. */
esp_err_t dual_end_espnow_link_send_control_to_until(const uint8_t peer[6],
    const void *payload, int len, int64_t deadline_us);
esp_err_t dual_end_espnow_link_register_control_callbacks(
    dual_end_espnow_recv_cb_t response_cb,
    dual_end_espnow_control_service_cb_t service_cb, void *ctx);
esp_err_t dual_end_espnow_link_send_ota(const void *payload, int len);
esp_err_t dual_end_espnow_link_set_tx_power(int8_t tx_power_qdbm);
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

#ifdef __cplusplus
}
#endif
