#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dual_end_link_config.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DUAL_END_OTA_TARGET_DONGLE = 1,
    DUAL_END_OTA_TARGET_SCEPTER = 2,
} dual_end_ota_target_t;

#define DUAL_END_OTA_MAGIC 0x43594f54u
#define DUAL_END_OTA_VERSION 2u
#define DUAL_END_OTA_MAX_PAYLOAD 180u

typedef enum {
    DUAL_END_OTA_MSG_BEGIN = 1,
    DUAL_END_OTA_MSG_DATA = 2,
    DUAL_END_OTA_MSG_END = 3,
    DUAL_END_OTA_MSG_ABORT = 4,
    DUAL_END_OTA_MSG_ERASE = 5,
    DUAL_END_OTA_MSG_BOOT = 6,
    DUAL_END_OTA_MSG_REBOOT = 7,
    DUAL_END_OTA_MSG_RESULT = 8,
    DUAL_END_OTA_MSG_ACK = 9,
} dual_end_ota_msg_type_t;

#define DUAL_END_OTA_ACK_VERSION 1u

typedef enum {
    DUAL_END_OTA_ACK_ACCEPTED = 1,
    DUAL_END_OTA_ACK_RETRYABLE = 2,
    DUAL_END_OTA_ACK_REJECTED = 3,
} dual_end_ota_ack_status_t;

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t acked_msg_type;
    uint8_t status;
    uint8_t duplicate;
    int32_t error_code;
    uint32_t received;
    uint32_t rolling_crc32;
    uint32_t payload_crc32;
} dual_end_ota_ack_payload_t;

#define DUAL_END_OTA_RESULT_VERSION 1u
#define DUAL_END_OTA_RESULT_APP_VERSION_LEN 32u
#define DUAL_END_OTA_RESULT_PARTITION_LEN 16u

typedef enum {
    DUAL_END_OTA_RESULT_INSTALL_CONFIRMED = 1,
    DUAL_END_OTA_RESULT_INSTALL_FAILED = 2,
} dual_end_ota_result_status_t;

typedef enum {
    DUAL_END_OTA_RESULT_STAGE_BEGIN = 1,
    DUAL_END_OTA_RESULT_STAGE_DATA = 2,
    DUAL_END_OTA_RESULT_STAGE_FINALIZE = 3,
    DUAL_END_OTA_RESULT_STAGE_BOOT_CONFIRM = 4,
    DUAL_END_OTA_RESULT_STAGE_RESOURCE_COMMIT = 5,
} dual_end_ota_result_stage_t;

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t status;
    uint8_t stage;
    uint8_t reserved;
    int32_t error_code;
    char app_version[DUAL_END_OTA_RESULT_APP_VERSION_LEN];
    char partition[DUAL_END_OTA_RESULT_PARTITION_LEN];
} dual_end_ota_result_payload_t;

typedef struct {
    uint32_t session_id;
    dual_end_ota_result_payload_t payload;
} dual_end_ota_result_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint8_t msg_type;
    uint8_t target;
    uint8_t reserved;
    uint32_t session_id;
    uint32_t sequence;
    uint32_t offset;
    uint32_t image_size;
    uint32_t image_crc32;
    uint16_t payload_len;
    uint16_t header_crc16;
    uint8_t payload[DUAL_END_OTA_MAX_PAYLOAD];
} dual_end_ota_packet_t;

typedef struct {
    dual_end_ota_target_t target;
    uint32_t session_id;
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t bytes_written;
    uint32_t packets;
    bool active;
} dual_end_ota_status_t;

typedef void (*dual_end_ota_activity_cb_t)(bool active, void *ctx);

esp_err_t dual_end_ota_service_init(scepter_role_t role);
esp_err_t dual_end_ota_register_activity_cb(dual_end_ota_activity_cb_t cb, void *ctx);
esp_err_t dual_end_ota_log_boot_partition(void);
/* Call after all required peripherals/tasks and the radio have initialized. */
esp_err_t dual_end_ota_confirm_running_image(void);
esp_err_t dual_end_ota_boot_confirmation_error(void);
bool dual_end_ota_relay_is_active(void);
esp_err_t dual_end_ota_send_scepter_abort(uint32_t session_id);
esp_err_t dual_end_ota_prepare_scepter_result(uint32_t session_id);
void dual_end_ota_cancel_scepter_result_wait(void);
esp_err_t dual_end_ota_wait_scepter_result(uint32_t timeout_ms, dual_end_ota_result_t *result);
void dual_end_ota_get_mac_string(char *buf, size_t len);
dual_end_ota_status_t dual_end_ota_get_status(void);

uint32_t dual_end_ota_crc32_update(uint32_t crc, const uint8_t *data, size_t len);
uint16_t dual_end_ota_crc16(const uint8_t *data, size_t len);
void dual_end_ota_packet_set_crc(dual_end_ota_packet_t *packet);
bool dual_end_ota_packet_valid(const dual_end_ota_packet_t *packet, size_t len);

esp_err_t dual_end_ota_self_begin(dual_end_ota_target_t target,
                                  uint32_t session_id,
                                  uint32_t image_size,
                                  uint32_t image_crc32);
esp_err_t dual_end_ota_self_begin_partition(dual_end_ota_target_t target,
                                            uint32_t session_id,
                                            uint32_t image_size,
                                            uint32_t image_crc32,
                                            const char *partition_label);
esp_err_t dual_end_ota_self_write(const uint8_t *data, size_t len);
esp_err_t dual_end_ota_self_finish(bool reboot_after_finish);
void dual_end_ota_self_abort(void);
esp_err_t dual_end_ota_erase_partition(const char *partition_label);
esp_err_t dual_end_ota_set_boot_partition_label(const char *partition_label);
bool dual_end_ota_partition_is_running(const char *partition_label);
const char *dual_end_ota_running_partition_label(void);
const char *dual_end_ota_image_key_from_version(const char *version);

esp_err_t dual_end_ota_send_scepter_begin(uint32_t image_size,
                                          uint32_t image_crc32,
                                          uint32_t session_id,
                                          const char *partition_label);
esp_err_t dual_end_ota_send_scepter_data(const uint8_t *data,
                                         size_t len,
                                         uint32_t offset,
                                         uint32_t sequence,
                                         uint32_t image_size,
                                         uint32_t image_crc32,
                                         uint32_t session_id);
esp_err_t dual_end_ota_send_scepter_end(uint32_t image_size,
                                        uint32_t image_crc32,
                                        uint32_t session_id,
                                        uint32_t sequence);
esp_err_t dual_end_ota_send_scepter_erase(const char *partition_label);
esp_err_t dual_end_ota_send_scepter_boot(const char *partition_label);
esp_err_t dual_end_ota_send_scepter_reboot(void);

bool dual_end_ota_handle_scepter_espnow_packet(const uint8_t *data, int len);

#ifdef __cplusplus
}
#endif
