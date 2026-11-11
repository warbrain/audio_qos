#include "dual_end_pairing.h"
#include "dual_end_runtime.h"

#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "driver/usb_serial_jtag.h"
#include "dual_end_espnow_link.h"
#include "dual_end_ota.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/platform_util.h"
#include "esp_app_desc.h"
#include "sdkconfig.h"

static const char *TAG = "pairing_service";

#define PAIRING_PROTOCOL_VERSION 2u
#define PAIRING_COMMAND_MAX_LEN 320
#define PAIRING_RESPONSE_MAX_LEN 512
#define PAIRING_USB_BUFFER_BYTES 1024
#define PAIRING_TASK_STACK_BYTES 4096
#define PAIRING_TASK_PRIORITY 5

#if CONFIG_SECURE_FLASH_ENC_ENABLED
#define PAIRING_REQUIRE_RELEASE_SECURITY 1
#define PAIRING_KEY_STORAGE_NAME "encrypted_flash"
#else
#define PAIRING_REQUIRE_RELEASE_SECURITY 0
#define PAIRING_KEY_STORAGE_NAME "plaintext_flash"
#endif

static scepter_role_t s_role;
static uint8_t s_local_mac[6];
static char s_local_serial[32];
static bool s_initialized;
static bool s_link_confirmation_pending;
static portMUX_TYPE s_confirmation_lock = portMUX_INITIALIZER_UNLOCKED;

static void set_link_confirmation_pending(bool pending)
{
    portENTER_CRITICAL(&s_confirmation_lock);
    s_link_confirmation_pending = pending;
    portEXIT_CRITICAL(&s_confirmation_lock);
}

bool dual_end_pairing_take_link_confirmation_pending(void)
{
    portENTER_CRITICAL(&s_confirmation_lock);
    bool pending = s_link_confirmation_pending;
    s_link_confirmation_pending = false;
    portEXIT_CRITICAL(&s_confirmation_lock);
    return pending;
}

static void trim_line(char *line)
{
    size_t len = strlen(line);
    while (len > 0 && isspace((unsigned char)line[len - 1])) {
        line[--len] = '\0';
    }
    char *start = line;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }
    if (start != line) {
        memmove(line, start, strlen(start) + 1);
    }
}

static int hex_nibble(char value)
{
    if (value >= '0' && value <= '9') {
        return value - '0';
    }
    value = (char)tolower((unsigned char)value);
    return value >= 'a' && value <= 'f' ? value - 'a' + 10 : -1;
}

static bool parse_mac(const char *text, uint8_t mac[6])
{
    if (!text || !mac || strlen(text) != 17) {
        return false;
    }
    for (size_t i = 0; i < 6; i++) {
        size_t offset = i * 3;
        if ((i > 0 && text[offset - 1] != ':') ||
            hex_nibble(text[offset]) < 0 ||
            hex_nibble(text[offset + 1]) < 0) {
            return false;
        }
        mac[i] = (uint8_t)((hex_nibble(text[offset]) << 4) |
                           hex_nibble(text[offset + 1]));
    }
    return true;
}

static bool parse_hex_exact(const char *text, uint8_t *out, size_t out_len)
{
    if (!text || !out || strlen(text) != out_len * 2) {
        return false;
    }
    for (size_t i = 0; i < out_len; i++) {
        int high = hex_nibble(text[i * 2]);
        int low = hex_nibble(text[i * 2 + 1]);
        if (high < 0 || low < 0) {
            return false;
        }
        out[i] = (uint8_t)((high << 4) | low);
    }
    return true;
}

static bool parse_u32_nonzero(const char *text, uint32_t *value)
{
    if (!text || !*text || !value || *text == '-') {
        return false;
    }
    errno = 0;
    char *end = NULL;
    unsigned long parsed = strtoul(text, &end, 10);
    if (errno != 0 || !end || *end != '\0' || parsed == 0 ||
        parsed > UINT32_MAX) {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

static void format_hex(const uint8_t *value, size_t length,
                       char *out, size_t out_size)
{
    if (!out || out_size < length * 2 + 1) {
        return;
    }
    for (size_t i = 0; i < length; i++) {
        snprintf(out + i * 2, out_size - i * 2, "%02x", value[i]);
    }
}

static const char *security_name(dual_end_pairing_security_t security)
{
    switch (security) {
    case DUAL_END_PAIRING_SECURITY_ENCRYPTED:
        return "ccmp";
    case DUAL_END_PAIRING_SECURITY_LEGACY_PLAIN:
        return "legacy_plain";
    default:
        return "none";
    }
}

static const char *pending_name(dual_end_pairing_pending_state_t state)
{
    switch (state) {
    case DUAL_END_PAIRING_PENDING_PREPARED:
        return "prepared";
    case DUAL_END_PAIRING_PENDING_COMMITTED:
        return "committed";
    default:
        return "none";
    }
}

static bool pairing_state_ready(bool check_maintenance_lock,
                                char *reason,
                                size_t reason_size)
{
    (void)check_maintenance_lock;
    const char *failure = NULL;
    dual_end_ota_status_t ota = dual_end_ota_get_status();
    if (ota.active || dual_end_ota_relay_is_active()) {
        failure = "ota_active";
    } else if (dual_end_ota_boot_confirmation_error() != ESP_OK) {
        failure = "abnormal_recovery";
    }
    snprintf(reason, reason_size, "%s", failure ? failure : "ok");
    return failure == NULL;
}

static esp_err_t pairing_mutation_begin(bool require_release_ready,
                                        char *reason,
                                        size_t reason_size)
{
    (void)require_release_ready;
    if (!pairing_state_ready(false, reason, reason_size)) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static void format_info(char *response, size_t response_size)
{
    dual_end_pairing_status_t status = {0};
    (void)dual_end_espnow_link_get_pairing_status(&status);

    char peer[24] = "none";
    char key_id[DUAL_END_PAIRING_KEY_ID_BYTES * 2 + 1] = "none";
    if (status.paired) {
        snprintf(peer, sizeof(peer), "%02x:%02x:%02x:%02x:%02x:%02x",
                 status.peer_mac[0], status.peer_mac[1], status.peer_mac[2],
                 status.peer_mac[3], status.peer_mac[4], status.peer_mac[5]);
    }
    if (status.security == DUAL_END_PAIRING_SECURITY_ENCRYPTED) {
        format_hex(status.key_id, sizeof(status.key_id), key_id,
                   sizeof(key_id));
    }

    char ready_reason[40];
    bool ready = pairing_state_ready(true, ready_reason,
                                     sizeof(ready_reason));
    snprintf(response, response_size,
             "OK PAIR INFO role=%s mac=%02x:%02x:%02x:%02x:%02x:%02x "
             "serial=%s pair_protocol=%u fw=%s paired=%d peer=%s source=%s "
             "encryption=%s key_storage=%s generation=%" PRIu32
             " key_id=%s link=%d "
             "pending=%s txn=%" PRIu32 " pair_ready=%d pair_reason=%s "
             "confirm_cap=ota_upgrade confirm_policy=auto_usb\n",
             scepter_role_name(s_role),
             s_local_mac[0], s_local_mac[1], s_local_mac[2],
             s_local_mac[3], s_local_mac[4], s_local_mac[5],
             s_local_serial, PAIRING_PROTOCOL_VERSION,
             (esp_app_get_description() ? esp_app_get_description()->version : "unknown"),
             status.paired ? 1 : 0,
             peer, status.from_nvs ? "nvs" : "none",
             security_name(status.security), PAIRING_KEY_STORAGE_NAME,
             status.generation, key_id,
             dual_end_espnow_link_is_connected() ? 1 : 0,
             pending_name(status.pending_state),
             status.pending_transaction_id, ready ? 1 : 0, ready_reason);
}

static bool parse_transaction_command(const char *command,
                                      const char *expected,
                                      uint32_t *transaction_id)
{
    char operation[16] = {0};
    char transaction_text[16] = {0};
    char extra[2] = {0};
    int fields = sscanf(command, "PAIR %15s %15s %1s",
                        operation, transaction_text, extra);
    return fields == 2 && strcasecmp(operation, expected) == 0 &&
           parse_u32_nonzero(transaction_text, transaction_id);
}

bool dual_end_pairing_handle_command(const char *line,
                                     char *response,
                                     size_t response_size)
{
    if (!line || !response || response_size == 0) {
        return false;
    }

    char command[PAIRING_COMMAND_MAX_LEN];
    size_t len = strnlen(line, sizeof(command));
    if (len >= sizeof(command)) {
        snprintf(response, response_size, "ERR PAIR reason=line_too_long\n");
        return true;
    }
    memcpy(command, line, len + 1);
    trim_line(command);

    if (strncasecmp(command, "PAIR", 4) != 0 ||
        (command[4] != '\0' && !isspace((unsigned char)command[4]))) {
        return false;
    }
    if (!s_initialized) {
        snprintf(response, response_size, "ERR PAIR reason=not_initialized\n");
        return true;
    }
    if (strcasecmp(command, "PAIR INFO") == 0) {
        format_info(response, response_size);
        return true;
    }
    if (strncasecmp(command, "PAIR SET", 8) == 0) {
        snprintf(response, response_size,
                 "ERR PAIR SET reason=secure_pairing_protocol_required\n");
        return true;
    }
    if (strcasecmp(command, "PAIR CLEAR") == 0) {
        char reason[40];
        esp_err_t ret = pairing_mutation_begin(false, reason, sizeof(reason));
        if (ret == ESP_OK) {
            ret = dual_end_espnow_link_clear_peer(true);
        }
        if (ret == ESP_OK) {
            set_link_confirmation_pending(false);
            snprintf(response, response_size,
                     "OK PAIR CLEAR role=%s persisted=0\n",
                     scepter_role_name(s_role));
        } else {
            snprintf(response, response_size,
                     "ERR PAIR CLEAR reason=%s error=%s\n",
                     reason, esp_err_to_name(ret));
        }
        return true;
    }

    char operation[16] = {0};
    char mac_text[24] = {0};
    char generation_text[16] = {0};
    char transaction_text[16] = {0};
    char lmk_text[DUAL_END_PAIRING_KEY_BYTES * 2 + 1] = {0};
    char pmk_text[DUAL_END_PAIRING_KEY_BYTES * 2 + 1] = {0};
    char key_id_text[DUAL_END_PAIRING_KEY_ID_BYTES * 2 + 1] = {0};
    char extra[2] = {0};
    int fields = sscanf(command,
                        "PAIR %15s %23s %15s %15s %32s %32s %16s %1s",
                        operation, mac_text, generation_text,
                        transaction_text, lmk_text, pmk_text,
                        key_id_text, extra);
    if (fields == 7 && strcasecmp(operation, "PREPARE") == 0) {
        uint8_t peer[6] = {0};
        uint8_t lmk[DUAL_END_PAIRING_KEY_BYTES] = {0};
        uint8_t pmk[DUAL_END_PAIRING_KEY_BYTES] = {0};
        uint8_t key_id[DUAL_END_PAIRING_KEY_ID_BYTES] = {0};
        uint32_t generation = 0;
        uint32_t transaction_id = 0;
        bool valid = parse_mac(mac_text, peer) &&
                     parse_u32_nonzero(generation_text, &generation) &&
                     parse_u32_nonzero(transaction_text, &transaction_id) &&
                     parse_hex_exact(lmk_text, lmk, sizeof(lmk)) &&
                     parse_hex_exact(pmk_text, pmk, sizeof(pmk)) &&
                     parse_hex_exact(key_id_text, key_id, sizeof(key_id));
        mbedtls_platform_zeroize(command, sizeof(command));
        mbedtls_platform_zeroize(lmk_text, sizeof(lmk_text));
        mbedtls_platform_zeroize(pmk_text, sizeof(pmk_text));
        if (!valid) {
            mbedtls_platform_zeroize(lmk, sizeof(lmk));
            mbedtls_platform_zeroize(pmk, sizeof(pmk));
            snprintf(response, response_size,
                     "ERR PAIR PREPARE reason=invalid_fields\n");
            return true;
        }

        char reason[40];
        esp_err_t ret = pairing_mutation_begin(true, reason, sizeof(reason));
        if (ret == ESP_OK) {
            ret = dual_end_espnow_link_pair_prepare(peer, lmk, pmk,
                                                    generation,
                                                    transaction_id,
                                                    key_id);
        }
        mbedtls_platform_zeroize(lmk, sizeof(lmk));
        mbedtls_platform_zeroize(pmk, sizeof(pmk));
        if (ret == ESP_OK) {
            char id_text[sizeof(key_id_text)];
            format_hex(key_id, sizeof(key_id), id_text, sizeof(id_text));
            snprintf(response, response_size,
                     "OK PAIR PREPARE role=%s txn=%" PRIu32
                     " generation=%" PRIu32 " key_id=%s\n",
                     scepter_role_name(s_role), transaction_id,
                     generation, id_text);
        } else {
            snprintf(response, response_size,
                     "ERR PAIR PREPARE reason=%s error=%s\n",
                     reason, esp_err_to_name(ret));
        }
        return true;
    }
    if (strcasecmp(operation, "PREPARE") == 0 ||
        strncasecmp(command, "PAIR PREPARE", 12) == 0) {
        /* Malformed PREPARE frames can still contain partial key material. */
        mbedtls_platform_zeroize(command, sizeof(command));
        mbedtls_platform_zeroize(lmk_text, sizeof(lmk_text));
        mbedtls_platform_zeroize(pmk_text, sizeof(pmk_text));
        mbedtls_platform_zeroize(key_id_text, sizeof(key_id_text));
        snprintf(response, response_size,
                 "ERR PAIR PREPARE reason=invalid_fields\n");
        return true;
    }

    uint32_t transaction_id = 0;
    const char *operation_name = NULL;
    enum { OP_NONE, OP_COMMIT, OP_CONFIRM, OP_ROLLBACK } transaction_op = OP_NONE;
    if (parse_transaction_command(command, "COMMIT", &transaction_id)) {
        transaction_op = OP_COMMIT;
        operation_name = "COMMIT";
    } else if (parse_transaction_command(command, "CONFIRM", &transaction_id)) {
        transaction_op = OP_CONFIRM;
        operation_name = "CONFIRM";
    } else if (parse_transaction_command(command, "ROLLBACK", &transaction_id)) {
        transaction_op = OP_ROLLBACK;
        operation_name = "ROLLBACK";
    }
    if (transaction_op != OP_NONE) {
        char reason[40];
        esp_err_t ret = pairing_mutation_begin(transaction_op == OP_COMMIT,
                                               reason, sizeof(reason));
        if (ret == ESP_OK) {
            if (transaction_op == OP_COMMIT) {
                ret = dual_end_espnow_link_pair_commit(transaction_id);
            } else if (transaction_op == OP_CONFIRM) {
                ret = dual_end_espnow_link_pair_confirm(transaction_id);
            } else {
                ret = dual_end_espnow_link_pair_rollback(transaction_id);
            }
        }
        if (ret == ESP_OK) {
            if (transaction_op == OP_COMMIT &&
                s_role == SCEPTER_ROLE_SCEPTER) {
                set_link_confirmation_pending(true);
            } else if (transaction_op == OP_ROLLBACK) {
                set_link_confirmation_pending(false);
            }
            snprintf(response, response_size,
                     "OK PAIR %s role=%s txn=%" PRIu32 "\n",
                     operation_name, scepter_role_name(s_role), transaction_id);
        } else {
            snprintf(response, response_size,
                     "ERR PAIR %s reason=%s error=%s\n",
                     operation_name, reason, esp_err_to_name(ret));
        }
        return true;
    }

    snprintf(response, response_size,
             "ERR PAIR reason=usage expected=\"PAIR INFO|PAIR PREPARE <mac> "
             "<generation> <txn> <lmk> <pmk> <key_id>|PAIR COMMIT <txn>|"
             "PAIR CONFIRM <txn>|PAIR ROLLBACK <txn>|PAIR CLEAR\"\n");
    return true;
}

static void native_write_all(const char *text)
{
    const uint8_t *cursor = (const uint8_t *)text;
    size_t remaining = strlen(text);
    while (remaining > 0) {
        int written = usb_serial_jtag_write_bytes(cursor, remaining,
                                                  pdMS_TO_TICKS(1000));
        if (written <= 0) {
            return;
        }
        cursor += written;
        remaining -= (size_t)written;
    }
}

static void pairing_usb_task(void *arg)
{
    (void)arg;
    uint8_t rx[256];
    char line[PAIRING_COMMAND_MAX_LEN];
    size_t line_len = 0;
    bool line_overflow = false;
    char response[PAIRING_RESPONSE_MAX_LEN];

    snprintf(response, sizeof(response),
             "OK PAIR READY role=%s pair_protocol=%u\n",
             scepter_role_name(s_role), PAIRING_PROTOCOL_VERSION);
    native_write_all(response);
    while (true) {
        int count = usb_serial_jtag_read_bytes(rx, sizeof(rx),
                                               pdMS_TO_TICKS(100));
        if (count <= 0) {
            continue;
        }
        for (int i = 0; i < count; i++) {
            char ch = (char)rx[i];
            if (ch == '\n') {
                if (line_overflow) {
                    native_write_all("ERR PAIR reason=line_too_long\n");
                } else {
                    line[line_len] = '\0';
                    if (dual_end_pairing_handle_command(line, response,
                                                        sizeof(response))) {
                        native_write_all(response);
                    }
                }
                mbedtls_platform_zeroize(line, sizeof(line));
                line_len = 0;
                line_overflow = false;
            } else if (ch != '\r' && !line_overflow) {
                if (line_len + 1 < sizeof(line)) {
                    line[line_len++] = ch;
                } else {
                    mbedtls_platform_zeroize(line, sizeof(line));
                    line_len = 0;
                    line_overflow = true;
                }
            }
        }
    }
}

esp_err_t dual_end_pairing_service_init(scepter_role_t role,
                                        bool start_native_usb)
{
    ESP_RETURN_ON_FALSE(role == SCEPTER_ROLE_SCEPTER ||
                        role == SCEPTER_ROLE_DONGLE,
                        ESP_ERR_INVALID_ARG, TAG, "invalid role");
    ESP_RETURN_ON_ERROR(esp_read_mac(s_local_mac, ESP_MAC_WIFI_STA),
                        TAG, "read local station mac");
    s_role = role;
    snprintf(s_local_serial, sizeof(s_local_serial),
             "%s-%02X%02X%02X%02X%02X%02X",
             role == SCEPTER_ROLE_SCEPTER ? "SCEPTER" : "DONGLE",
             s_local_mac[0], s_local_mac[1], s_local_mac[2],
             s_local_mac[3], s_local_mac[4], s_local_mac[5]);
    set_link_confirmation_pending(false);
    s_initialized = true;

    ESP_LOGI(TAG,
             "secure pairing service ready role=%s serial=%s protocol=%u transport=%s",
             scepter_role_name(role), s_local_serial,
             PAIRING_PROTOCOL_VERSION,
             start_native_usb ? "usb-serial-jtag" : "external-cdc");
    if (!start_native_usb) {
        return ESP_OK;
    }

    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = PAIRING_USB_BUFFER_BYTES,
        .rx_buffer_size = PAIRING_USB_BUFFER_BYTES,
    };
    ESP_RETURN_ON_ERROR(usb_serial_jtag_driver_install(&usb_config),
                        TAG, "install USB Serial/JTAG driver");
    BaseType_t task_ok = xTaskCreatePinnedToCore(pairing_usb_task, "pairing_usb",
                                     PAIRING_TASK_STACK_BYTES, NULL,
                                     PAIRING_TASK_PRIORITY, NULL, DUAL_END_COMM_TASK_CORE);
    return task_ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
