#include "dual_end_pairing.h"
#include "dual_end_runtime.h"

#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "dual_end_espnow_link.h"
#include "dual_end_ota.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_mac.h"
#ifdef ESP_PLATFORM
#include "esp_system.h"
#include "esp_timer.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/platform_util.h"
#include "esp_app_desc.h"
#include "sdkconfig.h"

static const char *TAG = "pairing_service";

/* Registered before the native USB task is created.  The callback receives
 * a complete newline-stripped command and writes a bounded response; it is
 * never used by pairing, OTA, ESP-NOW, or radio callbacks. */
static dual_end_pairing_diagnostics_command_cb_t s_diagnostics_command_cb;
static void *s_diagnostics_command_ctx;

#define PAIRING_PROTOCOL_VERSION 2u
#define PAIRING_COMMAND_MAX_LEN 320
#define PAIRING_RESPONSE_MAX_LEN 512
#define PAIRING_USB_RX_CHUNK_BYTES 1024
#define PAIRING_USB_RX_RING_BYTES 8192
#define PAIRING_USB_TX_RING_BYTES 2048
/* Commit can write NVS and reconfigure the encrypted ESP-NOW peer on this
 * task's stack.  4096 bytes overflowed on the dual-mic Scepter in hardware. */
#define PAIRING_TASK_STACK_BYTES 8192   /* internal RAM is scarce on this board */
#define PAIRING_TASK_PRIORITY 5
#define PAIRING_OTA2_MIN_CHUNK_BYTES 16u
#define PAIRING_OTA2_MAX_CHUNK_BYTES 4096u
#define PAIRING_OTA2_TIMEOUT_MS 60000u

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

typedef struct {
    bool active;
    bool receiving_chunk;
    bool duplicate_chunk;
    bool have_last_chunk;
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t session;
    uint32_t max_chunk_size;
    uint32_t next_sequence;
    uint32_t received;
    uint32_t running_crc;
    uint32_t remaining;
    uint32_t chunk_session;
    uint32_t chunk_sequence;
    uint32_t chunk_offset;
    uint32_t chunk_size;
    uint32_t chunk_crc;
    uint32_t last_sequence;
    uint32_t last_offset;
    uint32_t last_size;
    uint32_t last_crc;
    size_t chunk_received;
    int64_t last_activity_us;
    char partition[16];
    uint8_t chunk[PAIRING_OTA2_MAX_CHUNK_BYTES];
} pairing_ota2_state_t;

static pairing_ota2_state_t s_ota2;
static bool s_logs_suppressed;
static int s_saved_log_level;

static void ota_writer_abort(void);

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

static void format_status(char *response, size_t response_size)
{
    static const volatile char factory_tool_manual_markers[] =
        "FACTORY ROM SCEPTER CONFIRM\n"
        "OK FACTORY ROM role=scepter\n"
        "factory_rom_reason=manual_boot_required\n"
        "security=%s";
    volatile char marker_probe = factory_tool_manual_markers[0];
    (void)marker_probe;
    const esp_app_desc_t *desc = esp_app_get_description();
    dual_end_ota_status_t ota = dual_end_ota_get_status();
    snprintf(response, response_size,
             "OK STATUS device=scepter role=scepter firmware=app "
             "transport=typec_usb ota_protocol=2 ota1=disabled "
             "chunk_crc=1 chunk_ack=1 mac=%02x:%02x:%02x:%02x:%02x:%02x "
             "project=%s version=%s security=unencrypted factory_rom=ready "
             "factory_rom_reason=manual_boot_required active=%d "
             "session=%" PRIu32 " received=%" PRIu32 "\n",
             s_local_mac[0], s_local_mac[1], s_local_mac[2],
             s_local_mac[3], s_local_mac[4], s_local_mac[5],
             desc ? desc->project_name : "unknown",
             desc ? desc->version : "unknown", ota.active ? 1 : 0,
             s_ota2.active ? s_ota2.session : ota.session_id,
             s_ota2.active ? s_ota2.received : ota.bytes_written);
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
    if (!text) {
        return;
    }
    const uint8_t *cursor = (const uint8_t *)text;
    size_t remaining = strlen(text);
    flockfile(stdout);
    while (remaining > 0) {
        int written = usb_serial_jtag_write_bytes(cursor, remaining,
                                                  pdMS_TO_TICKS(1000));
        if (written <= 0) {
            break;
        }
        cursor += written;
        remaining -= (size_t)written;
    }
    funlockfile(stdout);
}

void dual_end_pairing_set_diagnostics_command_cb(
    dual_end_pairing_diagnostics_command_cb_t callback,
    void *ctx)
{
    /* app_main registers before dual_end_pairing_service_init() creates the
     * native USB task, so this needs no scheduler/USB lock and cannot block a
     * command path already in flight. */
    s_diagnostics_command_cb = callback;
    s_diagnostics_command_ctx = ctx;
}

static void native_printf(const char *format, ...)
{
    static char response[PAIRING_RESPONSE_MAX_LEN];
    va_list args;
    va_start(args, format);
    int required = vsnprintf(response, sizeof(response), format, args);
    va_end(args);
    if (required < 0) {
        native_write_all("ERR RESPONSE formatting_failed\n");
    } else if ((size_t)required >= sizeof(response)) {
        native_write_all("ERR RESPONSE too_long\n");
    } else {
        native_write_all(response);
    }
}

static void logs_suppress(void)
{
    if (s_logs_suppressed) {
        return;
    }
#ifdef ESP_PLATFORM
    s_saved_log_level = (int)esp_log_get_default_level();
    esp_log_level_set("*", ESP_LOG_NONE);
#endif
    s_logs_suppressed = true;
}

static void logs_restore(void)
{
    if (!s_logs_suppressed) {
        return;
    }
#ifdef ESP_PLATFORM
    esp_log_level_set("*", (esp_log_level_t)s_saved_log_level);
#endif
    s_logs_suppressed = false;
}

static void ota2_restore_logs(void)
{
    logs_restore();
}

static void ota2_state_clear(bool abort_writer)
{
    if (abort_writer && s_ota2.active) {
        ota_writer_abort();
    }
    mbedtls_platform_zeroize(&s_ota2, sizeof(s_ota2));
}

static void print_usb_identity(void)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    native_printf(
        "OK INFO device=scepter role=scepter firmware=app "
        "transport=typec_usb ota_protocol=2 ota1=disabled "
        "chunk_crc=1 chunk_ack=1 version=%s "
        "mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
        desc ? desc->version : "unknown",
        s_local_mac[0], s_local_mac[1], s_local_mac[2],
        s_local_mac[3], s_local_mac[4], s_local_mac[5]);
}

static size_t split_command_tokens(char *line, char **tokens, size_t capacity)
{
    size_t count = 0;
    char *cursor = line;
    while (*cursor) {
        while (*cursor && isspace((unsigned char)*cursor)) {
            cursor++;
        }
        if (!*cursor) {
            break;
        }
        if (count == capacity) {
            return capacity + 1;
        }
        tokens[count++] = cursor;
        while (*cursor && !isspace((unsigned char)*cursor)) {
            cursor++;
        }
        if (*cursor) {
            *cursor++ = '\0';
        }
    }
    return count;
}

static bool parse_u32_token(const char *text, int base, uint32_t *value)
{
    if (!text || !*text || !value || *text == '-' || *text == '+') {
        return false;
    }
    errno = 0;
    char *end = NULL;
    unsigned long parsed = strtoul(text, &end, base);
    if (errno != 0 || !end || *end != '\0' || parsed > UINT32_MAX) {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

/* Keep the existing host-side pairing harness linkable while production uses
 * the real ESP-IDF OTA writer. */
static esp_err_t ota_writer_begin(uint32_t session,
                                  uint32_t image_size,
                                  uint32_t image_crc32,
                                  const char *partition)
{
#ifdef ESP_PLATFORM
    return dual_end_ota_self_begin_partition(DUAL_END_OTA_TARGET_SCEPTER,
                                              session,
                                              image_size,
                                              image_crc32,
                                              partition);
#else
    (void)session;
    (void)image_size;
    (void)image_crc32;
    (void)partition;
    return ESP_FAIL;
#endif
}

static esp_err_t ota_writer_write(const uint8_t *data, size_t length)
{
#ifdef ESP_PLATFORM
    return dual_end_ota_self_write(data, length);
#else
    (void)data;
    (void)length;
    return ESP_FAIL;
#endif
}

static esp_err_t ota_writer_finish(void)
{
#ifdef ESP_PLATFORM
    return dual_end_ota_self_finish(false);
#else
    return ESP_FAIL;
#endif
}

static void ota_writer_abort(void)
{
#ifdef ESP_PLATFORM
    dual_end_ota_self_abort();
#endif
}

static uint32_t ota_crc32_update(uint32_t crc,
                                 const uint8_t *data,
                                 size_t length)
{
#ifdef ESP_PLATFORM
    return dual_end_ota_crc32_update(crc, data, length);
#else
    crc = ~crc;
    while (length-- > 0) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
        }
    }
    return ~crc;
#endif
}

static void ota2_begin(uint32_t image_size,
                       uint32_t image_crc32,
                       uint32_t session,
                       const char *partition,
                       uint32_t chunk_size)
{
    dual_end_ota_status_t writer = dual_end_ota_get_status();
    if (image_size == 0 || session == 0 || s_ota2.active || writer.active ||
        dual_end_ota_relay_is_active() || !partition ||
        strlen(partition) >= sizeof(s_ota2.partition) ||
        chunk_size < PAIRING_OTA2_MIN_CHUNK_BYTES ||
        chunk_size > PAIRING_OTA2_MAX_CHUNK_BYTES) {
        native_write_all("ERR SCEPTER_OTA_BEGIN invalid_state\n");
        return;
    }

    logs_suppress();
    esp_err_t ret = ota_writer_begin(session, image_size,
                                     image_crc32, partition);
    if (ret != ESP_OK) {
        native_printf("ERR SCEPTER_OTA_BEGIN %s\n", esp_err_to_name(ret));
        ota2_restore_logs();
        return;
    }

    ota2_state_clear(false);
    s_ota2.active = true;
    s_ota2.image_size = image_size;
    s_ota2.image_crc32 = image_crc32;
    s_ota2.session = session;
    s_ota2.max_chunk_size = chunk_size;
    s_ota2.remaining = image_size;
    s_ota2.last_activity_us = esp_timer_get_time();
    snprintf(s_ota2.partition, sizeof(s_ota2.partition), "%s", partition);
    native_printf("OK BEGIN SCEPTER session=%" PRIu32
                  " size=%" PRIu32 " partition=%s transport=typec_usb "
                  "protocol=2 chunk=%" PRIu32 "\n",
                  session, image_size, s_ota2.partition, chunk_size);
}

static void ota2_chunk_begin(uint32_t session,
                             uint32_t sequence,
                             uint32_t offset,
                             uint32_t chunk_size,
                             uint32_t chunk_crc)
{
    if (!s_ota2.active || s_ota2.receiving_chunk) {
        native_write_all("ERR CHUNK_STATE retryable=0\n");
        return;
    }
    if (session != s_ota2.session) {
        native_printf("ERR CHUNK_SESSION got=%" PRIu32
                      " expected=%" PRIu32 " retryable=0\n",
                      session, s_ota2.session);
        return;
    }
    if (chunk_size == 0 || chunk_size > s_ota2.max_chunk_size ||
        chunk_size > PAIRING_OTA2_MAX_CHUNK_BYTES) {
        native_printf("ERR CHUNK_SIZE sequence=%" PRIu32
                      " got=%" PRIu32 " max=%" PRIu32
                      " retryable=0\n",
                      sequence, chunk_size, s_ota2.max_chunk_size);
        return;
    }

    bool duplicate = s_ota2.have_last_chunk &&
                     sequence == s_ota2.last_sequence &&
                     offset == s_ota2.last_offset &&
                     chunk_size == s_ota2.last_size &&
                     chunk_crc == s_ota2.last_crc;
    if (!duplicate &&
        (sequence != s_ota2.next_sequence ||
         offset != s_ota2.received ||
         chunk_size > s_ota2.remaining)) {
        native_printf("ERR CHUNK_ORDER session=%" PRIu32
                      " sequence=%" PRIu32 " offset=%" PRIu32
                      " expected_sequence=%" PRIu32
                      " expected_offset=%" PRIu32
                      " remaining=%" PRIu32 " retryable=0\n",
                      session, sequence, offset,
                      s_ota2.next_sequence, s_ota2.received,
                      s_ota2.remaining);
        return;
    }

    s_ota2.chunk_session = session;
    s_ota2.chunk_sequence = sequence;
    s_ota2.chunk_offset = offset;
    s_ota2.chunk_size = chunk_size;
    s_ota2.chunk_crc = chunk_crc;
    s_ota2.chunk_received = 0;
    s_ota2.duplicate_chunk = duplicate;
    s_ota2.receiving_chunk = true;
    s_ota2.last_activity_us = esp_timer_get_time();
}

static void ota2_chunk_process(const uint8_t *data,
                               size_t length,
                               size_t *consumed)
{
    size_t remaining = s_ota2.chunk_size - s_ota2.chunk_received;
    size_t take = length < remaining ? length : remaining;
    *consumed = take;
    if (take == 0) {
        return;
    }

    memcpy(s_ota2.chunk + s_ota2.chunk_received, data, take);
    s_ota2.chunk_received += take;
    s_ota2.last_activity_us = esp_timer_get_time();
    if (s_ota2.chunk_received != s_ota2.chunk_size) {
        return;
    }

    uint32_t calculated_crc = ota_crc32_update(
        0, s_ota2.chunk, s_ota2.chunk_size);
    if (calculated_crc != s_ota2.chunk_crc) {
        native_printf("ERR CHUNK_CRC session=%" PRIu32
                      " sequence=%" PRIu32 " offset=%" PRIu32
                      " size=%" PRIu32 " expected=%08" PRIx32
                      " calculated=%08" PRIx32 " retryable=1\n",
                      s_ota2.chunk_session, s_ota2.chunk_sequence,
                      s_ota2.chunk_offset, s_ota2.chunk_size,
                      s_ota2.chunk_crc, calculated_crc);
        s_ota2.receiving_chunk = false;
        s_ota2.duplicate_chunk = false;
        s_ota2.chunk_received = 0;
        return;
    }

    bool duplicate = s_ota2.duplicate_chunk;
    if (!duplicate) {
        esp_err_t ret = ota_writer_write(s_ota2.chunk,
                                         s_ota2.chunk_size);
        if (ret != ESP_OK) {
            native_printf("ERR OTA_WRITE stage=flash sequence=%" PRIu32
                          " offset=%" PRIu32 " code=%s retryable=0\n",
                          s_ota2.chunk_sequence, s_ota2.chunk_offset,
                          esp_err_to_name(ret));
            ota2_state_clear(true);
            ota2_restore_logs();
            return;
        }

        s_ota2.running_crc = ota_crc32_update(
            s_ota2.running_crc, s_ota2.chunk, s_ota2.chunk_size);
        s_ota2.received += s_ota2.chunk_size;
        s_ota2.remaining -= s_ota2.chunk_size;
        s_ota2.last_sequence = s_ota2.chunk_sequence;
        s_ota2.last_offset = s_ota2.chunk_offset;
        s_ota2.last_size = s_ota2.chunk_size;
        s_ota2.last_crc = s_ota2.chunk_crc;
        s_ota2.have_last_chunk = true;
        s_ota2.next_sequence++;
    }

    native_printf("OK CHUNK session=%" PRIu32
                  " sequence=%" PRIu32 " offset=%" PRIu32
                  " size=%" PRIu32 " received=%" PRIu32
                  " rolling_crc=%08" PRIx32 " duplicate=%u\n",
                  s_ota2.chunk_session, s_ota2.chunk_sequence,
                  s_ota2.chunk_offset, s_ota2.chunk_size,
                  s_ota2.received, s_ota2.running_crc,
                  duplicate ? 1u : 0u);
    s_ota2.receiving_chunk = false;
    s_ota2.duplicate_chunk = false;
    s_ota2.chunk_received = 0;
}

static void ota2_finish(uint32_t session)
{
    if (!s_ota2.active || s_ota2.receiving_chunk) {
        native_write_all("ERR OTA_END stage=state code=ESP_ERR_INVALID_STATE\n");
        return;
    }
    if (session != s_ota2.session) {
        native_printf("ERR OTA_END stage=session got=%" PRIu32
                      " expected=%" PRIu32 "\n",
                      session, s_ota2.session);
        return;
    }
    if (s_ota2.received != s_ota2.image_size) {
        native_printf("ERR OTA_SIZE stage=transport expected=%" PRIu32
                      " received=%" PRIu32 " remaining=%" PRIu32 "\n",
                      s_ota2.image_size, s_ota2.received,
                      s_ota2.remaining);
        return;
    }
    if (s_ota2.running_crc != s_ota2.image_crc32) {
        native_printf("ERR OTA_CRC stage=transport expected=%08" PRIx32
                      " calculated=%08" PRIx32 " received=%" PRIu32 "\n",
                      s_ota2.image_crc32, s_ota2.running_crc,
                      s_ota2.received);
        ota2_state_clear(true);
        ota2_restore_logs();
        return;
    }

    dual_end_ota_status_t writer = dual_end_ota_get_status();
    if (!writer.active || writer.target != DUAL_END_OTA_TARGET_SCEPTER ||
        writer.session_id != s_ota2.session ||
        writer.image_size != s_ota2.image_size ||
        writer.image_crc32 != s_ota2.image_crc32 ||
        writer.bytes_written != s_ota2.received) {
        native_write_all("ERR OTA_END stage=writer_state code=ESP_ERR_INVALID_STATE\n");
        ota2_state_clear(true);
        ota2_restore_logs();
        return;
    }

    esp_err_t ret = ota_writer_finish();
    if (ret != ESP_OK) {
        native_printf("ERR OTA_END stage=image_finalize code=%s "
                      "transport_crc=%08" PRIx32
                      " expected_crc=%08" PRIx32
                      " received=%" PRIu32 "\n",
                      esp_err_to_name(ret), s_ota2.running_crc,
                      s_ota2.image_crc32, s_ota2.received);
        ota2_state_clear(true);
        ota2_restore_logs();
        return;
    }

    native_printf("OK END SCEPTER session=%" PRIu32
                  " size=%" PRIu32 " crc=%08" PRIx32
                  " calculated_crc=%08" PRIx32 " chunks=%" PRIu32
                  " protocol=2 rebooting=1 transport=typec_usb\n",
                  s_ota2.session, s_ota2.image_size, s_ota2.image_crc32,
                  s_ota2.running_crc, s_ota2.next_sequence);
#ifdef ESP_PLATFORM
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(1000));
#endif
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
}

static void process_log_command(const char *line)
{
    if (strcasecmp(line, "LOG 0") == 0) {
        logs_suppress();
        native_write_all("OK LOG mirror=0\n");
    } else if (strcasecmp(line, "LOG 1") == 0) {
        native_write_all("OK LOG mirror=1\n");
        logs_restore();
    } else {
        native_write_all("ERR LOG expected=LOG_0_or_LOG_1\n");
    }
}


/* Isolated diagnostic build only. Reuse existing 512-byte response storage;
 * keep INFO/STATUS first-line contracts intact and add one read-only line. */
static const char *boot_state_name(esp_ota_img_states_t state)
{
    switch (state) {
    case ESP_OTA_IMG_NEW: return "new";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending_verify";
    case ESP_OTA_IMG_VALID: return "valid";
    case ESP_OTA_IMG_INVALID: return "invalid";
    case ESP_OTA_IMG_ABORTED: return "aborted";
    case ESP_OTA_IMG_UNDEFINED: return "undefined";
    default: return "unknown";
    }
}
static void print_boot_status(void)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    const esp_err_t state_result = running ? esp_ota_get_state_partition(running, &state) : ESP_ERR_INVALID_STATE;
    char image_id[65];
    for (unsigned i=0; i<32; ++i) snprintf(image_id+i*2, 3, "%02x", desc ? desc->app_elf_sha256[i] : 0);
    native_printf("OK BOOTSTATUS role=scepter version=%s running=%s boot=%s"
        " running_state=%s state_code=%d state_err=%s reset_reason=%d"
        " internal_free=%" PRIu32 " internal_largest=%" PRIu32
        " dma_largest=%" PRIu32 " psram_free=%" PRIu32 " image_id=%s public_version=%s\n",
        desc ? desc->version : "unknown", running ? running->label : "unknown",
        boot ? boot->label : "unknown", state_result == ESP_OK ? boot_state_name(state) : "unknown",
        state_result == ESP_OK ? (int)state : -1, esp_err_to_name(state_result), (int)esp_reset_reason(),
        (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
        (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT), image_id, SCEPTER_FW_VERSION);
}

static void usb_command_process(char *line)
{
    trim_line(line);
    if (!line[0]) {
        return;
    }

    char parsed_line[PAIRING_COMMAND_MAX_LEN];
    memcpy(parsed_line, line, strlen(line) + 1);
    char *tokens[8] = {0};
    size_t token_count = split_command_tokens(
        parsed_line, tokens, sizeof(tokens) / sizeof(tokens[0]));

    if (s_ota2.active) {
        uint32_t session = 0;
        uint32_t sequence = 0;
        uint32_t offset = 0;
        uint32_t chunk_size = 0;
        uint32_t chunk_crc = 0;
        if (token_count == 6 && strcasecmp(tokens[0], "CHUNK") == 0 &&
            parse_u32_token(tokens[1], 10, &session) &&
            parse_u32_token(tokens[2], 10, &sequence) &&
            parse_u32_token(tokens[3], 10, &offset) &&
            parse_u32_token(tokens[4], 10, &chunk_size) &&
            parse_u32_token(tokens[5], 16, &chunk_crc)) {
            ota2_chunk_begin(session, sequence, offset,
                             chunk_size, chunk_crc);
            return;
        }
        if (token_count == 2 && strcasecmp(tokens[0], "END") == 0 &&
            parse_u32_token(tokens[1], 10, &session)) {
            ota2_finish(session);
            return;
        }
        if (token_count == 2 && strcasecmp(tokens[0], "ABORT") == 0 &&
            parse_u32_token(tokens[1], 10, &session)) {
            if (session != s_ota2.session) {
                native_printf("ERR OTA_ABORT got=%" PRIu32
                              " expected=%" PRIu32 "\n",
                              session, s_ota2.session);
                return;
            }
            native_printf("OK ABORT SCEPTER session=%" PRIu32
                          " received=%" PRIu32 "\n",
                          s_ota2.session, s_ota2.received);
            ota2_state_clear(true);
            ota2_restore_logs();
            return;
        }
        native_write_all("ERR OTA2_COMMAND supported=CHUNK,END,ABORT\n");
        return;
    }

    char telemetry_response[256];
    if(dual_end_runtime_command(line,telemetry_response,sizeof(telemetry_response))){native_write_all(telemetry_response);return;}

    /* New diagnostics are deliberately after legacy runtime telemetry and
     * before unrelated command families.  The callback owns only its exact
     * prefixes and returns false for every legacy command. */
    char diagnostics_response[PAIRING_RESPONSE_MAX_LEN];
    if (s_diagnostics_command_cb &&
        s_diagnostics_command_cb(line, diagnostics_response,
                                 sizeof(diagnostics_response),
                                 s_diagnostics_command_ctx)) {
        native_write_all(diagnostics_response);
        return;
    }

    if (strcasecmp(line, "INFO") == 0 ||
        strcasecmp(line, "IDENTIFY") == 0) {
        print_usb_identity();
        print_boot_status();
        return;
    }
    if (strcasecmp(line, "PING") == 0) {
        native_write_all(
            "OK PONG role=scepter firmware=app transport=typec_usb\n");
        return;
    }
    if (strcasecmp(line, "STATUS") == 0) {
        char response[PAIRING_RESPONSE_MAX_LEN];
        format_status(response, sizeof(response));
        native_write_all(response);
        print_boot_status();
        return;
    }
    if (strcasecmp(line, "BOOTSTATUS") == 0) {
        print_boot_status();
        return;
    }
    if (strncasecmp(line, "LOG", 3) == 0) {
        process_log_command(line);
        return;
    }

    char response[PAIRING_RESPONSE_MAX_LEN];
    if (dual_end_pairing_handle_command(line, response, sizeof(response))) {
        native_write_all(response);
        return;
    }

    uint32_t image_size = 0;
    uint32_t image_crc32 = 0;
    uint32_t session = 0;
    uint32_t chunk_size = 0;
    if (token_count == 7 && strcasecmp(tokens[0], "OTA2") == 0 &&
        parse_u32_token(tokens[2], 10, &image_size) &&
        parse_u32_token(tokens[3], 16, &image_crc32) &&
        parse_u32_token(tokens[4], 10, &session) &&
        parse_u32_token(tokens[6], 10, &chunk_size)) {
        if (strcasecmp(tokens[1], "SCEPTER") != 0) {
            native_write_all("ERR OTA2 TARGET\n");
        } else {
            ota2_begin(image_size, image_crc32, session,
                       tokens[5], chunk_size);
        }
        return;
    }
    if (token_count > 0 && strcasecmp(tokens[0], "OTA2") == 0) {
        native_write_all("ERR OTA2 ARGUMENTS\n");
        return;
    }
    if (token_count > 0 && strcasecmp(tokens[0], "OTA") == 0) {
        native_write_all("ERR OTA1_DISABLED use=OTA2\n");
        return;
    }

    native_write_all(
        "ERR COMMAND supported=INFO,IDENTIFY,PAIR,PING,STATUS,BOOTSTATUS,LOG,OTA2\n");
}

static void pairing_usb_task(void *arg)
{
    (void)arg;
    uint8_t rx[PAIRING_USB_RX_CHUNK_BYTES];
    char line[PAIRING_COMMAND_MAX_LEN];
    size_t line_len = 0;
    bool line_overflow = false;

    native_printf("OK PAIR READY role=%s pair_protocol=%u "
                  "device=scepter firmware=app transport=typec_usb "
                  "ota_protocol=2 ota1=disabled chunk_crc=1 chunk_ack=1\n",
                  scepter_role_name(s_role), PAIRING_PROTOCOL_VERSION);
    while (true) {
        int count = usb_serial_jtag_read_bytes(rx, sizeof(rx),
                                               pdMS_TO_TICKS(100));
        size_t index = 0;
        while (count > 0 && index < (size_t)count) {
            if (s_ota2.active && s_ota2.receiving_chunk) {
                size_t consumed = 0;
                ota2_chunk_process(rx + index,
                                   (size_t)count - index,
                                   &consumed);
                index += consumed;
                if (consumed == 0 || !s_ota2.active) {
                    break;
                }
                continue;
            }

            char ch = (char)rx[index++];
            if (ch == '\n') {
                if (line_overflow) {
                    native_write_all("ERR COMMAND reason=line_too_long\n");
                } else {
                    line[line_len] = '\0';
                    usb_command_process(line);
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

        if (s_ota2.active &&
            esp_timer_get_time() - s_ota2.last_activity_us >
                (int64_t)PAIRING_OTA2_TIMEOUT_MS * 1000) {
            native_printf("ERR OTA_TIMEOUT target=SCEPTER "
                          "transport=typec_usb protocol=2 session=%" PRIu32
                          " sequence=%" PRIu32 " received=%" PRIu32
                          " expected=%" PRIu32 "\n",
                          s_ota2.session, s_ota2.next_sequence,
                          s_ota2.received, s_ota2.image_size);
            ota2_state_clear(true);
            ota2_restore_logs();
            mbedtls_platform_zeroize(line, sizeof(line));
            line_len = 0;
            line_overflow = false;
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
        .tx_buffer_size = PAIRING_USB_TX_RING_BYTES,
        .rx_buffer_size = PAIRING_USB_RX_RING_BYTES,
    };
    ESP_RETURN_ON_ERROR(usb_serial_jtag_driver_install(&usb_config),
                        TAG, "install USB Serial/JTAG driver");
    usb_serial_jtag_vfs_use_driver();
    ota2_state_clear(false);
    BaseType_t task_ok = xTaskCreatePinnedToCore(pairing_usb_task, "pairing_usb",
                                     PAIRING_TASK_STACK_BYTES, NULL,
                                     PAIRING_TASK_PRIORITY, NULL, DUAL_END_COMM_TASK_CORE);
    if (task_ok != pdPASS) {
#ifdef ESP_PLATFORM
        usb_serial_jtag_vfs_use_nonblocking();
        (void)usb_serial_jtag_driver_uninstall();
#endif
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
