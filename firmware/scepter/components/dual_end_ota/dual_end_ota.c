#include "dual_end_ota.h"
#include "dual_end_runtime.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "dual_end_espnow_link.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "dual_end_ota";
static esp_ota_handle_t s_ota_handle = 0;
static const esp_partition_t *s_update_partition;
static dual_end_ota_status_t s_status = {0};
static uint32_t s_running_crc;
static uint32_t s_scepter_expected_sequence;
static scepter_role_t s_service_role;
static QueueHandle_t s_wireless_queue;
static SemaphoreHandle_t s_ack_sem;
static SemaphoreHandle_t s_result_sem;
static portMUX_TYPE s_ack_lock = portMUX_INITIALIZER_UNLOCKED;
static dual_end_ota_packet_t s_expected_ack;
static uint32_t s_expected_payload_crc;
static dual_end_ota_ack_payload_t s_last_ack;
static bool s_waiting_for_ack;
static bool s_waiting_for_result;
static bool s_result_received;
static uint32_t s_expected_result_session;
static dual_end_ota_result_t s_last_result;
static volatile bool s_relay_active;
static uint32_t s_relay_image_size;
static uint32_t s_relay_image_crc;
static bool s_wireless_session;
static int64_t s_wireless_activity_us;
static bool s_have_last_data;
static dual_end_ota_packet_t s_last_data;
static uint32_t s_last_data_crc;
static volatile bool s_boot_confirmed;
static esp_err_t s_boot_confirmation_error;
typedef struct {
    uint32_t magic;
    uint32_t session_id;
    uint32_t image_size;
    uint32_t image_crc32;
    char partition[DUAL_END_OTA_RESULT_PARTITION_LEN];
    char app_version[DUAL_END_OTA_RESULT_APP_VERSION_LEN];
    uint32_t crc32;
} ota_pending_result_t;
static void wireless_ota_task(void *arg);
static esp_err_t persist_wireless_install_result(void);
static esp_err_t pending_result_write(const ota_pending_result_t *record);
static esp_err_t send_packet_with_ack(dual_end_ota_packet_t *packet, uint32_t timeout_ms);
static dual_end_ota_activity_cb_t s_activity_cb;
static void *s_activity_ctx;
static const char s_unknown_partition[] = "unknown";

#define DUAL_END_OTA_ESPNOW_NO_MEM_RETRY_COUNT 30
#define DUAL_END_OTA_ESPNOW_NO_MEM_RETRY_DELAY_MS 10

#ifndef DUAL_END_OTA_ROUTE_LOGS_TO_UART0
#define DUAL_END_OTA_ROUTE_LOGS_TO_UART0 0
#endif

static void notify_ota_activity(bool active)
{
    if (s_activity_cb) {
        s_activity_cb(active, s_activity_ctx);
    }
}

static bool is_ota_partition_label(const char *label)
{
    return label && (strcmp(label, "ota_0") == 0 || strcmp(label, "ota_1") == 0);
}

static const esp_partition_t *find_app_partition(const char *label)
{
    if (!label || strcmp(label, "auto") == 0 || label[0] == '\0') {
        return esp_ota_get_next_update_partition(NULL);
    }
    return esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, label);
}

static const char *partition_code_to_label(uint8_t code)
{
    switch (code) {
    case 1:
        return "ota_0";
    case 2:
        return "ota_1";
    case 3:
        return "factory";
    default:
        return "auto";
    }
}

static uint8_t partition_label_to_code(const char *label)
{
    if (!label) {
        return 0;
    }
    if (strcmp(label, "ota_0") == 0) {
        return 1;
    }
    if (strcmp(label, "ota_1") == 0) {
        return 2;
    }
    if (strcmp(label, "factory") == 0) {
        return 3;
    }
    return 0;
}

const char *dual_end_ota_image_key_from_version(const char *version)
{
    if (!version) {
        return "unknown";
    }
    if (strncmp(version, DUAL_END_OTA_IMAGE_KEY_PREFIX_APP,
                strlen(DUAL_END_OTA_IMAGE_KEY_PREFIX_APP)) == 0) {
        return DUAL_END_OTA_APP_KEY;
    }
    if (strncmp(version, DUAL_END_OTA_IMAGE_KEY_PREFIX_FACTORY,
                strlen(DUAL_END_OTA_IMAGE_KEY_PREFIX_FACTORY)) == 0) {
        return DUAL_END_OTA_FACTORY_KEY;
    }
    return "unknown";
}

static const char *expected_project_name(dual_end_ota_target_t target)
{
    switch (target) {
    case DUAL_END_OTA_TARGET_DONGLE:
        return "cytoscepter_dongle";
    case DUAL_END_OTA_TARGET_SCEPTER:
        return "cytoscepter_scepter";
    default:
        return "";
    }
}

static dual_end_ota_target_t current_target_from_project(void)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    if (desc && strcmp(desc->project_name, "cytoscepter_scepter") == 0) {
        return DUAL_END_OTA_TARGET_SCEPTER;
    }
    return DUAL_END_OTA_TARGET_DONGLE;
}

static esp_err_t validate_written_image(const esp_partition_t *partition, dual_end_ota_target_t target)
{
    esp_app_desc_t desc = {0};
    ESP_RETURN_ON_FALSE(partition, ESP_ERR_INVALID_ARG, TAG, "missing OTA partition");
    ESP_RETURN_ON_ERROR(esp_ota_get_partition_description(partition, &desc), TAG, "read written app desc");

    const char *expected_project = expected_project_name(target);
    ESP_RETURN_ON_FALSE(strcmp(desc.project_name, expected_project) == 0,
                        ESP_ERR_INVALID_RESPONSE,
                        TAG,
                        "image project mismatch got=%s expected=%s",
                        desc.project_name,
                        expected_project);

    const char *key = dual_end_ota_image_key_from_version(desc.version);
    ESP_RETURN_ON_FALSE(strcmp(key, DUAL_END_OTA_APP_KEY) == 0,
                        ESP_ERR_INVALID_RESPONSE,
                        TAG,
                        "image key mismatch got=%s version=%s expected=%s",
                        key,
                        desc.version,
                        DUAL_END_OTA_APP_KEY);

    ESP_LOGI(TAG, "image verified partition=%s project=%s version=%s key=%s",
             partition->label, desc.project_name, desc.version, key);
    return ESP_OK;
}

#if DUAL_END_OTA_ROUTE_LOGS_TO_UART0
/* Compile UART routing only when enabled; preserve USB logging otherwise. */
static bool s_uart0_log_route_enabled;

static int uart0_log_vprintf(const char *fmt, va_list args)
{
    char line[256];
    int len = vsnprintf(line, sizeof(line), fmt, args);
    if (len > 0) {
        size_t write_len = (size_t)len < sizeof(line) ? (size_t)len : sizeof(line) - 1;
        uart_write_bytes(UART_NUM_0, line, write_len);
    }
    return len;
}

static esp_err_t route_logs_to_uart0(void)
{
    if (s_uart0_log_route_enabled) {
        return ESP_OK;
    }

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_param_config(UART_NUM_0, &uart_config), TAG, "configure UART0 log route");
    esp_err_t ret = uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    esp_log_set_vprintf(uart0_log_vprintf);
    s_uart0_log_route_enabled = true;
    return ESP_OK;
}

#endif /* DUAL_END_OTA_ROUTE_LOGS_TO_UART0 */

esp_err_t dual_end_ota_service_init(scepter_role_t role)
{
#if DUAL_END_OTA_ROUTE_LOGS_TO_UART0
    ESP_RETURN_ON_ERROR(route_logs_to_uart0(), TAG, "route logs to UART0");
#endif
    s_service_role = role;
    if (role == SCEPTER_ROLE_SCEPTER) {
        s_wireless_queue = xQueueCreate(8, sizeof(dual_end_ota_packet_t));
        ESP_RETURN_ON_FALSE(s_wireless_queue, ESP_ERR_NO_MEM, TAG, "create OTA receive queue");
        if (xTaskCreatePinnedToCore(wireless_ota_task, "wireless_ota", 8192, NULL, 5, NULL, DUAL_END_COMM_TASK_CORE) != pdPASS) {
            vQueueDelete(s_wireless_queue);
            s_wireless_queue = NULL;
            return ESP_ERR_NO_MEM;
        }
    } else {
        s_ack_sem = xSemaphoreCreateBinary();
        s_result_sem = xSemaphoreCreateBinary();
        ESP_RETURN_ON_FALSE(s_ack_sem && s_result_sem, ESP_ERR_NO_MEM, TAG, "create OTA ACK semaphores");
    }
    return dual_end_ota_log_boot_partition();
}

esp_err_t dual_end_ota_log_boot_partition(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_app_desc_t *desc = esp_app_get_description();
    ESP_LOGI(TAG, "running partition=%s app=%s version=%s image_key=%s",
             running ? running->label : "unknown",
             desc ? desc->project_name : "unknown",
             desc ? desc->version : "unknown",
             desc ? dual_end_ota_image_key_from_version(desc->version) : "unknown");
    return ESP_OK;
}

const char *dual_end_ota_running_partition_label(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    return running ? running->label : s_unknown_partition;
}

bool dual_end_ota_partition_is_running(const char *partition_label)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    return running && partition_label && strcmp(running->label, partition_label) == 0;
}

void dual_end_ota_get_mac_string(char *buf, size_t len)
{
    uint8_t mac[6] = {0};
    if (!buf || len == 0) {
        return;
    }
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        snprintf(buf, len, "unknown");
        return;
    }
    snprintf(buf, len, "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

dual_end_ota_status_t dual_end_ota_get_status(void)
{
    return s_status;
}

esp_err_t dual_end_ota_register_activity_cb(dual_end_ota_activity_cb_t cb, void *ctx)
{
    s_activity_cb = cb;
    s_activity_ctx = ctx;
    return ESP_OK;
}

uint32_t dual_end_ota_crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    return esp_crc32_le(crc, data, len);
}

uint16_t dual_end_ota_crc16(const uint8_t *data, size_t len)
{
    return esp_crc16_le(UINT16_MAX, data, len);
}

void dual_end_ota_packet_set_crc(dual_end_ota_packet_t *packet)
{
    if (!packet || packet->payload_len > DUAL_END_OTA_MAX_PAYLOAD) {
        return;
    }
    packet->header_crc16 = 0;
    packet->header_crc16 = dual_end_ota_crc16((const uint8_t *)packet,
                                              sizeof(*packet) - DUAL_END_OTA_MAX_PAYLOAD + packet->payload_len);
}

bool dual_end_ota_packet_valid(const dual_end_ota_packet_t *packet, size_t len)
{
    if (!packet || len < (sizeof(*packet) - DUAL_END_OTA_MAX_PAYLOAD)) {
        return false;
    }
    if (packet->magic != DUAL_END_OTA_MAGIC || packet->version != DUAL_END_OTA_VERSION) {
        return false;
    }
    if (packet->payload_len > DUAL_END_OTA_MAX_PAYLOAD) {
        return false;
    }
    size_t wire_len = sizeof(*packet) - DUAL_END_OTA_MAX_PAYLOAD + packet->payload_len;
    if (len != wire_len) {
        return false;
    }

    dual_end_ota_packet_t copy = {0};
    memcpy(&copy, packet, wire_len);
    uint16_t rx_crc = copy.header_crc16;
    copy.header_crc16 = 0;
    return dual_end_ota_crc16((const uint8_t *)&copy, wire_len) == rx_crc;
}

static bool same_partition(const esp_partition_t *left,
                           const esp_partition_t *right)
{
    return left && right && left->type == right->type &&
           left->subtype == right->subtype && left->address == right->address;
}

static esp_err_t select_safe_update_partition(const char *requested_label,
                                              const esp_partition_t **selected)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    ESP_RETURN_ON_FALSE(running && boot && selected,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "cannot resolve running/boot OTA partitions");
    ESP_RETURN_ON_FALSE(same_partition(running, boot),
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "refuse OTA while boot switch is pending running=%s boot=%s",
                        running->label,
                        boot->label);

#if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    if (running->subtype != ESP_PARTITION_SUBTYPE_APP_FACTORY) {
        esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
        esp_err_t state_ret = esp_ota_get_state_partition(running, &state);
        ESP_RETURN_ON_FALSE(state_ret == ESP_OK &&
                                (state == ESP_OTA_IMG_VALID ||
                                 state == ESP_OTA_IMG_UNDEFINED),
                            ESP_ERR_OTA_ROLLBACK_INVALID_STATE,
                            TAG,
                            "refuse OTA while running image is not confirmed state=%d ret=%s",
                            (int)state,
                            esp_err_to_name(state_ret));
    }
#endif

    const esp_partition_t *safe = esp_ota_get_next_update_partition(running);
    ESP_RETURN_ON_FALSE(safe && is_ota_partition_label(safe->label) &&
                            !same_partition(safe, running) &&
                            !same_partition(safe, boot),
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "no safe non-running OTA staging partition");
    if (requested_label && requested_label[0] != '\0' &&
        strcmp(requested_label, "auto") != 0) {
        ESP_RETURN_ON_FALSE(strcmp(requested_label, safe->label) == 0,
                            ESP_ERR_INVALID_ARG,
                            TAG,
                            "requested OTA partition=%s is not safe target=%s",
                            requested_label,
                            safe->label);
    }
    *selected = safe;
    return ESP_OK;
}

esp_err_t dual_end_ota_self_begin(dual_end_ota_target_t target,
                                  uint32_t session_id,
                                  uint32_t image_size,
                                  uint32_t image_crc32)
{
    return dual_end_ota_self_begin_partition(target, session_id, image_size, image_crc32, "auto");
}

esp_err_t dual_end_ota_self_begin_partition(dual_end_ota_target_t target,
                                            uint32_t session_id,
                                            uint32_t image_size,
                                            uint32_t image_crc32,
                                            const char *partition_label)
{
    if (s_status.active || s_relay_active) return ESP_ERR_INVALID_STATE;
    if (target != current_target_from_project() || !session_id || !image_size) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(select_safe_update_partition(partition_label, &s_update_partition),
                        TAG, "select safe OTA partition");
    if (image_size > s_update_partition->size) {
        s_update_partition = NULL;
        return ESP_ERR_INVALID_SIZE;
    }
    /* Sequential erase bounds BEGIN latency and USB/radio queue pressure. */
    esp_err_t ret = esp_ota_begin(s_update_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
    if (ret != ESP_OK) {
        s_update_partition = NULL;
        s_ota_handle = 0;
        return ret;
    }

    s_status = (dual_end_ota_status_t) {
        .target = target,
        .session_id = session_id,
        .image_size = image_size,
        .image_crc32 = image_crc32,
        .bytes_written = 0,
        .packets = 0,
        .active = true,
    };
    s_running_crc = 0;
    notify_ota_activity(true);
    ESP_LOGI(TAG, "OTA begin target=%u session=%" PRIu32 " size=%" PRIu32 " crc=0x%08" PRIx32 " partition=%s",
             target, session_id, image_size, image_crc32, s_update_partition->label);
    return ESP_OK;
}

esp_err_t dual_end_ota_self_write(const uint8_t *data, size_t len)
{
    if (!s_status.active || !data || len == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((uint64_t)s_status.bytes_written + len > s_status.image_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_RETURN_ON_ERROR(esp_ota_write(s_ota_handle, data, len), TAG, "ota write");
    s_running_crc = dual_end_ota_crc32_update(s_running_crc, data, len);
    s_status.bytes_written += len;
    s_status.packets++;
    return ESP_OK;
}

esp_err_t dual_end_ota_self_finish(bool reboot_after_finish)
{
    if (!s_status.active) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = ESP_OK;
    if (s_status.bytes_written != s_status.image_size) ret = ESP_ERR_INVALID_SIZE;
    else if (s_running_crc != s_status.image_crc32) ret = ESP_ERR_INVALID_CRC;
    if (ret != ESP_OK) {
        dual_end_ota_self_abort();
        return ret;
    }
    ret = esp_ota_end(s_ota_handle);
    /* esp_ota_end consumes the handle even on validation failure. */
    s_ota_handle = 0;
    if (ret == ESP_OK) ret = validate_written_image(s_update_partition, s_status.target);
    if (ret == ESP_OK) ret = persist_wireless_install_result();
    if (ret == ESP_OK) ret = esp_ota_set_boot_partition(s_update_partition);
    if (ret != ESP_OK) {
        if (s_wireless_session) (void)pending_result_write(NULL);
        dual_end_ota_self_abort();
        return ret;
    }
    ESP_LOGI(TAG, "OTA committed partition=%s bytes=%" PRIu32, s_update_partition->label, s_status.bytes_written);
    s_update_partition = NULL;
    s_status.active = false;
    if (!s_wireless_session) notify_ota_activity(false);
    if (reboot_after_finish) {
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
    }
    return ESP_OK;
}

esp_err_t dual_end_ota_erase_partition(const char *partition_label)
{
    if (s_status.active || s_relay_active) return ESP_ERR_INVALID_STATE;
    const esp_partition_t *partition = find_app_partition(partition_label);
    ESP_RETURN_ON_FALSE(partition, ESP_ERR_NOT_FOUND, TAG, "find partition for erase");
    ESP_RETURN_ON_FALSE(is_ota_partition_label(partition->label),
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "erase only allows ota_0/ota_1 partition=%s",
                        partition->label);
    ESP_RETURN_ON_FALSE(!dual_end_ota_partition_is_running(partition->label),
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "refuse to erase running partition=%s",
                        partition->label);
    ESP_RETURN_ON_FALSE(!same_partition(partition, esp_ota_get_boot_partition()),
                        ESP_ERR_INVALID_STATE, TAG, "refuse to erase selected boot partition");
    ESP_LOGI(TAG, "erase partition=%s offset=0x%08" PRIx32 " size=0x%08" PRIx32,
             partition->label, partition->address, partition->size);
    return esp_partition_erase_range(partition, 0, partition->size);
}

esp_err_t dual_end_ota_set_boot_partition_label(const char *partition_label)
{
    if (s_status.active || s_relay_active) return ESP_ERR_INVALID_STATE;
    const esp_partition_t *partition = find_app_partition(partition_label);
    ESP_RETURN_ON_FALSE(partition, ESP_ERR_NOT_FOUND, TAG, "find partition for boot switch");
    ESP_RETURN_ON_FALSE(strcmp(partition->label, "factory") == 0 || is_ota_partition_label(partition->label),
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "boot switch only allows factory/ota_0/ota_1 partition=%s",
                        partition->label);
    if (is_ota_partition_label(partition->label)) {
        ESP_RETURN_ON_ERROR(validate_written_image(partition, current_target_from_project()),
                            TAG,
                            "validate boot partition");
    }
    ESP_LOGI(TAG, "set boot partition=%s", partition->label);
    return esp_ota_set_boot_partition(partition);
}

void dual_end_ota_self_abort(void)
{
    if (s_ota_handle) {
        esp_ota_abort(s_ota_handle);
    }
    bool was_active = s_status.active;
    s_wireless_session = false;
    s_have_last_data = false;
    s_ota_handle = 0;
    s_update_partition = NULL;
    s_status.active = false;
    if (was_active) {
        notify_ota_activity(false);
    }
}

static esp_err_t send_packet(dual_end_ota_packet_t *packet)
{
    dual_end_ota_packet_set_crc(packet);
    size_t len = sizeof(*packet) - DUAL_END_OTA_MAX_PAYLOAD + packet->payload_len;
    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < DUAL_END_OTA_ESPNOW_NO_MEM_RETRY_COUNT; ++attempt) {
        ret = dual_end_espnow_link_send_ota(packet, (int)len);
        if (ret == ESP_OK) return ret;
        if (ret != ESP_ERR_ESPNOW_NO_MEM && ret != ESP_FAIL && ret != ESP_ERR_TIMEOUT) return ret;
        if (ret != ESP_ERR_ESPNOW_NO_MEM && attempt >= 2) return ret;
        vTaskDelay(pdMS_TO_TICKS(DUAL_END_OTA_ESPNOW_NO_MEM_RETRY_DELAY_MS));
    }
    return ret;
}

esp_err_t dual_end_ota_send_scepter_begin(uint32_t image_size,
                                          uint32_t image_crc32,
                                          uint32_t session_id,
                                          const char *partition_label)
{
    dual_end_ota_packet_t packet = {
        .magic = DUAL_END_OTA_MAGIC,
        .version = DUAL_END_OTA_VERSION,
        .msg_type = DUAL_END_OTA_MSG_BEGIN,
        .target = DUAL_END_OTA_TARGET_SCEPTER,
        .reserved = partition_label_to_code(partition_label),
        .session_id = session_id,
        .image_size = image_size,
        .image_crc32 = image_crc32,
    };
    if (!image_size || !session_id || s_status.active || s_relay_active ||
        (partition_label && partition_label[0] && strcmp(partition_label, "auto") != 0 &&
         !is_ota_partition_label(partition_label))) return ESP_ERR_INVALID_ARG;
    s_relay_active = true;
    notify_ota_activity(true);
    s_relay_image_size = image_size;
    s_relay_image_crc = image_crc32;
    esp_err_t ret = dual_end_ota_prepare_scepter_result(session_id);
    if (ret == ESP_OK) ret = send_packet_with_ack(&packet, 4000);
    if (ret != ESP_OK) (void)dual_end_ota_send_scepter_abort(session_id);
    return ret;
}

esp_err_t dual_end_ota_send_scepter_data(const uint8_t *data,
                                         size_t len,
                                         uint32_t offset,
                                         uint32_t sequence,
                                         uint32_t image_size,
                                         uint32_t image_crc32,
                                         uint32_t session_id)
{
    if (!data || len == 0 || len > DUAL_END_OTA_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_ARG;
    }

    dual_end_ota_packet_t packet = {
        .magic = DUAL_END_OTA_MAGIC,
        .version = DUAL_END_OTA_VERSION,
        .msg_type = DUAL_END_OTA_MSG_DATA,
        .target = DUAL_END_OTA_TARGET_SCEPTER,
        .session_id = session_id,
        .sequence = sequence,
        .offset = offset,
        .image_size = image_size,
        .image_crc32 = image_crc32,
        .payload_len = (uint16_t)len,
    };
    memcpy(packet.payload, data, len);
    if (!s_relay_active || session_id != s_expected_result_session ||
        image_size != s_relay_image_size || image_crc32 != s_relay_image_crc ||
        (uint64_t)offset + len > image_size) return ESP_ERR_INVALID_STATE;
    return send_packet_with_ack(&packet, 1500);
}

esp_err_t dual_end_ota_send_scepter_end(uint32_t image_size,
                                        uint32_t image_crc32,
                                        uint32_t session_id,
                                        uint32_t sequence)
{
    dual_end_ota_packet_t packet = {
        .magic = DUAL_END_OTA_MAGIC,
        .version = DUAL_END_OTA_VERSION,
        .msg_type = DUAL_END_OTA_MSG_END,
        .target = DUAL_END_OTA_TARGET_SCEPTER,
        .session_id = session_id,
        .sequence = sequence,
        .offset = image_size,
        .image_size = image_size,
        .image_crc32 = image_crc32,
    };
    if (!s_relay_active || session_id != s_expected_result_session ||
        image_size != s_relay_image_size || image_crc32 != s_relay_image_crc) return ESP_ERR_INVALID_STATE;
    return send_packet_with_ack(&packet, 5000);
}

esp_err_t dual_end_ota_send_scepter_erase(const char *partition_label)
{
    dual_end_ota_packet_t packet = {
        .magic = DUAL_END_OTA_MAGIC,
        .version = DUAL_END_OTA_VERSION,
        .msg_type = DUAL_END_OTA_MSG_ERASE,
        .target = DUAL_END_OTA_TARGET_SCEPTER,
        .reserved = partition_label_to_code(partition_label),
    };
    return send_packet_with_ack(&packet, 4000);
}

esp_err_t dual_end_ota_send_scepter_boot(const char *partition_label)
{
    dual_end_ota_packet_t packet = {
        .magic = DUAL_END_OTA_MAGIC,
        .version = DUAL_END_OTA_VERSION,
        .msg_type = DUAL_END_OTA_MSG_BOOT,
        .target = DUAL_END_OTA_TARGET_SCEPTER,
        .reserved = partition_label_to_code(partition_label),
    };
    return send_packet_with_ack(&packet, 4000);
}

esp_err_t dual_end_ota_send_scepter_reboot(void)
{
    dual_end_ota_packet_t packet = {
        .magic = DUAL_END_OTA_MAGIC,
        .version = DUAL_END_OTA_VERSION,
        .msg_type = DUAL_END_OTA_MSG_REBOOT,
        .target = DUAL_END_OTA_TARGET_SCEPTER,
    };
    return send_packet_with_ack(&packet, 4000);
}

#include "dual_end_ota_reliable.inc"
