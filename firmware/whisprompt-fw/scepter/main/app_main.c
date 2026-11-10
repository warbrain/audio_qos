#include <inttypes.h>
#include "dual_end_runtime.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "esp_random.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "dual_end_audio.h"
#include "dual_end_link_config.h"
#include "dual_end_espnow_link.h"
#include "dual_end_ota.h"
#include "dual_end_pairing.h"
#include "scepter_hardware.h"
#include "scepter_diagnostics.h"
#include "mic_audio.h"
#include "dual_end_codec_frame.h"

static const char *TAG = "scepter";

#define SCEPTER_CONTROL_QUEUE_DEPTH 8
#define SCEPTER_CONTROL_TASK_STACK_BYTES 3072
#define SCEPTER_CONTROL_TASK_PRIORITY 4
#define SCEPTER_PAIR_SUCCESS_VIBRATE_MS 200

typedef struct {
    uint32_t sequence;
    uint8_t command;
    uint16_t duration_ms;
    uint8_t mic_gain_x;
    uint8_t dsp[13];
} scepter_control_item_t;

static QueueHandle_t s_control_queue;
static bool s_link_status_initialized;
static bool s_link_connected;

static void scepter_control_task(void *arg)
{
    (void)arg;
    scepter_control_item_t item;

    while (true) {
        if (xQueueReceive(s_control_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        esp_err_t ret;
        if (item.command == 0x0B) {
            uint16_t actual=0;
            ret=mic_audio_parameter(item.dsp[1]==2,item.dsp[2],(uint16_t)item.dsp[7]|((uint16_t)item.dsp[8]<<8),&actual);
            item.dsp[7]=(uint8_t)actual; item.dsp[8]=(uint8_t)(actual>>8);
            item.dsp[9]=ret==ESP_OK ? 0 : ret==ESP_ERR_TIMEOUT ? 3 : 2;
            dual_end_control_packet_t response={
                .packet={.protocol_version=SCEPTER_PROTOCOL_VERSION,.source_role=SCEPTER_ROLE_SCEPTER,
                    .message_type=SCEPTER_MSG_CONTROL_RESPONSE,.sequence=item.sequence,
                    .payload_len=DUAL_END_CONTROL_PAYLOAD_BASE_BYTES+13},
                .control_version=DUAL_END_CONTROL_VERSION,.command=0x0B,.data_len=13};
            memcpy(response.data,item.dsp,13);
            ESP_LOGI(TAG,"DSP item=%u op=%u value=%u result=%s",item.dsp[2],item.dsp[1],actual,esp_err_to_name(ret));
            dual_end_espnow_link_send_control(&response,DUAL_END_CONTROL_PACKET_WIRE_BYTES(13));
            continue;
        }
        if (item.command == DUAL_END_CONTROL_CMD_MOTOR_VIBRATE) {
            ret = scepter_hardware_vibrate(item.duration_ms);
        } else if (item.command == DUAL_END_CONTROL_CMD_MIC_FIXED_GAIN) {
            ret = scepter_hardware_set_mic_fixed_gain(item.mic_gain_x, true);
        } else {
            ret = scepter_hardware_play_ui_animation(item.command);
        }
        ESP_LOGI(TAG,
                 "control execute seq=%" PRIu32 " cmd=0x%02x duration=%ums mic_gain=%ux ret=%s",
                 item.sequence,
                 item.command,
                 (unsigned)item.duration_ms,
                 (unsigned)item.mic_gain_x,
                 esp_err_to_name(ret));
    }
}

static bool scepter_queue_control_packet(const uint8_t *src_addr,
                                         const uint8_t *data,
                                         int len)
{
    uint8_t active_peer[6] = {0};
    if (!src_addr ||
        dual_end_espnow_link_get_peer(active_peer, NULL) != ESP_OK ||
        memcmp(src_addr, active_peer, sizeof(active_peer)) != 0 ||
        !data || len < (int)DUAL_END_CONTROL_PACKET_BASE_BYTES ||
        s_control_queue == NULL) {
        return false;
    }

    const dual_end_control_packet_t *packet =
        (const dual_end_control_packet_t *)data;
    if (packet->packet.protocol_version != SCEPTER_PROTOCOL_VERSION ||
        packet->packet.source_role != SCEPTER_ROLE_DONGLE ||
        packet->packet.message_type != SCEPTER_MSG_CONTROL_REQUEST ||
        packet->control_version != DUAL_END_CONTROL_VERSION ||
        packet->reserved != 0 ||
        packet->data_len > DUAL_END_CONTROL_MAX_DATA_BYTES) {
        return false;
    }

    uint16_t expected_payload_len =
        DUAL_END_CONTROL_PAYLOAD_BASE_BYTES + packet->data_len;
    int expected_wire_len =
        (int)DUAL_END_CONTROL_PACKET_WIRE_BYTES(packet->data_len);
    if (packet->packet.payload_len != expected_payload_len ||
        len != expected_wire_len) {
        ESP_LOGW(TAG, "invalid control packet len=%d", len);
        return true;
    }

    scepter_control_item_t item = {
        .sequence = packet->packet.sequence,
        .command = packet->command,
    };
    switch (packet->command) {
    case 0x0B:
        if (packet->data_len!=13 || !dual_end_dsp_version_item_valid(packet->data[0],packet->data[2]) || (packet->data[1]!=1 && packet->data[1]!=2) ||
            packet->data[9] || packet->data[10] || packet->data[11] || packet->data[12]) return true;
        memcpy(item.dsp,packet->data,13);
        break;
    case DUAL_END_CONTROL_CMD_UI_THINKING:
    case DUAL_END_CONTROL_CMD_UI_APPROVE:
    case DUAL_END_CONTROL_CMD_UI_DONE:
        if (packet->data_len != 0) {
            ESP_LOGW(TAG, "invalid UI control data_len=%u", (unsigned)packet->data_len);
            return true;
        }
        break;

    case DUAL_END_CONTROL_CMD_MOTOR_VIBRATE:
        if (packet->data_len != DUAL_END_CONTROL_MOTOR_VIBRATE_DATA_BYTES) {
            ESP_LOGW(TAG, "invalid vibrate data_len=%u", (unsigned)packet->data_len);
            return true;
        }
        item.duration_ms = (uint16_t)packet->data[0] |
                           ((uint16_t)packet->data[1] << 8);
        if (item.duration_ms < DUAL_END_CONTROL_MOTOR_VIBRATE_MIN_MS ||
            item.duration_ms > DUAL_END_CONTROL_MOTOR_VIBRATE_MAX_MS) {
            ESP_LOGW(TAG, "invalid vibrate duration=%u", (unsigned)item.duration_ms);
            return true;
        }
        break;

    case DUAL_END_CONTROL_CMD_MIC_FIXED_GAIN:
        if (packet->data_len != DUAL_END_CONTROL_MIC_GAIN_DATA_BYTES) {
            ESP_LOGW(TAG, "invalid microphone gain data_len=%u",
                     (unsigned)packet->data_len);
            return true;
        }
        item.mic_gain_x = packet->data[0];
        if (item.mic_gain_x < DUAL_END_MIC_GAIN_MIN_X ||
            item.mic_gain_x > DUAL_END_MIC_GAIN_MAX_X) {
            ESP_LOGW(TAG, "invalid microphone fixed gain=%u",
                     (unsigned)item.mic_gain_x);
            return true;
        }
        break;

    default:
        ESP_LOGW(TAG, "unsupported control cmd=0x%02x", packet->command);
        return true;
    }

    if (xQueueSend(s_control_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG,
                 "control queue full seq=%" PRIu32 " cmd=0x%02x",
                 item.sequence,
                 item.command);
    }
    return true;
}

static esp_err_t scepter_nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "erase nvs");
        ret = nvs_flash_init();
    }
    return ret;
}

static void scepter_link_status_cb(bool connected, int rssi, void *ctx)
{
    (void)ctx;
    if (!s_link_status_initialized || connected != s_link_connected) {
        s_link_status_initialized = true;
        s_link_connected = connected;
        ESP_LOGI(TAG, "link_status connected=%d rssi=%d", connected ? 1 : 0, rssi);
        scepter_hardware_set_connected(connected);
    }
    if (connected && dual_end_pairing_take_link_confirmation_pending()) {
        esp_err_t ret = scepter_hardware_vibrate(SCEPTER_PAIR_SUCCESS_VIBRATE_MS);
        ESP_LOGI(TAG,
                 "pairing link confirmed; vibrate=%ums ret=%s",
                 (unsigned)SCEPTER_PAIR_SUCCESS_VIBRATE_MS,
                 esp_err_to_name(ret));
    }
}
static void scepter_ota_activity_cb(bool active, void *ctx)
{
    (void)ctx;
    dual_end_espnow_link_set_ota_active(active);
    dual_end_audio_sender_set_ota_paused(active);
    scepter_hardware_set_ota_active(active);
}

static void scepter_espnow_recv_cb(const uint8_t *src_addr, const uint8_t *data, int len, int rssi, void *ctx)
{
    (void)rssi;
    (void)ctx;
    if (dual_end_audio_sender_handle_retry_request(data, len)) {
        return;
    }
    if (scepter_queue_control_packet(src_addr, data, len)) {
        return;
    }
    if (!dual_end_ota_handle_scepter_espnow_packet(data, len)) {
        ESP_LOGD(TAG, "drop unsupported packet len=%d", len);
    }
}


/* Diagnostic variant only. No new task, queue, driver or large log buffer.
 * Primary USB console uses the SDK FIFO path until existing pairing startup
 * installs its USB Serial/JTAG driver. Keep all original startup operations,
 * task stacks, priorities, codec defaults and rollback behavior unchanged. */
static uint32_t s_startup_diag_boot;
static void startup_diag(const char *stage, const char *phase, esp_err_t result)
{
    ESP_LOGI(TAG, "STARTUP_DIAG boot=%08" PRIx32 " stage=%s phase=%s err=%s"
        " int_free=%" PRIu32 " int_largest=%" PRIu32 " dma_largest=%" PRIu32
        " psram_free=%" PRIu32 " core=%d us=%" PRIu64,
        s_startup_diag_boot, stage, phase, esp_err_to_name(result),
        (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
        (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT),
        xPortGetCoreID(), (uint64_t)esp_timer_get_time());
}
#define STARTUP_DIAG_CHECK(stage, operation) do { \
    startup_diag((stage), "begin", ESP_OK); \
    const esp_err_t startup_result = (operation); \
    startup_diag((stage), startup_result == ESP_OK ? "done" : "fail", startup_result); \
    ESP_ERROR_CHECK(startup_result); \
} while (0)

static void startup_diag_identity(void)
{
    do { s_startup_diag_boot=esp_random(); } while(!s_startup_diag_boot);
    ESP_ERROR_CHECK(dual_end_runtime_set_boot_id(s_startup_diag_boot));
    const esp_app_desc_t *desc = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    const esp_err_t state_result = running ? esp_ota_get_state_partition(running, &state) : ESP_ERR_INVALID_STATE;
    char image_id[65];
    for (unsigned i=0; i<32; ++i) snprintf(image_id+i*2, 3, "%02x", desc ? desc->app_elf_sha256[i] : 0);
    ESP_EARLY_LOGI(TAG, "STARTUP_DIAG identity boot=%08" PRIx32
        " fw=%s project=%s reset=%d partition=%s image_state=%d state_err=%s image_id=%s",
        s_startup_diag_boot, desc ? desc->version : "unknown",
        desc ? desc->project_name : "unknown", (int)esp_reset_reason(),
        running ? running->label : "unknown", (int)state, esp_err_to_name(state_result), image_id);
    startup_diag("app_main", "begin", ESP_OK);
}

void app_main(void)
{
    startup_diag_identity();
    const scepter_role_t role = SCEPTER_ROLE_SCEPTER;
    ESP_LOGI(TAG, "CytoScepter scepter boot, fw=%s", SCEPTER_FW_VERSION);

    STARTUP_DIAG_CHECK("nvs", scepter_nvs_init());
    STARTUP_DIAG_CHECK("hardware", scepter_hardware_start());
    startup_diag("control_queue", "begin", ESP_OK);
    s_control_queue = xQueueCreate(SCEPTER_CONTROL_QUEUE_DEPTH,
                                   sizeof(scepter_control_item_t));
    startup_diag("control_queue", s_control_queue ? "done" : "fail", s_control_queue ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(s_control_queue ? ESP_OK : ESP_ERR_NO_MEM);
    startup_diag("control_task", "begin", ESP_OK);
    BaseType_t control_task_ok = xTaskCreatePinnedToCore(scepter_control_task,
                                             "scepter_ctrl",
                                             SCEPTER_CONTROL_TASK_STACK_BYTES,
                                             NULL,
                                             SCEPTER_CONTROL_TASK_PRIORITY,
                                             NULL, DUAL_END_COMM_TASK_CORE);
    startup_diag("control_task", control_task_ok == pdPASS ? "done" : "fail", control_task_ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(control_task_ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    STARTUP_DIAG_CHECK("ota_service", dual_end_ota_service_init(role));
    STARTUP_DIAG_CHECK("ota_activity", dual_end_ota_register_activity_cb(scepter_ota_activity_cb, NULL));
    /* USB Serial/JTAG needs ~10 KB of contiguous DMA-capable internal RAM.
     * ESP-NOW consumes most of the internal heap, so install the pairing
     * driver before the radio starts or the install fails with NO_MEM. */
    dual_end_pairing_set_diagnostics_command_cb(scepter_diagnostics_handle_command, NULL);
    if (scepter_diagnostics_start() != ESP_OK)
        ESP_LOGW(TAG, "diagnostics_cache_disabled; audio/control services unchanged");
    STARTUP_DIAG_CHECK("pairing_usb", dual_end_pairing_service_init(role, true));
    STARTUP_DIAG_CHECK("espnow", dual_end_espnow_link_init(role));
    STARTUP_DIAG_CHECK("link_status_cb", dual_end_espnow_link_register_status_cb(scepter_link_status_cb, NULL));
    STARTUP_DIAG_CHECK("radio_recv_cb", dual_end_espnow_link_register_recv_cb(scepter_espnow_recv_cb, NULL));
    STARTUP_DIAG_CHECK("audio_sender", dual_end_audio_sender_start());
    STARTUP_DIAG_CHECK("confirm_image", dual_end_ota_confirm_running_image());
    startup_diag("app_main", "confirmed", ESP_OK);
    (void)dual_end_runtime_set_io_probe(mic_audio_get_runtime_io);
    startup_diag("runtime_profiler", "begin", ESP_OK);
    esp_err_t profiler_result = dual_end_runtime_start(role);
    startup_diag("runtime_profiler", profiler_result == ESP_OK ? "done" : "fail", profiler_result);
    if (profiler_result != ESP_OK)
        ESP_LOGW(TAG, "profiling_disabled; starting original audio/control services");


    while (true) {
        dual_end_espnow_link_stats_t link_stats = {0};
        dual_end_espnow_link_get_stats(&link_stats);
        ESP_LOGI(TAG,
                 "health up=%" PRIu64 "s link=%d rssi=%d heap=%" PRIu32
                 " min_heap=%" PRIu32,
                 (uint64_t)(esp_timer_get_time() / 1000000ULL),
                 link_stats.connected ? 1 : 0,
                 link_stats.last_rssi,
                 esp_get_free_heap_size(),
                 esp_get_minimum_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
