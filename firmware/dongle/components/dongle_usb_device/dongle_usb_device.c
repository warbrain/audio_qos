#include "dongle_usb_device.h"
#include "dual_end_runtime.h"

#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "esp_app_desc.h"
#include "class/hid/hid_device.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "dual_end_link_config.h"
#include "dual_end_ota.h"
#include "dual_end_pairing.h"
#include "dongle_uac_bridge.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_default_config.h"
#include "usb_device_uac.h"

static const char *TAG = "scepter_usb";

_Static_assert(CONFIG_UAC_SAMPLE_RATE == DUAL_END_AUDIO_SAMPLE_RATE_HZ,
               "USB UAC sample rate must match wireless PCM");
_Static_assert(CONFIG_UAC_MIC_CHANNEL_NUM == DUAL_END_AUDIO_CHANNELS,
               "USB UAC channel count must match wireless PCM");
_Static_assert(CONFIG_UAC_BIT_RESOLUTION == DUAL_END_AUDIO_BITS_PER_SAMPLE,
               "USB UAC sample width must match wireless PCM");
_Static_assert(CONFIG_UAC_BYTES_PER_SAMPLE == (DUAL_END_AUDIO_BITS_PER_SAMPLE / 8),
               "USB UAC sample storage must match wireless PCM");

extern const tusb_desc_device_t dongle_usb_device_descriptor;
extern const uint8_t dongle_usb_configuration_descriptor[];
extern const uint8_t dongle_hid_report_descriptor[];
extern const char *dongle_usb_string_descriptor[];
extern const int dongle_usb_string_descriptor_count;
extern const int dongle_usb_uac_mic_interface;

#define CDC_RX_BLOCK_BYTES 64u
#define CDC_RX_QUEUE_BLOCKS 96u
#define CDC_RESPONSE_TEXT_BYTES 512u
typedef struct {
    uint32_t epoch;
    uint16_t length;
    uint8_t data[CDC_RX_BLOCK_BYTES];
} cdc_rx_block_t;
static QueueHandle_t s_cdc_rx_queue;
static portMUX_TYPE s_cdc_rx_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_cmd_line[128];
static size_t s_cmd_line_len;
static bool s_cmd_line_overflow, s_cmd_line_invalid;
static TaskHandle_t s_cdc_worker;
typedef struct {
    uint32_t epoch;
    uint16_t length;
    uint8_t itf;
    bool completion;
    bool log_line;
    char text[CDC_RESPONSE_TEXT_BYTES];
} cdc_response_t;
static QueueHandle_t s_cdc_responses;
static SemaphoreHandle_t s_cdc_delivery_sem;
static bool s_cdc_delivery_succeeded;
static volatile uint32_t s_cdc_epoch;
static uint32_t s_cdc_response_drops;
static uint32_t s_cdc_rx_commands;
static uint32_t s_cdc_rx_queue_overflows;
static uint64_t s_cdc_response_tx_queued_bytes;
static uint32_t s_cdc_response_tx_incomplete;
static volatile bool s_cdc_disconnect_pending;
static volatile bool s_cdc_rx_overflow_pending;
static volatile bool s_cdc_discard_until_disconnect;
static uint32_t s_cdc_processing_epoch;
static int64_t s_ota_last_rx_us;
#define CDC_OTA2_MAX_CHUNK_BYTES 4096u
_Static_assert(CDC_RX_BLOCK_BYTES * CDC_RX_QUEUE_BLOCKS >= CDC_OTA2_MAX_CHUNK_BYTES + 512u,
               "CDC RX queue must hold a complete host window with headroom");
#define CDC_OTA_RX_TIMEOUT_US 60000000LL
static uint8_t s_ota2_chunk[CDC_OTA2_MAX_CHUNK_BYTES];

#define KEYBOARD_REPORT_QUEUE_SIZE 32
#define MOUSE_SNAPSHOT_QUEUE_SIZE 32
#define HID_SERVICE_TASK_STACK_BYTES 4096
#define HID_SERVICE_TASK_PRIORITY 4

static QueueHandle_t s_keyboard_report_queue;
static QueueHandle_t s_mouse_snapshot_queue;
static portMUX_TYPE s_mouse_report_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_mouse_report_dirty;
static dongle_hid_mouse_report_t s_mouse_pending_report;
static uint8_t s_mouse_last_sent_buttons;
static uint32_t s_mouse_packets_received;
static uint32_t s_mouse_reports_sent;
static uint32_t s_mouse_hid_not_ready_count;
static dongle_usb_hid_config_report_cb_t s_hid_config_report_cb;
static void *s_hid_config_report_ctx;
static dongle_usb_diagnostics_command_cb_t s_diagnostics_command_cb;
static void *s_diagnostics_command_ctx;
static volatile bool s_cdc_connected;
static volatile bool s_cdc_dtr;
static vprintf_like_t s_prev_log_vprintf;
static bool s_cdc_log_mirror_enabled;
static bool s_cdc_log_mirror_runtime_enabled = true;
static atomic_uint s_cdc_log_attempted, s_cdc_log_enqueued, s_cdc_log_dropped;
static atomic_uint s_cdc_log_truncated, s_cdc_log_write_failed;
/* Only the response worker reads/writes this framing state. */
static bool s_cdc_tx_needs_boundary;

static void cdc_log_counter_increment(atomic_uint *counter)
{
    unsigned value = atomic_load_explicit(counter, memory_order_relaxed);
    while (value != UINT32_MAX && !atomic_compare_exchange_weak_explicit(
        counter, &value, value + 1U, memory_order_relaxed, memory_order_relaxed)) {}
}

void dongle_usb_cdc_get_log_stats(dongle_cdc_log_stats_t *out)
{
    if (!out) return;
    *out = (dongle_cdc_log_stats_t){
        .attempted = atomic_load_explicit(&s_cdc_log_attempted, memory_order_relaxed),
        .enqueued = atomic_load_explicit(&s_cdc_log_enqueued, memory_order_relaxed),
        .dropped = atomic_load_explicit(&s_cdc_log_dropped, memory_order_relaxed),
        .truncated = atomic_load_explicit(&s_cdc_log_truncated, memory_order_relaxed),
        .write_failed = atomic_load_explicit(&s_cdc_log_write_failed, memory_order_relaxed),
    };
}

typedef enum {
    CDC_OTA_IDLE = 0,
    CDC_OTA_DONGLE,
    CDC_OTA_SCEPTER,
} cdc_ota_mode_t;

static cdc_ota_mode_t s_ota_mode = CDC_OTA_IDLE;
static uint32_t s_ota_size;
static uint32_t s_ota_crc32;
static uint32_t s_ota_session;
static uint32_t s_ota_remaining;
static uint32_t s_ota_offset;
static uint32_t s_ota_sequence;
static char s_ota_partition[8];
static bool s_ota2_active;
static bool s_ota2_receiving_chunk;
static bool s_ota2_duplicate_chunk;
static bool s_ota2_have_last_chunk;
static uint32_t s_ota2_max_chunk_size;
static uint32_t s_ota2_next_sequence;
static uint32_t s_ota2_received;
static uint32_t s_ota2_running_crc;
static uint32_t s_ota2_chunk_session;
static uint32_t s_ota2_chunk_sequence;
static uint32_t s_ota2_chunk_offset;
static uint32_t s_ota2_chunk_size;
static uint32_t s_ota2_chunk_crc;
static uint32_t s_ota2_last_sequence;
static uint32_t s_ota2_last_offset;
static uint32_t s_ota2_last_size;
static uint32_t s_ota2_last_crc;
static size_t s_ota2_chunk_received;

static int8_t clamp_i32_to_i8(int32_t value)
{
    if (value > INT8_MAX) {
        return INT8_MAX;
    }
    if (value < INT8_MIN) {
        return INT8_MIN;
    }
    return (int8_t)value;
}

static int16_t clamp_i32_to_i16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static void clear_pending_mouse_state(void)
{
    portENTER_CRITICAL(&s_mouse_report_lock);
    memset(&s_mouse_pending_report, 0, sizeof(s_mouse_pending_report));
    s_mouse_report_dirty = false;
    s_mouse_last_sent_buttons = 0;
    portEXIT_CRITICAL(&s_mouse_report_lock);
}

static void hid_send_mouse_report_body(const dongle_hid_mouse_report_t *report)
{
    uint8_t mouse_report[7];
    mouse_report[0] = report->buttons;
    mouse_report[1] = (uint8_t)(report->x & 0xFF);
    mouse_report[2] = (uint8_t)((report->x >> 8) & 0xFF);
    mouse_report[3] = (uint8_t)(report->y & 0xFF);
    mouse_report[4] = (uint8_t)((report->y >> 8) & 0xFF);
    mouse_report[5] = (uint8_t)report->vertical;
    mouse_report[6] = (uint8_t)report->horizontal;
    tud_hid_report(DONGLE_HID_REPORT_ID_MOUSE, mouse_report, sizeof(mouse_report));
}

static void hid_service_task(void *arg)
{
    (void)arg;

    while (true) {
        bool mouse_dirty = false;
        bool keyboard_pending = false;
        bool mouse_snapshot_pending = false;

        portENTER_CRITICAL(&s_mouse_report_lock);
        mouse_dirty = s_mouse_report_dirty;
        portEXIT_CRITICAL(&s_mouse_report_lock);

        if (s_keyboard_report_queue) {
            keyboard_pending = uxQueueMessagesWaiting(s_keyboard_report_queue) > 0;
        }
        if (s_mouse_snapshot_queue) {
            mouse_snapshot_pending = uxQueueMessagesWaiting(s_mouse_snapshot_queue) > 0;
        }

        if (tud_suspended()) {
            if ((mouse_dirty || keyboard_pending || mouse_snapshot_pending) && tud_remote_wakeup()) {
                ESP_LOGI(TAG, "USB suspended, requesting remote wakeup for HID");
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (!tud_mounted()) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (!tud_hid_ready()) {
            if (mouse_dirty) {
                s_mouse_hid_not_ready_count++;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        dongle_hid_keyboard_report_t keyboard_report = {0};
        if (s_keyboard_report_queue &&
            xQueueReceive(s_keyboard_report_queue, &keyboard_report, 0) == pdTRUE) {
            tud_hid_keyboard_report(DONGLE_HID_REPORT_ID_KEYBOARD,
                                    keyboard_report.modifier,
                                    keyboard_report.keycode);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        dongle_hid_mouse_report_t snapshot = {0};
        if (s_mouse_snapshot_queue &&
            xQueueReceive(s_mouse_snapshot_queue, &snapshot, 0) == pdTRUE) {
            hid_send_mouse_report_body(&snapshot);
            s_mouse_last_sent_buttons = snapshot.buttons;
            s_mouse_reports_sent++;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        bool should_send = false;
        dongle_hid_mouse_report_t report = {0};

        portENTER_CRITICAL(&s_mouse_report_lock);
        if (s_mouse_report_dirty) {
            report = s_mouse_pending_report;
            should_send = (report.buttons != s_mouse_last_sent_buttons) ||
                          (report.x != 0) ||
                          (report.y != 0) ||
                          (report.vertical != 0) ||
                          (report.horizontal != 0);
            if (should_send) {
                s_mouse_pending_report.x = 0;
                s_mouse_pending_report.y = 0;
                s_mouse_pending_report.vertical = 0;
                s_mouse_pending_report.horizontal = 0;
                s_mouse_report_dirty = false;
                s_mouse_last_sent_buttons = report.buttons;
            }
        }
        portEXIT_CRITICAL(&s_mouse_report_lock);

        if (should_send) {
            hid_send_mouse_report_body(&report);
            s_mouse_reports_sent++;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    /* TinyUSB calls this at enumeration time to fetch the static HID layout. */
    (void)instance;
    return dongle_hid_report_descriptor;
}

static portMUX_TYPE s_config_reply_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_config_reply[16];
void dongle_usb_device_set_config_reply(const uint8_t reply[16]) {
    portENTER_CRITICAL(&s_config_reply_lock);
    memcpy(s_config_reply,reply,16);
    portEXIT_CRITICAL(&s_config_reply_lock);
}

uint16_t tud_hid_get_report_cb(uint8_t instance,
                               uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer,
                               uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    if (report_id!=DONGLE_HID_REPORT_ID_CONFIG || report_type!=HID_REPORT_TYPE_FEATURE || reqlen<16) return 0;
    portENTER_CRITICAL(&s_config_reply_lock);
    memcpy(buffer,s_config_reply,16);
    portEXIT_CRITICAL(&s_config_reply_lock);
    return 16;
}

void tud_hid_set_report_cb(uint8_t instance,
                           uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer,
                           uint16_t bufsize)
{
    (void)instance;

    uint8_t effective_report_id = report_id;
    const uint8_t *payload = buffer;
    uint16_t payload_size = bufsize;

    if (payload && payload_size > 0 && payload[0] == DONGLE_HID_REPORT_ID_CONFIG) {
        if (effective_report_id == 0 || payload_size == (DONGLE_HID_CONFIG_REPORT_SIZE + 1)) {
            effective_report_id = payload[0];
            payload++;
            payload_size--;
        }
    }

    if (effective_report_id != DONGLE_HID_REPORT_ID_CONFIG) {
        ESP_LOGW(TAG,
                 "HID config report ignored id=%u type=%u size=%u",
                 (unsigned)report_id,
                 (unsigned)report_type,
                 (unsigned)bufsize);
        return;
    }

    if (s_hid_config_report_cb) {
        esp_err_t ret = s_hid_config_report_cb(effective_report_id,
                                               (uint8_t)report_type,
                                               payload,
                                               payload_size,
                                               s_hid_config_report_ctx);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "HID config command failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGI(TAG,
                 "HID config report ignored type=%u size=%u",
                 (unsigned)report_type,
                 (unsigned)payload_size);
    }
}

static void cdc_invalidate_connection(void)
{
    portENTER_CRITICAL(&s_cdc_rx_lock);
    s_cdc_connected = false;
    s_cdc_dtr = false;
    ++s_cdc_epoch;
    s_cdc_disconnect_pending = true;
    s_cdc_rx_overflow_pending = false;
    s_cdc_discard_until_disconnect = false;
    portEXIT_CRITICAL(&s_cdc_rx_lock);
    /* Called only by USB-task callbacks. Queued copies keep their old epoch. */
    tud_cdc_n_read_flush(TINYUSB_CDC_ACM_0);
    if (s_cdc_worker) xTaskNotifyGive(s_cdc_worker);
}

static void usb_event_cb(tinyusb_event_t *event, void *arg)
{
    (void)arg;
    if (event->id == TINYUSB_EVENT_ATTACHED) {
        uac_device_on_usb_mount();
        ESP_LOGI(TAG, "USB composite device attached");
    } else if (event->id == TINYUSB_EVENT_DETACHED) {
        cdc_invalidate_connection();
        uac_device_on_usb_unmount();
        ESP_LOGI(TAG, "USB composite device detached");
    }
}

static bool cdc_enqueue(int itf, const char *text, size_t length,
                        bool log_line, bool completion)
{
    if (!text || !length || !s_cdc_responses ||
        (log_line && uxQueueMessagesWaiting(s_cdc_responses) >= 2)) {
        if (log_line) cdc_log_counter_increment(&s_cdc_log_dropped);
        return false;
    }
    cdc_response_t response = {.epoch = s_cdc_epoch, .itf = (uint8_t)itf,
                               .completion = completion, .log_line = log_line};
    if (log_line && length > sizeof(response.text)) {
        cdc_log_counter_increment(&s_cdc_log_truncated);
        cdc_log_counter_increment(&s_cdc_log_dropped);
        return false;
    }
    if (length > sizeof(response.text)) length = sizeof(response.text);
    response.length = (uint16_t)length;
    memcpy(response.text, text, length);
    bool command_worker = !log_line && xTaskGetCurrentTaskHandle() == s_cdc_worker;
    if (command_worker && s_cdc_processing_epoch != s_cdc_epoch) {
        portENTER_CRITICAL(&s_cdc_rx_lock);
        ++s_cdc_response_drops;
        portEXIT_CRITICAL(&s_cdc_rx_lock);
        return false;
    }
    int64_t deadline = esp_timer_get_time() + 5000000;
    do {
        if (xQueueSend(s_cdc_responses, &response,
                       command_worker ? pdMS_TO_TICKS(50) : 0) == pdTRUE) {
            if (log_line) cdc_log_counter_increment(&s_cdc_log_enqueued);
            return true;
        }
        if (!command_worker || !tud_mounted() || response.epoch != s_cdc_epoch) break;
    } while (esp_timer_get_time() < deadline);
    if (!log_line) {
        portENTER_CRITICAL(&s_cdc_rx_lock);
        ++s_cdc_response_drops;
        portEXIT_CRITICAL(&s_cdc_rx_lock);
    } else {
        cdc_log_counter_increment(&s_cdc_log_dropped);
    }
    return false;
}

static void cdc_write(int itf, const char *text)
{
    if (text) (void)cdc_enqueue(itf, text, strlen(text), false, false);
}

static void cdc_write_len(int itf, const char *text, size_t len)
{
    (void)cdc_enqueue(itf, text, len, true, false);
}

static bool cdc_write_completion(int itf, const char *text)
{
    while (xSemaphoreTake(s_cdc_delivery_sem, 0) == pdTRUE) {}
    s_cdc_delivery_succeeded = false;
    if (!cdc_enqueue(itf, text, strlen(text), false, true)) return false;
    return xSemaphoreTake(s_cdc_delivery_sem, pdMS_TO_TICKS(10000)) == pdTRUE &&
           s_cdc_delivery_succeeded;
}

/* Only this worker touches CDC TX. A stalled mirror gets a short budget,
 * while command replies retain their existing bounded delivery wait. */
static bool cdc_response_send(const cdc_response_t *response)
{
    size_t offset = 0;
    bool backend_failed = false;
    const int64_t deadline = esp_timer_get_time() +
        (response->log_line ? 100000LL : 10000000LL);
    while ((s_cdc_tx_needs_boundary || offset < response->length) &&
           response->epoch == s_cdc_epoch && tud_mounted() &&
           esp_timer_get_time() < deadline) {
        const bool boundary = s_cdc_tx_needs_boundary;
        const uint8_t *data = boundary ? (const uint8_t *)"\n" :
            (const uint8_t *)response->text + offset;
        const size_t remaining = boundary ? 1U : response->length - offset;
        const size_t queued = tinyusb_cdcacm_write_queue(response->itf, data, remaining);
        if (queued > remaining) {backend_failed = true; break;}
        if (queued > 0) {
            portENTER_CRITICAL(&s_cdc_rx_lock);
            if (UINT64_MAX - s_cdc_response_tx_queued_bytes < queued) {
                s_cdc_response_tx_queued_bytes = UINT64_MAX;
            } else {
                s_cdc_response_tx_queued_bytes += queued;
            }
            portEXIT_CRITICAL(&s_cdc_rx_lock);
            if (boundary) s_cdc_tx_needs_boundary = false;
            else offset += queued;
        }
        const esp_err_t flushed = tinyusb_cdcacm_write_flush(response->itf, 0);
        if (flushed != ESP_OK && flushed != ESP_ERR_NOT_FINISHED) {
            backend_failed = true;
            break;
        }
        if (s_cdc_tx_needs_boundary || offset < response->length) vTaskDelay(pdMS_TO_TICKS(2));
    }
    const bool complete = !backend_failed && offset == response->length &&
        response->epoch == s_cdc_epoch && tud_mounted();
    if (!complete) {
        portENTER_CRITICAL(&s_cdc_rx_lock);
        if (s_cdc_response_tx_incomplete != UINT32_MAX) ++s_cdc_response_tx_incomplete;
        portEXIT_CRITICAL(&s_cdc_rx_lock);
        if (offset && response->text[offset - 1] != '\n') s_cdc_tx_needs_boundary = true;
        if (response->log_line) cdc_log_counter_increment(&s_cdc_log_write_failed);
    }
    return complete;
}

static void cdc_response_task(void *arg)
{
    (void)arg;
    cdc_response_t response;
    while (true) {
        if (xQueueReceive(s_cdc_responses, &response, portMAX_DELAY) != pdTRUE) continue;
        const bool complete = cdc_response_send(&response);
        if (response.completion) {
            s_cdc_delivery_succeeded = complete &&
                tinyusb_cdcacm_write_flush(response.itf, pdMS_TO_TICKS(1000)) == ESP_OK;
            xSemaphoreGive(s_cdc_delivery_sem);
        }
    }
}

static void cdc_printf(int itf, const char *fmt, ...)
{
    char line[384];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    cdc_write(itf, line);
}

void dongle_usb_device_cdc_printf(const char *fmt, ...)
{
    if (!tud_mounted()) {
        return;
    }

    char line[384];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    cdc_write(TINYUSB_CDC_ACM_0, line);
}

static int cdc_log_vprintf(const char *fmt, va_list args)
{
    va_list cdc_args;
    va_copy(cdc_args, args);

    int ret = 0;
    if (s_prev_log_vprintf) {
        ret = s_prev_log_vprintf(fmt, args);
    } else {
        ret = vprintf(fmt, args);
    }

    if (s_ota_mode == CDC_OTA_IDLE && s_cdc_log_mirror_runtime_enabled && s_cdc_dtr && tud_mounted()) {
        cdc_log_counter_increment(&s_cdc_log_attempted);
        char line[CDC_RESPONSE_TEXT_BYTES] = {0};
        const int len = fmt ? vsnprintf(line, sizeof(line), fmt, cdc_args) : -1;
        if (len <= 0) {
            cdc_log_counter_increment(&s_cdc_log_dropped);
        } else if ((size_t)len >= sizeof(line) ||
                   ((size_t)len == sizeof(line) - 1U && line[len - 1] != '\n')) {
            /* Refuse the whole overlength record: a clipped JSON prefix must
             * never be joined to the next complete diagnostic/command reply. */
            cdc_log_counter_increment(&s_cdc_log_truncated);
            cdc_log_counter_increment(&s_cdc_log_dropped);
        } else {
            size_t bytes = (size_t)len;
            if (memchr(line, '\0', bytes) ||
                memchr(line, '\n', bytes - (line[bytes - 1] == '\n' ? 1U : 0U))) {
                cdc_log_counter_increment(&s_cdc_log_dropped);
            } else {
                if (line[bytes - 1] != '\n') {
                    line[bytes++] = '\n';
                    line[bytes] = '\0';
                }
                cdc_write_len(TINYUSB_CDC_ACM_0, line, bytes);
            }
        }
    }

    va_end(cdc_args);
    return ret;
}

static void cdc_enable_log_mirror(void)
{
    if (s_cdc_log_mirror_enabled) {
        return;
    }
    s_prev_log_vprintf = esp_log_set_vprintf(cdc_log_vprintf);
    s_cdc_log_mirror_enabled = true;
}

/* Text framing state is worker-owned and reset together on line/epoch end. */
static void cdc_reset_command_line(void)
{
    s_cmd_line_len = 0;
    s_cmd_line_overflow = false;
    s_cmd_line_invalid = false;
}

static void trim_line(char *line)
{
    size_t len = strlen(line);
    while (len > 0 && isspace((unsigned char)line[len - 1])) {
        line[--len] = '\0';
    }
}

static void cdc_print_status(int itf)
{
    /*
     * The stock Plain graphical flasher identifies eligible factory images
     * from these protocol strings before it opens USB.  This dual-mic target
     * intentionally keeps ROM entry manual, so retain the identifiers as
     * package compatibility metadata without changing any GPIO/USB routing.
     */
    static const volatile char factory_tool_manual_markers[] =
        "FACTORY ROM DONGLE CONFIRM\n"
        "OK FACTORY ROM role=dongle\n"
        "factory_rom_reason=manual_boot_required\n"
        "security=%s";
    volatile char factory_tool_marker_probe = factory_tool_manual_markers[0];
    (void)factory_tool_marker_probe;

    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_app_desc_t *desc = esp_app_get_description();
    dual_end_ota_status_t st = dual_end_ota_get_status();
    char mac[18] = {0};
    dual_end_ota_get_mac_string(mac, sizeof(mac));
    cdc_printf(itf,
               "OK STATUS role=dongle ota_protocol=2 ota1=disabled mac=%s partition=%s project=%s version=%s image_key=%s "
               "security=unencrypted factory_rom=ready factory_rom_reason=manual_boot_required "
               "active=%d written=%" PRIu32 " size=%" PRIu32 " response_drops=%" PRIu32 "\n",
               mac,
               running ? running->label : "unknown",
               desc ? desc->project_name : "unknown",
               desc ? desc->version : "unknown",
               desc ? dual_end_ota_image_key_from_version(desc->version) : "unknown",
               st.active ? 1 : 0,
               st.bytes_written,
               st.image_size, s_cdc_response_drops);
}

static void cdc_reset_ota_state(void)
{
    dual_end_ota_cancel_scepter_result_wait();
    s_ota_mode = CDC_OTA_IDLE;
    s_ota_size = 0;
    s_ota_crc32 = 0;
    s_ota_session = 0;
    s_ota_remaining = 0;
    s_ota_offset = 0;
    s_ota_sequence = 0;
    s_ota_partition[0] = '\0';
    s_ota2_active = false;
    s_ota2_receiving_chunk = false;
    s_ota2_duplicate_chunk = false;
    s_ota2_have_last_chunk = false;
    s_ota2_max_chunk_size = 0;
    s_ota2_next_sequence = 0;
    s_ota2_received = 0;
    s_ota2_running_crc = 0;
    s_ota2_chunk_session = 0;
    s_ota2_chunk_sequence = 0;
    s_ota2_chunk_offset = 0;
    s_ota2_chunk_size = 0;
    s_ota2_chunk_crc = 0;
    s_ota2_last_sequence = 0;
    s_ota2_last_offset = 0;
    s_ota2_last_size = 0;
    s_ota2_last_crc = 0;
    s_ota2_chunk_received = 0;
}

static void cdc_start_ota(int itf,
                          cdc_ota_mode_t mode,
                          uint32_t size,
                          uint32_t crc32,
                          uint32_t session,
                          const char *partition,
                          uint32_t chunk_size)
{
    if (size == 0 || session == 0 || chunk_size < 16 ||
        chunk_size > CDC_OTA2_MAX_CHUNK_BYTES ||
        s_ota_mode != CDC_OTA_IDLE) {
        cdc_write(itf, "ERR OTA_BEGIN invalid_size_or_session_or_busy\n");
        return;
    }
    cdc_reset_ota_state();
    s_ota_last_rx_us = esp_timer_get_time();
    s_ota_mode = mode;
    s_ota_size = size;
    s_ota_crc32 = crc32;
    s_ota_session = session;
    s_ota_remaining = size;
    snprintf(s_ota_partition, sizeof(s_ota_partition), "%s", partition && partition[0] ? partition : "auto");
    s_ota2_active = true;
    s_ota2_max_chunk_size = chunk_size;

    if (mode == CDC_OTA_DONGLE) {
        esp_err_t ret = dual_end_ota_self_begin_partition(DUAL_END_OTA_TARGET_DONGLE, session, size, crc32, s_ota_partition);
        if (ret != ESP_OK) {
            cdc_printf(itf, "ERR DONGLE_OTA_BEGIN %s\n", esp_err_to_name(ret));
            cdc_reset_ota_state();
            return;
        }
        cdc_printf(itf,
                   "OK BEGIN DONGLE session=%" PRIu32 " size=%" PRIu32
                   " partition=%s protocol=2 chunk=%" PRIu32 "\n",
                   session, size, s_ota_partition, chunk_size);
    } else {
        esp_err_t ret = dual_end_ota_send_scepter_begin(size, crc32, session, s_ota_partition);
        if (ret != ESP_OK) {
            cdc_printf(itf, "ERR SCEPTER_BEGIN %s\n", esp_err_to_name(ret));
            cdc_reset_ota_state();
            return;
        }
        cdc_printf(itf,
                   "OK BEGIN SCEPTER session=%" PRIu32 " size=%" PRIu32
                   " partition=%s protocol=2 chunk=%" PRIu32
                   " wireless_ack=1 retry=1 duplicate_safe=1\n",
                   session, size, s_ota_partition, chunk_size);
    }
}

static void cdc_process_erase(int itf, const char *target, const char *partition)
{
    if (strcasecmp(target, "DONGLE") == 0) {
        esp_err_t ret = dual_end_ota_erase_partition(partition);
        cdc_printf(itf, ret == ESP_OK ? "OK ERASE DONGLE partition=%s\n" : "ERR ERASE DONGLE %s\n",
                   ret == ESP_OK ? partition : esp_err_to_name(ret));
    } else if (strcasecmp(target, "SCEPTER") == 0) {
        esp_err_t ret = dual_end_ota_send_scepter_erase(partition);
        cdc_printf(itf, ret == ESP_OK ? "OK ERASE SCEPTER partition=%s forwarded=1\n" : "ERR ERASE SCEPTER %s\n",
                   ret == ESP_OK ? partition : esp_err_to_name(ret));
    } else {
        cdc_write(itf, "ERR ERASE TARGET\n");
    }
}

static void cdc_process_boot(int itf, const char *target, const char *partition)
{
    if (strcasecmp(target, "DONGLE") == 0) {
        esp_err_t ret = dual_end_ota_set_boot_partition_label(partition);
        cdc_printf(itf, ret == ESP_OK ? "OK BOOT DONGLE partition=%s reboot_required=1\n" : "ERR BOOT DONGLE %s\n",
                   ret == ESP_OK ? partition : esp_err_to_name(ret));
    } else if (strcasecmp(target, "SCEPTER") == 0) {
        esp_err_t ret = dual_end_ota_send_scepter_boot(partition);
        cdc_printf(itf, ret == ESP_OK ? "OK BOOT SCEPTER partition=%s forwarded=1 reboot_required=1\n" : "ERR BOOT SCEPTER %s\n",
                   ret == ESP_OK ? partition : esp_err_to_name(ret));
    } else {
        cdc_write(itf, "ERR BOOT TARGET\n");
    }
}

static void cdc_process_log_command(int itf, const char *value)
{
    if (!value) {
        cdc_printf(itf,
                   "OK LOG mirror=%d\n",
                   s_cdc_log_mirror_runtime_enabled ? 1 : 0);
        return;
    }

    if (strcasecmp(value, "0") == 0 || strcasecmp(value, "OFF") == 0) {
        s_cdc_log_mirror_runtime_enabled = false;
    } else if (strcasecmp(value, "1") == 0 || strcasecmp(value, "ON") == 0) {
        s_cdc_log_mirror_runtime_enabled = true;
    } else {
        cdc_write(itf, "ERR LOG ARG\n");
        return;
    }

    cdc_printf(itf,
               "OK LOG mirror=%d\n",
               s_cdc_log_mirror_runtime_enabled ? 1 : 0);
}

static void cdc_process_reboot(int itf, const char *target)
{
    if (strcasecmp(target, "DONGLE") == 0) {
        cdc_write(itf, "OK REBOOT DONGLE rebooting=1\n");
        tinyusb_cdcacm_write_flush(itf, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
    } else if (strcasecmp(target, "SCEPTER") == 0) {
        esp_err_t ret = dual_end_ota_send_scepter_reboot();
        cdc_printf(itf, ret == ESP_OK ? "OK REBOOT SCEPTER forwarded=1\n" : "ERR REBOOT SCEPTER %s\n",
                   ret == ESP_OK ? "" : esp_err_to_name(ret));
    } else {
        cdc_write(itf, "ERR REBOOT TARGET\n");
    }
}

static bool cdc_ota_epoch_valid(void)
{
    return tud_mounted() && !s_cdc_disconnect_pending &&
           !s_cdc_discard_until_disconnect &&
           s_cdc_processing_epoch == s_cdc_epoch;
}

static void cdc_abort_ota_stream(void)
{
    uint32_t epoch = s_cdc_processing_epoch;
    if (s_ota_mode == CDC_OTA_SCEPTER) (void)dual_end_ota_send_scepter_abort(s_ota_session);
    dual_end_ota_self_abort();
    cdc_reset_ota_state();
    if (epoch == s_cdc_epoch) {
        cdc_reset_command_line();
    }
}

static void cdc_ota2_chunk_begin(int itf,
                                 uint32_t session,
                                 uint32_t sequence,
                                 uint32_t offset,
                                 uint32_t chunk_size,
                                 uint32_t chunk_crc)
{
    if ((s_ota_mode != CDC_OTA_DONGLE && s_ota_mode != CDC_OTA_SCEPTER) ||
        !s_ota2_active || s_ota2_receiving_chunk) {
        cdc_write(itf, "ERR CHUNK_STATE retryable=0\n");
        return;
    }
    if (session != s_ota_session) {
        cdc_printf(itf,
                   "ERR CHUNK_SESSION session=%" PRIu32 " got=%" PRIu32
                   " expected=%" PRIu32 " retryable=0\n",
                   session, session, s_ota_session);
        return;
    }
    if (chunk_size == 0 || chunk_size > s_ota2_max_chunk_size ||
        chunk_size > CDC_OTA2_MAX_CHUNK_BYTES) {
        cdc_printf(itf,
                   "ERR CHUNK_SIZE session=%" PRIu32 " sequence=%" PRIu32
                   " got=%" PRIu32 " max=%" PRIu32 " retryable=0\n",
                   session, sequence, chunk_size, s_ota2_max_chunk_size);
        return;
    }

    bool duplicate = s_ota2_have_last_chunk &&
                     sequence == s_ota2_last_sequence &&
                     offset == s_ota2_last_offset &&
                     chunk_size == s_ota2_last_size &&
                     chunk_crc == s_ota2_last_crc;
    if (!duplicate &&
        (sequence != s_ota2_next_sequence ||
         offset != s_ota2_received ||
         chunk_size > s_ota_remaining)) {
        cdc_printf(itf,
                   "ERR CHUNK_ORDER session=%" PRIu32
                   " sequence=%" PRIu32 " offset=%" PRIu32
                   " expected_sequence=%" PRIu32 " expected_offset=%" PRIu32
                   " remaining=%" PRIu32 " retryable=0\n",
                   session, sequence, offset, s_ota2_next_sequence,
                   s_ota2_received, s_ota_remaining);
        return;
    }

    s_ota2_chunk_session = session;
    s_ota2_chunk_sequence = sequence;
    s_ota2_chunk_offset = offset;
    s_ota2_chunk_size = chunk_size;
    s_ota2_chunk_crc = chunk_crc;
    s_ota2_chunk_received = 0;
    s_ota2_duplicate_chunk = duplicate;
    s_ota2_receiving_chunk = true;
}

static size_t cdc_ota2_chunk_process(int itf,
                                     const uint8_t *data,
                                     size_t len)
{
    size_t remaining = s_ota2_chunk_size - s_ota2_chunk_received;
    size_t take = len < remaining ? len : remaining;
    if (take == 0) return 0;
    memcpy(s_ota2_chunk + s_ota2_chunk_received, data, take);
    s_ota2_chunk_received += take;
    if (s_ota2_chunk_received != s_ota2_chunk_size) return take;

    uint32_t calculated_crc = dual_end_ota_crc32_update(0,
                                                         s_ota2_chunk,
                                                         s_ota2_chunk_size);
    if (calculated_crc != s_ota2_chunk_crc) {
        cdc_printf(itf,
                   "ERR CHUNK_CRC session=%" PRIu32
                   " sequence=%" PRIu32 " offset=%" PRIu32
                   " size=%" PRIu32 " expected=%08" PRIx32
                   " calculated=%08" PRIx32 " retryable=1\n",
                   s_ota2_chunk_session, s_ota2_chunk_sequence,
                   s_ota2_chunk_offset, s_ota2_chunk_size,
                   s_ota2_chunk_crc, calculated_crc);
        s_ota2_receiving_chunk = false;
        s_ota2_duplicate_chunk = false;
        s_ota2_chunk_received = 0;
        return take;
    }

    bool duplicate = s_ota2_duplicate_chunk;
    if (!duplicate) {
        if (!cdc_ota_epoch_valid()) {
            cdc_abort_ota_stream();
            return take;
        }
        esp_err_t ret = ESP_OK;
        if (s_ota_mode == CDC_OTA_DONGLE) {
            ret = dual_end_ota_self_write(s_ota2_chunk, s_ota2_chunk_size);
        } else {
            size_t forwarded = 0;
            while (forwarded < s_ota2_chunk_size) {
                size_t fragment_size = s_ota2_chunk_size - forwarded;
                if (fragment_size > DUAL_END_OTA_MAX_PAYLOAD) {
                    fragment_size = DUAL_END_OTA_MAX_PAYLOAD;
                }
                ret = dual_end_ota_send_scepter_data(
                    s_ota2_chunk + forwarded,
                    fragment_size,
                    s_ota2_chunk_offset + (uint32_t)forwarded,
                    s_ota_sequence,
                    s_ota_size,
                    s_ota_crc32,
                    s_ota_session);
                if (ret != ESP_OK) break;
                ++s_ota_sequence;
                forwarded += fragment_size;
            }
        }

        if (ret != ESP_OK || !cdc_ota_epoch_valid()) {
            cdc_printf(itf,
                       "ERR CHUNK_WRITE session=%" PRIu32
                       " sequence=%" PRIu32 " offset=%" PRIu32
                       " stage=%s code=%s retryable=0\n",
                       s_ota_session, s_ota2_chunk_sequence,
                       s_ota2_chunk_offset,
                       s_ota_mode == CDC_OTA_SCEPTER ? "wireless_ack" : "flash",
                       esp_err_to_name(ret != ESP_OK ? ret : ESP_ERR_INVALID_STATE));
            cdc_abort_ota_stream();
            return take;
        }

        s_ota2_running_crc = dual_end_ota_crc32_update(s_ota2_running_crc,
                                                       s_ota2_chunk,
                                                       s_ota2_chunk_size);
        s_ota2_received += s_ota2_chunk_size;
        s_ota_remaining -= s_ota2_chunk_size;
        s_ota_offset += s_ota2_chunk_size;
        s_ota2_last_sequence = s_ota2_chunk_sequence;
        s_ota2_last_offset = s_ota2_chunk_offset;
        s_ota2_last_size = s_ota2_chunk_size;
        s_ota2_last_crc = s_ota2_chunk_crc;
        s_ota2_have_last_chunk = true;
        ++s_ota2_next_sequence;
    }

    cdc_printf(itf,
               "OK CHUNK session=%" PRIu32
               " sequence=%" PRIu32 " offset=%" PRIu32
               " size=%" PRIu32 " received=%" PRIu32
               " rolling_crc=%08" PRIx32 " duplicate=%u\n",
               s_ota2_chunk_session, s_ota2_chunk_sequence,
               s_ota2_chunk_offset, s_ota2_chunk_size,
               s_ota2_received, s_ota2_running_crc,
               duplicate ? 1u : 0u);
    s_ota2_receiving_chunk = false;
    s_ota2_duplicate_chunk = false;
    s_ota2_chunk_received = 0;
    return take;
}

static void cdc_ota2_finish(int itf, uint32_t session)
{
    if ((s_ota_mode != CDC_OTA_DONGLE && s_ota_mode != CDC_OTA_SCEPTER) ||
        !s_ota2_active || s_ota2_receiving_chunk) {
        cdc_write(itf, "ERR OTA_END stage=state code=ESP_ERR_INVALID_STATE\n");
        return;
    }
    if (session != s_ota_session) {
        cdc_printf(itf,
                   "ERR OTA_END stage=session got=%" PRIu32
                   " expected=%" PRIu32 "\n",
                   session, s_ota_session);
        return;
    }
    if (s_ota2_received != s_ota_size) {
        cdc_printf(itf,
                   "ERR OTA_SIZE stage=transport expected=%" PRIu32
                   " received=%" PRIu32 " remaining=%" PRIu32 "\n",
                   s_ota_size, s_ota2_received, s_ota_remaining);
        return;
    }
    if (s_ota2_running_crc != s_ota_crc32) {
        cdc_printf(itf,
                   "ERR OTA_CRC stage=transport expected=%08" PRIx32
                   " calculated=%08" PRIx32 " received=%" PRIu32 "\n",
                   s_ota_crc32, s_ota2_running_crc, s_ota2_received);
        cdc_abort_ota_stream();
        return;
    }

    if (s_ota_mode == CDC_OTA_SCEPTER) {
        esp_err_t ret = dual_end_ota_send_scepter_end(s_ota_size,
                                                      s_ota_crc32,
                                                      s_ota_session,
                                                      s_ota_sequence);
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT &&
            ret != ESP_ERR_INVALID_STATE && ret != ESP_FAIL) {
            cdc_printf(itf,
                       "ERR OTA_END stage=wireless_end code=%s\n",
                       esp_err_to_name(ret));
            cdc_abort_ota_stream();
            return;
        }

        uint32_t completed_session = s_ota_session;
        cdc_printf(itf,
                   "OK END SCEPTER session=%" PRIu32
                   " size=%" PRIu32 " crc=%08" PRIx32
                   " calculated_crc=%08" PRIx32 " chunks=%" PRIu32
                   " wireless_packets=%" PRIu32
                   " protocol=2 forwarded=1 awaiting_install=1\n",
                   completed_session, s_ota_size, s_ota_crc32,
                   s_ota2_running_crc, s_ota2_next_sequence, s_ota_sequence);

        dual_end_ota_result_t result = {0};
        ret = dual_end_ota_wait_scepter_result(60000, &result);
        if (ret == ESP_OK && result.session_id == completed_session &&
            result.payload.status == DUAL_END_OTA_RESULT_INSTALL_CONFIRMED &&
            result.payload.stage == DUAL_END_OTA_RESULT_STAGE_BOOT_CONFIRM &&
            result.payload.error_code == ESP_OK) {
            cdc_printf(itf,
                       "OK END SCEPTER session=%" PRIu32
                       " installed=1 boot_confirmed=1 version=%s partition=%s\n",
                       completed_session, result.payload.app_version,
                       result.payload.partition);
        } else {
            cdc_printf(itf,
                       "ERR SCEPTER_FINAL_ACK session=%" PRIu32
                       " stage=%u code=%" PRId32 " wait=%s\n",
                       completed_session, result.payload.stage,
                       result.payload.error_code, esp_err_to_name(ret));
            (void)dual_end_ota_send_scepter_abort(completed_session);
        }
        cdc_reset_ota_state();
        return;
    }

    esp_err_t ret = dual_end_ota_self_finish(false);
    if (ret != ESP_OK) {
        cdc_printf(itf,
                   "ERR OTA_END stage=image_finalize code=%s"
                   " transport_crc=%08" PRIx32 " expected_crc=%08" PRIx32
                   " received=%" PRIu32 "\n",
                   esp_err_to_name(ret), s_ota2_running_crc,
                   s_ota_crc32, s_ota2_received);
        dual_end_ota_self_abort();
        cdc_reset_ota_state();
        return;
    }

    char completion[256];
    snprintf(completion, sizeof(completion),
             "OK END DONGLE session=%" PRIu32
             " size=%" PRIu32 " crc=%08" PRIx32
             " calculated_crc=%08" PRIx32 " chunks=%" PRIu32
             " protocol=2 rebooting=1\n",
             s_ota_session, s_ota_size, s_ota_crc32,
             s_ota2_running_crc, s_ota2_next_sequence);
    bool delivered = cdc_write_completion(itf, completion);
    if (!delivered) {
        ESP_LOGW(TAG, "OTA committed but host did not accept completion response");
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

static size_t split_command_tokens(char *line, char **tokens, size_t capacity)
{
    size_t count = 0;
    char *cursor = line;
    while (*cursor) {
        while (*cursor && isspace((unsigned char)*cursor)) ++cursor;
        if (!*cursor) break;
        if (count == capacity) return capacity + 1;
        tokens[count++] = cursor;
        while (*cursor && !isspace((unsigned char)*cursor)) ++cursor;
        if (*cursor) *cursor++ = '\0';
    }
    return count;
}

static bool parse_u32_token(const char *text, int base, uint32_t *value)
{
    if (!text || !*text || !value || *text == '-' || *text == '+') return false;
    errno = 0;
    char *end = NULL;
    unsigned long parsed = strtoul(text, &end, base);
    if (errno != 0 || !end || *end != '\0' || parsed > UINT32_MAX) return false;
    *value = (uint32_t)parsed;
    return true;
}

static void cdc_process_command(int itf, char *line)
{
    trim_line(line);

    char parsed_line[sizeof(s_cmd_line)];
    memcpy(parsed_line, line, strlen(line) + 1);
    char *tokens[8] = {0};
    size_t token_count = split_command_tokens(parsed_line, tokens,
                                              sizeof(tokens) / sizeof(tokens[0]));

    if ((s_ota_mode == CDC_OTA_DONGLE || s_ota_mode == CDC_OTA_SCEPTER) &&
        s_ota2_active) {
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
            cdc_ota2_chunk_begin(itf, session, sequence, offset,
                                 chunk_size, chunk_crc);
            return;
        }
        if (token_count == 2 && strcasecmp(tokens[0], "END") == 0 &&
            parse_u32_token(tokens[1], 10, &session)) {
            cdc_ota2_finish(itf, session);
            return;
        }
        if (token_count == 2 && strcasecmp(tokens[0], "ABORT") == 0 &&
            parse_u32_token(tokens[1], 10, &session)) {
            if (session != s_ota_session) {
                cdc_printf(itf,
                           "ERR OTA_ABORT got=%" PRIu32 " expected=%" PRIu32 "\n",
                           session, s_ota_session);
                return;
            }
            const char *target_name = s_ota_mode == CDC_OTA_SCEPTER ?
                                          "SCEPTER" : "DONGLE";
            uint32_t received = s_ota2_received;
            if (s_ota_mode == CDC_OTA_SCEPTER) {
                (void)dual_end_ota_send_scepter_abort(s_ota_session);
            } else {
                dual_end_ota_self_abort();
            }
            cdc_printf(itf,
                       "OK ABORT %s session=%" PRIu32 " received=%" PRIu32 "\n",
                       target_name, s_ota_session, received);
            cdc_reset_ota_state();
            return;
        }
        cdc_write(itf, "ERR OTA2_COMMAND supported=CHUNK,END,ABORT\n");
        return;
    }

    char pairing_response[384];
    if (dual_end_pairing_handle_command(line,
                                        pairing_response,
                                        sizeof(pairing_response))) {
        cdc_write(itf, pairing_response);
        return;
    }

    uint32_t size = 0;
    uint32_t crc32 = 0;
    uint32_t session = (uint32_t)esp_timer_get_time();
    uint32_t chunk_size = 0;
    if (token_count == 7 && strcasecmp(tokens[0], "OTA2") == 0 &&
        parse_u32_token(tokens[2], 10, &size) &&
        parse_u32_token(tokens[3], 16, &crc32) &&
        parse_u32_token(tokens[4], 10, &session) &&
        parse_u32_token(tokens[6], 10, &chunk_size)) {
        cdc_ota_mode_t mode = CDC_OTA_IDLE;
        if (strcasecmp(tokens[1], "DONGLE") == 0) {
            mode = CDC_OTA_DONGLE;
        } else if (strcasecmp(tokens[1], "SCEPTER") == 0) {
            mode = CDC_OTA_SCEPTER;
        } else {
            cdc_write(itf, "ERR OTA2 TARGET\n");
            return;
        }
        cdc_start_ota(itf, mode, size, crc32, session, tokens[5], chunk_size);
        return;
    }
    if (token_count > 0 && strcasecmp(tokens[0], "OTA2") == 0) {
        cdc_write(itf, "ERR OTA2 ARGUMENTS\n");
        return;
    }

    char telemetry_response[256];
    if(dual_end_runtime_command(line,telemetry_response,sizeof(telemetry_response))){cdc_write(itf,telemetry_response);return;}

    /* CDC command replies follow the mounted data path, not the DTR-gated
     * console mirror. OTA hosts intentionally keep DTR low, so diagnostics
     * remain reachable without changing their serial line state. */
    char diagnostics_response[sizeof(((cdc_response_t *)0)->text)];
    if (s_diagnostics_command_cb &&
        s_diagnostics_command_cb(line, diagnostics_response,
                                 sizeof(diagnostics_response),
                                 s_diagnostics_command_ctx)) {
        cdc_write(itf, diagnostics_response);
        return;
    }

    if (strcasecmp(line, "PING") == 0) {
        cdc_write(itf, "OK PONG role=dongle\n");
    } else if (token_count > 0 && strcasecmp(tokens[0], "LOG") == 0) {
        cdc_process_log_command(itf, token_count == 2 ? tokens[1] : NULL);
    } else if (strcasecmp(line, "STATUS") == 0) {
        cdc_print_status(itf);
    } else if (token_count == 2 && strcasecmp(tokens[0], "UI") == 0) {
        uint8_t control_command = 0;
        if (strcasecmp(tokens[1], "THINKING") == 0) {
            control_command = DUAL_END_CONTROL_CMD_UI_THINKING;
        } else if (strcasecmp(tokens[1], "APPROVE") == 0) {
            control_command = DUAL_END_CONTROL_CMD_UI_APPROVE;
        } else if (strcasecmp(tokens[1], "DONE") == 0) {
            control_command = DUAL_END_CONTROL_CMD_UI_DONE;
        }

        if (control_command == 0) {
            cdc_write(itf, "ERR UI NAME\n");
        } else if (!s_hid_config_report_cb) {
            cdc_write(itf, "ERR UI NOT_READY\n");
        } else {
            uint8_t report[DONGLE_HID_CONFIG_REPORT_SIZE] = {0};
            report[0] = DONGLE_HID_CONFIG_CMD_FORWARD_TO_SCEPTER;
            report[1] = control_command;
            report[2] = 0;
            esp_err_t ret = s_hid_config_report_cb(DONGLE_HID_REPORT_ID_CONFIG,
                                                   DONGLE_HID_REPORT_TYPE_OUTPUT,
                                                   report,
                                                   sizeof(report),
                                                   s_hid_config_report_ctx);
            if (ret == ESP_OK) {
                cdc_printf(itf, "OK UI %s forwarded=1\n", tokens[1]);
            } else {
                cdc_printf(itf, "ERR UI %s\n", esp_err_to_name(ret));
            }
        }
    } else if (token_count > 0 && strcasecmp(tokens[0], "OTA") == 0) {
        cdc_write(itf, "ERR OTA1_DISABLED use=OTA2\n");
    } else if (token_count == 3 && strcasecmp(tokens[0], "ERASE") == 0) {
        cdc_process_erase(itf, tokens[1], tokens[2]);
    } else if (token_count == 3 && strcasecmp(tokens[0], "BOOT") == 0) {
        cdc_process_boot(itf, tokens[1], tokens[2]);
    } else if (token_count == 2 && strcasecmp(tokens[0], "REBOOT") == 0) {
        cdc_process_reboot(itf, tokens[1]);
    } else {
        cdc_write(itf, "ERR COMMAND unknown\n");
    }
}

static void cdc_process_rx(int itf, const uint8_t *data, size_t len)
{
    if (s_ota_mode == CDC_OTA_DONGLE || s_ota_mode == CDC_OTA_SCEPTER) {
        s_ota_last_rx_us = esp_timer_get_time();
    }
    size_t index = 0;
    while (index < len) {
        if (!cdc_ota_epoch_valid()) return;
        if (s_ota_mode != CDC_OTA_IDLE && s_ota2_receiving_chunk) {
            size_t consumed = cdc_ota2_chunk_process(itf,
                                                     data + index,
                                                     len - index);
            if (consumed == 0) return;
            index += consumed;
            continue;
        }
        char ch = (char)data[index++];
        if (ch == '\n') {
            if (s_cmd_line_len > 0 || s_cmd_line_overflow || s_cmd_line_invalid) {
                portENTER_CRITICAL(&s_cdc_rx_lock);
                ++s_cdc_rx_commands;
                portEXIT_CRITICAL(&s_cdc_rx_lock);
            }
            if (s_cmd_line_invalid) {
                cdc_write(itf, "ERR COMMAND invalid_character\n");
            } else if (s_cmd_line_overflow) {
                cdc_write(itf, "ERR COMMAND line_too_long\n");
            } else {
                s_cmd_line[s_cmd_line_len] = '\0';
                cdc_process_command(itf, s_cmd_line);
            }
            cdc_reset_command_line();
        } else if (ch == '\0') {
            /* Never let a C-string terminator hide trailing command bytes.
             * Binary OTA chunks are consumed above, before this text state. */
            s_cmd_line_invalid = true;
        } else if (ch != '\r' && !s_cmd_line_overflow && !s_cmd_line_invalid) {
            if (s_cmd_line_len + 1 < sizeof(s_cmd_line)) s_cmd_line[s_cmd_line_len++] = ch;
            else s_cmd_line_overflow = true;
        }
    }
}

static void cdc_rx_cb(int itf, cdcacm_event_t *event)
{
    (void)event;
    if (!tud_mounted()) return;
    /* OTA2 hosts intentionally keep DTR/RTS low to avoid reset pulses on the
     * native ESP32-S3 USB ports. Receiving bytes is sufficient proof that the
     * CDC data path is open. */
    portENTER_CRITICAL(&s_cdc_rx_lock);
    s_cdc_connected = true;
    portEXIT_CRITICAL(&s_cdc_rx_lock);
    /* TinyUSB clears OUT busy before copying its endpoint buffer to the FIFO.
     * Reading from another task can rearm that buffer too early. Keep all
     * reads here, and give the command worker independent byte copies. */
    cdc_rx_block_t block;
    while (true) {
        if (s_cdc_discard_until_disconnect) {
            tud_cdc_n_read_flush(itf);
            break;
        }
        size_t received = 0;
        block.epoch = s_cdc_epoch;
        esp_err_t ret = tinyusb_cdcacm_read(itf, block.data, sizeof(block.data), &received);
        if (ret != ESP_OK || received == 0) break;
        block.length = (uint16_t)received;
        if (xQueueSend(s_cdc_rx_queue, &block, 0) != pdTRUE) {
            /* Never drop a block and continue a binary OTA stream. */
            portENTER_CRITICAL(&s_cdc_rx_lock);
            ++s_cdc_epoch;
            ++s_cdc_rx_queue_overflows;
            s_cdc_rx_overflow_pending = true;
            s_cdc_discard_until_disconnect = true;
            portEXIT_CRITICAL(&s_cdc_rx_lock);
            tud_cdc_n_read_flush(itf);
            break;
        }
    }
    if (s_cdc_worker) xTaskNotifyGive(s_cdc_worker);
}

static void cdc_handle_rx_events(int itf)
{
    portENTER_CRITICAL(&s_cdc_rx_lock);
    bool disconnected = s_cdc_disconnect_pending;
    bool overflow = s_cdc_rx_overflow_pending;
    uint32_t epoch = s_cdc_epoch;
    s_cdc_disconnect_pending = false;
    s_cdc_rx_overflow_pending = false;
    portEXIT_CRITICAL(&s_cdc_rx_lock);
    if (!disconnected && !overflow) return;

    if (s_ota_mode == CDC_OTA_SCEPTER) (void)dual_end_ota_send_scepter_abort(s_ota_session);
    dual_end_ota_self_abort();
    cdc_reset_ota_state();
    cdc_reset_command_line();
    if (overflow && !disconnected) {
        s_cdc_processing_epoch = epoch;
        cdc_write(itf, "ERR CDC_RX_OVERFLOW reconnect_required=1\n");
    }
}

static void cdc_consume_rx_block(int itf, const cdc_rx_block_t *block)
{
    if (block->epoch != s_cdc_epoch || s_cdc_disconnect_pending ||
        s_cdc_discard_until_disconnect || !tud_mounted()) return;
    s_cdc_processing_epoch = block->epoch;
    cdc_process_rx(itf, block->data, block->length);
}

static void cdc_worker_task(void *arg)
{
    (void)arg;
    const int itf = TINYUSB_CDC_ACM_0;
    cdc_rx_block_t block;
    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        cdc_handle_rx_events(itf);
        if (s_ota_mode != CDC_OTA_IDLE &&
            esp_timer_get_time() - s_ota_last_rx_us > CDC_OTA_RX_TIMEOUT_US) {
            cdc_write(itf, "ERR OTA_TIMEOUT reconnect_required=1\n");
            cdc_abort_ota_stream();
        }
        while (xQueueReceive(s_cdc_rx_queue, &block, 0) == pdTRUE) {
            cdc_handle_rx_events(itf);
            cdc_consume_rx_block(itf, &block);
        }
    }
}

static void cdc_line_state_cb(int itf, cdcacm_event_t *event)
{
    bool dtr = event->line_state_changed_data.dtr != 0;
    portENTER_CRITICAL(&s_cdc_rx_lock);
    bool was_dtr = s_cdc_dtr;
    s_cdc_dtr = dtr;
    if (dtr) {
        s_cdc_connected = true;
    }
    portEXIT_CRITICAL(&s_cdc_rx_lock);
    if (!dtr && was_dtr) {
        cdc_invalidate_connection();
    }
    ESP_LOGI(TAG, "CDC%d line state: dtr=%d rts=%d",
             itf,
             dtr ? 1 : 0,
             event->line_state_changed_data.rts);

    if (dtr) {
        cdc_write(itf, "CytoScepter dongle CDC ready\n");
        cdc_write(itf, "CDC mirrors ESP_LOG output while DTR is active; use LOG 0/1 to disable/enable\n");
        cdc_write(itf, "Commands: PING, STATUS, PAIR INFO/SET/CLEAR, UI THINKING/APPROVE/DONE, OTA2/ERASE/BOOT/REBOOT\n");
        cdc_write(itf, "OTA protocol=2 ota1=disabled chunk_crc=1 chunk_ack=1 wireless_ack=1 retry=1 duplicate_safe=1\n");
        cdc_print_status(itf);
    }
}

void dongle_usb_device_reset_audio_session(void)
{
    uac_device_reset_mic_session();
}

static esp_err_t uac_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *ctx)
{
    (void)ctx;
    static uint32_t last_bridge_generation;
    size_t copied = dongle_uac_bridge_read_pcm(buf, len);
    dongle_uac_bridge_stats_t bridge = {0};
    dongle_uac_bridge_get_stats(&bridge);
    if (bridge.session_generation != last_bridge_generation) {
        last_bridge_generation = bridge.session_generation;
        /* Reject a capture straddling a timeout or session reset. The USB
         * callback clears queued history at the next endpoint boundary. */
        uac_device_reset_mic_session();
    }
    if (copied < len) {
        /* Keep the UAC endpoint clocking even when no wireless audio arrived. */
        memset(buf + copied, 0, len - copied);
    }

    if (bytes_read) {
        *bytes_read = len;
    }
    return ESP_OK;
}

esp_err_t dongle_usb_device_start(void)
{
    ESP_LOGI(TAG, "starting USB composite: UAC microphone + HID keyboard/touchpad + CDC ACM");
    s_cdc_responses = xQueueCreate(16, sizeof(cdc_response_t));
    s_cdc_rx_queue = xQueueCreate(CDC_RX_QUEUE_BLOCKS, sizeof(cdc_rx_block_t));
    s_cdc_delivery_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_cdc_responses && s_cdc_rx_queue && s_cdc_delivery_sem,
                        ESP_ERR_NO_MEM, TAG, "create CDC queues");

    /*
     * Descriptors are hand-authored in usb_descriptors.c so UAC, HID and CDC
     * can enumerate as one composite dongle instead of separate USB devices.
     */
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(usb_event_cb, NULL);
    /* This is the active composite USB task, not the skipped UAC default. */
    tusb_cfg.task.xCoreID = DUAL_END_COMM_TASK_CORE;
    tusb_cfg.descriptor.device = &dongle_usb_device_descriptor;
    tusb_cfg.descriptor.full_speed_config = dongle_usb_configuration_descriptor;
    tusb_cfg.descriptor.string = dongle_usb_string_descriptor;
    tusb_cfg.descriptor.string_count = dongle_usb_string_descriptor_count;

    s_keyboard_report_queue = xQueueCreate(KEYBOARD_REPORT_QUEUE_SIZE, sizeof(dongle_hid_keyboard_report_t));
    s_mouse_snapshot_queue = xQueueCreate(MOUSE_SNAPSHOT_QUEUE_SIZE, sizeof(dongle_hid_mouse_report_t));
    ESP_RETURN_ON_FALSE(s_keyboard_report_queue && s_mouse_snapshot_queue,
                        ESP_ERR_NO_MEM,
                        TAG,
                        "create HID queues");

    uac_device_config_t uac_cfg = {
        .skip_tinyusb_init = true,
        .input_at_usb_boundary = true,
        .output_cb = NULL,
        .input_cb = uac_input_cb,
        .set_mute_cb = NULL,
        .set_volume_cb = NULL,
        .cb_ctx = NULL,
#if CONFIG_USB_DEVICE_UAC_AS_PART
        .spk_itf_num = -1,
        .mic_itf_num = dongle_usb_uac_mic_interface,
#endif
    };
    ESP_RETURN_ON_ERROR(uac_device_init(&uac_cfg), TAG, "init UAC microphone");

    /* UAC state and HID queues must exist before a host can open interfaces. */
    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG, "install TinyUSB");
    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = cdc_rx_cb,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = cdc_line_state_cb,
        .callback_line_coding_changed = NULL,
    };
    ESP_RETURN_ON_ERROR(tinyusb_cdcacm_init(&acm_cfg), TAG, "init CDC ACM");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(cdc_worker_task, "cdc_command", 8192, NULL, 5,
                                    &s_cdc_worker, DUAL_END_COMM_TASK_CORE) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create CDC command worker");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(cdc_response_task, "cdc_response", 4096, NULL, 4, NULL, DUAL_END_COMM_TASK_CORE) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create CDC response worker");
    cdc_enable_log_mirror();

    BaseType_t hid_task_ok = xTaskCreatePinnedToCore(hid_service_task,
                                         "hid_service",
                                         HID_SERVICE_TASK_STACK_BYTES,
                                         NULL,
                                         HID_SERVICE_TASK_PRIORITY,
                                         NULL, DUAL_END_COMM_TASK_CORE);
    ESP_RETURN_ON_FALSE(hid_task_ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create HID service task");

    ESP_LOGI(TAG, "USB composite ready: sample=%u Hz bits=%u channels=%u",
             DUAL_END_AUDIO_SAMPLE_RATE_HZ,
             DUAL_END_AUDIO_BITS_PER_SAMPLE,
             DUAL_END_AUDIO_CHANNELS);
    return ESP_OK;
}
esp_err_t dongle_usb_device_send_keyboard_report(const dongle_hid_keyboard_report_t *report)
{
    ESP_RETURN_ON_FALSE(report, ESP_ERR_INVALID_ARG, TAG, "keyboard report is null");
    ESP_RETURN_ON_FALSE(s_keyboard_report_queue, ESP_ERR_INVALID_STATE, TAG, "keyboard queue not ready");

    if (xQueueSend(s_keyboard_report_queue, report, pdMS_TO_TICKS(5)) != pdTRUE) {
        ESP_LOGW(TAG, "keyboard queue full, report dropped");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t dongle_usb_device_send_mouse_report(const dongle_hid_mouse_report_t *report)
{
    ESP_RETURN_ON_FALSE(report, ESP_ERR_INVALID_ARG, TAG, "mouse report is null");

    if (!tud_mounted()) {
        clear_pending_mouse_state();
        return ESP_ERR_INVALID_STATE;
    }

    dongle_hid_mouse_report_t snapshot = {0};
    bool need_snapshot = false;

    portENTER_CRITICAL(&s_mouse_report_lock);
    bool buttons_changed = report->buttons != s_mouse_pending_report.buttons;
    if (buttons_changed && s_mouse_report_dirty) {
        snapshot = s_mouse_pending_report;
        need_snapshot = true;
        s_mouse_pending_report.x = 0;
        s_mouse_pending_report.y = 0;
        s_mouse_pending_report.vertical = 0;
        s_mouse_pending_report.horizontal = 0;
    }

    s_mouse_pending_report.buttons = report->buttons;
    s_mouse_pending_report.x = clamp_i32_to_i16((int32_t)s_mouse_pending_report.x + report->x);
    s_mouse_pending_report.y = clamp_i32_to_i16((int32_t)s_mouse_pending_report.y + report->y);
    s_mouse_pending_report.vertical = clamp_i32_to_i8((int32_t)s_mouse_pending_report.vertical + report->vertical);
    s_mouse_pending_report.horizontal = clamp_i32_to_i8((int32_t)s_mouse_pending_report.horizontal + report->horizontal);
    s_mouse_report_dirty = true;
    portEXIT_CRITICAL(&s_mouse_report_lock);

    if (need_snapshot && s_mouse_snapshot_queue) {
        if (xQueueSend(s_mouse_snapshot_queue, &snapshot, 0) != pdTRUE) {
            ESP_LOGW(TAG, "mouse snapshot queue full, button transition dropped");
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

void dongle_usb_device_note_mouse_packet_received(void)
{
    s_mouse_packets_received++;
}

void dongle_usb_device_get_stats(dongle_usb_device_stats_t *out_stats)
{
    if (!out_stats) {
        return;
    }

    out_stats->mounted = tud_mounted();
    out_stats->suspended = tud_suspended();
    portENTER_CRITICAL(&s_cdc_rx_lock);
    out_stats->cdc_connected = s_cdc_connected;
    out_stats->cdc_dtr = s_cdc_dtr;
    out_stats->cdc_rx_commands = s_cdc_rx_commands;
    out_stats->cdc_rx_queue_overflows = s_cdc_rx_queue_overflows;
    out_stats->cdc_response_drops = s_cdc_response_drops;
    out_stats->cdc_response_tx_queued_bytes = s_cdc_response_tx_queued_bytes;
    out_stats->cdc_response_tx_incomplete = s_cdc_response_tx_incomplete;
    portEXIT_CRITICAL(&s_cdc_rx_lock);
    portENTER_CRITICAL(&s_mouse_report_lock);
    out_stats->mouse_packets_received = s_mouse_packets_received;
    out_stats->mouse_reports_sent = s_mouse_reports_sent;
    out_stats->mouse_hid_not_ready = s_mouse_hid_not_ready_count;
    portEXIT_CRITICAL(&s_mouse_report_lock);
}
void dongle_usb_device_set_hid_config_report_cb(dongle_usb_hid_config_report_cb_t cb, void *ctx)
{
    s_hid_config_report_cb = cb;
    s_hid_config_report_ctx = ctx;
}

void dongle_usb_device_set_diagnostics_command_cb(dongle_usb_diagnostics_command_cb_t cb,
                                                   void *ctx)
{
    /* Registered during app initialization before USB starts; command handling
     * only reads these stable function/context pointers. */
    s_diagnostics_command_cb = cb;
    s_diagnostics_command_ctx = ctx;
}
